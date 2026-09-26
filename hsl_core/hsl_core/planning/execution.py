# hsl_core/hsl_core/planning/execution.py
"""Versioned planning candidates and cancellation/lease arbitration."""

from dataclasses import dataclass
import math
import threading

from ..types import MotionCandidate
from .astar import PlannedPath


@dataclass(frozen=True)
class CandidateEnvelope:
    candidate: MotionCandidate
    path: PlannedPath
    option_instance_id: str


class ExecutionLease:
    """Single-writer lease: cancellation revokes every older candidate."""

    def __init__(self) -> None:
        self._generation = 0
        self._cancelled = set()
        self._active_option_instance_id = ""
        self._candidate_authorized = False
        self._expected_map_version: int | None = None
        self._expected_topology_version: int | None = None
        self._expected_localization_epoch: str | None = None
        self._lock = threading.RLock()

    def activate(
        self,
        option_instance_id: str,
        *,
        candidate_authorized: bool = True,
        expected_map_version: int | None = None,
        expected_topology_version: int | None = None,
        expected_localization_epoch: str | None = None,
    ) -> int:
        if not option_instance_id:
            raise ValueError("option_instance_id must not be empty")
        if not isinstance(candidate_authorized, bool):
            raise ValueError("candidate_authorized must be boolean")
        expected_versions = (
            expected_map_version,
            expected_topology_version,
            expected_localization_epoch,
        )
        if any(value is not None for value in expected_versions):
            if (
                not isinstance(expected_map_version, int)
                or isinstance(expected_map_version, bool)
                or expected_map_version < 0
                or not isinstance(expected_topology_version, int)
                or isinstance(expected_topology_version, bool)
                or expected_topology_version < 0
                or not isinstance(expected_localization_epoch, str)
                or not expected_localization_epoch.strip()
            ):
                raise ValueError("expected path versions must be supplied together and valid")
        with self._lock:
            if self._active_option_instance_id:
                if self._active_option_instance_id == option_instance_id:
                    raise ValueError("option instance already owns the active lease")
                if self._active_option_instance_id not in self._cancelled:
                    raise ValueError("active option instance must be cancelled before replacement")
            if option_instance_id in self._cancelled:
                raise ValueError("cancelled option instance IDs must not be reused")
            self._generation += 1
            self._active_option_instance_id = option_instance_id
            self._candidate_authorized = candidate_authorized
            self._expected_map_version = expected_map_version
            self._expected_topology_version = expected_topology_version
            self._expected_localization_epoch = expected_localization_epoch
            return self._generation

    def cancel(self, option_instance_id: str) -> None:
        if not option_instance_id:
            return
        with self._lock:
            self._cancelled.add(option_instance_id)
            if option_instance_id == self._active_option_instance_id:
                self._candidate_authorized = False

    def set_candidate_authorized(
        self, option_instance_id: str, generation: int, authorized: bool
    ) -> None:
        if not option_instance_id:
            raise ValueError("option_instance_id must not be empty")
        if not isinstance(authorized, bool):
            raise ValueError("authorized must be boolean")
        with self._lock:
            if generation != self._generation:
                raise ValueError("candidate belongs to an inactive lease generation")
            if option_instance_id != self._active_option_instance_id:
                raise ValueError("option instance does not own the active lease")
            if option_instance_id in self._cancelled and authorized:
                raise ValueError("cancelled option instance cannot regain authority")
            self._candidate_authorized = authorized

    def advance_generation(self, option_instance_id: str, generation: int) -> int:
        """Invalidate every candidate from an earlier plan of the same option."""
        with self._lock:
            if generation != self._generation:
                raise ValueError("candidate belongs to an inactive lease generation")
            if option_instance_id != self._active_option_instance_id:
                raise ValueError("option instance does not own the active lease")
            if option_instance_id in self._cancelled:
                raise ValueError("cancelled option instance cannot regain authority")
            if self._candidate_authorized:
                raise ValueError("candidate authority must be revoked before generation advance")
            self._generation += 1
            return self._generation

    def admit(self, envelope: CandidateEnvelope, *, now_s: float, generation: int) -> MotionCandidate:
        if isinstance(now_s, bool) or not isinstance(now_s, (int, float)) or not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        with self._lock:
            if generation != self._generation:
                raise ValueError("candidate belongs to an inactive lease generation")
            if envelope.option_instance_id in self._cancelled:
                raise ValueError("candidate option instance was cancelled")
            if envelope.option_instance_id != self._active_option_instance_id:
                raise ValueError("candidate option instance does not own the active lease")
            if not self._candidate_authorized:
                raise ValueError("candidate authority is revoked")
            candidate = envelope.candidate
            if candidate.source_id != envelope.option_instance_id:
                raise ValueError("candidate source does not match option instance")
            if candidate.lease_generation != generation:
                raise ValueError("candidate lease generation does not match active generation")
            if now_s >= candidate.valid_until_s:
                raise ValueError("candidate lease has expired")
            if candidate.map_version != envelope.path.map_version:
                raise ValueError("candidate map version does not match path")
            if candidate.topology_version != envelope.path.topology_version:
                raise ValueError("candidate topology version does not match path")
            if envelope.path.localization_epoch != candidate.localization_epoch:
                raise ValueError("candidate localization epoch does not match path")
            if (
                self._expected_map_version is not None
                and candidate.map_version != self._expected_map_version
            ):
                raise ValueError("candidate map version does not match active option")
            if (
                self._expected_topology_version is not None
                and candidate.topology_version != self._expected_topology_version
            ):
                raise ValueError("candidate topology version does not match active option")
            if (
                self._expected_localization_epoch is not None
                and candidate.localization_epoch != self._expected_localization_epoch
            ):
                raise ValueError("candidate localization epoch does not match active option")
            return candidate
