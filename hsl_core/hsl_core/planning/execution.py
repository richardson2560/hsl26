# hsl_core/hsl_core/planning/execution.py
"""Versioned planning candidates and cancellation/lease arbitration."""

from dataclasses import dataclass

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

    def activate(self, option_instance_id: str) -> int:
        if not option_instance_id:
            raise ValueError("option_instance_id must not be empty")
        self._generation += 1
        return self._generation

    def cancel(self, option_instance_id: str) -> None:
        if option_instance_id:
            self._cancelled.add(option_instance_id)

    def admit(self, envelope: CandidateEnvelope, *, now_s: float, generation: int) -> MotionCandidate:
        if generation != self._generation:
            raise ValueError("candidate belongs to an inactive lease generation")
        if envelope.option_instance_id in self._cancelled:
            raise ValueError("candidate option instance was cancelled")
        candidate = envelope.candidate
        if candidate.source_id != envelope.option_instance_id:
            raise ValueError("candidate source does not match option instance")
        if now_s * 1e9 >= candidate.valid_until_s * 1e9:
            raise ValueError("candidate lease has expired")
        if candidate.map_version != envelope.path.map_version:
            raise ValueError("candidate map version does not match path")
        if envelope.path.localization_epoch != candidate.localization_epoch:
            raise ValueError("candidate localization epoch does not match path")
        return candidate
