# hsl_core/hsl_core/control/execution.py
"""Candidate creation through the planning lease boundary."""

from ..planning.astar import PlannedPath
from ..planning.execution import CandidateEnvelope, ExecutionLease
from ..types import MotionCandidate, Pose2D
from .regulated_pursuit import PursuitConfig, make_candidate


class OptionExecutor:
    def __init__(self, lease: ExecutionLease | None = None) -> None:
        self._lease = lease or ExecutionLease()

    def start(self, option_instance_id: str) -> int:
        return self._lease.activate(option_instance_id)

    def cancel(self, option_instance_id: str) -> None:
        self._lease.cancel(option_instance_id)

    def propose(
        self,
        path: PlannedPath,
        pose: Pose2D,
        *,
        option_instance_id: str,
        generation: int,
        config: PursuitConfig,
        now_s: float,
        lease_s: float,
    ) -> CandidateEnvelope:
        candidate = make_candidate(
            path,
            pose,
            config=config,
            now_s=now_s,
            lease_s=lease_s,
            source_id=option_instance_id,
        )
        envelope = CandidateEnvelope(candidate, path, option_instance_id)
        self._lease.admit(envelope, now_s=now_s, generation=generation)
        return envelope
