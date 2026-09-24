# ros_ws/src/hsl_safety/hsl_safety/supervisor_node.py
"""Process-boundary runtime for the Phase-1 safety supervisor.

The stateful shell is ROS-independent. A ROS node may call ``tick`` from a
timer, publish the returned decision, and publish the heartbeat only after
that decision succeeds.
"""

from dataclasses import dataclass
from typing import Callable, Optional

from hsl_core.control.safety import STOP, SafetyEvaluation, SafetySnapshot, SafetySupervisor


@dataclass(frozen=True)
class SupervisorTick:
    """Ordered outputs of one bounded supervisor cycle."""

    evaluation: SafetyEvaluation
    heartbeat_decision_seq: int
    heartbeat_permit_motion: bool


class SupervisorRuntime:
    """Run bounded safety ticks and fail closed on every runtime fault."""

    def __init__(
        self,
        evaluator: SafetySupervisor,
        *,
        config_hash: str,
        watchdog_health: Callable[[], bool],
    ) -> None:
        if not config_hash:
            raise ValueError("config_hash must be non-empty")
        self._evaluator = evaluator
        self._watchdog_health = watchdog_health
        self._decision_seq = 0

    @property
    def decision_seq(self) -> int:
        return self._decision_seq

    def tick(
        self,
        snapshot: Optional[SafetySnapshot],
        *,
        now_ros_ns: int,
        now_steady_ns: int,
    ) -> SupervisorTick:
        sequence = self._decision_seq
        self._decision_seq += 1
        try:
            if snapshot is None:
                raise ValueError("validated safety snapshot is missing")
            if not self._watchdog_health():
                raise ValueError("watchdog health is missing")
            evaluation = self._evaluator.evaluate(
                snapshot, now_ros_ns, now_steady_ns
            )
        except Exception:
            evaluation = self._zero_evaluation(snapshot, sequence)
        return SupervisorTick(
            evaluation=evaluation,
            heartbeat_decision_seq=sequence,
            heartbeat_permit_motion=evaluation.applied_v_mps > 0.0,
        )

    @staticmethod
    def _zero_evaluation(
        snapshot: Optional[SafetySnapshot], sequence: int
    ) -> SafetyEvaluation:
        return SafetyEvaluation(
            decision=STOP,
            primary_reason="INVALID_PAYLOAD",
            reasons=("INVALID_PAYLOAD",),
            candidate_seq=snapshot.candidate_seq if snapshot else sequence,
            proposed_v_mps=snapshot.candidate_v_mps if snapshot else 0.0,
            proposed_omega_rps=snapshot.candidate_omega_rps if snapshot else 0.0,
            applied_v_mps=0.0,
            applied_omega_rps=0.0,
            checked_clearance_m=snapshot.free_distance_m if snapshot else 0.0,
            required_stop_distance_m=0.0,
            response_bound_s=0.0,
            processing_time_s=0.0,
        )


class SafetySupervisorNode:
    """ROS-facing orchestration seam around the pure supervisor runtime."""

    def __init__(self, runtime: SupervisorRuntime) -> None:
        self._runtime = runtime

    def evaluate_tick(self, snapshot, *, now_ros_ns: int, now_steady_ns: int) -> SupervisorTick:
        return self._runtime.tick(
            snapshot, now_ros_ns=now_ros_ns, now_steady_ns=now_steady_ns
        )


def main(args=None):
    """ROS entry point reserved for the pinned ROS deployment wrapper."""

    del args
    raise RuntimeError(
        "ROS 2 runtime wrapper is unavailable until the pinned ROS environment is sourced"
    )
