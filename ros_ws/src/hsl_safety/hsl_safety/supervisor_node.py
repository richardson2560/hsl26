# ros_ws/src/hsl_safety/hsl_safety/supervisor_node.py
"""Process-boundary runtime for the Phase-1 safety supervisor.

The stateful shell is ROS-independent. A ROS node may call ``tick`` from a
timer, publish the returned decision, and publish the heartbeat only after
that decision succeeds.
"""

from dataclasses import dataclass
from typing import Callable, Optional
import time

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
    """ROS-facing supervisor which is deliberately motion-disabled in R2.

    R2 establishes the process and output boundary.  It does *not* install a
    calibrated limits profile, therefore this node can only publish a STOP.
    A later calibrated release may replace the evaluator, but must retain this
    output ordering: final command, status, then heartbeat.
    """

    def __init__(self) -> None:
        # Imports live here so the pure state-machine/adapters remain testable
        # on a host which has not sourced ROS.
        import rclpy
        from geometry_msgs.msg import Twist
        from hsl_interfaces.msg import SafetyStatus, SupervisorHeartbeat, WatchdogHealth
        from rclpy.node import Node

        class _Node(Node):
            pass

        self._node = _Node("safety_supervisor")
        self._Twist = Twist
        self._SafetyStatus = SafetyStatus
        self._SupervisorHeartbeat = SupervisorHeartbeat
        self._WatchdogHealth = WatchdogHealth
        self._config_hash = self._node.declare_parameter(
            "config_hash", "UNCONFIGURED_NO_MOTION"
        ).value
        self._stage_id = self._node.declare_parameter("stage_id", "r2-disarmed").value
        rate_hz = float(self._node.declare_parameter("tick_rate_hz", 20.0).value)
        if not self._config_hash or not self._stage_id or rate_hz <= 0.0:
            raise ValueError("supervisor runtime parameters are invalid")
        self._watchdog_healthy = False
        self._command_pub = self._node.create_publisher(Twist, "/hsl/cmd_vel_final", 10)
        self._status_pub = self._node.create_publisher(SafetyStatus, "/hsl/safety_status", 10)
        self._heartbeat_pub = self._node.create_publisher(
            SupervisorHeartbeat, "/hsl/supervisor_heartbeat", 10
        )
        self._node.create_subscription(
            WatchdogHealth, "/hsl/watchdog_health", self._on_watchdog_health, 10
        )
        self._runtime = SupervisorRuntime(
            _NoMotionEvaluator(),
            config_hash=self._config_hash,
            watchdog_health=lambda: self._watchdog_healthy,
        )
        self._node.create_timer(1.0 / rate_hz, self._on_tick)

    @property
    def node(self):
        """Expose the underlying rclpy node only to the entry point."""

        return self._node

    def _on_watchdog_health(self, message) -> None:
        # A watchdog declaring a stop is not a usable watchdog authority.
        self._watchdog_healthy = bool(message.healthy) and not bool(message.stop_asserted)

    def _header(self, *, now_ros_ns: int, seq: int):
        from hsl_interfaces.msg import ContractHeader

        header = ContractHeader()
        header.schema_version = ContractHeader.REVISION_2
        header.source_id = "hsl_safety_supervisor"
        header.source_session = "r2-no-motion"
        header.seq = seq
        header.stage_id = self._stage_id
        header.clock_epoch = "ros-clock"
        header.localization_epoch = ""
        header.frame_id = ""
        header.observation_stamp.sec = now_ros_ns // 1_000_000_000
        header.observation_stamp.nanosec = now_ros_ns % 1_000_000_000
        header.state_stamp.sec = header.observation_stamp.sec
        header.state_stamp.nanosec = header.observation_stamp.nanosec
        header.publication_stamp.sec = header.observation_stamp.sec
        header.publication_stamp.nanosec = header.observation_stamp.nanosec
        valid_until_ns = now_ros_ns + 200_000_000
        header.valid_until.sec = valid_until_ns // 1_000_000_000
        header.valid_until.nanosec = valid_until_ns % 1_000_000_000
        header.map_version = 0
        header.topology_version = 0
        header.validity = ContractHeader.VALID
        return header

    def _on_tick(self) -> None:
        now_ros_ns = self._node.get_clock().now().nanoseconds
        tick = self._runtime.tick(
            None, now_ros_ns=now_ros_ns, now_steady_ns=time.monotonic_ns()
        )
        command = self._Twist()  # geometry_msgs defaults are exactly zero.
        try:
            self._command_pub.publish(command)
            from .adapters import encode_safety_status, encode_supervisor_heartbeat

            status = encode_safety_status(
                tick.evaluation,
                self._SafetyStatus(),
                meta=self._header(now_ros_ns=now_ros_ns, seq=tick.heartbeat_decision_seq),
                mode=0,
                limits_id="UNCONFIGURED_NO_MOTION",
                evaluation_time_ns=0,
            )
            self._status_pub.publish(status)
            # This remains false in R2, independently of any incoming data.
            heartbeat = encode_supervisor_heartbeat(
                message=self._SupervisorHeartbeat(),
                meta=self._header(now_ros_ns=now_ros_ns, seq=tick.heartbeat_decision_seq),
                decision_seq=tick.heartbeat_decision_seq,
                mode=0,
                permit_motion=False,
                config_hash=self._config_hash,
                active_option_instance_id="",
            )
            self._heartbeat_pub.publish(heartbeat)
        except Exception as exc:  # no heartbeat on an incomplete publication cycle
            self._node.get_logger().error(f"fail-closed supervisor publication error: {exc}")


class _NoMotionEvaluator:
    """R2 evaluator sentinel: calibration is intentionally absent."""

    def evaluate(self, snapshot, now_ros_ns, now_steady_ns):
        del snapshot, now_ros_ns, now_steady_ns
        raise RuntimeError("R2 has no calibrated motion profile")


def main(args=None):
    """Run the no-motion R2 supervisor in a sourced ROS environment."""

    import rclpy

    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node.node)
    finally:
        node.node.destroy_node()
        rclpy.shutdown()
