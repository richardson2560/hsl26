# ros_ws/src/hsl_safety/hsl_safety/watchdog.py
"""Independent stop-watchdog state machine for the Phase-1 boundary."""

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional
import time


class WatchdogState(IntEnum):
    DISARMED = 0
    READY = 1
    ACTIVE = 2
    FAULT_LATCHED = 3


@dataclass(frozen=True)
class HeartbeatObservation:
    decision_seq: int
    stage_id: str
    config_hash: str
    received_steady_ns: int
    permit_motion: bool


@dataclass(frozen=True)
class WatchdogOutput:
    state: WatchdogState
    stop_asserted: bool
    healthy: bool
    last_decision_seq: int
    fault_reason: str


class WatchdogMachine:
    """Require fresh, ordered, identity-bound heartbeats before release."""

    def __init__(self, *, config_hash: str, heartbeat_lease_ns: int) -> None:
        if not config_hash or heartbeat_lease_ns <= 0:
            raise ValueError("watchdog configuration is invalid")
        self._config_hash = config_hash
        self._heartbeat_lease_ns = heartbeat_lease_ns
        self._state = WatchdogState.DISARMED
        self._last: Optional[HeartbeatObservation] = None
        self._stage_id = ""
        self._fault_reason = "STARTUP_DISARMED"

    @property
    def state(self) -> WatchdogState:
        return self._state

    def observe_heartbeat(self, heartbeat: HeartbeatObservation) -> None:
        if (
            heartbeat.decision_seq < 0
            or heartbeat.received_steady_ns < 0
            or not heartbeat.stage_id
            or heartbeat.config_hash != self._config_hash
        ):
            self._latch("INVALID_HEARTBEAT")
            return
        if self._last is not None:
            if heartbeat.decision_seq <= self._last.decision_seq:
                self._latch("HEARTBEAT_NOT_PROGRESSING")
                return
            if heartbeat.received_steady_ns < self._last.received_steady_ns:
                self._latch("STEADY_CLOCK_REGRESSION")
                return
        if self._stage_id and heartbeat.stage_id != self._stage_id:
            self._latch("STAGE_ID_MISMATCH")
            return
        self._stage_id = heartbeat.stage_id
        self._last = heartbeat
        if self._state == WatchdogState.DISARMED:
            self._state = WatchdogState.READY
            self._fault_reason = ""

    def tick(
        self,
        *,
        now_steady_ns: int,
        stage_authorized: bool,
        supervisor_healthy: bool,
    ) -> WatchdogOutput:
        if now_steady_ns < 0:
            raise ValueError("now_steady_ns must be non-negative")
        if self._state == WatchdogState.FAULT_LATCHED:
            return self._output(stop=True, healthy=False)
        if self._last is None:
            return self._output(stop=True, healthy=False)
        age = now_steady_ns - self._last.received_steady_ns
        if age < 0 or age > self._heartbeat_lease_ns:
            self._latch("STALE_HEARTBEAT")
            return self._output(stop=True, healthy=False)
        if not supervisor_healthy:
            self._latch("SUPERVISOR_UNHEALTHY")
            return self._output(stop=True, healthy=False)
        if not stage_authorized or not self._last.permit_motion:
            self._state = WatchdogState.READY
            return self._output(stop=True, healthy=True)
        self._state = WatchdogState.ACTIVE
        return self._output(stop=False, healthy=True)

    def rearm(
        self,
        *,
        stage_id: str,
        config_hash: str,
        authorization_ref: str,
        near_zero: bool,
        dwell_satisfied: bool,
    ) -> bool:
        if self._state != WatchdogState.FAULT_LATCHED:
            return False
        if (
            not authorization_ref
            or config_hash != self._config_hash
            or stage_id != self._stage_id
            or not near_zero
            or not dwell_satisfied
        ):
            return False
        self._state = WatchdogState.READY
        self._fault_reason = ""
        self._last = None
        return True

    def _latch(self, reason: str) -> None:
        self._state = WatchdogState.FAULT_LATCHED
        self._fault_reason = reason

    def _output(self, *, stop: bool, healthy: bool) -> WatchdogOutput:
        return WatchdogOutput(
            state=self._state,
            stop_asserted=stop,
            healthy=healthy,
            last_decision_seq=self._last.decision_seq if self._last else 0,
            fault_reason=self._fault_reason,
        )


class StopWatchdogNode:
    """Independent ROS watchdog which continuously asserts zero in R2."""

    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import Twist
        from hsl_interfaces.msg import MatchState, SupervisorHeartbeat, WatchdogHealth
        from hsl_interfaces.srv import RearmSafety
        from rclpy.node import Node

        class _Node(Node):
            pass

        self._node = _Node("safety_watchdog")
        self._Twist = Twist
        self._WatchdogHealth = WatchdogHealth
        self._config_hash = self._node.declare_parameter(
            "config_hash", "UNCONFIGURED_NO_MOTION"
        ).value
        self._stage_id = self._node.declare_parameter("stage_id", "r2-disarmed").value
        heartbeat_lease_s = float(
            self._node.declare_parameter("heartbeat_lease_s", 0.15).value
        )
        stop_rate_hz = float(self._node.declare_parameter("stop_rate_hz", 20.0).value)
        if not self._config_hash or not self._stage_id or heartbeat_lease_s <= 0.0 or stop_rate_hz <= 0.0:
            raise ValueError("watchdog runtime parameters are invalid")
        self._machine = WatchdogMachine(
            config_hash=self._config_hash,
            heartbeat_lease_ns=int(heartbeat_lease_s * 1_000_000_000),
        )
        self._last_heartbeat_steady_ns: Optional[int] = None
        self._motion_authorized = False
        self._stop_pub = self._node.create_publisher(Twist, "/hsl/cmd_vel_stop", 10)
        self._health_pub = self._node.create_publisher(
            WatchdogHealth, "/hsl/watchdog_health", 10
        )
        self._node.create_subscription(
            SupervisorHeartbeat, "/hsl/supervisor_heartbeat", self._on_heartbeat, 10
        )
        self._node.create_subscription(MatchState, "/match/match_state", self._on_match, 10)
        self._node.create_service(RearmSafety, "/hsl/rearm_safety", self._on_rearm)
        self._node.create_timer(1.0 / stop_rate_hz, self._on_tick)

    @property
    def node(self):
        return self._node

    def _on_heartbeat(self, message) -> None:
        received = time.monotonic_ns()
        self._last_heartbeat_steady_ns = received
        self._machine.observe_heartbeat(
            HeartbeatObservation(
                decision_seq=int(message.decision_seq),
                stage_id=message.meta.stage_id,
                config_hash=message.config_hash,
                received_steady_ns=received,
                permit_motion=bool(message.permit_motion),
            )
        )

    def _on_match(self, message) -> None:
        # The R2 supervisor never permits motion.  This input is retained for
        # an explicit later activation path, not treated as sufficient alone.
        self._motion_authorized = bool(message.motion_authorized)

    def _health_header(self, *, now_ros_ns: int, seq: int):
        from hsl_interfaces.msg import ContractHeader

        header = ContractHeader()
        header.schema_version = ContractHeader.REVISION_2
        header.source_id = "hsl_safety_watchdog"
        header.source_session = "r2-no-motion"
        header.seq = seq
        header.stage_id = self._stage_id
        header.clock_epoch = "ros-clock"
        header.localization_epoch = ""
        header.frame_id = ""
        for field in ("observation_stamp", "state_stamp", "publication_stamp"):
            stamp = getattr(header, field)
            stamp.sec = now_ros_ns // 1_000_000_000
            stamp.nanosec = now_ros_ns % 1_000_000_000
        valid_until_ns = now_ros_ns + 200_000_000
        header.valid_until.sec = valid_until_ns // 1_000_000_000
        header.valid_until.nanosec = valid_until_ns % 1_000_000_000
        header.map_version = 0
        header.topology_version = 0
        header.validity = ContractHeader.VALID
        return header

    def _on_tick(self) -> None:
        now_steady_ns = time.monotonic_ns()
        output = self._machine.tick(
            now_steady_ns=now_steady_ns,
            # R2 is unconditionally no-motion regardless of MatchState.
            stage_authorized=False,
            supervisor_healthy=self._last_heartbeat_steady_ns is not None,
        )
        # Publish a zero continuously, including cold start and all failures.
        self._stop_pub.publish(self._Twist())
        now_ros_ns = self._node.get_clock().now().nanoseconds
        health = self._WatchdogHealth()
        health.meta = self._health_header(
            now_ros_ns=now_ros_ns, seq=output.last_decision_seq
        )
        health.healthy = output.healthy
        health.stop_asserted = True
        health.last_decision_seq = output.last_decision_seq
        age_ns = (
            now_steady_ns - self._last_heartbeat_steady_ns
            if self._last_heartbeat_steady_ns is not None
            else 0
        )
        health.heartbeat_age_s = max(0.0, age_ns / 1_000_000_000.0)
        health.config_hash = self._config_hash
        health.fault_reason = output.fault_reason or "R2_NO_MOTION"
        self._health_pub.publish(health)

    def _on_rearm(self, request, response):
        # No measured base velocity and no calibrated recovery policy exist in
        # R2.  A service must fail explicitly rather than offering a fake rearm.
        del request
        response.accepted = False
        response.reason = "R2_NO_MOTION: rearm requires calibrated downstream evidence"
        return response


def main(args=None):
    """Run the independent no-motion watchdog in a sourced ROS environment."""

    import rclpy

    rclpy.init(args=args)
    node = StopWatchdogNode()
    try:
        rclpy.spin(node.node)
    finally:
        node.node.destroy_node()
        rclpy.shutdown()
