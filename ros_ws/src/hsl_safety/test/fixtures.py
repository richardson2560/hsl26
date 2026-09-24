# ros_ws/src/hsl_safety/test/fixtures.py
"""Deterministic, ROS-free P1.5 publisher and sink fixtures."""

from copy import deepcopy
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, List


@dataclass
class DeterministicClock:
    """Independent ROS and steady clocks controlled only by the test."""

    ros_ns: int = 0
    steady_ns: int = 0

    def advance(self, *, ros_ns: int = 0, steady_ns: int = 0) -> None:
        if ros_ns < 0 or steady_ns < 0:
            raise ValueError("clock increments must be non-negative")
        self.ros_ns += ros_ns
        self.steady_ns += steady_ns


@dataclass
class MockPublisher:
    """In-memory publisher that deep-copies messages and records order."""

    messages: List[Any] = field(default_factory=list)

    def publish(self, message: Any) -> None:
        self.messages.append(deepcopy(message))


@dataclass
class MockOutputSink:
    """Separate autonomy, watchdog-stop and mux observation channels."""

    autonomy: MockPublisher = field(default_factory=MockPublisher)
    watchdog_stop: MockPublisher = field(default_factory=MockPublisher)
    mux_observation: MockPublisher = field(default_factory=MockPublisher)

    def publish_autonomy(self, command: Any) -> None:
        self.autonomy.publish(command)

    def publish_watchdog_stop(self, command: Any) -> None:
        self.watchdog_stop.publish(command)

    def observe_mux(self, command: Any) -> None:
        self.mux_observation.publish(command)


@dataclass
class DeterministicMockScenario:
    """Controllable source of sequence, epoch and lease changes."""

    clock: DeterministicClock = field(default_factory=DeterministicClock)
    source_session: str = "mock-session"
    clock_epoch: str = "mock-clock-0"
    localization_epoch: str = "mock-localization-0"
    sequence: int = 0

    def next_sequence(self) -> int:
        current = self.sequence
        self.sequence += 1
        return current

    def restart_clock(self) -> None:
        self.clock_epoch = f"{self.clock_epoch}-restarted"
        self.sequence = 0

    def restart_localization(self) -> None:
        self.localization_epoch = f"{self.localization_epoch}-restarted"
        self.sequence = 0

    def header(self, *, frame_id: str, lease_ns: int) -> Any:
        if lease_ns <= 0:
            raise ValueError("lease_ns must be positive")
        stamp = SimpleNamespace(
            sec=self.clock.ros_ns // 1_000_000_000,
            nanosec=self.clock.ros_ns % 1_000_000_000,
        )
        valid_until = SimpleNamespace(
            sec=(self.clock.ros_ns + lease_ns) // 1_000_000_000,
            nanosec=(self.clock.ros_ns + lease_ns) % 1_000_000_000,
        )
        return SimpleNamespace(
            schema_version=2,
            source_id="mock",
            source_session=self.source_session,
            seq=self.next_sequence(),
            stage_id="stage-mock",
            clock_epoch=self.clock_epoch,
            localization_epoch=self.localization_epoch,
            frame_id=frame_id,
            observation_stamp=stamp,
            state_stamp=stamp,
            publication_stamp=stamp,
            valid_until=valid_until,
            validity=1,
        )

    def candidate(self, *, option_instance_id: str, v: float, omega: float, lease_ns: int) -> Any:
        return SimpleNamespace(
            meta=self.header(frame_id="base_link", lease_ns=lease_ns),
            option_instance_id=option_instance_id,
            path_id="mock-path",
            ego_seq=self.sequence,
            obstacle_seq=self.sequence,
            v=v,
            omega=omega,
            horizon=SimpleNamespace(sec=0, nanosec=100_000_000),
            control_mode=0,
            limits_id="mock-limits",
        )

    def ego_local(self, *, v: float, omega: float, lease_ns: int) -> Any:
        return SimpleNamespace(
            meta=self.header(frame_id="odom", lease_ns=lease_ns),
            pose=SimpleNamespace(x=0.0, y=0.0, theta=0.0),
            v=v,
            omega=omega,
            pose_covariance=[0.0] * 9,
            twist_covariance=[0.0] * 4,
            healthy=True,
            slip=False,
            localization_valid=True,
            calibration_id="mock-calibration",
        )

    def obstacles(self, *, complete: bool, lease_ns: int) -> Any:
        return SimpleNamespace(
            meta=self.header(frame_id="odom", lease_ns=lease_ns),
            obstacles=[],
            coverage=SimpleNamespace(),
            pose_error_bound_m=0.0,
            map_error_bound_m=0.0,
            bounds_id="mock-bounds",
            complete=complete,
        )

    def match(self, *, motion_authorized: bool, lease_ns: int) -> Any:
        return SimpleNamespace(
            meta=self.header(frame_id="", lease_ns=lease_ns),
            motion_authorized=motion_authorized,
            event_hold=False,
            phase=2,
        )

    def execution(self, *, candidate_authorized: bool, lease_ns: int) -> Any:
        return SimpleNamespace(
            meta=self.header(frame_id="", lease_ns=lease_ns),
            option_instance_id="option-a",
            candidate_authorized=candidate_authorized,
            phase=2,
        )

    def watchdog_health(self, *, ready: bool, stop_latched: bool, lease_ns: int) -> Any:
        return SimpleNamespace(
            meta=self.header(frame_id="", lease_ns=lease_ns),
            ready=ready,
            stop_latched=stop_latched,
            config_hash="mock-config",
        )
