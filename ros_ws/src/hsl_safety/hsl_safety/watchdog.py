# ros_ws/src/hsl_safety/hsl_safety/watchdog.py
"""Independent stop-watchdog state machine for the Phase-1 boundary."""

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


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
    """ROS-facing orchestration seam around the independent watchdog."""

    def __init__(self, machine: WatchdogMachine) -> None:
        self._machine = machine

    def observe_heartbeat(self, heartbeat: HeartbeatObservation) -> None:
        self._machine.observe_heartbeat(heartbeat)

    def watchdog_tick(self, *, now_steady_ns: int, stage_authorized: bool,
                      supervisor_healthy: bool) -> WatchdogOutput:
        return self._machine.tick(
            now_steady_ns=now_steady_ns,
            stage_authorized=stage_authorized,
            supervisor_healthy=supervisor_healthy,
        )


def main(args=None):
    """ROS entry point reserved for the pinned ROS deployment wrapper."""

    del args
    raise RuntimeError(
        "ROS 2 runtime wrapper is unavailable until the pinned ROS environment is sourced"
    )
