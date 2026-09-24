# ros_ws/src/hsl_safety/test/test_p16_runtime.py
from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from hsl_core.control.safety import SafetyEvaluation, STOP
from hsl_safety.adapters import encode_safety_status, encode_supervisor_heartbeat
from hsl_safety.supervisor_node import SupervisorRuntime
from hsl_safety.watchdog import (
    HeartbeatObservation,
    WatchdogMachine,
    WatchdogState,
)


@dataclass
class FakeSnapshot:
    candidate_seq: int = 8
    candidate_v_mps: float = 0.4
    candidate_omega_rps: float = 0.0
    free_distance_m: float = 1.0


def _evaluation() -> SafetyEvaluation:
    return SafetyEvaluation(
        decision=1,
        primary_reason="ADMIT",
        reasons=("ADMIT",),
        candidate_seq=8,
        proposed_v_mps=0.4,
        proposed_omega_rps=0.0,
        applied_v_mps=0.4,
        applied_omega_rps=0.0,
        checked_clearance_m=1.0,
        required_stop_distance_m=0.2,
        response_bound_s=0.1,
        processing_time_s=0.001,
    )


class FakeEvaluator:
    def evaluate(self, snapshot, now_ros_ns, now_steady_ns):
        return _evaluation()


class FailingEvaluator:
    def evaluate(self, snapshot, now_ros_ns, now_steady_ns):
        raise RuntimeError("simulated evaluator failure")


def test_supervisor_heartbeat_sequence_and_health_gate():
    runtime = SupervisorRuntime(
        FakeEvaluator(), config_hash="cfg", watchdog_health=lambda: True
    )
    first = runtime.tick(FakeSnapshot(), now_ros_ns=10, now_steady_ns=10)
    second = runtime.tick(FakeSnapshot(), now_ros_ns=20, now_steady_ns=20)
    assert first.heartbeat_decision_seq == 0
    assert second.heartbeat_decision_seq == 1
    assert first.heartbeat_permit_motion

    unhealthy = SupervisorRuntime(
        FakeEvaluator(), config_hash="cfg", watchdog_health=lambda: False
    )
    output = unhealthy.tick(FakeSnapshot(), now_ros_ns=10, now_steady_ns=10)
    assert output.evaluation.decision == STOP
    assert output.evaluation.applied_v_mps == 0.0


def test_supervisor_exception_fails_closed():
    runtime = SupervisorRuntime(
        FailingEvaluator(), config_hash="cfg", watchdog_health=lambda: True
    )
    output = runtime.tick(FakeSnapshot(), now_ros_ns=10, now_steady_ns=10)
    assert output.evaluation.decision == STOP
    assert output.evaluation.primary_reason == "INVALID_PAYLOAD"


def test_watchdog_requires_fresh_progressing_identity_bound_heartbeat():
    watchdog = WatchdogMachine(config_hash="cfg", heartbeat_lease_ns=100)
    assert watchdog.state == WatchdogState.DISARMED
    startup = watchdog.tick(
        now_steady_ns=0, stage_authorized=True, supervisor_healthy=True
    )
    assert startup.stop_asserted
    assert watchdog.state == WatchdogState.DISARMED
    assert not watchdog.rearm(
        stage_id="stage", config_hash="cfg", authorization_ref="auth",
        near_zero=True, dwell_satisfied=True
    )


def test_watchdog_active_only_after_fresh_heartbeat_and_authority():
    watchdog = WatchdogMachine(config_hash="cfg", heartbeat_lease_ns=100)
    watchdog.observe_heartbeat(
        HeartbeatObservation(4, "stage", "cfg", 1000, True)
    )
    ready = watchdog.tick(
        now_steady_ns=1050, stage_authorized=False, supervisor_healthy=True
    )
    assert ready.state == WatchdogState.READY
    assert ready.stop_asserted
    active = watchdog.tick(
        now_steady_ns=1050, stage_authorized=True, supervisor_healthy=True
    )
    assert active.state == WatchdogState.ACTIVE
    assert not active.stop_asserted

    watchdog.observe_heartbeat(
        HeartbeatObservation(4, "stage", "cfg", 1060, True)
    )
    assert watchdog.state == WatchdogState.FAULT_LATCHED


@pytest.mark.parametrize("reason", ["stale", "supervisor"])
def test_watchdog_latches_on_mutual_health_failure(reason):
    watchdog = WatchdogMachine(config_hash="cfg", heartbeat_lease_ns=100)
    watchdog.observe_heartbeat(
        HeartbeatObservation(1, "stage", "cfg", 1000, True)
    )
    output = watchdog.tick(
        now_steady_ns=1101 if reason == "stale" else 1001,
        stage_authorized=True,
        supervisor_healthy=reason != "supervisor",
    )
    assert output.stop_asserted
    assert output.state == WatchdogState.FAULT_LATCHED


def test_rearm_revokes_old_heartbeat_until_new_progress():
    watchdog = WatchdogMachine(config_hash="cfg", heartbeat_lease_ns=100)
    watchdog.observe_heartbeat(
        HeartbeatObservation(1, "stage", "cfg", 1000, True)
    )
    watchdog.tick(
        now_steady_ns=1101, stage_authorized=True, supervisor_healthy=True
    )
    assert watchdog.rearm(
        stage_id="stage",
        config_hash="cfg",
        authorization_ref="operator",
        near_zero=True,
        dwell_satisfied=True,
    )
    assert watchdog.state == WatchdogState.READY
    after_rearm = watchdog.tick(
        now_steady_ns=1200, stage_authorized=True, supervisor_healthy=True
    )
    assert after_rearm.stop_asserted


def test_output_encoders_copy_metadata_and_preserve_wire_values():
    status = SimpleNamespace(evaluation_time=SimpleNamespace(sec=0, nanosec=0))
    metadata = SimpleNamespace(token=["original"])
    encoded = encode_safety_status(
        _evaluation(),
        status,
        meta=metadata,
        mode=2,
        limits_id="limits-v1",
        evaluation_time_ns=1_234_567_890,
    )
    metadata.token.append("mutated")
    assert encoded.mode == 2
    assert encoded.decision == 1
    assert encoded.evaluation_time.sec == 1
    assert encoded.evaluation_time.nanosec == 234_567_890
    assert encoded.meta.token == ["original"]

    heartbeat = SimpleNamespace()
    encode_supervisor_heartbeat(
        message=heartbeat,
        meta=metadata,
        decision_seq=9,
        mode=2,
        permit_motion=True,
        config_hash="cfg",
        active_option_instance_id="option-a",
    )
    assert heartbeat.decision_seq == 9
    assert heartbeat.permit_motion is True


@pytest.mark.parametrize(
    "kwargs",
    [
        {"mode": 5},
        {"limits_id": ""},
        {"evaluation_time_ns": -1},
    ],
)
def test_output_encoder_rejects_invalid_contract_values(kwargs):
    defaults = {
        "meta": SimpleNamespace(),
        "mode": 2,
        "limits_id": "limits-v1",
        "evaluation_time_ns": 1,
    }
    defaults.update(kwargs)
    with pytest.raises(ValueError):
        encode_safety_status(
            _evaluation(),
            SimpleNamespace(evaluation_time=SimpleNamespace(sec=0, nanosec=0)),
            **defaults,
        )
