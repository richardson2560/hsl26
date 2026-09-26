# hsl_core/tests/test_safety.py
"""Adversarial tests for the bounded Phase-1 safety evaluator."""

import pytest

from hsl_core.control.safety import (
    ADMIT,
    LIMIT,
    STOP,
    LimitsProfile,
    SafetySnapshot,
    SafetySupervisor,
    TimingProfile,
)


def _supervisor() -> SafetySupervisor:
    return SafetySupervisor(
        LimitsProfile(
            limits_id="limits-a",
            calibration_id="calibration-a",
            wheel_separation_m=0.23,
            wheel_radius_m=0.035,
            max_wheel_rate_radps=20.0,
            speed_max_mps=0.6,
            yaw_rate_max_rps=1.0,
            b_forward_min_mps2=0.85,
            response_bound_s=0.075,
            clearance_margin_m=0.05,
        ),
        TimingProfile(
            candidate_lease_s=0.2,
            obstacle_lease_s=0.2,
            ego_lease_s=0.2,
            processing_budget_s=0.02,
        ),
    )


def _snapshot(**overrides) -> SafetySnapshot:
    values = dict(
        candidate_seq=4,
        candidate_v_mps=0.3,
        candidate_omega_rps=0.0,
        candidate_stamp_ns=1_000_000_000,
        candidate_valid_until_ns=1_200_000_000,
        obstacle_stamp_ns=1_000_000_000,
        ego_stamp_ns=1_000_000_000,
        now_ros_ns=1_100_000_000,
        frame_id="odom",
        expected_frame_id="odom",
        clock_epoch="clock-a",
        expected_clock_epoch="clock-a",
        localization_epoch="loc-a",
        expected_localization_epoch="loc-a",
        coverage_valid=True,
        free_distance_m=0.8,
        ego_speed_mps=0.0,
        option_instance_id="option-a",
        expected_option_instance_id="option-a",
        received_steady_ns=0,
        candidate_map_version=7,
        expected_map_version=7,
        candidate_topology_version=9,
        expected_topology_version=9,
        candidate_lease_generation=1,
        expected_lease_generation=1,
    )
    values.update(overrides)
    return SafetySnapshot(**values)


def test_admit_limit_and_stop_are_distinct_and_traceable():
    supervisor = _supervisor()
    admitted = supervisor.evaluate(_snapshot(), 1_110_000_000, 50)
    assert admitted.decision is ADMIT
    assert admitted.response_bound_s == pytest.approx(0.075)
    assert admitted.required_stop_distance_m > 0.0
    limited = supervisor.evaluate(
        _snapshot(candidate_v_mps=0.6, free_distance_m=0.2),
        1_100_000_000,
        50,
    )
    assert limited.decision is LIMIT
    assert limited.applied_v_mps < limited.proposed_v_mps
    stopped = supervisor.evaluate(
        _snapshot(coverage_valid=False), 1_100_000_000, 50
    )
    assert stopped.decision is STOP
    assert stopped.applied_v_mps == 0.0
    assert stopped.primary_reason == "OUTSIDE_COVERAGE"


@pytest.mark.parametrize(
    "field, value, reason",
    [
        ("candidate_valid_until_ns", 1_100_000_000, "STALE_CANDIDATE"),
        ("obstacle_stamp_ns", 700_000_000, "STALE_OBSTACLES"),
        ("ego_stamp_ns", 700_000_000, "STALE_EGO"),
        ("frame_id", "map", "TF_UNAVAILABLE"),
        ("clock_epoch", "clock-old", "EPOCH_MISMATCH"),
        ("option_instance_id", "option-old", "OPTION_REVOKED"),
        ("candidate_lease_generation", 2, "OPTION_REVOKED"),
        ("candidate_map_version", 6, "STALE_CANDIDATE"),
        ("candidate_topology_version", 8, "STALE_CANDIDATE"),
    ],
)
def test_stale_identity_and_epoch_failures_force_zero(field, value, reason):
    snapshot = _snapshot(**{field: value})
    result = _supervisor().evaluate(snapshot, 1_100_000_000, 50)
    assert result.decision is STOP
    assert reason in result.reasons
    assert result.applied_v_mps == 0.0


def test_future_candidate_and_nonzero_rotation_are_not_silently_admitted():
    future = _supervisor().evaluate(
        _snapshot(candidate_stamp_ns=1_101_000_000), 1_100_000_000, 50
    )
    assert future.decision is STOP
    assert future.primary_reason == "STALE_CANDIDATE"
    rotating = _supervisor().evaluate(
        _snapshot(candidate_omega_rps=0.2), 1_100_000_000, 50
    )
    assert rotating.decision is STOP
    assert rotating.primary_reason == "LIMITS_INVALID"
    future_obstacles = _supervisor().evaluate(
        _snapshot(obstacle_stamp_ns=1_101_000_000), 1_100_000_000, 50
    )
    assert future_obstacles.decision is STOP
    assert "STALE_OBSTACLES" in future_obstacles.reasons


def test_candidate_lease_rejects_long_valid_until_and_negative_velocity_without_crash():
    old_candidate = _supervisor().evaluate(
        _snapshot(
            candidate_valid_until_ns=2_000_000_000,
            candidate_stamp_ns=800_000_000,
        ),
        1_100_000_000,
        50,
    )
    assert old_candidate.decision is STOP
    assert old_candidate.primary_reason == "STALE_CANDIDATE"

    negative = _supervisor().evaluate(
        _snapshot(candidate_v_mps=-0.1),
        1_100_000_000,
        50,
    )
    assert negative.decision is STOP
    assert negative.primary_reason == "LIMITS_INVALID"


def test_steady_clock_regression_forces_zero():
    result = _supervisor().evaluate(_snapshot(received_steady_ns=100), 1_100_000_000, 50)
    assert result.decision is STOP
    assert result.primary_reason == "COMPUTE_OVERRUN"


@pytest.mark.parametrize(
    "field, value",
    [
        ("candidate_seq", -1),
        ("candidate_seq", True),
        ("coverage_valid", 1),
    ],
)
def test_snapshot_rejects_structurally_invalid_metadata(field, value):
    with pytest.raises(ValueError):
        _snapshot(**{field: value})


def test_acceleration_during_response_delay_is_used_by_supervisor():
    supervisor = SafetySupervisor(
        LimitsProfile(
            limits_id="limits-a",
            calibration_id="calibration-a",
            wheel_separation_m=0.23,
            wheel_radius_m=0.035,
            max_wheel_rate_radps=20.0,
            speed_max_mps=0.6,
            yaw_rate_max_rps=1.0,
            b_forward_min_mps2=0.85,
            response_bound_s=0.075,
            clearance_margin_m=0.05,
            acceleration_delay_mps2=0.2,
        ),
        TimingProfile(
            candidate_lease_s=0.2,
            obstacle_lease_s=0.2,
            ego_lease_s=0.2,
            processing_budget_s=0.02,
        ),
    )
    result = supervisor.evaluate(
        _snapshot(candidate_v_mps=0.45, free_distance_m=0.20),
        1_100_000_000,
        50,
    )
    assert result.decision is LIMIT
    assert result.applied_v_mps < result.proposed_v_mps


def test_already_infeasible_state_reports_braking_infeasible():
    result = _supervisor().evaluate(
        _snapshot(ego_speed_mps=0.6, free_distance_m=0.05),
        1_100_000_000,
        50,
    )
    assert result.decision is STOP
    assert result.primary_reason == "BRAKING_INFEASIBLE"


def test_compute_overrun_forces_zero_even_with_valid_inputs():
    result = _supervisor().evaluate(
        _snapshot(ego_speed_mps=0.5), 1_100_000_000, 21_000_000
    )
    assert result.decision is STOP
    assert result.primary_reason == "COMPUTE_OVERRUN"
    assert result.applied_v_mps == 0.0
    assert result.required_stop_distance_m > 0.0
