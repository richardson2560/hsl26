# hsl_core/tests/test_safety.py
"""Adversarial tests for the bounded Phase-1 safety evaluator."""

import pytest

from hsl_core.control.safety import (
    ADMIT,
    ControlMode,
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
            rotation_radius_m=0.20,
            rotation_clearance_margin_m=0.05,
            linear_rest_tolerance_mps=0.005,
            angular_acceleration_max_rps2=0.5,
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
        control_mode=ControlMode.TRACK_PATH,
        free_distance_360_m=1.0,
        coverage_360_valid=True,
        position_error_bound_m=0.01,
        control_period_s=0.05,
        ego_angular_velocity_rps=0.0,
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


def test_align_mode_admits_only_resting_clearance_checked_and_slew_limited_turns():
    result = _supervisor().evaluate(
        _snapshot(
            control_mode=ControlMode.ALIGN,
            candidate_v_mps=0.0,
            candidate_omega_rps=0.025,
        ),
        1_100_000_000,
        50,
    )
    assert result.decision is ADMIT
    assert result.applied_v_mps == 0.0
    assert result.applied_omega_rps == pytest.approx(0.025)
    assert result.checked_clearance_m == pytest.approx(1.0)


@pytest.mark.parametrize(
    "overrides, reason",
    [
        ({"candidate_v_mps": 0.01}, "LIMITS_INVALID"),
        ({"ego_speed_mps": 0.006}, "WAITING_LINEAR_REST"),
        ({"coverage_360_valid": False}, "OUTSIDE_COVERAGE"),
        ({"free_distance_360_m": 0.259}, "ROTATION_CLEARANCE_INSUFFICIENT"),
        ({"candidate_omega_rps": 0.026}, "ANGULAR_ACCELERATION_EXCEEDED"),
        ({"candidate_omega_rps": 1.01}, "LIMITS_INVALID"),
        ({"control_period_s": 0.0}, "ROTATION_PROFILE_INVALID"),
    ],
)
def test_align_mode_fails_closed_for_invalid_or_unsafe_rotation(overrides, reason):
    values = {
        "control_mode": ControlMode.ALIGN,
        "candidate_v_mps": 0.0,
        "candidate_omega_rps": 0.025,
    }
    values.update(overrides)
    result = _supervisor().evaluate(
        _snapshot(**values), 1_100_000_000, 50
    )
    assert result.decision is STOP
    assert result.primary_reason == reason
    assert result.applied_v_mps == 0.0
    assert result.applied_omega_rps == 0.0


def test_align_mode_checks_wheel_rate_and_measured_position_uncertainty():
    low_wheel_supervisor = SafetySupervisor(
        LimitsProfile(
            limits_id="limits-low-wheel",
            calibration_id="calibration-low-wheel",
            wheel_separation_m=0.23,
            wheel_radius_m=0.035,
            max_wheel_rate_radps=3.0,
            speed_max_mps=0.6,
            yaw_rate_max_rps=1.0,
            b_forward_min_mps2=0.85,
            response_bound_s=0.075,
            clearance_margin_m=0.05,
            rotation_radius_m=0.20,
            rotation_clearance_margin_m=0.05,
            linear_rest_tolerance_mps=0.005,
            angular_acceleration_max_rps2=0.5,
        ),
        TimingProfile(0.2, 0.2, 0.2, 0.02),
    )
    beyond_wheel_rate = low_wheel_supervisor.evaluate(
        _snapshot(
            control_mode=ControlMode.ALIGN,
            candidate_v_mps=0.0,
            candidate_omega_rps=1.0,
            ego_angular_velocity_rps=0.975,
            free_distance_360_m=0.3,
        ),
        1_100_000_000,
        50,
    )
    assert beyond_wheel_rate.decision is STOP
    assert beyond_wheel_rate.primary_reason == "LIMITS_INVALID"

    insufficient_with_pose_error = _supervisor().evaluate(
        _snapshot(
            control_mode=ControlMode.ALIGN,
            candidate_v_mps=0.0,
            candidate_omega_rps=0.0,
            free_distance_360_m=0.26,
            position_error_bound_m=0.02,
        ),
        1_100_000_000,
        50,
    )
    assert insufficient_with_pose_error.decision is STOP
    assert insufficient_with_pose_error.primary_reason == "ROTATION_CLEARANCE_INSUFFICIENT"


def _curved_supervisor(*, lateral_acceleration=0.5, max_wheel_rate=20.0):
    return SafetySupervisor(
        LimitsProfile(
            limits_id="curved-limits",
            calibration_id="sil-fixture-only",
            wheel_separation_m=0.23,
            wheel_radius_m=0.035,
            max_wheel_rate_radps=max_wheel_rate,
            speed_max_mps=0.6,
            yaw_rate_max_rps=1.0,
            b_forward_min_mps2=0.85,
            response_bound_s=0.075,
            clearance_margin_m=0.05,
            lateral_acceleration_max_mps2=lateral_acceleration,
        ),
        TimingProfile(0.2, 0.2, 0.2, 0.02),
    )


def test_curved_path_requires_explicit_swept_tube_coverage():
    result = _curved_supervisor().evaluate(
        _snapshot(candidate_v_mps=0.3, candidate_omega_rps=0.2),
        1_100_000_000,
        50,
    )
    assert result.decision is STOP
    assert result.primary_reason == "CURVED_PATH_UNCERTIFIED"
    assert result.applied_v_mps == 0.0
    assert result.applied_omega_rps == 0.0


def test_curved_path_is_admitted_only_with_lateral_braking_and_wheel_bounds():
    result = _curved_supervisor().evaluate(
        _snapshot(
            candidate_v_mps=0.3,
            candidate_omega_rps=0.2,
            curved_path_coverage_valid=True,
            curved_path_clearance_m=0.8,
        ),
        1_100_000_000,
        50,
    )
    assert result.decision is ADMIT
    assert result.applied_v_mps == pytest.approx(0.3)
    assert result.applied_omega_rps == pytest.approx(0.2)
    assert result.checked_clearance_m == pytest.approx(0.8)
    assert abs(result.applied_v_mps * result.applied_omega_rps) <= 0.5


def test_curved_limit_scales_linear_and_angular_velocity_together():
    supervisor = _curved_supervisor(lateral_acceleration=0.2)
    result = supervisor.evaluate(
        _snapshot(
            candidate_v_mps=0.5,
            candidate_omega_rps=0.5,
            curved_path_coverage_valid=True,
            curved_path_clearance_m=2.0,
        ),
        1_100_000_000,
        50,
    )
    assert result.decision is LIMIT
    assert 0.0 < result.applied_v_mps < result.proposed_v_mps
    assert result.applied_omega_rps == pytest.approx(
        result.proposed_omega_rps * result.applied_v_mps / result.proposed_v_mps
    )
    assert abs(result.applied_v_mps * result.applied_omega_rps) <= 0.2 + 1e-12


@pytest.mark.parametrize(
    "supervisor, speed, yaw_rate",
    [
        (_curved_supervisor(), 0.5, 1.2),
        (_curved_supervisor(max_wheel_rate=8.0), 0.5, 0.5),
    ],
)
def test_curved_yaw_and_wheel_bounds_limit_without_changing_curvature(
    supervisor, speed, yaw_rate
):
    result = supervisor.evaluate(
        _snapshot(
            candidate_v_mps=speed,
            candidate_omega_rps=yaw_rate,
            curved_path_coverage_valid=True,
            curved_path_clearance_m=2.0,
        ),
        1_100_000_000,
        50,
    )
    assert result.decision is LIMIT
    assert 0.0 < result.applied_v_mps < speed
    assert result.applied_omega_rps == pytest.approx(
        yaw_rate * result.applied_v_mps / speed
    )


@pytest.mark.parametrize(
    "overrides, reason",
    [
        ({"candidate_v_mps": 0.0}, "LIMITS_INVALID"),
        ({"curved_path_clearance_m": 0.0}, "COMMAND_LIMITED"),
        ({"coverage_valid": False}, "OUTSIDE_COVERAGE"),
    ],
)
def test_curved_path_still_fails_closed_for_invalid_speed_clearance_or_coverage(
    overrides, reason
):
    values = {
        "candidate_v_mps": 0.3,
        "candidate_omega_rps": 0.2,
        "curved_path_coverage_valid": True,
        "curved_path_clearance_m": 0.8,
    }
    values.update(overrides)
    result = _curved_supervisor().evaluate(
        _snapshot(**values), 1_100_000_000, 50
    )
    assert result.decision in (STOP, LIMIT)
    assert result.primary_reason == reason
    if result.decision == LIMIT:
        assert result.applied_v_mps == 0.0
        assert result.applied_omega_rps == 0.0
    if result.decision is STOP or result.applied_v_mps == 0.0:
        assert result.applied_v_mps == 0.0
        assert result.applied_omega_rps == 0.0
