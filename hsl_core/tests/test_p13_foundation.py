# hsl_core/tests/test_p13_foundation.py
"""Boundary and degenerate fixtures for Phase 1.3."""

import math

import pytest

from hsl_core.contracts import ContractHeader, validate_covariance, validate_header, validate_polygon
from hsl_core.geometry import transform_point
from hsl_core.kinematics import integrate_unicycle, twist_from_wheel_rates, wheel_rates
from hsl_core.rules import (
    capture_predicate,
    first_contour_arrival,
    robust_capture_interval,
    segment_intersection,
)
from hsl_core.types import Pose2D


def _header(**overrides):
    values = dict(
        schema_version=2,
        source_id="fixture",
        source_session="session-a",
        seq=1,
        stage_id="stage-1",
        clock_epoch="clock-a",
        localization_epoch="loc-a",
        frame_id="odom",
        observation_stamp_ns=10,
        state_stamp_ns=10,
        publication_stamp_ns=11,
        valid_until_ns=20,
        map_version=1,
        topology_version=1,
        validity=1,
    )
    values.update(overrides)
    return ContractHeader(**values)


def test_contract_header_rejects_mixed_epoch_and_expiry():
    header = _header()
    validate_header(
        header,
        now_ns=12,
        expected_clock_epoch="clock-a",
        expected_localization_epoch="loc-a",
        expected_stage_id="stage-1",
        require_frame=True,
    )
    with pytest.raises(ValueError, match="clock_epoch"):
        validate_header(
            header,
            now_ns=12,
            expected_clock_epoch="clock-old",
            expected_localization_epoch="loc-a",
        )
    with pytest.raises(ValueError, match="expired"):
        validate_header(
            _header(valid_until_ns=12),
            now_ns=12,
            expected_clock_epoch="clock-a",
            expected_localization_epoch="loc-a",
        )
    with pytest.raises(ValueError, match="future"):
        validate_header(
            _header(publication_stamp_ns=30, valid_until_ns=40),
            now_ns=12,
            expected_clock_epoch="clock-a",
            expected_localization_epoch="loc-a",
        )
    assert _header(stage_id="") is not None
    with pytest.raises(ValueError, match="stage_id"):
        validate_header(
            _header(stage_id=""),
            now_ns=12,
            expected_clock_epoch="clock-a",
            expected_localization_epoch="loc-a",
            require_stage=True,
        )
    with pytest.raises(ValueError, match="INVALID"):
        validate_header(
            _header(validity=0),
            now_ns=12,
            expected_clock_epoch="clock-a",
            expected_localization_epoch="loc-a",
        )


def test_covariance_and_polygon_validation_are_explicit():
    assert validate_covariance((0.0,) * 9, 3) == (0.0,) * 9
    with pytest.raises(ValueError, match="symmetric"):
        validate_covariance((0.0, 1.0, 0.0, 0.0), 2)
    assert validate_polygon(((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)))
    with pytest.raises(ValueError, match="counterclockwise"):
        validate_polygon(((0.0, 0.0), (0.0, 1.0), (1.0, 0.0)))
    with pytest.raises(ValueError, match="distinct"):
        validate_polygon(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 0.0)))
    with pytest.raises(ValueError, match="simple|non-collinear"):
        validate_polygon(((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)))
    with pytest.raises(ValueError, match="positive integer"):
        validate_covariance((0.0,), 0)
    with pytest.raises(ValueError, match="non-negative integer"):
        ContractHeader(**_header().__dict__ | {"seq": True})


def test_exact_kinematics_straight_turn_and_small_omega_limit():
    pose = Pose2D(0.0, 0.0, 0.0)
    straight = integrate_unicycle(pose, 0.5, 0.0, 2.0)
    assert straight.x_m == pytest.approx(1.0)
    assert straight.y_m == pytest.approx(0.0)
    turn = integrate_unicycle(pose, 0.0, 1.0, 2.0)
    assert turn.x_m == pytest.approx(0.0)
    assert turn.y_m == pytest.approx(0.0)
    near_zero = integrate_unicycle(pose, 0.5, 1e-12, 2.0)
    assert near_zero.x_m == pytest.approx(1.0)
    assert near_zero.y_m == pytest.approx(1e-12, abs=1e-10)
    assert wheel_rates(0.5, 0.2, 0.23, 0.035) == pytest.approx(
        (13.628571428571429, 14.942857142857144)
    )
    assert twist_from_wheel_rates(
        *wheel_rates(0.5, 0.2, 0.23, 0.035), 0.23, 0.035
    ) == pytest.approx((0.5, 0.2))


def test_capture_boundaries_and_unknown_los():
    guardian = Pose2D(0.0, 0.0, 0.0)
    assert capture_predicate(guardian, Pose2D(0.449, 0.0, 0.0), line_of_sight=True)
    assert not capture_predicate(guardian, Pose2D(0.45, 0.0, 0.0), line_of_sight=True)
    assert capture_predicate(
        guardian,
        Pose2D(0.4 / math.sqrt(2.0), 0.4 / math.sqrt(2.0), 0.0),
        line_of_sight=True,
    )
    assert not capture_predicate(
        guardian,
        Pose2D(0.4 * math.cos(math.pi / 4.0 + 1e-6), 0.4 * math.sin(math.pi / 4.0 + 1e-6), 0.0),
        line_of_sight=True,
    )
    assert not capture_predicate(guardian, Pose2D(0.4, 0.0, 0.0), line_of_sight=None)
    with pytest.raises(ValueError, match="coincident"):
        capture_predicate(guardian, guardian, line_of_sight=True)
    with pytest.raises(ValueError, match="positive"):
        capture_predicate(guardian, Pose2D(0.1, 0.0, 0.0), line_of_sight=True, max_distance_m=math.nan)


def test_robust_capture_interval_and_geometry_degenerate_cases():
    interval = robust_capture_interval(0.17, 0.17, 0.01, 0.01)
    assert interval.lower_m == pytest.approx(0.36)
    assert interval.upper_m == pytest.approx(0.44)
    angular_interval = robust_capture_interval(
        0.17,
        0.17,
        0.01,
        0.0,
        angular_error_rad=0.1,
        angular_reference_radius_m=0.1,
    )
    assert angular_interval.lower_m == pytest.approx(0.36)
    assert segment_intersection((0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)) == pytest.approx(
        (0.5, 0.5)
    )
    assert segment_intersection((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)) is None
    assert transform_point((1.0, 0.0), 1.0, 2.0, math.pi / 2.0) == pytest.approx((1.0, 3.0))


def test_first_contour_contact_is_interpolated_and_not_endpoint_chord():
    contour = ((1.0, 0.0), (1.0, 1.0), (2.0, 1.0), (2.0, 0.0))
    trajectory = ((0.0, 0.5), (0.5, 0.5), (1.5, 0.5), (2.5, 0.5))
    event = first_contour_arrival(trajectory, contour, start_time_s=10.0, sample_period_s=1.0)
    assert event is not None
    assert event.segment_index == 1
    assert event.fraction == pytest.approx(0.5)
    assert event.time_s == pytest.approx(11.5)
    assert event.point == pytest.approx((1.0, 0.5))
    assert first_contour_arrival(((0.0, 0.0), (0.5, 0.0)), contour) is None


def test_covariance_psd_boundary_and_kinematic_time_scaling():
    assert validate_covariance((1.0, 1.0, 1.0, 1.0), 2) == pytest.approx(
        (1.0, 1.0, 1.0, 1.0)
    )
    with pytest.raises(ValueError, match="positive semidefinite"):
        validate_covariance((1.0, 2.0, 2.0, 1.0), 2)
    pose = Pose2D(0.2, -0.3, 0.7)
    one_step = integrate_unicycle(pose, 0.4, 0.6, 1.0)
    two_steps = integrate_unicycle(
        integrate_unicycle(pose, 0.4, 0.6, 0.5), 0.4, 0.6, 0.5
    )
    assert two_steps.x_m == pytest.approx(one_step.x_m)
    assert two_steps.y_m == pytest.approx(one_step.y_m)
    assert two_steps.theta_rad == pytest.approx(one_step.theta_rad)
