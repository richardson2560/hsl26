# hsl_core/hsl_core/kinematics.py
# hsl_core/hsl_core/kinematics.py
"""Exact planar differential-drive kinematics without ROS or implicit clocks.

"""

import math
from typing import Tuple

from .types import Pose2D


def _sinc(value: float) -> float:
    if abs(value) < 1e-8:
        return 1.0 - value * value / 6.0 + value**4 / 120.0
    return math.sin(value) / value


def integrate_unicycle(
    pose: Pose2D,
    linear_mps: float,
    angular_rps: float,
    dt_s: float,
) -> Pose2D:
    """Integrate a constant body twist over ``dt_s`` using the SE(2) exponential."""

    if not all(math.isfinite(float(value)) for value in (linear_mps, angular_rps, dt_s)):
        raise ValueError("kinematic inputs must be finite")
    if dt_s < 0.0:
        raise ValueError("dt_s must be non-negative")
    half_turn = 0.5 * angular_rps * dt_s
    scale = linear_mps * dt_s * _sinc(half_turn)
    heading = pose.theta_rad + half_turn
    return Pose2D(
        pose.x_m + scale * math.cos(heading),
        pose.y_m + scale * math.sin(heading),
        pose.theta_rad + angular_rps * dt_s,
    )


def wheel_rates(
    linear_mps: float,
    angular_rps: float,
    wheel_separation_m: float,
    wheel_radius_m: float,
) -> Tuple[float, float]:
    """Return left/right wheel angular rates in rad/s."""

    if wheel_separation_m <= 0.0 or wheel_radius_m <= 0.0:
        raise ValueError("wheel geometry must be positive")
    if not all(math.isfinite(float(value)) for value in (
        linear_mps, angular_rps, wheel_separation_m, wheel_radius_m
    )):
        raise ValueError("wheel inputs must be finite")
    return (
        (linear_mps - angular_rps * wheel_separation_m / 2.0) / wheel_radius_m,
        (linear_mps + angular_rps * wheel_separation_m / 2.0) / wheel_radius_m,
    )


def twist_from_wheel_rates(
    left_rate_rps: float,
    right_rate_rps: float,
    wheel_separation_m: float,
    wheel_radius_m: float,
) -> Tuple[float, float]:
    """Recover body ``(v, omega)`` from wheel rates.

    Wheel rates are rad/s; multiplying by wheel radius (m) gives m/s.
    Dividing their difference by wheel separation (m) gives rad/s because
    radians are dimensionless in SI.
    """

    if wheel_separation_m <= 0.0 or wheel_radius_m <= 0.0:
        raise ValueError("wheel geometry must be positive")
    if not all(math.isfinite(float(value)) for value in (
        left_rate_rps, right_rate_rps, wheel_separation_m, wheel_radius_m
    )):
        raise ValueError("wheel inputs must be finite")
    left_speed_mps = left_rate_rps * wheel_radius_m
    right_speed_mps = right_rate_rps * wheel_radius_m
    return (
        (left_speed_mps + right_speed_mps) / 2.0,
        (right_speed_mps - left_speed_mps) / wheel_separation_m,
    )
