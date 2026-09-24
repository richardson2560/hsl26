# hsl_core/hsl_core/control/braking.py
"""Validated one-dimensional braking models and wheel-limit helpers.

All distances are metres, speeds are metres per second, accelerations are
metres per second squared, and angular wheel rates are radians per second.
"""

import math
from typing import Tuple


def _finite_nonnegative(value: float, name: str) -> float:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return float(value)


def _finite_positive(value: float, name: str) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def stopping_distance(speed: float, b_min: float, tau: float) -> float:
    """Return constant-speed delay plus minimum-braking distance."""

    speed = _finite_nonnegative(speed, "speed")
    b_min = _finite_positive(b_min, "b_min")
    tau = _finite_nonnegative(tau, "tau")
    return speed * tau + speed * speed / (2.0 * b_min)


def admissible_speed(
    clearance: float,
    margin: float,
    b_min: float,
    tau: float,
    speed_max: float,
) -> float:
    """Invert the constant-speed braking envelope and apply ``speed_max``."""

    clearance = _finite_nonnegative(clearance, "clearance")
    margin = _finite_nonnegative(margin, "margin")
    b_min = _finite_positive(b_min, "b_min")
    tau = _finite_nonnegative(tau, "tau")
    speed_max = _finite_nonnegative(speed_max, "speed_max")
    effective = max(0.0, clearance - margin)
    if effective == 0.0:
        return 0.0
    delay_term = b_min * tau
    root = math.sqrt(delay_term * delay_term + 2.0 * b_min * effective)
    limit = 2.0 * b_min * effective / (root + delay_term)
    return min(speed_max, limit)


def stopping_distance_accelerating(
    speed: float,
    a_plus: float,
    b_min: float,
    tau: float,
) -> float:
    """Return stopping distance when speed may increase during response delay."""

    speed = _finite_nonnegative(speed, "speed")
    a_plus = _finite_nonnegative(a_plus, "a_plus")
    b_min = _finite_positive(b_min, "b_min")
    tau = _finite_nonnegative(tau, "tau")
    delayed_speed = speed + a_plus * tau
    return (
        speed * tau
        + 0.5 * a_plus * tau * tau
        + delayed_speed * delayed_speed / (2.0 * b_min)
    )


def admissible_speed_accelerating(
    clearance: float,
    margin: float,
    a_plus: float,
    b_min: float,
    tau: float,
    speed_max: float,
) -> float:
    """Invert the delay-acceleration braking envelope.

    If acceleration during the delay already consumes the effective
    clearance, no positive speed is certified.
    """

    clearance = _finite_nonnegative(clearance, "clearance")
    margin = _finite_nonnegative(margin, "margin")
    a_plus = _finite_nonnegative(a_plus, "a_plus")
    b_min = _finite_positive(b_min, "b_min")
    tau = _finite_nonnegative(tau, "tau")
    speed_max = _finite_nonnegative(speed_max, "speed_max")
    effective = max(0.0, clearance - margin)
    baseline = stopping_distance_accelerating(0.0, a_plus, b_min, tau)
    if effective <= baseline:
        return 0.0
    coefficient = 1.0 / (2.0 * b_min)
    linear = tau + a_plus * tau / b_min
    constant = baseline - effective
    discriminant = linear * linear - 4.0 * coefficient * constant
    if discriminant < 0.0:
        raise ArithmeticError("braking inversion produced a negative discriminant")
    limit = (-linear + math.sqrt(max(0.0, discriminant))) / (2.0 * coefficient)
    return min(speed_max, max(0.0, limit))


def wheel_rates(
    speed: float,
    yaw_rate: float,
    wheel_separation_m: float,
    wheel_radius_m: float,
) -> Tuple[float, float]:
    """Return left/right wheel rates for a body command."""

    if not all(math.isfinite(float(value)) for value in (
        speed, yaw_rate, wheel_separation_m, wheel_radius_m
    )):
        raise ValueError("wheel inputs must be finite")
    if wheel_separation_m <= 0.0 or wheel_radius_m <= 0.0:
        raise ValueError("wheel geometry must be positive")
    return (
        (speed - yaw_rate * wheel_separation_m / 2.0) / wheel_radius_m,
        (speed + yaw_rate * wheel_separation_m / 2.0) / wheel_radius_m,
    )


def wheel_rates_within_limits(
    speed: float,
    yaw_rate: float,
    wheel_separation_m: float,
    wheel_radius_m: float,
    max_wheel_rate_radps: float,
) -> bool:
    """Check both wheel rates against a finite calibrated absolute limit."""

    max_wheel_rate_radps = _finite_nonnegative(
        max_wheel_rate_radps, "max_wheel_rate_radps"
    )
    left, right = wheel_rates(
        speed, yaw_rate, wheel_separation_m, wheel_radius_m
    )
    return abs(left) <= max_wheel_rate_radps and abs(right) <= max_wheel_rate_radps
