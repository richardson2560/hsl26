# hsl_core/tests/test_braking.py
# hsl_core/tests/test_braking.py
"""Numerical and invalid-input tests for Phase 1.4 braking models."""

import math

import pytest

from hsl_core.control.braking import (
    admissible_speed,
    admissible_speed_accelerating,
    stopping_distance,
    stopping_distance_accelerating,
    wheel_rates_within_limits,
)


def test_constant_speed_fixture_and_rationalized_inverse():
    assert stopping_distance(0.5, 0.85, 0.075) == pytest.approx(
        0.18455882352941178
    )
    speed = admissible_speed(0.20, 0.05, 0.85, 0.075, 2.0)
    assert speed == pytest.approx(0.445233, abs=1e-6)
    assert stopping_distance(speed, 0.85, 0.075) + 0.05 == pytest.approx(0.20)


def test_zero_clearance_caps_and_accelerating_model():
    assert admissible_speed(0.05, 0.05, 0.85, 0.075, 1.0) == 0.0
    assert stopping_distance_accelerating(0.5, 0.2, 0.85, 0.075) > stopping_distance(
        0.5, 0.85, 0.075
    )
    speed = admissible_speed_accelerating(0.30, 0.05, 0.2, 0.85, 0.075, 1.0)
    assert stopping_distance_accelerating(speed, 0.2, 0.85, 0.075) + 0.05 == pytest.approx(
        0.30, abs=1e-9
    )


def test_invalid_braking_and_wheel_inputs_are_rejected():
    with pytest.raises(ValueError):
        stopping_distance(-0.1, 0.85, 0.075)
    with pytest.raises(ValueError):
        stopping_distance(0.5, 0.0, 0.075)
    with pytest.raises(ValueError):
        admissible_speed(math.nan, 0.05, 0.85, 0.075, 1.0)
    assert wheel_rates_within_limits(0.5, 0.0, 0.23, 0.035, 20.0)
    assert not wheel_rates_within_limits(0.5, 0.5, 0.23, 0.035, 15.0)
