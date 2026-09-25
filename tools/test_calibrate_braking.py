# tools/test_calibrate_braking.py
"""Tests for the ROS-free Phase-2 braking calibration preparation tool."""

import json
from pathlib import Path

import pytest

from calibrate_braking import estimate_b_min, run_dry_run, write_report


def test_estimator_matches_constant_deceleration_model():
    distance = 0.4 * 0.075 + 0.4 * 0.4 / (2.0 * 0.85)
    assert estimate_b_min(0.4, distance, 0.075) == pytest.approx(0.85)


def test_dry_run_is_reproducible_and_explicitly_not_hardware_calibration():
    first = run_dry_run(
        speed_mps=0.4,
        sample_rate_hz=50.0,
        response_delay_s=0.075,
        deceleration_mps2=0.85,
        noise_std_m=0.001,
        trials=3,
        seed=20260925,
    )
    second = run_dry_run(
        speed_mps=0.4,
        sample_rate_hz=50.0,
        response_delay_s=0.075,
        deceleration_mps2=0.85,
        noise_std_m=0.001,
        trials=3,
        seed=20260925,
    )
    assert first == second
    assert first.status == "PASS_SYNTHETIC_ONLY"
    assert "not hardware" in first.source
    assert first.trial_count == 3
    assert first.measured_b_min_mps2 > 0.0


def test_report_is_json_serializable_and_target_is_created(tmp_path: Path):
    report = run_dry_run(
        speed_mps=0.4,
        sample_rate_hz=50.0,
        response_delay_s=0.075,
        deceleration_mps2=0.85,
        noise_std_m=0.0,
        trials=1,
        seed=7,
    )
    target = tmp_path / "nested" / "braking_report.json"
    write_report(report, target)
    payload = json.loads(target.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert payload["profile"] == "kinematic"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"speed_mps": 0.0},
        {"sample_rate_hz": 0.0},
        {"response_delay_s": -0.1},
        {"deceleration_mps2": 0.0},
        {"noise_std_m": -0.1},
        {"trials": 0},
    ],
)
def test_invalid_trial_parameters_are_rejected(kwargs):
    values = dict(
        speed_mps=0.4,
        sample_rate_hz=50.0,
        response_delay_s=0.075,
        deceleration_mps2=0.85,
        noise_std_m=0.001,
        trials=3,
        seed=1,
    )
    values.update(kwargs)
    with pytest.raises(ValueError):
        run_dry_run(**values)
