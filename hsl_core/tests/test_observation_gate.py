import numpy as np
import pytest

from hsl_core.kinematics import PoseSample
from hsl_core.mapping import LocalObstacleBuilder
from hsl_core.perception.observation_gate import (
    NormalizedCloud,
    ObservationContext,
    build_local_observation,
)


def _context(now_ns=2_000_000_000):
    return ObservationContext(
        now_ns=now_ns,
        max_age_ns=500_000_000,
        expected_clock_epoch="clock-a",
        expected_localization_epoch="loc-a",
        expected_lidar_frame_id="lidar_link",
        expected_calibration_id="cal-a",
    )


def _cloud(**changes):
    values = {
        "points_xyz": np.array([[1.0, 0.0, 0.1]], dtype=float),
        "point_stamps_ns": np.array([1_900_000_000], dtype=np.int64),
        "observation_stamp_ns": 2_000_000_000,
        "frame_id": "lidar_link",
        "clock_epoch": "clock-a",
        "localization_epoch": "loc-a",
        "calibration_id": "cal-a",
    }
    values.update(changes)
    return NormalizedCloud(**values)


def _history():
    return (
        PoseSample(1.9, np.eye(4)),
        PoseSample(2.0, np.eye(4)),
    )


def test_r3_observation_gate_builds_only_coherent_fresh_geometry():
    snapshot = build_local_observation(
        _cloud(), _history(), np.eye(4), context=_context(), builder=LocalObstacleBuilder()
    )
    assert snapshot.complete
    assert snapshot.obstacles_xyz.shape == (1, 3)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"frame_id": "wrong"}, "frame_id"),
        ({"clock_epoch": "old"}, "clock_epoch"),
        ({"localization_epoch": "old"}, "localization_epoch"),
        ({"calibration_id": "unknown"}, "calibration_id"),
        ({"observation_stamp_ns": 1_000_000_000}, "stale"),
        ({"point_stamps_ns": np.array([2_100_000_000], dtype=np.int64)}, "cannot follow"),
    ],
)
def test_r3_observation_gate_rejects_incoherent_or_stale_input(change, message):
    with pytest.raises(ValueError, match=message):
        build_local_observation(
            _cloud(**change), _history(), np.eye(4), context=_context(), builder=LocalObstacleBuilder()
        )
