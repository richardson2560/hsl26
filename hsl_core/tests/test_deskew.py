# hsl_core/tests/test_deskew.py
"""Adversarial SIL tests for the Phase-2 point-cloud deskew contract."""

import math

import numpy as np
import pytest

from hsl_core.kinematics import DeskewResult, PoseSample, TimedPointCloud, deskew_cloud


def _yaw_transform(yaw: float, x: float = 0.0, y: float = 0.0) -> np.ndarray:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[cosine, -sine, 0.0, x], [sine, cosine, 0.0, y], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        dtype=float,
    )


def _history() -> tuple[PoseSample, ...]:
    return (
        PoseSample(0.0, _yaw_transform(0.0)),
        PoseSample(0.1, _yaw_transform(0.05, 0.1, 0.0)),
        PoseSample(0.2, _yaw_transform(0.10, 0.2, 0.0)),
    )


def test_static_wall_with_translation_rotation_and_3d_extrinsic_is_restored():
    history = _history()
    extrinsic = _yaw_transform(math.pi / 2.0, 0.2, 0.1)
    points_reference = np.array([[2.0, y, z] for y, z in ((-0.4, 0.0), (0.0, 0.2), (0.4, 0.5))])
    points = []
    times = np.array([0.0, 0.1, 0.2])
    for point, stamp in zip(points_reference, times):
        sensor_pose = next(sample.transform_ob for sample in history if sample.stamp_s == stamp) @ extrinsic
        points.append((np.linalg.inv(sensor_pose) @ np.r_[point, 1.0])[:3])
    result = deskew_cloud(TimedPointCloud(np.array(points), times), history, extrinsic, 0.1)
    reference_sensor_pose = history[1].transform_ob @ extrinsic
    expected = np.array([
        (np.linalg.inv(reference_sensor_pose) @ np.r_[point, 1.0])[:3]
        for point in points_reference
    ])
    assert np.max(np.abs(result.points_xyz - expected)) < 1e-10
    assert not result.unsupported_mask.any()


def test_interpolation_uses_se3_translation_and_slerp_not_nearest_pose():
    history = (PoseSample(0.0, _yaw_transform(0.0)), PoseSample(1.0, _yaw_transform(math.pi / 2.0, 1.0, 2.0)))
    world_point = np.array([2.0, 0.0, 0.0])
    cloud = TimedPointCloud(world_point.reshape(1, 3), np.array([0.0]))
    result = deskew_cloud(cloud, history, np.eye(4), 0.5)
    expected = (np.linalg.inv(_yaw_transform(math.pi / 4.0, 0.5, 1.0)) @ np.r_[world_point, 1.0])[:3].reshape(1, 3)
    assert result.points_xyz == pytest.approx(expected)


def test_identity_motion_is_exact_and_input_is_not_mutated():
    points = np.array([[1.0, 2.0, 3.0], [-1.0, 0.5, 0.0]])
    original = points.copy()
    cloud = TimedPointCloud(points, np.array([0.0, 1.0]))
    history = (PoseSample(0.0, np.eye(4)), PoseSample(1.0, np.eye(4)))
    result = deskew_cloud(cloud, history, np.eye(4), 0.5)
    assert result.points_xyz == pytest.approx(points)
    assert np.array_equal(points, original)


@pytest.mark.parametrize(
    "cloud",
    [
        object(),
        TimedPointCloud(np.empty((0, 3)), np.empty((0,))),
    ],
)
def test_cloud_type_and_empty_cloud_contract(cloud):
    history = (PoseSample(0.0, np.eye(4)), PoseSample(1.0, np.eye(4)))
    if isinstance(cloud, TimedPointCloud):
        result = deskew_cloud(cloud, history, np.eye(4), 0.5)
        assert result.points_xyz.shape == (0, 3)
    else:
        with pytest.raises(TypeError, match="TimedPointCloud"):
            deskew_cloud(cloud, history, np.eye(4), 0.5)


@pytest.mark.parametrize(
    "factory, message",
    [
        (lambda: TimedPointCloud(np.ones((2, 3)), np.array([0.0])), "one value"),
        (lambda: TimedPointCloud(np.ones((2, 2)), np.array([0.0, 1.0])), "shape"),
        (lambda: TimedPointCloud(np.array([[math.nan, 0.0, 0.0]]), np.array([0.0])), "finite"),
    ],
)
def test_cloud_validation_rejects_malformed_inputs(factory, message):
    with pytest.raises(ValueError, match=message):
        factory()


def test_missing_per_point_time_is_rejected_instead_of_assuming_scan_time():
    history = (PoseSample(0.0, np.eye(4)), PoseSample(1.0, np.eye(4)))
    with pytest.raises(TypeError, match="per-point times"):
        deskew_cloud(np.ones((1, 3)), history, np.eye(4), 0.5)


@pytest.mark.parametrize(
    "times, reference, message",
    [
        (np.array([-0.1]), 0.5, "support"),
        (np.array([0.5]), 1.1, "support"),
        (np.array([math.nan]), 0.5, "finite"),
    ],
)
def test_time_support_and_reference_are_strict(times, reference, message):
    history = (PoseSample(0.0, np.eye(4)), PoseSample(1.0, np.eye(4)))
    if np.isnan(times).any():
        with pytest.raises(ValueError, match="finite"):
            TimedPointCloud(np.ones((1, 3)), times)
    else:
        with pytest.raises(ValueError, match=message):
            deskew_cloud(TimedPointCloud(np.ones((1, 3)), times), history, np.eye(4), reference)


def test_history_rejects_empty_unsorted_duplicate_and_non_pose_entries():
    cloud = TimedPointCloud(np.ones((1, 3)), np.array([0.5]))
    with pytest.raises(ValueError, match="must not be empty"):
        deskew_cloud(cloud, (), np.eye(4), 0.5)
    with pytest.raises(ValueError, match="strictly increasing"):
        deskew_cloud(cloud, (PoseSample(0.0, np.eye(4)), PoseSample(0.0, np.eye(4))), np.eye(4), 0.5)
    with pytest.raises(TypeError, match="PoseSample"):
        deskew_cloud(cloud, (object(),), np.eye(4), 0.5)


@pytest.mark.parametrize(
    "transform, message",
    [
        (np.eye(3), "shape"),
        (np.full((4, 4), np.nan), "finite"),
        (np.diag([2.0, 1.0, 1.0, 1.0]), "orthonormal"),
        (np.diag([1.0, 1.0, -1.0, 1.0]), "determinant"),
        (np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 2.0]]), "last row"),
    ],
)
def test_extrinsic_validation_rejects_non_rigid_transform(transform, message):
    history = (PoseSample(0.0, np.eye(4)), PoseSample(1.0, np.eye(4)))
    cloud = TimedPointCloud(np.ones((1, 3)), np.array([0.5]))
    with pytest.raises(ValueError, match=message):
        deskew_cloud(cloud, history, transform, 0.5)


def test_result_is_structurally_validated_and_not_aliasing_inputs():
    result = DeskewResult(np.zeros((1, 3)), np.array([False]), 0.5)
    with pytest.raises(ValueError, match="one value"):
        DeskewResult(np.zeros((2, 3)), np.array([False]), 0.5)
    result.points_xyz[0, 0] = 99.0
    assert result.points_xyz[0, 0] == 99.0
