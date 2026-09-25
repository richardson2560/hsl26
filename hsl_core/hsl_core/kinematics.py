# hsl_core/hsl_core/kinematics.py
"""Kinematics and deterministic point-cloud deskewing without ROS or clocks."""

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

from .types import Pose2D


@dataclass(frozen=True)
class TimedPointCloud:
    """A finite point cloud with an acquisition time for every point.

    Points are expressed in the LiDAR frame at their individual acquisition
    times.  ``point_times_s`` uses the same clock epoch as pose samples.
    """

    points_xyz: np.ndarray
    point_times_s: np.ndarray

    def __post_init__(self) -> None:
        points = np.asarray(self.points_xyz, dtype=float)
        times = np.asarray(self.point_times_s, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_xyz must have shape (N, 3)")
        if times.ndim != 1 or times.shape[0] != points.shape[0]:
            raise ValueError("point_times_s must have one value per point")
        if not np.isfinite(points).all() or not np.isfinite(times).all():
            raise ValueError("point cloud coordinates and times must be finite")
        object.__setattr__(self, "points_xyz", points.copy())
        object.__setattr__(self, "point_times_s", times.copy())


@dataclass(frozen=True)
class PoseSample:
    """A timestamped homogeneous ``T_OB`` transform in one clock epoch."""

    stamp_s: float
    transform_ob: np.ndarray

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.stamp_s)):
            raise ValueError("pose stamp must be finite")
        transform = _validated_transform(self.transform_ob, "transform_ob")
        object.__setattr__(self, "transform_ob", transform.copy())


@dataclass(frozen=True)
class DeskewResult:
    """Deskewed points and a per-point unsupported-data mask.

    A point is unsupported only when it is outside the supplied pose-history
    interval.  The current implementation rejects such points by default
    rather than extrapolating; the mask remains part of the result contract
    for future adapters that may carry rejected samples diagnostically.
    """

    points_xyz: np.ndarray
    unsupported_mask: np.ndarray
    reference_time_s: float

    def __post_init__(self) -> None:
        points = np.asarray(self.points_xyz, dtype=float)
        mask = np.asarray(self.unsupported_mask, dtype=bool)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("deskewed points must have shape (N, 3)")
        if mask.ndim != 1 or mask.shape[0] != points.shape[0]:
            raise ValueError("unsupported_mask must have one value per point")
        if not np.isfinite(points).all() or not math.isfinite(float(self.reference_time_s)):
            raise ValueError("deskew result must be finite")
        object.__setattr__(self, "points_xyz", points.copy())
        object.__setattr__(self, "unsupported_mask", mask.copy())


def _validated_transform(value: np.ndarray, field_name: str) -> np.ndarray:
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError(f"{field_name} must have shape (4, 4)")
    if not np.isfinite(transform).all():
        raise ValueError(f"{field_name} must be finite")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-10, rtol=0.0):
        raise ValueError(f"{field_name} must have a homogeneous last row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0.0):
        raise ValueError(f"{field_name} rotation must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-8):
        raise ValueError(f"{field_name} rotation must have determinant +1")
    return transform


def _quaternion_from_rotation(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [(rotation[2, 1] - rotation[1, 2]) / scale,
             (rotation[0, 2] - rotation[2, 0]) / scale,
             (rotation[1, 0] - rotation[0, 1]) / scale,
             0.25 * scale],
            dtype=float,
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 0.0)) * 2.0
            quaternion = np.array(
                [0.25 * scale,
                 (rotation[0, 1] + rotation[1, 0]) / scale,
                 (rotation[0, 2] + rotation[2, 0]) / scale,
                 (rotation[2, 1] - rotation[1, 2]) / scale],
                dtype=float,
            )
        elif index == 1:
            scale = math.sqrt(max(1.0 - rotation[0, 0] + rotation[1, 1] - rotation[2, 2], 0.0)) * 2.0
            quaternion = np.array(
                [(rotation[0, 1] + rotation[1, 0]) / scale,
                 0.25 * scale,
                 (rotation[1, 2] + rotation[2, 1]) / scale,
                 (rotation[0, 2] - rotation[2, 0]) / scale],
                dtype=float,
            )
        else:
            scale = math.sqrt(max(1.0 - rotation[0, 0] - rotation[1, 1] + rotation[2, 2], 0.0)) * 2.0
            quaternion = np.array(
                [(rotation[0, 2] + rotation[2, 0]) / scale,
                 (rotation[1, 2] + rotation[2, 1]) / scale,
                 0.25 * scale,
                 (rotation[1, 0] - rotation[0, 1]) / scale],
                dtype=float,
            )
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("rotation cannot be converted to a quaternion")
    return quaternion / norm


def _rotation_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion
    return np.array(
        [[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
         [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
         [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)]],
        dtype=float,
    )


def _slerp(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    second = second.copy()
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    if dot > 1.0 - 1e-10:
        result = first + fraction * (second - first)
        return result / np.linalg.norm(result)
    angle = math.acos(max(-1.0, min(1.0, dot)))
    sine = math.sin(angle)
    return (
        math.sin((1.0 - fraction) * angle) / sine * first
        + math.sin(fraction * angle) / sine * second
    )


def _interpolate_pose(history: Sequence[PoseSample], stamp_s: float) -> np.ndarray:
    if stamp_s < history[0].stamp_s or stamp_s > history[-1].stamp_s:
        raise ValueError("pose history does not support every point time and reference time")
    for index in range(len(history) - 1):
        first = history[index]
        second = history[index + 1]
        if stamp_s <= second.stamp_s:
            if stamp_s == first.stamp_s:
                return first.transform_ob.copy()
            fraction = (stamp_s - first.stamp_s) / (second.stamp_s - first.stamp_s)
            first_q = _quaternion_from_rotation(first.transform_ob[:3, :3])
            second_q = _quaternion_from_rotation(second.transform_ob[:3, :3])
            transform = np.eye(4, dtype=float)
            transform[:3, :3] = _rotation_from_quaternion(_slerp(first_q, second_q, fraction))
            transform[:3, 3] = (
                first.transform_ob[:3, 3]
                + fraction * (second.transform_ob[:3, 3] - first.transform_ob[:3, 3])
            )
            return transform
    return history[-1].transform_ob.copy()


def deskew_cloud(
    cloud: TimedPointCloud,
    pose_history: Sequence[PoseSample],
    T_B_L: np.ndarray,
    t_ref: float,
) -> DeskewResult:
    """Deskew LiDAR points into the LiDAR frame at ``t_ref``.

    The transform follows TS §6.1:
    ``(T_OB(t_ref) T_BL)^-1 T_OB(t_i) T_BL p_i``.  Pose history must cover
    every point timestamp and the reference timestamp; arbitrary extrapolation
    is rejected.  The input is never mutated.
    """

    if not isinstance(cloud, TimedPointCloud):
        raise TypeError("cloud must be a TimedPointCloud with per-point times")
    if not math.isfinite(float(t_ref)):
        raise ValueError("t_ref must be finite")
    if not pose_history:
        raise ValueError("pose_history must not be empty")
    history = tuple(pose_history)
    if any(not isinstance(sample, PoseSample) for sample in history):
        raise TypeError("pose_history must contain PoseSample values")
    if any(history[index].stamp_s >= history[index + 1].stamp_s for index in range(len(history) - 1)):
        raise ValueError("pose history stamps must be strictly increasing")
    extrinsic = _validated_transform(T_B_L, "T_B_L")
    if cloud.point_times_s.size and (
        float(np.min(cloud.point_times_s)) < history[0].stamp_s
        or float(np.max(cloud.point_times_s)) > history[-1].stamp_s
    ):
        raise ValueError("pose history does not support every point time and reference time")
    reference_pose = _interpolate_pose(history, t_ref)
    reference_sensor_pose = reference_pose @ extrinsic
    inverse_reference = np.linalg.inv(reference_sensor_pose)
    corrected = np.empty_like(cloud.points_xyz)
    for index, stamp_s in enumerate(cloud.point_times_s):
        sensor_pose = _interpolate_pose(history, float(stamp_s)) @ extrinsic
        point_h = np.concatenate((cloud.points_xyz[index], (1.0,)))
        corrected[index] = (inverse_reference @ sensor_pose @ point_h)[:3]
    return DeskewResult(corrected, np.zeros(cloud.points_xyz.shape[0], dtype=bool), t_ref)


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
