"""Fail-closed admission of normalized LiDAR observations for R3.

This module is intentionally ROS-independent.  Driver-specific code must first
normalize a scan into :class:`NormalizedCloud`; only then may it reach the
deskew and local-obstacle path.  Missing point times, calibration identity,
frame identity, freshness or epoch coherence are rejected rather than treated
as observed-free space.
"""

from dataclasses import dataclass
import math

import numpy as np

from ..kinematics import PoseSample, TimedPointCloud, deskew_cloud
from ..mapping import LocalObstacleBuilder, LocalObstacleSnapshot


@dataclass(frozen=True)
class ObservationContext:
    """Receiver-local identities and age bound for one normalized scan."""

    now_ns: int
    max_age_ns: int
    expected_clock_epoch: str
    expected_localization_epoch: str
    expected_lidar_frame_id: str
    expected_calibration_id: str

    def __post_init__(self) -> None:
        if not isinstance(self.now_ns, int) or self.now_ns < 0:
            raise ValueError("now_ns must be a non-negative integer")
        if not isinstance(self.max_age_ns, int) or self.max_age_ns <= 0:
            raise ValueError("max_age_ns must be a positive integer")
        for value, name in (
            (self.expected_clock_epoch, "expected_clock_epoch"),
            (self.expected_localization_epoch, "expected_localization_epoch"),
            (self.expected_lidar_frame_id, "expected_lidar_frame_id"),
            (self.expected_calibration_id, "expected_calibration_id"),
        ):
            if not isinstance(value, str) or not value:
                raise ValueError(f"{name} must be non-empty")


@dataclass(frozen=True)
class NormalizedCloud:
    """Finite XYZ samples whose per-point stamps share one declared epoch."""

    points_xyz: np.ndarray
    point_stamps_ns: np.ndarray
    observation_stamp_ns: int
    frame_id: str
    clock_epoch: str
    localization_epoch: str
    calibration_id: str

    def __post_init__(self) -> None:
        points = np.asarray(self.points_xyz, dtype=float)
        stamps = np.asarray(self.point_stamps_ns)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError("points_xyz must be finite with shape (N, 3)")
        if stamps.ndim != 1 or stamps.shape[0] != points.shape[0]:
            raise ValueError("point_stamps_ns must have one value per point")
        if not np.issubdtype(stamps.dtype, np.integer) or np.any(stamps < 0):
            raise ValueError("point_stamps_ns must contain non-negative integers")
        if not isinstance(self.observation_stamp_ns, int) or self.observation_stamp_ns < 0:
            raise ValueError("observation_stamp_ns must be a non-negative integer")
        for value, name in (
            (self.frame_id, "frame_id"),
            (self.clock_epoch, "clock_epoch"),
            (self.localization_epoch, "localization_epoch"),
            (self.calibration_id, "calibration_id"),
        ):
            if not isinstance(value, str) or not value:
                raise ValueError(f"{name} must be non-empty")
        object.__setattr__(self, "points_xyz", points.copy())
        object.__setattr__(self, "point_stamps_ns", stamps.astype(np.int64, copy=True))


def build_local_observation(
    cloud: NormalizedCloud,
    pose_history: tuple[PoseSample, ...],
    base_to_lidar: np.ndarray,
    *,
    context: ObservationContext,
    builder: LocalObstacleBuilder | None = None,
) -> LocalObstacleSnapshot:
    """Deskew and build conservative local geometry after exact validation."""

    if not isinstance(cloud, NormalizedCloud):
        raise TypeError("cloud must be a NormalizedCloud")
    if cloud.frame_id != context.expected_lidar_frame_id:
        raise ValueError("lidar frame_id mismatch")
    if cloud.clock_epoch != context.expected_clock_epoch:
        raise ValueError("clock_epoch mismatch")
    if cloud.localization_epoch != context.expected_localization_epoch:
        raise ValueError("localization_epoch mismatch")
    if cloud.calibration_id != context.expected_calibration_id:
        raise ValueError("calibration_id mismatch")
    if cloud.observation_stamp_ns > context.now_ns:
        raise ValueError("observation timestamp is in the future")
    if context.now_ns - cloud.observation_stamp_ns > context.max_age_ns:
        raise ValueError("observation is stale")
    if cloud.point_stamps_ns.size and np.any(
        cloud.point_stamps_ns > cloud.observation_stamp_ns
    ):
        raise ValueError("point timestamp cannot follow observation timestamp")

    timed = TimedPointCloud(
        cloud.points_xyz, cloud.point_stamps_ns.astype(float) / 1_000_000_000.0
    )
    reference_s = cloud.observation_stamp_ns / 1_000_000_000.0
    result = deskew_cloud(timed, pose_history, base_to_lidar, reference_s)
    if np.any(result.unsupported_mask):
        raise ValueError("unsupported point times cannot establish coverage")
    return (builder or LocalObstacleBuilder()).build(result.points_xyz)
