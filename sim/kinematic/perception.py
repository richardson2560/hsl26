"""Observation-only synthetic opponent extraction for the kinematic SIL."""

from __future__ import annotations

from dataclasses import dataclass
import math

from hsl_core.perception.ekf_opponent import OpponentDetection
from hsl_core.types import Pose2D

from .common import Segment, SensorObservation
from .raycaster import FirstHitRaycaster


@dataclass(frozen=True)
class ExtractorConfig:
    static_residual_threshold_m: float = 0.20
    minimum_cluster_beams: int = 2
    maximum_cluster_gap_beams: int = 1
    measurement_noise_std_m: float = 0.02

    def __post_init__(self) -> None:
        for value, name in (
            (self.static_residual_threshold_m, "static_residual_threshold_m"),
            (self.measurement_noise_std_m, "measurement_noise_std_m"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for value, name in (
            (self.minimum_cluster_beams, "minimum_cluster_beams"),
            (self.maximum_cluster_gap_beams, "maximum_cluster_gap_beams"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if self.minimum_cluster_beams < 2:
            raise ValueError("minimum_cluster_beams must be at least two")


def extract_synthetic_opponent(
    observation: SensorObservation,
    pose: Pose2D,
    static_segments: tuple[Segment, ...],
    *,
    stamp_s: float,
    source_id: str,
    map_version: int,
    localization_epoch: str,
    opponent_radius_m: float,
    config: ExtractorConfig = ExtractorConfig(),
) -> tuple[OpponentDetection, ...]:
    """Subtract known static first returns and cluster residual LiDAR points.

    The function consumes only the role's scan, estimated pose, and its
    versioned static map. It is not given simulator targets or plant truth.
    """
    if not isinstance(observation, SensorObservation):
        raise ValueError("observation must be a SensorObservation")
    if not isinstance(pose, Pose2D):
        raise ValueError("pose must be a Pose2D estimate")
    if not isinstance(static_segments, tuple) or any(
        not isinstance(segment, Segment) for segment in static_segments
    ):
        raise ValueError("static_segments must contain Segment values")
    if not isinstance(config, ExtractorConfig):
        raise ValueError("config must be an ExtractorConfig")
    if not math.isfinite(stamp_s) or abs(stamp_s - observation.stamp_s) > 1e-9:
        raise ValueError("extractor stamp must match the observation stamp")
    if not source_id or not localization_epoch:
        raise ValueError("source_id and localization_epoch must not be empty")
    if not isinstance(map_version, int) or isinstance(map_version, bool) or map_version < 0:
        raise ValueError("map_version must be a non-negative integer")
    if not math.isfinite(opponent_radius_m) or opponent_radius_m <= 0.0:
        raise ValueError("opponent_radius_m must be finite and positive")
    count = len(observation.ranges_m)
    if count < 4 or not all(observation.coverage_mask):
        return ()

    max_range = max(observation.ranges_m)
    if max_range <= 0.0:
        return ()
    raycaster = FirstHitRaycaster(max_range_m=max_range)
    residual_indices: list[int] = []
    local_angles = tuple(
        -math.pi + 2.0 * math.pi * index / count for index in range(count)
    )
    expected_ranges = raycaster.cast_many(
        (pose.x_m, pose.y_m),
        tuple(pose.theta_rad + angle for angle in local_angles),
        static_segments,
    )
    for index, (distance, valid, expected) in enumerate(
        zip(observation.ranges_m, observation.valid_mask, expected_ranges)
    ):
        if not valid:
            continue
        if expected is None or expected - distance >= config.static_residual_threshold_m:
            residual_indices.append(index)
    clusters = _cluster_indices(
        residual_indices,
        count,
        config.maximum_cluster_gap_beams,
    )
    detections = []
    angle_step = 2.0 * math.pi / count
    for cluster in clusters:
        if len(cluster) < config.minimum_cluster_beams:
            continue
        points = []
        for index in cluster:
            angle = pose.theta_rad + local_angles[index]
            center_range = observation.ranges_m[index] + opponent_radius_m
            points.append((
                pose.x_m + center_range * math.cos(angle),
                pose.y_m + center_range * math.sin(angle),
            ))
        mean_x = math.fsum(point[0] for point in points) / len(points)
        mean_y = math.fsum(point[1] for point in points) / len(points)
        range_variance = config.measurement_noise_std_m ** 2
        lateral_sigma = max(
            config.measurement_noise_std_m,
            opponent_radius_m * angle_step / math.sqrt(12.0),
        )
        variance = range_variance + lateral_sigma ** 2
        detections.append(
            OpponentDetection(
                position_xy_m=(mean_x, mean_y),
                covariance_xy_m2=(variance, 0.0, 0.0, variance),
                stamp_s=stamp_s,
                source_id=source_id,
                map_version=map_version,
                localization_epoch=localization_epoch,
            )
        )
    return tuple(sorted(
        detections,
        key=lambda detection: (
            math.hypot(
                detection.position_xy_m[0] - pose.x_m,
                detection.position_xy_m[1] - pose.y_m,
            ),
            detection.position_xy_m,
        ),
    ))


def line_of_sight_clear(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    static_segments: tuple[Segment, ...],
) -> bool:
    """Return false when the open line segment crosses a static wall."""
    if not all(math.isfinite(value) for value in (*start_xy, *end_xy)):
        raise ValueError("line-of-sight coordinates must be finite")
    dx, dy = end_xy[0] - start_xy[0], end_xy[1] - start_xy[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return True
    for segment in static_segments:
        sx = segment.end_xy[0] - segment.start_xy[0]
        sy = segment.end_xy[1] - segment.start_xy[1]
        denominator = dx * sy - dy * sx
        if abs(denominator) <= 1e-12:
            qx = segment.start_xy[0] - start_xy[0]
            qy = segment.start_xy[1] - start_xy[1]
            if abs(qx * dy - qy * dx) <= 1e-12:
                first = (qx * dx + qy * dy) / length_squared
                end_qx = segment.end_xy[0] - start_xy[0]
                end_qy = segment.end_xy[1] - start_xy[1]
                second = (end_qx * dx + end_qy * dy) / length_squared
                if max(min(first, second), 0.0) < min(max(first, second), 1.0):
                    return False
            continue
        qx = segment.start_xy[0] - start_xy[0]
        qy = segment.start_xy[1] - start_xy[1]
        along_ray = (qx * sy - qy * sx) / denominator
        along_wall = (qx * dy - qy * dx) / denominator
        if (
            1e-9 < along_ray < 1.0 - 1e-9
            and -1e-9 <= along_wall <= 1.0 + 1e-9
        ):
            return False
    return True


def _cluster_indices(
    indices: list[int], count: int, maximum_gap_beams: int
) -> tuple[tuple[int, ...], ...]:
    if not indices:
        return ()
    groups: list[list[int]] = [[indices[0]]]
    for index in indices[1:]:
        if index - groups[-1][-1] <= maximum_gap_beams + 1:
            groups[-1].append(index)
        else:
            groups.append([index])
    if len(groups) > 1 and (
        groups[0][0] + count - groups[-1][-1] <= maximum_gap_beams + 1
    ):
        groups[0] = groups[-1] + groups[0]
        groups.pop()
    return tuple(tuple(group) for group in groups)
