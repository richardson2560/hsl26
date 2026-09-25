# hsl_core/hsl_core/control/regulated_pursuit.py
"""Geometry-only regulated pursuit candidate generation."""

from dataclasses import dataclass
import math

from ..planning.astar import PlannedPath
from ..types import MotionCandidate, Pose2D


@dataclass(frozen=True)
class PursuitConfig:
    lookahead_m: float = 0.4
    speed_max_mps: float = 0.5
    yaw_rate_max_rps: float = 1.5
    lateral_accel_max_mps2: float = 0.8
    goal_tolerance_m: float = 0.1

    def __post_init__(self) -> None:
        if self.lookahead_m <= 0.0 or self.speed_max_mps <= 0.0 or self.yaw_rate_max_rps <= 0.0:
            raise ValueError("pursuit limits must be positive")
        if self.lateral_accel_max_mps2 <= 0.0 or self.goal_tolerance_m < 0.0:
            raise ValueError("pursuit acceleration/tolerance is invalid")


def _lookahead(path: PlannedPath, pose: Pose2D, distance: float) -> tuple[float, float]:
    points = path.polyline_xy_m
    nearest = min(
        range(len(points)),
        key=lambda index: math.hypot(points[index][0] - pose.x_m, points[index][1] - pose.y_m),
    )
    remaining = distance
    for start, end in zip(points[nearest:], points[nearest + 1:]):
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if length >= remaining:
            fraction = remaining / length if length else 0.0
            return (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            )
        remaining -= length
    return points[-1]


def make_candidate(
    path: PlannedPath,
    pose: Pose2D,
    *,
    config: PursuitConfig,
    now_s: float,
    lease_s: float,
    source_id: str,
) -> MotionCandidate:
    if lease_s <= 0.0 or not source_id or not math.isfinite(now_s):
        raise ValueError("time, lease and source_id must be valid")
    target = _lookahead(path, pose, config.lookahead_m)
    dx, dy = target[0] - pose.x_m, target[1] - pose.y_m
    cosine, sine = math.cos(pose.theta_rad), math.sin(pose.theta_rad)
    x_local = cosine * dx + sine * dy
    y_local = -sine * dx + cosine * dy
    squared = x_local * x_local + y_local * y_local
    if squared < 1e-12:
        linear, omega = 0.0, 0.0
    else:
        curvature = 2.0 * y_local / squared
        linear = min(
            config.speed_max_mps,
            math.sqrt(config.lateral_accel_max_mps2 / max(abs(curvature), 1e-12)),
        )
        omega = max(
            -config.yaw_rate_max_rps,
            min(config.yaw_rate_max_rps, linear * curvature),
        )
    return MotionCandidate(
        linear,
        omega,
        now_s,
        now_s + lease_s,
        lease_s,
        source_id,
        path.map_version,
        path.localization_epoch,
    )
