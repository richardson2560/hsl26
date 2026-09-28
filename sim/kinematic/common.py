# sim/kinematic/common.py
"""Small, deterministic contracts shared by the Phase-3 kinematic testbed."""

from dataclasses import dataclass
import math
from typing import Tuple

from hsl_core.types import Pose2D


def _finite(value: float, field: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{field} must be finite")
    return value


@dataclass(frozen=True)
class Segment:
    start_xy: Tuple[float, float]
    end_xy: Tuple[float, float]

    def __post_init__(self) -> None:
        values = (*self.start_xy, *self.end_xy)
        if len(values) != 4 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError("segment coordinates must be finite")
        if self.start_xy == self.end_xy:
            raise ValueError("segment must have nonzero length")


@dataclass(frozen=True)
class CircleTarget:
    center_xy: Tuple[float, float]
    radius_m: float
    target_id: str

    def __post_init__(self) -> None:
        if not self.target_id:
            raise ValueError("target_id must not be empty")
        if not all(math.isfinite(float(v)) for v in self.center_xy):
            raise ValueError("circle center must be finite")
        if not math.isfinite(float(self.radius_m)) or self.radius_m <= 0.0:
            raise ValueError("circle radius must be positive and finite")


@dataclass(frozen=True)
class WorldGeometry:
    static_segments: Tuple[Segment, ...]
    dynamic_targets: Tuple[CircleTarget, ...] = ()


@dataclass(frozen=True)
class Actuation:
    linear_mps: float
    angular_rps: float

    def __post_init__(self) -> None:
        _finite(self.linear_mps, "linear_mps")
        _finite(self.angular_rps, "angular_rps")


@dataclass(frozen=True)
class PlantState:
    stamp_s: float
    pose: Pose2D


@dataclass(frozen=True)
class SensorObservation:
    stamp_s: float
    frame_id: str
    ranges_m: Tuple[float, ...]
    valid_mask: Tuple[bool, ...]
    coverage_mask: Tuple[bool, ...] | None = None

    def __post_init__(self) -> None:
        _finite(self.stamp_s, "stamp_s")
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if len(self.ranges_m) != len(self.valid_mask):
            raise ValueError("ranges and valid mask must have equal length")
        coverage = (
            tuple(self.valid_mask)
            if self.coverage_mask is None
            else tuple(self.coverage_mask)
        )
        if len(coverage) != len(self.ranges_m):
            raise ValueError("ranges and coverage mask must have equal length")
        for distance, valid in zip(self.ranges_m, self.valid_mask):
            _finite(distance, "ranges_m")
            if distance < 0.0:
                raise ValueError("ranges_m must be non-negative")
            if not isinstance(valid, bool):
                raise ValueError("valid_mask must contain bool values")
        if any(not isinstance(value, bool) for value in coverage):
            raise ValueError("coverage_mask must contain bool values")
        if any(valid and not covered for valid, covered in zip(self.valid_mask, coverage)):
            raise ValueError("a valid return must also have valid sensor coverage")
        object.__setattr__(self, "coverage_mask", tuple(coverage))


@dataclass(frozen=True)
class PoseEstimate:
    """Role-scoped estimate; never aliases a simulator plant state."""

    stamp_ns: int
    frame_id: str
    clock_epoch: str
    localization_epoch: str
    pose: Pose2D
    position_error_bound_m: float
    yaw_error_bound_rad: float
    linear_velocity_mps: float = 0.0
    angular_velocity_rps: float = 0.0

    def __post_init__(self) -> None:
        if isinstance(self.stamp_ns, bool) or not isinstance(self.stamp_ns, int) or self.stamp_ns < 0:
            raise ValueError("pose estimate stamp must be a non-negative integer")
        if not self.frame_id or not self.clock_epoch or not self.localization_epoch:
            raise ValueError("pose estimate frame and epochs must not be empty")
        if not isinstance(self.pose, Pose2D):
            raise ValueError("pose estimate requires a Pose2D")
        for value, name in (
            (self.position_error_bound_m, "position_error_bound_m"),
            (self.yaw_error_bound_rad, "yaw_error_bound_rad"),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        for value, name in (
            (self.linear_velocity_mps, "linear_velocity_mps"),
            (self.angular_velocity_rps, "angular_velocity_rps"),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")


@dataclass(frozen=True)
class TraceEvent:
    stamp_s: float
    kind: str
    payload: Tuple[float, ...]
