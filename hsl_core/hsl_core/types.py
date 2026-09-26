# hsl_core/hsl_core/types.py
"""Immutable, ROS-independent domain contracts for the HSL26 safety ring."""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Tuple

from .contracts import validate_covariance


def _finite(value: float, field_name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{field_name} must be finite")
    return value


def _non_empty(value: str, field_name: str) -> str:
    if not value:
        raise ValueError(f"{field_name} must not be empty")
    return value


def _fixed_vector(values: Tuple[float, ...], length: int, field_name: str) -> Tuple[float, ...]:
    if len(values) != length:
        raise ValueError(f"{field_name} must contain exactly {length} values")
    return tuple(_finite(float(value), field_name) for value in values)


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in metres and radians."""

    x_m: float
    y_m: float
    theta_rad: float

    def __post_init__(self) -> None:
        _finite(self.x_m, "x_m")
        _finite(self.y_m, "y_m")
        _finite(self.theta_rad, "theta_rad")


@dataclass(frozen=True)
class Twist2D:
    """Planar body velocity in metres per second and radians per second."""

    linear_mps: float
    angular_rps: float

    def __post_init__(self) -> None:
        _finite(self.linear_mps, "linear_mps")
        _finite(self.angular_rps, "angular_rps")


@dataclass(frozen=True)
class EgoState:
    """Validated robot state snapshot used by safety decisions."""

    pose: Pose2D
    twist: Twist2D
    pose_covariance: Tuple[float, ...]
    localization_epoch: str
    healthy: bool
    slip: bool
    observation_time_s: float
    frame_id: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "pose_covariance",
            validate_covariance(self.pose_covariance, 3),
        )
        _finite(self.observation_time_s, "observation_time_s")
        _non_empty(self.localization_epoch, "localization_epoch")
        _non_empty(self.frame_id, "frame_id")


class TrackState(IntEnum):
    """Canonical lifecycle states for an opponent track."""

    SEARCHING = 0
    TRACKED = 1
    COASTING = 2
    OCCLUDED_BELIEF = 3
    LOST = 4


@dataclass(frozen=True)
class OpponentTrack:
    """Opponent estimate with explicit temporal and spatial validity."""

    track_id: str
    pose: Pose2D
    velocity_x_mps: float
    velocity_y_mps: float
    covariance: Tuple[float, ...]
    yaw_valid: bool
    last_measurement_s: float
    valid_until_s: float
    localization_epoch: str
    map_version: int
    state: TrackState
    source_id: str
    frame_id: str

    def __post_init__(self) -> None:
        _non_empty(self.track_id, "track_id")
        _non_empty(self.localization_epoch, "localization_epoch")
        _non_empty(self.source_id, "source_id")
        _non_empty(self.frame_id, "frame_id")
        object.__setattr__(
            self,
            "covariance",
            validate_covariance(self.covariance, 4),
        )
        _finite(self.velocity_x_mps, "velocity_x_mps")
        _finite(self.velocity_y_mps, "velocity_y_mps")
        _finite(self.last_measurement_s, "last_measurement_s")
        _finite(self.valid_until_s, "valid_until_s")
        if self.valid_until_s < self.last_measurement_s:
            raise ValueError("valid_until_s must not precede last_measurement_s")
        if self.map_version < 0:
            raise ValueError("map_version must be non-negative")
        object.__setattr__(self, "state", TrackState(self.state))


@dataclass(frozen=True)
class MotionCandidate:
    """Untrusted velocity proposal; never a physical motor command."""

    linear_velocity_mps: float
    angular_velocity_rps: float
    observation_time_s: float
    valid_until_s: float
    horizon_s: float
    source_id: str
    map_version: int
    localization_epoch: str
    topology_version: int
    lease_generation: int

    def __post_init__(self) -> None:
        _finite(self.linear_velocity_mps, "linear_velocity_mps")
        _finite(self.angular_velocity_rps, "angular_velocity_rps")
        _finite(self.observation_time_s, "observation_time_s")
        _finite(self.valid_until_s, "valid_until_s")
        _finite(self.horizon_s, "horizon_s")
        if self.valid_until_s < self.observation_time_s:
            raise ValueError("valid_until_s must not precede observation_time_s")
        if self.horizon_s < 0.0:
            raise ValueError("horizon_s must be non-negative")
        for value, field_name in (
            (self.map_version, "map_version"),
            (self.topology_version, "topology_version"),
            (self.lease_generation, "lease_generation"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{field_name} must be a non-negative integer")
        if self.lease_generation == 0:
            raise ValueError("lease_generation must be positive")
        _non_empty(self.source_id, "source_id")
        _non_empty(self.localization_epoch, "localization_epoch")


@dataclass(frozen=True)
class SafetyStatus:
    """Auditable result of evaluating one motion candidate."""

    vetoed: bool
    reason: str
    free_distance_m: float
    measured_latency_s: float
    admissible_velocity_mps: float
    healthy: bool
    source_id: str
    publication_time_s: float

    def __post_init__(self) -> None:
        _finite(self.free_distance_m, "free_distance_m")
        _finite(self.measured_latency_s, "measured_latency_s")
        _finite(self.admissible_velocity_mps, "admissible_velocity_mps")
        _finite(self.publication_time_s, "publication_time_s")
        _non_empty(self.reason, "reason")
        _non_empty(self.source_id, "source_id")
