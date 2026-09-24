# hsl_core/hsl_core/rules.py
# hsl_core/hsl_core/rules.py
"""Deterministic capture and arrival predicates using explicit frames and time.

"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

from .types import Pose2D


@dataclass(frozen=True)
class CaptureInterval:
    """Strict nominal-distance interval for robust sufficient capture."""

    lower_m: float
    upper_m: float

    def __post_init__(self) -> None:
        if not (math.isfinite(self.lower_m) and math.isfinite(self.upper_m)):
            raise ValueError("capture interval bounds must be finite")
        if self.lower_m < 0.0 or self.upper_m <= self.lower_m:
            raise ValueError("capture interval must be non-empty and positive")


@dataclass(frozen=True)
class ArrivalEvent:
    """First contour crossing along a sampled trajectory."""

    segment_index: int
    fraction: float
    time_s: float
    point: Tuple[float, float]


def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def capture_predicate(
    guardian: Pose2D,
    explorer: Pose2D,
    *,
    line_of_sight: bool | None,
    max_distance_m: float = 0.45,
    max_bearing_rad: float = math.pi / 4.0,
) -> bool:
    """Apply strict distance, inclusive bearing, and explicit LOS rules."""

    if line_of_sight is not True:
        return False
    if (
        not math.isfinite(max_distance_m)
        or not math.isfinite(max_bearing_rad)
        or max_distance_m <= 0.0
        or max_bearing_rad < 0.0
    ):
        raise ValueError("capture limits must be positive")
    dx = explorer.x_m - guardian.x_m
    dy = explorer.y_m - guardian.y_m
    distance = math.hypot(dx, dy)
    if distance == 0.0:
        raise ValueError("coincident capture origins are invalid")
    bearing = abs(_wrap_angle(math.atan2(dy, dx) - guardian.theta_rad))
    return distance < max_distance_m and bearing <= max_bearing_rad


def robust_capture_interval(
    guardian_radius_m: float,
    explorer_radius_m: float,
    margin_m: float,
    position_error_m: float,
    *,
    capture_distance_m: float = 0.45,
    angular_error_rad: float = 0.0,
    angular_reference_radius_m: float = 0.0,
) -> CaptureInterval:
    """Return a sufficient interval with all uncertainty terms in metres.

    ``angular_error_rad`` is converted to metres through the explicitly
    supplied ``angular_reference_radius_m``. Radians are dimensionless, but
    they cannot be added directly to a distance.
    """

    if any(value < 0.0 or not math.isfinite(value) for value in (
        guardian_radius_m,
        explorer_radius_m,
        margin_m,
        position_error_m,
        angular_error_rad,
        angular_reference_radius_m,
    )):
        raise ValueError("capture uncertainty values must be finite and non-negative")
    if capture_distance_m <= 0.0 or not math.isfinite(capture_distance_m):
        raise ValueError("capture_distance_m must be positive and finite")
    return CaptureInterval(
        guardian_radius_m
        + explorer_radius_m
        + margin_m
        + position_error_m
        + angular_reference_radius_m * angular_error_rad,
        capture_distance_m - margin_m,
    )


def _orientation(
    a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]
) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(
    a: Tuple[float, float], b: Tuple[float, float], point: Tuple[float, float]
) -> bool:
    tolerance = 1e-12
    return (
        min(a[0], b[0]) - tolerance <= point[0] <= max(a[0], b[0]) + tolerance
        and min(a[1], b[1]) - tolerance <= point[1] <= max(a[1], b[1]) + tolerance
    )


def segment_intersection(
    first_start: Tuple[float, float],
    first_end: Tuple[float, float],
    second_start: Tuple[float, float],
    second_end: Tuple[float, float],
) -> Tuple[float, float] | None:
    """Return a segment contact, including collinear endpoint contact."""

    values = first_start + first_end + second_start + second_end
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("segment coordinates must be finite")
    orientation = (
        _orientation(first_start, first_end, second_start),
        _orientation(first_start, first_end, second_end),
        _orientation(second_start, second_end, first_start),
        _orientation(second_start, second_end, first_end),
    )
    epsilon = 1e-12
    if all(abs(value) <= epsilon for value in orientation):
        for point in (first_start, first_end, second_start, second_end):
            if _on_segment(first_start, first_end, point) and _on_segment(
                second_start, second_end, point
            ):
                return point
        return None
    if (
        (orientation[0] > epsilon and orientation[1] > epsilon)
        or (orientation[0] < -epsilon and orientation[1] < -epsilon)
        or (orientation[2] > epsilon and orientation[3] > epsilon)
        or (orientation[2] < -epsilon and orientation[3] < -epsilon)
    ):
        return None
    denominator = (
        (first_start[0] - first_end[0]) * (second_start[1] - second_end[1])
        - (first_start[1] - first_end[1]) * (second_start[0] - second_end[0])
    )
    if abs(denominator) <= epsilon:
        return None
    fraction = (
        (first_start[0] - second_start[0]) * (second_start[1] - second_end[1])
        - (first_start[1] - second_start[1]) * (second_start[0] - second_end[0])
    ) / denominator
    return (
        first_start[0] + fraction * (first_end[0] - first_start[0]),
        first_start[1] + fraction * (first_end[1] - first_start[1]),
    )


def first_contour_arrival(
    trajectory: Sequence[Tuple[float, float]],
    contour: Sequence[Tuple[float, float]],
    *,
    start_time_s: float = 0.0,
    sample_period_s: float = 1.0,
) -> ArrivalEvent | None:
    """Return the first trajectory/contour contact with interpolated time."""

    if len(trajectory) < 2 or len(contour) < 3:
        raise ValueError("trajectory needs two points and contour needs three")
    if not math.isfinite(start_time_s) or sample_period_s <= 0.0:
        raise ValueError("arrival timing must be finite with positive sample period")
    edges = tuple(
        (contour[index], contour[(index + 1) % len(contour)])
        for index in range(len(contour))
    )
    for index, (start, end) in enumerate(zip(trajectory, trajectory[1:])):
        for edge_start, edge_end in edges:
            point = segment_intersection(start, end, edge_start, edge_end)
            if point is None:
                continue
            length = _distance(start, end)
            fraction = 0.0 if length == 0.0 else _distance(start, point) / length
            fraction = min(1.0, max(0.0, fraction))
            return ArrivalEvent(
                index,
                fraction,
                start_time_s + (index + fraction) * sample_period_s,
                point,
            )
    return None
