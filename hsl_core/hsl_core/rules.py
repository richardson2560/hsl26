# hsl_core/hsl_core/rules.py
"""Deterministic capture and arrival predicates using explicit frames and time.

"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

from .contracts import validate_polygon
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


@dataclass(frozen=True)
class TimedPose:
    """Pose sample on a strictly increasing, common referee clock."""

    stamp_ns: int
    pose: Pose2D

    def __post_init__(self) -> None:
        if not isinstance(self.stamp_ns, int) or isinstance(self.stamp_ns, bool) or self.stamp_ns < 0:
            raise ValueError("stamp_ns must be a non-negative integer")
        if not isinstance(self.pose, Pose2D):
            raise ValueError("pose must be a Pose2D")


@dataclass(frozen=True)
class CaptureEventInterval:
    """Conservative bracket containing the earliest certified capture time."""

    segment_index: int
    lower_ns: int
    upper_ns: int

    def __post_init__(self) -> None:
        if (
            not isinstance(self.segment_index, int)
            or isinstance(self.segment_index, bool)
            or self.segment_index < 0
            or not isinstance(self.lower_ns, int)
            or isinstance(self.lower_ns, bool)
            or self.lower_ns < 0
            or not isinstance(self.upper_ns, int)
            or isinstance(self.upper_ns, bool)
            or self.upper_ns < self.lower_ns
        ):
            raise ValueError("capture interval indices and bounds are invalid")


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


def _point_segment_distance(
    point: Tuple[float, float],
    start: Tuple[float, float],
    end: Tuple[float, float],
) -> Tuple[float, Tuple[float, float]]:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    fraction = (
        0.0
        if length_sq == 0.0
        else max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq))
    )
    closest = (start[0] + fraction * dx, start[1] + fraction * dy)
    return _distance(point, closest), closest


def _point_in_polygon(point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]) -> bool:
    inside = False
    x, y = point
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        if (first[1] > y) != (second[1] > y):
            crossing_x = first[0] + (y - first[1]) * (second[0] - first[0]) / (second[1] - first[1])
            if x < crossing_x:
                inside = not inside
    return inside


def _distance_to_polygon(
    point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]
) -> Tuple[float, Tuple[float, float]]:
    closest = None
    minimum = math.inf
    for index, start in enumerate(polygon):
        distance, candidate = _point_segment_distance(point, start, polygon[(index + 1) % len(polygon)])
        if distance < minimum:
            minimum, closest = distance, candidate
    if _point_in_polygon(point, polygon):
        return 0.0, point
    return minimum, closest if closest is not None else point


def first_footprint_arrival(
    trajectory: Sequence[Tuple[float, float]],
    contour: Sequence[Tuple[float, float]],
    footprint_radius_m: float,
    *,
    start_time_s: float = 0.0,
    sample_period_s: float = 1.0,
) -> ArrivalEvent | None:
    """Find first continuous contact of a circular footprint along linear segments.

    The trajectory is piecewise linear between consecutive samples. Initial
    overlap is rejected because it cannot establish a first arrival.
    """
    if len(trajectory) < 2 or len(contour) < 3:
        raise ValueError("trajectory needs two points and contour needs three")
    if (
        isinstance(footprint_radius_m, bool)
        or not math.isfinite(float(footprint_radius_m))
        or footprint_radius_m < 0.0
        or isinstance(start_time_s, bool)
        or not math.isfinite(float(start_time_s))
        or isinstance(sample_period_s, bool)
        or not math.isfinite(float(sample_period_s))
        or sample_period_s <= 0.0
    ):
        raise ValueError("footprint radius and arrival timing are invalid")
    if any(not isinstance(point, (tuple, list)) or len(point) != 2 for point in (*contour, *trajectory)):
        raise ValueError("trajectory and contour coordinates must be 2D points")
    polygon = tuple((float(point[0]), float(point[1])) for point in contour)
    path = tuple((float(point[0]), float(point[1])) for point in trajectory)
    if any(not all(math.isfinite(value) for value in point) for point in (*polygon, *path)):
        raise ValueError("trajectory and contour coordinates must be finite 2D points")
    polygon = validate_polygon(polygon)
    if _distance_to_polygon(path[0], polygon)[0] <= footprint_radius_m:
        raise ValueError("initial footprint intersects goal boundary")

    radius = float(footprint_radius_m)
    for segment_index, (start, end) in enumerate(zip(path, path[1:])):
        vx, vy = end[0] - start[0], end[1] - start[1]
        candidates: list[Tuple[float, Tuple[float, float]]] = []
        if vx == 0.0 and vy == 0.0:
            continue
        for edge_index, edge_start in enumerate(polygon):
            edge_end = polygon[(edge_index + 1) % len(polygon)]
            ex, ey = edge_end[0] - edge_start[0], edge_end[1] - edge_start[1]
            edge_length = math.hypot(ex, ey)

            # The two parallel offset lines cover contact with the edge interior.
            if radius == 0.0:
                contact = segment_intersection(start, end, edge_start, edge_end)
                if contact is not None:
                    fraction = min(
                        1.0,
                        max(0.0, _distance(start, contact) / math.hypot(vx, vy)),
                    )
                    candidates.append((fraction, contact))
            else:
                initial_cross = ex * (start[1] - edge_start[1]) - ey * (start[0] - edge_start[0])
                cross_delta = ex * vy - ey * vx
                if cross_delta != 0.0:
                    for signed_offset in (-radius * edge_length, radius * edge_length):
                        fraction = (signed_offset - initial_cross) / cross_delta
                        if 0.0 <= fraction <= 1.0:
                            point = (start[0] + fraction * vx, start[1] + fraction * vy)
                            projection = ((point[0] - edge_start[0]) * ex + (point[1] - edge_start[1]) * ey) / (edge_length * edge_length)
                            if 0.0 <= projection <= 1.0:
                                candidates.append((fraction, (edge_start[0] + projection * ex, edge_start[1] + projection * ey)))

                # Vertex-circle roots cover rounded corners of the Minkowski sum.
                for vertex in (edge_start,):
                    ox, oy = start[0] - vertex[0], start[1] - vertex[1]
                    a = vx * vx + vy * vy
                    b = 2.0 * (ox * vx + oy * vy)
                    c = ox * ox + oy * oy - radius * radius
                    discriminant = b * b - 4.0 * a * c
                    if discriminant >= 0.0:
                        root = math.sqrt(discriminant)
                        for fraction in ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a)):
                            if 0.0 <= fraction <= 1.0:
                                point = (start[0] + fraction * vx, start[1] + fraction * vy)
                                candidates.append((fraction, vertex))

        for fraction, boundary_point in sorted(candidates, key=lambda item: item[0]):
            center = (start[0] + fraction * vx, start[1] + fraction * vy)
            distance, closest_boundary = _distance_to_polygon(center, polygon)
            if distance <= radius:
                return ArrivalEvent(
                    segment_index,
                    fraction,
                    start_time_s + (segment_index + fraction) * sample_period_s,
                    closest_boundary if radius == 0.0 else boundary_point,
                )
    return None


def _segment_blocks_los(
    guardian: Tuple[float, float],
    explorer: Tuple[float, float],
    obstacles: Sequence[Tuple[Tuple[float, float], Tuple[float, float]]],
    circle_obstacles: Sequence[Tuple[Tuple[float, float], float]],
) -> bool:
    for start, end in obstacles:
        if segment_intersection(guardian, explorer, start, end) is not None:
            return True
    for center, radius in circle_obstacles:
        if _point_segment_distance(center, guardian, explorer)[0] <= radius:
            return True
    return False


def first_capture_interval(
    guardian: Sequence[TimedPose],
    explorer: Sequence[TimedPose],
    obstacles: Sequence[Tuple[Tuple[float, float], Tuple[float, float]]],
    *,
    max_distance_m: float = 0.45,
    max_bearing_rad: float = math.pi / 4.0,
    max_depth: int = 10,
    circle_obstacles: Sequence[Tuple[Tuple[float, float], float]] = (),
) -> CaptureEventInterval | None:
    """Find a conservative time bracket for continuous capture under linear interpolation.

    A positive interval is accepted only when distance and bearing are bounded
    over the whole interval and the swept convex LOS envelope is obstacle-free.
    Ambiguous subintervals are subdivided; unresolved intervals never become a
    capture merely because endpoint distances crossed the threshold.
    """
    if len(guardian) < 2 or len(guardian) != len(explorer):
        raise ValueError("capture trajectories must have at least two paired samples")
    if (
        isinstance(max_distance_m, bool)
        or not isinstance(max_distance_m, (int, float))
        or not math.isfinite(max_distance_m)
        or max_distance_m <= 0.0
        or isinstance(max_bearing_rad, bool)
        or not isinstance(max_bearing_rad, (int, float))
        or not math.isfinite(max_bearing_rad)
        or not 0.0 <= max_bearing_rad <= math.pi
        or not isinstance(max_depth, int)
        or isinstance(max_depth, bool)
        or not 1 <= max_depth <= 16
    ):
        raise ValueError("capture bounds or subdivision depth are invalid")
    if any(not isinstance(sample, TimedPose) for sample in (*guardian, *explorer)):
        raise ValueError("capture trajectories must contain TimedPose samples")
    stamps = tuple(sample.stamp_ns for sample in guardian)
    if any(g.stamp_ns != e.stamp_ns for g, e in zip(guardian, explorer)) or any(
        second <= first for first, second in zip(stamps, stamps[1:])
    ):
        raise ValueError("capture samples must have matching strictly increasing timestamps")
    normalized_obstacles = []
    for obstacle in obstacles:
        if not isinstance(obstacle, (tuple, list)) or len(obstacle) != 2:
            raise ValueError("LOS obstacles must be finite non-degenerate segments")
        start, end = obstacle
        if (
            not isinstance(start, (tuple, list))
            or not isinstance(end, (tuple, list))
            or len(start) != 2
            or len(end) != 2
            or any(
                isinstance(value, bool) or not isinstance(value, (int, float))
                for value in (*start, *end)
            )
        ):
            raise ValueError("LOS obstacles must be finite non-degenerate segments")
        segment = (
            tuple(float(value) for value in start),
            tuple(float(value) for value in end),
        )
        if (
            not all(math.isfinite(value) for point in segment for value in point)
            or segment[0] == segment[1]
        ):
            raise ValueError("LOS obstacles must be finite non-degenerate segments")
        normalized_obstacles.append(segment)
    obstacles = tuple(normalized_obstacles)

    normalized_circles = []
    for obstacle in circle_obstacles:
        if (
            not isinstance(obstacle, (tuple, list))
            or len(obstacle) != 2
            or not isinstance(obstacle[0], (tuple, list))
            or len(obstacle[0]) != 2
            or any(
                isinstance(value, bool) or not isinstance(value, (int, float))
                for value in obstacle[0]
            )
            or isinstance(obstacle[1], bool)
            or not isinstance(obstacle[1], (int, float))
            or not math.isfinite(float(obstacle[1]))
            or obstacle[1] <= 0.0
        ):
            raise ValueError("LOS circle obstacles need a finite center and positive radius")
        center = (float(obstacle[0][0]), float(obstacle[0][1]))
        radius = float(obstacle[1])
        if not all(math.isfinite(value) for value in center) or not math.isfinite(radius):
            raise ValueError("LOS circle obstacles need a finite center and positive radius")
        normalized_circles.append((center, radius))
    circle_obstacles = tuple(normalized_circles)

    def interpolate(first: Pose2D, second: Pose2D, fraction: float) -> Pose2D:
        return Pose2D(
            first.x_m + (second.x_m - first.x_m) * fraction,
            first.y_m + (second.y_m - first.y_m) * fraction,
            first.theta_rad + (second.theta_rad - first.theta_rad) * fraction,
        )

    def certified(
        g0: Pose2D, g1: Pose2D, e0: Pose2D, e1: Pose2D, low: float, high: float
    ) -> bool:
        middle = (low + high) / 2.0
        gm, em = interpolate(g0, g1, middle), interpolate(e0, e1, middle)
        rx, ry = em.x_m - gm.x_m, em.y_m - gm.y_m
        rvx = (e1.x_m - e0.x_m) - (g1.x_m - g0.x_m)
        rvy = (e1.y_m - e0.y_m) - (g1.y_m - g0.y_m)
        half_width = (high - low) / 2.0
        relative_speed = math.hypot(rvx, rvy)
        distance_lower = math.hypot(rx, ry) - relative_speed * half_width
        distance_upper = math.hypot(rx, ry) + relative_speed * half_width
        if distance_lower <= 0.0 or math.nextafter(distance_upper, math.inf) >= max_distance_m:
            return False
        bearing = abs(_wrap_angle(math.atan2(ry, rx) - gm.theta_rad))
        angular_speed = abs(g1.theta_rad - g0.theta_rad)
        bearing_rate = abs(rx * rvy - ry * rvx) / (distance_lower * distance_lower) + angular_speed
        bearing_upper = math.nextafter(bearing + bearing_rate * half_width, math.inf)
        if bearing_upper > max_bearing_rad:
            return False

        corners = (
            (interpolate(g0, g1, low).x_m, interpolate(g0, g1, low).y_m),
            (interpolate(g0, g1, high).x_m, interpolate(g0, g1, high).y_m),
            (interpolate(e0, e1, low).x_m, interpolate(e0, e1, low).y_m),
            (interpolate(e0, e1, high).x_m, interpolate(e0, e1, high).y_m),
        )
        if any(_segment_intersects_convex_envelope(start, end, corners) for start, end in obstacles):
            return False
        return not any(
            _circle_intersects_convex_envelope(center, radius, corners)
            for center, radius in circle_obstacles
        )

    def is_capture(gp: Pose2D, ep: Pose2D) -> bool:
        if math.hypot(ep.x_m - gp.x_m, ep.y_m - gp.y_m) == 0.0:
            return False
        return capture_predicate(
            gp,
            ep,
            line_of_sight=not _segment_blocks_los(
                (gp.x_m, gp.y_m),
                (ep.x_m, ep.y_m),
                obstacles,
                circle_obstacles,
            ),
            max_distance_m=max_distance_m,
            max_bearing_rad=max_bearing_rad,
        )

    for index, (g0, g1, e0, e1) in enumerate(zip(guardian, guardian[1:], explorer, explorer[1:])):
        def search(low: float, high: float, depth: int) -> bool:
            relative_start = (
                interpolate(e0.pose, e1.pose, low).x_m - interpolate(g0.pose, g1.pose, low).x_m,
                interpolate(e0.pose, e1.pose, low).y_m - interpolate(g0.pose, g1.pose, low).y_m,
            )
            relative_end = (
                interpolate(e0.pose, e1.pose, high).x_m - interpolate(g0.pose, g1.pose, high).x_m,
                interpolate(e0.pose, e1.pose, high).y_m - interpolate(g0.pose, g1.pose, high).y_m,
            )
            relative_delta = (
                relative_end[0] - relative_start[0],
                relative_end[1] - relative_start[1],
            )
            delta_squared = relative_delta[0] ** 2 + relative_delta[1] ** 2
            closest_fraction = 0.0 if delta_squared == 0.0 else max(
                0.0,
                min(
                    1.0,
                    -(
                        relative_start[0] * relative_delta[0]
                        + relative_start[1] * relative_delta[1]
                    )
                    / delta_squared,
                ),
            )
            closest_distance = math.hypot(
                relative_start[0] + closest_fraction * relative_delta[0],
                relative_start[1] + closest_fraction * relative_delta[1],
            )
            if closest_distance >= max_distance_m:
                return False
            if certified(g0.pose, g1.pose, e0.pose, e1.pose, low, high):
                return True
            for fraction in (low, (low + high) / 2.0, high):
                gp, ep = interpolate(g0.pose, g1.pose, fraction), interpolate(e0.pose, e1.pose, fraction)
                if is_capture(gp, ep):
                    return True
            if depth >= max_depth:
                return False
            middle = (low + high) / 2.0
            return search(low, middle, depth + 1) or search(middle, high, depth + 1)

        if search(0.0, 1.0, 0):
            return CaptureEventInterval(index, stamps[index], stamps[index + 1])
    return None


def _segment_intersects_convex_envelope(
    start: Tuple[float, float],
    end: Tuple[float, float],
    points: Sequence[Tuple[float, float]],
) -> bool:
    """Conservative LOS test against the convex hull swept by both origins."""
    hull = _convex_hull(points)
    if len(hull) == 1:
        return _point_segment_distance(hull[0], start, end)[0] == 0.0
    if len(hull) == 2:
        return segment_intersection(start, end, hull[0], hull[1]) is not None
    if _point_in_polygon(start, hull) or _point_in_polygon(end, hull):
        return True
    return any(
        segment_intersection(start, end, hull[index], hull[(index + 1) % len(hull)]) is not None
        for index in range(len(hull))
    )


def _circle_intersects_convex_envelope(
    center: Tuple[float, float],
    radius: float,
    points: Sequence[Tuple[float, float]],
) -> bool:
    hull = _convex_hull(points)
    if len(hull) == 1:
        distance = _distance(center, hull[0])
    elif len(hull) == 2:
        distance = _point_segment_distance(center, hull[0], hull[1])[0]
    elif _point_in_polygon(center, hull):
        return True
    else:
        distance = min(
            _point_segment_distance(center, hull[index], hull[(index + 1) % len(hull)])[0]
            for index in range(len(hull))
        )
    return distance <= radius


def _convex_hull(points: Sequence[Tuple[float, float]]) -> Tuple[Tuple[float, float], ...]:
    unique = sorted(set(points))
    if len(unique) <= 1:
        return tuple(unique)

    def cross(origin, first, second):
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (first[1] - origin[1]) * (second[0] - origin[0])

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return tuple(lower[:-1] + upper[:-1])
