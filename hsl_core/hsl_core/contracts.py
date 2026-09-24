# hsl_core/hsl_core/contracts.py
"""ROS-independent validation for revision-2 temporal and spatial contracts.

"""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


def _non_empty(value: str, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must not be empty")
    return value


@dataclass(frozen=True)
class ContractHeader:
    """Revision-2 metadata required by every leased domain record."""

    schema_version: int
    source_id: str
    source_session: str
    seq: int
    stage_id: str
    clock_epoch: str
    localization_epoch: str
    frame_id: str
    observation_stamp_ns: int
    state_stamp_ns: int
    publication_stamp_ns: int
    valid_until_ns: int
    map_version: int
    topology_version: int
    validity: int

    def __post_init__(self) -> None:
        if self.schema_version != 2:
            raise ValueError("schema_version must be 2")
        for value, name in (
            (self.seq, "seq"),
            (self.observation_stamp_ns, "observation_stamp_ns"),
            (self.state_stamp_ns, "state_stamp_ns"),
            (self.publication_stamp_ns, "publication_stamp_ns"),
            (self.valid_until_ns, "valid_until_ns"),
            (self.map_version, "map_version"),
            (self.topology_version, "topology_version"),
        ):
            if (
                not isinstance(value, int)
                or isinstance(value, bool)
                or value < 0
            ):
                raise ValueError(f"{name} must be a non-negative integer")
        _non_empty(self.source_id, "source_id")
        _non_empty(self.source_session, "source_session")
        _non_empty(self.clock_epoch, "clock_epoch")
        _non_empty(self.localization_epoch, "localization_epoch")
        if self.validity not in (0, 1, 2):
            raise ValueError("validity must be INVALID, VALID, or DEGRADED")
        if self.observation_stamp_ns > self.publication_stamp_ns:
            raise ValueError("observation_stamp_ns cannot exceed publication_stamp_ns")
        if self.state_stamp_ns > self.publication_stamp_ns:
            raise ValueError("state_stamp_ns cannot exceed publication_stamp_ns")
        if self.valid_until_ns <= self.publication_stamp_ns:
            raise ValueError("valid_until_ns must be after publication_stamp_ns")


def validate_header(
    header: ContractHeader,
    *,
    now_ns: int,
    expected_clock_epoch: str,
    expected_localization_epoch: str,
    expected_stage_id: str | None = None,
    require_frame: bool = False,
    require_stage: bool = False,
    allowed_future_ns: int = 0,
) -> None:
    """Reject a header that cannot participate in the current evaluation."""

    if header.clock_epoch != expected_clock_epoch:
        raise ValueError("clock_epoch mismatch")
    if header.localization_epoch != expected_localization_epoch:
        raise ValueError("localization_epoch mismatch")
    if expected_stage_id is not None and header.stage_id != expected_stage_id:
        raise ValueError("stage_id mismatch")
    if require_stage and not header.stage_id:
        raise ValueError("stage_id is required for authority records")
    if require_frame and not header.frame_id:
        raise ValueError("frame_id is required for geometric records")
    if not isinstance(now_ns, int) or isinstance(now_ns, bool):
        raise ValueError("now_ns must be an integer")
    if (
        not isinstance(allowed_future_ns, int)
        or isinstance(allowed_future_ns, bool)
        or allowed_future_ns < 0
    ):
        raise ValueError("allowed_future_ns must be a non-negative integer")
    if header.validity == 0:
        raise ValueError("record validity is INVALID")
    if header.publication_stamp_ns > now_ns + allowed_future_ns:
        raise ValueError("publication_stamp_ns is in the future")
    if now_ns >= header.valid_until_ns:
        raise ValueError("record lease has expired")


def validate_covariance(
    values: Iterable[float],
    dimension: int,
    *,
    symmetry_tolerance: float = 1e-9,
    psd_tolerance: float = 1e-9,
) -> Tuple[float, ...]:
    """Validate a finite row-major covariance and return an immutable tuple."""

    if not isinstance(dimension, int) or isinstance(dimension, bool) or dimension < 1:
        raise ValueError("dimension must be a positive integer")
    if symmetry_tolerance < 0.0 or psd_tolerance < 0.0:
        raise ValueError("covariance tolerances must be non-negative")
    flat = tuple(float(value) for value in values)
    if len(flat) != dimension * dimension:
        raise ValueError(f"covariance must contain exactly {dimension * dimension} values")
    if any(not math.isfinite(value) for value in flat):
        raise ValueError("covariance must be finite")
    matrix = tuple(
        tuple(flat[row * dimension + column] for column in range(dimension))
        for row in range(dimension)
    )
    for row in range(dimension):
        for column in range(row):
            if abs(matrix[row][column] - matrix[column][row]) > symmetry_tolerance:
                raise ValueError("covariance must be symmetric")

    def determinant(values_: Tuple[Tuple[float, ...], ...]) -> float:
        size = len(values_)
        if size == 1:
            return values_[0][0]
        return sum(
            ((-1.0) ** column) * values_[0][column]
            * determinant(
                tuple(
                    tuple(values_[row][other] for other in range(size) if other != column)
                    for row in range(1, size)
                )
            )
            for column in range(size)
        )

    for mask in range(1, 1 << dimension):
        indices = tuple(index for index in range(dimension) if mask & (1 << index))
        minor = tuple(tuple(matrix[row][column] for column in indices) for row in indices)
        if determinant(minor) < -psd_tolerance:
            raise ValueError("covariance must be positive semidefinite")
    return flat


def validate_polygon(vertices: Iterable[Tuple[float, float]]) -> Tuple[Tuple[float, float], ...]:
    """Validate a simple polygon's minimum geometric preconditions."""

    points = tuple((float(x), float(y)) for x, y in vertices)
    if len(points) < 3:
        raise ValueError("polygon requires at least three vertices")
    if any(not math.isfinite(value) for point in points for value in point):
        raise ValueError("polygon coordinates must be finite")
    if len(set(points)) != len(points):
        raise ValueError("polygon vertices must be distinct")
    if points[0] == points[-1]:
        raise ValueError("polygon must not repeat its closing vertex")
    area2 = sum(
        points[index][0] * points[(index + 1) % len(points)][1]
        - points[(index + 1) % len(points)][0] * points[index][1]
        for index in range(len(points))
    )
    if abs(area2) <= 1e-12:
        raise ValueError("polygon vertices must be non-collinear")
    if area2 < 0.0:
        raise ValueError("polygon must be counterclockwise")
    for first in range(len(points)):
        first_end = (first + 1) % len(points)
        for second in range(first + 1, len(points)):
            second_end = (second + 1) % len(points)
            if first == second or first_end == second or second_end == first:
                continue
            if _segments_intersect(
                points[first],
                points[first_end],
                points[second],
                points[second_end],
            ):
                raise ValueError("polygon must be simple")
    return points


def _segments_intersect(
    first_start: Tuple[float, float],
    first_end: Tuple[float, float],
    second_start: Tuple[float, float],
    second_end: Tuple[float, float],
) -> bool:
    def orientation(
        a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]
    ) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def on_segment(
        a: Tuple[float, float], b: Tuple[float, float], point: Tuple[float, float]
    ) -> bool:
        return (
            min(a[0], b[0]) <= point[0] <= max(a[0], b[0])
            and min(a[1], b[1]) <= point[1] <= max(a[1], b[1])
        )

    values = (
        orientation(first_start, first_end, second_start),
        orientation(first_start, first_end, second_end),
        orientation(second_start, second_end, first_start),
        orientation(second_start, second_end, first_end),
    )
    epsilon = 1e-12
    if all(abs(value) <= epsilon for value in values):
        return any(
            on_segment(first_start, first_end, point)
            and on_segment(second_start, second_end, point)
            for point in (first_start, first_end, second_start, second_end)
        )
    return (
        not (
            (values[0] > epsilon and values[1] > epsilon)
            or (values[0] < -epsilon and values[1] < -epsilon)
        )
        and not (
            (values[2] > epsilon and values[3] > epsilon)
            or (values[2] < -epsilon and values[3] < -epsilon)
        )
    )
