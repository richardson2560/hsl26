# sim/kinematic/raycaster.py
"""First-hit planar ray casting with explicit blind-sector handling."""

import math
from typing import Optional

from .common import CircleTarget, Segment


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class FirstHitRaycaster:
    def __init__(self, *, max_range_m: float = 10.0) -> None:
        if not math.isfinite(max_range_m) or max_range_m <= 0.0:
            raise ValueError("max_range_m must be positive and finite")
        self.max_range_m = float(max_range_m)

    def cast(
        self,
        origin_xy: tuple[float, float],
        angle_rad: float,
        segments: tuple[Segment, ...],
        targets: tuple[CircleTarget, ...] = (),
    ) -> Optional[float]:
        """Return only distance, never target identity or truth metadata."""
        if not all(math.isfinite(float(v)) for v in (*origin_xy, angle_rad)):
            raise ValueError("ray inputs must be finite")
        direction = (math.cos(angle_rad), math.sin(angle_rad))
        nearest = self.max_range_m
        for segment in segments:
            distance = self._segment_hit(origin_xy, direction, segment)
            if distance is not None:
                nearest = min(nearest, distance)
        for target in targets:
            distance = self._circle_hit(origin_xy, direction, target)
            if distance is not None:
                nearest = min(nearest, distance)
        return nearest if nearest < self.max_range_m else None

    @staticmethod
    def _segment_hit(
        origin: tuple[float, float],
        direction: tuple[float, float],
        segment: Segment,
    ) -> Optional[float]:
        ax, ay = origin
        dx, dy = direction
        bx, by = segment.start_xy
        sx = segment.end_xy[0] - bx
        sy = segment.end_xy[1] - by
        cross = dx * sy - dy * sx
        if abs(cross) < 1e-12:
            return None
        qx, qy = bx - ax, by - ay
        distance = (qx * sy - qy * sx) / cross
        along = (qx * dy - qy * dx) / cross
        if distance >= 0.0 and 0.0 <= along <= 1.0:
            return distance
        return None

    @staticmethod
    def _circle_hit(
        origin: tuple[float, float],
        direction: tuple[float, float],
        target: CircleTarget,
    ) -> Optional[float]:
        ox, oy = origin
        cx, cy = target.center_xy
        vx, vy = cx - ox, cy - oy
        projection = vx * direction[0] + vy * direction[1]
        discriminant = projection * projection - (vx * vx + vy * vy - target.radius_m ** 2)
        if projection < 0.0 or discriminant < 0.0:
            return None
        distance = projection - math.sqrt(discriminant)
        return distance if distance >= 0.0 else 0.0

    @staticmethod
    def in_blind_sector(
        angle_rad: float,
        center_rad: float,
        half_width_rad: float,
    ) -> bool:
        if half_width_rad < 0.0 or half_width_rad > math.pi:
            raise ValueError("blind-sector half width must be in [0, pi]")
        return abs(_angle_delta(angle_rad, center_rad)) <= half_width_rad

