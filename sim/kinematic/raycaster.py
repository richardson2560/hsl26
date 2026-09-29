# sim/kinematic/raycaster.py
"""First-hit planar ray casting with explicit blind-sector handling."""

import math
from typing import Optional, Sequence

import numpy as np

from .common import CircleTarget, Segment


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class FirstHitRaycaster:
    def __init__(self, *, max_range_m: float = 10.0) -> None:
        if not math.isfinite(max_range_m) or max_range_m <= 0.0:
            raise ValueError("max_range_m must be positive and finite")
        self.max_range_m = float(max_range_m)
        self._cached_segments: tuple[Segment, ...] | None = None
        self._segment_starts = np.empty((0, 2), dtype=np.float64)
        self._segment_vectors = np.empty((0, 2), dtype=np.float64)

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

    def cast_many(
        self,
        origin_xy: tuple[float, float],
        angles_rad: Sequence[float],
        segments: tuple[Segment, ...],
        targets: tuple[CircleTarget, ...] = (),
    ) -> tuple[Optional[float], ...]:
        """Vectorized equivalent of ``cast`` for a shared origin/map snapshot."""
        if not all(math.isfinite(float(value)) for value in origin_xy):
            raise ValueError("ray origin must be finite")
        if not isinstance(segments, tuple) or any(
            not isinstance(segment, Segment) for segment in segments
        ):
            raise ValueError("segments must be a tuple of Segment values")
        if not isinstance(targets, tuple) or any(
            not isinstance(target, CircleTarget) for target in targets
        ):
            raise ValueError("targets must be a tuple of CircleTarget values")
        if any(
            isinstance(angle, bool)
            or not isinstance(angle, (int, float))
            or not math.isfinite(angle)
            for angle in angles_rad
        ):
            raise ValueError("ray angles must be finite numbers")
        if len(angles_rad) == 0:
            return ()

        angles = np.asarray(angles_rad, dtype=np.float64)
        directions_x = np.cos(angles)
        directions_y = np.sin(angles)
        nearest = np.full(angles.shape, self.max_range_m, dtype=np.float64)
        if segments:
            self._prepare_segments(segments)
            bx = self._segment_starts[:, 0]
            by = self._segment_starts[:, 1]
            sx = self._segment_vectors[:, 0]
            sy = self._segment_vectors[:, 1]
            qx = bx - float(origin_xy[0])
            qy = by - float(origin_xy[1])
            cross = (
                directions_x[:, None] * sy[None, :]
                - directions_y[:, None] * sx[None, :]
            )
            with np.errstate(divide="ignore", invalid="ignore"):
                distances = (qx[None, :] * sy - qy[None, :] * sx) / cross
                along = (
                    qx[None, :] * directions_y[:, None]
                    - qy[None, :] * directions_x[:, None]
                ) / cross
            hits = (
                (np.abs(cross) >= 1e-12)
                & (distances >= 0.0)
                & (along >= 0.0)
                & (along <= 1.0)
            )
            nearest = np.minimum(
                nearest,
                np.min(np.where(hits, distances, self.max_range_m), axis=1),
            )

        origin_x, origin_y = float(origin_xy[0]), float(origin_xy[1])
        for target in targets:
            vx = target.center_xy[0] - origin_x
            vy = target.center_xy[1] - origin_y
            projection = vx * directions_x + vy * directions_y
            discriminant = (
                projection * projection
                - (vx * vx + vy * vy - target.radius_m**2)
            )
            hits = (projection >= 0.0) & (discriminant >= 0.0)
            distances = np.maximum(
                0.0,
                projection - np.sqrt(np.maximum(0.0, discriminant)),
            )
            nearest = np.minimum(
                nearest,
                np.where(hits, distances, self.max_range_m),
            )
        return tuple(
            float(distance) if distance < self.max_range_m else None
            for distance in nearest
        )

    def _prepare_segments(self, segments: tuple[Segment, ...]) -> None:
        if segments is self._cached_segments:
            return
        self._segment_starts = np.asarray(
            [segment.start_xy for segment in segments], dtype=np.float64
        )
        self._segment_vectors = np.asarray(
            [
                (
                    segment.end_xy[0] - segment.start_xy[0],
                    segment.end_xy[1] - segment.start_xy[1],
                )
                for segment in segments
            ],
            dtype=np.float64,
        )
        self._cached_segments = segments

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
