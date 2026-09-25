# hsl_core/hsl_core/perception/segmenter.py
"""Conservative point-cloud candidate segmentation for opponent perception.

This module only creates semantic candidates. It never mutates or filters the
collision/safety return layer; callers retain the complete valid cloud.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class PointCluster:
    points_m: np.ndarray
    source_indices: tuple[int, ...]

    def __post_init__(self) -> None:
        points = np.asarray(self.points_m, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("cluster points must have shape (N, 3), N > 0")
        if not np.all(np.isfinite(points)):
            raise ValueError("cluster points must be finite")
        if len(self.source_indices) != len(points):
            raise ValueError("source_indices must match cluster points")
        object.__setattr__(self, "points_m", points.copy())


@dataclass(frozen=True)
class SegmentationConfig:
    connectivity_radius_m: float = 0.12
    min_points: int = 6
    max_points: int = 2000
    min_extent_m: float = 0.03
    max_extent_m: float = 1.5

    def __post_init__(self) -> None:
        if not math.isfinite(self.connectivity_radius_m) or self.connectivity_radius_m <= 0:
            raise ValueError("connectivity_radius_m must be positive and finite")
        if self.min_points <= 0 or self.max_points < self.min_points:
            raise ValueError("invalid point-count bounds")
        if not math.isfinite(self.min_extent_m) or self.min_extent_m < 0:
            raise ValueError("min_extent_m must be finite and non-negative")
        if not math.isfinite(self.max_extent_m) or self.max_extent_m < self.min_extent_m:
            raise ValueError("invalid extent bounds")


class CandidateSegmenter:
    """Deterministic Euclidean clustering with explicit validity rejection."""

    def __init__(self, config: SegmentationConfig = SegmentationConfig()) -> None:
        self.config = config

    def segment(
        self,
        points_m: Sequence[Sequence[float]],
        *,
        valid_mask: Sequence[bool] | None = None,
        static_mask: Sequence[bool] | None = None,
    ) -> tuple[PointCluster, ...]:
        points = np.asarray(points_m, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_m must have shape (N, 3)")
        if not np.all(np.isfinite(points)):
            raise ValueError("points_m must be finite")
        n = len(points)
        valid = np.ones(n, dtype=bool) if valid_mask is None else np.asarray(valid_mask, dtype=bool)
        static = np.zeros(n, dtype=bool) if static_mask is None else np.asarray(static_mask, dtype=bool)
        if valid.shape != (n,) or static.shape != (n,):
            raise ValueError("masks must have shape (N,)")
        active_indices = [index for index in range(n) if valid[index] and not static[index]]
        unseen = set(active_indices)
        clusters: list[PointCluster] = []
        radius_sq = self.config.connectivity_radius_m ** 2
        while unseen:
            seed = min(unseen)
            queue = [seed]
            unseen.remove(seed)
            members = []
            while queue:
                current = queue.pop(0)
                members.append(current)
                if not unseen:
                    continue
                remaining = np.fromiter(unseen, dtype=int)
                distances = np.sum((points[remaining] - points[current]) ** 2, axis=1)
                neighbours = remaining[distances <= radius_sq]
                for neighbour in neighbours.tolist():
                    unseen.remove(neighbour)
                    queue.append(neighbour)
            cluster_points = points[members]
            extent = float(np.max(np.ptp(cluster_points, axis=0)))
            if (
                self.config.min_points <= len(members) <= self.config.max_points
                and self.config.min_extent_m <= extent <= self.config.max_extent_m
            ):
                clusters.append(PointCluster(cluster_points, tuple(members)))
        return tuple(clusters)
