# hsl_core/hsl_core/mapping.py
"""Deterministic local collision occupancy and observed-free coverage."""

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


UNKNOWN = np.uint8(0)
OBSERVED_FREE = np.uint8(1)
OCCUPIED = np.uint8(2)


@dataclass(frozen=True)
class LocalObstacleConfig:
    """Explicit, profile-owned parameters for the bounded fast path."""

    z_min_m: float = -0.10
    z_max_m: float = 0.40
    self_radius_m: float = 0.18
    corridor_width_m: float = 0.40
    resolution_m: float = 0.05
    origin_x_m: float = -1.0
    origin_y_m: float = -2.0
    width: int = 120
    height: int = 80
    max_points: int = 100_000

    def __post_init__(self) -> None:
        values = (
            self.z_min_m, self.z_max_m, self.self_radius_m,
            self.corridor_width_m, self.resolution_m,
            self.origin_x_m, self.origin_y_m,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("mapping configuration must be finite")
        if self.z_min_m > self.z_max_m:
            raise ValueError("z_min_m must not exceed z_max_m")
        if self.self_radius_m < 0.0 or self.corridor_width_m <= 0.0:
            raise ValueError("self radius and corridor width must be valid")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if not isinstance(self.width, int) or isinstance(self.width, bool) or self.width <= 0:
            raise ValueError("width must be a positive integer")
        if not isinstance(self.height, int) or isinstance(self.height, bool) or self.height <= 0:
            raise ValueError("height must be a positive integer")
        if not isinstance(self.max_points, int) or isinstance(self.max_points, bool) or self.max_points <= 0:
            raise ValueError("max_points must be a positive integer")


@dataclass(frozen=True)
class CoverageGrid:
    """Bounded row-major coverage state in the local odom frame."""

    origin_xy_m: Tuple[float, float]
    resolution_m: float
    width: int
    height: int
    cells: np.ndarray

    def __post_init__(self) -> None:
        if len(self.origin_xy_m) != 2 or not all(math.isfinite(float(v)) for v in self.origin_xy_m):
            raise ValueError("origin_xy_m must contain two finite values")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError("coverage resolution must be positive and finite")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("coverage dimensions must be positive")
        cells = np.asarray(self.cells, dtype=np.uint8)
        if cells.shape != (self.height, self.width):
            raise ValueError("coverage cells must have shape (height, width)")
        if not np.isin(cells, (UNKNOWN, OBSERVED_FREE, OCCUPIED)).all():
            raise ValueError("coverage cells contain an invalid state")
        object.__setattr__(self, "cells", cells.copy())


@dataclass(frozen=True)
class LocalObstacleSnapshot:
    """Fast-path occupied returns plus coverage and a bounded corridor metric."""

    obstacles_xyz: np.ndarray
    coverage: CoverageGrid
    free_distance_m: float
    complete: bool
    frontal_coverage_valid: bool = False

    def __post_init__(self) -> None:
        obstacles = np.asarray(self.obstacles_xyz, dtype=float)
        if obstacles.ndim != 2 or obstacles.shape[1] != 3:
            raise ValueError("obstacles_xyz must have shape (N, 3)")
        if not np.isfinite(obstacles).all() or not math.isfinite(self.free_distance_m):
            raise ValueError("obstacle snapshot values must be finite")
        if not isinstance(self.complete, (bool, np.bool_)):
            raise ValueError("complete must be boolean")
        if not isinstance(self.frontal_coverage_valid, (bool, np.bool_)):
            raise ValueError("frontal_coverage_valid must be boolean")
        object.__setattr__(self, "obstacles_xyz", obstacles.copy())


def _cell_index(x_m: float, y_m: float, config: LocalObstacleConfig) -> Tuple[int, int] | None:
    ix = math.floor((x_m - config.origin_x_m) / config.resolution_m)
    iy = math.floor((y_m - config.origin_y_m) / config.resolution_m)
    if 0 <= ix < config.width and 0 <= iy < config.height:
        return ix, iy
    return None


def _ray_cells(x_m: float, y_m: float, config: LocalObstacleConfig) -> list[Tuple[int, int]]:
    distance = math.hypot(x_m, y_m)
    if distance == 0.0:
        return []
    steps = max(1, int(math.ceil(distance / (config.resolution_m * 0.5))))
    result: list[Tuple[int, int]] = []
    for fraction in np.linspace(0.0, 1.0, steps + 1)[:-1]:
        cell = _cell_index(x_m * float(fraction), y_m * float(fraction), config)
        if cell is not None and (not result or result[-1] != cell):
            result.append(cell)
    return result


def _central_corridor_distance(
    cells: np.ndarray,
    config: LocalObstacleConfig,
) -> Tuple[float, bool]:
    """Return certified centreline clearance up to the first unknown/hit cell."""

    start_x = max(0.0, config.self_radius_m)
    first_cell = _cell_index(start_x, 0.0, config)
    if first_cell is None:
        return 0.0, False
    ix = first_cell[0]
    corridor_rows = [
        row
        for row in range(config.height)
        if abs(
            config.origin_y_m + (row + 0.5) * config.resolution_m
        ) <= config.corridor_width_m / 2.0
    ]
    observed_free_cells = 0
    for current_ix in range(ix, config.width):
        cell_start_x = config.origin_x_m + current_ix * config.resolution_m
        if any(cells[row, current_ix] == OCCUPIED for row in corridor_rows):
            distance = max(0.0, cell_start_x)
            return distance, observed_free_cells > 0
        state = cells[first_cell[1], current_ix]
        if state == UNKNOWN:
            distance = max(0.0, cell_start_x)
            if observed_free_cells == 0:
                return 0.0, False
            return distance, True
        if state == OBSERVED_FREE:
            observed_free_cells += 1
    max_grid_x = config.origin_x_m + config.width * config.resolution_m
    distance = max(0.0, max_grid_x)
    return distance, distance >= start_x + config.resolution_m


class LocalObstacleBuilder:
    """Build local collision returns and conservative observed-free coverage.

    Points are expected in the local sensor/base frame after deskew and
    extrinsic normalization.  Filtering is deliberately explicit and never
    treats missing returns as free space.
    """

    def __init__(self, config: LocalObstacleConfig | None = None) -> None:
        self.config = config or LocalObstacleConfig()

    def build(self, points_xyz: np.ndarray) -> LocalObstacleSnapshot:
        points = np.asarray(points_xyz, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_xyz must have shape (N, 3)")
        if points.shape[0] > self.config.max_points:
            raise ValueError("point cloud exceeds configured bound")
        if not np.isfinite(points).all():
            raise ValueError("point cloud must contain only finite values")

        cfg = self.config
        height_mask = (points[:, 2] >= cfg.z_min_m) & (points[:, 2] <= cfg.z_max_m)
        radius_mask = np.hypot(points[:, 0], points[:, 1]) >= cfg.self_radius_m
        valid = points[height_mask & radius_mask]
        cells = np.full((cfg.height, cfg.width), UNKNOWN, dtype=np.uint8)
        distances = np.hypot(valid[:, 0], valid[:, 1])
        valid = valid[np.argsort(distances, kind="stable")]

        in_grid = np.array(
            [_cell_index(float(point[0]), float(point[1]), cfg) is not None for point in valid],
            dtype=bool,
        )
        for point in valid:
            hit_cell = _cell_index(float(point[0]), float(point[1]), cfg)
            if hit_cell is None:
                continue
            for cell in _ray_cells(float(point[0]), float(point[1]), cfg):
                if cells[cell[1], cell[0]] == OCCUPIED:
                    break
                cells[cell[1], cell[0]] = OBSERVED_FREE
            cells[hit_cell[1], hit_cell[0]] = OCCUPIED

        free_distance, frontal_coverage_valid = _central_corridor_distance(cells, cfg)

        return LocalObstacleSnapshot(
            obstacles_xyz=valid,
            coverage=CoverageGrid(
                (cfg.origin_x_m, cfg.origin_y_m),
                cfg.resolution_m,
                cfg.width,
                cfg.height,
                cells,
            ),
            free_distance_m=free_distance,
            complete=True,
            frontal_coverage_valid=frontal_coverage_valid,
        )
