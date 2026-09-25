# hsl_core/hsl_core/occupancy.py
"""Versioned layered occupancy mapping for Phase 3 P3.1.

The mapper is deliberately ROS-independent.  A batch is validated completely
before it is committed, so callers receive either one atomic new snapshot or
no state change.  Correlated ray evidence is capped per cell and transient
semantic evidence decays to UNKNOWN rather than to observed free space.
"""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Optional, Sequence, Tuple

import numpy as np

from .mapping import OCCUPIED, OBSERVED_FREE, UNKNOWN, _cell_index


class RayKind(IntEnum):
    """Source semantics for the terminal return of a ray."""

    STATIC = 0
    DYNAMIC = 1


def _finite(value: float, name: str) -> float:
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _probability(value: float, name: str) -> float:
    value = _finite(value, name)
    if not 0.0 < value < 1.0:
        raise ValueError(f"{name} must be strictly between zero and one")
    return value


@dataclass(frozen=True)
class OccupancyMapConfig:
    """Profile-owned bounded grid and Bayesian evidence parameters."""

    origin_xy_m: Tuple[float, float] = (-1.0, -1.0)
    resolution_m: float = 0.1
    width: int = 40
    height: int = 30
    p_free: float = 0.35
    p_occupied: float = 0.70
    min_probability: float = 0.20
    max_probability: float = 0.70
    max_log_odds: float = 4.0
    max_batch_abs_update: float = 1.5
    dynamic_decay_tau_s: float = 2.0
    max_rays_per_batch: int = 10_000

    def __post_init__(self) -> None:
        if len(self.origin_xy_m) != 2 or not all(
            math.isfinite(float(value)) for value in self.origin_xy_m
        ):
            raise ValueError("origin_xy_m must contain two finite values")
        _finite(self.resolution_m, "resolution_m")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        for value, name in (
            (self.width, "width"),
            (self.height, "height"),
            (self.max_rays_per_batch, "max_rays_per_batch"),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        _probability(self.p_free, "p_free")
        _probability(self.p_occupied, "p_occupied")
        _probability(self.min_probability, "min_probability")
        _probability(self.max_probability, "max_probability")
        if not self.p_free < 0.5 or not self.p_occupied > 0.5:
            raise ValueError("free and occupied probabilities must straddle 0.5")
        if not 0.0 < self.min_probability < 0.5 <= self.max_probability < 1.0:
            raise ValueError("probability thresholds must leave an unknown band")
        for value, name in (
            (self.max_log_odds, "max_log_odds"),
            (self.max_batch_abs_update, "max_batch_abs_update"),
            (self.dynamic_decay_tau_s, "dynamic_decay_tau_s"),
        ):
            _finite(value, name)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")

    @property
    def shape(self) -> Tuple[int, int]:
        return self.height, self.width

    @property
    def free_log_odds(self) -> float:
        return math.log(self.p_free / (1.0 - self.p_free))

    @property
    def occupied_log_odds(self) -> float:
        return math.log(self.p_occupied / (1.0 - self.p_occupied))

    @property
    def free_threshold_log_odds(self) -> float:
        return math.log(self.min_probability / (1.0 - self.min_probability))

    @property
    def occupied_threshold_log_odds(self) -> float:
        return math.log(self.max_probability / (1.0 - self.max_probability))


@dataclass(frozen=True)
class RayEvidence:
    """One bounded ray in the mapper's parent frame."""

    origin_xy_m: Tuple[float, float]
    endpoint_xy_m: Tuple[float, float]
    stamp_s: float
    kind: RayKind = RayKind.STATIC
    hit: bool = True

    def __post_init__(self) -> None:
        for point_name, point in (("origin_xy_m", self.origin_xy_m), ("endpoint_xy_m", self.endpoint_xy_m)):
            if len(point) != 2 or not all(math.isfinite(float(value)) for value in point):
                raise ValueError(f"{point_name} must contain two finite values")
        _finite(self.stamp_s, "stamp_s")
        object.__setattr__(self, "kind", RayKind(self.kind))
        if not isinstance(self.hit, (bool, np.bool_)):
            raise ValueError("hit must be boolean")


@dataclass(frozen=True)
class GridSnapshot:
    """Immutable layered map snapshot with one atomic map version."""

    map_version: int
    stamp_s: float
    structural: np.ndarray
    collision: np.ndarray
    semantic: np.ndarray
    observed_free: np.ndarray
    last_observed_s: np.ndarray

    def __post_init__(self) -> None:
        if isinstance(self.map_version, bool) or not isinstance(self.map_version, int) or self.map_version < 0:
            raise ValueError("map_version must be a non-negative integer")
        _finite(self.stamp_s, "stamp_s")
        arrays = {
            "structural": np.asarray(self.structural),
            "collision": np.asarray(self.collision),
            "semantic": np.asarray(self.semantic),
            "observed_free": np.asarray(self.observed_free),
            "last_observed_s": np.asarray(self.last_observed_s, dtype=float),
        }
        shape = arrays["structural"].shape
        if len(shape) != 2 or shape[0] <= 0 or shape[1] <= 0:
            raise ValueError("map layers must be non-empty two-dimensional arrays")
        for name, array in arrays.items():
            if array.shape != shape:
                raise ValueError(f"{name} layer shape mismatch")
            if not np.isfinite(array).all():
                raise ValueError(f"{name} layer must be finite")
        for name in ("structural", "collision", "semantic", "observed_free"):
            layer = arrays[name]
            if not np.issubdtype(layer.dtype, np.integer) or not np.isin(
                layer, (UNKNOWN, OBSERVED_FREE, OCCUPIED)
            ).all():
                raise ValueError(f"{name} layer contains an invalid state")
        if np.any(arrays["last_observed_s"] < 0.0):
            raise ValueError("last_observed_s cannot contain negative values")
        for name, array in arrays.items():
            object.__setattr__(
                self,
                name,
                array.astype(np.uint8, copy=True)
                if name != "last_observed_s"
                else array.copy(),
            )


def _state(log_odds: np.ndarray, config: OccupancyMapConfig) -> np.ndarray:
    result = np.full(log_odds.shape, UNKNOWN, dtype=np.uint8)
    result[log_odds <= config.free_threshold_log_odds] = OBSERVED_FREE
    result[log_odds >= config.occupied_threshold_log_odds] = OCCUPIED
    return result


class OccupancyMapper:
    """Atomic layered occupancy mapper with bounded correlated evidence."""

    def __init__(self, config: Optional[OccupancyMapConfig] = None) -> None:
        self.config = config or OccupancyMapConfig()
        shape = self.config.shape
        zeros = np.zeros(shape, dtype=float)
        self._static_log_odds = zeros.copy()
        self._dynamic_log_odds = zeros.copy()
        self._observed_free = np.zeros(shape, dtype=np.uint8)
        self._last_observed_s = np.zeros(shape, dtype=float)
        self._last_dynamic_s = np.zeros(shape, dtype=float)
        self._stamp_s = 0.0
        self._map_version = 0

    @property
    def snapshot(self) -> GridSnapshot:
        structural = _state(self._static_log_odds, self.config)
        semantic = _state(self._dynamic_log_odds, self.config)
        collision = np.where(
            (structural == OCCUPIED) | (semantic == OCCUPIED),
            OCCUPIED,
            np.where(
                (structural == OBSERVED_FREE) & (semantic != OCCUPIED),
                OBSERVED_FREE,
                UNKNOWN,
            ),
        ).astype(np.uint8)
        return GridSnapshot(
            self._map_version,
            self._stamp_s,
            structural,
            collision,
            semantic,
            self._observed_free,
            self._last_observed_s,
        )

    def update(self, rays: Sequence[RayEvidence]) -> GridSnapshot:
        """Atomically apply a validated scan batch and return its new version."""

        rays = tuple(rays)
        if not all(isinstance(ray, RayEvidence) for ray in rays):
            raise TypeError("rays must contain only RayEvidence records")
        if len(rays) > self.config.max_rays_per_batch:
            raise ValueError("ray batch exceeds configured bound")
        if not rays:
            return self.snapshot
        if any(ray.stamp_s < self._stamp_s for ray in rays):
            raise ValueError("ray timestamps must not move backwards")

        static = self._static_log_odds.copy()
        dynamic = self._dynamic_log_odds.copy()
        observed_free = self._observed_free.copy()
        last_observed = self._last_observed_s.copy()
        last_dynamic = self._last_dynamic_s.copy()
        cfg = self.config
        static_delta: dict[Tuple[int, int], float] = {}
        dynamic_delta: dict[Tuple[int, int], float] = {}
        free_cells: set[Tuple[int, int]] = set()

        for ray in rays:
            cells = _grid_ray_cells(ray.origin_xy_m, ray.endpoint_xy_m, cfg)
            endpoint = _cell_index(
                float(ray.endpoint_xy_m[0]),
                float(ray.endpoint_xy_m[1]),
                _mapping_config(cfg),
            )
            if ray.hit and endpoint is not None:
                target = dynamic_delta if ray.kind == RayKind.DYNAMIC else static_delta
                target[endpoint] = target.get(endpoint, 0.0) + cfg.occupied_log_odds
                cells = cells[:-1] if cells and cells[-1] == endpoint else cells
            free_cells.update(cells)
            if ray.kind == RayKind.STATIC:
                for cell in cells:
                    static_delta[cell] = static_delta.get(cell, 0.0) + cfg.free_log_odds

        for cell, value in static_delta.items():
            static[cell[1], cell[0]] = float(np.clip(
                static[cell[1], cell[0]] + np.clip(value, -cfg.max_batch_abs_update, cfg.max_batch_abs_update),
                -cfg.max_log_odds,
                cfg.max_log_odds,
            ))
        for cell, value in dynamic_delta.items():
            dynamic[cell[1], cell[0]] = float(np.clip(
                dynamic[cell[1], cell[0]] + np.clip(value, -cfg.max_batch_abs_update, cfg.max_batch_abs_update),
                -cfg.max_log_odds,
                cfg.max_log_odds,
            ))
        for cell in free_cells:
            observed_free[cell[1], cell[0]] = OBSERVED_FREE
        for ray in rays:
            for cell in _grid_ray_cells(ray.origin_xy_m, ray.endpoint_xy_m, cfg):
                last_observed[cell[1], cell[0]] = max(last_observed[cell[1], cell[0]], ray.stamp_s)
            if ray.hit and ray.kind == RayKind.DYNAMIC:
                endpoint = _cell_index(
                    float(ray.endpoint_xy_m[0]),
                    float(ray.endpoint_xy_m[1]),
                    _mapping_config(cfg),
                )
                if endpoint is not None:
                    last_dynamic[endpoint[1], endpoint[0]] = max(last_dynamic[endpoint[1], endpoint[0]], ray.stamp_s)

        self._static_log_odds = static
        self._dynamic_log_odds = dynamic
        self._observed_free = observed_free
        self._last_observed_s = last_observed
        self._last_dynamic_s = last_dynamic
        if rays:
            self._stamp_s = max(ray.stamp_s for ray in rays)
        self._map_version += 1
        return self.snapshot

    def decay(self, now_s: float) -> GridSnapshot:
        """Decay only dynamic evidence; aged occupancy becomes UNKNOWN."""

        now_s = _finite(now_s, "now_s")
        if now_s < self._stamp_s:
            raise ValueError("decay time must not move backwards")
        age = np.maximum(0.0, now_s - self._last_dynamic_s)
        factor = np.exp(-age / self.config.dynamic_decay_tau_s)
        self._dynamic_log_odds *= factor
        self._stamp_s = now_s
        self._map_version += 1
        return self.snapshot


@dataclass(frozen=True)
class _GridConfig:
    origin_x_m: float
    origin_y_m: float
    resolution_m: float
    width: int
    height: int


def _mapping_config(config: OccupancyMapConfig) -> _GridConfig:
    return _GridConfig(
        origin_x_m=config.origin_xy_m[0],
        origin_y_m=config.origin_xy_m[1],
        resolution_m=config.resolution_m,
        width=config.width,
        height=config.height,
    )


def _grid_ray_cells(
    origin_xy_m: Tuple[float, float],
    endpoint_xy_m: Tuple[float, float],
    config: OccupancyMapConfig,
) -> list[Tuple[int, int]]:
    grid = _mapping_config(config)
    dx = float(endpoint_xy_m[0] - origin_xy_m[0])
    dy = float(endpoint_xy_m[1] - origin_xy_m[1])
    distance = math.hypot(dx, dy)
    if distance == 0.0:
        return []
    steps = max(1, int(math.ceil(distance / (config.resolution_m * 0.5))))
    cells: list[Tuple[int, int]] = []
    for fraction in np.linspace(0.0, 1.0, steps + 1):
        x = origin_xy_m[0] + dx * float(fraction)
        y = origin_xy_m[1] + dy * float(fraction)
        cell = _cell_index(x, y, grid)
        if cell is not None and (not cells or cells[-1] != cell):
            cells.append(cell)
    return cells
