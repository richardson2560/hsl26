# sim/kinematic/visualizer.py
"""Headless diagnostic renderers for the Phase-3 kinematic testbed.

The renderers consume a pose, geometry and sensor observation.  They do not
run the plant, query the referee or alter policy-visible state.  Matplotlib is
optional and imported only by ``show_*`` helpers.
"""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from hsl_core.types import Pose2D

from .common import SensorObservation, WorldGeometry


@dataclass(frozen=True)
class TopViewConfig:
    width_px: int = 800
    height_px: int = 600
    world_bounds: Optional[tuple[float, float, float, float]] = None
    ray_length_m: float = 8.0

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0 or self.ray_length_m <= 0.0:
            raise ValueError("top-view dimensions and ray length must be positive")
        if self.world_bounds is not None:
            if len(self.world_bounds) != 4:
                raise ValueError("world_bounds must be (xmin, ymin, xmax, ymax)")
            xmin, ymin, xmax, ymax = self.world_bounds
            if not (xmin < xmax and ymin < ymax):
                raise ValueError("world_bounds must have increasing limits")


@dataclass(frozen=True)
class DoomConfig:
    width_px: int = 640
    height_px: int = 360
    wall_height_m: float = 1.0
    focal_px: float = 320.0
    max_range_m: float = 10.0
    fov_rad: float = math.pi / 3.0

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("Doom dimensions must be positive")
        if (
            self.wall_height_m <= 0.0
            or self.focal_px <= 0.0
            or self.max_range_m <= 0.0
        ):
            raise ValueError("Doom projection parameters must be positive")
        if self.fov_rad <= 0.0 or self.fov_rad >= math.pi:
            raise ValueError("fov_rad must be strictly between 0 and pi")


def _bounds(geometry: WorldGeometry, pose: Pose2D, config: TopViewConfig) -> tuple[float, float, float, float]:
    if config.world_bounds is not None:
        return config.world_bounds
    points = [(pose.x_m, pose.y_m)]
    for segment in geometry.static_segments:
        points.extend((segment.start_xy, segment.end_xy))
    for target in geometry.dynamic_targets:
        points.append(target.center_xy)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    margin = max(0.5, config.ray_length_m * 0.05)
    return min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin


def render_top_view(
    pose: Pose2D,
    geometry: WorldGeometry,
    observation: SensorObservation,
    config: TopViewConfig = TopViewConfig(),
) -> np.ndarray:
    """Return an RGB uint8 image; valid rays are green and blind rays red."""
    if len(observation.ranges_m) == 0:
        raise ValueError("observation must contain at least one beam")
    xmin, ymin, xmax, ymax = _bounds(geometry, pose, config)
    image = np.full((config.height_px, config.width_px, 3), 248, dtype=np.uint8)

    def pixel(point: tuple[float, float]) -> tuple[int, int]:
        px = round((point[0] - xmin) / (xmax - xmin) * (config.width_px - 1))
        py = round((ymax - point[1]) / (ymax - ymin) * (config.height_px - 1))
        return max(0, min(config.width_px - 1, px)), max(0, min(config.height_px - 1, py))

    def draw_line(a: tuple[int, int], b: tuple[int, int], color: tuple[int, int, int]) -> None:
        length = max(abs(b[0] - a[0]), abs(b[1] - a[1])) + 1
        for fraction in np.linspace(0.0, 1.0, length):
            x = round(a[0] + fraction * (b[0] - a[0]))
            y = round(a[1] + fraction * (b[1] - a[1]))
            image[y, x] = color

    for segment in geometry.static_segments:
        draw_line(pixel(segment.start_xy), pixel(segment.end_xy), (20, 20, 20))
    for target in geometry.dynamic_targets:
        center = pixel(target.center_xy)
        radius = max(2, round(target.radius_m / (xmax - xmin) * config.width_px))
        for angle in np.linspace(0.0, 2.0 * math.pi, 48):
            draw_line(center, pixel((target.center_xy[0] + radius / config.width_px * (xmax - xmin) * math.cos(angle),
                                     target.center_xy[1] + radius / config.width_px * (ymax - ymin) * math.sin(angle))), (40, 80, 210))

    origin = pixel((pose.x_m, pose.y_m))
    count = len(observation.ranges_m)
    for index, (distance, valid) in enumerate(zip(observation.ranges_m, observation.valid_mask)):
        angle = pose.theta_rad - math.pi + 2.0 * math.pi * index / count
        length = min(distance if valid else config.ray_length_m, config.ray_length_m)
        endpoint = (pose.x_m + length * math.cos(angle), pose.y_m + length * math.sin(angle))
        draw_line(origin, pixel(endpoint), (35, 170, 70) if valid else (210, 60, 60))
    heading = (pose.x_m + 0.35 * math.cos(pose.theta_rad), pose.y_m + 0.35 * math.sin(pose.theta_rad))
    draw_line(origin, pixel(heading), (20, 80, 220))
    return image


def render_doom(observation: SensorObservation, config: DoomConfig = DoomConfig()) -> np.ndarray:
    """Return a deterministic RGB 2.5D projection from the frontal FOV."""
    if not observation.ranges_m:
        raise ValueError("observation must contain at least one beam")
    image = np.zeros((config.height_px, config.width_px, 3), dtype=np.uint8)
    horizon = config.height_px // 2
    image[:horizon] = (92, 128, 170)
    image[horizon:] = (70, 70, 70)
    count = len(observation.ranges_m)
    for column in range(config.width_px):
        fraction = (column + 0.5) / config.width_px
        relative = -config.fov_rad / 2.0 + fraction * config.fov_rad
        beam_fraction = (relative + math.pi) / (2.0 * math.pi)
        beam = min(count - 1, max(0, int(beam_fraction * count)))
        if not observation.valid_mask[beam]:
            continue
        distance = max(observation.ranges_m[beam], 1e-6)
        corrected = max(distance * math.cos(relative), 1e-6)
        wall_height = min(config.height_px, config.wall_height_m * config.focal_px / corrected)
        top = max(0, round(horizon - wall_height / 2.0))
        bottom = min(config.height_px, round(horizon + wall_height / 2.0))
        if observation.valid_mask[beam]:
            shade = max(35, min(220, round(220.0 / (1.0 + corrected / config.max_range_m))))
            image[top:bottom, column] = (shade, shade, shade)
    return image


def show_top_view(pose: Pose2D, geometry: WorldGeometry, observation: SensorObservation, config: TopViewConfig = TopViewConfig()) -> None:
    """Display a top-view image using optional Matplotlib."""
    import matplotlib.pyplot as plt
    plt.imshow(render_top_view(pose, geometry, observation, config))
    plt.axis("off")
    plt.show()


def show_doom(observation: SensorObservation, config: DoomConfig = DoomConfig()) -> None:
    """Display a Doom-style image using optional Matplotlib."""
    import matplotlib.pyplot as plt
    plt.imshow(render_doom(observation, config))
    plt.axis("off")
    plt.show()
