# sim/kinematic/sensors.py
"""Observation-only deterministic LiDAR adapter."""

import math
import random

from hsl_core.types import Pose2D

from .common import SensorObservation, WorldGeometry
from .raycaster import FirstHitRaycaster


class LidarSensor:
    def __init__(
        self,
        *,
        raycaster: FirstHitRaycaster,
        beam_count: int = 360,
        max_range_m: float = 10.0,
        noise_std_m: float = 0.0,
        seed: int = 0,
        blind_sectors: tuple[tuple[float, float], ...] = (),
        frame_id: str = "lidar_link",
    ) -> None:
        if beam_count <= 0 or max_range_m <= 0.0 or noise_std_m < 0.0:
            raise ValueError("sensor parameters are out of range")
        if not frame_id:
            raise ValueError("frame_id must not be empty")
        self._raycaster = raycaster
        self._beam_count = int(beam_count)
        self._max_range = float(max_range_m)
        self._noise_std = float(noise_std_m)
        self._rng = random.Random(seed)
        self._blind_sectors = tuple(blind_sectors)
        self._frame_id = frame_id

    def observe(
        self,
        pose: Pose2D,
        stamp_s: float,
        geometry: WorldGeometry,
    ) -> SensorObservation:
        if not math.isfinite(float(stamp_s)):
            raise ValueError("stamp_s must be finite")
        ranges = []
        valid = []
        for index in range(self._beam_count):
            local_angle = -math.pi + 2.0 * math.pi * index / self._beam_count
            world_angle = pose.theta_rad + local_angle
            blind = any(
                self._raycaster.in_blind_sector(local_angle, center, width)
                for center, width in self._blind_sectors
            )
            distance = None if blind else self._raycaster.cast(
                (pose.x_m, pose.y_m),
                world_angle,
                geometry.static_segments,
                geometry.dynamic_targets,
            )
            if distance is None:
                ranges.append(self._max_range)
                valid.append(False)
            else:
                measured = max(0.0, min(self._max_range, distance + self._rng.gauss(0.0, self._noise_std)))
                ranges.append(measured)
                valid.append(True)
        return SensorObservation(stamp_s, self._frame_id, tuple(ranges), tuple(valid))

