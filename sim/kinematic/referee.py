# sim/kinematic/referee.py
"""Truth-only referee boundary for deterministic kinematic scenarios."""

from dataclasses import dataclass
import math

from hsl_core.types import Pose2D

from .common import PlantState, WorldGeometry


@dataclass(frozen=True)
class RefereeResult:
    collision: bool
    collision_target_ids: tuple[str, ...]
    pose: Pose2D


class Referee:
    """The only public component allowed to inspect scenario truth."""

    def __init__(self, geometry: WorldGeometry, robot_radius_m: float) -> None:
        if robot_radius_m <= 0.0 or not math.isfinite(float(robot_radius_m)):
            raise ValueError("robot_radius_m must be positive and finite")
        self._geometry = geometry
        self._robot_radius = float(robot_radius_m)

    def evaluate(self, state: PlantState) -> RefereeResult:
        hits = []
        x, y = state.pose.x_m, state.pose.y_m
        for target in self._geometry.dynamic_targets:
            distance = math.hypot(x - target.center_xy[0], y - target.center_xy[1])
            if distance <= self._robot_radius + target.radius_m:
                hits.append(target.target_id)
        return RefereeResult(bool(hits), tuple(sorted(hits)), state.pose)
