# sim/kinematic/plant.py
"""Deterministic unicycle plant; it never exposes or stores referee truth."""

import math

from hsl_core.types import Pose2D

from .common import Actuation, PlantState


class KinematicPlant:
    """Integrate accepted commands with a bounded, exact unicycle step."""

    def __init__(
        self,
        initial_pose: Pose2D,
        *,
        max_linear_mps: float = 1.0,
        max_angular_rps: float = 4.0,
    ) -> None:
        if max_linear_mps <= 0.0 or max_angular_rps <= 0.0:
            raise ValueError("plant limits must be positive")
        self._pose = initial_pose
        self._stamp_s = 0.0
        self._max_linear = float(max_linear_mps)
        self._max_angular = float(max_angular_rps)

    @property
    def state(self) -> PlantState:
        return PlantState(self._stamp_s, self._pose)

    def step(self, command: Actuation, dt_s: float) -> PlantState:
        if not math.isfinite(float(dt_s)) or dt_s <= 0.0:
            raise ValueError("dt_s must be positive and finite")
        if abs(command.linear_mps) > self._max_linear:
            raise ValueError("linear command exceeds plant limit")
        if abs(command.angular_rps) > self._max_angular:
            raise ValueError("angular command exceeds plant limit")

        theta = self._pose.theta_rad
        omega = command.angular_rps
        if abs(omega) < 1e-12:
            dx = command.linear_mps * dt_s * math.cos(theta)
            dy = command.linear_mps * dt_s * math.sin(theta)
        else:
            radius = command.linear_mps / omega
            theta_next = theta + omega * dt_s
            dx = radius * (math.sin(theta_next) - math.sin(theta))
            dy = radius * (-math.cos(theta_next) + math.cos(theta))
        self._pose = Pose2D(
            self._pose.x_m + dx,
            self._pose.y_m + dy,
            theta + omega * dt_s,
        )
        self._stamp_s += dt_s
        return self.state

