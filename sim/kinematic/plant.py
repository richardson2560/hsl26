# sim/kinematic/plant.py
"""Deterministic unicycle plant; it never exposes or stores referee truth."""

import math

from hsl_core.types import Pose2D

from .common import Actuation, PlantState, PoseEstimate


def integrate_unicycle(pose: Pose2D, command: Actuation, dt_s: float) -> Pose2D:
    theta = pose.theta_rad
    omega = command.angular_rps
    if abs(omega) < 1e-12:
        dx = command.linear_mps * dt_s * math.cos(theta)
        dy = command.linear_mps * dt_s * math.sin(theta)
    else:
        radius = command.linear_mps / omega
        theta_next = theta + omega * dt_s
        dx = radius * (math.sin(theta_next) - math.sin(theta))
        dy = radius * (-math.cos(theta_next) + math.cos(theta))
    return Pose2D(pose.x_m + dx, pose.y_m + dy, theta + omega * dt_s)


class SilDeadReckoningEstimator:
    """Deterministic SIL odometry with explicit, monotonically growing bounds."""

    def __init__(
        self,
        initial: PoseEstimate,
        *,
        position_drift_bound_per_m: float = 0.0,
        yaw_drift_bound_per_rad: float = 0.0,
    ) -> None:
        if not isinstance(initial, PoseEstimate):
            raise ValueError("initial must be a PoseEstimate")
        for value, name in (
            (position_drift_bound_per_m, "position_drift_bound_per_m"),
            (yaw_drift_bound_per_rad, "yaw_drift_bound_per_rad"),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        self._estimate = initial
        self._position_drift_bound_per_m = float(position_drift_bound_per_m)
        self._yaw_drift_bound_per_rad = float(yaw_drift_bound_per_rad)

    @property
    def estimate(self) -> PoseEstimate:
        return self._estimate

    def predict(self, command: Actuation, dt_s: float) -> PoseEstimate:
        if not isinstance(command, Actuation):
            raise ValueError("command must be an Actuation")
        if (
            isinstance(dt_s, bool)
            or not isinstance(dt_s, (int, float))
            or not math.isfinite(float(dt_s))
            or dt_s <= 0.0
        ):
            raise ValueError("dt_s must be positive and finite")
        dt_ns = round(float(dt_s) * 1_000_000_000)
        if dt_ns <= 0:
            raise ValueError("dt_s must resolve to at least one nanosecond")
        old = self._estimate
        return PoseEstimate(
            stamp_ns=old.stamp_ns + dt_ns,
            frame_id=old.frame_id,
            clock_epoch=old.clock_epoch,
            localization_epoch=old.localization_epoch,
            pose=integrate_unicycle(old.pose, command, float(dt_s)),
            position_error_bound_m=(
                old.position_error_bound_m
                + abs(command.linear_mps * float(dt_s))
                * self._position_drift_bound_per_m
            ),
            yaw_error_bound_rad=(
                old.yaw_error_bound_rad
                + abs(command.angular_rps * float(dt_s))
                * self._yaw_drift_bound_per_rad
            ),
            linear_velocity_mps=command.linear_mps,
            angular_velocity_rps=command.angular_rps,
        )

    def commit(self, estimate: PoseEstimate) -> None:
        if not isinstance(estimate, PoseEstimate):
            raise ValueError("estimate must be a PoseEstimate")
        if estimate.stamp_ns <= self._estimate.stamp_ns:
            raise ValueError("pose estimate time must advance monotonically")
        if (
            estimate.frame_id != self._estimate.frame_id
            or estimate.clock_epoch != self._estimate.clock_epoch
            or estimate.localization_epoch != self._estimate.localization_epoch
        ):
            raise ValueError("pose estimate identity cannot change without reset")
        self._estimate = estimate


class KinematicPlant:
    """Integrate accepted commands with a bounded, exact unicycle step."""

    def __init__(
        self,
        initial_pose: Pose2D,
        *,
        max_linear_mps: float = 1.0,
        max_angular_rps: float = 4.0,
    ) -> None:
        if (
            isinstance(max_linear_mps, bool)
            or not math.isfinite(float(max_linear_mps))
            or max_linear_mps <= 0.0
            or isinstance(max_angular_rps, bool)
            or not math.isfinite(float(max_angular_rps))
            or max_angular_rps <= 0.0
        ):
            raise ValueError("plant limits must be positive and finite")
        self._pose = initial_pose
        self._stamp_s = 0.0
        self._max_linear = float(max_linear_mps)
        self._max_angular = float(max_angular_rps)

    @property
    def state(self) -> PlantState:
        return PlantState(self._stamp_s, self._pose)

    def validate_step(self, command: Actuation, dt_s: float) -> None:
        self.predict_step(command, dt_s)

    def predict_step(self, command: Actuation, dt_s: float) -> Pose2D:
        """Validate and return the next pose without committing plant state."""
        if not isinstance(command, Actuation):
            raise ValueError("command must be an Actuation")
        if (
            isinstance(dt_s, bool)
            or not isinstance(dt_s, (int, float))
            or not math.isfinite(float(dt_s))
            or dt_s <= 0.0
        ):
            raise ValueError("dt_s must be positive and finite")
        if abs(command.linear_mps) > self._max_linear:
            raise ValueError("linear command exceeds plant limit")
        if abs(command.angular_rps) > self._max_angular:
            raise ValueError("angular command exceeds plant limit")
        if not math.isfinite(self._stamp_s + float(dt_s)):
            raise ValueError("plant time exceeds finite range")
        return integrate_unicycle(self._pose, command, float(dt_s))

    def step(self, command: Actuation, dt_s: float) -> PlantState:
        next_pose = self.predict_step(command, dt_s)
        self._pose = next_pose
        self._stamp_s += dt_s
        return self.state
