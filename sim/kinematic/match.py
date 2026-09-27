"""Two-role kinematic SIL runner with a one-way private truth boundary."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Callable

from hsl_core.match import (
    EventEvidence,
    EventPredicate,
    MatchState,
    Role,
    RuleEvent,
    StageManager,
    TerminalKind,
)
from hsl_core.rules import CaptureEventInterval, TimedPose, first_capture_interval
from hsl_core.types import Pose2D

from .common import Actuation, CircleTarget, Segment, SensorObservation, WorldGeometry
from .plant import KinematicPlant
from .sensors import LidarSensor


class MatchRole(str, Enum):
    GUARDIAN = "guardian"
    EXPLORER = "explorer"


@dataclass(frozen=True)
class PolicyInput:
    """Role-scoped observation and optional leased stage state for policy."""

    role: MatchRole
    namespace: str
    stamp_ns: int
    observation: SensorObservation
    match_state: MatchState | None = None


Policy = Callable[[PolicyInput], Actuation]


@dataclass(frozen=True)
class RoleEndpoint:
    role: MatchRole
    namespace: str
    plant: KinematicPlant
    sensor: LidarSensor
    policy: Policy
    radius_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.role, MatchRole):
            raise ValueError("role must be a MatchRole")
        if self.namespace != f"/robot_{self.role.value}":
            raise ValueError("robot namespace must be role-specific and absolute")
        if not isinstance(self.plant, KinematicPlant) or not isinstance(self.sensor, LidarSensor):
            raise ValueError("role endpoint requires a kinematic plant and sensor")
        if not callable(self.policy):
            raise ValueError("role endpoint requires a policy callable")
        if (
            isinstance(self.radius_m, bool)
            or not isinstance(self.radius_m, (int, float))
            or not math.isfinite(float(self.radius_m))
            or self.radius_m <= 0.0
        ):
            raise ValueError("robot radius must be positive and finite")


@dataclass(frozen=True)
class RefereeTruth:
    """Evaluation-only truth; never passed to a role policy."""

    capture_interval: CaptureEventInterval | None
    robot_collision: bool


@dataclass(frozen=True)
class MatchTick:
    stamp_ns: int
    guardian_observation: SensorObservation
    explorer_observation: SensorObservation
    truth: RefereeTruth
    guardian_command: Actuation
    explorer_command: Actuation
    match_state: MatchState | None


def capture_rule_event(
    truth: RefereeTruth,
    *,
    event_id: str,
    stage_id: str,
    clock_epoch: str,
    localization_epoch: str,
    input_ids: tuple[str, ...],
    authorization_ref: str,
) -> RuleEvent | None:
    """Convert referee capture evidence without implying official authority."""
    if not isinstance(truth, RefereeTruth):
        raise ValueError("truth must be a RefereeTruth result")
    interval = truth.capture_interval
    if interval is None:
        return None
    return RuleEvent(
        event_id=event_id,
        stage_id=stage_id,
        clock_epoch=clock_epoch,
        localization_epoch=localization_epoch,
        kind=TerminalKind.CAPTURE,
        evidence=EventEvidence.SIM_TRUTH,
        predicate=EventPredicate.TRUE,
        event_time_lower_ns=interval.lower_ns,
        event_time_upper_ns=interval.upper_ns,
        input_ids=input_ids,
        authorization_ref=authorization_ref,
        reason="kinematic_referee_capture",
    )


class _TruthReferee:
    def __init__(
        self,
        geometry: WorldGeometry,
        guardian_radius_m: float,
        explorer_radius_m: float,
    ) -> None:
        self._geometry = geometry
        self._guardian_radius = guardian_radius_m
        self._explorer_radius = explorer_radius_m

    def evaluate(
        self,
        previous_ns: int,
        current_ns: int,
        previous_guardian: Pose2D,
        current_guardian: Pose2D,
        previous_explorer: Pose2D,
        current_explorer: Pose2D,
    ) -> RefereeTruth:
        guardian = (
            TimedPose(previous_ns, previous_guardian),
            TimedPose(current_ns, current_guardian),
        )
        explorer = (
            TimedPose(previous_ns, previous_explorer),
            TimedPose(current_ns, current_explorer),
        )
        capture = first_capture_interval(
            guardian,
            explorer,
            tuple((segment.start_xy, segment.end_xy) for segment in self._geometry.static_segments),
            circle_obstacles=tuple(
                (target.center_xy, target.radius_m)
                for target in self._geometry.dynamic_targets
            ),
        )
        start_distance = math.hypot(
            previous_guardian.x_m - previous_explorer.x_m,
            previous_guardian.y_m - previous_explorer.y_m,
        )
        end_distance = math.hypot(
            current_guardian.x_m - current_explorer.x_m,
            current_guardian.y_m - current_explorer.y_m,
        )
        relative_start = (
            previous_explorer.x_m - previous_guardian.x_m,
            previous_explorer.y_m - previous_guardian.y_m,
        )
        relative_delta = (
            (current_explorer.x_m - current_guardian.x_m) - relative_start[0],
            (current_explorer.y_m - current_guardian.y_m) - relative_start[1],
        )
        delta_sq = relative_delta[0] ** 2 + relative_delta[1] ** 2
        closest_fraction = 0.0 if delta_sq == 0.0 else max(
            0.0,
            min(
                1.0,
                -(
                    relative_start[0] * relative_delta[0]
                    + relative_start[1] * relative_delta[1]
                )
                / delta_sq,
            ),
        )
        closest_distance = math.hypot(
            relative_start[0] + closest_fraction * relative_delta[0],
            relative_start[1] + closest_fraction * relative_delta[1],
        )
        return RefereeTruth(
            capture,
            closest_distance <= self._guardian_radius + self._explorer_radius,
        )


class TwoRobotMatch:
    """Deterministic SIL stepper; referee output is separated from policy input."""

    def __init__(
        self,
        guardian: RoleEndpoint,
        explorer: RoleEndpoint,
        geometry: WorldGeometry,
        *,
        stage_manager: StageManager | None = None,
    ) -> None:
        if not isinstance(geometry, WorldGeometry):
            raise ValueError("match geometry must be a WorldGeometry")
        if any(not isinstance(segment, Segment) for segment in geometry.static_segments):
            raise ValueError("world static geometry must contain Segment values")
        if any(not isinstance(target, CircleTarget) for target in geometry.dynamic_targets):
            raise ValueError("world dynamic geometry must contain CircleTarget values")
        if guardian.role != MatchRole.GUARDIAN or explorer.role != MatchRole.EXPLORER:
            raise ValueError("match endpoints must provide one guardian and one explorer")
        if guardian.namespace == explorer.namespace:
            raise ValueError("robot namespaces must be unique")
        guardian_state = guardian.plant.state
        explorer_state = explorer.plant.state
        if guardian_state.stamp_s != explorer_state.stamp_s:
            raise ValueError("role plants must start on the same simulation clock")
        if round(guardian_state.stamp_s * 1_000_000_000) < 0:
            raise ValueError("initial simulation time must be non-negative")
        initial_distance = math.hypot(
            guardian_state.pose.x_m - explorer_state.pose.x_m,
            guardian_state.pose.y_m - explorer_state.pose.y_m,
        )
        if initial_distance <= guardian.radius_m + explorer.radius_m:
            raise ValueError("role footprints overlap at match initialization")
        self.guardian = guardian
        self.explorer = explorer
        self._geometry = geometry
        if stage_manager is not None and not isinstance(stage_manager, StageManager):
            raise ValueError("stage_manager must be a StageManager")
        self._stage_manager = stage_manager
        self._referee = _TruthReferee(geometry, guardian.radius_m, explorer.radius_m)
        self._stamp_ns = round(guardian_state.stamp_s * 1_000_000_000)

    @property
    def stamp_ns(self) -> int:
        return self._stamp_ns

    def step(self, dt_s: float, *, watchdog_healthy: bool = True) -> MatchTick:
        if (
            isinstance(dt_s, bool)
            or not isinstance(dt_s, (int, float))
            or not math.isfinite(float(dt_s))
            or dt_s <= 0.0
        ):
            raise ValueError("dt_s must be positive and finite")
        if not isinstance(watchdog_healthy, bool):
            raise ValueError("watchdog_healthy must be boolean")
        dt_ns = round(float(dt_s) * 1_000_000_000)
        if dt_ns <= 0:
            raise ValueError("dt_s must resolve to at least one nanosecond")
        start_ns = self._stamp_ns
        end_ns = start_ns + dt_ns
        if end_ns > (1 << 63) - 1:
            raise ValueError("simulation clock exceeds supported nanosecond range")
        guardian_state = self.guardian.plant.state
        explorer_state = self.explorer.plant.state
        if (
            round(guardian_state.stamp_s * 1_000_000_000) != start_ns
            or round(explorer_state.stamp_s * 1_000_000_000) != start_ns
        ):
            raise ValueError("role plant clock diverged from match clock")
        match_state = (
            self._stage_manager.snapshot(now_ns=start_ns)
            if self._stage_manager is not None
            else None
        )
        guardian_geometry = WorldGeometry(
            self._geometry.static_segments,
            self._geometry.dynamic_targets
            + (CircleTarget(
                (explorer_state.pose.x_m, explorer_state.pose.y_m),
                self.explorer.radius_m,
                "opponent",
            ),),
        )
        explorer_geometry = WorldGeometry(
            self._geometry.static_segments,
            self._geometry.dynamic_targets
            + (CircleTarget(
                (guardian_state.pose.x_m, guardian_state.pose.y_m),
                self.guardian.radius_m,
                "opponent",
            ),),
        )
        guardian_observation = self.guardian.sensor.observe(
            guardian_state.pose, start_ns / 1_000_000_000, guardian_geometry
        )
        explorer_observation = self.explorer.sensor.observe(
            explorer_state.pose, start_ns / 1_000_000_000, explorer_geometry
        )
        if watchdog_healthy:
            guardian_command = self.guardian.policy(
                PolicyInput(
                    MatchRole.GUARDIAN,
                    self.guardian.namespace,
                    start_ns,
                    guardian_observation,
                    match_state,
                )
            )
            explorer_command = self.explorer.policy(
                PolicyInput(
                    MatchRole.EXPLORER,
                    self.explorer.namespace,
                    start_ns,
                    explorer_observation,
                    match_state,
                )
            )
        else:
            guardian_command = Actuation(0.0, 0.0)
            explorer_command = Actuation(0.0, 0.0)
        if not isinstance(guardian_command, Actuation) or not isinstance(explorer_command, Actuation):
            raise ValueError("role policies must return Actuation values")
        stage_step_authorized = (
            match_state is not None
            and match_state.motion_authorized
            and match_state.meta.valid_until_ns >= end_ns
            and end_ns <= match_state.stage_ends_at_ns
        )
        if self._stage_manager is not None and not stage_step_authorized:
            guardian_command = Actuation(0.0, 0.0)
            explorer_command = Actuation(0.0, 0.0)
        elif match_state is not None and match_state.role != Role.GUARDIAN:
            guardian_command = Actuation(0.0, 0.0)
        if (
            match_state is not None
            and stage_step_authorized
            and match_state.role != Role.EXPLORER
        ):
            explorer_command = Actuation(0.0, 0.0)
        guardian_next_pose = self.guardian.plant.predict_step(guardian_command, dt_s)
        explorer_next_pose = self.explorer.plant.predict_step(explorer_command, dt_s)
        truth = self._referee.evaluate(
            start_ns,
            end_ns,
            guardian_state.pose,
            guardian_next_pose,
            explorer_state.pose,
            explorer_next_pose,
        )
        self.guardian.plant.step(guardian_command, dt_s)
        self.explorer.plant.step(explorer_command, dt_s)
        self._stamp_ns = end_ns
        return MatchTick(
            end_ns,
            guardian_observation,
            explorer_observation,
            truth,
            guardian_command,
            explorer_command,
            match_state,
        )
