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
from hsl_core.topology import TopologyGraph
from hsl_core.rules import CaptureEventInterval, TimedPose, first_capture_interval
from hsl_core.types import Pose2D

from .common import Actuation, CircleTarget, PoseEstimate, Segment, SensorObservation, WorldGeometry
from .plant import KinematicPlant, SilDeadReckoningEstimator
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
    pose_estimate: PoseEstimate | None = None
    topology_graph: TopologyGraph | None = None
    robot_radius_m: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.role, MatchRole):
            raise ValueError("policy input role must be a MatchRole")
        if isinstance(self.stamp_ns, bool) or not isinstance(self.stamp_ns, int) or self.stamp_ns < 0:
            raise ValueError("policy input stamp must be a non-negative integer")
        if not isinstance(self.observation, SensorObservation):
            raise ValueError("policy input requires a SensorObservation")
        if self.match_state is not None and not isinstance(self.match_state, MatchState):
            raise ValueError("match_state must be a validated MatchState or None")
        if self.pose_estimate is not None and not isinstance(self.pose_estimate, PoseEstimate):
            raise ValueError("pose_estimate must be a PoseEstimate or None")
        if self.topology_graph is not None and not isinstance(self.topology_graph, TopologyGraph):
            raise ValueError("topology_graph must be a TopologyGraph or None")
        if self.robot_radius_m is not None and (
            isinstance(self.robot_radius_m, bool)
            or not isinstance(self.robot_radius_m, (int, float))
            or not math.isfinite(self.robot_radius_m)
            or self.robot_radius_m <= 0.0
        ):
            raise ValueError("robot_radius_m must be finite and positive when supplied")


@dataclass(frozen=True)
class SILPolicyInput(PolicyInput):
    """Fixture-only geometry context used by the kinematic SIL policies."""

    opponent_radius_m: float = 0.15
    static_segments: tuple[Segment, ...] = ()
    synthetic_goal_node_id: int | None = None
    synthetic_goal_zone_id: str = ""
    opponent_speed_bound_mps: float = 0.0
    simultaneous_sil: bool = False

    def __post_init__(self) -> None:
        super().__post_init__()
        if (
            isinstance(self.opponent_radius_m, bool)
            or not isinstance(self.opponent_radius_m, (int, float))
            or not math.isfinite(self.opponent_radius_m)
            or self.opponent_radius_m <= 0.0
        ):
            raise ValueError("opponent_radius_m must be finite and positive")
        if not isinstance(self.static_segments, tuple) or any(
            not isinstance(segment, Segment) for segment in self.static_segments
        ):
            raise ValueError("static_segments must be a tuple of Segment values")
        if self.synthetic_goal_node_id is not None and (
            not isinstance(self.synthetic_goal_node_id, int)
            or isinstance(self.synthetic_goal_node_id, bool)
            or self.synthetic_goal_node_id < 0
        ):
            raise ValueError("synthetic_goal_node_id must be non-negative")
        if not isinstance(self.synthetic_goal_zone_id, str):
            raise ValueError("synthetic_goal_zone_id must be a string")
        if not isinstance(self.simultaneous_sil, bool):
            raise ValueError("simultaneous_sil must be boolean")
        if self.simultaneous_sil and self.role == MatchRole.EXPLORER and (
            self.synthetic_goal_node_id is None
            or not self.synthetic_goal_zone_id.startswith("sil-fixture:")
        ):
            raise ValueError("simultaneous SIL explorer goal must be fixture-scoped")
        if (
            isinstance(self.opponent_speed_bound_mps, bool)
            or not isinstance(self.opponent_speed_bound_mps, (int, float))
            or not math.isfinite(self.opponent_speed_bound_mps)
            or self.opponent_speed_bound_mps < 0.0
        ):
            raise ValueError("opponent_speed_bound_mps must be finite and non-negative")


Policy = Callable[[PolicyInput], Actuation]


@dataclass(frozen=True)
class RoleEndpoint:
    role: MatchRole
    namespace: str
    plant: KinematicPlant
    sensor: LidarSensor
    policy: Policy
    radius_m: float
    pose_estimator: SilDeadReckoningEstimator | None = None
    topology_graph: TopologyGraph | None = None
    synthetic_goal_node_id: int | None = None
    synthetic_goal_zone_id: str = ""

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
            self.pose_estimator is not None
            and not isinstance(self.pose_estimator, SilDeadReckoningEstimator)
        ):
            raise ValueError("pose_estimator must be a SilDeadReckoningEstimator")
        if self.topology_graph is not None and not isinstance(self.topology_graph, TopologyGraph):
            raise ValueError("topology_graph must be a versioned TopologyGraph")
        if self.synthetic_goal_node_id is not None:
            if (
                not isinstance(self.synthetic_goal_node_id, int)
                or isinstance(self.synthetic_goal_node_id, bool)
                or self.synthetic_goal_node_id < 0
            ):
                raise ValueError("synthetic_goal_node_id must be non-negative")
            if self.topology_graph is None or self.synthetic_goal_node_id not in {
                node.node_id for node in self.topology_graph.nodes
            }:
                raise ValueError("synthetic goal must identify a node in the endpoint topology")
        if self.synthetic_goal_zone_id and not self.synthetic_goal_zone_id.startswith(
            "sil-fixture:"
        ):
            raise ValueError("synthetic goal zone IDs must be explicitly SIL-fixture scoped")
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
    role_match_states: tuple[MatchState, MatchState] | None = None


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
        duel_stage_managers: tuple[StageManager, StageManager] | None = None,
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
        if duel_stage_managers is not None:
            if stage_manager is not None:
                raise ValueError("single-stage and simultaneous SIL stage managers are exclusive")
            if (
                not isinstance(duel_stage_managers, tuple)
                or len(duel_stage_managers) != 2
                or any(not isinstance(manager, StageManager) for manager in duel_stage_managers)
            ):
                raise ValueError("duel_stage_managers must contain guardian and explorer managers")
            guardian_manager, explorer_manager = duel_stage_managers
            if (
                guardian_manager is explorer_manager
                or guardian_manager.role != Role.GUARDIAN
                or explorer_manager.role != Role.EXPLORER
            ):
                raise ValueError("simultaneous SIL requires independent role-scoped managers")
            if explorer.synthetic_goal_node_id is None or not explorer.synthetic_goal_zone_id:
                raise ValueError(
                    "simultaneous SIL requires an explicit explorer synthetic goal fixture"
                )
        self._stage_manager = stage_manager
        self._duel_stage_managers = duel_stage_managers
        self._referee = _TruthReferee(geometry, guardian.radius_m, explorer.radius_m)
        self._stamp_ns = round(guardian_state.stamp_s * 1_000_000_000)
        topology_snapshots = tuple(
            endpoint.topology_graph
            for endpoint in (guardian, explorer)
            if endpoint.topology_graph is not None
        )
        if len(topology_snapshots) == 2:
            first_graph, second_graph = topology_snapshots
            if (
                first_graph.map_version != second_graph.map_version
                or first_graph.topology_version != second_graph.topology_version
                or first_graph.localization_epoch != second_graph.localization_epoch
            ):
                raise ValueError("P5.6 role endpoints require one coherent topology identity")
            self._map_version = first_graph.map_version
            self._topology_version = first_graph.topology_version
        else:
            self._map_version = 0
            self._topology_version = 0
        for endpoint in (guardian, explorer):
            if endpoint.pose_estimator is not None:
                estimate = endpoint.pose_estimator.estimate
                if estimate.stamp_ns != self._stamp_ns:
                    raise ValueError("pose estimator clock must match the plant clock")

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
        if self._duel_stage_managers is not None:
            guardian_match_state, explorer_match_state = tuple(
                manager.snapshot(
                    now_ns=start_ns,
                    map_version=self._map_version,
                    topology_version=self._topology_version,
                )
                for manager in self._duel_stage_managers
            )
            self._validate_duel_states(guardian_match_state, explorer_match_state)
            match_state = None
            role_match_states = (guardian_match_state, explorer_match_state)
        else:
            match_state = (
                self._stage_manager.snapshot(
                now_ns=start_ns,
                map_version=self._map_version,
                topology_version=self._topology_version,
            )
                if self._stage_manager is not None
                else None
            )
            role_match_states = (
                (match_state, match_state) if match_state is not None else None
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
            guardian_policy_state = (
                role_match_states[0] if role_match_states is not None else match_state
            )
            explorer_policy_state = (
                role_match_states[1] if role_match_states is not None else match_state
            )

            def policy_input(endpoint, observation, state, opponent):
                common = dict(
                    role=endpoint.role,
                    namespace=endpoint.namespace,
                    stamp_ns=start_ns,
                    observation=observation,
                    match_state=state,
                    pose_estimate=(
                        endpoint.pose_estimator.estimate
                        if endpoint.pose_estimator is not None
                        else None
                    ),
                    topology_graph=endpoint.topology_graph,
                    robot_radius_m=endpoint.radius_m,
                )
                if (
                    endpoint.topology_graph is None
                    or endpoint.pose_estimator is None
                ):
                    return PolicyInput(**common)
                return SILPolicyInput(
                    **common,
                    opponent_radius_m=opponent.radius_m,
                    static_segments=self._geometry.static_segments,
                    synthetic_goal_node_id=endpoint.synthetic_goal_node_id,
                    synthetic_goal_zone_id=endpoint.synthetic_goal_zone_id,
                    opponent_speed_bound_mps=opponent.plant.max_linear_mps,
                    simultaneous_sil=self._duel_stage_managers is not None,
                )

            guardian_command = self.guardian.policy(
                policy_input(
                    self.guardian,
                    guardian_observation,
                    guardian_policy_state,
                    self.explorer,
                )
            )
            explorer_command = self.explorer.policy(
                policy_input(
                    self.explorer,
                    explorer_observation,
                    explorer_policy_state,
                    self.guardian,
                )
            )
        else:
            guardian_command = Actuation(0.0, 0.0)
            explorer_command = Actuation(0.0, 0.0)
        if not isinstance(guardian_command, Actuation) or not isinstance(explorer_command, Actuation):
            raise ValueError("role policies must return Actuation values")
        if self._duel_stage_managers is not None:
            duel_authorized = (
                role_match_states is not None
                and all(
                    state.phase.name == "ACTIVE"
                    and state.motion_authorized
                    and not state.event_hold
                    and state.meta.valid_until_ns >= end_ns
                    and end_ns <= state.stage_ends_at_ns
                    for state in role_match_states
                )
            )
            if not duel_authorized:
                guardian_command = Actuation(0.0, 0.0)
                explorer_command = Actuation(0.0, 0.0)
        else:
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
        guardian_pose_estimate = (
            self.guardian.pose_estimator.predict(guardian_command, dt_s)
            if self.guardian.pose_estimator is not None
            else None
        )
        explorer_pose_estimate = (
            self.explorer.pose_estimator.predict(explorer_command, dt_s)
            if self.explorer.pose_estimator is not None
            else None
        )
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
        if guardian_pose_estimate is not None:
            self.guardian.pose_estimator.commit(guardian_pose_estimate)
        if explorer_pose_estimate is not None:
            self.explorer.pose_estimator.commit(explorer_pose_estimate)
        self._stamp_ns = end_ns
        return MatchTick(
            end_ns,
            guardian_observation,
            explorer_observation,
            truth,
            guardian_command,
            explorer_command,
            match_state,
            role_match_states,
        )

    @staticmethod
    def _validate_duel_states(
        guardian: MatchState, explorer: MatchState
    ) -> None:
        if guardian.role != Role.GUARDIAN or explorer.role != Role.EXPLORER:
            raise ValueError("simultaneous SIL stage snapshots must remain role-scoped")
        if (
            guardian.meta.stage_id != explorer.meta.stage_id
            or guardian.meta.clock_epoch != explorer.meta.clock_epoch
            or guardian.meta.localization_epoch != explorer.meta.localization_epoch
            or guardian.meta.map_version != explorer.meta.map_version
            or guardian.meta.topology_version != explorer.meta.topology_version
            or guardian.phase != explorer.phase
            or guardian.stage_started_at_ns != explorer.stage_started_at_ns
            or guardian.freeze_ends_at_ns != explorer.freeze_ends_at_ns
            or guardian.stage_ends_at_ns != explorer.stage_ends_at_ns
            or guardian.event_hold != explorer.event_hold
            or guardian.terminal_kind != explorer.terminal_kind
        ):
            raise ValueError("simultaneous SIL role stages must have coherent lifecycle state")
