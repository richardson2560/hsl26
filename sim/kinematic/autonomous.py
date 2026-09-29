"""Fail-closed role policy composition for the bounded P5.6 SIL profile."""

from dataclasses import dataclass, replace
import math

from hsl_core.control import PursuitConfig, make_candidate
from hsl_core.control.regulated_pursuit import lookahead_point
from hsl_core.control.safety import (
    ADMIT,
    ControlMode,
    LIMIT,
    LimitsProfile,
    SafetySnapshot,
    SafetySupervisor,
    STOP,
    TimingProfile,
)
from hsl_core.match import Role, StagePhase
from hsl_core.perception.ekf_opponent import FilterConfig, OpponentFilter
from hsl_core.planning import CandidateEnvelope, PlannedPath, astar
from hsl_core.tactics import (
    OptionAuthority,
    OptionContext,
    OptionGoal,
    OptionKind,
    OptionProposal,
    OptionRegistry,
    MAX_TACTICAL_PROPOSALS,
    ROLE_FEATURES,
    TacticalGuard,
    TacticalSelector,
    TacticalSnapshot,
    UtilityProfile,
)
from hsl_core.topology import EdgeState, NodeKind, TopologyGraph
from hsl_core.types import MotionCandidate, OpponentTrack, Pose2D, TrackState

from .common import Actuation, PoseEstimate
from .match import MatchRole, PolicyInput, SILPolicyInput
from .perception import extract_synthetic_opponent, line_of_sight_clear


def _core_role(role: MatchRole) -> Role:
    return Role.GUARDIAN if role == MatchRole.GUARDIAN else Role.EXPLORER


@dataclass(frozen=True)
class P56Profile:
    """Explicit software-only parameters; none are physical calibrations."""

    profile_id: str = "p56-kinematic-fixture-v1"
    parameters_id: str = "p56-kinematic-parameters-v1"
    sensor_frame_id: str = "lidar_link"
    localization_frame_id: str = "map"
    forward_half_angle_rad: float = 0.12
    minimum_observation_coverage: float = 0.75
    pose_error_limit_m: float = 0.10
    yaw_error_limit_rad: float = 0.10
    robot_radius_m: float = 0.15
    ego_yaw_rate_tolerance_rps: float = 0.50
    candidate_lease_s: float = 0.10
    speed_max_mps: float = 0.20
    planner_yaw_rate_max_rps: float = 1.5
    wheel_separation_m: float = 0.23
    wheel_radius_m: float = 0.035
    max_wheel_rate_radps: float = 12.0
    minimum_braking_deceleration_mps2: float = 0.85
    response_bound_s: float = 0.075
    clearance_margin_m: float = 0.20
    processing_budget_s: float = 0.01
    rotation_radius_m: float = 0.20
    rotation_clearance_margin_m: float = 0.05
    linear_rest_tolerance_mps: float = 0.005
    align_yaw_rate_max_rps: float = 0.50
    angular_acceleration_max_rps2: float = 0.50
    align_yaw_tolerance_rad: float = 0.05
    rotation_minimum_scan_beams: int = 360
    rotation_range_error_bound_m: float = 0.0
    control_period_s: float = 0.05
    lookahead_m: float = 0.75
    lateral_acceleration_max_mps2: float = 0.5
    goal_tolerance_m: float = 0.10
    curvature_range_error_bound_m: float = 0.0
    curvature_minimum_scan_beams: int = 360
    curvature_sample_step_m: float = 0.02
    smooth_turn_heading_limit_rad: float = 0.6
    opponent_threat_distance_m: float = 1.5

    def __post_init__(self) -> None:
        if not self.profile_id.strip() or not self.parameters_id.strip():
            raise ValueError("P5.6 profile identifiers must not be empty")
        if not self.sensor_frame_id or not self.localization_frame_id:
            raise ValueError("P5.6 frame identifiers must not be empty")
        for value, name in (
            (self.forward_half_angle_rad, "forward_half_angle_rad"),
            (self.minimum_observation_coverage, "minimum_observation_coverage"),
            (self.pose_error_limit_m, "pose_error_limit_m"),
            (self.yaw_error_limit_rad, "yaw_error_limit_rad"),
            (self.robot_radius_m, "robot_radius_m"),
            (self.candidate_lease_s, "candidate_lease_s"),
            (self.speed_max_mps, "speed_max_mps"),
            (self.planner_yaw_rate_max_rps, "planner_yaw_rate_max_rps"),
            (self.wheel_separation_m, "wheel_separation_m"),
            (self.wheel_radius_m, "wheel_radius_m"),
            (self.max_wheel_rate_radps, "max_wheel_rate_radps"),
            (self.minimum_braking_deceleration_mps2, "minimum_braking_deceleration_mps2"),
            (self.response_bound_s, "response_bound_s"),
            (self.clearance_margin_m, "clearance_margin_m"),
            (self.processing_budget_s, "processing_budget_s"),
            (self.rotation_radius_m, "rotation_radius_m"),
            (self.rotation_clearance_margin_m, "rotation_clearance_margin_m"),
            (self.linear_rest_tolerance_mps, "linear_rest_tolerance_mps"),
            (self.align_yaw_rate_max_rps, "align_yaw_rate_max_rps"),
            (self.angular_acceleration_max_rps2, "angular_acceleration_max_rps2"),
            (self.align_yaw_tolerance_rad, "align_yaw_tolerance_rad"),
            (self.control_period_s, "control_period_s"),
            (self.lookahead_m, "lookahead_m"),
            (self.lateral_acceleration_max_mps2, "lateral_acceleration_max_mps2"),
            (self.goal_tolerance_m, "goal_tolerance_m"),
            (self.curvature_sample_step_m, "curvature_sample_step_m"),
            (self.smooth_turn_heading_limit_rad, "smooth_turn_heading_limit_rad"),
            (self.opponent_threat_distance_m, "opponent_threat_distance_m"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.minimum_observation_coverage > 1.0:
            raise ValueError("minimum_observation_coverage must not exceed one")
        if self.forward_half_angle_rad > math.pi:
            raise ValueError("forward_half_angle_rad must not exceed pi")
        if self.clearance_margin_m < self.robot_radius_m:
            raise ValueError("clearance_margin_m must include at least the robot radius")
        if (
            not isinstance(self.rotation_minimum_scan_beams, int)
            or isinstance(self.rotation_minimum_scan_beams, bool)
            or self.rotation_minimum_scan_beams < 4
        ):
            raise ValueError("rotation_minimum_scan_beams must be an integer >= 4")
        if (
            not isinstance(self.curvature_minimum_scan_beams, int)
            or isinstance(self.curvature_minimum_scan_beams, bool)
            or self.curvature_minimum_scan_beams < 4
        ):
            raise ValueError("curvature_minimum_scan_beams must be an integer >= 4")
        if (
            not math.isfinite(self.ego_yaw_rate_tolerance_rps)
            or self.ego_yaw_rate_tolerance_rps < 0.0
        ):
            raise ValueError("ego_yaw_rate_tolerance_rps must be finite and non-negative")
        if (
            not math.isfinite(self.rotation_range_error_bound_m)
            or self.rotation_range_error_bound_m < 0.0
        ):
            raise ValueError("rotation_range_error_bound_m must be finite and non-negative")
        if (
            not math.isfinite(self.curvature_range_error_bound_m)
            or self.curvature_range_error_bound_m < 0.0
        ):
            raise ValueError("curvature_range_error_bound_m must be finite and non-negative")


@dataclass(frozen=True)
class PolicyCycleTrace:
    status: str
    selected_kind: OptionKind | None
    selection_reason: str
    authority_reason: str
    lease_generation: int
    candidate_v_mps: float
    candidate_omega_rps: float
    safety_decision: int | None
    safety_reason: str
    command: Actuation
    effect_evidence_id: str


def _utility_profile(role: Role) -> UtilityProfile:
    weights = (
        {
            "capture_opportunity": 0.40,
            "portal_time_advantage": 0.25,
            "observation_gain": 0.15,
            "pursuit_value": 0.10,
            "duration_cost": -0.10,
        }
        if role == Role.GUARDIAN
        else {
            "base_progress": 0.35,
            "visibility_loss": 0.20,
            "alternative_exits": 0.15,
            "escape_safety": 0.15,
            "observation_gain": 0.05,
            "capture_risk": -0.05,
            "duration_cost": -0.05,
        }
    )
    return UtilityProfile(
        f"{role.name.lower()}-p56-fixture-utility-v1",
        role,
        tuple(weights.items()),
        1,
    )


class KinematicAutonomousPolicy:
    """Observation/authority/plan/safety chain; never receives referee truth."""

    def __init__(
        self,
        role: MatchRole,
        *,
        profile: P56Profile = P56Profile(),
        supervisor: SafetySupervisor | None = None,
        utility_profile: UtilityProfile | None = None,
        hysteresis_delta_u: float = 0.0,
        minimum_dwell_ns: int = 0,
    ) -> None:
        if not isinstance(role, MatchRole):
            raise ValueError("role must be a MatchRole")
        if not isinstance(profile, P56Profile):
            raise ValueError("profile must be a P56Profile")
        self.role = role
        core_role = _core_role(role)
        self.profile = profile
        selected_utility = utility_profile or _utility_profile(core_role)
        if selected_utility.role is not core_role:
            raise ValueError("utility profile role does not match policy endpoint")
        self.registry = OptionRegistry()
        self.authority = OptionAuthority(self.registry)
        self.selector = TacticalSelector(
            selected_utility,
            registry=self.registry,
            hysteresis_delta_u=hysteresis_delta_u,
            minimum_dwell_ns=minimum_dwell_ns,
            max_proposals=MAX_TACTICAL_PROPOSALS,
        )
        self.supervisor = supervisor or SafetySupervisor(
            LimitsProfile(
                limits_id="p56-sil-model-limits",
                calibration_id="not-a-hardware-calibration",
                wheel_separation_m=profile.wheel_separation_m,
                wheel_radius_m=profile.wheel_radius_m,
                max_wheel_rate_radps=profile.max_wheel_rate_radps,
                speed_max_mps=profile.speed_max_mps,
                yaw_rate_max_rps=max(
                    profile.align_yaw_rate_max_rps,
                    profile.planner_yaw_rate_max_rps,
                ),
                b_forward_min_mps2=profile.minimum_braking_deceleration_mps2,
                response_bound_s=profile.response_bound_s,
                clearance_margin_m=profile.clearance_margin_m,
                rotation_radius_m=profile.rotation_radius_m,
                rotation_clearance_margin_m=profile.rotation_clearance_margin_m,
                linear_rest_tolerance_mps=profile.linear_rest_tolerance_mps,
                angular_acceleration_max_rps2=profile.angular_acceleration_max_rps2,
                lateral_acceleration_max_mps2=profile.lateral_acceleration_max_mps2,
                align_yaw_rate_max_rps=profile.align_yaw_rate_max_rps,
            ),
            TimingProfile(
                candidate_lease_s=profile.candidate_lease_s,
                obstacle_lease_s=profile.candidate_lease_s,
                ego_lease_s=profile.candidate_lease_s,
                processing_budget_s=profile.processing_budget_s,
            ),
        )
        self._counter = 0
        self._active: tuple[str, str, OptionGoal, PlannedPath] | None = None
        self._completed_nodes: set[int] = set()
        self._safety_latched = False
        self._last_route_rejection = ""
        self._last_policy_stamp_ns: int | None = None
        self._opponent_filter = OpponentFilter(FilterConfig(confirmation_count=2))
        self._opponent_filter_identity: tuple[int, str] | None = None
        self._opponent_track: OpponentTrack | None = None
        self._control_period_s = profile.control_period_s
        self.last_trace = PolicyCycleTrace(
            "NOT_RUN", None, "", "", 0, 0.0, 0.0, None, "",
            Actuation(0.0, 0.0), "",
        )

    def __call__(self, frame: PolicyInput) -> Actuation:
        if not isinstance(frame, PolicyInput):
            raise ValueError("frame must be a validated PolicyInput")
        if frame.role != self.role:
            raise ValueError("policy role does not match its endpoint")
        if self._last_policy_stamp_ns is not None:
            if frame.stamp_ns <= self._last_policy_stamp_ns:
                return self._stop("non_monotonic_policy_time", frame)
            self._control_period_s = (
                frame.stamp_ns - self._last_policy_stamp_ns
            ) / 1_000_000_000.0
        self._last_policy_stamp_ns = frame.stamp_ns
        match = frame.match_state
        estimate = frame.pose_estimate
        graph = frame.topology_graph
        if match is None or estimate is None or graph is None:
            return self._stop("missing_stage_pose_or_topology", frame)
        if not self._coherent(frame, estimate, graph):
            return self._stop("incoherent_observation_or_versions", frame)
        self._update_opponent_track(frame, estimate, graph)
        now_ns = frame.stamp_ns
        coverage_fraction, free_distance, forward_coverage = self._forward_clearance(
            frame
        )
        self._last_route_rejection = ""
        stage_lease_valid = (
            match.meta.valid_until_ns > now_ns
            and match.meta.clock_epoch == estimate.clock_epoch
            and match.meta.localization_epoch == estimate.localization_epoch
        )
        system_ready = (
            stage_lease_valid
            and estimate.position_error_bound_m <= self.profile.pose_error_limit_m
            and estimate.yaw_error_bound_rad <= self.profile.yaw_error_limit_rad
            and estimate.linear_velocity_mps <= self.profile.speed_max_mps
            and abs(estimate.angular_velocity_rps)
            <= self.profile.ego_yaw_rate_tolerance_rps
            and coverage_fraction >= self.profile.minimum_observation_coverage
        )
        safety_stop = self._safety_latched
        context = self._context(
            frame,
            graph,
            safety_stop=safety_stop,
            path_valid=False,
        )
        proposals, paths = self._proposals(
            frame,
            context,
            graph,
            coverage_fraction,
            forward_coverage,
        )
        fallback = self._hold_proposal(frame, graph)
        tactical = TacticalSnapshot(
            context=context,
            proposals=tuple((*proposals, fallback)),
            system_ready=system_ready,
            health_reason=(
                ""
                if system_ready
                else "pose_stage_or_sensor_evidence_unavailable"
            ),
        )
        current_key = self._active[0] if self._active else ""
        current_id = self._active[1] if self._active else ""
        current_started = (
            self._active[2].deadline_ns - (match.stage_ends_at_ns - now_ns)
            if self._active
            else None
        )
        selection = self.selector.select(
            tactical,
            current_stable_key=current_key,
            current_option_instance_id=current_id,
            current_started_at_ns=current_started,
        )
        selected = selection.selected
        if selected.goal.kind == OptionKind.HOLD_SAFE:
            if self._active is not None:
                self._cancel_active(frame, graph)
            return self._trace_stop(
                self._last_route_rejection or "hold_safe",
                selected.goal.kind,
                selection.reason,
                "not_admitted",
            )
        selected_path = paths.get(selected.goal.target_node_id)
        if (
            selected.goal.kind != OptionKind.OBSERVE_SAFE
            and selected_path is None
        ):
            return self._trace_stop(
                "path_unavailable",
                selected.goal.kind,
                selection.reason,
                "no_versioned_open_path",
            )
        if self._active is None or self._active[1] != selected.goal.option_instance_id:
            if self._active is not None:
                self._cancel_active(frame, graph)
            if selected.goal.kind == OptionKind.OBSERVE_SAFE:
                return self._execute_observation_option(
                    frame, context, selected, selection.reason
                )
            if selected_path is None:
                return self._trace_stop(
                    "path_unavailable",
                    selected.goal.kind,
                    selection.reason,
                    "no_versioned_open_path",
                )
            return self._start_motion_option(
                frame,
                graph,
                selected,
                selected_path,
                selection.reason,
                free_distance,
                forward_coverage,
            )
        if selected.goal.kind == OptionKind.OBSERVE_SAFE:
            return self._execute_observation_option(
                frame, context, selected, selection.reason
            )
        if self._at_goal(estimate, selected_path):
            evidence_id = (
                f"p56-arrival:{selected.goal.option_instance_id}:{frame.stamp_ns}"
            )
            effect_context = self._context(
                frame,
                graph,
                safety_stop=safety_stop,
                path_valid=True,
                effect_satisfied=True,
                effect_instance_id=selected.goal.option_instance_id,
                effect_evidence_id=evidence_id,
            )
            self.authority.tick(effect_context)
            self._completed_nodes.add(selected.goal.target_node_id)
            self._active = None
            return self._trace_stop(
                "option_effect_satisfied",
                selected.goal.kind,
                selection.reason,
                "effect_satisfied",
                evidence_id=evidence_id,
            )
        try:
            replanning_context = self._context(
                frame, graph, safety_stop=safety_stop, path_valid=True
            )
            action_id = self._active_action_id()
            state = self.authority.begin_replan(action_id, replanning_context)
            self.authority.mark_executing(action_id, replanning_context)
            if selected_path is None:
                raise ValueError("active option has no current versioned path")
            return self._execute_candidate(
                frame,
                graph,
                selected_path,
                selected.goal,
                state.lease_generation,
                free_distance,
                forward_coverage,
                selection.reason,
            )
        except ValueError as error:
            if self._active is not None:
                self._cancel_active(frame, graph)
            return self._trace_stop(
                "authority_or_planning_rejected",
                selected.goal.kind,
                selection.reason,
                str(error),
            )

    def _coherent(
        self, frame: PolicyInput, estimate: PoseEstimate, graph: TopologyGraph
    ) -> bool:
        match = frame.match_state
        if match is None:
            return False
        return (
            isinstance(frame.robot_radius_m, (int, float))
            and not isinstance(frame.robot_radius_m, bool)
            and math.isfinite(frame.robot_radius_m)
            and math.isclose(
                frame.robot_radius_m,
                self.profile.robot_radius_m,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            and frame.observation.frame_id == self.profile.sensor_frame_id
            and round(frame.observation.stamp_s * 1_000_000_000) == frame.stamp_ns
            and estimate.stamp_ns == frame.stamp_ns
            and estimate.frame_id == self.profile.localization_frame_id
            and estimate.clock_epoch == match.meta.clock_epoch
            and estimate.localization_epoch == match.meta.localization_epoch
            and graph.localization_epoch == estimate.localization_epoch
            and graph.map_version == match.meta.map_version
            and graph.topology_version == match.meta.topology_version
            and match.meta.observation_stamp_ns <= frame.stamp_ns
            and match.meta.state_stamp_ns <= frame.stamp_ns
        )

    @property
    def opponent_track(self) -> OpponentTrack | None:
        return self._opponent_track

    def _update_opponent_track(
        self,
        frame: PolicyInput,
        estimate: PoseEstimate,
        graph: TopologyGraph,
    ) -> None:
        if not isinstance(frame, SILPolicyInput):
            self._opponent_filter.reset()
            self._opponent_filter_identity = None
            self._opponent_track = None
            return
        identity = (graph.map_version, estimate.localization_epoch)
        if self._opponent_filter_identity != identity:
            self._opponent_filter.reset()
            self._opponent_filter_identity = identity
            self._opponent_track = None
        detections = extract_synthetic_opponent(
            frame.observation,
            estimate.pose,
            frame.static_segments,
            stamp_s=frame.observation.stamp_s,
            source_id=f"sil-lidar:{frame.role.value}",
            map_version=graph.map_version,
            localization_epoch=estimate.localization_epoch,
            opponent_radius_m=frame.opponent_radius_m,
        )
        stamp_s = frame.stamp_ns / 1e9
        if not self._opponent_filter.initialized:
            if len(detections) != 1:
                self._opponent_track = None
                return
            output = self._opponent_filter.initialize(detections[0])
        else:
            output = self._opponent_filter.update(detections, stamp_s)
        if output.last_measurement_s < 0.0:
            self._opponent_track = None
            return
        state_x, state_y, velocity_x, velocity_y = output.state_vector
        self._opponent_track = OpponentTrack(
            track_id=f"sil-opponent:{frame.role.value}",
            pose=Pose2D(state_x, state_y, 0.0),
            velocity_x_mps=velocity_x,
            velocity_y_mps=velocity_y,
            covariance=output.covariance,
            yaw_valid=False,
            last_measurement_s=output.last_measurement_s,
            valid_until_s=(
                output.last_measurement_s
                + self._opponent_filter.config.lost_timeout_s
            ),
            localization_epoch=estimate.localization_epoch,
            map_version=graph.map_version,
            state=output.state,
            source_id=f"sil-lidar:{frame.role.value}",
            frame_id=estimate.frame_id,
        )

    def _context(
        self,
        frame: PolicyInput,
        graph: TopologyGraph,
        *,
        safety_stop: bool,
        path_valid: bool,
        effect_satisfied: bool = False,
        effect_instance_id: str = "",
        effect_evidence_id: str = "",
    ) -> OptionContext:
        state = frame.match_state
        if state is None:
            raise ValueError("leased match state is required")
        accepted_zones = tuple(
            zone_id
            for zone_id in (state.own_start_zone_id, state.target_zone_id)
            if zone_id
        )
        if (
            isinstance(frame, SILPolicyInput)
            and frame.simultaneous_sil
            and frame.synthetic_goal_zone_id
        ):
            accepted_zones = (*accepted_zones, frame.synthetic_goal_zone_id)
        return OptionContext(
            now_ns=frame.stamp_ns,
            stage_id=state.meta.stage_id,
            stage_phase=state.phase,
            stage_ends_at_ns=state.stage_ends_at_ns,
            role=_core_role(frame.role),
            clock_epoch=state.meta.clock_epoch,
            localization_epoch=state.meta.localization_epoch,
            map_version=graph.map_version,
            topology_version=graph.topology_version,
            motion_authorized=(
                state.motion_authorized
                and state.role == _core_role(frame.role)
            ),
            lease_valid=state.meta.validity == 1 and state.meta.valid_until_ns > frame.stamp_ns,
            safety_stop=safety_stop,
            accepted_goal_zone_ids=accepted_zones,
            path_valid=path_valid,
            effect_satisfied=effect_satisfied,
            effect_instance_id=effect_instance_id,
            effect_evidence_id=effect_evidence_id,
        )

    def _forward_clearance(
        self, frame: PolicyInput
    ) -> tuple[float, float, bool]:
        observation = frame.observation
        if not observation.ranges_m:
            return 0.0, 0.0, False
        coverage_fraction = (
            sum(observation.coverage_mask) / len(observation.coverage_mask)
        )
        count = len(observation.ranges_m)
        beam_angles = tuple(
            -math.pi + 2.0 * math.pi * index / count
            for index in range(count)
        )
        selected = tuple(
            (index, angle)
            for index, angle in enumerate(beam_angles)
            if abs(angle) <= self.profile.forward_half_angle_rad
        )
        if not selected:
            return coverage_fraction, 0.0, False
        coverage = tuple(
            observation.coverage_mask[index] for index, _angle in selected
        )
        if not all(coverage):
            return coverage_fraction, 0.0, False
        free_distance = min(
            observation.ranges_m[index] * math.cos(angle)
            for index, angle in selected
        )
        estimate = frame.pose_estimate
        if estimate is not None:
            free_distance = max(
                0.0, free_distance - estimate.position_error_bound_m
            )
        return coverage_fraction, free_distance, True

    def _rotation_clearance(
        self, frame: PolicyInput
    ) -> tuple[bool, float]:
        observation = frame.observation
        count = len(observation.ranges_m)
        coverage = observation.coverage_mask
        if (
            count < self.profile.rotation_minimum_scan_beams
            or len(coverage) != count
            or not all(coverage)
        ):
            return False, 0.0
        if not observation.ranges_m:
            return False, 0.0
        angular_gap_bound = max(observation.ranges_m) * math.sin(
            math.pi / count
        )
        measured_clearance = (
            min(observation.ranges_m)
            - angular_gap_bound
            - self.profile.rotation_range_error_bound_m
        )
        return True, max(0.0, measured_clearance)

    def _proposals(
        self,
        frame: PolicyInput,
        context: OptionContext,
        graph: TopologyGraph,
        coverage_fraction: float,
        forward_coverage: bool,
    ) -> tuple[list[OptionProposal], dict[int, PlannedPath]]:
        estimate = frame.pose_estimate
        if estimate is None:
            return [], {}
        if self._safety_latched:
            return [], {}
        if self.role == MatchRole.EXPLORER and not (
            isinstance(frame, SILPolicyInput) and frame.simultaneous_sil
        ):
            if coverage_fraction < self.profile.minimum_observation_coverage:
                return [], {}
            proposal = self._make_proposal(
                frame,
                OptionKind.OBSERVE_SAFE,
                target_node_id=0,
                guards=(TacticalGuard.INFORMATIVE_SAFE_OBSERVATION,),
                evidence=f"lidar-coverage:{frame.stamp_ns}:{coverage_fraction:.6f}",
                features={
                    name: (
                        coverage_fraction if name == "observation_gain" else 0.0
                    )
                    for name in ROLE_FEATURES[Role.EXPLORER]
                },
            )
            return [proposal], {}
        if not forward_coverage:
            return [], {}

        if self.role == MatchRole.EXPLORER:
            return self._explorer_proposals(
                frame, graph, coverage_fraction, forward_coverage
            )

        if isinstance(frame, SILPolicyInput) and frame.simultaneous_sil:
            rival = self._usable_opponent_track(frame)
            if rival is not None:
                return self._guardian_pursuit_proposal(
                    frame, graph, rival, coverage_fraction
                )

        proposals: list[OptionProposal] = []
        paths: dict[int, PlannedPath] = {}
        for node in graph.nodes:
            if node.kind not in (NodeKind.PORTAL, NodeKind.FRONTIER):
                continue
            if node.node_id in self._completed_nodes:
                continue
            path = self._path_to_node(frame, graph, node.node_id)
            if path is None:
                self._last_route_rejection = f"no_supported_open_route_to_node:{node.node_id}"
                continue
            paths[node.node_id] = path
            distance = math.hypot(
                node.x_m - estimate.pose.x_m,
                node.y_m - estimate.pose.y_m,
            )
            proposal = self._make_proposal(
                frame,
                OptionKind.SEARCH_PORTAL,
                target_node_id=node.node_id,
                guards=(TacticalGuard.GRAPH_SEARCH_VIEWPOINT,),
                evidence=(
                    f"fixture-topology:{graph.map_version}:{graph.topology_version}:"
                    f"portal:{node.node_id}:open-route"
                ),
                features={
                    "capture_opportunity": 0.0,
                    "portal_time_advantage": 0.0,
                    "observation_gain": coverage_fraction,
                    "pursuit_value": 1.0 / (1.0 + path.cost),
                    "duration_cost": min(
                        1.0,
                        distance / max(
                            self.profile.speed_max_mps
                            * max(
                                0.1,
                                (context.stage_ends_at_ns - context.now_ns)
                                / 1e9,
                            ),
                            1e-9,
                        ),
                    ),
                },
            )
            proposals.append(proposal)
        return proposals, paths

    def _path_to_node(
        self, frame: PolicyInput, graph: TopologyGraph, node_id: int
    ) -> PlannedPath | None:
        estimate = frame.pose_estimate
        if estimate is None:
            return None
        graph_edges = {edge.edge_id: edge for edge in graph.edges}
        starts = sorted(
            graph.nodes,
            key=lambda node: (
                math.hypot(node.x_m - estimate.pose.x_m, node.y_m - estimate.pose.y_m),
                node.node_id,
            ),
        )
        for start in starts:
            try:
                candidate = astar(graph, start.node_id, node_id)
            except ValueError:
                continue
            if not candidate.edge_ids or any(
                graph_edges[edge_id].state != EdgeState.OPEN
                for edge_id in candidate.edge_ids
            ):
                continue
            clearance = min(
                graph_edges[edge_id].min_clearance_radius_m
                for edge_id in candidate.edge_ids
            )
            try:
                return self._clip_path_to_pose(candidate, estimate, clearance)
            except ValueError:
                continue
        return None

    def _usable_opponent_track(self, frame: PolicyInput) -> OpponentTrack | None:
        track = self._opponent_track
        estimate = frame.pose_estimate
        graph = frame.topology_graph
        now_s = frame.stamp_ns / 1e9
        if (
            track is None
            or estimate is None
            or graph is None
            or track.state != TrackState.TRACKED
            or track.valid_until_s <= now_s
            or track.map_version != graph.map_version
            or track.localization_epoch != estimate.localization_epoch
        ):
            return None
        return track

    def _guardian_pursuit_proposal(
        self,
        frame: PolicyInput,
        graph: TopologyGraph,
        rival: OpponentTrack,
        coverage_fraction: float,
    ) -> tuple[list[OptionProposal], dict[int, PlannedPath]]:
        node = min(
            graph.nodes,
            key=lambda item: (
                math.hypot(item.x_m - rival.pose.x_m, item.y_m - rival.pose.y_m),
                item.node_id,
            ),
        )
        path = self._path_to_node(frame, graph, node.node_id)
        if path is None:
            self._last_route_rejection = "no_supported_open_route_to_opponent_belief"
            return [], {}
        distance = math.hypot(
            rival.pose.x_m - frame.pose_estimate.pose.x_m,
            rival.pose.y_m - frame.pose_estimate.pose.y_m,
        )
        evidence = f"opponent-track:{rival.track_id}:{frame.stamp_ns}"
        proposal = self._make_proposal(
            frame,
            OptionKind.PRESSURE_ROUTE,
            target_node_id=node.node_id,
            guards=(TacticalGuard.USEFUL_RIVAL,),
            evidence=evidence,
            features={
                "capture_opportunity": 0.0,
                "portal_time_advantage": 0.0,
                "observation_gain": coverage_fraction,
                "pursuit_value": 1.0 / (1.0 + path.cost),
                "duration_cost": min(
                    1.0,
                    distance
                    / max(
                        self.profile.speed_max_mps
                        * max(0.1, (frame.match_state.stage_ends_at_ns - frame.stamp_ns) / 1e9),
                        1e-9,
                    ),
                ),
            },
        )
        return [proposal], {node.node_id: path}

    def _explorer_proposals(
        self,
        frame: PolicyInput,
        graph: TopologyGraph,
        coverage_fraction: float,
        forward_coverage: bool,
    ) -> tuple[list[OptionProposal], dict[int, PlannedPath]]:
        if (
            not isinstance(frame, SILPolicyInput)
            or not frame.simultaneous_sil
            or not forward_coverage
            or frame.synthetic_goal_node_id is None
            or not frame.synthetic_goal_zone_id.startswith("sil-fixture:")
        ):
            return [], {}
        rival = self._usable_opponent_track(frame)
        if rival is None:
            path = self._path_to_node(
                frame, graph, frame.synthetic_goal_node_id
            )
            if path is None:
                self._last_route_rejection = "no_supported_open_unobserved_route"
                return [], {}
            proposal = self._make_proposal(
                frame,
                OptionKind.ADVANCE_KNOWN_ROUTE,
                target_node_id=frame.synthetic_goal_node_id,
                guards=(TacticalGuard.VERSIONED_OPEN_ROUTE,),
                evidence=(
                    f"sil-open-route:{graph.map_version}:"
                    f"{graph.topology_version}:{path.goal_node_id}"
                ),
                features={
                    "base_progress": min(1.0, 1.0 / (1.0 + path.cost)),
                    "visibility_loss": 0.0,
                    "alternative_exits": 0.0,
                    "escape_safety": 0.0,
                    "observation_gain": coverage_fraction,
                    "capture_risk": 0.0,
                    "duration_cost": min(1.0, path.cost),
                },
            )
            return [proposal], {frame.synthetic_goal_node_id: path}

        estimate = frame.pose_estimate
        if estimate is None:
            return [], {}
        rival_distance = math.hypot(
            rival.pose.x_m - estimate.pose.x_m,
            rival.pose.y_m - estimate.pose.y_m,
        )
        target_node_id = frame.synthetic_goal_node_id
        target_kind = OptionKind.ADVANCE_BASE
        urgency_rank = None
        urgency_evidence_id = ""
        if rival_distance <= self.profile.opponent_threat_distance_m:
            candidate_routes: list[tuple[bool, float, int, PlannedPath]] = []
            for node in graph.nodes:
                if node.node_id in self._completed_nodes:
                    continue
                path = self._path_to_node(frame, graph, node.node_id)
                if path is None:
                    continue
                distance_from_rival = math.hypot(
                    node.x_m - rival.pose.x_m, node.y_m - rival.pose.y_m
                )
                hidden = not line_of_sight_clear(
                    (rival.pose.x_m, rival.pose.y_m),
                    (node.x_m, node.y_m),
                    frame.static_segments,
                )
                score = distance_from_rival - 0.25 * path.cost
                candidate_routes.append((hidden, score, node.node_id, path))
            if not candidate_routes:
                return [], {}
            hidden_routes = [route for route in candidate_routes if route[0]]
            selected_route = max(
                hidden_routes or candidate_routes,
                key=lambda route: (route[1], -route[2]),
            )
            target_node_id = selected_route[2]
            target_kind = (
                OptionKind.BREAK_LOS
                if selected_route[0]
                else OptionKind.KEEP_ESCAPE_ROUTE
            )
            urgency_rank = max(
                1,
                min(
                    255,
                    round(
                        255.0
                        * (self.profile.opponent_threat_distance_m - rival_distance)
                        / self.profile.opponent_threat_distance_m
                    ),
                ),
            )
            urgency_evidence_id = (
                f"opponent-threat:{rival.track_id}:{frame.stamp_ns}:"
                f"distance={rival_distance:.6f}"
            )

        path = self._path_to_node(frame, graph, target_node_id)
        if path is None:
            self._last_route_rejection = (
                f"no_supported_open_explorer_route:{target_node_id}"
            )
            return [], {}
        goal_node = next(node for node in graph.nodes if node.node_id == target_node_id)
        distance_to_rival = math.hypot(
            goal_node.x_m - rival.pose.x_m, goal_node.y_m - rival.pose.y_m
        )
        hidden = not line_of_sight_clear(
            (rival.pose.x_m, rival.pose.y_m),
            (goal_node.x_m, goal_node.y_m),
            frame.static_segments,
        )
        if target_kind == OptionKind.ADVANCE_BASE:
            guards = (
                TacticalGuard.ACCEPTED_GOAL,
                TacticalGuard.GOAL_THREAT_ACCEPTABLE,
            )
            goal_zone_id = frame.synthetic_goal_zone_id
            evidence = (
                f"sil-fixture-goal:{frame.synthetic_goal_zone_id}:"
                f"track:{rival.track_id}:{frame.stamp_ns}"
            )
        elif target_kind == OptionKind.ADVANCE_KNOWN_ROUTE:
            guards = (TacticalGuard.VERSIONED_OPEN_ROUTE,)
            goal_zone_id = ""
            evidence = (
                f"sil-open-route:{graph.map_version}:"
                f"{graph.topology_version}:{target_node_id}"
            )
        elif target_kind == OptionKind.BREAK_LOS:
            guards = (
                TacticalGuard.IMMEDIATE_TRAP_RISK,
                TacticalGuard.BREAK_LOS_FEASIBLE,
            )
            goal_zone_id = ""
            evidence = f"sil-break-los:{rival.track_id}:{frame.stamp_ns}"
        else:
            guards = (
                TacticalGuard.IMMEDIATE_TRAP_RISK,
                TacticalGuard.VERIFIED_ESCAPE_ROUTE,
            )
            goal_zone_id = ""
            evidence = f"sil-escape-route:{rival.track_id}:{frame.stamp_ns}"
        proposal = self._make_proposal(
            frame,
            target_kind,
            target_node_id=target_node_id,
            guards=guards,
            evidence=evidence,
            features={
                "base_progress": min(1.0, 1.0 / (1.0 + path.cost)),
                "visibility_loss": float(hidden),
                "alternative_exits": min(
                    1.0,
                    sum(
                        edge.state == EdgeState.OPEN
                        and target_node_id in (edge.from_node, edge.to_node)
                        for edge in graph.edges
                    )
                    / 4.0,
                ),
                "escape_safety": min(
                    1.0,
                    distance_to_rival / self.profile.opponent_threat_distance_m,
                ),
                "observation_gain": coverage_fraction,
                "capture_risk": max(
                    0.0,
                    min(
                        1.0,
                        (self.profile.opponent_threat_distance_m - distance_to_rival)
                        / self.profile.opponent_threat_distance_m,
                    ),
                ),
                "duration_cost": min(
                    1.0,
                    path.cost
                    / max(
                        self.profile.speed_max_mps
                        * max(
                            0.1,
                            (frame.match_state.stage_ends_at_ns - frame.stamp_ns)
                            / 1e9,
                        ),
                        1e-9,
                    ),
                ),
            },
            goal_zone_id=goal_zone_id,
            urgency_rank=urgency_rank,
            urgency_evidence_id=urgency_evidence_id,
        )
        return [proposal], {target_node_id: path}

    def _clip_path_to_pose(
        self,
        path: PlannedPath,
        estimate: PoseEstimate,
        corridor_clearance_m: float,
    ) -> PlannedPath:
        if corridor_clearance_m <= (
            self.profile.robot_radius_m
            + estimate.position_error_bound_m
        ):
            raise ValueError("route clearance cannot contain robot footprint and pose bound")
        x, y = estimate.pose.x_m, estimate.pose.y_m
        best_distance = math.inf
        best_index = -1
        projection = (0.0, 0.0)
        for index, (start, end) in enumerate(
            zip(path.polyline_xy_m, path.polyline_xy_m[1:])
        ):
            dx, dy = end[0] - start[0], end[1] - start[1]
            length_squared = dx * dx + dy * dy
            if length_squared <= 0.0:
                continue
            fraction = max(
                0.0,
                min(
                    1.0,
                    ((x - start[0]) * dx + (y - start[1]) * dy)
                    / length_squared,
                ),
            )
            point = (start[0] + fraction * dx, start[1] + fraction * dy)
            distance = math.hypot(x - point[0], y - point[1])
            if distance < best_distance:
                best_distance = distance
                best_index = index
                projection = point
        if (
            best_index < 0
            or best_distance + estimate.position_error_bound_m
            > corridor_clearance_m - self.profile.robot_radius_m
        ):
            raise ValueError("pose is outside the supported open-corridor envelope")
        points = [projection]
        points.extend(path.polyline_xy_m[best_index + 1 :])
        compact_points = [points[0]]
        for point in points[1:]:
            if math.dist(point, compact_points[-1]) > 1e-9:
                compact_points.append(point)
        if len(compact_points) < 2:
            raise ValueError("route has no remaining non-zero segment")
        remaining_cost = math.fsum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(compact_points, compact_points[1:])
        )
        return PlannedPath(
            path.map_version,
            path.topology_version,
            path.localization_epoch,
            path.start_node_id,
            path.goal_node_id,
            path.node_ids,
            path.edge_ids,
            tuple(compact_points),
            remaining_cost,
        )

    def _make_proposal(
        self,
        frame: PolicyInput,
        kind: OptionKind,
        *,
        target_node_id: int,
        guards: tuple[TacticalGuard, ...],
        evidence: str,
        features: dict[str, float],
        goal_zone_id: str = "",
        urgency_rank: int | None = None,
        urgency_evidence_id: str = "",
    ) -> OptionProposal:
        match = frame.match_state
        graph = frame.topology_graph
        if match is None or graph is None:
            raise ValueError("match state and topology are required")
        stable_key = f"{kind.name.lower()}:{target_node_id}"
        instance_id = (
            self._active[1]
            if self._active is not None and self._active[0] == stable_key
            else f"p56:{self.role.value}:{match.meta.stage_id}:{stable_key}:"
            f"{self._counter + 1}"
        )
        goal = OptionGoal(
            option_instance_id=instance_id,
            kind=kind,
            role=_core_role(frame.role),
            stage_id=match.meta.stage_id,
            clock_epoch=match.meta.clock_epoch,
            localization_epoch=match.meta.localization_epoch,
            deadline_ns=match.stage_ends_at_ns,
            map_version=graph.map_version,
            topology_version=graph.topology_version,
            parameters_id=self.profile.parameters_id,
            schema_version=2,
            has_target_node=kind != OptionKind.OBSERVE_SAFE,
            target_node_id=target_node_id if kind != OptionKind.OBSERVE_SAFE else 0,
            position_tolerance_m=self.profile.goal_tolerance_m,
            yaw_tolerance_rad=0.2,
            goal_zone_id=goal_zone_id,
        )
        return OptionProposal(
            stable_key=stable_key,
            goal=goal,
            feasible=True,
            infeasible_reason="",
            guard_facts=guards,
            guard_evidence=tuple((guard, evidence) for guard in guards),
            features=tuple(features.items()),
            urgency_rank=urgency_rank,
            urgency_evidence_id=urgency_evidence_id,
        )

    def _hold_proposal(
        self, frame: PolicyInput, graph: TopologyGraph
    ) -> OptionProposal:
        match = frame.match_state
        if match is None:
            raise ValueError("match state is required")
        goal = OptionGoal(
            option_instance_id=f"p56:{self.role.value}:{match.meta.stage_id}:hold",
            kind=OptionKind.HOLD_SAFE,
            role=_core_role(frame.role),
            stage_id=match.meta.stage_id,
            clock_epoch=match.meta.clock_epoch,
            localization_epoch=match.meta.localization_epoch,
            deadline_ns=match.stage_ends_at_ns,
            map_version=graph.map_version,
            topology_version=graph.topology_version,
            parameters_id=self.profile.parameters_id,
            schema_version=2,
            position_tolerance_m=0.0,
            yaw_tolerance_rad=0.0,
        )
        return OptionProposal(
            stable_key="hold-safe",
            goal=goal,
            feasible=True,
            infeasible_reason="",
            guard_facts=(),
            guard_evidence=(),
            features=(),
        )

    def _start_motion_option(
        self,
        frame: PolicyInput,
        graph: TopologyGraph,
        proposal: OptionProposal,
        path: PlannedPath,
        selection_reason: str,
        free_distance: float,
        coverage_valid: bool,
    ) -> Actuation:
        try:
            self._counter += 1
            action_id = f"p56-action-{self.role.value}-{self._counter}"
            authority_context = self._context(
                frame,
                graph,
                safety_stop=False,
                path_valid=True,
            )
            admission = self.authority.submit(
                action_id, proposal.goal, authority_context
            )
            if not admission.accepted:
                return self._trace_stop(
                    "option_not_admitted",
                    proposal.goal.kind,
                    selection_reason,
                    admission.reason,
                )
            self._active = (
                proposal.stable_key,
                proposal.goal.option_instance_id,
                proposal.goal,
                path,
            )
            state = self.authority.mark_executing(action_id, authority_context)
            return self._execute_candidate(
                frame,
                graph,
                path,
                proposal.goal,
                state.lease_generation,
                free_distance,
                coverage_valid,
                selection_reason,
            )
        except ValueError as error:
            if self._active is not None:
                self._cancel_active(frame, frame.topology_graph)
            return self._trace_stop(
                "option_admission_or_plan_failed",
                proposal.goal.kind,
                selection_reason,
                str(error),
            )

    def _execute_candidate(
        self,
        frame: PolicyInput,
        graph: TopologyGraph | None,
        path: PlannedPath,
        goal: OptionGoal,
        generation: int,
        free_distance: float,
        coverage_valid: bool,
        selection_reason: str,
    ) -> Actuation:
        if graph is None or frame.pose_estimate is None:
            return self._trace_stop(
                "missing_planning_inputs", goal.kind, selection_reason, "missing_inputs"
            )
        try:
            (
                candidate,
                control_mode,
                coverage_360_valid,
                free_distance_360,
                curved_path_coverage_valid,
                curved_path_clearance,
            ) = (
                self._stop_turn_go_candidate(
                    frame, path, goal, generation
                )
            )
            envelope = CandidateEnvelope(candidate, path, goal.option_instance_id)
            admitted = self.authority.lease.admit(
                envelope,
                now_s=frame.stamp_ns / 1e9,
                generation=generation,
            )
            safety = self.supervisor.evaluate(
                SafetySnapshot(
                    candidate_seq=frame.stamp_ns,
                    candidate_v_mps=admitted.linear_velocity_mps,
                    candidate_omega_rps=admitted.angular_velocity_rps,
                    candidate_stamp_ns=frame.stamp_ns,
                    candidate_valid_until_ns=round(admitted.valid_until_s * 1e9),
                    obstacle_stamp_ns=round(frame.observation.stamp_s * 1e9),
                    ego_stamp_ns=frame.pose_estimate.stamp_ns,
                    now_ros_ns=frame.stamp_ns,
                    frame_id=frame.pose_estimate.frame_id,
                    expected_frame_id=self.profile.localization_frame_id,
                    clock_epoch=frame.pose_estimate.clock_epoch,
                    expected_clock_epoch=goal.clock_epoch,
                    localization_epoch=frame.pose_estimate.localization_epoch,
                    expected_localization_epoch=goal.localization_epoch,
                    coverage_valid=(
                        coverage_360_valid if control_mode is ControlMode.ALIGN
                        else coverage_valid
                    ),
                    free_distance_m=free_distance,
                    ego_speed_mps=frame.pose_estimate.linear_velocity_mps,
                    candidate_map_version=admitted.map_version,
                    expected_map_version=graph.map_version,
                    candidate_topology_version=admitted.topology_version,
                    expected_topology_version=graph.topology_version,
                    candidate_lease_generation=generation,
                    expected_lease_generation=generation,
                    option_instance_id=goal.option_instance_id,
                    expected_option_instance_id=goal.option_instance_id,
                    received_steady_ns=0,
                    control_mode=control_mode,
                    free_distance_360_m=free_distance_360,
                    coverage_360_valid=coverage_360_valid,
                    position_error_bound_m=(
                        frame.pose_estimate.position_error_bound_m
                    ),
                    control_period_s=self._control_period_s,
                    ego_angular_velocity_rps=(
                        frame.pose_estimate.angular_velocity_rps
                    ),
                    curved_path_clearance_m=curved_path_clearance,
                    curved_path_coverage_valid=curved_path_coverage_valid,
                ),
                frame.stamp_ns,
                1,
            )
        except ValueError as error:
            self._cancel_active(frame, graph)
            return self._trace_stop(
                "candidate_or_safety_contract_rejected",
                goal.kind,
                selection_reason,
                str(error),
            )
        command = (
            Actuation(safety.applied_v_mps, safety.applied_omega_rps)
            if safety.decision in (ADMIT, LIMIT)
            else Actuation(0.0, 0.0)
        )
        if safety.decision == STOP:
            self._safety_latched = True
            self.authority.tick(
                self._context(
                    frame,
                    graph,
                    safety_stop=True,
                    path_valid=True,
                )
            )
            self._active = None
        self.last_trace = PolicyCycleTrace(
            "candidate_supervised",
            goal.kind,
            selection_reason,
            (
                self.authority.execution_state.reason
                if safety.decision == STOP
                else "executing"
            ),
            generation,
            admitted.linear_velocity_mps,
            admitted.angular_velocity_rps,
            safety.decision,
            safety.primary_reason,
            command,
            "",
        )
        return command

    def _stop_turn_go_candidate(
        self,
        frame: PolicyInput,
        path: PlannedPath,
        goal: OptionGoal,
        generation: int,
    ) -> tuple[MotionCandidate, ControlMode, bool, float, bool, float]:
        estimate = frame.pose_estimate
        if estimate is None:
            raise ValueError("pose estimate is required for stop-turn-go control")
        current = (estimate.pose.x_m, estimate.pose.y_m)
        waypoint = next(
            (
                point
                for point in path.polyline_xy_m[1:]
                if math.dist(current, point) > 1e-9
            ),
            None,
        )
        if waypoint is None:
            raise ValueError("planned path has no forward waypoint")
        target = lookahead_point(
            path,
            estimate.pose,
            self.profile.lookahead_m,
        )
        desired_yaw = math.atan2(
            target[1] - current[1], target[0] - current[0]
        )
        yaw_error = math.atan2(
            math.sin(desired_yaw - estimate.pose.theta_rad),
            math.cos(desired_yaw - estimate.pose.theta_rad),
        )
        alignment_needed = (
            abs(yaw_error) > self.profile.smooth_turn_heading_limit_rad
            or (
                abs(estimate.linear_velocity_mps)
                <= self.profile.linear_rest_tolerance_mps
                and abs(estimate.angular_velocity_rps) > 1e-3
            )
        )
        now_s = frame.stamp_ns / 1e9
        if alignment_needed:
            if self._control_period_s <= 0.0:
                raise ValueError("alignment control period must be positive")
            if (
                abs(estimate.linear_velocity_mps)
                > self.profile.linear_rest_tolerance_mps
            ):
                return (
                    MotionCandidate(
                        0.0,
                        0.0,
                        now_s,
                        now_s + self.profile.candidate_lease_s,
                        self.profile.candidate_lease_s,
                        goal.option_instance_id,
                        path.map_version,
                        path.localization_epoch,
                        path.topology_version,
                        generation,
                    ),
                    ControlMode.TRACK_PATH,
                    False,
                    0.0,
                    False,
                    0.0,
                )
            coverage_360_valid, free_distance_360 = self._rotation_clearance(frame)
            if abs(yaw_error) <= self.profile.align_yaw_tolerance_rad:
                requested_omega = 0.0
            else:
                requested_omega = math.copysign(
                    min(
                        self.profile.align_yaw_rate_max_rps,
                        math.sqrt(
                            2.0
                            * self.profile.angular_acceleration_max_rps2
                            * abs(yaw_error)
                        ),
                    ),
                    yaw_error,
                )
            max_omega_delta = (
                self.profile.angular_acceleration_max_rps2
                * self._control_period_s
            )
            angular = max(
                estimate.angular_velocity_rps - max_omega_delta,
                min(
                    estimate.angular_velocity_rps + max_omega_delta,
                    requested_omega,
                ),
            )
            candidate = MotionCandidate(
                0.0,
                angular,
                now_s,
                now_s + self.profile.candidate_lease_s,
                self.profile.candidate_lease_s,
                goal.option_instance_id,
                path.map_version,
                path.localization_epoch,
                path.topology_version,
                generation,
            )
            return (
                candidate,
                ControlMode.ALIGN,
                coverage_360_valid,
                free_distance_360,
                False,
                0.0,
            )

        candidate = make_candidate(
            path,
            estimate.pose,
            config=PursuitConfig(
                lookahead_m=self.profile.lookahead_m,
                speed_max_mps=self.profile.speed_max_mps,
                yaw_rate_max_rps=self.profile.planner_yaw_rate_max_rps,
                lateral_accel_max_mps2=self.profile.lateral_acceleration_max_mps2,
                goal_tolerance_m=self.profile.goal_tolerance_m,
            ),
            now_s=now_s,
            lease_s=self.profile.candidate_lease_s,
            source_id=goal.option_instance_id,
            lease_generation=generation,
        )
        distance = math.dist(current, path.polyline_xy_m[-1])
        speed_bound = math.sqrt(
            2.0 * self.profile.minimum_braking_deceleration_mps2 * distance
        )
        candidate = replace(
            candidate,
            linear_velocity_mps=min(
                candidate.linear_velocity_mps,
                self.profile.speed_max_mps,
                speed_bound,
            ),
            angular_velocity_rps=(
                candidate.angular_velocity_rps
                * min(
                    candidate.linear_velocity_mps,
                    self.profile.speed_max_mps,
                    speed_bound,
                )
                / candidate.linear_velocity_mps
                if candidate.linear_velocity_mps > 0.0
                else 0.0
            ),
        )
        coverage_360_valid, free_distance_360 = self._rotation_clearance(frame)
        curved_valid, curved_clearance = (
            self._curved_path_clearance(frame, candidate)
            if candidate.angular_velocity_rps != 0.0
            else (False, 0.0)
        )
        if candidate.angular_velocity_rps != 0.0 and curved_valid:
            required_stop = (
                candidate.linear_velocity_mps * self.profile.response_bound_s
                + candidate.linear_velocity_mps ** 2
                / (2.0 * self.profile.minimum_braking_deceleration_mps2)
            )
            if curved_clearance < required_stop + self.profile.clearance_margin_m:
                if (
                    abs(estimate.linear_velocity_mps)
                    <= self.profile.linear_rest_tolerance_mps
                ):
                    waypoint_yaw_error = math.atan2(
                        math.sin(
                            math.atan2(
                                waypoint[1] - current[1],
                                waypoint[0] - current[0],
                            )
                            - estimate.pose.theta_rad
                        ),
                        math.cos(
                            math.atan2(
                                waypoint[1] - current[1],
                                waypoint[0] - current[0],
                            )
                            - estimate.pose.theta_rad
                        ),
                    )
                    rotation_coverage, rotation_clearance = self._rotation_clearance(
                        frame
                    )
                    if abs(waypoint_yaw_error) > self.profile.align_yaw_tolerance_rad:
                        requested_omega = math.copysign(
                            min(
                                self.profile.align_yaw_rate_max_rps,
                                math.sqrt(
                                    2.0
                                    * self.profile.angular_acceleration_max_rps2
                                    * abs(waypoint_yaw_error)
                                ),
                            ),
                            waypoint_yaw_error,
                        )
                        max_omega_delta = (
                            self.profile.angular_acceleration_max_rps2
                            * self._control_period_s
                        )
                        angular = max(
                            estimate.angular_velocity_rps - max_omega_delta,
                            min(
                                estimate.angular_velocity_rps + max_omega_delta,
                                requested_omega,
                            ),
                        )
                    else:
                        angular = 0.0
                    return (
                        MotionCandidate(
                            0.0,
                            angular,
                            now_s,
                            now_s + self.profile.candidate_lease_s,
                            self.profile.candidate_lease_s,
                            goal.option_instance_id,
                            path.map_version,
                            path.localization_epoch,
                            path.topology_version,
                            generation,
                        ),
                        ControlMode.ALIGN,
                        rotation_coverage,
                        rotation_clearance,
                        False,
                        0.0,
                    )
                return (
                    replace(candidate, linear_velocity_mps=0.0, angular_velocity_rps=0.0),
                    ControlMode.TRACK_PATH,
                    coverage_360_valid,
                    free_distance_360,
                    False,
                    0.0,
                )
        return (
            candidate,
            ControlMode.TRACK_PATH,
            coverage_360_valid,
            free_distance_360,
            curved_valid,
            curved_clearance,
        )

    def _curved_path_clearance(
        self, frame: PolicyInput, candidate: MotionCandidate
    ) -> tuple[bool, float]:
        if not isinstance(frame, SILPolicyInput):
            return False, 0.0
        observation = frame.observation
        estimate = frame.pose_estimate
        count = len(observation.ranges_m)
        if (
            estimate is None
            or count < self.profile.curvature_minimum_scan_beams
            or len(observation.coverage_mask) != count
            or not all(observation.coverage_mask)
            or candidate.linear_velocity_mps <= 0.0
            or candidate.angular_velocity_rps == 0.0
        ):
            return False, 0.0

        step = self.profile.curvature_sample_step_m
        tube_radius = (
            self.profile.robot_radius_m
            + estimate.position_error_bound_m
            + self.profile.curvature_range_error_bound_m
            + frame.opponent_speed_bound_mps
            * (
                self.profile.response_bound_s
                + candidate.linear_velocity_mps
                / self.profile.minimum_braking_deceleration_mps2
            )
        )
        max_range = max(observation.ranges_m)
        horizon = min(
            max(0.0, max_range - tube_radius),
            max(0.5, 2.0 * self.profile.lookahead_m),
        )
        if horizon < step:
            return True, 0.0

        curvature = candidate.angular_velocity_rps / candidate.linear_velocity_mps
        half_beam = math.pi / count
        angles = tuple(-math.pi + 2.0 * math.pi * index / count for index in range(count))
        certified_distance = 0.0
        sample_count = math.ceil(horizon / step)
        for sample in range(1, sample_count + 1):
            arc_distance = min(sample * step, horizon)
            angle = curvature * arc_distance
            if abs(curvature) <= 1e-12:
                x_local, y_local = arc_distance, 0.0
            else:
                x_local = math.sin(angle) / curvature
                y_local = (1.0 - math.cos(angle)) / curvature
            radial_distance = math.hypot(x_local, y_local)
            if radial_distance <= tube_radius + step:
                certified_distance = max(0.0, arc_distance - step / 2.0)
                continue
            center_bearing = math.atan2(y_local, x_local)
            inflated_radius = (
                tube_radius
                + step / 2.0
                + radial_distance * math.sin(half_beam)
            )
            angular_radius = math.asin(
                min(1.0, inflated_radius / radial_distance)
            )
            beams = [
                (index, self._angle_difference(angle, center_bearing))
                for index, angle in enumerate(angles)
                if abs(self._angle_difference(angle, center_bearing))
                <= angular_radius + 1e-12
            ]
            if not beams:
                break
            point_clear = True
            for index, delta in beams:
                perpendicular_sq = (
                    radial_distance * math.sin(delta)
                ) ** 2
                discriminant = inflated_radius * inflated_radius - perpendicular_sq
                if discriminant < 0.0:
                    continue
                near_intersection = (
                    radial_distance * math.cos(delta)
                    - math.sqrt(max(0.0, discriminant))
                )
                range_lower_bound = (
                    observation.ranges_m[index]
                    - self.profile.curvature_range_error_bound_m
                )
                if range_lower_bound < max(0.0, near_intersection):
                    point_clear = False
                    break
            if not point_clear:
                break
            certified_distance = max(0.0, arc_distance - step / 2.0)
        return True, certified_distance

    @staticmethod
    def _angle_difference(first: float, second: float) -> float:
        return (first - second + math.pi) % (2.0 * math.pi) - math.pi

    def _execute_observation_option(
        self,
        frame: PolicyInput,
        context: OptionContext,
        proposal: OptionProposal,
        selection_reason: str,
    ) -> Actuation:
        self._counter += 1
        action_id = f"p56-action-{self.role.value}-{self._counter}"
        graph = frame.topology_graph
        if graph is None:
            return self._trace_stop(
                "missing_topology", proposal.goal.kind, selection_reason, "missing_inputs"
            )
        try:
            admitted = self.authority.submit(action_id, proposal.goal, context)
            if not admitted.accepted:
                return self._trace_stop(
                    "observation_option_not_admitted",
                    proposal.goal.kind,
                    selection_reason,
                    admitted.reason,
                )
            observation_context = self._context(
                frame, graph, safety_stop=False, path_valid=True
            )
            state = self.authority.mark_executing(action_id, observation_context)
            evidence_id = f"lidar-observation:{frame.stamp_ns}:{frame.observation.frame_id}"
            self.authority.tick(
                self._context(
                    frame,
                    graph,
                    safety_stop=False,
                    path_valid=True,
                    effect_satisfied=True,
                    effect_instance_id=proposal.goal.option_instance_id,
                    effect_evidence_id=evidence_id,
                )
            )
        except ValueError as error:
            state = self.authority.execution_state
            if (
                state.active_option_instance_id == proposal.goal.option_instance_id
                and state.phase.name != "FINISHED"
            ):
                cancel_context = self._context(
                    frame, graph, safety_stop=True, path_valid=True
                )
                self.authority.request_cancel(
                    action_id, cancel_context, cancel_timeout_ns=1
                )
                self.authority.acknowledge_cancel(action_id, cancel_context)
            return self._trace_stop(
                "observation_authority_rejected",
                proposal.goal.kind,
                selection_reason,
                str(error),
            )
        self.last_trace = PolicyCycleTrace(
            "observation_completed",
            proposal.goal.kind,
            selection_reason,
            self.authority.execution_state.reason,
            state.lease_generation,
            0.0,
            0.0,
            None,
            "no_motion_option",
            Actuation(0.0, 0.0),
            evidence_id,
        )
        return Actuation(0.0, 0.0)

    def _at_goal(self, estimate: PoseEstimate, path: PlannedPath) -> bool:
        goal = path.polyline_xy_m[-1]
        return (
            math.hypot(
                estimate.pose.x_m - goal[0],
                estimate.pose.y_m - goal[1],
            )
            + estimate.position_error_bound_m
            <= self.profile.goal_tolerance_m
        )

    def _active_action_id(self) -> str:
        if self._active is None:
            raise ValueError("no active option")
        state = self.authority.execution_state
        for record in reversed(self.authority.journal):
            if (
                record.option_instance_id == self._active[1]
                and record.action_id
            ):
                return record.action_id
        raise ValueError(f"active option {state.active_option_instance_id} has no action record")

    def _cancel_active(
        self, frame: PolicyInput, graph: TopologyGraph
    ) -> None:
        if self._active is None:
            return
        action_id = self._active_action_id()
        context = self._context(
            frame, graph, safety_stop=True, path_valid=False
        )
        self.authority.request_cancel(
            action_id, context, cancel_timeout_ns=1
        )
        self.authority.acknowledge_cancel(action_id, context)
        self._active = None

    def _trace_stop(
        self,
        status: str,
        kind: OptionKind | None,
        selection_reason: str,
        authority_reason: str,
        *,
        evidence_id: str = "",
    ) -> Actuation:
        command = Actuation(0.0, 0.0)
        self.last_trace = PolicyCycleTrace(
            status,
            kind,
            selection_reason,
            authority_reason,
            self.authority.execution_state.lease_generation,
            0.0,
            0.0,
            None,
            status,
            command,
            evidence_id,
        )
        return command

    def _stop(self, status: str, frame: PolicyInput) -> Actuation:
        if self._active is not None:
            self._revoke_active(frame)
        return self._trace_stop(status, None, "", "not_attempted")

    def _revoke_active(self, frame: PolicyInput) -> None:
        if self._active is None:
            return
        goal = self._active[2]
        now_ns = max(
            frame.stamp_ns,
            self.authority.execution_state.updated_at_ns,
        )
        match = frame.match_state
        context = OptionContext(
            now_ns=now_ns,
            stage_id=match.meta.stage_id if match is not None else goal.stage_id,
            stage_phase=match.phase if match is not None else StagePhase.TERMINAL,
            stage_ends_at_ns=max(
                now_ns + 1,
                match.stage_ends_at_ns if match is not None else goal.deadline_ns,
            ),
            role=goal.role,
            clock_epoch=goal.clock_epoch,
            localization_epoch=goal.localization_epoch,
            map_version=goal.map_version,
            topology_version=goal.topology_version,
            motion_authorized=False,
            lease_valid=False,
            safety_stop=True,
            path_valid=False,
        )
        action_id = self._active_action_id()
        self.authority.request_cancel(
            action_id, context, cancel_timeout_ns=1, reason="invalid_policy_input"
        )
        self.authority.acknowledge_cancel(action_id, context)
        self._active = None
