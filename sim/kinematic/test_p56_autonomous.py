"""Adversarial P5.6 observation-to-authority-to-safety SIL integration."""

from dataclasses import fields, replace
import math

import pytest

from hsl_core.control.safety import ControlMode
from hsl_core.match import Role, StageManager, StageProfile
from hsl_core.planning import PlannedPath
from hsl_core.tactics import OptionGoal, OptionKind
from hsl_core.topology import EdgeState, NodeKind, TopologyEdge, TopologyGraph, TopologyNode
from hsl_core.types import Pose2D
from sim.kinematic.autonomous import KinematicAutonomousPolicy
from sim.kinematic.common import (
    Actuation,
    CircleTarget,
    PoseEstimate,
    Segment,
    SensorObservation,
    WorldGeometry,
)
from sim.kinematic.match import (
    MatchRole,
    PolicyInput,
    RoleEndpoint,
    SILPolicyInput,
    TwoRobotMatch,
)
from sim.kinematic.maze_bank import load_maze_bank
from sim.kinematic.plant import KinematicPlant, SilDeadReckoningEstimator
from sim.kinematic.raycaster import FirstHitRaycaster
from sim.kinematic.sensors import LidarSensor


AUTH = "p56-sil-fixture-only"
CONFIG_HASH = "d" * 64


def _manager(
    role: Role,
    *,
    clock_epoch="p56-clock",
    localization_epoch="p56-localization",
) -> StageManager:
    profile = StageProfile(
        profile_id="p56-sil-only",
        score_profile_id="unapproved-test-fixture",
        freeze_duration_ns=100_000_000,
        stage_duration_ns=10_000_000_000,
        lease_duration_ns=500_000_000,
        zone_update_window_ns=100_000_000,
        max_request_records=64,
        max_event_records=16,
        max_audit_records=128,
        config_hash=CONFIG_HASH,
        source_id="p56-policy-test",
        source_session="test-session",
        clock_epoch=clock_epoch,
        localization_epoch=localization_epoch,
        organizer_authorization_ref=AUTH,
        approved_authorization_refs=(AUTH,),
        approved_zone_frames=("map",),
        approved_memory_profile_ids=(),
    )
    manager = StageManager(
        profile,
        initial_stage_id="p56-stage",
        stage_number=1,
        role=role,
    )
    result = manager.start_stage(
        request_id="start-p56",
        expected_stage_id=manager.stage_id,
        official_start_stamp_ns=0,
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    assert result.accepted
    return manager


def _graph(*, diagonal=False, blocked=False, map_version=0):
    target_y = 1.0 if diagonal else 0.0
    return TopologyGraph(
        map_version,
        0,
        "p56-localization",
        (
            TopologyNode(0, 0.0, 0.0, NodeKind.ANCHOR, 0.25),
            TopologyNode(1, 1.0, target_y, NodeKind.PORTAL, 0.25),
        ),
        (
            TopologyEdge(
                1,
                0,
                1,
                ((0.0, 0.0), (1.0, target_y)),
                0.25,
                state=EdgeState.BLOCKED if blocked else EdgeState.OPEN,
            ),
        ),
    )


def _endpoint(role, pose, policy, graph, *, blind=()):
    return RoleEndpoint(
        role=role,
        namespace=f"/robot_{role.value}",
        plant=KinematicPlant(pose, max_linear_mps=0.5),
        sensor=LidarSensor(
            raycaster=FirstHitRaycaster(max_range_m=4.0),
            beam_count=360,
            max_range_m=4.0,
            blind_sectors=blind,
            frame_id="lidar_link",
        ),
        policy=policy,
        radius_m=0.15,
        pose_estimator=SilDeadReckoningEstimator(
            PoseEstimate(
                0,
                "map",
                "p56-clock",
                "p56-localization",
                pose,
                0.0,
                0.0,
            )
        ),
        topology_graph=graph,
    )


def _runner(
    role,
    graph,
    *,
    guardian_blind=(),
    explorer_blind=(),
    geometry=WorldGeometry(()),
):
    guardian_policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    explorer_policy = KinematicAutonomousPolicy(MatchRole.EXPLORER)
    guardian = _endpoint(
        MatchRole.GUARDIAN,
        Pose2D(0.0, 0.0, 0.0),
        guardian_policy,
        graph,
        blind=guardian_blind,
    )
    explorer = _endpoint(
        MatchRole.EXPLORER,
        Pose2D(2.0, 0.0, math.pi),
        explorer_policy,
        graph,
        blind=explorer_blind,
    )
    return (
        TwoRobotMatch(
            guardian,
            explorer,
            geometry,
            stage_manager=_manager(role),
        ),
        guardian_policy,
        explorer_policy,
    )


def _corridor_duel():
    nodes = tuple(
        TopologyNode(index, 0.5 + index, 1.0, NodeKind.PORTAL, 0.8)
        for index in range(4)
    )
    edges = tuple(
        TopologyEdge(
            index,
            index,
            index + 1,
            ((0.5 + index, 1.0), (1.5 + index, 1.0)),
            0.8,
            state=EdgeState.OPEN,
        )
        for index in range(3)
    )
    graph = TopologyGraph(0, 0, "p56-localization", nodes, edges)
    geometry = WorldGeometry(
        (
            Segment((0.0, 0.0), (4.0, 0.0)),
            Segment((0.0, 2.0), (4.0, 2.0)),
            Segment((0.0, 0.0), (0.0, 2.0)),
            Segment((4.0, 0.0), (4.0, 2.0)),
        )
    )
    guardian_policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    explorer_policy = KinematicAutonomousPolicy(MatchRole.EXPLORER)

    def endpoint(role, pose, policy, *, goal_node=None, goal_zone=""):
        return RoleEndpoint(
            role=role,
            namespace=f"/robot_{role.value}",
            plant=KinematicPlant(pose, max_linear_mps=0.5),
            sensor=LidarSensor(
                raycaster=FirstHitRaycaster(max_range_m=5.0),
                beam_count=360,
                max_range_m=5.0,
            ),
            policy=policy,
            radius_m=0.15,
            pose_estimator=SilDeadReckoningEstimator(
                PoseEstimate(
                    0,
                    "map",
                    "p56-clock",
                    "p56-localization",
                    pose,
                    0.0,
                    0.0,
                )
            ),
            topology_graph=graph,
            synthetic_goal_node_id=goal_node,
            synthetic_goal_zone_id=goal_zone,
        )

    guardian = endpoint(
        MatchRole.GUARDIAN,
        Pose2D(0.5, 1.0, 0.0),
        guardian_policy,
    )
    explorer = endpoint(
        MatchRole.EXPLORER,
        Pose2D(3.5, 1.0, math.pi),
        explorer_policy,
        goal_node=0,
        goal_zone="sil-fixture:duel-goal",
    )
    guardian_manager = _manager(Role.GUARDIAN)
    explorer_manager = _manager(Role.EXPLORER)
    return (
        TwoRobotMatch(
            guardian,
            explorer,
            geometry,
            duel_stage_managers=(guardian_manager, explorer_manager),
        ),
        guardian_policy,
        explorer_policy,
        graph,
        geometry,
    )


def _loop_maze_duel():
    fixture = load_maze_bank(
        "sim/kinematic/scenarios/phase6_maze_bank.json"
    ).training_fixtures()[0]
    localization_epoch = fixture.topology.localization_epoch
    guardian_policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    explorer_policy = KinematicAutonomousPolicy(MatchRole.EXPLORER)

    def endpoint(role, start_rc, yaw, policy, goal=None):
        pose = Pose2D(*fixture.cell_pose(start_rc), yaw)
        return RoleEndpoint(
            role=role,
            namespace=f"/robot_{role.value}",
            plant=KinematicPlant(pose, max_linear_mps=0.5),
            sensor=LidarSensor(
                raycaster=FirstHitRaycaster(max_range_m=5.0),
                beam_count=720,
                max_range_m=5.0,
            ),
            policy=policy,
            radius_m=0.15,
            pose_estimator=SilDeadReckoningEstimator(
                PoseEstimate(
                    0,
                    "map",
                    "p56-clock",
                    localization_epoch,
                    pose,
                    0.0,
                    0.0,
                )
            ),
            topology_graph=fixture.topology,
            synthetic_goal_node_id=goal,
            synthetic_goal_zone_id=(
                "sil-fixture:phase6-loop-goal" if goal is not None else ""
            ),
        )

    width = len(fixture.rows[0])
    goal_node = (
        fixture.synthetic_goal_rc[0] * width + fixture.synthetic_goal_rc[1]
    )
    match = TwoRobotMatch(
        endpoint(
            MatchRole.GUARDIAN,
            fixture.guardian_start_rc,
            0.0,
            guardian_policy,
        ),
        endpoint(
            MatchRole.EXPLORER,
            fixture.explorer_start_rc,
            math.pi,
            explorer_policy,
            goal_node,
        ),
        fixture.geometry,
        duel_stage_managers=(
            _manager(Role.GUARDIAN, localization_epoch=localization_epoch),
            _manager(Role.EXPLORER, localization_epoch=localization_epoch),
        ),
    )
    return match, guardian_policy, explorer_policy


def _corner_policy_frame(*, dynamic_target=None):
    graph = TopologyGraph(
        0,
        0,
        "p56-localization",
        (
            TopologyNode(0, 0.5, 0.5, NodeKind.ANCHOR, 0.5),
            TopologyNode(1, 1.5, 0.5, NodeKind.PORTAL, 0.5),
            TopologyNode(2, 1.5, 1.5, NodeKind.PORTAL, 0.5),
        ),
        (
            TopologyEdge(0, 0, 1, ((0.5, 0.5), (1.5, 0.5)), 0.5),
            TopologyEdge(1, 1, 2, ((1.5, 0.5), (1.5, 1.5)), 0.5),
        ),
    )
    walls = (
        Segment((0.0, 0.0), (4.0, 0.0)),
        Segment((4.0, 0.0), (4.0, 4.0)),
        Segment((4.0, 4.0), (0.0, 4.0)),
        Segment((0.0, 4.0), (0.0, 0.0)),
    )
    targets = () if dynamic_target is None else (dynamic_target,)
    geometry = WorldGeometry(walls, targets)
    pose = Pose2D(1.0, 0.5, 0.0)
    sensor = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=6.0),
        beam_count=720,
        max_range_m=6.0,
    )
    stamp_ns = 150_000_000
    observation = sensor.observe(pose, stamp_ns / 1e9, geometry)
    manager = _manager(Role.GUARDIAN)
    state = manager.snapshot(
        now_ns=stamp_ns,
        map_version=graph.map_version,
        topology_version=graph.topology_version,
    )
    estimate = PoseEstimate(
        stamp_ns,
        "map",
        "p56-clock",
        "p56-localization",
        pose,
        0.0,
        0.0,
    )
    frame = SILPolicyInput(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        stamp_ns,
        observation,
        state,
        estimate,
        graph,
        0.15,
        0.15,
        walls,
    )
    path = PlannedPath(
        graph.map_version,
        graph.topology_version,
        graph.localization_epoch,
        0,
        2,
        (0, 1, 2),
        (0, 1),
        ((0.5, 0.5), (1.5, 0.5), (1.5, 1.5)),
        2.0,
    )
    goal = OptionGoal(
        "corner-option",
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        state.meta.stage_id,
        state.meta.clock_epoch,
        state.meta.localization_epoch,
        state.stage_ends_at_ns,
        graph.map_version,
        graph.topology_version,
        "p56-kinematic-parameters-v1",
        2,
        has_target_node=True,
        target_node_id=2,
    )
    return frame, path, goal


def test_guardian_executes_selected_search_through_plan_lease_and_safety():
    match, guardian_policy, _ = _runner(Role.GUARDIAN, _graph())
    observed_motion = False
    effect_completed = False
    for _ in range(120):
        for _ in range(3):
            tick = match.step(0.05)
        assert "truth" not in {field.name for field in fields(PolicyInput)}
        assert not hasattr(tick.guardian_observation, "target_id")
        if tick.guardian_command.linear_mps > 0.0:
            observed_motion = True
            assert tick.guardian_command.angular_rps == 0.0
            assert guardian_policy.last_trace.safety_decision in (0, 1, 2)
        if guardian_policy.last_trace.status == "option_effect_satisfied":
            effect_completed = True
            break
    assert observed_motion
    assert effect_completed
    assert guardian_policy.last_trace.selected_kind.name == "SEARCH_PORTAL"
    assert guardian_policy.last_trace.authority_reason == "effect_satisfied"
    assert guardian_policy.authority.execution_state.phase.name == "FINISHED"
    assert guardian_policy.authority.journal
    assert guardian_policy.last_trace.selected_kind.name == "SEARCH_PORTAL"
    active_goals = [
        record for record in guardian_policy.authority.journal
        if record.option_instance_id == "p56:guardian:p56-stage:search_portal:1:1"
    ]
    assert active_goals


def test_explorer_observe_safe_is_authorized_and_never_moves():
    match, _, explorer_policy = _runner(Role.EXPLORER, _graph())
    active_tick = None
    for _ in range(3):
        active_tick = match.step(0.05)
    assert active_tick is not None
    assert active_tick.explorer_command.linear_mps == 0.0
    assert active_tick.explorer_command.angular_rps == 0.0
    assert explorer_policy.last_trace.status == "observation_completed"
    assert explorer_policy.last_trace.selected_kind.name == "OBSERVE_SAFE"
    assert explorer_policy.last_trace.effect_evidence_id.startswith("lidar-observation:")
    assert not explorer_policy.authority.execution_state.candidate_authorized


def test_runner_binds_match_lease_to_shared_topology_versions():
    graph = _graph(map_version=7)
    match, guardian_policy, _ = _runner(Role.GUARDIAN, graph)
    for _ in range(3):
        tick = match.step(0.05)
    assert tick.match_state.meta.map_version == 7
    assert tick.match_state.meta.topology_version == graph.topology_version
    assert guardian_policy.last_trace.status == "candidate_supervised"


def test_blind_forward_sector_fails_closed_to_hold_safe():
    blind_sector = ((0.0, 0.20),)
    match, guardian_policy, _ = _runner(
        Role.GUARDIAN, _graph(), guardian_blind=blind_sector
    )
    for _ in range(3):
        tick = match.step(0.05)
    assert tick.guardian_command.linear_mps == 0.0
    assert tick.guardian_command.angular_rps == 0.0
    assert guardian_policy.last_trace.status == "hold_safe"
    assert guardian_policy.last_trace.selected_kind.name == "HOLD_SAFE"


def test_unreachable_portal_is_not_fabricated_as_a_path():
    match, guardian_policy, _ = _runner(Role.GUARDIAN, _graph(blocked=True))
    for _ in range(3):
        tick = match.step(0.05)
    assert tick.guardian_command.linear_mps == 0.0
    assert guardian_policy.last_trace.status.startswith(
        "no_supported_open_route_to_node"
    )


def test_corner_heading_uses_supervised_stop_turn_before_forward_motion():
    match, guardian_policy, _ = _runner(
        Role.GUARDIAN, _graph(diagonal=True)
    )
    for _ in range(3):
        tick = match.step(0.05)
    assert tick.guardian_command.linear_mps == 0.0
    assert tick.guardian_command.angular_rps > 0.0
    assert guardian_policy.last_trace.safety_decision == 1
    assert guardian_policy.last_trace.safety_reason == "NONE"
    assert guardian_policy.last_trace.status == "candidate_supervised"


def test_moving_robot_is_supervised_to_stop_before_stationary_alignment():
    policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    frame, path, goal = _corner_policy_frame()
    moving_estimate = replace(
        frame.pose_estimate,
        pose=Pose2D(1.45, 0.5, 0.0),
        linear_velocity_mps=0.2,
    )
    moving_frame = replace(frame, pose_estimate=moving_estimate)
    moving_candidate, moving_mode, *_ = policy._stop_turn_go_candidate(
        moving_frame, path, goal, 1
    )
    assert moving_mode is ControlMode.TRACK_PATH
    assert moving_candidate.linear_velocity_mps == 0.0
    assert moving_candidate.angular_velocity_rps == 0.0

    stopped_frame = replace(
        moving_frame,
        pose_estimate=replace(moving_estimate, linear_velocity_mps=0.0),
    )
    stopped_candidate, stopped_mode, *_ = policy._stop_turn_go_candidate(
        stopped_frame, path, goal, 1
    )
    assert stopped_mode is ControlMode.ALIGN
    assert stopped_candidate.linear_velocity_mps == 0.0
    assert stopped_candidate.angular_velocity_rps > 0.0


def test_sharp_corner_aligns_then_uses_supervised_curvature():
    match, policy, _ = _runner(Role.GUARDIAN, _graph(diagonal=True))
    saw_turn = False
    saw_translation = False
    saw_curved_translation = False
    for _ in range(160):
        tick = match.step(0.05)
        command = tick.guardian_command
        if command.angular_rps != 0.0:
            saw_turn = True
            assert policy.last_trace.safety_decision == 1
        if command.linear_mps > 0.0:
            saw_translation = True
            if command.angular_rps != 0.0:
                saw_curved_translation = True
                assert (
                    abs(command.linear_mps * command.angular_rps)
                    <= policy.profile.lateral_acceleration_max_mps2 + 1e-12
                )
        if saw_turn and saw_translation:
            break
    assert saw_turn
    assert saw_translation
    assert saw_curved_translation


def test_continuous_turn_is_allowed_only_for_sensor_certified_swept_arc():
    policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    frame, path, goal = _corner_policy_frame()
    (
        candidate,
        mode,
        _rotation_coverage,
        _rotation_clearance,
        curved_valid,
        clearance,
    ) = policy._stop_turn_go_candidate(frame, path, goal, 1)
    assert mode.name == "TRACK_PATH"
    assert curved_valid
    assert candidate.linear_velocity_mps > 0.0
    assert candidate.angular_velocity_rps > 0.0
    assert clearance > policy.profile.clearance_margin_m
    assert (
        abs(candidate.linear_velocity_mps * candidate.angular_velocity_rps)
        <= policy.profile.lateral_acceleration_max_mps2 + 1e-12
    )

    target = CircleTarget((1.244, 0.549), 0.15, "fixture-opponent")
    blocked_frame, _, _ = _corner_policy_frame(dynamic_target=target)
    blocked, blocked_clearance = policy._curved_path_clearance(
        blocked_frame, candidate
    )
    assert blocked
    assert blocked_clearance < clearance


def test_rotation_requires_fully_covered_scan_and_conservative_radial_clearance():
    match, policy, _ = _runner(
        Role.GUARDIAN,
        _graph(diagonal=True),
        guardian_blind=((math.pi / 2.0, 0.1),),
    )
    for _ in range(3):
        tick = match.step(0.05)
    assert tick.guardian_command == Actuation(0.0, 0.0)
    assert policy.last_trace.safety_reason == "OUTSIDE_COVERAGE"

    match, policy, _ = _runner(
        Role.GUARDIAN,
        _graph(diagonal=True),
        geometry=WorldGeometry((Segment((0.22, -0.3), (0.22, 0.3)),)),
    )
    for _ in range(3):
        tick = match.step(0.05)
    assert tick.guardian_command == Actuation(0.0, 0.0)
    assert policy.last_trace.safety_reason == "ROTATION_CLEARANCE_INSUFFICIENT"


def test_close_obstacle_limits_candidate_to_zero_before_actuation():
    match, guardian_policy, _ = _runner(
        Role.GUARDIAN,
        _graph(),
        geometry=WorldGeometry(
            (Segment((0.18, -0.5), (0.18, 0.5)),)
        ),
    )
    for _ in range(3):
        tick = match.step(0.05)
    assert tick.guardian_command.linear_mps == 0.0
    assert guardian_policy.last_trace.safety_decision == 2
    assert guardian_policy.last_trace.safety_reason == "COMMAND_LIMITED"


def test_pose_uncertainty_above_profile_bound_fails_closed():
    policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    manager = _manager(Role.GUARDIAN)
    state = manager.snapshot(now_ns=100_000_000)
    pose = Pose2D(0.0, 0.0, 0.0)
    observation = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=4.0),
        beam_count=360,
        max_range_m=4.0,
    ).observe(pose, 0.1, WorldGeometry(()))
    frame = PolicyInput(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        100_000_000,
        observation,
        state,
        PoseEstimate(
            100_000_000,
            "map",
            "p56-clock",
            "p56-localization",
            pose,
            0.11,
            0.0,
        ),
        _graph(),
        0.15,
    )
    assert policy(frame).linear_mps == 0.0
    assert policy.last_trace.status.startswith("no_supported_open_route_to_node")


def test_forward_clearance_uses_projection_not_oblique_ray_length():
    policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    count = 360
    ranges = [4.0] * count
    valid = [False] * count
    coverage = [True] * count
    index = 186
    angle = -math.pi + 2.0 * math.pi * index / count
    ranges[index] = 1.0
    valid[index] = True
    frame = PolicyInput(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        100_000_000,
        SensorObservation(
            0.1,
            "lidar_link",
            tuple(ranges),
            tuple(valid),
            tuple(coverage),
        ),
    )
    _, clearance, covered = policy._forward_clearance(frame)
    assert covered
    assert clearance == pytest.approx(math.cos(angle))


def test_negative_forward_speed_estimate_is_rejected_by_safety():
    policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    manager = _manager(Role.GUARDIAN)
    state = manager.snapshot(now_ns=100_000_000)
    pose = Pose2D(0.0, 0.0, 0.0)
    observation = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=4.0),
        beam_count=360,
        max_range_m=4.0,
    ).observe(pose, 0.1, WorldGeometry(()))
    frame = PolicyInput(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        100_000_000,
        observation,
        state,
        PoseEstimate(
            100_000_000,
            "map",
            "p56-clock",
            "p56-localization",
            pose,
            0.0,
            0.0,
            linear_velocity_mps=-0.1,
        ),
        _graph(),
        0.15,
    )
    assert policy(frame).linear_mps == 0.0
    assert policy.last_trace.safety_decision == 0
    assert policy.last_trace.safety_reason == "LIMITS_INVALID"


def test_nonzero_current_yaw_rate_is_braked_before_straight_travel():
    policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    manager = _manager(Role.GUARDIAN)
    state = manager.snapshot(now_ns=100_000_000)
    pose = Pose2D(0.0, 0.0, 0.0)
    observation = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=4.0),
        beam_count=360,
        max_range_m=4.0,
    ).observe(pose, 0.1, WorldGeometry(()))
    frame = PolicyInput(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        100_000_000,
        observation,
        state,
        PoseEstimate(
            100_000_000,
            "map",
            "p56-clock",
            "p56-localization",
            pose,
            0.0,
            0.0,
            angular_velocity_rps=0.1,
        ),
        _graph(),
        0.15,
    )
    assert policy(frame).linear_mps == 0.0
    assert policy.last_trace.command.angular_rps == pytest.approx(0.075)
    assert policy.last_trace.safety_decision == 1


def test_expired_match_state_lease_fails_closed():
    policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    manager = _manager(Role.GUARDIAN)
    state = manager.snapshot(now_ns=100_000_000)
    now_ns = state.meta.valid_until_ns + 1
    pose = Pose2D(0.0, 0.0, 0.0)
    observation = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=4.0),
        beam_count=360,
        max_range_m=4.0,
    ).observe(pose, now_ns / 1e9, WorldGeometry(()))
    frame = PolicyInput(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        now_ns,
        observation,
        state,
        PoseEstimate(
            now_ns,
            "map",
            "p56-clock",
            "p56-localization",
            pose,
            0.0,
            0.0,
        ),
        _graph(),
        0.15,
    )
    assert policy(frame).linear_mps == 0.0
    assert policy.last_trace.status == "hold_safe"


def test_stale_graph_identity_is_rejected_before_tactics():
    policy = KinematicAutonomousPolicy(MatchRole.GUARDIAN)
    manager = _manager(Role.GUARDIAN)
    state = manager.snapshot(now_ns=100_000_000)
    observation = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=4.0),
        beam_count=360,
        max_range_m=4.0,
    ).observe(Pose2D(0.0, 0.0, 0.0), 0.1, WorldGeometry(()))
    estimate = PoseEstimate(
        100_000_000,
        "map",
        "p56-clock",
        "p56-localization",
        Pose2D(0.0, 0.0, 0.0),
        0.0,
        0.0,
    )
    frame = PolicyInput(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        100_000_000,
        observation,
        state,
        estimate,
        _graph(map_version=1),
        0.15,
    )
    assert policy(frame).linear_mps == 0.0
    assert policy.last_trace.status == "incoherent_observation_or_versions"


def test_invalid_replan_input_revokes_active_execution_lease():
    match, guardian_policy, _ = _runner(Role.GUARDIAN, _graph())
    for _ in range(3):
        tick = match.step(0.05)
    active = guardian_policy.authority.execution_state
    assert active.candidate_authorized
    estimator = match.guardian.pose_estimator
    state = match._stage_manager.snapshot(now_ns=tick.stamp_ns)
    observation = match.guardian.sensor.observe(
        estimator.estimate.pose,
        tick.stamp_ns / 1e9,
        WorldGeometry(()),
    )
    stale_frame = PolicyInput(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        tick.stamp_ns,
        observation,
        state,
        estimator.estimate,
        _graph(map_version=1),
        0.15,
    )
    assert guardian_policy(stale_frame).linear_mps == 0.0
    assert guardian_policy.authority.execution_state.phase.name == "FINISHED"
    with pytest.raises(ValueError, match="cancelled option"):
        guardian_policy.authority.lease.set_candidate_authorized(
            active.active_option_instance_id,
            active.lease_generation,
            True,
        )


def test_pose_and_sensor_timestamps_must_match_policy_cycle():
    match, guardian_policy, _ = _runner(Role.GUARDIAN, _graph())
    tick = match.step(0.05)
    bad_frame = PolicyInput(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        tick.stamp_ns,
        tick.guardian_observation,
        tick.match_state,
        PoseEstimate(
            tick.stamp_ns - 1,
            "map",
            "p56-clock",
            "p56-localization",
            Pose2D(0.0, 0.0, 0.0),
            0.0,
            0.0,
        ),
        _graph(),
        0.15,
    )
    assert guardian_policy(bad_frame).linear_mps == 0.0
    assert guardian_policy.last_trace.status == "incoherent_observation_or_versions"


def test_simultaneous_sil_authorizes_both_roles_only_during_active_phase():
    match, _, _, _, _ = _corridor_duel()
    freeze = match.step(0.05)
    assert freeze.role_match_states is not None
    assert all(state.phase.name == "FREEZE" for state in freeze.role_match_states)
    assert freeze.guardian_command == Actuation(0.0, 0.0)
    assert freeze.explorer_command == Actuation(0.0, 0.0)

    active = None
    for _ in range(3):
        active = match.step(0.05)
    assert active is not None and active.role_match_states is not None
    guardian_state, explorer_state = active.role_match_states
    assert guardian_state.role == Role.GUARDIAN
    assert explorer_state.role == Role.EXPLORER
    assert guardian_state.phase.name == explorer_state.phase.name == "ACTIVE"
    assert guardian_state.motion_authorized and explorer_state.motion_authorized


def test_simultaneous_sil_moves_both_policies_using_only_scan_beliefs():
    match, guardian_policy, explorer_policy, _, _ = _corridor_duel()
    guardian_moved = False
    explorer_moved = False
    guardian_options = set()
    explorer_options = set()
    for _ in range(80):
        tick = match.step(0.05)
        guardian_moved |= tick.guardian_command.linear_mps > 0.0
        explorer_moved |= tick.explorer_command.linear_mps > 0.0
        if guardian_policy.last_trace.selected_kind is not None:
            guardian_options.add(guardian_policy.last_trace.selected_kind)
        if explorer_policy.last_trace.selected_kind is not None:
            explorer_options.add(explorer_policy.last_trace.selected_kind)
        assert guardian_policy.opponent_track is None or (
            guardian_policy.opponent_track.source_id == "sil-lidar:guardian"
        )
        assert explorer_policy.opponent_track is None or (
            explorer_policy.opponent_track.source_id == "sil-lidar:explorer"
        )
        assert not hasattr(tick.guardian_observation, "target_id")
        assert not hasattr(tick.explorer_observation, "target_id")
    assert guardian_moved
    assert explorer_moved
    assert guardian_policy.opponent_track is not None
    assert explorer_policy.opponent_track is not None
    assert guardian_policy.opponent_track.state.name == "TRACKED"
    assert explorer_policy.opponent_track.state.name == "TRACKED"
    assert OptionKind.PRESSURE_ROUTE in guardian_options
    assert explorer_options & {
        OptionKind.ADVANCE_BASE,
        OptionKind.ADVANCE_KNOWN_ROUTE,
        OptionKind.BREAK_LOS,
        OptionKind.KEEP_ESCAPE_ROUTE,
    }


def test_both_roles_make_progress_on_connected_loop_maze_fixture():
    match, guardian_policy, explorer_policy = _loop_maze_duel()
    guardian_start = match.guardian.plant.state.pose
    explorer_start = match.explorer.plant.state.pose
    for _ in range(40):
        tick = match.step(0.05)
        assert tick.guardian_command.linear_mps >= 0.0
        assert tick.explorer_command.linear_mps >= 0.0

    guardian_end = match.guardian.plant.state.pose
    explorer_end = match.explorer.plant.state.pose
    assert math.dist(
        (guardian_start.x_m, guardian_start.y_m),
        (guardian_end.x_m, guardian_end.y_m),
    ) > 0.2
    assert math.dist(
        (explorer_start.x_m, explorer_start.y_m),
        (explorer_end.x_m, explorer_end.y_m),
    ) > 0.2
    assert guardian_policy.last_trace.selected_kind is OptionKind.SEARCH_PORTAL
    assert explorer_policy.last_trace.selected_kind is OptionKind.ADVANCE_KNOWN_ROUTE
    assert guardian_policy.last_trace.safety_reason == "NONE"
    assert explorer_policy.last_trace.safety_reason == "NONE"


def test_duel_freeze_and_terminal_faults_stop_both_roles():
    match, _, _, _, _ = _corridor_duel()
    for _ in range(3):
        active = match.step(0.05)
    assert active.role_match_states is not None
    assert all(state.phase.name == "ACTIVE" for state in active.role_match_states)
    watchdog_stop = match.step(0.05, watchdog_healthy=False)
    assert watchdog_stop.guardian_command == Actuation(0.0, 0.0)
    assert watchdog_stop.explorer_command == Actuation(0.0, 0.0)

    for _ in range(200):
        stopped = match.step(0.05)
    assert stopped.role_match_states is not None
    assert all(state.phase.name == "TERMINAL" for state in stopped.role_match_states)
    assert stopped.guardian_command == Actuation(0.0, 0.0)
    assert stopped.explorer_command == Actuation(0.0, 0.0)


def test_duel_requires_role_scoped_stage_managers_and_explicit_fixture_goal():
    match, _, _, graph, geometry = _corridor_duel()
    with pytest.raises(ValueError, match="explicit explorer synthetic goal"):
        TwoRobotMatch(
            match.guardian,
            _endpoint(
                MatchRole.EXPLORER,
                Pose2D(2.0, 0.0, math.pi),
                KinematicAutonomousPolicy(MatchRole.EXPLORER),
                graph,
            ),
            geometry,
            duel_stage_managers=(
                _manager(Role.GUARDIAN),
                _manager(Role.EXPLORER),
            ),
        )
    with pytest.raises(ValueError, match="role-scoped managers"):
        TwoRobotMatch(
            match.guardian,
            match.explorer,
            geometry,
            duel_stage_managers=(
                _manager(Role.EXPLORER),
                _manager(Role.GUARDIAN),
            ),
        )
