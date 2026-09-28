"""Adversarial P5.6 observation-to-authority-to-safety SIL integration."""

from dataclasses import fields
import math

import pytest

from hsl_core.match import Role, StageManager, StageProfile
from hsl_core.topology import EdgeState, NodeKind, TopologyEdge, TopologyGraph, TopologyNode
from hsl_core.types import Pose2D
from sim.kinematic.autonomous import KinematicAutonomousPolicy
from sim.kinematic.common import PoseEstimate, Segment, SensorObservation, WorldGeometry
from sim.kinematic.match import MatchRole, PolicyInput, RoleEndpoint, TwoRobotMatch
from sim.kinematic.plant import KinematicPlant, SilDeadReckoningEstimator
from sim.kinematic.raycaster import FirstHitRaycaster
from sim.kinematic.sensors import LidarSensor


AUTH = "p56-sil-fixture-only"
CONFIG_HASH = "d" * 64


def _manager(role: Role) -> StageManager:
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
        clock_epoch="p56-clock",
        localization_epoch="p56-localization",
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


def test_guardian_executes_selected_search_through_plan_lease_and_safety():
    match, guardian_policy, _ = _runner(Role.GUARDIAN, _graph())
    observed_motion = False
    effect_completed = False
    for _ in range(120):
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


def test_nonzero_curvature_is_rejected_by_existing_safety_supervisor():
    match, guardian_policy, _ = _runner(
        Role.GUARDIAN, _graph(diagonal=True)
    )
    for _ in range(3):
        tick = match.step(0.05)
    assert tick.guardian_command.linear_mps == 0.0
    assert guardian_policy.last_trace.safety_decision == 0
    assert guardian_policy.last_trace.safety_reason == "LIMITS_INVALID"
    assert guardian_policy.last_trace.status == "candidate_supervised"


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


def test_nonzero_current_yaw_rate_is_not_certified_by_straight_safety_profile():
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
    assert policy.last_trace.status == "hold_safe"


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
