"""P5.5 lifecycle and fault-injection rehearsal for the kinematic SIL profile."""

from dataclasses import replace
import math

import pytest

from hsl_core.match import (
    EventEvidence,
    EventPredicate,
    Role,
    RuleEvent,
    StageManager,
    StagePhase,
    StageProfile,
    TerminalKind,
)
from hsl_core.types import Pose2D
from sim.kinematic.common import Actuation, Segment, WorldGeometry
from sim.kinematic.match import MatchRole, PolicyInput, RoleEndpoint, TwoRobotMatch
from sim.kinematic.plant import KinematicPlant
from sim.kinematic.raycaster import FirstHitRaycaster
from sim.kinematic.sensors import LidarSensor


AUTH = "sil-organizer-fixture"
HASH = "c" * 64
RESET_ACKS = frozenset(("actions", "execution", "plans", "candidates", "tracks", "memory"))
FREEZE_NS = 100_000_000
STAGE_NS = 300_000_000
DT = 0.05


def _manager():
    stage_ids = iter(("stage-2", "stage-3"))
    profile = StageProfile(
        profile_id="p55-development-only",
        score_profile_id="unapproved-fixture-only",
        freeze_duration_ns=FREEZE_NS,
        stage_duration_ns=STAGE_NS,
        lease_duration_ns=1_000_000_000,
        zone_update_window_ns=FREEZE_NS,
        max_request_records=64,
        max_event_records=16,
        max_audit_records=128,
        config_hash=HASH,
        source_id="p55-runner",
        source_session="p55-test",
        clock_epoch="sil-clock-epoch",
        localization_epoch="sil-localization-epoch",
        organizer_authorization_ref=AUTH,
        approved_authorization_refs=(AUTH,),
        approved_zone_frames=("map",),
        approved_memory_profile_ids=(),
    )
    return StageManager(
        profile,
        initial_stage_id="stage-1",
        stage_number=1,
        role=Role.GUARDIAN,
        stage_id_factory=lambda: next(stage_ids),
    )


def _start(manager, stage_id, now_ns):
    result = manager.start_stage(
        request_id=f"start-{stage_id}",
        expected_stage_id=stage_id,
        official_start_stamp_ns=now_ns,
        organizer_authorization_ref=AUTH,
        now_ns=now_ns,
    )
    assert result.accepted
    return result


def _endpoint(role, pose, policy, seed):
    return RoleEndpoint(
        role=role,
        namespace=f"/robot_{role.value}",
        plant=KinematicPlant(pose, max_linear_mps=0.5),
        sensor=LidarSensor(
            raycaster=FirstHitRaycaster(max_range_m=4.0),
            beam_count=8,
            max_range_m=4.0,
            noise_std_m=0.01,
            seed=seed,
        ),
        policy=policy,
        radius_m=0.15,
    )


def _match(
    manager,
    guardian_policy,
    explorer_policy,
    *,
    geometry=WorldGeometry(()),
):
    return TwoRobotMatch(
        _endpoint(
            MatchRole.GUARDIAN,
            Pose2D(-1.0, 0.0, 0.0),
            guardian_policy,
            21,
        ),
        _endpoint(
            MatchRole.EXPLORER,
            Pose2D(1.0, 0.0, math.pi),
            explorer_policy,
            22,
        ),
        geometry,
        stage_manager=manager,
    )


def _reset_to_explorer(manager, now_ns):
    result = manager.reset_stage(
        request_id="swap-role",
        expected_stage_id=manager.stage_id,
        stage_number=2,
        role=Role.EXPLORER,
        memory_profile_id="",
        organizer_authorization_ref=AUTH,
        now_ns=now_ns,
    )
    assert result.accepted
    assert not manager.acknowledge_reset(
        stage_id=result.stage_id, acknowledgements=frozenset(("actions",))
    )
    assert manager.acknowledge_reset(
        stage_id=result.stage_id, acknowledgements=RESET_ACKS
    )
    return result.stage_id


def test_cold_start_freeze_active_role_swap_terminal_and_reset_are_reproducible():
    manager = _manager()
    stage_id = manager.stage_id
    _start(manager, stage_id, 0)
    seen: list[tuple[MatchRole, StagePhase, bool]] = []

    def policy(frame: PolicyInput):
        assert frame.match_state is not None
        seen.append((frame.role, frame.match_state.phase, frame.match_state.motion_authorized))
        return Actuation(0.1, 0.0)

    match = _match(manager, policy, policy)

    # Freeze spans both samples below and exactly at its boundary.
    first = match.step(DT)
    second = match.step(DT)
    assert first.guardian_command == first.explorer_command == Actuation(0.0, 0.0)
    assert second.guardian_command == second.explorer_command == Actuation(0.0, 0.0)
    assert second.match_state is not None and second.match_state.phase == StagePhase.FREEZE
    assert match.guardian.plant.state.pose.x_m == pytest.approx(-1.0)
    assert match.explorer.plant.state.pose.x_m == pytest.approx(1.0)

    # Active authority is exclusive to the stage's assigned role.
    active_guardian = match.step(DT)
    assert active_guardian.match_state is not None
    assert active_guardian.match_state.phase == StagePhase.ACTIVE
    assert active_guardian.guardian_command == Actuation(0.1, 0.0)
    assert active_guardian.explorer_command == Actuation(0.0, 0.0)
    assert match.guardian.plant.state.pose.x_m > -1.0
    explorer_pose_before_swap = match.explorer.plant.state.pose

    # The final crossing step is conservatively held rather than commanding
    # beyond the deadline; the following observation latches TIMEOUT.
    crossing = match.step(0.2)
    assert crossing.guardian_command == crossing.explorer_command == Actuation(0.0, 0.0)
    terminal = match.step(DT)
    assert terminal.match_state is not None
    assert terminal.match_state.phase == StagePhase.TERMINAL
    assert terminal.match_state.terminal_kind == TerminalKind.TIMEOUT
    assert not terminal.match_state.motion_authorized

    new_stage_id = _reset_to_explorer(manager, match.stamp_ns)
    _start(manager, new_stage_id, match.stamp_ns)
    freeze_tick = match.step(DT)
    assert freeze_tick.guardian_command == freeze_tick.explorer_command == Actuation(0.0, 0.0)
    match.step(DT)
    active_explorer = match.step(DT)
    assert active_explorer.match_state is not None
    assert active_explorer.match_state.role == Role.EXPLORER
    assert active_explorer.guardian_command == Actuation(0.0, 0.0)
    assert active_explorer.explorer_command == Actuation(0.1, 0.0)
    assert match.explorer.plant.state.pose != explorer_pose_before_swap
    assert {role for role, _, _ in seen} == {MatchRole.GUARDIAN, MatchRole.EXPLORER}
    assert all(
        not authorized or phase == StagePhase.ACTIVE
        for _, phase, authorized in seen
    )
    # The fixture deliberately supplies no accepted start/target geometry.
    assert active_explorer.match_state.own_start_zone_id == ""
    assert active_explorer.match_state.target_zone_id == ""


def test_stage_deadline_straddling_tick_is_held_without_crossing_authority():
    manager = _manager()
    _start(manager, manager.stage_id, 0)
    match = _match(manager, lambda _: Actuation(0.4, 0.0), lambda _: Actuation(0.4, 0.0))
    match.step(0.1)
    active = match.step(0.05)
    assert active.match_state is not None and active.match_state.motion_authorized
    crossing = match.step(0.2)
    assert crossing.stamp_ns > STAGE_NS
    assert crossing.guardian_command == crossing.explorer_command == Actuation(0.0, 0.0)
    assert match.guardian.plant.state.pose.x_m == pytest.approx(-0.98)


def test_tick_cannot_outlive_the_match_state_lease():
    base = _manager()
    profile = replace(
        base.profile,
        freeze_duration_ns=1,
        zone_update_window_ns=1,
        lease_duration_ns=20_000_000,
    )
    manager = StageManager(
        profile,
        initial_stage_id="short-lease-stage",
        stage_number=1,
        role=Role.GUARDIAN,
        stage_id_factory=lambda: "unused-stage",
    )
    _start(manager, manager.stage_id, 0)
    match = _match(manager, lambda _: Actuation(0.2, 0.0), lambda _: Actuation(0.2, 0.0))
    match.step(1e-9)
    tick = match.step(DT)
    assert tick.match_state is not None and tick.match_state.motion_authorized
    assert tick.match_state.meta.valid_until_ns < tick.stamp_ns
    assert tick.guardian_command == tick.explorer_command == Actuation(0.0, 0.0)


def test_reset_ack_barrier_and_terminal_event_both_fail_closed():
    manager = _manager()
    reset = manager.reset_stage(
        request_id="cold-reset",
        expected_stage_id=manager.stage_id,
        stage_number=1,
        role=Role.GUARDIAN,
        memory_profile_id="",
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    assert reset.accepted
    assert not manager.acknowledge_reset(
        stage_id=reset.stage_id, acknowledgements=frozenset(("actions",))
    )
    match = _match(
        manager,
        lambda _: Actuation(0.2, 0.0),
        lambda _: Actuation(0.2, 0.0),
    )
    held = match.step(DT)
    assert held.match_state is not None
    assert held.match_state.phase == StagePhase.INIT
    assert not held.match_state.motion_authorized
    assert held.guardian_command == held.explorer_command == Actuation(0.0, 0.0)

    assert manager.acknowledge_reset(
        stage_id=reset.stage_id, acknowledgements=RESET_ACKS
    )
    _start(manager, reset.stage_id, match.stamp_ns)
    match.step(0.1)
    active = match.step(DT)
    assert active.match_state is not None and active.match_state.motion_authorized
    terminal_event = RuleEvent(
        event_id="approved-sil-capture",
        stage_id=manager.stage_id,
        clock_epoch=manager.profile.clock_epoch,
        localization_epoch=manager.profile.localization_epoch,
        kind=TerminalKind.CAPTURE,
        evidence=EventEvidence.OFFICIAL,
        predicate=EventPredicate.TRUE,
        event_time_lower_ns=160_000_000,
        event_time_upper_ns=170_000_000,
        input_ids=("approved-sil-adjudication-fixture",),
        authorization_ref=AUTH,
    )
    result = manager.resolve_event(terminal_event, now_ns=active.stamp_ns)
    assert result.accepted
    stopped = match.step(DT)
    assert stopped.match_state is not None
    assert stopped.match_state.phase == StagePhase.TERMINAL
    assert stopped.guardian_command == stopped.explorer_command == Actuation(0.0, 0.0)


def test_watchdog_fault_bypasses_both_policies_and_forces_zero_without_advancing_pose():
    manager = _manager()
    _start(manager, manager.stage_id, 0)
    calls = []

    def policy(frame):
        calls.append(frame.role)
        return Actuation(0.2, 0.0)

    match = _match(manager, policy, policy)
    match.step(0.1)
    match.step(0.05)
    before = (match.guardian.plant.state, match.explorer.plant.state)
    tick = match.step(0.05, watchdog_healthy=False)
    assert tick.guardian_command == tick.explorer_command == Actuation(0.0, 0.0)
    assert match.guardian.plant.state.pose == before[0].pose
    assert match.explorer.plant.state.pose == before[1].pose
    assert match.guardian.plant.state.stamp_s > before[0].stamp_s
    assert tick.match_state is not None
    assert tick.match_state.phase == StagePhase.ACTIVE
    assert calls == [MatchRole.GUARDIAN, MatchRole.EXPLORER] * 2


def test_stage_clock_fault_fails_closed_before_sensors_or_policies_run():
    manager = _manager()
    _start(manager, manager.stage_id, 0)
    match = _match(manager, lambda _: pytest.fail("policy must not run"), lambda _: pytest.fail("policy must not run"))
    manager.snapshot(now_ns=1)
    before = (match.guardian.plant.state, match.explorer.plant.state)
    with pytest.raises(ValueError, match="monotonic"):
        match.step(DT)
    assert match.stamp_ns == 0
    assert (match.guardian.plant.state, match.explorer.plant.state) == before


def test_sensor_failure_is_not_silently_replaced_and_tick_is_transactional(monkeypatch):
    match = _match(
        None,
        lambda _: Actuation(0.1, 0.0),
        lambda _: Actuation(0.1, 0.0),
    )
    before = (match.guardian.plant.state, match.explorer.plant.state)

    def fail_sensor(*args, **kwargs):
        raise RuntimeError("injected sensor failure")

    monkeypatch.setattr(match.explorer.sensor, "observe", fail_sensor)
    with pytest.raises(RuntimeError, match="injected sensor failure"):
        match.step(DT)
    assert match.stamp_ns == 0
    assert (match.guardian.plant.state, match.explorer.plant.state) == before


def test_referee_failure_is_not_silently_replaced_and_tick_is_transactional(monkeypatch):
    match = _match(
        None,
        lambda _: Actuation(0.1, 0.0),
        lambda _: Actuation(-0.1, 0.0),
    )
    before = (match.guardian.plant.state, match.explorer.plant.state)

    def fail_referee(*args, **kwargs):
        raise RuntimeError("injected referee failure")

    monkeypatch.setattr(match._referee, "evaluate", fail_referee)
    with pytest.raises(RuntimeError, match="injected referee failure"):
        match.step(DT)
    assert match.stamp_ns == 0
    assert (match.guardian.plant.state, match.explorer.plant.state) == before


def test_planner_failure_or_invalid_second_command_cannot_partially_step_either_robot():
    manager = _manager()
    _start(manager, manager.stage_id, 0)

    def fail_during_active(frame):
        if frame.stamp_ns < 150_000_000:
            return Actuation(0.0, 0.0)
        raise RuntimeError("injected planner failure")

    match = _match(
        manager,
        lambda _: Actuation(0.1, 0.0),
        fail_during_active,
    )
    match.step(0.1)
    match.step(0.05)
    before = (match.guardian.plant.state, match.explorer.plant.state)
    with pytest.raises(RuntimeError, match="injected planner failure"):
        match.step(DT)
    assert match.stamp_ns == 150_000_000
    assert (match.guardian.plant.state, match.explorer.plant.state) == before

    invalid = _match(
        None,
        lambda _: Actuation(0.1, 0.0),
        lambda _: Actuation(0.6, 0.0),
    )
    before_invalid = (invalid.guardian.plant.state, invalid.explorer.plant.state)
    with pytest.raises(ValueError, match="plant limit"):
        invalid.step(DT)
    assert invalid.stamp_ns == 0
    assert (invalid.guardian.plant.state, invalid.explorer.plant.state) == before_invalid


def test_sim_truth_is_not_automatically_scored_and_wrong_role_never_gets_motion():
    manager = _manager()
    _start(manager, manager.stage_id, 0)
    state = manager.snapshot(now_ns=0)
    capture = RuleEvent(
        event_id="synthetic-capture",
        stage_id=state.meta.stage_id,
        clock_epoch=state.meta.clock_epoch,
        localization_epoch=state.meta.localization_epoch,
        kind=TerminalKind.CAPTURE,
        evidence=EventEvidence.SIM_TRUTH,
        predicate=EventPredicate.TRUE,
        event_time_lower_ns=100_000_001,
        event_time_upper_ns=100_000_002,
        input_ids=("sim-trace",),
        authorization_ref=AUTH,
    )
    result = manager.resolve_event(capture, now_ns=102_000_000)
    assert not result.accepted
    assert result.reason == "simulation_truth_terminal_not_enabled"

    swapped = StageManager(
        manager.profile,
        initial_stage_id="explorer-stage",
        stage_number=1,
        role=Role.EXPLORER,
        stage_id_factory=lambda: "next-stage",
    )
    _start(swapped, "explorer-stage", 0)
    match = _match(swapped, lambda _: Actuation(0.1, 0.0), lambda _: Actuation(0.1, 0.0))
    tick = match.step(0.05)
    assert tick.guardian_command == tick.explorer_command == Actuation(0.0, 0.0)
    assert tick.truth.capture_interval is None


def test_same_seed_cold_runs_replay_observations_and_pose_trace_exactly():
    def run():
        manager = _manager()
        _start(manager, manager.stage_id, 0)
        match = _match(
            manager,
            lambda _: Actuation(0.1, 0.0),
            lambda _: Actuation(0.0, 0.0),
            geometry=WorldGeometry((Segment((0.0, -2.0), (0.0, 2.0)),)),
        )
        trace = []
        for _ in range(4):
            tick = match.step(DT)
            trace.append(
                (
                    tick.stamp_ns,
                    tick.guardian_observation.ranges_m,
                    tick.guardian_observation.valid_mask,
                    tick.explorer_observation.ranges_m,
                    tick.explorer_observation.valid_mask,
                    tick.guardian_command,
                    tick.explorer_command,
                    match.guardian.plant.state.pose,
                    match.explorer.plant.state.pose,
                )
            )
        return tuple(trace)

    first = run()
    assert first == run()
    assert any(any(mask) for row in first for mask in (row[2], row[4]))


@pytest.mark.parametrize("bad_watchdog", [0, 1, None, "healthy"])
def test_watchdog_health_requires_a_real_boolean(bad_watchdog):
    match = _match(None, lambda _: Actuation(0.0, 0.0), lambda _: Actuation(0.0, 0.0))
    with pytest.raises(ValueError, match="watchdog_healthy must be boolean"):
        match.step(DT, watchdog_healthy=bad_watchdog)
