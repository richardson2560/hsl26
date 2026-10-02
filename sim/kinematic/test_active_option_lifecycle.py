"""B acceptance: real caller + authority, exact dwell boundaries and revocation.

Controlled, coherent observations are development fixtures. Only candidate
generation is injected to expose decisions; selector/authority/control/safety
run normally. This does not claim physical or learned performance.
"""

from dataclasses import FrozenInstanceError, replace

import pytest

from hsl_core.match import Role, StagePhase, TerminalKind
from hsl_core.planning import CandidateEnvelope
from hsl_core.tactics import (
    OptionAuthority, OptionKind, OptionOutcome, OptionPhase, ROLE_FEATURES,
    TacticalGuard, UtilityProfile,
)
from hsl_core.topology import EdgeState, NodeKind, TopologyEdge, TopologyGraph, TopologyNode
from hsl_core.types import Pose2D
from sim.kinematic.autonomous import ActiveOption, KinematicAutonomousPolicy
from sim.kinematic.common import Actuation, PoseEstimate, WorldGeometry
from sim.kinematic.match import MatchRole, PolicyInput
from sim.kinematic.test_p56_autonomous import _manager


START_NS = 100_000_000
DWELL_NS = 250_000_000


def _fixture(monkeypatch, role=Role.GUARDIAN, *, dwell=DWELL_NS, hysteresis=0.125, audit_limit=128):
    feature = "pursuit_value" if role == Role.GUARDIAN else "escape_safety"
    profile = UtilityProfile("b-control", role, tuple(
        (f, float(f == feature)) for f in ROLE_FEATURES[role]
    ), 1)
    policy = KinematicAutonomousPolicy(
        MatchRole[role.name], utility_profile=profile,
        minimum_dwell_ns=dwell, hysteresis_delta_u=hysteresis,
    )
    policy.authority = OptionAuthority(policy.registry, max_audit_records=audit_limit)
    graph = TopologyGraph(0, 0, "p56-localization", (
        TopologyNode(0, 0.0, 0.0, NodeKind.ANCHOR, 0.5),
        TopologyNode(1, 2.0, 0.0, NodeKind.PORTAL, 0.5),
        TopologyNode(2, 0.0, 2.0, NodeKind.PORTAL, 0.5),
    ), (
        TopologyEdge(1, 0, 1, ((0.0, 0.0), (2.0, 0.0)), 0.5, state=EdgeState.OPEN),
        TopologyEdge(2, 0, 2, ((0.0, 0.0), (0.0, 2.0)), 0.5, state=EdgeState.OPEN),
    ))
    preferences = {1: 0.25, 2: 0.0}
    kinds = {i: OptionKind.SEARCH_PORTAL if role == Role.GUARDIAN else OptionKind.KEEP_ESCAPE_ROUTE
             for i in (1, 2)}
    urgencies = {1: 1, 2: 1}
    candidates = {1, 2}
    manager = _manager(role)

    def propose(frame, context, graph_arg, *args):
        proposals, paths = [], {}
        for target in sorted(candidates):
            path = policy._path_to_node(frame, graph_arg, target)
            if path is None:
                continue
            kind = kinds[target]
            guards = {
                OptionKind.SEARCH_PORTAL: (TacticalGuard.GRAPH_SEARCH_VIEWPOINT,),
                OptionKind.PRESSURE_ROUTE: (TacticalGuard.USEFUL_RIVAL,),
                OptionKind.KEEP_ESCAPE_ROUTE: (TacticalGuard.IMMEDIATE_TRAP_RISK, TacticalGuard.VERIFIED_ESCAPE_ROUTE),
            }[kind]
            proposal = policy._make_proposal(
                frame, kind, target_node_id=target, guards=guards, evidence="b-controlled-evidence",
                features={f: preferences[target] if f == feature else 0.0 for f in ROLE_FEATURES[role]},
                urgency_rank=urgencies[target] if role == Role.EXPLORER else None,
                urgency_evidence_id="b-common-threat" if role == Role.EXPLORER else "",
            )
            proposals.append(proposal)
            paths[target] = path
        return proposals, paths

    monkeypatch.setattr(policy, "_proposals", propose)
    return policy, manager, graph, preferences, kinds, urgencies, candidates


def _frame(policy, manager, graph, stamp_ns, *, pose=Pose2D(0.0, 0.0, 0.0)):
    # Use the production sensor builder; no fabricated tactical evaluations.
    from sim.kinematic.raycaster import FirstHitRaycaster
    from sim.kinematic.sensors import LidarSensor
    state = manager.snapshot(now_ns=stamp_ns)
    observation = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=4.0), beam_count=360, max_range_m=4.0,
    ).observe(pose, stamp_ns / 1e9, WorldGeometry(()))
    estimate = PoseEstimate(stamp_ns, "map", state.meta.clock_epoch,
                            state.meta.localization_epoch, pose, 0.0, 0.0)
    return PolicyInput(policy.role, f"/robot_{policy.role.value}", stamp_ns,
                       observation, state, estimate, graph, 0.15)


def _assert_partition(policy):
    trace = policy.last_trace
    assert trace.diagnostic_valid, trace.diagnostic_error
    assert 0 <= trace.n_choice <= trace.n_available <= trace.n_active <= 1
    assert trace.n_choice + trace.n_forced + trace.n_empty == trace.n_active


def _assert_revoked(policy, active, frame):
    assert policy._active is None
    assert not policy.authority.execution_state.candidate_authorized
    result = policy.authority.result(active.action_id)
    assert result is not None and result.started_at_ns == active.started_at_ns
    assert result.finished_at_ns >= result.started_at_ns
    with pytest.raises(ValueError):
        policy.authority.lease.set_candidate_authorized(active.option_instance_id, active.lease_generation, True)
    # A real previously valid command envelope cannot be re-admitted.
    candidate, *_ = policy._stop_turn_go_candidate(frame, active.path, active.goal, active.lease_generation)
    with pytest.raises(ValueError):
        policy.authority.lease.admit(CandidateEnvelope(candidate, active.path, active.option_instance_id),
                                    now_s=frame.stamp_ns / 1e9, generation=active.lease_generation)


@pytest.mark.parametrize("role", [Role.GUARDIAN, Role.EXPLORER])
def test_exact_dwell_boundary_uses_admission_time_and_resets_only_on_new_instance(monkeypatch, role):
    policy, manager, graph, preferences, *_ = _fixture(monkeypatch, role)
    policy(_frame(policy, manager, graph, START_NS))
    original = policy._active
    assert isinstance(original, ActiveOption) and original.goal.target_node_id == 1
    assert original.started_at_ns == START_NS
    preferences[2] = 0.75
    policy(_frame(policy, manager, graph, START_NS + DWELL_NS - 1))
    assert policy.last_trace.selection_reason == "minimum_dwell"
    assert policy._active.started_at_ns == START_NS
    assert policy._active.action_id == original.action_id
    assert policy._active.lease_generation > original.lease_generation
    _assert_partition(policy)
    policy(_frame(policy, manager, graph, START_NS + DWELL_NS))
    switched = policy._active
    assert policy.last_trace.selection_reason == "utility_improvement_exceeds_hysteresis"
    assert switched.goal.target_node_id == 2
    assert switched.option_instance_id != original.option_instance_id
    assert switched.action_id != original.action_id
    assert switched.started_at_ns == START_NS + DWELL_NS
    result = policy.authority.result(original.action_id)
    assert result.outcome == OptionOutcome.CANCELED
    assert result.started_at_ns == START_NS and result.elapsed_ns == DWELL_NS
    with pytest.raises(ValueError):
        policy.authority.lease.set_candidate_authorized(original.option_instance_id, original.lease_generation, True)
    policy(_frame(policy, manager, graph, START_NS + DWELL_NS + 50_000_000))
    assert policy._active.started_at_ns == switched.started_at_ns
    assert policy._active.action_id == switched.action_id
    _assert_partition(policy)


@pytest.mark.parametrize("replans", [1, 5, 12])
def test_replans_preserve_start_goal_action_and_follow_authority_generation(monkeypatch, replans):
    policy, manager, graph, *_ = _fixture(monkeypatch, audit_limit=2)
    first_frame = _frame(policy, manager, graph, START_NS)
    policy(first_frame)
    original = policy._active
    old_candidate, *_ = policy._stop_turn_go_candidate(first_frame, original.path, original.goal, original.lease_generation)
    for tick in range(1, replans + 1):
        frame = _frame(policy, manager, graph, START_NS + tick * 50_000_000,
                       pose=Pose2D(0.01 * tick, 0.0, 0.0))
        policy(frame)
        active = policy._active
        assert active.started_at_ns == original.started_at_ns
        assert active.action_id == original.action_id and active.goal is original.goal
        assert active.lease_generation == policy.authority.execution_state.lease_generation
        assert active.lease_generation == original.lease_generation + tick
        assert active.path.polyline_xy_m[0] != original.path.polyline_xy_m[0]
        _assert_partition(policy)
    assert policy.authority.audit_dropped_count > 0
    with pytest.raises(ValueError, match="generation"):
        policy.authority.lease.admit(CandidateEnvelope(old_candidate, original.path, original.option_instance_id),
                                    now_s=frame.stamp_ns / 1e9, generation=original.lease_generation)
    # Cancellation works even after the admission record leaves the bounded journal.
    policy(_frame(policy, manager, graph, frame.stamp_ns + 1, pose=Pose2D(0.01 * replans, 0.0, 0.0)))
    current = policy._active
    policy._cancel_active(_frame(policy, manager, graph, frame.stamp_ns + 2), graph)
    assert policy.authority.result(current.action_id).started_at_ns == START_NS
    assert policy._active is None


def test_hysteresis_blocks_at_and_below_delta_even_after_dwell(monkeypatch):
    policy, manager, graph, preferences, *_ = _fixture(monkeypatch, hysteresis=0.25)
    policy(_frame(policy, manager, graph, START_NS))
    original = policy._active
    for tick, value in enumerate((0.25, 0.375, 0.5), start=1):
        preferences[2] = value
        policy(_frame(policy, manager, graph, START_NS + DWELL_NS + tick * 50_000_000))
        assert policy._active.option_instance_id == original.option_instance_id
        assert policy.last_trace.selection_reason in ("current_remains_best", "utility_tie", "hysteresis")
    preferences[2] = 0.75
    policy(_frame(policy, manager, graph, START_NS + DWELL_NS + 200_000_000))
    assert policy._active.goal.target_node_id == 2


def test_priority_preempts_before_dwell_even_with_lower_utility(monkeypatch):
    policy, manager, graph, preferences, kinds, *_ = _fixture(monkeypatch, dwell=5_000_000_000)
    policy(_frame(policy, manager, graph, START_NS))
    original = policy._active
    kinds[2] = OptionKind.PRESSURE_ROUTE
    preferences[2] = 0.0
    policy(_frame(policy, manager, graph, START_NS + 1))
    assert policy.last_trace.selection_reason == "higher_priority_preemption"
    assert policy._active.goal.target_node_id == 2
    assert policy.authority.result(original.action_id).elapsed_ns == 1


def test_urgency_preempts_before_dwell_even_with_lower_utility(monkeypatch):
    policy, manager, graph, preferences, _, urgencies, _ = _fixture(monkeypatch, Role.EXPLORER, dwell=5_000_000_000)
    policy(_frame(policy, manager, graph, START_NS))
    original = policy._active
    urgencies[2] = 200
    preferences[2] = 0.0
    policy(_frame(policy, manager, graph, START_NS + 1))
    assert policy.last_trace.selection_reason == "higher_urgency_preemption"
    assert policy._active.goal.target_node_id == 2
    assert policy.authority.result(original.action_id).elapsed_ns == 1


def test_zero_dwell_switches_immediately_and_missing_current_is_not_protected(monkeypatch):
    policy, manager, graph, preferences, _, _, candidates = _fixture(monkeypatch, dwell=0)
    policy(_frame(policy, manager, graph, START_NS))
    preferences[2] = 0.75
    policy(_frame(policy, manager, graph, START_NS + 1))
    assert policy._active.goal.target_node_id == 2
    policy.selector.minimum_dwell_ns = 5_000_000_000
    candidates.remove(2)
    policy(_frame(policy, manager, graph, START_NS + 2))
    assert policy._active.goal.target_node_id == 1


@pytest.mark.parametrize("fault", ["freeze", "terminal", "lease", "missing", "rewind", "safety"])
def test_hard_stops_revoke_before_dwell_and_do_not_retain_cached_option(monkeypatch, fault):
    policy, manager, graph, *_ = _fixture(monkeypatch, dwell=5_000_000_000)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    frame = _frame(policy, manager, graph, START_NS + 50_000_000)
    if fault in ("freeze", "terminal"):
        phase = StagePhase[fault.upper()]
        frame = replace(frame, match_state=replace(frame.match_state, phase=phase, motion_authorized=False,
                         terminal_kind=TerminalKind.NONE if phase == StagePhase.FREEZE else TerminalKind.OFFICIAL_ABORT))
    elif fault == "lease":
        frame = replace(frame, match_state=replace(frame.match_state,
                         meta=replace(frame.match_state.meta, observation_stamp_ns=START_NS,
                                      state_stamp_ns=START_NS, publication_stamp_ns=START_NS,
                                      valid_until_ns=frame.stamp_ns)))
    elif fault == "missing":
        frame = replace(frame, topology_graph=None)
    elif fault == "rewind":
        frame = replace(frame, stamp_ns=START_NS - 1)
    else:
        policy._safety_latched = True
    assert policy(frame) == Actuation(0.0, 0.0)
    _assert_revoked(policy, active, _frame(policy, manager, graph, START_NS + 50_000_000))
    _assert_partition(policy)


@pytest.mark.parametrize("identity", ["stage", "localization", "map", "topology"])
def test_new_context_closes_old_instance_then_requires_new_admission(monkeypatch, identity):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    policy(_frame(policy, manager, graph, START_NS))
    old = policy._active
    policy._completed_nodes.add(999)
    frame = _frame(policy, manager, graph, START_NS + 50_000_000)
    state, estimate, new_graph = frame.match_state, frame.pose_estimate, graph
    if identity == "stage":
        state = replace(state, meta=replace(state.meta, stage_id="b-next-stage"))
    elif identity == "localization":
        state = replace(state, meta=replace(state.meta, localization_epoch="b-next-localization"))
        estimate = replace(estimate, localization_epoch="b-next-localization")
        new_graph = replace(graph, localization_epoch="b-next-localization")
    else:
        field = identity + "_version"
        state = replace(state, meta=replace(state.meta, **{field: 1}))
        new_graph = replace(graph, **{field: 1})
    changed = replace(frame, match_state=state, pose_estimate=estimate, topology_graph=new_graph)
    assert policy(changed) == Actuation(0.0, 0.0)
    assert not policy.last_trace.selector_invoked
    assert policy._active is None and not policy._completed_nodes
    assert policy.authority.result(old.action_id).started_at_ns == START_NS
    next_stamp = changed.stamp_ns + 50_000_000
    next_frame = replace(changed, stamp_ns=next_stamp,
        observation=replace(changed.observation, stamp_s=next_stamp / 1e9),
        pose_estimate=replace(changed.pose_estimate, stamp_ns=next_stamp))
    policy(next_frame)
    assert policy._active is not None
    assert policy._active.started_at_ns == next_stamp
    assert policy._active.option_instance_id != old.option_instance_id
    assert policy._active.action_id != old.action_id
    assert policy._active.lease_generation > old.lease_generation
    assert policy._active.goal.localization_epoch == new_graph.localization_epoch
    assert policy._active.goal.map_version == new_graph.map_version
    _assert_partition(policy)


@pytest.mark.parametrize("new_stamp", [1, START_NS + 50_000_000])
def test_clock_epoch_change_revokes_with_monotonic_old_clock_and_stays_disarmed(monkeypatch, new_stamp):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    frame = _frame(policy, manager, graph, START_NS + 50_000_000)
    changed = replace(frame, stamp_ns=new_stamp,
        match_state=replace(frame.match_state, meta=replace(frame.match_state.meta, clock_epoch="new-clock")))
    policy._completed_nodes.add(999)
    assert policy(changed) == Actuation(0.0, 0.0)
    assert policy.last_trace.status == "clock_epoch_changed_requires_policy_reset"
    _assert_revoked(policy, active, frame)
    assert not policy._completed_nodes and policy._route_graph is None
    result = policy.authority.result(active.action_id)
    assert result.finished_at_ns == START_NS
    assert policy(_frame(policy, manager, graph, START_NS + 100_000_000)) == Actuation(0.0, 0.0)
    assert policy._active is None and not policy.last_trace.selector_invoked


def test_effect_success_matches_authority_and_next_instance_has_new_start(monkeypatch):
    policy, manager, graph, _, _, _, candidates = _fixture(monkeypatch)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    # Avoid selecting a zero remaining path: allow the production proposal to
    # carry its last valid path at the effect checkpoint.
    original = policy._proposals
    def at_endpoint(frame, *args):
        proposals, paths = original(frame, *args)
        p = policy._make_proposal(frame, OptionKind.SEARCH_PORTAL, target_node_id=1,
            guards=(TacticalGuard.GRAPH_SEARCH_VIEWPOINT,), evidence="b-effect",
            features={f: float(f == "pursuit_value") for f in ROLE_FEATURES[Role.GUARDIAN]})
        return [p], {1: active.path}
    monkeypatch.setattr(policy, "_proposals", at_endpoint)
    end = START_NS + 50_000_000
    assert policy(_frame(policy, manager, graph, end, pose=Pose2D(2.0, 0.0, 0.0))) == Actuation(0.0, 0.0)
    assert policy.last_trace.status == "option_effect_satisfied"
    result = policy.authority.result(active.action_id)
    assert result.outcome == OptionOutcome.SUCCESS and result.elapsed_ns == end - START_NS
    assert policy._active is None and 1 in policy._completed_nodes
    candidates.remove(1)
    monkeypatch.setattr(policy, "_proposals", original)
    policy(_frame(policy, manager, graph, end + 50_000_000))
    assert policy._active.goal.target_node_id == 2
    assert policy._active.started_at_ns == end + 50_000_000


def test_authority_finished_externally_does_not_leave_cache_or_double_cancel(monkeypatch):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    frame = _frame(policy, manager, graph, START_NS + 50_000_000)
    policy.authority.fail(active.action_id, policy._context(frame, graph, safety_stop=True, path_valid=False),
                          outcome=OptionOutcome.INTERNAL_ERROR, reason="external-executor-failure")
    policy._cancel_active(frame, graph)
    assert policy._active is None
    assert policy.authority.result(active.action_id).reason == "external-executor-failure"
    policy(_frame(policy, manager, graph, START_NS + 100_000_000))
    assert policy._active.started_at_ns == START_NS + 100_000_000


def test_admission_rejected_does_not_create_start_and_goal_mutation_is_revoked(monkeypatch):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    original = policy.authority.submit
    monkeypatch.setattr(policy.authority, "submit", lambda *a, **k: type("Denied", (), {
        "accepted": False, "reason": "rejected-by-authority"})())
    policy(_frame(policy, manager, graph, START_NS))
    assert policy._active is None and policy.last_trace.status == "option_not_admitted"
    monkeypatch.setattr(policy.authority, "submit", original)
    policy(_frame(policy, manager, graph, START_NS + 50_000_000))
    active = policy._active
    original_propose = policy._proposals
    def mutated(*args):
        proposals, paths = original_propose(*args)
        proposals = [replace(p, goal=replace(p.goal, position_tolerance_m=0.2))
                     if p.goal.option_instance_id == active.option_instance_id else p for p in proposals]
        return proposals, paths
    monkeypatch.setattr(policy, "_proposals", mutated)
    frame = _frame(policy, manager, graph, START_NS + 100_000_000)
    assert policy(frame) == Actuation(0.0, 0.0)
    assert policy.last_trace.status == "active_goal_mutation_rejected"
    _assert_revoked(policy, active, frame)


def test_positive_control_distinguishes_old_reset_clock_from_real_dwell(monkeypatch):
    def drive(reset_start):
        policy, manager, graph, preferences, *_ = _fixture(monkeypatch)
        original = policy.selector.select
        ages = []
        def observe(snapshot, **kwargs):
            if kwargs["current_started_at_ns"] is not None:
                if reset_start:
                    kwargs["current_started_at_ns"] = snapshot.context.now_ns
                ages.append(snapshot.context.now_ns - kwargs["current_started_at_ns"])
            return original(snapshot, **kwargs)
        monkeypatch.setattr(policy.selector, "select", observe)
        policy(_frame(policy, manager, graph, START_NS))
        preferences[2] = 0.75
        policy(_frame(policy, manager, graph, START_NS + DWELL_NS))
        return policy._active.goal.target_node_id, policy.last_trace.selection_reason, ages
    assert drive(False) == (2, "utility_improvement_exceeds_hysteresis", [DWELL_NS])
    assert drive(True) == (1, "minimum_dwell", [0])


def test_active_record_is_frozen_and_rejects_invalid_start_or_generation(monkeypatch):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    with pytest.raises(FrozenInstanceError):
        active.started_at_ns = 0
    for changes in ({"started_at_ns": -1}, {"started_at_ns": True}, {"lease_generation": 0},
                    {"option_instance_id": "wrong-instance"}, {"path": replace(active.path, map_version=1)}):
        with pytest.raises(ValueError):
            replace(active, **changes)


@pytest.mark.parametrize("replan", [False, True])
def test_authority_can_finish_during_plan_validation_without_candidate_or_double_cancel(monkeypatch, replan):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    if replan:
        policy(_frame(policy, manager, graph, START_NS))
        active = policy._active
    original = policy.authority.mark_executing
    actions = []
    def reject_path(action_id, context):
        actions.append(action_id)
        return original(action_id, replace(context, path_valid=False))
    monkeypatch.setattr(policy.authority, "mark_executing", reject_path)
    def no_candidate(*args, **kwargs):
        pytest.fail("finished authority must never generate a candidate")
    monkeypatch.setattr(policy, "_execute_candidate", no_candidate)
    stamp = START_NS + 50_000_000 if replan else START_NS
    assert policy(_frame(policy, manager, graph, stamp)) == Actuation(0.0, 0.0)
    result = policy.authority.result(actions[0])
    assert result.outcome == OptionOutcome.FEASIBILITY_LOST
    assert result.started_at_ns == START_NS and result.finished_at_ns == stamp
    assert policy._active is None and not policy.authority.execution_state.candidate_authorized
    assert policy.last_trace.status == "option_not_executing"
    if replan:
        assert actions == [active.action_id]
    _assert_partition(policy)


def test_supervisor_stop_revokes_and_latch_survives_navigation_reset(monkeypatch):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    original = policy.supervisor.evaluate
    monkeypatch.setattr(policy.supervisor, "evaluate",
                        lambda snapshot, *args: original(replace(snapshot, coverage_valid=False), *args))
    frame = _frame(policy, manager, graph, START_NS + 50_000_000)
    assert policy(frame) == Actuation(0.0, 0.0)
    assert policy._safety_latched
    _assert_revoked(policy, active, frame)
    policy._reset_navigation_context()
    monkeypatch.setattr(policy.supervisor, "evaluate", original)
    assert policy(_frame(policy, manager, graph, START_NS + 100_000_000)) == Actuation(0.0, 0.0)
    assert policy._safety_latched and policy._active is None
    _assert_partition(policy)


@pytest.mark.parametrize("already_active", [False, True])
def test_inconsistent_path_closes_accepted_planning_lease_or_active_replan(monkeypatch, already_active):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    if already_active:
        policy(_frame(policy, manager, graph, START_NS))
        active = policy._active
    original = policy._proposals
    def mismatched(*args):
        proposals, paths = original(*args)
        paths[1] = replace(paths[1], map_version=1)
        return proposals, paths
    monkeypatch.setattr(policy, "_proposals", mismatched)
    actions = []
    submit = policy.authority.submit
    def capture(action_id, *args):
        actions.append(action_id)
        return submit(action_id, *args)
    monkeypatch.setattr(policy.authority, "submit", capture)
    stamp = START_NS + 50_000_000 if already_active else START_NS
    assert policy(_frame(policy, manager, graph, stamp)) == Actuation(0.0, 0.0)
    action_id = active.action_id if already_active else actions[0]
    result = policy.authority.result(action_id)
    assert result.outcome == OptionOutcome.CANCELED and result.started_at_ns == START_NS
    assert policy._active is None and not policy.authority.execution_state.candidate_authorized
    assert not policy.last_trace.diagnostic_valid  # Missing evidence must remain explicit.
    assert policy.last_trace.status == ("authority_or_planning_rejected" if already_active
                                         else "option_admission_or_plan_failed")


def test_missing_selected_path_revokes_existing_execution(monkeypatch):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    original = policy._proposals
    def no_path(*args):
        proposals, _ = original(*args)
        return proposals, {}
    monkeypatch.setattr(policy, "_proposals", no_path)
    frame = _frame(policy, manager, graph, START_NS + 50_000_000)
    assert policy(frame) == Actuation(0.0, 0.0)
    assert policy.last_trace.status == "path_unavailable" and not policy.last_trace.diagnostic_valid
    _assert_revoked(policy, active, frame)


def test_endpoint_without_authority_success_does_not_mark_effect_complete(monkeypatch):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    def at_endpoint(frame, *args):
        proposal = policy._make_proposal(frame, active.goal.kind, target_node_id=1,
            guards=(TacticalGuard.GRAPH_SEARCH_VIEWPOINT,), evidence="b-effect",
            features={f: float(f == "pursuit_value") for f in ROLE_FEATURES[Role.GUARDIAN]})
        return [proposal], {1: active.path}
    def fail_effect(context):
        policy.authority.fail(active.action_id, context,
                              outcome=OptionOutcome.INTERNAL_ERROR, reason="effect-not-confirmed")
        return policy.authority.execution_state
    monkeypatch.setattr(policy, "_proposals", at_endpoint)
    monkeypatch.setattr(policy.authority, "tick", fail_effect)
    stamp = START_NS + 50_000_000
    assert policy(_frame(policy, manager, graph, stamp, pose=Pose2D(2.0, 0.0, 0.0))) == Actuation(0.0, 0.0)
    assert policy.last_trace.status == "option_effect_not_confirmed"
    assert 1 not in policy._completed_nodes and policy._active is None
    assert policy.authority.result(active.action_id).outcome == OptionOutcome.INTERNAL_ERROR
    _assert_partition(policy)


def test_real_stage_timeout_revokes_before_even_long_dwell(monkeypatch):
    policy, manager, graph, *_ = _fixture(monkeypatch, dwell=20_000_000_000)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    frame = _frame(policy, manager, graph, 10_000_000_000)
    assert frame.match_state.phase == StagePhase.TERMINAL
    assert frame.match_state.terminal_kind == TerminalKind.TIMEOUT
    assert policy(frame) == Actuation(0.0, 0.0)
    _assert_revoked(policy, active, frame)
    _assert_partition(policy)


@pytest.mark.parametrize("version", ["map_version", "topology_version"])
def test_version_change_clears_old_completion_and_route_cache_even_without_active_option(monkeypatch, version):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    policy(_frame(policy, manager, graph, START_NS))
    frame = _frame(policy, manager, graph, START_NS + 50_000_000)
    policy._cancel_active(frame, graph)
    policy._completed_nodes.add(1)
    assert policy._route_graph is not None
    changed_graph = replace(graph, **{version: 1})
    changed_state = replace(frame.match_state, meta=replace(frame.match_state.meta, **{version: 1}))
    assert policy(replace(frame, topology_graph=changed_graph, match_state=changed_state)) == Actuation(0.0, 0.0)
    assert policy.last_trace.status == "route_versions_changed"
    assert policy._route_graph is None and not policy._completed_nodes
    assert policy._active is None and not policy.last_trace.selector_invoked


@pytest.mark.parametrize("mismatch", ["target", "versions", "geometry"])
def test_mismatched_path_endpoint_never_fabricates_success_for_admitted_goal(monkeypatch, mismatch):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    first_frame = _frame(policy, manager, graph, START_NS)
    policy(first_frame)
    active = policy._active
    if mismatch == "target":
        bad_path = policy._path_to_node(first_frame, graph, 2)
    elif mismatch == "versions":
        bad_path = replace(active.path, map_version=1)
    else:
        bad_path = replace(active.path, polyline_xy_m=((0.0, 0.0), (0.0, 2.0)))
    def mismatched(frame, *args):
        proposal = policy._make_proposal(frame, active.goal.kind, target_node_id=1,
            guards=(TacticalGuard.GRAPH_SEARCH_VIEWPOINT,), evidence="b-mismatched-endpoint",
            features={f: float(f == "pursuit_value") for f in ROLE_FEATURES[Role.GUARDIAN]})
        return [proposal], {1: bad_path}
    monkeypatch.setattr(policy, "_proposals", mismatched)
    x, y = bad_path.polyline_xy_m[-1]
    frame = _frame(policy, manager, graph, START_NS + 50_000_000, pose=Pose2D(x, y, 0.0))
    assert policy(frame) == Actuation(0.0, 0.0)
    assert policy.last_trace.status == "authority_or_planning_rejected"
    assert not policy.last_trace.diagnostic_valid
    assert 1 not in policy._completed_nodes
    assert policy.authority.result(active.action_id).outcome == OptionOutcome.CANCELED
    _assert_revoked(policy, active, _frame(policy, manager, graph, frame.stamp_ns))


def test_geometric_identity_tolerance_does_not_expand_admitted_arrival_tolerance(monkeypatch):
    policy, manager, graph, *_ = _fixture(monkeypatch)
    policy.profile = replace(policy.profile, goal_tolerance_m=0.125)
    policy(_frame(policy, manager, graph, START_NS))
    active = policy._active
    # Endpoint discrepancy is inside the 1e-9 identity tolerance. All values
    # below are binary-exact, including the strictly outside target pose.
    shifted_path = replace(active.path, polyline_xy_m=((0.0, 0.0), (2.0 - 2**-30, 0.0)))
    def shifted(frame, *args):
        proposal = policy._make_proposal(frame, active.goal.kind, target_node_id=1,
            guards=(TacticalGuard.GRAPH_SEARCH_VIEWPOINT,), evidence="b-arrival-boundary",
            features={f: float(f == "pursuit_value") for f in ROLE_FEATURES[Role.GUARDIAN]})
        return [proposal], {1: shifted_path}
    monkeypatch.setattr(policy, "_proposals", shifted)
    outside = _frame(policy, manager, graph, START_NS + 50_000_000,
                     pose=Pose2D(1.875 - 2**-31, 0.0, 0.0))
    policy(outside)
    assert policy.authority.result(active.action_id) is None
    assert policy._active.option_instance_id == active.option_instance_id
    assert 1 not in policy._completed_nodes
    _assert_partition(policy)
    boundary = _frame(policy, manager, graph, START_NS + 100_000_000,
                      pose=Pose2D(1.875, 0.0, 0.0))
    assert policy(boundary) == Actuation(0.0, 0.0)
    assert policy.authority.result(active.action_id).outcome == OptionOutcome.SUCCESS
    assert policy._active is None and 1 in policy._completed_nodes
