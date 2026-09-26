"""Adversarial P5.2 tests for option admission, lifecycle and authority revocation."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace

import pytest

from hsl_core.match import Role, StagePhase
from hsl_core.planning.astar import PlannedPath
from hsl_core.planning.execution import CandidateEnvelope, ExecutionLease
from hsl_core.tactics import (
    OptionDefinition,
    OptionAuthority,
    OptionContext,
    OptionGoal,
    OptionKind,
    OptionOutcome,
    OptionPhase,
    OptionRegistry,
)
from hsl_core.types import MotionCandidate, Pose2D


def _context(**changes):
    values = dict(
        now_ns=10_000_000_000,
        stage_id="stage-1",
        stage_phase=StagePhase.ACTIVE,
        stage_ends_at_ns=100_000_000_000,
        role=Role.GUARDIAN,
        clock_epoch="clock-1",
        localization_epoch="loc-1",
        map_version=7,
        topology_version=9,
        motion_authorized=True,
        lease_valid=True,
        safety_stop=False,
        accepted_goal_zone_ids=("target-zone", "own-zone"),
        path_valid=False,
        effect_satisfied=False,
        feasibility_lost=False,
    )
    values.update(changes)
    return OptionContext(**values)


def _goal(kind=OptionKind.SEARCH_PORTAL, **changes):
    role = Role.EXPLORER if kind in (
        OptionKind.ADVANCE_BASE,
        OptionKind.BREAK_LOS,
        OptionKind.TAKE_ALTERNATE_PORTAL,
        OptionKind.KEEP_ESCAPE_ROUTE,
        OptionKind.OBSERVE_SAFE,
    ) else Role.GUARDIAN
    values = dict(
        option_instance_id="option-1",
        kind=kind,
        role=role,
        stage_id="stage-1",
        clock_epoch="clock-1",
        localization_epoch="loc-1",
        deadline_ns=90_000_000_000,
        map_version=7,
        topology_version=9,
        parameters_id="params-v1",
        schema_version=2,
        has_target_node=kind not in (OptionKind.HOLD_SAFE, OptionKind.OBSERVE_SAFE),
        target_node_id=12 if kind not in (OptionKind.HOLD_SAFE, OptionKind.OBSERVE_SAFE) else 0,
        target_pose=None,
        goal_zone_id=(
            "target-zone" if kind == OptionKind.ADVANCE_BASE
            else "own-zone" if kind == OptionKind.FALLBACK_DEFEND_BASE
            else ""
        ),
        position_tolerance_m=0.0 if kind == OptionKind.HOLD_SAFE else 0.05,
        yaw_tolerance_rad=0.0 if kind == OptionKind.HOLD_SAFE else 0.1,
    )
    values.update(changes)
    return OptionGoal(**values)


def _submit(authority, *, action_id="action-1", goal=None, context=None):
    return authority.submit(
        action_id,
        goal or _goal(),
        context or _context(),
    )


def test_registry_is_complete_and_matches_serialized_option_enum():
    registry = OptionRegistry()
    assert [definition.kind.value for definition in registry.definitions] == list(range(12))
    assert len({definition.kind for definition in registry.definitions}) == len(OptionKind)


def test_registry_role_target_and_semantic_zone_matrix_is_exact():
    guardian = (Role.GUARDIAN,)
    explorer = (Role.EXPLORER,)
    both = (Role.EXPLORER, Role.GUARDIAN)
    expected = {
        OptionKind.HOLD_SAFE: (both, False, True, False),
        OptionKind.SEARCH_PORTAL: (guardian, True, False, False),
        OptionKind.INTERCEPT_PORTAL: (guardian, True, False, False),
        OptionKind.PRESSURE_ROUTE: (guardian, True, False, False),
        OptionKind.APPROACH_CAPTURE: (guardian, True, False, False),
        OptionKind.RECOVER_VIEW: (guardian, True, False, False),
        OptionKind.FALLBACK_DEFEND_BASE: (guardian, True, False, True),
        OptionKind.ADVANCE_BASE: (explorer, True, False, True),
        OptionKind.BREAK_LOS: (explorer, True, False, False),
        OptionKind.TAKE_ALTERNATE_PORTAL: (explorer, True, False, False),
        OptionKind.KEEP_ESCAPE_ROUTE: (explorer, True, False, False),
        OptionKind.OBSERVE_SAFE: (explorer, False, False, False),
    }
    actual = {
        definition.kind: (
            definition.allowed_roles,
            definition.target_required,
            definition.target_forbidden,
            definition.goal_zone_required,
        )
        for definition in OptionRegistry().definitions
    }
    assert actual == expected


def test_registry_rejects_duplicate_incomplete_or_invalid_definitions():
    defaults = OptionRegistry().definitions
    with pytest.raises(ValueError, match="duplicate"):
        OptionRegistry((*defaults, defaults[0]))
    with pytest.raises(ValueError, match="every OptionKind"):
        OptionRegistry(defaults[:-1])
    broken = list(defaults)
    broken[0] = OptionDefinition(
        kind=OptionKind.HOLD_SAFE,
        allowed_roles=(),
        target_required=False,
        target_forbidden=True,
        goal_zone_required=False,
    )
    with pytest.raises(ValueError, match="allowed roles"):
        OptionRegistry(broken)


@pytest.mark.parametrize(
    "changes",
    [
        {"schema_version": 1},
        {"deadline_ns": True},
        {"map_version": -1},
        {"topology_version": 1.5},
        {"has_target_node": False, "target_node_id": 5},
        {"has_target_node": True, "target_pose": Pose2D(1.0, 2.0, 0.0)},
        {"position_tolerance_m": -0.1},
        {"yaw_tolerance_rad": float("nan")},
    ],
)
def test_goal_contract_rejects_malformed_or_ambiguous_values(changes):
    with pytest.raises(ValueError):
        _goal(**changes)


@pytest.mark.parametrize(
    ("goal_changes", "context_changes", "reason"),
    [
        ({"role": Role.EXPLORER}, {}, "role_not_allowed"),
        ({"stage_id": "old-stage"}, {}, "stage_mismatch"),
        ({"clock_epoch": "old-clock"}, {}, "clock_epoch_mismatch"),
        ({"localization_epoch": "old-loc"}, {}, "localization_epoch_mismatch"),
        ({"deadline_ns": 10_000_000_000}, {}, "goal_deadline_expired"),
        ({"deadline_ns": 101_000_000_000}, {}, "goal_deadline_exceeds_stage"),
        ({}, {"stage_phase": StagePhase.FREEZE}, "stage_not_active"),
        ({}, {"motion_authorized": False}, "motion_authority_unavailable"),
        ({}, {"lease_valid": False}, "motion_authority_unavailable"),
        ({}, {"safety_stop": True}, "safety_stop_active"),
        ({"map_version": 8}, {}, "map_version_mismatch"),
        ({"topology_version": 10}, {}, "topology_version_mismatch"),
        ({"parameters_id": ""}, {}, "parameters_id_missing"),
        ({"has_target_node": False, "target_node_id": 0}, {}, "target_required"),
    ],
)
def test_admission_rejects_invalid_authority_and_goal_predicates(
    goal_changes, context_changes, reason
):
    goal = _goal(**goal_changes)
    context = _context(**context_changes)
    decision = OptionRegistry().check_initiation(goal, context)
    assert not decision.accepted
    assert decision.reason == reason


@pytest.mark.parametrize(
    ("kind", "role", "zone", "target_node"),
    [
        (OptionKind.SEARCH_PORTAL, Role.GUARDIAN, "", True),
        (OptionKind.FALLBACK_DEFEND_BASE, Role.GUARDIAN, "own-zone", True),
        (OptionKind.ADVANCE_BASE, Role.EXPLORER, "target-zone", True),
        (OptionKind.OBSERVE_SAFE, Role.EXPLORER, "", False),
        (OptionKind.OBSERVE_SAFE, Role.EXPLORER, "", True),
    ],
)
def test_registered_role_target_and_zone_semantics(kind, role, zone, target_node):
    goal = _goal(
        kind,
        role=role,
        goal_zone_id=zone,
        has_target_node=target_node,
        target_node_id=12 if target_node else 0,
    )
    context = _context(role=role)
    assert OptionRegistry().check_initiation(goal, context).accepted


def test_pose_target_is_explicit_and_finite_without_aliasing_node_target():
    goal = _goal(
        has_target_node=False,
        target_node_id=0,
        target_pose=Pose2D(0.0, 0.0, 0.0),
    )
    assert goal.has_target
    assert OptionRegistry().check_initiation(goal, _context()).accepted


@pytest.mark.parametrize(
    ("kind", "role", "zone", "expected"),
    [
        (OptionKind.ADVANCE_BASE, Role.EXPLORER, "", "goal_zone_required"),
        (OptionKind.ADVANCE_BASE, Role.EXPLORER, "unresolved", "goal_zone_unresolved"),
        (OptionKind.FALLBACK_DEFEND_BASE, Role.EXPLORER, "own-zone", "role_not_allowed"),
        (OptionKind.HOLD_SAFE, Role.GUARDIAN, "", "accepted"),
        (OptionKind.OBSERVE_SAFE, Role.GUARDIAN, "", "role_not_allowed"),
    ],
)
def test_zone_and_role_requirements_are_not_inferred(kind, role, zone, expected):
    goal = _goal(kind, role=role, goal_zone_id=zone)
    context = _context(role=Role.EXPLORER if kind == OptionKind.ADVANCE_BASE else Role.GUARDIAN)
    decision = OptionRegistry().check_initiation(goal, context)
    assert decision.accepted is (expected == "accepted")
    if expected != "accepted":
        assert decision.reason == expected


def test_hold_safe_is_admissible_during_freeze_and_never_authorizes_motion():
    context = _context(
        stage_phase=StagePhase.FREEZE,
        motion_authorized=False,
        safety_stop=True,
    )
    goal = _goal(OptionKind.HOLD_SAFE)
    authority = OptionAuthority()
    admitted = _submit(authority, goal=goal, context=context)
    assert admitted.accepted
    assert admitted.execution_state.phase == OptionPhase.PLANNING
    assert not admitted.execution_state.candidate_authorized
    state = authority.mark_executing(
        "action-1", replace(context, path_valid=True)
    )
    assert state.phase == OptionPhase.EXECUTING
    assert not state.candidate_authorized


def test_hold_safe_remains_admissible_after_stage_authority_ends_but_never_moves():
    context = _context(
        now_ns=110_000_000_000,
        stage_phase=StagePhase.TERMINAL,
        stage_ends_at_ns=100_000_000_000,
        motion_authorized=False,
        lease_valid=False,
        safety_stop=True,
    )
    goal = _goal(OptionKind.HOLD_SAFE, deadline_ns=120_000_000_000)
    decision = OptionRegistry().check_initiation(goal, context)
    assert decision.accepted
    authority = OptionAuthority()
    assert _submit(authority, goal=goal, context=context).accepted
    state = authority.tick(context)
    assert state.phase == OptionPhase.FINISHED
    assert not state.candidate_authorized
    assert authority.result("action-1").outcome == OptionOutcome.STAGE_ENDED


def test_hold_safe_without_started_stage_terminates_as_stage_ended_not_timeout():
    context = _context(
        now_ns=1,
        stage_phase=StagePhase.INIT,
        stage_ends_at_ns=0,
        motion_authorized=False,
        lease_valid=False,
    )
    goal = _goal(OptionKind.HOLD_SAFE, deadline_ns=10)
    authority = OptionAuthority()
    assert _submit(authority, goal=goal, context=context).accepted
    authority.tick(context)
    assert authority.result("action-1").outcome == OptionOutcome.STAGE_ENDED


def test_hold_safe_rejects_movement_target_even_if_target_would_be_ignored():
    decision = OptionRegistry().check_initiation(
        _goal(OptionKind.HOLD_SAFE, has_target_node=True, target_node_id=12),
        _context(),
    )
    assert not decision.accepted
    assert decision.reason == "hold_safe_must_not_have_target"


def test_movement_goal_requires_positive_target_tolerances():
    goal = _goal(position_tolerance_m=0.0)
    decision = OptionRegistry().check_initiation(goal, _context())
    assert not decision.accepted
    assert decision.reason == "target_tolerance_invalid"


def test_hold_safe_still_requires_the_current_role_and_epoch_identity():
    decision = OptionRegistry().check_initiation(
        _goal(OptionKind.HOLD_SAFE),
        _context(role=Role.EXPLORER),
    )
    assert not decision.accepted
    assert decision.reason == "role_mismatch"


def test_goal_admission_starts_planning_without_candidate_authority():
    authority = OptionAuthority()
    admission = _submit(authority)
    assert admission.accepted
    assert admission.execution_state.phase == OptionPhase.PLANNING
    assert not admission.execution_state.candidate_authorized
    assert authority.execution_state.sequence == 1


def test_only_validated_current_path_can_enable_candidate_authority():
    authority = OptionAuthority()
    _submit(authority)
    invalid = authority.mark_executing("action-1", _context(path_valid=False))
    assert invalid.phase == OptionPhase.FINISHED
    assert not invalid.candidate_authorized
    result = authority.result("action-1")
    assert result.outcome == OptionOutcome.FEASIBILITY_LOST
    assert result.reason == "path_invalid"

    authority = OptionAuthority()
    _submit(authority)
    state = authority.mark_executing(
        "action-1", _context(path_valid=True)
    )
    assert state.phase == OptionPhase.EXECUTING
    assert state.candidate_authorized


def test_authority_and_candidate_lease_share_revocation_and_instance_identity():
    lease = ExecutionLease()
    authority = OptionAuthority(lease=lease)
    _submit(authority)
    path = PlannedPath(
        map_version=7,
        topology_version=9,
        localization_epoch="loc-1",
        start_node_id=0,
        goal_node_id=1,
        node_ids=(0, 1),
        edge_ids=(3,),
        polyline_xy_m=((0.0, 0.0), (1.0, 0.0)),
        cost=1.0,
    )
    candidate = MotionCandidate(
        0.1,
        0.0,
        10.5,
        12.0,
        0.5,
        "option-1",
        7,
        "loc-1",
        9,
        authority.journal[-1].lease_generation,
    )
    envelope = CandidateEnvelope(candidate, path, "option-1")
    generation = authority.journal[-1].lease_generation
    with pytest.raises(ValueError, match="authority is revoked"):
        lease.admit(envelope, now_s=11.0, generation=generation)

    authority.mark_executing(
        "action-1", _context(now_ns=11_000_000_000, path_valid=True)
    )
    assert lease.admit(envelope, now_s=11.0, generation=generation) is candidate
    authority.request_cancel(
        "action-1",
        _context(now_ns=11_100_000_000, path_valid=True),
        cancel_timeout_ns=1_000_000,
    )
    with pytest.raises(ValueError, match="cancelled"):
        lease.admit(envelope, now_s=11.1, generation=generation)


def test_replan_generation_fences_delayed_candidate_from_previous_path():
    lease = ExecutionLease()
    authority = OptionAuthority(lease=lease)
    _submit(authority)
    path = PlannedPath(
        map_version=7,
        topology_version=9,
        localization_epoch="loc-1",
        start_node_id=0,
        goal_node_id=1,
        node_ids=(0, 1),
        edge_ids=(3,),
        polyline_xy_m=((0.0, 0.0), (1.0, 0.0)),
        cost=1.0,
    )
    first_generation = authority.journal[-1].lease_generation
    first_candidate = MotionCandidate(
        0.1, 0.0, 10.5, 12.0, 0.5, "option-1", 7, "loc-1", 9, first_generation
    )
    old_envelope = CandidateEnvelope(first_candidate, path, "option-1")
    authority.mark_executing(
        "action-1", _context(now_ns=11_000_000_000, path_valid=True)
    )
    assert lease.admit(old_envelope, now_s=11.0, generation=first_generation)

    replan = authority.begin_replan(
        "action-1", _context(now_ns=11_100_000_000, path_valid=True)
    )
    assert not replan.candidate_authorized
    assert replan.lease_generation > first_generation
    with pytest.raises(ValueError, match="inactive lease generation"):
        lease.admit(old_envelope, now_s=11.1, generation=first_generation)

    authority.mark_executing(
        "action-1", _context(now_ns=11_200_000_000, path_valid=True)
    )
    replacement = MotionCandidate(
        0.1,
        0.0,
        11.2,
        12.0,
        0.5,
        "option-1",
        7,
        "loc-1",
        9,
        replan.lease_generation,
    )
    assert lease.admit(
        CandidateEnvelope(replacement, path, "option-1"),
        now_s=11.2,
        generation=replan.lease_generation,
    )


def test_deadline_during_planning_terminates_as_timeout_not_feasibility_loss():
    authority = OptionAuthority()
    _submit(authority)
    state = authority.mark_executing(
        "action-1",
        _context(now_ns=90_000_000_000, path_valid=True),
    )
    assert state.phase == OptionPhase.FINISHED
    assert authority.result("action-1").outcome == OptionOutcome.TIMEOUT


def test_replanning_revokes_candidates_until_new_path_is_validated():
    authority = OptionAuthority()
    _submit(authority)
    authority.mark_executing("action-1", _context(path_valid=True))
    replanning = authority.begin_replan("action-1", _context(now_ns=11_000_000_000))
    assert replanning.phase == OptionPhase.REPLANNING
    assert not replanning.candidate_authorized
    executing = authority.mark_executing(
        "action-1", _context(now_ns=12_000_000_000, path_valid=True)
    )
    assert executing.phase == OptionPhase.EXECUTING
    assert executing.candidate_authorized


def test_new_action_is_rejected_until_cancel_revocation_is_acknowledged():
    authority = OptionAuthority()
    old_goal = _goal()
    assert _submit(authority, goal=old_goal).accepted
    denied = _submit(
        authority,
        action_id="action-2",
        goal=_goal(option_instance_id="option-2"),
    )
    assert not denied.accepted
    assert denied.reason == "another_option_is_active"
    cancel_state = authority.request_cancel(
        "action-1", _context(now_ns=11_000_000_000), cancel_timeout_ns=5_000_000
    )
    assert cancel_state.phase == OptionPhase.CANCELING
    assert not cancel_state.candidate_authorized
    assert authority.result("action-1") is None
    canceled = authority.acknowledge_cancel(
        "action-1", _context(now_ns=11_500_000_000)
    )
    assert canceled.outcome == OptionOutcome.CANCELED
    assert canceled.elapsed_ns == 1_500_000_000
    replacement = _submit(
        authority,
        action_id="action-3",
        goal=_goal(option_instance_id="option-3"),
        context=_context(now_ns=12_000_000_000),
    )
    assert replacement.accepted
    assert replacement.execution_state.active_option_instance_id == "option-3"


def test_cancel_timeout_keeps_old_authority_revoked_and_allows_new_instance():
    authority = OptionAuthority()
    _submit(authority)
    authority.mark_executing("action-1", _context(path_valid=True))
    authority.request_cancel(
        "action-1", _context(now_ns=11_000_000_000), cancel_timeout_ns=1_000_000
    )
    timed_out = authority.tick(_context(now_ns=11_001_000_000))
    assert timed_out.phase == OptionPhase.FINISHED
    assert not timed_out.candidate_authorized
    assert authority.result("action-1").outcome == OptionOutcome.INTERNAL_ERROR
    assert authority.result("action-1").reason == "cancel_timeout_authority_revoked"
    next_goal = _goal(option_instance_id="option-2")
    assert _submit(
        authority,
        action_id="action-2",
        goal=next_goal,
        context=_context(now_ns=11_001_000_000),
    ).accepted


@pytest.mark.parametrize(
    ("context_changes", "outcome", "reason"),
    [
        ({"safety_stop": True}, OptionOutcome.SAFETY_STOP, "safety_stop_active"),
        ({"stage_phase": StagePhase.TERMINAL}, OptionOutcome.STAGE_ENDED, "stage_ended"),
        ({"stage_id": "stage-2"}, OptionOutcome.STAGE_ENDED, "stage_ended"),
        ({"clock_epoch": "clock-2"}, OptionOutcome.FEASIBILITY_LOST, "clock_epoch_mismatch"),
        ({"localization_epoch": "loc-2"}, OptionOutcome.FEASIBILITY_LOST, "localization_epoch_mismatch"),
        ({"map_version": 8}, OptionOutcome.FEASIBILITY_LOST, "map_version_mismatch"),
        ({"topology_version": 10}, OptionOutcome.FEASIBILITY_LOST, "topology_version_mismatch"),
        ({"motion_authorized": False}, OptionOutcome.FEASIBILITY_LOST, "motion_authority_unavailable"),
        ({"lease_valid": False}, OptionOutcome.FEASIBILITY_LOST, "motion_authority_unavailable"),
        ({"path_valid": False}, OptionOutcome.FEASIBILITY_LOST, "path_invalid"),
        ({"feasibility_lost": True}, OptionOutcome.FEASIBILITY_LOST, "feasibility_lost"),
    ],
)
def test_runtime_invariant_violation_revokes_authority_once(context_changes, outcome, reason):
    authority = OptionAuthority()
    _submit(authority)
    authority.mark_executing("action-1", _context(path_valid=True))
    tick_values = {"now_ns": 11_000_000_000, "path_valid": True}
    tick_values.update(context_changes)
    context = _context(**tick_values)
    state = authority.tick(context)
    assert state.phase == OptionPhase.FINISHED
    assert not state.candidate_authorized
    assert state.stage_id == context.stage_id
    assert state.clock_epoch == context.clock_epoch
    assert state.localization_epoch == context.localization_epoch
    result = authority.result("action-1")
    assert result.outcome == outcome
    assert result.reason == reason
    assert authority.tick(_context(now_ns=12_000_000_000)).sequence == state.sequence
    assert authority.result("action-1") == result


def test_expiry_is_inclusive_and_effect_is_only_success_for_non_hold_options():
    authority = OptionAuthority()
    _submit(authority)
    authority.mark_executing("action-1", _context(path_valid=True))
    result = authority.tick(
        _context(now_ns=90_000_000_000, path_valid=True)
    )
    assert result.phase == OptionPhase.FINISHED
    assert authority.result("action-1").outcome == OptionOutcome.TIMEOUT

    authority = OptionAuthority()
    hold = _goal(OptionKind.HOLD_SAFE)
    _submit(authority, goal=hold, context=_context(safety_stop=True))
    authority.mark_executing(
        "action-1", _context(safety_stop=True, path_valid=True)
    )
    authority.tick(
        _context(
            now_ns=11_000_000_000,
            safety_stop=True,
            effect_satisfied=True,
            effect_instance_id="option-1",
            effect_evidence_id="evidence-1",
        )
    )
    assert authority.result("action-1") is None


def test_effect_completion_records_success_and_revokes_candidates():
    authority = OptionAuthority()
    _submit(authority)
    authority.mark_executing("action-1", _context(path_valid=True))
    authority.tick(
        _context(
            now_ns=11_000_000_000,
            path_valid=True,
            effect_satisfied=True,
            effect_instance_id="option-1",
            effect_evidence_id="effect-event-1",
        )
    )
    result = authority.result("action-1")
    assert result.outcome == OptionOutcome.SUCCESS
    assert result.reason == "effect_satisfied"
    assert not authority.execution_state.candidate_authorized


def test_effect_cannot_complete_before_validated_execution_begins():
    authority = OptionAuthority()
    _submit(authority)
    state = authority.tick(
        _context(
            now_ns=11_000_000_000,
            effect_satisfied=True,
            effect_instance_id="option-1",
            effect_evidence_id="premature-effect",
        )
    )
    assert state.phase == OptionPhase.PLANNING
    assert authority.result("action-1") is None


def test_delayed_effect_evidence_for_previous_option_cannot_finish_replacement():
    authority = OptionAuthority()
    _submit(authority)
    authority.mark_executing("action-1", _context(path_valid=True))
    authority.request_cancel(
        "action-1", _context(now_ns=11_000_000_000), cancel_timeout_ns=1_000
    )
    authority.acknowledge_cancel("action-1", _context(now_ns=11_000_001_000))
    _submit(
        authority,
        action_id="action-2",
        goal=_goal(option_instance_id="option-2"),
        context=_context(now_ns=11_000_002_000),
    )
    state = authority.tick(
        _context(
            now_ns=11_000_003_000,
            effect_satisfied=True,
            effect_instance_id="option-1",
            effect_evidence_id="delayed-old-effect",
        )
    )
    assert state.phase == OptionPhase.PLANNING
    assert authority.result("action-2") is None


@pytest.mark.parametrize(
    "changes",
    [
        {"effect_satisfied": True},
        {"effect_satisfied": True, "effect_instance_id": "option-1"},
        {"effect_instance_id": "option-1", "effect_evidence_id": "event-1"},
    ],
)
def test_effect_evidence_requires_consistent_instance_and_event_identity(changes):
    with pytest.raises(ValueError, match="effect"):
        _context(**changes)


def test_action_uuid_and_option_instance_ids_are_never_rebound():
    authority = OptionAuthority()
    goal = _goal()
    admitted = _submit(authority, goal=goal)
    assert authority.submit("action-1", goal, _context(now_ns=1)).execution_state == admitted.execution_state
    conflict = authority.submit(
        "action-1",
        _goal(option_instance_id="different"),
        _context(now_ns=11_000_000_000),
    )
    assert not conflict.accepted
    assert conflict.reason == "action_id_reused_with_different_goal"
    authority.request_cancel(
        "action-1", _context(now_ns=11_000_000_000), cancel_timeout_ns=1_000
    )
    authority.acknowledge_cancel("action-1", _context(now_ns=11_000_001_000))
    reused = _submit(
        authority,
        action_id="action-2",
        goal=goal,
        context=_context(now_ns=12_000_000_000),
    )
    assert not reused.accepted
    assert reused.reason == "option_instance_id_reused"


def test_action_record_capacity_fails_closed_without_forgetting_prior_ids():
    authority = OptionAuthority(max_action_records=1)
    first = _submit(authority, goal=_goal(OptionKind.HOLD_SAFE))
    assert first.accepted
    authority.request_cancel(
        "action-1", _context(now_ns=11_000_000_000), cancel_timeout_ns=1_000
    )
    authority.acknowledge_cancel("action-1", _context(now_ns=11_000_001_000))
    second = _submit(
        authority,
        action_id="action-2",
        goal=_goal(OptionKind.HOLD_SAFE, option_instance_id="option-2"),
        context=_context(now_ns=12_000_000_000),
    )
    assert not second.accepted
    assert second.reason == "action_record_capacity"


def test_concurrent_goal_acceptance_has_exactly_one_winner():
    authority = OptionAuthority()
    goals = (
        ("action-a", _goal(option_instance_id="option-a")),
        ("action-b", _goal(option_instance_id="option-b")),
    )
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = tuple(
            pool.map(
                lambda item: authority.submit(item[0], item[1], _context()),
                goals,
            )
        )
    assert sum(result.accepted for result in results) == 1
    assert sum(result.reason == "another_option_is_active" for result in results) == 1
    assert authority.execution_state.sequence == 1


def test_time_regression_is_rejected_without_mutating_current_state():
    authority = OptionAuthority()
    _submit(authority)
    before = authority.execution_state
    with pytest.raises(ValueError, match="monotonic"):
        authority.tick(_context(now_ns=9_999_999_999))
    assert authority.execution_state == before


def test_causal_audit_is_ordered_bounded_and_reports_dropped_records():
    authority = OptionAuthority(max_audit_records=2)
    _submit(authority)
    authority.mark_executing("action-1", _context(path_valid=True))
    authority.request_cancel(
        "action-1", _context(now_ns=11_000_000_000), cancel_timeout_ns=1_000
    )
    authority.acknowledge_cancel("action-1", _context(now_ns=11_000_001_000))
    journal = authority.journal
    assert len(journal) == 2
    assert [record.sequence for record in journal] == [3, 4]
    assert [record.event for record in journal] == ["state_transition", "terminal_result"]
    assert journal[0].reason == "cancel_requested"
    assert not journal[0].candidate_authorized
    assert journal[1].phase == OptionPhase.FINISHED
    assert authority.audit_dropped_count == 2
