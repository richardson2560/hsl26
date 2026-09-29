"""Adversarial P5.3 role selection, guard, utility and hysteresis tests."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import math

import pytest

from hsl_core.match import Role, StagePhase
from hsl_core.tactics import (
    OptionContext,
    OptionGoal,
    OptionKind,
    OptionProposal,
    OptionRegistry,
    InterceptionTiming,
    MAX_TACTICAL_PROPOSALS,
    ROLE_FEATURES,
    TacticalGuard,
    TacticalSelector,
    TacticalSnapshot,
    UtilityProfile,
    UtilityResult,
    score,
)
from hsl_core.types import Pose2D


_GUARDIAN_WEIGHTS = {
    "capture_opportunity": 0.40,
    "portal_time_advantage": 0.25,
    "observation_gain": 0.15,
    "pursuit_value": 0.10,
    "duration_cost": -0.10,
}
_EXPLORER_WEIGHTS = {
    "base_progress": 0.35,
    "visibility_loss": 0.20,
    "alternative_exits": 0.15,
    "escape_safety": 0.15,
    "observation_gain": 0.05,
    "capture_risk": -0.05,
    "duration_cost": -0.05,
}
_REQUIRED_GUARDS = {
    OptionKind.SEARCH_PORTAL: (TacticalGuard.GRAPH_SEARCH_VIEWPOINT,),
    OptionKind.INTERCEPT_PORTAL: (TacticalGuard.SUPPORTED_INTERCEPT,),
    OptionKind.PRESSURE_ROUTE: (TacticalGuard.USEFUL_RIVAL,),
    OptionKind.APPROACH_CAPTURE: (TacticalGuard.ROBUST_CAPTURE_OPPORTUNITY,),
    OptionKind.RECOVER_VIEW: (TacticalGuard.RECENT_OCCLUSION,),
    OptionKind.FALLBACK_DEFEND_BASE: (
        TacticalGuard.CREDIBLE_IMMINENT_BASE_THREAT,
    ),
    OptionKind.ADVANCE_BASE: (
        TacticalGuard.ACCEPTED_GOAL,
        TacticalGuard.GOAL_THREAT_ACCEPTABLE,
    ),
    OptionKind.BREAK_LOS: (
        TacticalGuard.IMMEDIATE_TRAP_RISK,
        TacticalGuard.BREAK_LOS_FEASIBLE,
    ),
    OptionKind.TAKE_ALTERNATE_PORTAL: (
        TacticalGuard.CONTESTED_ROUTE,
        TacticalGuard.ALTERNATE_ROUTE_AVAILABLE,
    ),
    OptionKind.KEEP_ESCAPE_ROUTE: (
        TacticalGuard.IMMEDIATE_TRAP_RISK,
        TacticalGuard.VERIFIED_ESCAPE_ROUTE,
    ),
    OptionKind.OBSERVE_SAFE: (
        TacticalGuard.INFORMATIVE_SAFE_OBSERVATION,
    ),
    OptionKind.ADVANCE_KNOWN_ROUTE: (
        TacticalGuard.VERSIONED_OPEN_ROUTE,
    ),
}
_UNSET = object()


def _context(role, **changes):
    values = dict(
        now_ns=10_000_000_000,
        stage_id="stage-1",
        stage_phase=StagePhase.ACTIVE,
        stage_ends_at_ns=100_000_000_000,
        role=role,
        clock_epoch="clock-1",
        localization_epoch="loc-1",
        map_version=7,
        topology_version=9,
        motion_authorized=True,
        lease_valid=True,
        safety_stop=False,
        accepted_goal_zone_ids=("target-zone", "own-zone"),
    )
    values.update(changes)
    return OptionContext(**values)


def _goal(
    kind,
    role,
    *,
    instance_id=None,
    target_node_id=12,
    target_pose=None,
    goal_zone_id="",
    deadline_ns=90_000_000_000,
    **changes,
):
    hold = kind == OptionKind.HOLD_SAFE
    has_node = not hold and target_pose is None
    values = dict(
        option_instance_id=instance_id or f"option-{kind.name.lower()}-{target_node_id}",
        kind=kind,
        role=role,
        stage_id="stage-1",
        clock_epoch="clock-1",
        localization_epoch="loc-1",
        deadline_ns=deadline_ns,
        map_version=7,
        topology_version=9,
        parameters_id="policy-v1",
        schema_version=2,
        has_target_node=has_node,
        target_node_id=target_node_id if has_node else 0,
        target_pose=target_pose,
        target_yaw_required=False,
        position_tolerance_m=0.0 if hold else 0.05,
        yaw_tolerance_rad=0.0 if hold else 0.1,
        goal_zone_id=goal_zone_id,
    )
    values.update(changes)
    return OptionGoal(**values)


def _features(role, **overrides):
    values = {name: 0.0 for name in ROLE_FEATURES[role]}
    values.update(overrides)
    return tuple(values.items())


def _proposal(
    kind,
    role,
    *,
    feasible=True,
    infeasible_reason="",
    guards=None,
    guard_evidence=_UNSET,
    features=None,
    stable_key=None,
    instance_id=None,
    target_node_id=12,
    target_pose=None,
    goal_zone_id="",
    urgency_rank=_UNSET,
    urgency_evidence_id=_UNSET,
    interception_timing=None,
    **goal_changes,
):
    if guards is None:
        guards = _REQUIRED_GUARDS.get(kind, ())
    if guard_evidence is _UNSET:
        guard_evidence = tuple(
            (guard, f"evidence-{guard.name.lower()}") for guard in guards
        )
    if urgency_rank is _UNSET:
        urgency_rank = (
            1
            if role == Role.EXPLORER
            and kind in (OptionKind.KEEP_ESCAPE_ROUTE, OptionKind.BREAK_LOS)
            else None
        )
    if urgency_evidence_id is _UNSET:
        urgency_evidence_id = (
            f"urgency-{stable_key or kind.name.lower()}"
            if urgency_rank is not None
            else ""
        )
    return OptionProposal(
        stable_key=stable_key or f"{kind.name.lower()}-{target_node_id}",
        goal=_goal(
            kind,
            role,
            instance_id=instance_id,
            target_node_id=target_node_id,
            target_pose=target_pose,
            goal_zone_id=goal_zone_id,
            **goal_changes,
        ),
        feasible=feasible,
        infeasible_reason=infeasible_reason,
        guard_facts=tuple(guards),
        guard_evidence=tuple(guard_evidence),
        features=() if kind == OptionKind.HOLD_SAFE else (
            _features(role) if features is None else tuple(features.items())
        ),
        urgency_rank=urgency_rank,
        urgency_evidence_id=urgency_evidence_id,
        interception_timing=interception_timing,
    )


def _profile(role, weights=None):
    if weights is None:
        weights = _GUARDIAN_WEIGHTS if role == Role.GUARDIAN else _EXPLORER_WEIGHTS
    return UtilityProfile(
        profile_id=f"utility-{role.name.lower()}-v1",
        role=role,
        weights=tuple(weights.items()),
        schema_version=1,
    )


def _selector(role, *, hysteresis=0.05, dwell_ns=0, max_proposals=32, profile=None):
    return TacticalSelector(
        profile or _profile(role),
        registry=OptionRegistry(),
        hysteresis_delta_u=hysteresis,
        minimum_dwell_ns=dwell_ns,
        max_proposals=max_proposals,
    )


def _snapshot(role, proposals, **context_changes):
    return TacticalSnapshot(
        context=_context(role, **context_changes),
        proposals=tuple(proposals),
        system_ready=True,
        health_reason="",
    )


def _fallback(role):
    return _proposal(
        OptionKind.HOLD_SAFE,
        role,
        stable_key="hold",
        instance_id=f"hold-{role.name.lower()}",
    )


@pytest.mark.parametrize(
    ("role", "kind", "priority"),
    [
        (Role.GUARDIAN, OptionKind.APPROACH_CAPTURE, 0),
        (Role.GUARDIAN, OptionKind.FALLBACK_DEFEND_BASE, 1),
        (Role.GUARDIAN, OptionKind.INTERCEPT_PORTAL, 2),
        (Role.GUARDIAN, OptionKind.RECOVER_VIEW, 3),
        (Role.GUARDIAN, OptionKind.PRESSURE_ROUTE, 4),
        (Role.GUARDIAN, OptionKind.SEARCH_PORTAL, 5),
        (Role.EXPLORER, OptionKind.KEEP_ESCAPE_ROUTE, 0),
        (Role.EXPLORER, OptionKind.BREAK_LOS, 0),
        (Role.EXPLORER, OptionKind.TAKE_ALTERNATE_PORTAL, 1),
        (Role.EXPLORER, OptionKind.ADVANCE_BASE, 2),
        (Role.EXPLORER, OptionKind.ADVANCE_KNOWN_ROUTE, 2),
        (Role.EXPLORER, OptionKind.OBSERVE_SAFE, 3),
    ],
)
def test_every_role_option_has_documented_priority_and_guard(role, kind, priority):
    zone = "own-zone" if kind == OptionKind.FALLBACK_DEFEND_BASE else (
        "target-zone" if kind == OptionKind.ADVANCE_BASE else ""
    )
    proposal = _proposal(kind, role, goal_zone_id=zone)
    result = _selector(role).select(_snapshot(role, (proposal, _fallback(role))))
    assert result.selected == proposal
    assert result.priority == priority
    assert result.reason == "highest_priority_then_utility"
    assert all(candidate.applicable for candidate in result.alternatives)


def test_guardian_capture_dominates_every_lower_priority_even_at_minimum_utility():
    capture = _proposal(
        OptionKind.APPROACH_CAPTURE,
        Role.GUARDIAN,
        features={name: 0.0 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    search = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        features={name: 1.0 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    selected = _selector(Role.GUARDIAN).select(
        _snapshot(Role.GUARDIAN, (search, capture, _fallback(Role.GUARDIAN)))
    )
    assert selected.selected == capture
    assert selected.priority == 0
    assert selected.utility.value < next(
        item.utility.value for item in selected.alternatives if item.kind == search.goal.kind
    )


def test_intercept_can_share_urgent_base_threat_priority_only_with_proof_guard():
    defense = _proposal(
        OptionKind.FALLBACK_DEFEND_BASE, Role.GUARDIAN, goal_zone_id="own-zone"
    )
    intercept_without_proof = _proposal(
        OptionKind.INTERCEPT_PORTAL,
        Role.GUARDIAN,
        guards=(TacticalGuard.SUPPORTED_INTERCEPT,),
    )
    with_proof = _proposal(
        OptionKind.INTERCEPT_PORTAL,
        Role.GUARDIAN,
        stable_key="intercept-proved",
        instance_id="option-intercept-proved",
        guards=(
            TacticalGuard.SUPPORTED_INTERCEPT,
            TacticalGuard.CREDIBLE_IMMINENT_BASE_THREAT,
            TacticalGuard.INTERCEPT_DOMINATES_BASE_DEFENSE,
        ),
        interception_timing=InterceptionTiming(
            hypothesis_id="rival-hypothesis-1",
            evidence_id="timing-evidence-1",
            guardian_arrival_upper_ns=80,
            explorer_arrival_lower_ns=100,
            required_buffer_ns=5,
        ),
    )
    selector = _selector(Role.GUARDIAN)
    without = selector.select(
        _snapshot(
            Role.GUARDIAN,
            (defense, intercept_without_proof, _fallback(Role.GUARDIAN)),
        )
    )
    assert without.selected == defense
    assert without.priority == 1
    with_result = selector.select(
        _snapshot(Role.GUARDIAN, (defense, with_proof, _fallback(Role.GUARDIAN)))
    )
    assert with_result.selected == with_proof
    assert with_result.priority == 1


@pytest.mark.parametrize(
    ("guardian_upper", "explorer_lower", "buffer"),
    [
        (80, 100, 20),
        (80, 100, 21),
        (100, 100, 0),
        (101, 100, 0),
    ],
)
def test_intercept_urgency_requires_strict_worst_case_time_separation(
    guardian_upper, explorer_lower, buffer
):
    timing = InterceptionTiming(
        hypothesis_id="rival-hypothesis",
        evidence_id="arrival-intervals-1",
        guardian_arrival_upper_ns=guardian_upper,
        explorer_arrival_lower_ns=explorer_lower,
        required_buffer_ns=buffer,
    )
    proposal = _proposal(
        OptionKind.INTERCEPT_PORTAL,
        Role.GUARDIAN,
        guards=(
            TacticalGuard.SUPPORTED_INTERCEPT,
            TacticalGuard.CREDIBLE_IMMINENT_BASE_THREAT,
            TacticalGuard.INTERCEPT_DOMINATES_BASE_DEFENSE,
        ),
        interception_timing=timing,
    )
    result = _selector(Role.GUARDIAN).select(
        _snapshot(
            Role.GUARDIAN,
            (
                proposal,
                _proposal(
                    OptionKind.FALLBACK_DEFEND_BASE,
                    Role.GUARDIAN,
                    goal_zone_id="own-zone",
                    instance_id="defend-base",
                ),
                _fallback(Role.GUARDIAN),
            ),
        )
    )
    if guardian_upper + buffer < explorer_lower:
        assert result.selected == proposal
        assert result.priority == 1
    else:
        assert result.selected.goal.kind == OptionKind.FALLBACK_DEFEND_BASE
        denied = next(item for item in result.alternatives if item.stable_key == proposal.stable_key)
        assert not denied.applicable
        assert denied.reason == "intercept_time_advantage_not_proven"


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("guardian_arrival_upper_ns", True),
        ("explorer_arrival_lower_ns", -1),
        ("required_buffer_ns", 1.5),
    ],
)
def test_interception_timing_rejects_invalid_nanosecond_intervals(field, value):
    values = dict(
        hypothesis_id="hypothesis-1",
        evidence_id="arrival-evidence-1",
        guardian_arrival_upper_ns=80,
        explorer_arrival_lower_ns=100,
        required_buffer_ns=5,
    )
    values[field] = value
    with pytest.raises(ValueError):
        InterceptionTiming(**values)


@pytest.mark.parametrize(
    ("kind", "role", "guards", "reason"),
    [
        (
            OptionKind.APPROACH_CAPTURE,
            Role.GUARDIAN,
            (),
            "guard_not_satisfied:ROBUST_CAPTURE_OPPORTUNITY",
        ),
        (
            OptionKind.KEEP_ESCAPE_ROUTE,
            Role.EXPLORER,
            (TacticalGuard.IMMEDIATE_TRAP_RISK,),
            "guard_not_satisfied:VERIFIED_ESCAPE_ROUTE",
        ),
        (
            OptionKind.INTERCEPT_PORTAL,
            Role.GUARDIAN,
            (
                TacticalGuard.SUPPORTED_INTERCEPT,
                TacticalGuard.INTERCEPT_DOMINATES_BASE_DEFENSE,
            ),
            "guard_fact_not_applicable",
        ),
        (
            OptionKind.SEARCH_PORTAL,
            Role.GUARDIAN,
            (
                TacticalGuard.GRAPH_SEARCH_VIEWPOINT,
                TacticalGuard.USEFUL_RIVAL,
            ),
            "guard_fact_not_applicable",
        ),
    ],
)
def test_missing_or_logically_inconsistent_guards_reject_option(kind, role, guards, reason):
    proposal = _proposal(kind, role, guards=guards)
    result = _selector(role).select(_snapshot(role, (proposal, _fallback(role))))
    assert result.selected.goal.kind == OptionKind.HOLD_SAFE
    evaluation = next(item for item in result.alternatives if item.stable_key == proposal.stable_key)
    assert not evaluation.applicable
    assert evaluation.reason == reason


def test_every_guard_requires_a_unique_nonempty_traceable_evidence_id():
    with pytest.raises(ValueError, match="every guard fact"):
        _proposal(
            OptionKind.APPROACH_CAPTURE,
            Role.GUARDIAN,
            guard_evidence=(),
        )
    with pytest.raises(ValueError, match="evidence IDs"):
        _proposal(
            OptionKind.APPROACH_CAPTURE,
            Role.GUARDIAN,
            guard_evidence=((TacticalGuard.ROBUST_CAPTURE_OPPORTUNITY, ""),),
        )
    proposal = _proposal(OptionKind.APPROACH_CAPTURE, Role.GUARDIAN)
    result = _selector(Role.GUARDIAN).select(
        _snapshot(Role.GUARDIAN, (proposal, _fallback(Role.GUARDIAN)))
    )
    evaluation = next(
        item for item in result.alternatives if item.stable_key == proposal.stable_key
    )
    assert evaluation.guard_evidence == proposal.guard_evidence


@pytest.mark.parametrize(
    ("kind", "role", "zone"),
    [
        (OptionKind.FALLBACK_DEFEND_BASE, Role.GUARDIAN, ""),
        (OptionKind.ADVANCE_BASE, Role.EXPLORER, ""),
        (OptionKind.ADVANCE_BASE, Role.EXPLORER, "unresolved-zone"),
    ],
)
def test_unresolved_semantic_zones_never_produce_movement_selection(kind, role, zone):
    proposal = _proposal(kind, role, goal_zone_id=zone)
    result = _selector(role).select(_snapshot(role, (proposal, _fallback(role))))
    assert result.selected.goal.kind == OptionKind.HOLD_SAFE
    denied = next(item for item in result.alternatives if item.stable_key == proposal.stable_key)
    assert not denied.applicable
    assert denied.reason in ("goal_zone_required", "goal_zone_unresolved")


@pytest.mark.parametrize(
    ("role", "kind", "reason"),
    [
        (Role.GUARDIAN, OptionKind.ADVANCE_BASE, "role_not_allowed"),
        (Role.EXPLORER, OptionKind.APPROACH_CAPTURE, "role_not_allowed"),
    ],
)
def test_cross_role_capabilities_are_rejected(role, kind, reason):
    wrong_role = Role.EXPLORER if role == Role.GUARDIAN else Role.GUARDIAN
    proposal = _proposal(kind, wrong_role, goal_zone_id="target-zone" if kind == OptionKind.ADVANCE_BASE else "")
    result = _selector(role).select(_snapshot(role, (proposal, _fallback(role))))
    evaluation = next(item for item in result.alternatives if item.stable_key == proposal.stable_key)
    assert not evaluation.applicable
    assert evaluation.reason == reason


@pytest.mark.parametrize(
    ("context_changes", "ready", "health_reason", "expected"),
    [
        ({"stage_phase": StagePhase.FREEZE, "motion_authorized": False}, True, "", "stage_motion_not_authorized"),
        ({"stage_phase": StagePhase.TERMINAL, "motion_authorized": False}, True, "", "stage_motion_not_authorized"),
        ({"motion_authorized": False}, True, "", "stage_motion_not_authorized"),
        ({"lease_valid": False}, True, "", "stage_lease_invalid"),
        ({"safety_stop": True}, True, "", "safety_stop_active"),
        ({}, False, "watchdog_not_ready", "system_not_ready:watchdog_not_ready"),
    ],
)
def test_hard_authority_and_health_gates_force_hold_before_ranking(
    context_changes, ready, health_reason, expected
):
    approach = _proposal(OptionKind.APPROACH_CAPTURE, Role.GUARDIAN)
    snapshot = TacticalSnapshot(
        context=_context(Role.GUARDIAN, **context_changes),
        proposals=(approach, _fallback(Role.GUARDIAN)),
        system_ready=ready,
        health_reason=health_reason,
    )
    result = _selector(Role.GUARDIAN).select(snapshot)
    assert result.selected.goal.kind == OptionKind.HOLD_SAFE
    assert result.reason == expected
    assert not result.selected.goal.kind == approach.goal.kind
    blocked = next(
        item for item in result.alternatives if item.stable_key == approach.stable_key
    )
    assert not blocked.applicable
    assert blocked.reason.startswith(("global_gate:", "stage_not_active", "motion_authority_unavailable", "safety_stop_active"))
    assert blocked.utility is None


def test_no_proposals_or_only_infeasible_proposals_select_hold_with_reasons():
    with pytest.raises(ValueError, match="exactly one HOLD_SAFE"):
        _selector(Role.GUARDIAN).select(_snapshot(Role.GUARDIAN, ()))
    infeasible = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        feasible=False,
        infeasible_reason="route_blocked_by_opponent",
    )
    result = _selector(Role.GUARDIAN).select(
        _snapshot(Role.GUARDIAN, (infeasible, _fallback(Role.GUARDIAN)))
    )
    assert result.selected.goal.kind == OptionKind.HOLD_SAFE
    assert result.reason == "no_feasible_option"
    denied = next(item for item in result.alternatives if item.stable_key == infeasible.stable_key)
    assert denied.reason == "route_blocked_by_opponent"


def test_feasibility_cannot_be_bought_with_extreme_utility_features():
    blocked = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        feasible=False,
        infeasible_reason="collision_sweep_invalid",
        features={name: 1.0 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    feasible = _proposal(
        OptionKind.PRESSURE_ROUTE,
        Role.GUARDIAN,
        features={name: 0.0 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    result = _selector(Role.GUARDIAN).select(
        _snapshot(Role.GUARDIAN, (blocked, feasible, _fallback(Role.GUARDIAN)))
    )
    assert result.selected == feasible
    assert next(item for item in result.alternatives if item.stable_key == blocked.stable_key).reason == "collision_sweep_invalid"


def test_utility_profile_requires_complete_signed_bounded_role_schema():
    weights = dict(_GUARDIAN_WEIGHTS)
    with pytest.raises(ValueError, match="feature schema"):
        _profile(Role.GUARDIAN, {key: value for key, value in weights.items() if key != "pursuit_value"})
    weights["unknown_feature"] = 0.0
    with pytest.raises(ValueError, match="feature schema"):
        _profile(Role.GUARDIAN, weights)
    weights = dict(_GUARDIAN_WEIGHTS, duration_cost=0.1)
    with pytest.raises(ValueError, match="cost feature"):
        _profile(Role.GUARDIAN, weights)
    weights = dict(_GUARDIAN_WEIGHTS, capture_opportunity=-0.1)
    with pytest.raises(ValueError, match="benefit feature"):
        _profile(Role.GUARDIAN, weights)
    weights = dict(_GUARDIAN_WEIGHTS, capture_opportunity=0.8)
    with pytest.raises(ValueError, match="sum of absolute"):
        _profile(Role.GUARDIAN, weights)


@pytest.mark.parametrize("bad", [True, -0.01, 1.01, float("nan"), float("inf"), "1"])
def test_score_rejects_bool_nonfinite_and_out_of_range_features(bad):
    features = {name: 0.5 for name in ROLE_FEATURES[Role.GUARDIAN]}
    features["capture_opportunity"] = bad
    with pytest.raises(ValueError, match="capture_opportunity"):
        score(features, _profile(Role.GUARDIAN))


def test_score_is_exact_dot_product_with_contributions_and_proven_range():
    features = {
        "capture_opportunity": 0.8,
        "portal_time_advantage": 0.4,
        "observation_gain": 0.6,
        "pursuit_value": 0.2,
        "duration_cost": 0.5,
    }
    result = score(features, _profile(Role.GUARDIAN))
    expected = math.fsum(
        weight * features[name] for name, weight in _GUARDIAN_WEIGHTS.items()
    )
    assert result.value == pytest.approx(expected, abs=1e-15)
    assert result.profile_id == "utility-guardian-v1"
    assert dict(result.contributions)["duration_cost"] == pytest.approx(-0.05)
    assert -1.0 <= result.value <= 1.0


@pytest.mark.parametrize(
    "values",
    [
        {"value": True, "contributions": (("x", 1.0),)},
        {"value": 0.5, "contributions": (("x", 0.4),)},
        {"value": 0.5, "contributions": (("x", 0.25), ("x", 0.25))},
        {"value": 0.5, "contributions": (("x", float("nan")),)},
    ],
)
def test_utility_result_rejects_fabricated_or_inconsistent_contributions(values):
    with pytest.raises(ValueError):
        UtilityResult(
            profile_id="profile-v1",
            schema_version=1,
            **values,
        )


def test_missing_extra_features_and_wrong_role_profile_never_rank():
    profile = _profile(Role.GUARDIAN)
    complete = {name: 0.5 for name in ROLE_FEATURES[Role.GUARDIAN]}
    with pytest.raises(ValueError, match="feature set"):
        score({key: value for key, value in complete.items() if key != "pursuit_value"}, profile)
    with pytest.raises(ValueError, match="feature set"):
        score(dict(complete, extra=0.0), profile)
    with pytest.raises(ValueError, match="role"):
        _selector(Role.EXPLORER, profile=profile).select(
            _snapshot(Role.EXPLORER, (_fallback(Role.EXPLORER),))
        )


def test_utility_is_bounded_by_l1_coefficient_norm():
    for role, weights in (
        (Role.GUARDIAN, _GUARDIAN_WEIGHTS),
        (Role.EXPLORER, _EXPLORER_WEIGHTS),
    ):
        profile = _profile(role, weights)
        features = {
            name: (1.0 if coefficient > 0.0 else 0.0)
            for name, coefficient in weights.items()
        }
        best = score(features, profile)
        features = {
            name: (1.0 if coefficient < 0.0 else 0.0)
            for name, coefficient in weights.items()
        }
        worst = score(features, profile)
        assert best.value <= 1.0
        assert worst.value >= -1.0


def test_lower_priority_class_wins_regardless_of_larger_weighted_utility():
    advance = _proposal(
        OptionKind.ADVANCE_BASE,
        Role.EXPLORER,
        goal_zone_id="target-zone",
        features={name: 0.0 for name in ROLE_FEATURES[Role.EXPLORER]},
    )
    observe = _proposal(
        OptionKind.OBSERVE_SAFE,
        Role.EXPLORER,
        features={name: 1.0 for name in ROLE_FEATURES[Role.EXPLORER]},
    )
    result = _selector(Role.EXPLORER).select(
        _snapshot(Role.EXPLORER, (observe, advance, _fallback(Role.EXPLORER)))
    )
    assert result.selected == advance
    assert result.priority == 2
    assert next(item for item in result.alternatives if item.kind == observe.goal.kind).utility.value > result.utility.value


def test_hysteresis_uses_utility_units_and_strict_improvement_threshold():
    current = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="current",
        instance_id="option-current",
        features={name: 0.5 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    challenger = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="challenger",
        instance_id="option-challenger",
        target_node_id=13,
        features={
            **{name: 0.5 for name in ROLE_FEATURES[Role.GUARDIAN]},
            "capture_opportunity": 0.52,
        },
    )
    profile = _profile(
        Role.GUARDIAN,
        {
            "capture_opportunity": 0.5,
            "portal_time_advantage": 0.0,
            "observation_gain": 0.0,
            "pursuit_value": 0.0,
            "duration_cost": -0.0,
        },
    )
    selector = _selector(Role.GUARDIAN, hysteresis=0.1, profile=profile)
    same_utility = selector.select(
        _snapshot(Role.GUARDIAN, (current, challenger, _fallback(Role.GUARDIAN))),
        current_stable_key="current",
        current_option_instance_id="option-current",
        current_started_at_ns=0,
    )
    assert same_utility.selected == current
    assert same_utility.reason == "hysteresis"

    higher = replace(
        challenger,
        features=tuple(
            {**dict(challenger.features), "capture_opportunity": 0.8}.items()
        ),
    )
    improved = selector.select(
        _snapshot(Role.GUARDIAN, (current, higher, _fallback(Role.GUARDIAN))),
        current_stable_key="current",
        current_option_instance_id="option-current",
        current_started_at_ns=0,
    )
    assert improved.selected == higher
    assert improved.reason == "utility_improvement_exceeds_hysteresis"


def test_minimum_dwell_blocks_only_nonurgent_switches():
    current = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="current",
        instance_id="option-current",
        features={name: 0.0 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    challenger = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="challenger",
        instance_id="option-challenger",
        target_node_id=13,
        features={name: 1.0 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    selector = _selector(Role.GUARDIAN, hysteresis=0.0, dwell_ns=5_000_000_000)
    recent = selector.select(
        _snapshot(Role.GUARDIAN, (current, challenger, _fallback(Role.GUARDIAN))),
        current_stable_key="current",
        current_option_instance_id="option-current",
        current_started_at_ns=9_000_000_000,
    )
    assert recent.selected == current
    assert recent.reason == "minimum_dwell"

    urgent = _proposal(
        OptionKind.APPROACH_CAPTURE,
        Role.GUARDIAN,
        instance_id="option-urgent",
    )
    preempted = selector.select(
        _snapshot(
            Role.GUARDIAN,
            (current, challenger, urgent, _fallback(Role.GUARDIAN)),
            now_ns=10_000_000_001,
        ),
        current_stable_key="current",
        current_option_instance_id="option-current",
        current_started_at_ns=9_000_000_000,
    )
    assert preempted.selected == urgent
    assert preempted.reason == "higher_priority_preemption"


def test_explorer_urgency_rank_preempts_utility_and_dwell_without_claiming_probability():
    less_urgent = _proposal(
        OptionKind.KEEP_ESCAPE_ROUTE,
        Role.EXPLORER,
        stable_key="less-urgent",
        instance_id="less-urgent-instance",
        urgency_rank=1,
        features={
            **{name: 0.0 for name in ROLE_FEATURES[Role.EXPLORER]},
            "escape_safety": 0.0,
        },
    )
    more_urgent = _proposal(
        OptionKind.BREAK_LOS,
        Role.EXPLORER,
        stable_key="more-urgent",
        instance_id="more-urgent-instance",
        urgency_rank=2,
        features={name: 1.0 for name in ROLE_FEATURES[Role.EXPLORER]},
    )
    selector = _selector(Role.EXPLORER, hysteresis=1.0, dwell_ns=50_000_000_000)
    retained = selector.select(
        _snapshot(
            Role.EXPLORER,
            (less_urgent, more_urgent, _fallback(Role.EXPLORER)),
        ),
        current_stable_key="less-urgent",
        current_option_instance_id="less-urgent-instance",
        current_started_at_ns=0,
    )
    assert retained.selected == more_urgent
    assert retained.reason == "higher_urgency_preemption"


def test_higher_risk_rank_preempts_even_when_its_nominal_utility_is_lower():
    less_urgent = _proposal(
        OptionKind.KEEP_ESCAPE_ROUTE,
        Role.EXPLORER,
        stable_key="current-safe-exit",
        instance_id="current-safe-exit-instance",
        urgency_rank=2,
        features={name: 1.0 for name in ROLE_FEATURES[Role.EXPLORER]},
    )
    more_urgent = _proposal(
        OptionKind.BREAK_LOS,
        Role.EXPLORER,
        stable_key="urgent-break-los",
        instance_id="urgent-break-los-instance",
        urgency_rank=3,
        features={name: 0.0 for name in ROLE_FEATURES[Role.EXPLORER]},
    )
    selector = _selector(Role.EXPLORER, hysteresis=1.0, dwell_ns=50_000_000_000)
    result = selector.select(
        _snapshot(
            Role.EXPLORER,
            (less_urgent, more_urgent, _fallback(Role.EXPLORER)),
        ),
        current_stable_key="current-safe-exit",
        current_option_instance_id="current-safe-exit-instance",
        current_started_at_ns=0,
    )
    assert result.selected == more_urgent
    assert result.utility.value < next(
        item.utility.value
        for item in result.alternatives
        if item.stable_key == less_urgent.stable_key
    )
    assert result.reason == "higher_urgency_preemption"


@pytest.mark.parametrize("rank", [True, -1, 256, 1.5])
def test_urgency_rank_is_a_bounded_ordinal_not_an_unchecked_score(rank):
    with pytest.raises(ValueError, match="urgency_rank"):
        _proposal(
            OptionKind.BREAK_LOS,
            Role.EXPLORER,
            urgency_rank=rank,
        )


def test_explorer_emergency_options_cannot_omit_the_risk_urgency_rank():
    with pytest.raises(ValueError, match="require an ordinal urgency_rank"):
        _proposal(
            OptionKind.KEEP_ESCAPE_ROUTE,
            Role.EXPLORER,
            urgency_rank=None,
        )


def test_explorer_emergency_options_cannot_omit_urgency_evidence():
    with pytest.raises(ValueError, match="require urgency evidence"):
        _proposal(
            OptionKind.BREAK_LOS,
            Role.EXPLORER,
            urgency_rank=4,
            urgency_evidence_id="",
        )


def test_invalid_or_ended_current_option_is_never_protected_by_hysteresis():
    current = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="current",
        instance_id="option-current",
        features={name: 0.0 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    better = _proposal(
        OptionKind.PRESSURE_ROUTE,
        Role.GUARDIAN,
        stable_key="better",
        instance_id="option-better",
        features={name: 1.0 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    infeasible_current = replace(
        current, feasible=False, infeasible_reason="blocked_now"
    )
    result = _selector(Role.GUARDIAN, hysteresis=1.0, dwell_ns=99_000_000_000).select(
        _snapshot(
            Role.GUARDIAN,
            (infeasible_current, better, _fallback(Role.GUARDIAN)),
            now_ns=10_000_000_001,
        ),
        current_stable_key="current",
        current_option_instance_id="option-current",
        current_started_at_ns=10_000_000_000,
    )
    assert result.selected == better


def test_current_instance_mismatch_does_not_reuse_same_stable_key_state():
    current = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="same-key",
        instance_id="new-instance",
    )
    challenger = _proposal(
        OptionKind.PRESSURE_ROUTE,
        Role.GUARDIAN,
        stable_key="challenger",
        instance_id="challenger-instance",
        features={name: 1.0 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    result = _selector(Role.GUARDIAN).select(
        _snapshot(Role.GUARDIAN, (current, challenger, _fallback(Role.GUARDIAN))),
        current_stable_key="same-key",
        current_option_instance_id="old-instance",
        current_started_at_ns=0,
    )
    assert result.selected == challenger
    assert result.changed


def test_same_stable_key_with_different_instance_is_reported_as_a_switch():
    replacement = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="same-key",
        instance_id="replacement-instance",
    )
    result = _selector(Role.GUARDIAN).select(
        _snapshot(Role.GUARDIAN, (replacement, _fallback(Role.GUARDIAN))),
        current_stable_key="same-key",
        current_option_instance_id="revoked-instance",
        current_started_at_ns=0,
    )
    assert result.selected == replacement
    assert result.changed


def test_deterministic_tie_break_uses_option_kind_then_node_then_stable_key():
    later_node = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="z-node",
        instance_id="z-instance",
        target_node_id=9,
    )
    earlier_node = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="a-node",
        instance_id="a-instance",
        target_node_id=3,
    )
    selected = _selector(Role.GUARDIAN).select(
        _snapshot(
            Role.GUARDIAN,
            (later_node, earlier_node, _fallback(Role.GUARDIAN)),
        )
    )
    assert selected.selected == earlier_node
    reversed_result = _selector(Role.GUARDIAN).select(
        _snapshot(
            Role.GUARDIAN,
            (earlier_node, later_node, _fallback(Role.GUARDIAN)),
        )
    )
    assert reversed_result.selected == earlier_node
    assert selected.alternatives == reversed_result.alternatives


def test_pose_tie_break_is_deterministic_for_integer_valued_pose_components():
    pose_later = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="z-pose",
        instance_id="z-pose-instance",
        target_node_id=0,
        target_pose=Pose2D(2, 0, 0),
    )
    pose_first = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="a-pose",
        instance_id="a-pose-instance",
        target_node_id=0,
        target_pose=Pose2D(1, 0, 0),
    )
    result = _selector(Role.GUARDIAN).select(
        _snapshot(Role.GUARDIAN, (pose_later, pose_first, _fallback(Role.GUARDIAN)))
    )
    assert result.selected == pose_first


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"hysteresis": -0.1}, "hysteresis_delta_u"),
        ({"dwell_ns": -1}, "minimum_dwell_ns"),
        ({"max_proposals": 0}, "max_proposals"),
    ],
)
def test_selector_rejects_invalid_configuration(changes, message):
    with pytest.raises(ValueError, match=message):
        _selector(Role.GUARDIAN, **changes)


def test_candidate_bound_is_hard_capped_for_predictable_selector_work():
    with pytest.raises(ValueError, match="max_proposals"):
        _selector(Role.GUARDIAN, max_proposals=MAX_TACTICAL_PROPOSALS + 1)
    assert _selector(
        Role.GUARDIAN,
        max_proposals=MAX_TACTICAL_PROPOSALS,
    ).max_proposals == MAX_TACTICAL_PROPOSALS


def test_duplicate_missing_or_invalid_hold_fallback_is_not_silently_repaired():
    selector = _selector(Role.GUARDIAN)
    search = _proposal(OptionKind.SEARCH_PORTAL, Role.GUARDIAN)
    with pytest.raises(ValueError, match="exactly one HOLD_SAFE"):
        selector.select(_snapshot(Role.GUARDIAN, (search,)))
    with pytest.raises(ValueError, match="stable keys"):
        selector.select(
            _snapshot(
                Role.GUARDIAN,
                (search, replace(_fallback(Role.GUARDIAN), stable_key=search.stable_key)),
            )
        )
    with pytest.raises(ValueError, match="instance_id values"):
        selector.select(
            _snapshot(
                Role.GUARDIAN,
                (search, replace(_fallback(Role.GUARDIAN), goal=replace(
                    _fallback(Role.GUARDIAN).goal,
                    option_instance_id=search.goal.option_instance_id,
                ))),
            )
        )
    bad_hold = replace(_fallback(Role.GUARDIAN), feasible=False, infeasible_reason="not feasible")
    with pytest.raises(ValueError, match="must always be feasible"):
        selector.select(_snapshot(Role.GUARDIAN, (search, bad_hold)))


@pytest.mark.parametrize(
    "changes",
    [
        {"current_stable_key": "current"},
        {"current_option_instance_id": "current"},
        {"current_started_at_ns": 1},
        {"current_stable_key": "current", "current_option_instance_id": "current", "current_started_at_ns": 20_000_000_000},
    ],
)
def test_current_selection_identity_and_time_must_be_complete_and_valid(changes):
    with pytest.raises(ValueError, match="current"):
        _selector(Role.GUARDIAN).select(
            _snapshot(Role.GUARDIAN, (_fallback(Role.GUARDIAN),)),
            **changes,
        )


def test_selector_rejects_nonmonotonic_time_without_state_mutation():
    selector = _selector(Role.GUARDIAN)
    snapshot = _snapshot(Role.GUARDIAN, (_fallback(Role.GUARDIAN),))
    first = selector.select(snapshot)
    before = selector._last_now_ns
    with pytest.raises(ValueError, match="monotonic"):
        selector.select(
            _snapshot(
                Role.GUARDIAN,
                (_fallback(Role.GUARDIAN),),
                now_ns=before - 1,
            )
        )
    assert selector._last_now_ns == before
    assert first.selected.goal.kind == OptionKind.HOLD_SAFE


def test_candidate_bound_is_checked_and_hard_stop_still_reports_evaluations():
    selector = _selector(Role.GUARDIAN, max_proposals=1)
    second_fallback = _proposal(
        OptionKind.HOLD_SAFE,
        Role.GUARDIAN,
        stable_key="hold-2",
        instance_id="hold-2",
    )
    with pytest.raises(ValueError, match="proposal count"):
        selector.select(
            _snapshot(Role.GUARDIAN, (_fallback(Role.GUARDIAN), second_fallback))
        )
    proposals = (
        _proposal(OptionKind.APPROACH_CAPTURE, Role.GUARDIAN),
        _fallback(Role.GUARDIAN),
    )
    stopped = _selector(Role.GUARDIAN).select(
        TacticalSnapshot(
            context=_context(Role.GUARDIAN, safety_stop=True),
            proposals=proposals,
            system_ready=True,
            health_reason="",
        )
    )
    assert stopped.selected.goal.kind == OptionKind.HOLD_SAFE
    assert len(stopped.alternatives) == 2


def test_rejected_future_snapshot_does_not_advance_selector_monotonic_clock():
    selector = _selector(Role.GUARDIAN, max_proposals=1)
    selector.select(_snapshot(Role.GUARDIAN, (_fallback(Role.GUARDIAN),)))
    too_many = (
        _fallback(Role.GUARDIAN),
        _proposal(
            OptionKind.HOLD_SAFE,
            Role.GUARDIAN,
            stable_key="second-hold",
            instance_id="second-hold",
        ),
    )
    with pytest.raises(ValueError, match="proposal count"):
        selector.select(
            _snapshot(Role.GUARDIAN, too_many, now_ns=20_000_000_000)
        )
    valid = selector.select(
        _snapshot(
            Role.GUARDIAN,
            (_fallback(Role.GUARDIAN),),
            now_ns=15_000_000_000,
        )
    )
    assert valid.selected.goal.kind == OptionKind.HOLD_SAFE


def test_selection_and_alternatives_are_invariant_to_proposal_order():
    search = _proposal(
        OptionKind.SEARCH_PORTAL,
        Role.GUARDIAN,
        stable_key="search",
        instance_id="option-search",
        features={name: 0.3 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    recover = _proposal(
        OptionKind.RECOVER_VIEW,
        Role.GUARDIAN,
        stable_key="recover",
        instance_id="option-recover",
        features={name: 0.3 for name in ROLE_FEATURES[Role.GUARDIAN]},
    )
    fallbacks = _fallback(Role.GUARDIAN)
    left = _selector(Role.GUARDIAN).select(
        _snapshot(Role.GUARDIAN, (search, recover, fallbacks))
    )
    right = _selector(Role.GUARDIAN).select(
        _snapshot(Role.GUARDIAN, (fallbacks, recover, search))
    )
    assert left.selected == right.selected
    assert left.alternatives == right.alternatives


def test_parallel_evaluation_is_deterministic_and_single_selector_is_thread_safe():
    proposals = (
        _proposal(OptionKind.SEARCH_PORTAL, Role.GUARDIAN, stable_key="search"),
        _proposal(
            OptionKind.RECOVER_VIEW,
            Role.GUARDIAN,
            stable_key="recover",
            instance_id="recover-instance",
        ),
        _fallback(Role.GUARDIAN),
    )
    snapshots = tuple(
        _snapshot(Role.GUARDIAN, proposals)
        for _ in range(8)
    )
    selector = _selector(Role.GUARDIAN)
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = tuple(pool.map(lambda item: selector.select(item), snapshots))
    assert all(result.selected == results[0].selected for result in results)
    assert all(result.alternatives == results[0].alternatives for result in results)
