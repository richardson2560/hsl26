"""Acceptance tests for selection telemetry, not for learned performance."""

from dataclasses import asdict, replace
import json

import pytest

from hsl_core.match import Role, StagePhase
from hsl_core.planning import PlannedPath
from hsl_core.tactics import (
    OptionContext, OptionGoal, OptionKind, OptionProposal, OptionRegistry,
    ROLE_FEATURES, TacticalGuard, TacticalSelector, TacticalSnapshot,
)
from hsl_core.topology import EdgeState, NodeKind, TopologyEdge, TopologyGraph, TopologyNode
from sim.kinematic.autonomous import _utility_profile
from sim.kinematic.selection_diagnostics import (
    SelectionDiagnostics, behavior_signature, begin_selection, finish_selection,
)


def _graph():
    points = ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))
    return TopologyGraph(7, 9, "loc", tuple(
        TopologyNode(i, *p, NodeKind.PORTAL, 0.5) for i, p in enumerate(points)
    ), tuple(
        TopologyEdge(i, 0, i, (points[0], points[i]), 0.5, state=EdgeState.OPEN)
        for i in range(1, 5)
    ))


def _proposal(kind=OptionKind.SEARCH_PORTAL, *, target=1, key=None, role=Role.GUARDIAN):
    key = key or f"{kind.name}:{target}"
    no_target = kind in (OptionKind.HOLD_SAFE, OptionKind.OBSERVE_SAFE)
    goal = OptionGoal(key + ":instance", kind, role, "stage", "clock", "loc",
                      10_000_000_000, 7, 9, "parameters", 2,
                      has_target_node=not no_target, target_node_id=0 if no_target else target,
                      position_tolerance_m=0.0 if kind == OptionKind.HOLD_SAFE else 0.05,
                      yaw_tolerance_rad=0.0 if kind == OptionKind.HOLD_SAFE else 0.1)
    guards = {
        OptionKind.SEARCH_PORTAL: (TacticalGuard.GRAPH_SEARCH_VIEWPOINT,),
        OptionKind.PRESSURE_ROUTE: (TacticalGuard.USEFUL_RIVAL,),
        OptionKind.BREAK_LOS: (TacticalGuard.IMMEDIATE_TRAP_RISK, TacticalGuard.BREAK_LOS_FEASIBLE),
        OptionKind.KEEP_ESCAPE_ROUTE: (TacticalGuard.IMMEDIATE_TRAP_RISK, TacticalGuard.VERIFIED_ESCAPE_ROUTE),
        OptionKind.OBSERVE_SAFE: (TacticalGuard.INFORMATIVE_SAFE_OBSERVATION,),
    }.get(kind, ())
    escape = kind in (OptionKind.BREAK_LOS, OptionKind.KEEP_ESCAPE_ROUTE)
    return OptionProposal(
        key, goal, True, "", guards, tuple((g, "evidence") for g in guards),
        () if kind == OptionKind.HOLD_SAFE else tuple((f, 0.0) for f in ROLE_FEATURES[role]),
        urgency_rank=1 if escape else None, urgency_evidence_id="threat" if escape else "",
    )


def _path(graph, target=1):
    edge = next(e for e in graph.edges if e.to_node == target)
    return PlannedPath(7, 9, "loc", 0, target, (0, target), (edge.edge_id,),
                       edge.polyline_xy_m, edge.metric_length_m)


def _measure(proposals, *, role=Role.GUARDIAN, snapshot_changes=None, context_changes=None,
             paths=None, graph=None):
    graph = graph or _graph()
    context = OptionContext(
        now_ns=1_000_000_000, stage_id="stage", stage_phase=StagePhase.ACTIVE,
        stage_ends_at_ns=10_000_000_000, role=role, clock_epoch="clock",
        localization_epoch="loc", map_version=7, topology_version=9,
        motion_authorized=True, lease_valid=True, safety_stop=False,
    )
    context = replace(context, **(context_changes or {}))
    snapshot = TacticalSnapshot(context, (*proposals, _proposal(OptionKind.HOLD_SAFE, role=role)), True, "")
    snapshot = replace(snapshot, **(snapshot_changes or {}))
    selector = TacticalSelector(_utility_profile(role), registry=OptionRegistry(),
                                hysteresis_delta_u=0.0, minimum_dwell_ns=0, max_proposals=256)
    result = selector.select(snapshot)
    if paths is None:
        paths = {p.stable_key: _path(graph, p.goal.target_node_id)
                 for p in proposals if p.goal.has_target_node}
    base = begin_selection(SelectionDiagnostics(cycle_sequence=1), snapshot)
    diagnostic = finish_selection(base, snapshot, result, paths, graph)
    return diagnostic, result


@pytest.mark.parametrize("size", [0, 1, 4])
def test_partition_counts_invocations_not_proposals(size):
    diagnostic, result = _measure(tuple(_proposal(target=i) for i in range(1, size + 1)))
    assert diagnostic.diagnostic_valid
    assert diagnostic.proposal_total == size + 1  # Includes HOLD_SAFE.
    assert diagnostic.proposal_feasible == size
    assert diagnostic.proposal_choice == size
    assert diagnostic.n_active == 1
    assert diagnostic.n_available == int(size > 0)
    assert diagnostic.n_choice == int(size >= 2)
    assert diagnostic.n_choice + diagnostic.n_forced + diagnostic.n_empty == 1
    assert diagnostic.candidate_evaluations == result.alternatives
    assert diagnostic.selected_stable_key == result.selected.stable_key
    assert diagnostic.utility_profile_id == result.utility_profile_id
    assert len(diagnostic.candidate_features) == size + 1
    json.dumps(asdict(diagnostic), allow_nan=False)


@pytest.mark.parametrize("change,context", [
    ({"system_ready": False, "health_reason": "sensor_missing"}, {}),
    ({}, {"safety_stop": True}), ({}, {"lease_valid": False}),
    ({}, {"motion_authorized": False}),
])
def test_global_gates_override_raw_feasibility(change, context):
    d, result = _measure((_proposal(),), snapshot_changes=change, context_changes=context)
    assert result.selected.goal.kind == OptionKind.HOLD_SAFE
    assert d.proposal_feasible_raw == 1
    assert d.proposal_feasible == d.n_available == d.n_choice == d.n_forced == 0
    assert d.n_active == d.n_empty == 1
    assert d.rejected_candidates


@pytest.mark.parametrize("failure", ["guard", "registry", "feature_schema", "raw_infeasible"])
def test_final_selector_rejections_are_not_available(failure):
    p = _proposal()
    if failure == "guard":
        p = replace(p, guard_facts=(), guard_evidence=())
    elif failure == "registry":
        p = replace(p, goal=replace(p.goal, map_version=8))
    elif failure == "feature_schema":
        p = replace(p, features=())
    else:
        p = replace(p, feasible=False, infeasible_reason="no_path")
    d, _ = _measure((p,))
    assert d.n_available == 0 and d.n_empty == 1
    assert d.proposal_feasible == 0 and d.rejected_candidates[0][0] == p.stable_key


@pytest.mark.parametrize("phase", [StagePhase.INIT, StagePhase.FREEZE, StagePhase.TERMINAL])
def test_non_active_invocations_do_not_inflate_active_counts(phase):
    d, _ = _measure((_proposal(),), context_changes={"stage_phase": phase, "motion_authorized": False})
    assert d.selector_invoked and d.phase == phase.name
    assert (d.n_active, d.n_available, d.n_choice, d.n_forced, d.n_empty) == (0, 0, 0, 0, 0)


def test_only_highest_priority_cohort_counts_for_choice():
    d, _ = _measure((_proposal(OptionKind.PRESSURE_ROUTE), _proposal(target=2), _proposal(target=3)))
    assert d.proposal_feasible == 3
    assert d.winning_cohort_size == d.proposal_choice == 1
    assert d.winning_priority == 4 and d.n_choice == 0 and d.n_forced == 1


def test_only_highest_urgency_within_priority_counts_for_choice():
    low = _proposal(OptionKind.KEEP_ESCAPE_ROUTE, role=Role.EXPLORER)
    high = replace(_proposal(OptionKind.BREAK_LOS, role=Role.EXPLORER, target=2), urgency_rank=200)
    d, _ = _measure((low, high), role=Role.EXPLORER)
    assert d.proposal_feasible == 2 and d.winning_priority == 0
    assert d.winning_urgency == 200 and d.winning_cohort_keys == (high.stable_key,)
    assert d.proposal_choice == 1 and d.n_choice == 0


def test_equal_priority_and_urgency_allow_choice_across_escape_kinds():
    a = _proposal(OptionKind.KEEP_ESCAPE_ROUTE, role=Role.EXPLORER)
    b = _proposal(OptionKind.BREAK_LOS, role=Role.EXPLORER, target=2)
    d, _ = _measure((a, b), role=Role.EXPLORER)
    assert d.proposal_choice == 2 and d.n_choice == 1


def test_instance_ids_keys_and_redundant_points_do_not_create_diversity():
    graph = _graph()
    a, b = _proposal(key="a"), _proposal(key="b")
    path = _path(graph)
    redundant = replace(path, polyline_xy_m=((0.0, 0.0), (0.5, 0.0), (0.5, 0.0), (1.0, 0.0)))
    d, _ = _measure((a, b), paths={"a": path, "b": redundant}, graph=graph)
    assert d.diagnostic_valid and d.proposal_feasible == d.winning_cohort_size == 2
    assert d.proposal_choice == 1 and d.n_choice == 0 and d.deduplicated_keys == ("b",)


def test_same_prefix_with_later_divergence_is_distinct():
    graph = _graph()
    graph = replace(graph, edges=(
        replace(graph.edges[0], polyline_xy_m=((0.0, 0.0), (0.5, 0.0), (1.0, 0.0))),
        replace(graph.edges[1], polyline_xy_m=((0.0, 0.0), (0.5, 0.0), (0.0, 1.0))),
        *graph.edges[2:],
    ))
    d, _ = _measure((_proposal(), _proposal(target=2)), graph=graph)
    assert d.diagnostic_valid and d.n_choice == 1


def test_parallel_edges_remain_distinct_even_with_same_endpoints():
    graph = _graph()
    parallel = replace(graph.edges[0], edge_id=99)
    graph = replace(graph, edges=(*graph.edges, parallel))
    a, b = _proposal(key="a"), _proposal(key="b")
    path = _path(graph)
    d, _ = _measure((a, b), graph=graph, paths={"a": path, "b": replace(path, edge_ids=(99,))})
    assert d.diagnostic_valid and d.proposal_choice == 2 and d.n_choice == 1


def test_target_renumbering_alone_does_not_create_distinct_execution():
    graph = _graph()
    goal = _proposal().goal
    path = _path(graph)
    renamed_graph = replace(graph,
        nodes=tuple(replace(n, node_id=100) if n.node_id == 1 else n for n in graph.nodes),
        edges=tuple(replace(e, to_node=100) if e.to_node == 1 else e for e in graph.edges),
    )
    renamed = replace(path, goal_node_id=100, node_ids=(0, 100))
    assert behavior_signature(goal, path, graph) == behavior_signature(
        replace(goal, target_node_id=100), renamed, renamed_graph
    )


def test_completed_route_prefix_is_not_a_distinct_remaining_behavior():
    graph = _graph()
    graph = replace(graph, edges=(*graph.edges,
        TopologyEdge(99, 2, 0, ((0.0, 1.0), (0.0, 0.0)), 0.5, state=EdgeState.OPEN)))
    path = _path(graph)
    with_past = replace(path, start_node_id=2, node_ids=(2, 0, 1), edge_ids=(99, 1))
    assert behavior_signature(_proposal().goal, path, graph) == behavior_signature(
        _proposal().goal, with_past, graph
    )


def test_same_route_with_different_terminal_tolerance_is_distinct():
    graph = _graph()
    goal, path = _proposal().goal, _path(graph)
    assert behavior_signature(goal, path, graph) != behavior_signature(
        replace(goal, position_tolerance_m=0.2), path, graph
    )


def test_observation_requires_no_route_and_deduplicates_instances():
    a = _proposal(OptionKind.OBSERVE_SAFE, role=Role.EXPLORER, key="a")
    b = _proposal(OptionKind.OBSERVE_SAFE, role=Role.EXPLORER, key="b")
    d, _ = _measure((a, b), role=Role.EXPLORER)
    assert d.diagnostic_valid and d.proposal_choice == 1 and d.n_forced == 1


@pytest.mark.parametrize("failure", ["missing", "wrong_target", "versions", "geometry", "unknown", "blocked"])
def test_unresolved_motion_evidence_is_incomplete_not_a_fabricated_choice(failure):
    graph = _graph()
    p, path = _proposal(), _path(graph)
    paths = {p.stable_key: path}
    if failure == "missing":
        paths = {}
    elif failure == "wrong_target":
        paths = {p.stable_key: _path(graph, 2)}
    elif failure == "versions":
        paths[p.stable_key] = replace(path, map_version=8)
    elif failure == "geometry":
        paths[p.stable_key] = replace(path, polyline_xy_m=((0.0, 0.0), (0.7, 0.4), (1.0, 0.0)))
    else:
        graph = replace(graph, edges=(replace(graph.edges[0], state=EdgeState[failure.upper()]), *graph.edges[1:]))
    d, _ = _measure((p,), paths=paths, graph=graph)
    assert not d.diagnostic_valid and d.diagnostic_error.startswith("behavior_signature:")
    assert d.n_active == d.n_available == 1 and d.proposal_feasible == 1
    assert d.proposal_choice is d.n_choice is d.n_forced is None
    assert '"n_choice": null' in json.dumps(asdict(d), allow_nan=False)


def test_invalid_partition_is_rejected_without_python_asserts():
    with pytest.raises(ValueError, match="binary"):
        SelectionDiagnostics(phase="ACTIVE", selector_invoked=True, n_active=1, n_available=4)
    with pytest.raises(ValueError, match="partition"):
        SelectionDiagnostics(phase="ACTIVE", selector_invoked=True, n_active=1, n_available=1)


def test_pending_selector_result_is_explicitly_unknown():
    context = OptionContext(1, "stage", StagePhase.ACTIVE, 10, Role.GUARDIAN,
                            "clock", "loc", 7, 9, True, True, False)
    snapshot = TacticalSnapshot(context, (_proposal(OptionKind.HOLD_SAFE),), True, "")
    d = begin_selection(SelectionDiagnostics(), snapshot)
    assert d.selector_invoked and not d.diagnostic_valid
    assert d.n_active == 1 and d.n_available is None and d.n_empty is None
