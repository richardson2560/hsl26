# hsl_core/tests/test_p44_topological_belief.py
"""Adversarial P4.4 tests for graph reachability and observation updates."""

import math

import pytest

from hsl_core.perception.topological_belief import (
    BeliefCell,
    CoverageInterval,
    NegativeObservation,
    TopologicalBelief,
)
from hsl_core.topology import EdgeState, NodeKind, TopologyEdge, TopologyGraph, TopologyNode


def _graph(*, edge_state=EdgeState.OPEN, length=10.0, branch=False):
    nodes = [
        TopologyNode(0, 0.0, 0.0, NodeKind.ANCHOR, 1.0),
        TopologyNode(1, length, 0.0, NodeKind.JUNCTION, 1.0),
    ]
    edges = [
        TopologyEdge(10, 0, 1, ((0.0, 0.0), (length, 0.0)), 0.5, state=edge_state)
    ]
    if branch:
        nodes.extend(
            [
                TopologyNode(2, length + 5.0, 5.0, NodeKind.DEAD_END, 1.0),
                TopologyNode(3, length + 5.0, -5.0, NodeKind.DEAD_END, 1.0),
            ]
        )
        edges.extend(
            [
                TopologyEdge(11, 1, 2, ((length, 0.0), (length + 5.0, 5.0)), 0.5),
                TopologyEdge(12, 1, 3, ((length, 0.0), (length + 5.0, -5.0)), 0.5),
            ]
        )
    return TopologyGraph(1, 2, "epoch-0", tuple(nodes), tuple(edges))


def _parallel_graph():
    nodes = (
        TopologyNode(0, 0.0, 0.0, NodeKind.ANCHOR, 1.0),
        TopologyNode(1, 10.0, 0.0, NodeKind.DEAD_END, 1.0),
        TopologyNode(2, 0.0, 1.0, NodeKind.ANCHOR, 1.0),
        TopologyNode(3, 10.0, 1.0, NodeKind.DEAD_END, 1.0),
    )
    edges = (
        TopologyEdge(10, 0, 1, ((0.0, 0.0), (10.0, 0.0)), 0.5),
        TopologyEdge(20, 2, 3, ((0.0, 1.0), (10.0, 1.0)), 0.5),
    )
    return TopologyGraph(1, 2, "epoch-0", nodes, edges)


def _frontier_graph():
    nodes = (
        TopologyNode(0, 0.0, 0.0, NodeKind.ANCHOR, 1.0),
        TopologyNode(1, 2.0, 0.0, NodeKind.FRONTIER, 1.0),
    )
    edges = (
        TopologyEdge(10, 0, 1, ((0.0, 0.0), (2.0, 0.0)), 0.5),
    )
    return TopologyGraph(1, 2, "epoch-0", nodes, edges)


def _belief(
    cells=None,
    *,
    unknown=0.0,
    graph=None,
    last_measurement=0.0,
    current=0.0,
    vmax=1.0,
    v0=None,
    acceleration=None,
):
    return TopologicalBelief(
        graph=graph or _graph(),
        cells=tuple(cells if cells is not None else [BeliefCell(10, 0.0, 0.0, 0.1, 1.0)]),
        unknown_mass=unknown,
        last_measurement_stamp_s=last_measurement,
        current_stamp_s=current,
        max_speed_mps=vmax,
        initial_speed_bound_mps=v0,
        acceleration_bound_mps2=acceleration,
    )


def _negative(
    stamp,
    group,
    coverage,
    *,
    valid=True,
    map_version=1,
    topology_version=2,
    epoch="epoch-0",
):
    return NegativeObservation(
        stamp, group, map_version, topology_version, epoch, tuple(coverage), valid
    )


def test_long_edge_cannot_reach_far_endpoint_before_bounded_travel_time():
    belief = _belief()
    before = belief.propagate(9.9)
    assert all(cell.s_end_m < 10.0 - 1e-9 for cell in before.cells)
    at_arrival = belief.propagate(10.0)
    assert any(cell.s_end_m == pytest.approx(10.0) for cell in at_arrival.cells)
    assert sum(cell.mass for cell in before.cells) + before.unknown_mass == pytest.approx(1.0)


def test_acceleration_envelope_uses_integrated_distance_not_vmax_times_t():
    belief = _belief(v0=0.0, acceleration=1.0)
    propagated = belief.propagate(1.0)
    assert max(cell.s_end_m for cell in propagated.cells) == pytest.approx(0.5)


def test_belief_branches_only_after_reaching_junction_and_conserves_mass():
    graph = _graph(length=1.0, branch=True)
    belief = _belief(
        [BeliefCell(10, 0.0, 0.0, 0.1, 1.0)],
        graph=graph,
        vmax=1.0,
    )
    just_short = belief.propagate(0.99)
    assert {cell.edge_id for cell in just_short.cells} == {10}
    at_junction = belief.propagate(1.5)
    assert {cell.edge_id for cell in at_junction.cells} == {10, 11, 12}
    assert sum(cell.mass for cell in at_junction.cells) == pytest.approx(1.0)


def test_frontier_adds_explicit_unknown_branch_only_when_reachable():
    graph = _frontier_graph()
    belief = _belief(
        [BeliefCell(10, 0.9, 0.9, 0.1, 1.0)],
        graph=graph,
        vmax=1.0,
    )
    before = belief.propagate(1.0)
    assert before.unknown_mass == 0.0
    reached = belief.propagate(1.2)
    assert reached.unknown_mass == pytest.approx(0.5)
    assert sum(cell.mass for cell in reached.cells) + reached.unknown_mass == pytest.approx(1.0)


def test_equal_prior_negative_likelihoods_match_normative_t18():
    graph = _graph(length=2.0, branch=True)
    belief = _belief(
        [
            BeliefCell(10, 0.0, 1.0, 0.1, 0.5),
            BeliefCell(11, 0.0, 1.0, 0.1, 0.5),
        ],
        graph=graph,
    )
    scan = _negative(
        0.0,
        "scan-1",
        [CoverageInterval(10, 0.0, 1.0, 0.95)],
    )
    result = belief.apply_negative_observation(
        scan, now_s=0.0, max_age_s=0.2, unknown_likelihood=1.0
    )
    assert result.accepted
    posterior = {cell.edge_id: cell.mass for cell in result.belief.cells}
    assert posterior[10] == pytest.approx(0.047619047619, rel=1e-9)
    assert posterior[11] == pytest.approx(0.952380952381, rel=1e-9)
    assert result.belief.unknown_mass == 0.0


def test_occluded_blind_or_uncovered_regions_keep_their_mass():
    belief = _belief([BeliefCell(10, 0.0, 5.0, 0.2, 0.7)], unknown=0.3)
    invalid = belief.apply_negative_observation(
        _negative(0.0, "blind", [CoverageInterval(10, 0.0, 5.0, 1.0)], valid=False),
        now_s=0.0,
        max_age_s=1.0,
    )
    assert not invalid.accepted
    assert invalid.belief == belief
    uncovered = belief.apply_negative_observation(
        _negative(0.0, "no-coverage", []), now_s=0.0, max_age_s=1.0
    )
    assert uncovered.accepted
    assert uncovered.belief.cells == belief.cells
    assert uncovered.belief.unknown_mass == belief.unknown_mass


def test_partial_coverage_averages_pd_over_cell_measure():
    belief = _belief(
        [
            BeliefCell(10, 0.0, 2.0, 0.0, 0.5),
            BeliefCell(10, 4.0, 6.0, 0.0, 0.5),
        ]
    )
    result = belief.apply_negative_observation(
        _negative(0.0, "partial", [CoverageInterval(10, 0.0, 1.0, 1.0)]),
        now_s=0.0,
        max_age_s=1.0,
    )
    posterior = {cell.s_begin_m: cell.mass for cell in result.belief.cells}
    assert posterior[0.0] == pytest.approx(1.0 / 3.0)
    assert posterior[4.0] == pytest.approx(2.0 / 3.0)
    # Half the interval is observed with PD=1; the other half remains possible.
    assert result.reason == "UPDATED_COVERED_REGIONS"


def test_future_negative_observation_propagates_prior_to_scan_time():
    belief = _belief()
    result = belief.apply_negative_observation(
        _negative(1.0, "future-scan", []),
        now_s=1.0,
        max_age_s=0.2,
    )
    assert result.accepted
    assert result.belief.current_stamp_s == pytest.approx(1.0)
    assert max(cell.s_end_m for cell in result.belief.cells) == pytest.approx(1.0)


def test_negative_evidence_rejects_stale_duplicate_out_of_order_and_version_mismatch():
    belief = _belief([BeliefCell(10, 0.0, 2.0, 0.0, 1.0)])
    stale = belief.apply_negative_observation(
        _negative(0.0, "stale", []), now_s=1.0, max_age_s=0.5
    )
    assert stale.reason == "STALE_OBSERVATION"
    mismatch = belief.apply_negative_observation(
        _negative(1.0, "wrong-version", [], topology_version=99),
        now_s=1.0,
        max_age_s=1.0,
    )
    assert mismatch.reason == "VERSION_MISMATCH"
    first = belief.apply_negative_observation(
        _negative(0.8, "group", []), now_s=1.0, max_age_s=1.0
    )
    duplicate = first.belief.apply_negative_observation(
        _negative(0.8, "group", []), now_s=1.0, max_age_s=1.0
    )
    assert duplicate.reason == "DUPLICATE_OBSERVATION_GROUP"
    older = first.belief.apply_negative_observation(
        _negative(0.7, "older", []), now_s=1.0, max_age_s=1.0
    )
    assert older.reason == "OUT_OF_ORDER_OBSERVATION"


def test_zero_normalizer_broadens_to_unknown_instead_of_dividing_by_zero():
    belief = _belief([BeliefCell(10, 0.0, 2.0, 0.0, 1.0)])
    result = belief.apply_negative_observation(
        _negative(0.0, "perfect", [CoverageInterval(10, 0.0, 2.0, 1.0)]),
        now_s=0.0,
        max_age_s=1.0,
        unknown_likelihood=0.0,
    )
    assert not result.accepted
    assert result.reason == "INCONSISTENT_EVIDENCE_BROADENED_TO_UNKNOWN"
    assert result.belief.unknown_mass == 1.0
    assert result.belief.cells == ()


def test_reacquisition_reweights_and_renormalizes_without_erasing_unknown():
    graph = _parallel_graph()
    belief = _belief(
        [
            BeliefCell(10, 0.0, 0.0, 0.1, 0.45),
            BeliefCell(20, 0.0, 0.0, 0.1, 0.45),
        ],
        unknown=0.1,
        graph=graph,
    )
    result = belief.reacquire(
        (0.1, 0.0),
        (0.25, 0.0, 0.0, 0.25),
        stamp_s=0.1,
        map_version=1,
        topology_version=2,
        localization_epoch="epoch-0",
    )
    assert result.accepted
    posterior = {cell.edge_id: cell.mass for cell in result.belief.cells}
    assert posterior[10] > posterior[20]
    assert result.belief.unknown_mass > 0.0
    assert sum(cell.mass for cell in result.belief.cells) + result.belief.unknown_mass == pytest.approx(1.0)


def test_reacquisition_rejects_wrong_versions_and_stale_measurement():
    belief = _belief()
    kwargs = dict(
        position_xy_m=(0.0, 0.0),
        covariance_xy_m2=(0.1, 0.0, 0.0, 0.1),
        stamp_s=0.1,
        map_version=1,
        topology_version=2,
        localization_epoch="epoch-0",
    )
    wrong = belief.reacquire(**{**kwargs, "map_version": 9})
    assert wrong.reason == "VERSION_MISMATCH"
    stale = belief.reacquire(**{**kwargs, "stamp_s": 0.0})
    assert stale.reason == "STALE_REACQUISITION"
    with pytest.raises(ValueError):
        belief.reacquire(**{**kwargs, "covariance_xy_m2": (1.0, 0.0, 0.0, -1.0)})


def test_blocked_edge_is_not_used_as_a_route_and_mass_is_preserved():
    graph = _graph(edge_state=EdgeState.BLOCKED, length=10.0, branch=True)
    belief = _belief(
        [BeliefCell(10, 9.0, 10.0, 0.0, 1.0)],
        graph=graph,
        vmax=2.0,
    )
    propagated = belief.propagate(1.0)
    assert all(cell.edge_id == 10 for cell in propagated.cells)
    assert sum(cell.mass for cell in propagated.cells) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "cells,unknown",
    [
        ([BeliefCell(10, 0.0, 1.0, 0.0, 0.8)], 0.1),
        ([BeliefCell(999, 0.0, 1.0, 0.0, 1.0)], 0.0),
        ([BeliefCell(10, 0.0, 11.0, 0.0, 1.0)], 0.0),
    ],
)
def test_invalid_initial_belief_mass_or_edge_support_is_rejected(cells, unknown):
    with pytest.raises(ValueError):
        _belief(cells, unknown=unknown)


def test_invalid_coverage_intervals_are_not_applied():
    belief = _belief()
    result = belief.apply_negative_observation(
        _negative(0.0, "bad-coverage", [CoverageInterval(10, 0.0, 12.0, 1.0)]),
        now_s=0.0,
        max_age_s=1.0,
    )
    assert result.reason == "INVALID_COVERAGE_INTERVAL"
    assert result.belief == belief


def test_negative_observation_rejects_non_boolean_valid_flag():
    with pytest.raises(ValueError):
        NegativeObservation(0.0, "bad", 1, 2, "epoch-0", (), 1)


def test_nonfinite_and_nonmonotonic_propagation_are_rejected():
    belief = _belief()
    with pytest.raises(ValueError):
        belief.propagate(math.nan)
    with pytest.raises(ValueError):
        belief.propagate(-0.1)
