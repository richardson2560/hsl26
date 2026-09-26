# hsl_core/tests/test_p35_planning_control.py
"""Adversarial P3.5 tests for metric planning, control and lease races."""

import pytest

from hsl_core.control import OptionExecutor, PursuitConfig, make_candidate
from hsl_core.planning import (
    CandidateEnvelope,
    ExecutionLease,
    PlannedPath,
    astar,
    dijkstra,
    smooth_polyline,
)
from hsl_core.topology import EdgeState, TopologyEdge, TopologyGraph, TopologyNode
from hsl_core.types import MotionCandidate, Pose2D


def _graph(block_middle=False):
    nodes = tuple(
        TopologyNode(index, float(index), 0.0, 0, 0.5)
        for index in range(4)
    )
    edges = (
        TopologyEdge(10, 0, 1, ((0.0, 0.0), (1.0, 0.0)), 0.5, state=EdgeState.OPEN),
        TopologyEdge(
            11, 1, 2, ((1.0, 0.0), (2.0, 0.0)), 0.5,
            state=EdgeState.BLOCKED if block_middle else EdgeState.OPEN,
        ),
        TopologyEdge(12, 2, 3, ((2.0, 0.0), (3.0, 0.0)), 0.5, state=EdgeState.OPEN),
        TopologyEdge(20, 0, 3, ((0.0, 0.0), (3.0, 1.0)), 0.5, cost=4.0, state=EdgeState.OPEN),
    )
    return TopologyGraph(7, 9, "epoch-a", nodes, edges)


def test_astar_matches_dijkstra_and_uses_metric_units():
    graph = _graph()
    planned = astar(graph, 0, 3)
    reference = dijkstra(graph, 0, 3)
    assert planned.cost == pytest.approx(reference.cost)
    assert planned.node_ids == (0, 1, 2, 3)
    assert planned.edge_ids == (10, 11, 12)
    assert planned.map_version == 7
    assert planned.topology_version == 9


def test_blocked_overlay_excludes_edge_but_preserves_structural_identity():
    graph = _graph(block_middle=True)
    planned = astar(graph, 0, 3)
    assert planned.edge_ids == (20,)
    assert 11 in {edge.edge_id for edge in graph.edges}


def test_unreachable_goal_is_explicit_failure_not_fake_path():
    graph = _graph(block_middle=True)
    isolated = TopologyGraph(
        graph.map_version,
        graph.topology_version,
        graph.localization_epoch,
        graph.nodes,
        tuple(edge for edge in graph.edges if edge.edge_id != 20),
    )
    with pytest.raises(ValueError, match="unreachable"):
        astar(isolated, 0, 3)


def test_smoothing_requires_external_swept_validator_and_never_shortcuts_walls():
    path = astar(_graph(), 0, 3)
    rejected = smooth_polyline(path, segment_is_valid=lambda _a, _b: False)
    assert rejected.polyline_xy_m == path.polyline_xy_m
    accepted = smooth_polyline(path, segment_is_valid=lambda _a, _b: True)
    assert accepted.polyline_xy_m == (path.polyline_xy_m[0], path.polyline_xy_m[-1])
    with pytest.raises(ValueError):
        smooth_polyline(path, segment_is_valid=None)


def test_regulated_pursuit_respects_speed_yaw_and_lateral_acceleration():
    path = astar(_graph(), 0, 3)
    candidate = make_candidate(
        path,
        Pose2D(0.0, 0.0, 0.0),
        config=PursuitConfig(speed_max_mps=0.4, yaw_rate_max_rps=1.0),
        now_s=2.0,
        lease_s=0.2,
        source_id="option-1",
        lease_generation=1,
    )
    assert 0.0 <= candidate.linear_velocity_mps <= 0.4
    assert abs(candidate.angular_velocity_rps) <= 1.0
    assert candidate.valid_until_s == pytest.approx(2.2)
    assert candidate.map_version == 7


def test_lease_cancellation_and_replacement_reject_old_candidates():
    graph = _graph()
    executor = OptionExecutor()
    generation_one = executor.start("old")
    old = executor.propose(
        astar(graph, 0, 3), Pose2D(0.0, 0.0, 0.0),
        option_instance_id="old", generation=generation_one,
        config=PursuitConfig(), now_s=1.0, lease_s=1.0,
    )
    executor.cancel("old")
    generation_two = executor.start("new")
    with pytest.raises(ValueError, match="inactive lease"):
        executor._lease.admit(old, now_s=1.1, generation=generation_one)
    with pytest.raises(ValueError, match="cancelled"):
        executor._lease.admit(old, now_s=1.1, generation=generation_two)


def test_candidate_rejects_stale_version_and_expired_lease():
    graph = _graph()
    lease = ExecutionLease()
    generation = lease.activate("option")
    path = astar(graph, 0, 3)
    candidate = MotionCandidate(
        0.1, 0.0, 1.0, 1.5, 0.5, "option", 8, "epoch-a", 9, generation
    )
    envelope = CandidateEnvelope(candidate, path, "option")
    with pytest.raises(ValueError, match="map version"):
        lease.admit(envelope, now_s=1.1, generation=generation)
    expired = MotionCandidate(
        0.1, 0.0, 1.0, 1.5, 0.5, "option", 7, "epoch-a", 9, generation
    )
    with pytest.raises(ValueError, match="expired"):
        lease.admit(CandidateEnvelope(expired, path, "option"), now_s=1.5, generation=generation)


def test_lease_requires_cancellation_and_active_instance_ownership():
    lease = ExecutionLease()
    generation = lease.activate("active", candidate_authorized=False)
    graph = _graph()
    path = astar(graph, 0, 3)
    not_authorized = MotionCandidate(
        0.1, 0.0, 1.0, 2.0, 0.5, "active", 7, "epoch-a", 9, generation
    )
    with pytest.raises(ValueError, match="authority is revoked"):
        lease.admit(
            CandidateEnvelope(not_authorized, path, "active"),
            now_s=1.1,
            generation=generation,
        )
    lease.set_candidate_authorized("active", generation, True)
    wrong_owner = MotionCandidate(
        0.1, 0.0, 1.0, 2.0, 0.5, "other", 7, "epoch-a", 9, generation
    )
    with pytest.raises(ValueError, match="does not own"):
        lease.admit(
            CandidateEnvelope(wrong_owner, path, "other"),
            now_s=1.1,
            generation=generation,
        )
    active = CandidateEnvelope(not_authorized, path, "active")
    assert lease.admit(active, now_s=1.1, generation=generation) is not_authorized
    with pytest.raises(ValueError, match="cancelled before replacement"):
        lease.activate("next", candidate_authorized=False)


def test_lease_rejects_candidate_with_topology_version_different_from_path():
    lease = ExecutionLease()
    generation = lease.activate(
        "option",
        expected_map_version=7,
        expected_topology_version=9,
        expected_localization_epoch="epoch-a",
    )
    path = astar(_graph(), 0, 3)
    wrong_topology = MotionCandidate(
        0.1, 0.0, 1.0, 2.0, 0.5, "option", 7, "epoch-a", 8, generation
    )
    with pytest.raises(ValueError, match="topology version"):
        lease.admit(
            CandidateEnvelope(wrong_topology, path, "option"),
            now_s=1.1,
            generation=generation,
        )


def test_lease_rejects_matching_candidate_and_path_from_stale_option_topology():
    lease = ExecutionLease()
    generation = lease.activate(
        "option",
        expected_map_version=7,
        expected_topology_version=9,
        expected_localization_epoch="epoch-a",
    )
    path = astar(_graph(), 0, 3)
    stale_path = PlannedPath(
        map_version=path.map_version,
        topology_version=10,
        localization_epoch=path.localization_epoch,
        start_node_id=path.start_node_id,
        goal_node_id=path.goal_node_id,
        node_ids=path.node_ids,
        edge_ids=path.edge_ids,
        polyline_xy_m=path.polyline_xy_m,
        cost=path.cost,
    )
    stale_candidate = MotionCandidate(
        0.1, 0.0, 1.0, 2.0, 0.5, "option", 7, "epoch-a", 10, generation
    )
    with pytest.raises(ValueError, match="active option"):
        lease.admit(
            CandidateEnvelope(stale_candidate, stale_path, "option"),
            now_s=1.1,
            generation=generation,
        )


def test_invalid_edge_cost_cannot_make_euclidean_astar_heuristic_inadmissible():
    nodes = (
        TopologyNode(0, 0.0, 0.0, 0, 0.5),
        TopologyNode(1, 1.0, 0.0, 0, 0.5),
    )
    edge = TopologyEdge(1, 0, 1, ((0.0, 0.0), (1.0, 0.0)), 0.5, cost=0.5)
    with pytest.raises(ValueError, match="dominate"):
        astar(TopologyGraph(1, 1, "epoch", nodes, (edge,)), 0, 1)
