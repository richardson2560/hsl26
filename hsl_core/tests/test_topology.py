# hsl_core/tests/test_topology.py
"""SIL contract tests for the Phase-3 versioned topology boundary."""

import pytest

from hsl_core.topology import (
    EdgeState,
    NodeKind,
    TopologyEdge,
    TopologyGraph,
    TopologyNode,
    validate_graph,
)


def _graph() -> TopologyGraph:
    nodes = (
        TopologyNode(10, 0.0, 0.0, NodeKind.JUNCTION, 0.6),
        TopologyNode(20, 1.0, 0.0, NodeKind.PORTAL, 0.45),
        TopologyNode(30, 1.0, 1.0, NodeKind.PORTAL, 0.45),
    )
    edges = (
        TopologyEdge(100, 10, 20, ((0.0, 0.0), (1.0, 0.0)), 0.45),
        TopologyEdge(101, 10, 20, ((0.0, 0.0), (0.5, 0.2), (1.0, 0.0)), 0.45),
        TopologyEdge(102, 20, 30, ((1.0, 0.0), (1.0, 1.0)), 0.50),
    )
    return TopologyGraph(4, 7, "epoch-0", nodes, edges)


def test_parallel_edges_keep_distinct_identity_and_metric_polyline_length():
    graph = _graph()
    assert [edge.edge_id for edge in graph.adjacency()[10]] == [
        100,
        101,
    ]
    assert [edge.edge_id for edge in graph.adjacency()[20]] == [
        100,
        101,
        102,
    ]
    assert graph.adjacency()[20][0].from_node == 20
    assert graph.edges[1].metric_length_m > 1.0
    validate_graph(graph)


@pytest.mark.parametrize(
    "factory, message",
    [
        (
            lambda: TopologyGraph(
                0,
                0,
                "epoch",
                (TopologyNode(1, 0.0, 0.0, NodeKind.JUNCTION, 0.5),),
                (
                    TopologyEdge(1, 1, 2, ((0.0, 0.0), (1.0, 0.0)), 0.5),
                ),
            ),
            "existing nodes",
        ),
        (
            lambda: TopologyGraph(
                0,
                0,
                "epoch",
                (
                    TopologyNode(1, 0.0, 0.0, NodeKind.JUNCTION, 0.5),
                    TopologyNode(1, 1.0, 0.0, NodeKind.PORTAL, 0.5),
                ),
                (),
            ),
            "unique",
        ),
    ],
)
def test_graph_rejects_invalid_identity(factory, message):
    with pytest.raises(ValueError, match=message):
        factory()


def test_non_structural_edge_cannot_be_traversable():
    graph = TopologyGraph(
        0,
        0,
        "epoch",
        (
            TopologyNode(1, 0.0, 0.0, NodeKind.JUNCTION, 0.5),
            TopologyNode(2, 1.0, 0.0, NodeKind.PORTAL, 0.5),
        ),
        (TopologyEdge(1, 1, 2, ((0.0, 0.0), (1.0, 0.0)), 0.5, state=EdgeState.OPEN),),
    )
    validate_graph(graph)
