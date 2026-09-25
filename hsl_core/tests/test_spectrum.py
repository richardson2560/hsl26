# hsl_core/tests/test_spectrum.py
"""Adversarial SIL tests for the optional P3.3 spectral descriptor."""

import numpy as np
import pytest

from hsl_core.topology import (
    EdgeState,
    NodeKind,
    TopologyEdge,
    TopologyGraph,
    TopologyNode,
    compute_spectrum,
)


def _nodes(count):
    return tuple(
        TopologyNode(index + 1, float(index), 0.0, NodeKind.JUNCTION, 1.0)
        for index in range(count)
    )


def _graph(edges, count, version=7):
    return TopologyGraph(
        4,
        version,
        "epoch-0",
        _nodes(count),
        tuple(
            TopologyEdge(edge_id, left, right, ((float(left), 0.0), (float(right), 0.0)), length)
            for edge_id, left, right, length in edges
        ),
    )


def test_three_node_unit_path_has_normative_spectrum_and_one_component():
    signature = compute_spectrum(
        _graph(((1, 1, 2, 1.0), (2, 2, 3, 1.0)), 3)
    )
    assert signature.eigenvalues == pytest.approx((0.0, 1.0, 2.0))
    assert signature.component_count == 1
    assert signature.node_count == 3
    assert signature.valid is True


def test_triangle_has_two_repeated_nonzero_eigenvalues():
    signature = compute_spectrum(
        _graph(((1, 1, 2, 1.0), (2, 1, 3, 1.0), (3, 2, 3, 1.0)), 3)
    )
    assert signature.eigenvalues == pytest.approx((0.0, 1.5, 1.5))


def test_isolated_node_contributes_zero_and_component_count():
    signature = compute_spectrum(_graph(((1, 1, 2, 1.0),), 3))
    assert signature.eigenvalues == pytest.approx((0.0, 0.0, 2.0))
    assert signature.component_count == 2


def test_parallel_edges_are_summed_in_affinity_without_extra_nodes():
    graph = _graph(((1, 1, 2, 1.0), (2, 1, 2, 1.0)), 2)
    signature = compute_spectrum(graph)
    assert signature.eigenvalues == pytest.approx((0.0, 2.0))
    assert signature.node_count == 2


def test_inverse_length_normalized_spectrum_is_invariant_to_uniform_scale():
    first = compute_spectrum(
        _graph(((1, 1, 2, 1.0), (2, 2, 3, 2.0)), 3),
        affinity="inverse_length",
    )
    second = compute_spectrum(
        _graph(((1, 1, 2, 10.0), (2, 2, 3, 20.0)), 3),
        affinity="inverse_length",
    )
    assert first.eigenvalues == pytest.approx(second.eigenvalues)


def test_node_permutation_preserves_spectrum():
    original = _graph(((1, 1, 2, 1.0), (2, 2, 3, 1.0)), 3)
    permuted = TopologyGraph(
        original.map_version,
        original.topology_version,
        original.localization_epoch,
        (
            original.nodes[2],
            original.nodes[0],
            original.nodes[1],
        ),
        original.edges,
    )
    assert compute_spectrum(original).eigenvalues == pytest.approx(
        compute_spectrum(permuted).eigenvalues
    )


def test_local_k_hop_recomputes_degrees_and_is_not_global_submatrix():
    graph = _graph(
        ((1, 1, 2, 1.0), (2, 2, 3, 1.0), (3, 3, 4, 1.0)),
        4,
    )
    local = compute_spectrum(graph, center_node_id=2, hops=1)
    global_signature = compute_spectrum(graph)
    assert local.node_count == 3
    assert len(local.eigenvalues) == local.node_count
    assert local.eigenvalues != pytest.approx(global_signature.eigenvalues[:3])


def test_spectral_signature_records_version_scope_and_affinity():
    signature = compute_spectrum(
        _graph(((1, 1, 2, 1.0),), 2, version=11),
        center_node_id=1,
        hops=0,
        affinity="inverse_length",
    )
    assert signature.topology_version == 11
    assert signature.center_node_id == 1
    assert signature.hops == 0
    assert signature.affinity_definition == "inverse_length"
    assert signature.eigenvalues == pytest.approx((0.0,))


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"affinity": "bad"}, "affinity"),
        ({"center_node_id": 1}, "provided together"),
        ({"hops": 1}, "provided together"),
        ({"center_node_id": 99, "hops": 1}, "existing"),
        ({"center_node_id": 1, "hops": -1}, "non-negative"),
        ({"max_nodes": 0}, "max_nodes"),
    ],
)
def test_spectrum_rejects_invalid_scope_or_affinity(kwargs, message):
    with pytest.raises(ValueError, match=message):
        compute_spectrum(_graph(((1, 1, 2, 1.0),), 2), **kwargs)


def test_runtime_blocked_overlay_does_not_change_structural_spectrum():
    open_graph = _graph(((1, 1, 2, 1.0),), 2)
    blocked_edge = TopologyEdge(
        1,
        1,
        2,
        ((1.0, 0.0), (2.0, 0.0)),
        1.0,
        state=EdgeState.BLOCKED,
    )
    blocked_graph = TopologyGraph(4, 7, "epoch-0", _nodes(2), (blocked_edge,))
    assert compute_spectrum(open_graph).eigenvalues == pytest.approx(
        compute_spectrum(blocked_graph).eigenvalues
    )


def test_empty_graph_has_empty_valid_spectrum_without_fabricated_eigenvalues():
    graph = TopologyGraph(1, 1, "epoch-0", (), ())
    signature = compute_spectrum(graph)
    assert signature.valid is True
    assert signature.node_count == 0
    assert signature.component_count == 0
    assert signature.eigenvalues == ()


def test_spectrum_is_sorted_bounded_and_nonnegative():
    signature = compute_spectrum(
        _graph(((1, 1, 2, 1.0), (2, 2, 3, 1.0), (3, 1, 3, 1.0)), 3)
    )
    assert tuple(signature.eigenvalues) == tuple(sorted(signature.eigenvalues))
    assert all(0.0 <= value <= 2.0 for value in signature.eigenvalues)
