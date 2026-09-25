# hsl_core/tests/test_topology_extraction.py
"""Adversarial SIL tests for P3.2 grid-to-topology extraction."""

import numpy as np
import pytest

from hsl_core.mapping import OCCUPIED, OBSERVED_FREE, UNKNOWN
from hsl_core.topology import (
    NodeKind,
    TopologyBuildConfig,
    TopologyBuilder,
)


def _builder(radius=0.0):
    return TopologyBuilder(
        TopologyBuildConfig(
            footprint_radius_m=radius,
            grid_error_margin_m=0.0,
            max_cells=10_000,
        )
    )


def _corridor_grid():
    grid = np.full((9, 13), UNKNOWN, dtype=np.uint8)
    grid[4, 1:12] = OBSERVED_FREE
    return grid


def _extract(grid, radius=0.0):
    return _builder(radius).extract(
        grid,
        origin_xy_m=(0.0, 0.0),
        resolution_m=1.0,
        map_version=3,
        localization_epoch="epoch-0",
    )


def test_unknown_cells_are_not_route_free_and_frontier_is_not_dead_end():
    extraction = _extract(_corridor_grid())
    assert np.all(extraction.configuration_free[:, 0] == 0)
    assert any(node.kind == NodeKind.FRONTIER for node in extraction.graph.nodes)
    assert not any(
        node.kind == NodeKind.DEAD_END
        and node.x_m in (1.5, 11.5)
        for node in extraction.graph.nodes
    )


def test_diagonal_corner_touch_does_not_connect_regions():
    grid = np.full((5, 5), UNKNOWN, dtype=np.uint8)
    grid[1, 1] = OBSERVED_FREE
    grid[2, 2] = OBSERVED_FREE
    grid[1, 2] = OCCUPIED
    grid[2, 1] = OCCUPIED
    extraction = _extract(grid)
    assert len(extraction.graph.edges) == 0


def test_inflation_blocks_known_free_cell_within_footprint_radius():
    grid = np.full((9, 9), UNKNOWN, dtype=np.uint8)
    grid[4, 1:8] = OBSERVED_FREE
    grid[3, 4] = OCCUPIED
    extraction = _extract(grid, radius=1.0)
    assert extraction.configuration_free[4, 4] == 0


def test_pure_cycle_gets_one_deterministic_anchor_and_edges():
    grid = np.full((9, 9), OCCUPIED, dtype=np.uint8)
    grid[2, 2:7] = OBSERVED_FREE
    grid[6, 2:7] = OBSERVED_FREE
    grid[2:7, 2] = OBSERVED_FREE
    grid[2:7, 6] = OBSERVED_FREE
    first = _extract(grid)
    second = _extract(grid)
    anchors = [node for node in first.graph.nodes if node.kind == NodeKind.ANCHOR]
    assert len(anchors) == 1
    assert first.graph.nodes == second.graph.nodes
    assert first.graph.edges == second.graph.edges


def test_parallel_corridors_keep_distinct_edges_and_metric_polyline():
    grid = np.full((11, 13), UNKNOWN, dtype=np.uint8)
    grid[2, 1:12] = OBSERVED_FREE
    grid[8, 1:12] = OBSERVED_FREE
    grid[2:9, 1] = OBSERVED_FREE
    grid[2:9, 11] = OBSERVED_FREE
    extraction = _extract(grid)
    assert len(extraction.graph.edges) >= 2
    assert len({edge.edge_id for edge in extraction.graph.edges}) == len(extraction.graph.edges)
    assert all(edge.metric_length_m > 0.0 for edge in extraction.graph.edges)


def test_map_and_topology_versions_are_explicit_and_previous_increments():
    grid = _corridor_grid()
    first = _extract(grid)
    second = _builder().extract(
        grid,
        origin_xy_m=(0.0, 0.0),
        resolution_m=1.0,
        map_version=4,
        localization_epoch="epoch-0",
        previous_graph=first.graph,
    )
    assert first.graph.map_version == 3
    assert second.graph.map_version == 4
    assert second.graph.topology_version == first.graph.topology_version + 1


def test_extracted_edges_use_canonical_endpoint_order():
    extraction = _extract(_corridor_grid())
    assert all(edge.from_node <= edge.to_node for edge in extraction.graph.edges)


def test_reverse_trace_is_canonicalized_with_reversed_polyline():
    grid = np.full((7, 9), OCCUPIED, dtype=np.uint8)
    grid[3, 1:8] = OBSERVED_FREE
    builder = _builder()
    extraction = builder.extract(
        grid,
        origin_xy_m=(0.0, 0.0),
        resolution_m=1.0,
        map_version=1,
        localization_epoch="epoch-0",
    )
    assert all(edge.from_node < edge.to_node for edge in extraction.graph.edges)
    for edge in extraction.graph.edges:
        assert edge.polyline_xy_m[0] == (1.5, 3.5)


@pytest.mark.parametrize(
    "grid, message",
    [
        (np.zeros((0, 2), dtype=np.uint8), "non-empty"),
        (np.array([[3]], dtype=np.uint8), "invalid"),
    ],
)
def test_extractor_rejects_invalid_grid(grid, message):
    with pytest.raises(ValueError, match=message):
        _extract(grid)
