# hsl_core/hsl_core/topology.py
"""ROS-independent contracts for the Phase-3 embedded world graph.

This module intentionally contains contracts and deterministic validation only.
Grid extraction, persistence and ROS conversion remain Phase-3 work packages.
"""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Mapping, Optional, Tuple

import numpy as np

from .mapping import OCCUPIED, OBSERVED_FREE, UNKNOWN


class NodeKind(IntEnum):
    """Serialized node categories from the technical specification."""

    ANCHOR = 0
    JUNCTION = 1
    DEAD_END = 2
    PORTAL = 3
    FRONTIER = 4
    ZONE_ACCESS = 5


TopologyNodeKind = NodeKind


class EdgeState(IntEnum):
    """Runtime traversal overlay; structural identity is retained when blocked."""

    UNKNOWN = 0
    OPEN = 1
    BLOCKED = 2


def _finite(value: float, name: str) -> float:
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _identifier(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


@dataclass(frozen=True)
class TopologyNode:
    """A stable structural graph node embedded in the local world frame."""

    node_id: int
    x_m: float
    y_m: float
    kind: NodeKind
    clearance_radius_m: float
    articulation: bool = False
    semantic_zone_id: str = ""

    def __post_init__(self) -> None:
        _identifier(self.node_id, "node_id")
        _finite(self.x_m, "x_m")
        _finite(self.y_m, "y_m")
        if not math.isfinite(self.clearance_radius_m) or self.clearance_radius_m < 0.0:
            raise ValueError("clearance_radius_m must be finite and non-negative")
        object.__setattr__(self, "kind", NodeKind(self.kind))


@dataclass(frozen=True)
class TopologyEdge:
    """A directed view of a structural corridor with an embedded polyline."""

    edge_id: int
    from_node: int
    to_node: int
    polyline_xy_m: Tuple[Tuple[float, float], ...]
    min_clearance_radius_m: float
    min_width_m: float = 0.0
    width_valid: bool = False
    structural_bridge: bool = False
    state: EdgeState = EdgeState.UNKNOWN
    cost: float = 0.0

    def __post_init__(self) -> None:
        _identifier(self.edge_id, "edge_id")
        _identifier(self.from_node, "from_node")
        _identifier(self.to_node, "to_node")
        if len(self.polyline_xy_m) < 2:
            raise ValueError("edge polyline must contain at least two points")
        points = tuple(
            (_finite(point[0], "polyline x"), _finite(point[1], "polyline y"))
            for point in self.polyline_xy_m
        )
        if not math.isfinite(self.min_clearance_radius_m) or self.min_clearance_radius_m < 0.0:
            raise ValueError("min_clearance_radius_m must be finite and non-negative")
        if not math.isfinite(self.min_width_m) or self.min_width_m < 0.0:
            raise ValueError("min_width_m must be finite and non-negative")
        if not math.isfinite(self.cost) or self.cost < 0.0:
            raise ValueError("cost must be finite and non-negative")
        object.__setattr__(self, "state", EdgeState(self.state))
        object.__setattr__(self, "polyline_xy_m", points)

    @property
    def metric_length_m(self) -> float:
        return float(sum(
            math.hypot(current[0] - previous[0], current[1] - previous[1])
            for previous, current in zip(self.polyline_xy_m, self.polyline_xy_m[1:])
        ))


def _reversed_edge(edge: TopologyEdge) -> TopologyEdge:
    """Return a traversal view without changing the canonical edge ID."""
    return TopologyEdge(
        edge_id=edge.edge_id,
        from_node=edge.to_node,
        to_node=edge.from_node,
        polyline_xy_m=edge.polyline_xy_m[::-1],
        min_clearance_radius_m=edge.min_clearance_radius_m,
        min_width_m=edge.min_width_m,
        width_valid=edge.width_valid,
        structural_bridge=edge.structural_bridge,
        state=edge.state,
        cost=edge.cost,
    )


@dataclass(frozen=True)
class TopologyGraph:
    """Versioned multigraph snapshot with derived undirected adjacency."""

    map_version: int
    topology_version: int
    localization_epoch: str
    nodes: Tuple[TopologyNode, ...]
    edges: Tuple[TopologyEdge, ...]

    def __post_init__(self) -> None:
        if self.map_version < 0 or self.topology_version < 0:
            raise ValueError("graph versions must be non-negative")
        if not self.localization_epoch:
            raise ValueError("localization_epoch must not be empty")
        node_ids = [node.node_id for node in self.nodes]
        edge_ids = [edge.edge_id for edge in self.edges]
        if len(node_ids) != len(set(node_ids)):
            raise ValueError("node IDs must be unique")
        if len(edge_ids) != len(set(edge_ids)):
            raise ValueError("edge IDs must be unique")
        node_set = set(node_ids)
        if any(edge.from_node not in node_set or edge.to_node not in node_set for edge in self.edges):
            raise ValueError("edges must reference existing nodes")

    def adjacency(self) -> Mapping[int, Tuple[TopologyEdge, ...]]:
        result = {node.node_id: [] for node in self.nodes}
        for edge in self.edges:
            result[edge.from_node].append(edge)
            result[edge.to_node].append(_reversed_edge(edge))
        return {node_id: tuple(edges) for node_id, edges in result.items()}


@dataclass(frozen=True)
class TopologyBuildConfig:
    """Explicit profile parameters for deterministic grid-to-graph extraction."""

    footprint_radius_m: float
    grid_error_margin_m: float = 0.0
    portal_clearance_m: float = 0.0
    max_cells: int = 250_000

    def __post_init__(self) -> None:
        for value, name in (
            (self.footprint_radius_m, "footprint_radius_m"),
            (self.grid_error_margin_m, "grid_error_margin_m"),
            (self.portal_clearance_m, "portal_clearance_m"),
        ):
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if isinstance(self.max_cells, bool) or not isinstance(self.max_cells, int) or self.max_cells <= 0:
            raise ValueError("max_cells must be a positive integer")


@dataclass(frozen=True)
class TopologyExtraction:
    """Graph plus the exact derived masks used to construct it."""

    graph: TopologyGraph
    known_free: np.ndarray
    configuration_free: np.ndarray
    clearance_m: np.ndarray

    def __post_init__(self) -> None:
        for name in ("known_free", "configuration_free", "clearance_m"):
            value = np.asarray(getattr(self, name))
            if value.ndim != 2 or not np.isfinite(value).all():
                raise ValueError(f"{name} must be a finite two-dimensional array")
            object.__setattr__(self, name, value.copy())
        if self.known_free.shape != self.configuration_free.shape or self.known_free.shape != self.clearance_m.shape:
            raise ValueError("topology masks must have equal shapes")


_NEIGHBOR_STEPS = (
    (-1, -1), (0, -1), (1, -1),
    (-1, 0), (1, 0),
    (-1, 1), (0, 1), (1, 1),
)
_ORTHOGONAL_STEPS = ((-1, 0), (1, 0), (0, -1), (0, 1))


def _valid_neighbors(mask: np.ndarray, cell: Tuple[int, int]) -> list[Tuple[int, int]]:
    """Return 8-neighbors, rejecting diagonal corner cutting."""
    x, y = cell
    height, width = mask.shape
    result: list[Tuple[int, int]] = []
    for dx, dy in _NEIGHBOR_STEPS:
        nx, ny = x + dx, y + dy
        if not (0 <= nx < width and 0 <= ny < height) or not mask[ny, nx]:
            continue
        if dx and dy and (not mask[y, nx] or not mask[ny, x]):
            continue
        result.append((nx, ny))
    return result


def _orthogonal_neighbors(mask: np.ndarray, cell: Tuple[int, int]) -> list[Tuple[int, int]]:
    x, y = cell
    height, width = mask.shape
    return [
        (x + dx, y + dy)
        for dx, dy in _ORTHOGONAL_STEPS
        if 0 <= x + dx < width
        and 0 <= y + dy < height
        and mask[y + dy, x + dx]
    ]


def _inflate_blocked(
    blocking: np.ndarray,
    resolution_m: float,
    inflation_radius_m: float,
) -> np.ndarray:
    """Inflate blocking cells conservatively by a circular grid footprint."""
    radius_cells = int(math.ceil(inflation_radius_m / resolution_m))
    if radius_cells == 0:
        return blocking.copy()
    height, width = blocking.shape
    inflated = blocking.copy()
    offsets = [
        (dx, dy)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if math.hypot(dx, dy) * resolution_m <= inflation_radius_m + 1e-12
    ]
    for y, x in zip(*np.nonzero(blocking)):
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                inflated[ny, nx] = True
    return inflated


def _clearance_transform(
    free: np.ndarray,
    resolution_m: float,
) -> np.ndarray:
    """Compute conservative center-to-blocking clearance by exhaustive distance."""
    blocking = ~free
    # The finite grid boundary is not traversable beyond its declared extent.
    blocking[0, :] = True
    blocking[-1, :] = True
    blocking[:, 0] = True
    blocking[:, -1] = True
    blocked_points = np.argwhere(blocking)
    clearance = np.zeros(free.shape, dtype=float)
    if blocked_points.size == 0:
        clearance[free] = math.inf
        return clearance
    for y, x in zip(*np.nonzero(free)):
        distance_cells = np.sqrt(
            np.square(blocked_points[:, 0] - y) + np.square(blocked_points[:, 1] - x)
        )
        clearance[y, x] = float(np.min(distance_cells)) * resolution_m
    return clearance


class TopologyBuilder:
    """Extract a deterministic embedded multigraph from a layered grid."""

    def __init__(self, config: TopologyBuildConfig) -> None:
        self.config = config

    def extract(
        self,
        cells: np.ndarray,
        *,
        origin_xy_m: Tuple[float, float],
        resolution_m: float,
        map_version: int,
        localization_epoch: str,
        previous_graph: Optional[TopologyGraph] = None,
    ) -> TopologyExtraction:
        grid = np.asarray(cells)
        if grid.ndim != 2 or grid.size == 0 or grid.size > self.config.max_cells:
            raise ValueError("cells must be a non-empty bounded two-dimensional grid")
        if not np.issubdtype(grid.dtype, np.integer) or not np.isin(
            grid, (UNKNOWN, OBSERVED_FREE, OCCUPIED)
        ).all():
            raise ValueError("cells contain an invalid coverage state")
        if len(origin_xy_m) != 2 or not all(math.isfinite(float(value)) for value in origin_xy_m):
            raise ValueError("origin_xy_m must contain two finite values")
        if not math.isfinite(float(resolution_m)) or resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if isinstance(map_version, bool) or not isinstance(map_version, int) or map_version < 0:
            raise ValueError("map_version must be a non-negative integer")
        if not localization_epoch:
            raise ValueError("localization_epoch must not be empty")

        known_free = grid == OBSERVED_FREE
        blocking = (grid == UNKNOWN) | (grid == OCCUPIED)
        inflation = self.config.footprint_radius_m + self.config.grid_error_margin_m
        inflated_blocking = _inflate_blocked(blocking, resolution_m, inflation)
        configuration_free = known_free & ~inflated_blocking
        clearance = _clearance_transform(known_free, resolution_m)
        clearance[~configuration_free] = 0.0
        nodes, node_cells = self._extract_nodes(
            configuration_free,
            known_free,
            grid == UNKNOWN,
            clearance,
            origin_xy_m,
            resolution_m,
        )
        edges = self._trace_edges(configuration_free, node_cells, clearance, origin_xy_m, resolution_m)
        topology_version = 1 if previous_graph is None else previous_graph.topology_version + 1
        graph = TopologyGraph(
            map_version,
            topology_version,
            localization_epoch,
            tuple(nodes),
            tuple(edges),
        )
        validate_graph(graph)
        return TopologyExtraction(graph, known_free, configuration_free, clearance)

    def _extract_nodes(
        self,
        free: np.ndarray,
        known_free: np.ndarray,
        unknown: np.ndarray,
        clearance: np.ndarray,
        origin_xy_m: Tuple[float, float],
        resolution_m: float,
    ) -> Tuple[list[TopologyNode], set[Tuple[int, int]]]:
        candidates: dict[Tuple[int, int], NodeKind] = {}
        for y, x in zip(*np.nonzero(free)):
            cell = (int(x), int(y))
            # Degree events use the fixed 4-connected skeleton convention.
            # Traversal still permits safe diagonals, so corner cutting is
            # checked separately by _valid_neighbors.
            neighbors = _orthogonal_neighbors(free, cell)
            frontier = any(
                0 <= cell[0] + dx < free.shape[1]
                and 0 <= cell[1] + dy < free.shape[0]
                and unknown[cell[1] + dy, cell[0] + dx]
                for dx, dy in _NEIGHBOR_STEPS
            )
            if frontier:
                candidates[cell] = NodeKind.FRONTIER
            elif len(neighbors) == 1:
                candidates[cell] = NodeKind.DEAD_END
            elif len(neighbors) >= 3:
                candidates[cell] = NodeKind.JUNCTION

        if not candidates:
            free_cells = [(int(x), int(y)) for y, x in zip(*np.nonzero(free))]
            if free_cells:
                anchor = min(free_cells, key=lambda cell: (cell[1], cell[0]))
                candidates[anchor] = NodeKind.ANCHOR

        nodes: list[TopologyNode] = []
        node_cells = set(candidates)
        for node_id, cell in enumerate(sorted(node_cells, key=lambda value: (value[1], value[0])), start=1):
            x, y = cell
            nodes.append(
                TopologyNode(
                    node_id,
                    origin_xy_m[0] + (x + 0.5) * resolution_m,
                    origin_xy_m[1] + (y + 0.5) * resolution_m,
                    candidates[cell],
                    float(clearance[y, x]),
                )
            )
        return nodes, node_cells

    def _trace_edges(
        self,
        free: np.ndarray,
        node_cells: set[Tuple[int, int]],
        clearance: np.ndarray,
        origin_xy_m: Tuple[float, float],
        resolution_m: float,
    ) -> list[TopologyEdge]:
        cell_to_node = {
            cell: node_id
            for node_id, cell in enumerate(
                sorted(node_cells, key=lambda value: (value[1], value[0])), start=1
            )
        }
        visited: set[Tuple[Tuple[int, int], Tuple[int, int]]] = set()
        edges: list[TopologyEdge] = []
        edge_id = 1
        for start in sorted(node_cells, key=lambda value: (value[1], value[0])):
            for neighbor in _valid_neighbors(free, start):
                segment = tuple(sorted((start, neighbor)))
                if segment in visited:
                    continue
                path = [start, neighbor]
                visited.add(segment)
                previous, current = start, neighbor
                while current not in node_cells:
                    next_cells = [cell for cell in _valid_neighbors(free, current) if cell != previous]
                    if not next_cells:
                        break
                    next_cell = min(next_cells, key=lambda value: (value[1], value[0]))
                    segment = tuple(sorted((current, next_cell)))
                    if segment in visited:
                        break
                    visited.add(segment)
                    path.append(next_cell)
                    previous, current = current, next_cell
                if current not in cell_to_node or len(path) < 2:
                    continue
                polyline = tuple(
                    (
                        origin_xy_m[0] + (x + 0.5) * resolution_m,
                        origin_xy_m[1] + (y + 0.5) * resolution_m,
                    )
                    for x, y in path
                )
                min_clearance = min(float(clearance[y, x]) for x, y in path)
                length = sum(
                    math.hypot(polyline[index + 1][0] - polyline[index][0], polyline[index + 1][1] - polyline[index][1])
                    for index in range(len(polyline) - 1)
                )
                from_node = cell_to_node[start]
                to_node = cell_to_node[current]
                if from_node <= to_node:
                    canonical_from, canonical_to = from_node, to_node
                    canonical_polyline = polyline
                else:
                    canonical_from, canonical_to = to_node, from_node
                    canonical_polyline = polyline[::-1]
                edges.append(
                    TopologyEdge(
                        edge_id,
                        canonical_from,
                        canonical_to,
                        canonical_polyline,
                        min_clearance,
                        min_width_m=2.0 * min_clearance,
                        width_valid=math.isfinite(min_clearance),
                        state=EdgeState.OPEN,
                        cost=length,
                    )
                )
                edge_id += 1
        return edges


@dataclass(frozen=True)
class SpectralSignature:
    """Optional versioned local spectral descriptor."""

    topology_version: int
    center_node_id: int
    hops: int
    node_count: int
    component_count: int
    eigenvalues: Tuple[float | None, ...]
    affinity_definition: str
    valid: bool

    def __post_init__(self) -> None:
        if self.topology_version < 0 or self.hops < 0:
            raise ValueError("spectral versions and hops must be non-negative")
        if self.node_count < 0 or self.component_count < 0:
            raise ValueError("spectral counts must be non-negative")
        _identifier(self.center_node_id, "center_node_id")
        if not self.affinity_definition:
            raise ValueError("affinity_definition must not be empty")
        for value in self.eigenvalues:
            if value is not None:
                _finite(value, "eigenvalue")


def validate_graph(graph: TopologyGraph) -> None:
    """Validate graph invariants at an adapter boundary."""

    for edge in graph.edges:
        if edge.metric_length_m <= 0.0:
            raise ValueError(f"edge {edge.edge_id} must have positive length")
        if edge.min_clearance_radius_m < 0.0:
            raise ValueError(f"edge {edge.edge_id} has invalid clearance")
        if edge.from_node > edge.to_node:
            raise ValueError(f"edge {edge.edge_id} endpoints are not canonical")
