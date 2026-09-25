# hsl_core/hsl_core/planning/astar.py
"""Deterministic metric A*/Dijkstra over the versioned topology multigraph."""

from dataclasses import dataclass
import heapq
import math
from typing import Callable

from ..topology import EdgeState, TopologyEdge, TopologyGraph


@dataclass(frozen=True)
class PlannedPath:
    map_version: int
    topology_version: int
    localization_epoch: str
    start_node_id: int
    goal_node_id: int
    node_ids: tuple[int, ...]
    edge_ids: tuple[int, ...]
    polyline_xy_m: tuple[tuple[float, float], ...]
    cost: float

    def __post_init__(self) -> None:
        if not self.node_ids or self.node_ids[0] != self.start_node_id:
            raise ValueError("path must start at start_node_id")
        if self.node_ids[-1] != self.goal_node_id:
            raise ValueError("path must end at goal_node_id")
        if len(self.edge_ids) != len(self.node_ids) - 1:
            raise ValueError("path edge/node cardinality mismatch")
        if len(self.polyline_xy_m) < 2 or not math.isfinite(self.cost) or self.cost < 0.0:
            raise ValueError("path geometry or cost is invalid")


def _edge_cost(edge: TopologyEdge) -> float:
    cost = edge.cost if edge.cost > 0.0 else edge.metric_length_m
    if not math.isfinite(cost) or cost < edge.metric_length_m - 1e-9:
        raise ValueError("edge cost must dominate metric edge length")
    return cost


def _heuristic(graph: TopologyGraph, node_id: int, goal_id: int) -> float:
    nodes = {node.node_id: node for node in graph.nodes}
    a, b = nodes[node_id], nodes[goal_id]
    return math.hypot(a.x_m - b.x_m, a.y_m - b.y_m)


def _search(graph: TopologyGraph, start: int, goal: int, use_heuristic: bool) -> PlannedPath:
    node_ids = {node.node_id for node in graph.nodes}
    if start not in node_ids or goal not in node_ids:
        raise ValueError("start and goal must reference graph nodes")
    adjacency = graph.adjacency()
    queue: list[tuple[float, float, int]] = [(0.0, 0.0, start)]
    best = {start: 0.0}
    previous: dict[int, tuple[int, TopologyEdge]] = {}
    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current_cost > best.get(current, math.inf) + 1e-12:
            continue
        if current == goal:
            break
        for edge in sorted(adjacency[current], key=lambda item: (item.edge_id, item.to_node)):
            if edge.state == EdgeState.BLOCKED:
                continue
            step = _edge_cost(edge)
            candidate = current_cost + step
            if candidate + 1e-12 < best.get(edge.to_node, math.inf):
                best[edge.to_node] = candidate
                previous[edge.to_node] = (current, edge)
                priority = candidate + (_heuristic(graph, edge.to_node, goal) if use_heuristic else 0.0)
                heapq.heappush(queue, (priority, candidate, edge.to_node))
    if goal not in best:
        raise ValueError("goal is unreachable in the open structural graph")

    reversed_nodes = [goal]
    reversed_edges: list[TopologyEdge] = []
    current = goal
    while current != start:
        parent, edge = previous[current]
        reversed_nodes.append(parent)
        reversed_edges.append(edge)
        current = parent
    nodes = tuple(reversed(reversed_nodes))
    edges = tuple(reversed(reversed_edges))
    polyline: list[tuple[float, float]] = []
    for index, edge in enumerate(edges):
        points = edge.polyline_xy_m
        if index and polyline and math.isclose(polyline[-1][0], points[0][0], abs_tol=1e-9) and math.isclose(polyline[-1][1], points[0][1], abs_tol=1e-9):
            polyline.extend(points[1:])
        else:
            polyline.extend(points)
    return PlannedPath(
        graph.map_version,
        graph.topology_version,
        graph.localization_epoch,
        start,
        goal,
        nodes,
        tuple(edge.edge_id for edge in edges),
        tuple(polyline),
        best[goal],
    )


def astar(graph: TopologyGraph, start_node_id: int, goal_node_id: int) -> PlannedPath:
    """Find a least-cost path with an admissible Euclidean heuristic."""
    return _search(graph, start_node_id, goal_node_id, True)


def dijkstra(graph: TopologyGraph, start_node_id: int, goal_node_id: int) -> PlannedPath:
    """Reference least-cost path used for A* validation."""
    return _search(graph, start_node_id, goal_node_id, False)


def smooth_polyline(
    path: PlannedPath,
    *,
    segment_is_valid: Callable[[tuple[float, float], tuple[float, float]], bool],
) -> PlannedPath:
    """Shortcut only segments explicitly accepted by a swept-footprint validator."""
    if not callable(segment_is_valid):
        raise ValueError("segment_is_valid must be callable")
    points = list(path.polyline_xy_m)
    if len(points) < 2:
        raise ValueError("path polyline is too short")
    result = [points[0]]
    anchor = 0
    while anchor < len(points) - 1:
        selected = anchor + 1
        for candidate in range(len(points) - 1, anchor, -1):
            if segment_is_valid(points[anchor], points[candidate]):
                selected = candidate
                break
        result.append(points[selected])
        anchor = selected
    length = sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(result, result[1:])
    )
    return PlannedPath(
        path.map_version,
        path.topology_version,
        path.localization_epoch,
        path.start_node_id,
        path.goal_node_id,
        path.node_ids,
        path.edge_ids,
        tuple(result),
        length,
    )
