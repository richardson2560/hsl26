# hsl_core/hsl_core/perception/topological_belief.py
"""Version-bound opponent belief with finite-speed graph propagation."""

from __future__ import annotations

from dataclasses import dataclass, replace
import heapq
import math
from typing import Iterable

import numpy as np

from ..topology import EdgeState, NodeKind, TopologyEdge, TopologyGraph


_MASS_TOLERANCE = 1e-9


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class BeliefCell:
    """Probability support on a metric interval of one versioned graph edge."""

    edge_id: int
    s_begin_m: float
    s_end_m: float
    lateral_bound_m: float
    mass: float

    def __post_init__(self) -> None:
        for value, name in ((self.edge_id, "edge_id"),):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        begin = _finite(self.s_begin_m, "s_begin_m")
        end = _finite(self.s_end_m, "s_end_m")
        lateral = _finite(self.lateral_bound_m, "lateral_bound_m")
        mass = _finite(self.mass, "mass")
        if begin < 0.0 or end < begin:
            raise ValueError("belief interval must satisfy 0 <= begin <= end")
        if lateral < 0.0:
            raise ValueError("lateral_bound_m must be non-negative")
        if mass < 0.0:
            raise ValueError("belief mass must be non-negative")


@dataclass(frozen=True)
class CoverageInterval:
    """Fresh, valid coverage and calibrated detection probability on an edge."""

    edge_id: int
    s_begin_m: float
    s_end_m: float
    detection_probability: float

    def __post_init__(self) -> None:
        if isinstance(self.edge_id, bool) or not isinstance(self.edge_id, int) or self.edge_id < 0:
            raise ValueError("edge_id must be a non-negative integer")
        begin = _finite(self.s_begin_m, "s_begin_m")
        end = _finite(self.s_end_m, "s_end_m")
        probability = _finite(self.detection_probability, "detection_probability")
        if begin < 0.0 or end < begin:
            raise ValueError("coverage interval must satisfy 0 <= begin <= end")
        if not 0.0 <= probability <= 1.0:
            raise ValueError("detection_probability must be in [0, 1]")


@dataclass(frozen=True)
class NegativeObservation:
    stamp_s: float
    observation_group_id: str
    map_version: int
    topology_version: int
    localization_epoch: str
    coverage: tuple[CoverageInterval, ...]
    valid: bool = True

    def __post_init__(self) -> None:
        if _finite(self.stamp_s, "stamp_s") < 0.0:
            raise ValueError("stamp_s must be non-negative")
        if not self.observation_group_id or not self.localization_epoch:
            raise ValueError("observation_group_id and localization_epoch must not be empty")
        if self.map_version < 0 or self.topology_version < 0:
            raise ValueError("observation versions must be non-negative")
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be boolean")
        coverage = tuple(self.coverage)
        if any(not isinstance(item, CoverageInterval) for item in coverage):
            raise ValueError("coverage must contain CoverageInterval values")
        object.__setattr__(self, "coverage", coverage)


@dataclass(frozen=True)
class BeliefUpdateResult:
    accepted: bool
    reason: str
    belief: "TopologicalBelief"


@dataclass(frozen=True)
class TopologicalBelief:
    """Normalized edge-interval belief tied to one immutable topology snapshot.

    Propagation uses an uninformative uniform arc-length measure over the
    reachable support. Unknown mass is kept separate and never silently
    redistributed onto known graph edges.
    """

    graph: TopologyGraph
    cells: tuple[BeliefCell, ...]
    unknown_mass: float
    last_measurement_stamp_s: float
    current_stamp_s: float
    max_speed_mps: float
    belief_origin_stamp_s: float | None = None
    initial_speed_bound_mps: float | None = None
    acceleration_bound_mps2: float | None = None
    unknown_frontier_branch_probability: float = 0.5
    last_negative_stamp_s: float | None = None
    observation_group_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        cells = tuple(self.cells)
        if any(not isinstance(cell, BeliefCell) for cell in cells):
            raise ValueError("cells must contain BeliefCell values")
        object.__setattr__(self, "cells", cells)
        group_ids = tuple(self.observation_group_ids)
        if any(not isinstance(group_id, str) or not group_id for group_id in group_ids):
            raise ValueError("observation_group_ids must be non-empty strings")
        object.__setattr__(self, "observation_group_ids", group_ids)
        unknown = _finite(self.unknown_mass, "unknown_mass")
        measurement_stamp = _finite(self.last_measurement_stamp_s, "last_measurement_stamp_s")
        current_stamp = _finite(self.current_stamp_s, "current_stamp_s")
        max_speed = _finite(self.max_speed_mps, "max_speed_mps")
        if not 0.0 <= unknown <= 1.0:
            raise ValueError("unknown_mass must be in [0, 1]")
        if current_stamp < measurement_stamp:
            raise ValueError("current_stamp_s must not precede last_measurement_stamp_s")
        if measurement_stamp < 0.0:
            raise ValueError("last_measurement_stamp_s must be non-negative")
        if max_speed <= 0.0:
            raise ValueError("max_speed_mps must be positive")
        frontier_probability = _finite(
            self.unknown_frontier_branch_probability,
            "unknown_frontier_branch_probability",
        )
        if not 0.0 <= frontier_probability < 1.0:
            raise ValueError("unknown_frontier_branch_probability must be in [0, 1)")
        edge_by_id = {edge.edge_id: edge for edge in self.graph.edges}
        for cell in self.cells:
            edge = edge_by_id.get(cell.edge_id)
            if edge is None:
                raise ValueError(f"belief references unknown edge {cell.edge_id}")
            if cell.s_end_m > edge.metric_length_m + 1e-9:
                raise ValueError("belief interval exceeds edge length")
        total_mass = sum(cell.mass for cell in self.cells) + unknown
        if not math.isfinite(total_mass) or abs(total_mass - 1.0) > _MASS_TOLERANCE:
            raise ValueError("known cell mass plus unknown_mass must sum to one")
        origin = measurement_stamp if self.belief_origin_stamp_s is None else _finite(
            self.belief_origin_stamp_s, "belief_origin_stamp_s"
        )
        if origin > current_stamp:
            raise ValueError("belief_origin_stamp_s must not follow current_stamp_s")
        object.__setattr__(self, "belief_origin_stamp_s", origin)
        if self.initial_speed_bound_mps is not None:
            speed = _finite(self.initial_speed_bound_mps, "initial_speed_bound_mps")
            if speed < 0.0 or speed > max_speed:
                raise ValueError("initial_speed_bound_mps must be in [0, max_speed_mps]")
        if self.acceleration_bound_mps2 is not None:
            acceleration = _finite(self.acceleration_bound_mps2, "acceleration_bound_mps2")
            if acceleration <= 0.0 or self.initial_speed_bound_mps is None:
                raise ValueError("positive acceleration requires a bounded initial speed")
        if self.last_negative_stamp_s is not None:
            _finite(self.last_negative_stamp_s, "last_negative_stamp_s")
        if len(set(self.observation_group_ids)) != len(self.observation_group_ids):
            raise ValueError("observation group IDs must be unique")

    @property
    def map_version(self) -> int:
        return self.graph.map_version

    @property
    def topology_version(self) -> int:
        return self.graph.topology_version

    @property
    def localization_epoch(self) -> str:
        return self.graph.localization_epoch

    def propagate(self, stamp_s: float) -> "TopologicalBelief":
        """Expand support by a path-length bound; never teleport across edges."""
        target_stamp = _finite(stamp_s, "stamp_s")
        if target_stamp < self.current_stamp_s:
            raise ValueError("propagation stamp must be monotonic")
        if target_stamp == self.current_stamp_s or not self.cells:
            return replace(self, current_stamp_s=target_stamp)
        previous_age = self.current_stamp_s - self.belief_origin_stamp_s
        new_age = target_stamp - self.belief_origin_stamp_s
        distance_budget = self._max_path_length(new_age) - self._max_path_length(previous_age)
        distance_budget = max(0.0, distance_budget)
        if distance_budget <= 1e-15:
            return replace(self, current_stamp_s=target_stamp)

        edge_by_id = {edge.edge_id: edge for edge in self.graph.edges}
        new_cells: list[BeliefCell] = []
        added_unknown_mass = 0.0
        for source in self.cells:
            edge = edge_by_id[source.edge_id]
            if edge.state == EdgeState.BLOCKED:
                new_cells.append(source)
                continue
            reachable = self._reachable_intervals(source, distance_budget)
            measure = sum(end - begin for _, begin, end in reachable)
            if measure <= 1e-15:
                new_cells.append(source)
                continue
            source_mass = source.mass
            if (
                self.unknown_frontier_branch_probability > 0.0
                and self._frontier_reachable(source, distance_budget)
            ):
                unknown_branch = (
                    source.mass * self.unknown_frontier_branch_probability
                )
                added_unknown_mass += unknown_branch
                source_mass -= unknown_branch
            for edge_id, begin, end in reachable:
                if end - begin <= 1e-15:
                    continue
                mass = source_mass * (end - begin) / measure
                if mass > 0.0:
                    new_cells.append(
                        BeliefCell(edge_id, begin, end, source.lateral_bound_m, mass)
                    )
        # Degenerate point support and blocked-only graphs retain their mass.
        distributed = sum(cell.mass for cell in new_cells)
        expected_known = sum(cell.mass for cell in self.cells) - added_unknown_mass
        if expected_known - distributed > _MASS_TOLERANCE:
            raise ArithmeticError("belief propagation lost known probability mass")
        if distributed > expected_known + _MASS_TOLERANCE:
            raise ArithmeticError("belief propagation created probability mass")
        if expected_known > 0.0 and distributed < expected_known:
            scale = expected_known / distributed if distributed > 0.0 else 0.0
            if scale == 0.0:
                new_cells = list(self.cells)
            else:
                new_cells = [
                    replace(cell, mass=cell.mass * scale) for cell in new_cells
                ]
        return replace(
            self,
            cells=tuple(new_cells),
            unknown_mass=self.unknown_mass + added_unknown_mass,
            current_stamp_s=target_stamp,
        )

    def apply_negative_observation(
        self,
        observation: NegativeObservation,
        *,
        now_s: float,
        max_age_s: float,
        unknown_likelihood: float = 1.0,
    ) -> BeliefUpdateResult:
        """Apply one fresh independent scan using the specified Bayes update."""
        now = _finite(now_s, "now_s")
        max_age = _finite(max_age_s, "max_age_s")
        unknown_like = _finite(unknown_likelihood, "unknown_likelihood")
        if max_age <= 0.0 or not 0.0 <= unknown_like <= 1.0:
            raise ValueError("invalid freshness or unknown likelihood")
        reason = self._observation_rejection(observation, now, max_age)
        if reason:
            return BeliefUpdateResult(False, reason, self)
        prior = (
            self.propagate(observation.stamp_s)
            if observation.stamp_s > self.current_stamp_s
            else self
        )
        likelihoods = []
        for cell in prior.cells:
            pd = self._average_detection_probability(cell, observation.coverage)
            likelihoods.append(1.0 - pd)
        weighted = [cell.mass * likelihood for cell, likelihood in zip(prior.cells, likelihoods)]
        unknown_weight = prior.unknown_mass * unknown_like
        denominator = sum(weighted) + unknown_weight
        group_ids = prior.observation_group_ids + (observation.observation_group_id,)
        if denominator <= np.finfo(float).tiny:
            # Contradictory perfect evidence must broaden uncertainty, not assert absence.
            broadened = replace(
                prior,
                cells=(),
                unknown_mass=1.0,
                last_negative_stamp_s=observation.stamp_s,
                observation_group_ids=group_ids,
            )
            return BeliefUpdateResult(False, "INCONSISTENT_EVIDENCE_BROADENED_TO_UNKNOWN", broadened)
        normalized = tuple(
            replace(cell, mass=weight / denominator)
            for cell, weight in zip(prior.cells, weighted)
            if weight > 0.0
        )
        posterior_unknown = unknown_weight / denominator
        updated = replace(
            prior,
            cells=normalized,
            unknown_mass=posterior_unknown,
            last_negative_stamp_s=observation.stamp_s,
            observation_group_ids=group_ids,
        )
        return BeliefUpdateResult(True, "UPDATED_COVERED_REGIONS", updated)

    def reacquire(
        self,
        position_xy_m: tuple[float, float],
        covariance_xy_m2: tuple[float, float, float, float],
        *,
        stamp_s: float,
        map_version: int,
        topology_version: int,
        localization_epoch: str,
        unknown_likelihood: float = 0.05,
    ) -> BeliefUpdateResult:
        """Reweight existing edge hypotheses from a position-only measurement."""
        position = np.asarray(position_xy_m, dtype=float)
        covariance = np.asarray(covariance_xy_m2, dtype=float)
        if position.shape != (2,) or not np.all(np.isfinite(position)):
            raise ValueError("position_xy_m must contain two finite values")
        if covariance.shape == (4,):
            covariance = covariance.reshape(2, 2)
        if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
            raise ValueError("covariance_xy_m2 must be a finite 2x2 matrix")
        covariance = 0.5 * (covariance + covariance.T)
        if np.min(np.linalg.eigvalsh(covariance)) <= 0.0:
            raise ValueError("covariance_xy_m2 must be positive definite")
        stamp = _finite(stamp_s, "stamp_s")
        unknown_like = _finite(unknown_likelihood, "unknown_likelihood")
        if not 0.0 < unknown_like <= 1.0:
            raise ValueError("unknown_likelihood must be in (0, 1]")
        if (
            map_version != self.map_version
            or topology_version != self.topology_version
            or localization_epoch != self.localization_epoch
        ):
            return BeliefUpdateResult(False, "VERSION_MISMATCH", self)
        if stamp < self.current_stamp_s or stamp <= self.last_measurement_stamp_s:
            return BeliefUpdateResult(False, "STALE_REACQUISITION", self)
        propagated = self.propagate(stamp)
        edge_by_id = {edge.edge_id: edge for edge in self.graph.edges}
        weighted_cells = []
        for cell in propagated.cells:
            edge = edge_by_id[cell.edge_id]
            likelihood = self._cell_position_likelihood(
                edge, cell, position, covariance
            )
            weighted_cells.append(cell.mass * likelihood)
        unknown_weight = propagated.unknown_mass * unknown_like
        denominator = sum(weighted_cells) + unknown_weight
        if denominator <= np.finfo(float).tiny:
            broadened = replace(
                propagated,
                cells=(),
                unknown_mass=1.0,
                last_measurement_stamp_s=stamp,
                belief_origin_stamp_s=stamp,
                current_stamp_s=stamp,
                initial_speed_bound_mps=self.max_speed_mps,
                acceleration_bound_mps2=None,
            )
            return BeliefUpdateResult(False, "REACQUISITION_INCONSISTENT_BROADENED", broadened)
        updated_cells = tuple(
            replace(cell, mass=weight / denominator)
            for cell, weight in zip(propagated.cells, weighted_cells)
            if weight > 0.0
        )
        updated = replace(
            propagated,
            cells=updated_cells,
            unknown_mass=unknown_weight / denominator,
            last_measurement_stamp_s=stamp,
            current_stamp_s=stamp,
            belief_origin_stamp_s=stamp,
            initial_speed_bound_mps=self.max_speed_mps,
            acceleration_bound_mps2=None,
        )
        return BeliefUpdateResult(True, "REACQUIRED", updated)

    def _max_path_length(self, elapsed_s: float) -> float:
        if self.acceleration_bound_mps2 is None:
            return self.max_speed_mps * elapsed_s
        speed0 = self.initial_speed_bound_mps
        acceleration = self.acceleration_bound_mps2
        assert speed0 is not None and acceleration is not None
        acceleration_time = (self.max_speed_mps - speed0) / acceleration
        if elapsed_s <= acceleration_time:
            return speed0 * elapsed_s + 0.5 * acceleration * elapsed_s**2
        accelerated_distance = speed0 * acceleration_time + 0.5 * acceleration * acceleration_time**2
        return accelerated_distance + self.max_speed_mps * (elapsed_s - acceleration_time)

    def _reachable_intervals(
        self, source: BeliefCell, distance_budget: float
    ) -> list[tuple[int, float, float]]:
        edge_by_id = {edge.edge_id: edge for edge in self.graph.edges}
        source_edge = edge_by_id[source.edge_id]
        source_length = source_edge.metric_length_m
        initial_costs = {
            source_edge.from_node: source.s_begin_m,
            source_edge.to_node: source_length - source.s_end_m,
        }
        node_distance = self._node_distances(initial_costs)
        result = []
        for edge in self.graph.edges:
            if edge.state == EdgeState.BLOCKED:
                continue
            length = edge.metric_length_m
            intervals = []
            if edge.edge_id == source.edge_id:
                intervals.append(
                    (
                        max(0.0, source.s_begin_m - distance_budget),
                        min(length, source.s_end_m + distance_budget),
                    )
                )
            from_distance = node_distance.get(edge.from_node, math.inf)
            to_distance = node_distance.get(edge.to_node, math.inf)
            if from_distance <= distance_budget:
                intervals.append((0.0, min(length, distance_budget - from_distance)))
            if to_distance <= distance_budget:
                intervals.append((max(0.0, length - (distance_budget - to_distance)), length))
            for begin, end in self._union_intervals(intervals):
                if end >= begin:
                    result.append((edge.edge_id, begin, end))
        return result

    def _frontier_reachable(
        self, source: BeliefCell, distance_budget: float
    ) -> bool:
        edge_by_id = {edge.edge_id: edge for edge in self.graph.edges}
        source_edge = edge_by_id[source.edge_id]
        initial_costs = {
            source_edge.from_node: source.s_begin_m,
            source_edge.to_node: source_edge.metric_length_m - source.s_end_m,
        }
        distances = self._node_distances(initial_costs)
        return any(
            node.kind == NodeKind.FRONTIER
            and distances.get(node.node_id, math.inf) <= distance_budget
            for node in self.graph.nodes
        )

    def _node_distances(self, initial_costs: dict[int, float]) -> dict[int, float]:
        adjacency: dict[int, list[tuple[int, float]]] = {
            node.node_id: [] for node in self.graph.nodes
        }
        for edge in self.graph.edges:
            if edge.state == EdgeState.BLOCKED:
                continue
            length = edge.metric_length_m
            adjacency[edge.from_node].append((edge.to_node, length))
            adjacency[edge.to_node].append((edge.from_node, length))
        distances = {node_id: math.inf for node_id in adjacency}
        queue = []
        for node_id, cost in initial_costs.items():
            if cost < distances[node_id]:
                distances[node_id] = cost
                heapq.heappush(queue, (cost, node_id))
        while queue:
            distance, node_id = heapq.heappop(queue)
            if distance != distances[node_id]:
                continue
            for neighbour, length in adjacency[node_id]:
                candidate = distance + length
                if candidate < distances[neighbour]:
                    distances[neighbour] = candidate
                    heapq.heappush(queue, (candidate, neighbour))
        return distances

    @staticmethod
    def _union_intervals(
        intervals: Iterable[tuple[float, float]],
    ) -> list[tuple[float, float]]:
        valid = sorted((max(0.0, a), b) for a, b in intervals if b >= a)
        merged: list[list[float]] = []
        for begin, end in valid:
            if not merged or begin > merged[-1][1] + 1e-12:
                merged.append([begin, end])
            else:
                merged[-1][1] = max(merged[-1][1], end)
        return [(begin, end) for begin, end in merged]

    def _observation_rejection(
        self, observation: NegativeObservation, now_s: float, max_age_s: float
    ) -> str:
        if not observation.valid:
            return "INVALID_OBSERVATION"
        if (
            observation.map_version != self.map_version
            or observation.topology_version != self.topology_version
            or observation.localization_epoch != self.localization_epoch
        ):
            return "VERSION_MISMATCH"
        if observation.observation_group_id in self.observation_group_ids:
            return "DUPLICATE_OBSERVATION_GROUP"
        if observation.stamp_s > now_s or now_s - observation.stamp_s > max_age_s:
            return "STALE_OBSERVATION"
        if self.last_negative_stamp_s is not None and observation.stamp_s <= self.last_negative_stamp_s:
            return "OUT_OF_ORDER_OBSERVATION"
        if observation.stamp_s < self.current_stamp_s:
            return "OBSERVATION_BEFORE_BELIEF_TIME"
        edge_lengths = {edge.edge_id: edge.metric_length_m for edge in self.graph.edges}
        for item in observation.coverage:
            length = edge_lengths.get(item.edge_id)
            if length is None or item.s_end_m > length + 1e-9:
                return "INVALID_COVERAGE_INTERVAL"
        return ""

    @staticmethod
    def _average_detection_probability(
        cell: BeliefCell, coverage: tuple[CoverageInterval, ...]
    ) -> float:
        applicable = [item for item in coverage if item.edge_id == cell.edge_id]
        if not applicable:
            return 0.0
        width = cell.s_end_m - cell.s_begin_m
        if width <= 1e-12:
            return max(
                (
                    item.detection_probability
                    for item in applicable
                    if item.s_begin_m - 1e-12 <= cell.s_begin_m <= item.s_end_m + 1e-12
                ),
                default=0.0,
            )
        breakpoints = {cell.s_begin_m, cell.s_end_m}
        for item in applicable:
            if cell.s_begin_m < item.s_begin_m < cell.s_end_m:
                breakpoints.add(item.s_begin_m)
            if cell.s_begin_m < item.s_end_m < cell.s_end_m:
                breakpoints.add(item.s_end_m)
        ordered = sorted(breakpoints)
        integral = 0.0
        for begin, end in zip(ordered, ordered[1:]):
            midpoint = 0.5 * (begin + end)
            probability = max(
                (
                    item.detection_probability
                    for item in applicable
                    if item.s_begin_m <= midpoint <= item.s_end_m
                ),
                default=0.0,
            )
            integral += probability * (end - begin)
        return min(1.0, max(0.0, integral / width))

    @staticmethod
    def _cell_position_likelihood(
        edge: TopologyEdge,
        cell: BeliefCell,
        measurement: np.ndarray,
        covariance: np.ndarray,
    ) -> float:
        points = np.asarray(edge.polyline_xy_m, dtype=float)
        cumulative = [0.0]
        for first, second in zip(points, points[1:]):
            cumulative.append(cumulative[-1] + float(np.linalg.norm(second - first)))
        positions = (cell.s_begin_m, 0.5 * (cell.s_begin_m + cell.s_end_m), cell.s_end_m)
        likelihoods = []
        for arc in positions:
            for index, (start, end) in enumerate(zip(cumulative, cumulative[1:])):
                if arc <= end or index == len(cumulative) - 2:
                    fraction = 0.0 if end == start else (arc - start) / (end - start)
                    point = points[index] + fraction * (points[index + 1] - points[index])
                    delta = point - measurement
                    whitened = np.linalg.solve(covariance, delta)
                    likelihoods.append(math.exp(-0.5 * float(delta @ whitened)))
                    break
        return sum(likelihoods) / len(likelihoods)
