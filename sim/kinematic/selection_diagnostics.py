"""Observation-only selection diagnostics; no ranking or actuation authority."""

from dataclasses import dataclass, fields, replace
import math
from typing import Mapping

from hsl_core.match import StagePhase
from hsl_core.planning import PlannedPath
from hsl_core.tactics import OptionGoal, OptionKind, SelectionResult, TacticalSnapshot
from hsl_core.tactics.fsm import CandidateEvaluation
from hsl_core.topology import EdgeState, TopologyGraph


GEOMETRY_TOLERANCE_M = 1e-9
BEHAVIOR_SIGNATURE_SCHEMA = "hsl26.realized-behavior.v1"


@dataclass(frozen=True, kw_only=True)
class SelectionDiagnostics:
    """v2: cardinalities and per-invocation indicators have separate units.

    proposal_total includes HOLD_SAFE. proposal_feasible counts final applicable
    non-HOLD entries; proposal_choice counts distinct winning behaviors.
    None means unknown, including choice counters in incomplete diagnostics.
    """

    trace_schema: str = "hsl26.policy-cycle.v2"
    cycle_sequence: int = 0
    cycle_stamp_ns: int = 0
    phase: str = ""
    system_ready: bool | None = None
    motion_authorized: bool | None = None
    stage_lease_valid: bool | None = None
    safety_stop_active: bool | None = None
    selector_invoked: bool = False
    diagnostic_valid: bool = True
    diagnostic_error: str = ""
    proposal_total: int = 0
    proposal_feasible_raw: int = 0
    proposal_feasible: int = 0
    winning_cohort_size: int = 0
    winning_priority: int | None = None
    winning_urgency: int | None = None
    winning_cohort_keys: tuple[str, ...] = ()
    behavior_signatures: tuple[tuple[str, tuple], ...] = ()
    deduplicated_keys: tuple[str, ...] = ()
    rejected_candidates: tuple[tuple[str, str], ...] = ()
    candidate_features: tuple[tuple[str, tuple[tuple[str, float], ...]], ...] = ()
    candidate_evaluations: tuple[CandidateEvaluation, ...] = ()
    selected_stable_key: str = ""
    selected_option_instance_id: str = ""
    utility_profile_id: str = ""
    selection_changed: bool | None = None
    proposal_choice: int | None = 0
    n_active: int = 0
    n_available: int | None = 0
    n_choice: int | None = 0
    n_forced: int | None = 0
    n_empty: int | None = 0

    def __post_init__(self) -> None:
        if self.n_active not in (0, 1):
            raise ValueError("n_active must be a binary invocation indicator")
        if self.n_active != int(self.selector_invoked and self.phase == "ACTIVE"):
            raise ValueError("n_active does not match phase and selector invocation")
        if self.diagnostic_valid:
            values = (self.n_choice, self.n_available, self.n_forced, self.n_empty)
            if any(v not in (0, 1) for v in values):
                raise ValueError("selection indicators must be binary")
            if not 0 <= self.n_choice <= self.n_available <= self.n_active:
                raise ValueError("selection indicators are not ordered")
            if (self.n_forced != self.n_available - self.n_choice
                    or self.n_empty != self.n_active - self.n_available):
                raise ValueError("selection partition is inconsistent")
            if self.diagnostic_error:
                raise ValueError("valid diagnostics cannot carry an error")
            counts = (self.proposal_choice, self.winning_cohort_size,
                      self.proposal_feasible, self.proposal_feasible_raw, self.proposal_total)
            if any(not isinstance(c, int) or isinstance(c, bool) or c < 0 for c in counts):
                raise ValueError("proposal cardinalities must be non-negative integers")
            if tuple(sorted(counts)) != counts:
                raise ValueError("proposal cardinalities are not ordered")
            if self.n_available != int(self.n_active == 1 and self.proposal_feasible > 0):
                raise ValueError("availability does not match applicable cardinality")
            if self.n_choice != int(self.n_active == 1 and self.proposal_choice >= 2):
                raise ValueError("choice does not match behavioral cardinality")
        elif not self.diagnostic_error:
            raise ValueError("incomplete diagnostics require an explicit reason")

    def trace_fields(self) -> dict:
        # Use shallow extraction: signatures are immutable and need no copying.
        return {f.name: getattr(self, f.name) for f in fields(SelectionDiagnostics)}


def begin_selection(base: SelectionDiagnostics, snapshot: TacticalSnapshot) -> SelectionDiagnostics:
    """Mark an invocation before calling the selector, without assuming success."""
    return replace(
        base, phase=snapshot.context.stage_phase.name,
        system_ready=snapshot.system_ready,
        motion_authorized=snapshot.context.motion_authorized,
        stage_lease_valid=snapshot.context.lease_valid,
        safety_stop_active=snapshot.context.safety_stop,
        selector_invoked=True, diagnostic_valid=False,
        diagnostic_error="selector_result_pending",
        proposal_total=len(snapshot.proposals),
        proposal_feasible_raw=sum(
            p.feasible and p.goal.kind != OptionKind.HOLD_SAFE for p in snapshot.proposals
        ),
        candidate_features=tuple((p.stable_key, p.features) for p in snapshot.proposals),
        n_active=int(snapshot.context.stage_phase == StagePhase.ACTIVE),
        n_available=None, n_choice=None, n_forced=None, n_empty=None,
        proposal_choice=None,
    )


def _canonical_points(points) -> tuple[tuple[float, float], ...]:
    """Remove duplicate/forward-collinear vertices, retaining reversals and bends."""
    compact = []
    for point in points:
        if len(point) != 2 or any(not math.isfinite(v) for v in point):
            raise ValueError("non_finite_route_geometry")
        p = tuple(point)
        if compact and math.dist(compact[-1], p) <= GEOMETRY_TOLERANCE_M:
            continue
        while len(compact) >= 2:
            a, b = compact[-2:]
            ab = (b[0] - a[0], b[1] - a[1])
            bp = (p[0] - b[0], p[1] - b[1])
            cross = abs(ab[0] * bp[1] - ab[1] * bp[0])
            length = math.dist(a, p)
            if (ab[0] * bp[0] + ab[1] * bp[1] < 0
                    or cross > GEOMETRY_TOLERANCE_M * length):
                break
            compact.pop()
        compact.append(p)
    if len(compact) < 2:
        raise ValueError("route_has_no_remaining_motion")
    return tuple(compact)


def _remaining_traversal(path: PlannedPath, edges: Mapping) -> tuple:
    """Resolve the clipped geometry to an OPEN directed suffix of its route.

    _clip_path_to_pose retains historical edge_ids. They must not create false
    diversity when two proposals execute exactly the same remaining suffix.
    """
    segments = []
    for i, edge_id in enumerate(path.edge_ids):
        edge = edges.get(edge_id)
        if edge is None or edge.state != EdgeState.OPEN:
            raise ValueError("route_edge_missing_or_not_open")
        start, end = path.node_ids[i:i + 2]
        if (start, end) == (edge.from_node, edge.to_node):
            points, direction = edge.polyline_xy_m, 1
        elif (start, end) == (edge.to_node, edge.from_node):
            points, direction = edge.polyline_xy_m[::-1], -1
        else:
            raise ValueError("route_edge_node_mismatch")
        for a, b in zip(points, points[1:]):
            if math.dist(a, b) > GEOMETRY_TOLERANCE_M:
                if segments and math.dist(segments[-1][1], a) > GEOMETRY_TOLERANCE_M:
                    raise ValueError("discontinuous_route_geometry")
                segments.append((a, b, (edge_id, direction)))
    realized = _canonical_points(path.polyline_xy_m)
    first = realized[0]
    for i, (a, b, _) in enumerate(segments):
        dx, dy = b[0] - a[0], b[1] - a[1]
        fraction = ((first[0] - a[0]) * dx + (first[1] - a[1]) * dy) / (dx * dx + dy * dy)
        if not 0.0 <= fraction <= 1.0:
            continue
        projection = (a[0] + fraction * dx, a[1] + fraction * dy)
        if math.dist(first, projection) > GEOMETRY_TOLERANCE_M:
            continue
        # At an edge endpoint only the following, nonzero traversal remains.
        offset = i + int(math.dist(first, b) <= GEOMETRY_TOLERANCE_M)
        suffix = segments[offset:]
        if not suffix:
            continue
        points = _canonical_points((first, *(s[1] for s in suffix)))
        if len(points) != len(realized) or any(
            math.dist(p, q) > GEOMETRY_TOLERANCE_M for p, q in zip(points, realized)
        ):
            continue
        traversal = []
        for _, _, identity in suffix:
            if not traversal or traversal[-1] != identity:
                traversal.append(identity)
        return tuple(traversal)
    raise ValueError("realized_geometry_is_not_a_route_suffix")


def behavior_signature(
    goal: OptionGoal, path: PlannedPath | None, graph: TopologyGraph,
    *, edges: Mapping | None = None,
) -> tuple:
    """No instance IDs or arbitrary target IDs in the behavior equivalence key."""
    if goal.kind in (OptionKind.OBSERVE_SAFE, OptionKind.HOLD_SAFE):
        return (BEHAVIOR_SIGNATURE_SCHEMA, goal.kind.name, goal.role.name)
    if path is None:
        raise ValueError("motion_path_unavailable")
    if (path.map_version, path.topology_version, path.localization_epoch) != (
        goal.map_version, goal.topology_version, goal.localization_epoch
    ) or (path.map_version, path.topology_version, path.localization_epoch) != (
        graph.map_version, graph.topology_version, graph.localization_epoch
    ):
        raise ValueError("route_versions_mismatch")
    if goal.has_target_node and path.goal_node_id != goal.target_node_id:
        raise ValueError("route_target_mismatch")
    geometry = _canonical_points(path.polyline_xy_m)
    traversal = _remaining_traversal(
        path, edges if edges is not None else {e.edge_id: e for e in graph.edges}
    )
    return (
        BEHAVIOR_SIGNATURE_SCHEMA, goal.kind.name, goal.role.name,
        traversal, geometry, goal.goal_zone_id, goal.position_tolerance_m,
        goal.target_yaw_required,
        goal.yaw_tolerance_rad if goal.target_yaw_required else None,
        goal.target_pose.theta_rad if goal.target_yaw_required and goal.target_pose else None,
    )


def finish_selection(
    base: SelectionDiagnostics, snapshot: TacticalSnapshot, result: SelectionResult,
    paths_by_key: Mapping[str, PlannedPath], graph: TopologyGraph,
) -> SelectionDiagnostics:
    """Consume final selector evaluations; never evaluate or score candidates again."""
    eligible = tuple(
        e for e in result.alternatives if e.applicable and e.kind != OptionKind.HOLD_SAFE
    )
    rejected = tuple((e.stable_key, e.reason) for e in result.alternatives if not e.applicable)
    priority = min((e.priority for e in eligible), default=None)
    cohort = tuple(e for e in eligible if e.priority == priority)
    urgency = max((e.urgency_rank or 0 for e in cohort), default=None)
    cohort = tuple(e for e in cohort if (e.urgency_rank or 0) == urgency)
    available = int(base.n_active == 1 and bool(eligible))
    common = dict(
        proposal_feasible=len(eligible), winning_cohort_size=len(cohort),
        winning_priority=priority, winning_urgency=urgency,
        winning_cohort_keys=tuple(e.stable_key for e in cohort),
        rejected_candidates=rejected, n_available=available,
        candidate_evaluations=result.alternatives,
        selected_stable_key=result.selected.stable_key,
        selected_option_instance_id=result.selected.goal.option_instance_id,
        utility_profile_id=result.utility_profile_id, selection_changed=result.changed,
    )
    proposals = {p.stable_key: p for p in snapshot.proposals}
    edges = {e.edge_id: e for e in graph.edges} if cohort else {}
    signatures, duplicates, seen = [], [], set()
    try:
        for e in cohort:
            proposal = proposals.get(e.stable_key)
            if proposal is None:
                raise ValueError("evaluated_proposal_missing")
            signature = behavior_signature(
                proposal.goal, paths_by_key.get(e.stable_key), graph, edges=edges
            )
            signatures.append((e.stable_key, signature))
            if signature in seen:
                duplicates.append(e.stable_key)
            seen.add(signature)
    except ValueError as error:
        return replace(
            base, **common, diagnostic_error=f"behavior_signature:{error}",
            diagnostic_valid=False, behavior_signatures=tuple(signatures),
            deduplicated_keys=tuple(duplicates), proposal_choice=None,
            n_choice=None, n_forced=None, n_empty=base.n_active - available,
        )
    choice = int(base.n_active == 1 and len(seen) >= 2)
    return replace(
        base, **common, diagnostic_valid=True, diagnostic_error="",
        behavior_signatures=tuple(signatures), deduplicated_keys=tuple(duplicates),
        proposal_choice=len(seen), n_choice=choice,
        n_forced=available - choice, n_empty=base.n_active - available,
    )
