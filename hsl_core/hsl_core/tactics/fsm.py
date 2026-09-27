"""Deterministic role tactics: feasibility/urgency first, utility second."""

from dataclasses import dataclass, replace
from enum import IntEnum
import math
import threading
from typing import Mapping

from ..match import Role, StagePhase
from .options import (
    OptionContext,
    OptionGoal,
    OptionKind,
    OptionRegistry,
    PredicateDecision,
)
from .utility import UtilityProfile, UtilityResult, ROLE_FEATURES, score


MAX_TACTICAL_PROPOSALS = 256


class TacticalGuard(IntEnum):
    ROBUST_CAPTURE_OPPORTUNITY = 0
    CREDIBLE_IMMINENT_BASE_THREAT = 1
    INTERCEPT_DOMINATES_BASE_DEFENSE = 2
    SUPPORTED_INTERCEPT = 3
    RECENT_OCCLUSION = 4
    USEFUL_RIVAL = 5
    GRAPH_SEARCH_VIEWPOINT = 6
    IMMEDIATE_TRAP_RISK = 7
    VERIFIED_ESCAPE_ROUTE = 8
    BREAK_LOS_FEASIBLE = 9
    CONTESTED_ROUTE = 10
    ALTERNATE_ROUTE_AVAILABLE = 11
    ACCEPTED_GOAL = 12
    GOAL_THREAT_ACCEPTABLE = 13
    INFORMATIVE_SAFE_OBSERVATION = 14


@dataclass(frozen=True)
class InterceptionTiming:
    """Strict worst/best arrival comparison for one declared rival hypothesis."""

    hypothesis_id: str
    evidence_id: str
    guardian_arrival_upper_ns: int
    explorer_arrival_lower_ns: int
    required_buffer_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.hypothesis_id, str) or not self.hypothesis_id.strip():
            raise ValueError("hypothesis_id must be non-empty")
        if not isinstance(self.evidence_id, str) or not self.evidence_id.strip():
            raise ValueError("evidence_id must be non-empty")
        for value, name in (
            (self.guardian_arrival_upper_ns, "guardian_arrival_upper_ns"),
            (self.explorer_arrival_lower_ns, "explorer_arrival_lower_ns"),
            (self.required_buffer_ns, "required_buffer_ns"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")

    @property
    def dominates_base_defense(self) -> bool:
        return (
            self.guardian_arrival_upper_ns + self.required_buffer_ns
            < self.explorer_arrival_lower_ns
        )


@dataclass(frozen=True)
class OptionProposal:
    """One semantically guarded option plus its independently established feasibility.

    Explorer escape/LOS proposals require a discrete 0-255 urgency rank;
    larger ranks are more urgent and are not calibrated probabilities.
    """

    stable_key: str
    goal: OptionGoal
    feasible: bool
    infeasible_reason: str
    guard_facts: tuple[TacticalGuard, ...]
    guard_evidence: tuple[tuple[TacticalGuard, str], ...]
    features: tuple[tuple[str, float], ...]
    urgency_rank: int | None = None
    urgency_evidence_id: str = ""
    interception_timing: InterceptionTiming | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.stable_key, str) or not self.stable_key.strip():
            raise ValueError("stable_key must be non-empty")
        if not isinstance(self.goal, OptionGoal):
            raise ValueError("goal must be a validated OptionGoal")
        if not isinstance(self.feasible, bool):
            raise ValueError("feasible must be boolean")
        if self.feasible and self.infeasible_reason:
            raise ValueError("feasible proposals must not carry an infeasibility reason")
        if not self.feasible and (
            not isinstance(self.infeasible_reason, str)
            or not self.infeasible_reason.strip()
        ):
            raise ValueError("infeasible proposals require a reason")
        escape_options = (OptionKind.KEEP_ESCAPE_ROUTE, OptionKind.BREAK_LOS)
        urgency_is_required = (
            self.goal.role == Role.EXPLORER and self.goal.kind in escape_options
        )
        if urgency_is_required and self.urgency_rank is None:
            raise ValueError("explorer escape options require an ordinal urgency_rank")
        if urgency_is_required and (
            not isinstance(self.urgency_evidence_id, str)
            or not self.urgency_evidence_id.strip()
        ):
            raise ValueError("explorer escape options require urgency evidence")
        if self.urgency_rank is not None:
            if (
                not isinstance(self.urgency_rank, int)
                or isinstance(self.urgency_rank, bool)
                or not 0 <= self.urgency_rank <= 255
            ):
                raise ValueError("urgency_rank must be an integer in [0, 255]")
            if not urgency_is_required:
                raise ValueError("urgency_rank is only defined for explorer escape options")
        elif self.urgency_evidence_id:
            raise ValueError("urgency evidence requires an urgency_rank")
        if self.interception_timing is not None and not isinstance(
            self.interception_timing, InterceptionTiming
        ):
            raise ValueError("interception_timing must be an InterceptionTiming")
        if (
            self.interception_timing is not None
            and self.goal.kind != OptionKind.INTERCEPT_PORTAL
        ):
            raise ValueError("interception timing is only valid for INTERCEPT_PORTAL")
        if not isinstance(self.guard_facts, (tuple, list, set, frozenset)):
            raise ValueError("guard_facts must be a sequence of TacticalGuard values")
        facts = tuple(sorted((_guard(fact) for fact in self.guard_facts), key=int))
        if len(set(facts)) != len(facts):
            raise ValueError("guard_facts must be unique")
        if not isinstance(self.guard_evidence, (tuple, list)):
            raise ValueError("guard_evidence must be a sequence of guard/evidence pairs")
        evidence_by_guard: dict[TacticalGuard, str] = {}
        for entry in self.guard_evidence:
            if not isinstance(entry, (tuple, list)) or len(entry) != 2:
                raise ValueError("each guard evidence entry must contain a guard and ID")
            fact, evidence_id = entry
            fact = _guard(fact)
            if fact in evidence_by_guard:
                raise ValueError("guard evidence entries must be unique")
            if not isinstance(evidence_id, str) or not evidence_id.strip():
                raise ValueError("guard evidence IDs must be non-empty")
            evidence_by_guard[fact] = evidence_id
        if set(evidence_by_guard) != set(facts):
            raise ValueError("every guard fact requires exactly one evidence ID")
        if not isinstance(self.features, (tuple, list)):
            raise ValueError("features must be a sequence of name/value pairs")
        parsed: dict[str, float] = {}
        for entry in self.features:
            if not isinstance(entry, (tuple, list)) or len(entry) != 2:
                raise ValueError("each feature must contain a name and value")
            name, value = entry
            if not isinstance(name, str) or not name:
                raise ValueError("feature names must be non-empty")
            if name in parsed:
                raise ValueError(f"duplicate feature: {name}")
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"feature {name} must be numeric")
            if not math.isfinite(value):
                raise ValueError(f"feature {name} must be finite")
            parsed[name] = float(value)
        if self.goal.kind == OptionKind.HOLD_SAFE:
            if parsed:
                raise ValueError("HOLD_SAFE must not be utility-ranked")
        object.__setattr__(self, "guard_facts", facts)
        object.__setattr__(
            self,
            "guard_evidence",
            tuple(sorted(evidence_by_guard.items(), key=lambda item: int(item[0]))),
        )
        object.__setattr__(self, "features", tuple(sorted(parsed.items())))


@dataclass(frozen=True)
class TacticalSnapshot:
    context: OptionContext
    proposals: tuple[OptionProposal, ...]
    system_ready: bool
    health_reason: str

    def __post_init__(self) -> None:
        if not isinstance(self.context, OptionContext):
            raise ValueError("context must be an OptionContext")
        if not isinstance(self.proposals, (tuple, list)):
            raise ValueError("proposals must be a sequence")
        proposals = tuple(self.proposals)
        if any(not isinstance(item, OptionProposal) for item in proposals):
            raise ValueError("proposals must contain OptionProposal values")
        object.__setattr__(self, "proposals", proposals)
        if not isinstance(self.system_ready, bool):
            raise ValueError("system_ready must be boolean")
        if self.system_ready:
            if self.health_reason:
                raise ValueError("ready snapshot must not have a health failure reason")
        elif not isinstance(self.health_reason, str) or not self.health_reason.strip():
            raise ValueError("unready snapshot requires a health failure reason")
        keys = [item.stable_key for item in proposals]
        instance_ids = [item.goal.option_instance_id for item in proposals]
        if len(set(keys)) != len(keys):
            raise ValueError("proposal stable keys must be unique")
        if len(set(instance_ids)) != len(instance_ids):
            raise ValueError("proposal option_instance_id values must be unique")


@dataclass(frozen=True)
class CandidateEvaluation:
    stable_key: str
    kind: OptionKind
    applicable: bool
    reason: str
    priority: int | None
    utility: UtilityResult | None
    guard_evidence: tuple[tuple[TacticalGuard, str], ...]
    urgency_rank: int | None
    urgency_evidence_id: str


@dataclass(frozen=True)
class SelectionResult:
    selected: OptionProposal
    priority: int | None
    utility: UtilityResult | None
    reason: str
    changed: bool
    alternatives: tuple[CandidateEvaluation, ...]
    utility_profile_id: str


class TacticalSelector:
    """Pure ranking with explicit caller-provided state and bounded parameters."""

    def __init__(
        self,
        utility_profile: UtilityProfile,
        *,
        registry: OptionRegistry,
        hysteresis_delta_u: float,
        minimum_dwell_ns: int,
        max_proposals: int,
    ) -> None:
        if not isinstance(utility_profile, UtilityProfile):
            raise ValueError("utility_profile must be explicit and validated")
        if not isinstance(registry, OptionRegistry):
            raise ValueError("registry must be an OptionRegistry")
        if (
            isinstance(hysteresis_delta_u, bool)
            or not isinstance(hysteresis_delta_u, (int, float))
            or not math.isfinite(hysteresis_delta_u)
            or not 0.0 <= hysteresis_delta_u <= 2.0
        ):
            raise ValueError("hysteresis_delta_u must be finite and within [0, 2]")
        if (
            not isinstance(minimum_dwell_ns, int)
            or isinstance(minimum_dwell_ns, bool)
            or minimum_dwell_ns < 0
        ):
            raise ValueError("minimum_dwell_ns must be a non-negative integer")
        if (
            not isinstance(max_proposals, int)
            or isinstance(max_proposals, bool)
            or max_proposals < 1
            or max_proposals > MAX_TACTICAL_PROPOSALS
        ):
            raise ValueError(
                f"max_proposals must be an integer in [1, {MAX_TACTICAL_PROPOSALS}]"
            )
        self.utility_profile = utility_profile
        self.registry = registry
        self.hysteresis_delta_u = float(hysteresis_delta_u)
        self.minimum_dwell_ns = minimum_dwell_ns
        self.max_proposals = max_proposals
        self._lock = threading.RLock()
        self._last_now_ns = 0

    def select(
        self,
        snapshot: TacticalSnapshot,
        *,
        current_stable_key: str = "",
        current_option_instance_id: str = "",
        current_started_at_ns: int | None = None,
    ) -> SelectionResult:
        if not isinstance(snapshot, TacticalSnapshot):
            raise ValueError("snapshot must be a TacticalSnapshot")
        self._validate_current(
            snapshot.context.now_ns,
            current_stable_key,
            current_option_instance_id,
            current_started_at_ns,
        )
        with self._lock:
            now_ns = snapshot.context.now_ns
            if now_ns < self._last_now_ns:
                raise ValueError("tactical selector time must be monotonic")
            if len(snapshot.proposals) > self.max_proposals:
                raise ValueError("proposal count exceeds configured bound")
            role = snapshot.context.role
            if role != self.utility_profile.role:
                raise ValueError("utility profile role does not match snapshot role")
            fallback = self._fallback(snapshot, role)
            forced_reason = self._hard_stop_reason(snapshot)
            evaluations = tuple(
                sorted(
                    (
                        self._evaluate(proposal, snapshot.context, role)
                        for proposal in snapshot.proposals
                    ),
                    key=lambda item: item.stable_key,
                )
            )
            if forced_reason:
                evaluations = tuple(
                    _blocked_by_global_gate(item, forced_reason)
                    for item in evaluations
                )
            by_key = {item.stable_key: item for item in evaluations}
            proposal_by_key = {item.stable_key: item for item in snapshot.proposals}

            if forced_reason:
                result = SelectionResult(
                    selected=fallback,
                    priority=None,
                    utility=None,
                    reason=forced_reason,
                    changed=(
                        current_stable_key != fallback.stable_key
                        or current_option_instance_id
                        != fallback.goal.option_instance_id
                    ),
                    alternatives=evaluations,
                    utility_profile_id=self.utility_profile.profile_id,
                )
                self._last_now_ns = now_ns
                return result

            eligible = tuple(
                item
                for item in evaluations
                if item.applicable and item.kind != OptionKind.HOLD_SAFE
            )
            if not eligible:
                result = SelectionResult(
                    selected=fallback,
                    priority=None,
                    utility=None,
                    reason="no_feasible_option",
                    changed=(
                        current_stable_key != fallback.stable_key
                        or current_option_instance_id
                        != fallback.goal.option_instance_id
                    ),
                    alternatives=evaluations,
                    utility_profile_id=self.utility_profile.profile_id,
                )
                self._last_now_ns = now_ns
                return result

            best_eval = min(
                eligible,
                key=lambda item: (
                    item.priority,
                    -_urgency_rank(proposal_by_key[item.stable_key]),
                    -item.utility.value,
                    int(item.kind),
                    _target_order(proposal_by_key[item.stable_key].goal),
                    item.stable_key,
                ),
            )
            current_eval = by_key.get(current_stable_key) if current_stable_key else None
            current_proposal = proposal_by_key.get(current_stable_key) if current_stable_key else None
            current_is_valid = (
                current_eval is not None
                and current_eval.applicable
                and current_eval.kind != OptionKind.HOLD_SAFE
                and current_proposal is not None
                and current_proposal.goal.option_instance_id == current_option_instance_id
            )
            chosen = best_eval
            reason = "highest_priority_then_utility"
            if current_is_valid:
                if best_eval.priority < current_eval.priority:
                    chosen = best_eval
                    reason = "higher_priority_preemption"
                elif current_eval.priority < best_eval.priority:
                    chosen = current_eval
                    reason = "current_higher_priority"
                elif (
                    current_eval.priority == best_eval.priority
                    and current_eval.stable_key != best_eval.stable_key
                ):
                    current_proposal = proposal_by_key[current_eval.stable_key]
                    best_proposal = proposal_by_key[best_eval.stable_key]
                    if _urgency_rank(best_proposal) > _urgency_rank(current_proposal):
                        chosen = best_eval
                        reason = "higher_urgency_preemption"
                    elif _urgency_rank(best_proposal) < _urgency_rank(current_proposal):
                        chosen = current_eval
                        reason = "current_higher_urgency"
                    elif best_eval.utility.value <= (
                        current_eval.utility.value + self.hysteresis_delta_u
                    ):
                        chosen = current_eval
                        reason = "hysteresis" if best_eval.utility.value > current_eval.utility.value else "utility_tie"
                    elif now_ns - current_started_at_ns < self.minimum_dwell_ns:
                        chosen = current_eval
                        reason = "minimum_dwell"
                    else:
                        reason = "utility_improvement_exceeds_hysteresis"
                elif current_eval.stable_key == best_eval.stable_key:
                    chosen = current_eval
                    reason = "current_remains_best"
                else:
                    chosen = current_eval
                    reason = "utility_tie"
            selected = proposal_by_key[chosen.stable_key]
            changed = (
                selected.stable_key != current_stable_key
                or selected.goal.option_instance_id != current_option_instance_id
            )
            result = SelectionResult(
                selected=selected,
                priority=chosen.priority,
                utility=chosen.utility,
                reason=reason,
                changed=changed,
                alternatives=evaluations,
                utility_profile_id=self.utility_profile.profile_id,
            )
            self._last_now_ns = now_ns
            return result

    @staticmethod
    def _validate_current(
        now_ns: int,
        stable_key: str,
        instance_id: str,
        started_at_ns: int | None,
    ) -> None:
        provided = (bool(stable_key), bool(instance_id), started_at_ns is not None)
        if any(provided) and not all(provided):
            raise ValueError("current option key, instance ID and start time must be supplied together")
        if started_at_ns is not None:
            if (
                not isinstance(started_at_ns, int)
                or isinstance(started_at_ns, bool)
                or started_at_ns < 0
                or started_at_ns > now_ns
            ):
                raise ValueError("current_started_at_ns must be within [0, now_ns]")

    def _fallback(self, snapshot: TacticalSnapshot, role: Role) -> OptionProposal:
        fallbacks = [
            proposal
            for proposal in snapshot.proposals
            if proposal.goal.kind == OptionKind.HOLD_SAFE
        ]
        if len(fallbacks) != 1:
            raise ValueError("exactly one HOLD_SAFE proposal is required")
        fallback = fallbacks[0]
        decision = self.registry.check_initiation(fallback.goal, snapshot.context)
        if fallback.goal.role != role or not decision.accepted:
            raise ValueError(f"HOLD_SAFE fallback is invalid: {decision.reason}")
        if not fallback.feasible:
            raise ValueError("HOLD_SAFE fallback must always be feasible")
        if fallback.guard_facts:
            raise ValueError("HOLD_SAFE fallback must not require guard facts")
        return fallback

    def _hard_stop_reason(self, snapshot: TacticalSnapshot) -> str:
        context = snapshot.context
        if not snapshot.system_ready:
            return f"system_not_ready:{snapshot.health_reason}"
        if context.safety_stop:
            return "safety_stop_active"
        if not context.lease_valid:
            return "stage_lease_invalid"
        if context.stage_phase != StagePhase.ACTIVE or not context.motion_authorized:
            return "stage_motion_not_authorized"
        return ""

    def _evaluate(
        self, proposal: OptionProposal, context: OptionContext, role: Role
    ) -> CandidateEvaluation:
        kind = proposal.goal.kind
        if kind == OptionKind.HOLD_SAFE:
            decision = self.registry.check_initiation(proposal.goal, context)
            if not decision.accepted:
                return _candidate_evaluation(
                    proposal, False, decision.reason, None, None
                )
            return _candidate_evaluation(proposal, True, "safe_fallback", None, None)
        if not proposal.feasible:
            return _candidate_evaluation(
                proposal, False, proposal.infeasible_reason, None, None
            )
        if proposal.goal.role != role:
            return _candidate_evaluation(proposal, False, "role_not_allowed", None, None)
        decision = self.registry.check_initiation(proposal.goal, context)
        if not decision.accepted:
            return _candidate_evaluation(proposal, False, decision.reason, None, None)
        if not _guard_facts_valid(proposal):
            return _candidate_evaluation(
                proposal, False, "guard_fact_not_applicable", None, None
            )
        if (
            TacticalGuard.INTERCEPT_DOMINATES_BASE_DEFENSE in proposal.guard_facts
            and (
                proposal.interception_timing is None
                or not proposal.interception_timing.dominates_base_defense
            )
        ):
            return _candidate_evaluation(
                proposal, False, "intercept_time_advantage_not_proven", None, None
            )
        required = _required_guards(role, kind, proposal.guard_facts)
        if required is None:
            return _candidate_evaluation(
                proposal, False, "option_not_defined_for_role", None, None
            )
        missing = required - set(proposal.guard_facts)
        if missing:
            names = ",".join(item.name for item in sorted(missing, key=int))
            return _candidate_evaluation(
                proposal, False, f"guard_not_satisfied:{names}", None, None
            )
        features = dict(proposal.features)
        if set(features) != ROLE_FEATURES[role]:
            return _candidate_evaluation(
                proposal, False, "utility_feature_schema_mismatch", None, None
            )
        try:
            utility = score(features, self.utility_profile)
        except ValueError as error:
            return _candidate_evaluation(
                proposal, False, f"invalid_utility_features:{error}", None, None
            )
        priority = _priority(role, kind, proposal.guard_facts)
        if priority is None:
            return _candidate_evaluation(
                proposal, False, "option_has_no_role_priority", None, None
            )
        return _candidate_evaluation(proposal, True, "applicable", priority, utility)


def _candidate_evaluation(
    proposal: OptionProposal,
    applicable: bool,
    reason: str,
    priority: int | None,
    utility: UtilityResult | None,
) -> CandidateEvaluation:
    return CandidateEvaluation(
        stable_key=proposal.stable_key,
        kind=proposal.goal.kind,
        applicable=applicable,
        reason=reason,
        priority=priority,
        utility=utility,
        guard_evidence=proposal.guard_evidence,
        urgency_rank=proposal.urgency_rank,
        urgency_evidence_id=proposal.urgency_evidence_id,
    )


def _blocked_by_global_gate(
    evaluation: CandidateEvaluation, reason: str
) -> CandidateEvaluation:
    if evaluation.kind == OptionKind.HOLD_SAFE or not evaluation.applicable:
        return evaluation
    return replace(
        evaluation,
        applicable=False,
        reason=f"global_gate:{reason}",
        priority=None,
        utility=None,
    )


def _required_guards(
    role: Role, kind: OptionKind, facts: tuple[TacticalGuard, ...]
) -> set[TacticalGuard] | None:
    if role == Role.GUARDIAN:
        required = {
            OptionKind.APPROACH_CAPTURE: {TacticalGuard.ROBUST_CAPTURE_OPPORTUNITY},
            OptionKind.FALLBACK_DEFEND_BASE: {
                TacticalGuard.CREDIBLE_IMMINENT_BASE_THREAT
            },
            OptionKind.INTERCEPT_PORTAL: {TacticalGuard.SUPPORTED_INTERCEPT},
            OptionKind.RECOVER_VIEW: {TacticalGuard.RECENT_OCCLUSION},
            OptionKind.PRESSURE_ROUTE: {TacticalGuard.USEFUL_RIVAL},
            OptionKind.SEARCH_PORTAL: {TacticalGuard.GRAPH_SEARCH_VIEWPOINT},
        }
    else:
        required = {
            OptionKind.KEEP_ESCAPE_ROUTE: {
                TacticalGuard.IMMEDIATE_TRAP_RISK,
                TacticalGuard.VERIFIED_ESCAPE_ROUTE,
            },
            OptionKind.BREAK_LOS: {
                TacticalGuard.IMMEDIATE_TRAP_RISK,
                TacticalGuard.BREAK_LOS_FEASIBLE,
            },
            OptionKind.TAKE_ALTERNATE_PORTAL: {
                TacticalGuard.CONTESTED_ROUTE,
                TacticalGuard.ALTERNATE_ROUTE_AVAILABLE,
            },
            OptionKind.ADVANCE_BASE: {
                TacticalGuard.ACCEPTED_GOAL,
                TacticalGuard.GOAL_THREAT_ACCEPTABLE,
            },
            OptionKind.OBSERVE_SAFE: {
                TacticalGuard.INFORMATIVE_SAFE_OBSERVATION
            },
        }
    result = required.get(kind)
    if result is None:
        return None
    if (
        role == Role.GUARDIAN
        and kind == OptionKind.INTERCEPT_PORTAL
        and TacticalGuard.CREDIBLE_IMMINENT_BASE_THREAT in facts
    ):
        return result | {
            TacticalGuard.CREDIBLE_IMMINENT_BASE_THREAT,
            TacticalGuard.INTERCEPT_DOMINATES_BASE_DEFENSE,
        }
    return result


def _guard_facts_valid(proposal: OptionProposal) -> bool:
    role = proposal.goal.role
    kind = proposal.goal.kind
    fact_set = set(proposal.guard_facts)
    if role == Role.GUARDIAN and kind == OptionKind.INTERCEPT_PORTAL:
        allowed = {
            TacticalGuard.SUPPORTED_INTERCEPT,
            TacticalGuard.CREDIBLE_IMMINENT_BASE_THREAT,
            TacticalGuard.INTERCEPT_DOMINATES_BASE_DEFENSE,
        }
        if (
            TacticalGuard.INTERCEPT_DOMINATES_BASE_DEFENSE in fact_set
            and (
                TacticalGuard.CREDIBLE_IMMINENT_BASE_THREAT not in fact_set
                or proposal.interception_timing is None
            )
        ):
            return False
        if (
            proposal.interception_timing is not None
            and TacticalGuard.INTERCEPT_DOMINATES_BASE_DEFENSE not in fact_set
        ):
            return False
        return fact_set <= allowed
    if proposal.interception_timing is not None:
        return False
    required = _required_guards(role, kind, ())
    return required is not None and fact_set <= required


def _priority(
    role: Role, kind: OptionKind, facts: tuple[TacticalGuard, ...]
) -> int | None:
    fact_set = set(facts)
    if role == Role.GUARDIAN:
        if kind == OptionKind.APPROACH_CAPTURE:
            return 0
        if kind == OptionKind.FALLBACK_DEFEND_BASE:
            return 1
        if kind == OptionKind.INTERCEPT_PORTAL:
            if (
                TacticalGuard.CREDIBLE_IMMINENT_BASE_THREAT in fact_set
                and TacticalGuard.INTERCEPT_DOMINATES_BASE_DEFENSE in fact_set
            ):
                return 1
            return 2
        return {
            OptionKind.RECOVER_VIEW: 3,
            OptionKind.PRESSURE_ROUTE: 4,
            OptionKind.SEARCH_PORTAL: 5,
        }.get(kind)
    if kind in (OptionKind.KEEP_ESCAPE_ROUTE, OptionKind.BREAK_LOS):
        return 0
    return {
        OptionKind.TAKE_ALTERNATE_PORTAL: 1,
        OptionKind.ADVANCE_BASE: 2,
        OptionKind.OBSERVE_SAFE: 3,
    }.get(kind)


def _guard(value: TacticalGuard | int) -> TacticalGuard:
    if isinstance(value, TacticalGuard):
        return value
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError("guard facts must be TacticalGuard values")
    try:
        return TacticalGuard(value)
    except ValueError as error:
        raise ValueError("guard facts must be TacticalGuard values") from error


def _target_order(goal: OptionGoal) -> tuple[object, ...]:
    if goal.has_target_node:
        return (0, goal.target_node_id, goal.goal_zone_id)
    if goal.target_pose is not None:
        return (
            1,
            goal.target_pose.x_m,
            goal.target_pose.y_m,
            goal.target_pose.theta_rad,
            goal.goal_zone_id,
        )
    return (2, goal.goal_zone_id)


def _urgency_rank(proposal: OptionProposal) -> int:
    return 0 if proposal.urgency_rank is None else proposal.urgency_rank
