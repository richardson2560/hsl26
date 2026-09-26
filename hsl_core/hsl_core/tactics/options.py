"""Canonical option contracts and single-owner execution authority."""

from collections import deque
from dataclasses import dataclass, replace
from enum import IntEnum
import math
import threading
from types import MappingProxyType
from typing import Iterable

from ..match import Role, StagePhase
from ..planning.execution import ExecutionLease
from ..types import Pose2D


class OptionKind(IntEnum):
    HOLD_SAFE = 0
    SEARCH_PORTAL = 1
    INTERCEPT_PORTAL = 2
    PRESSURE_ROUTE = 3
    APPROACH_CAPTURE = 4
    RECOVER_VIEW = 5
    FALLBACK_DEFEND_BASE = 6
    ADVANCE_BASE = 7
    BREAK_LOS = 8
    TAKE_ALTERNATE_PORTAL = 9
    KEEP_ESCAPE_ROUTE = 10
    OBSERVE_SAFE = 11


class OptionPhase(IntEnum):
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    REPLANNING = 3
    CANCELING = 4
    FINISHED = 5


class OptionOutcome(IntEnum):
    SUCCESS = 0
    CANCELED = 1
    TIMEOUT = 2
    PRECONDITION_FAILED = 3
    FEASIBILITY_LOST = 4
    SAFETY_STOP = 5
    STAGE_ENDED = 6
    INTERNAL_ERROR = 7


@dataclass(frozen=True)
class OptionGoal:
    """Validated, ROS-independent form of the ExecuteOption goal."""

    option_instance_id: str
    kind: OptionKind
    role: Role
    stage_id: str
    clock_epoch: str
    localization_epoch: str
    deadline_ns: int
    map_version: int
    topology_version: int
    parameters_id: str
    schema_version: int
    has_target_node: bool = False
    target_node_id: int = 0
    target_pose: Pose2D | None = None
    target_yaw_required: bool = False
    position_tolerance_m: float = 0.05
    yaw_tolerance_rad: float = 0.1
    goal_zone_id: str = ""

    def __post_init__(self) -> None:
        for value, field_name in (
            (self.option_instance_id, "option_instance_id"),
            (self.stage_id, "stage_id"),
            (self.clock_epoch, "clock_epoch"),
            (self.localization_epoch, "localization_epoch"),
        ):
            _nonempty(value, field_name)
        object.__setattr__(self, "kind", _enum(OptionKind, self.kind, "kind"))
        object.__setattr__(self, "role", _enum(Role, self.role, "role"))
        _integer(self.schema_version, "schema_version", minimum=1)
        if self.schema_version != 2:
            raise ValueError("unsupported option schema_version")
        _integer(self.deadline_ns, "deadline_ns", minimum=1)
        _integer(self.map_version, "map_version", minimum=0)
        _integer(self.topology_version, "topology_version", minimum=0)
        _integer(self.target_node_id, "target_node_id", minimum=0)
        if not isinstance(self.has_target_node, bool):
            raise ValueError("has_target_node must be boolean")
        if not isinstance(self.target_yaw_required, bool):
            raise ValueError("target_yaw_required must be boolean")
        if self.target_pose is not None and not isinstance(self.target_pose, Pose2D):
            raise ValueError("target_pose must be a Pose2D or None")
        if self.has_target_node and self.target_pose is not None:
            raise ValueError("goal must not specify both a target node and target pose")
        if not self.has_target_node and self.target_node_id != 0:
            raise ValueError("target_node_id must be zero when has_target_node is false")
        if not isinstance(self.parameters_id, str):
            raise ValueError("parameters_id must be a string")
        if not isinstance(self.goal_zone_id, str):
            raise ValueError("goal_zone_id must be a string")
        _nonnegative_finite(self.position_tolerance_m, "position_tolerance_m")
        _nonnegative_finite(self.yaw_tolerance_rad, "yaw_tolerance_rad")

    @property
    def has_target(self) -> bool:
        return self.has_target_node or self.target_pose is not None


@dataclass(frozen=True)
class OptionContext:
    """One coherent authority snapshot supplied to admission and lifecycle checks."""

    now_ns: int
    stage_id: str
    stage_phase: StagePhase
    stage_ends_at_ns: int
    role: Role
    clock_epoch: str
    localization_epoch: str
    map_version: int
    topology_version: int
    motion_authorized: bool
    lease_valid: bool
    safety_stop: bool
    accepted_goal_zone_ids: tuple[str, ...] = ()
    path_valid: bool = False
    effect_satisfied: bool = False
    effect_instance_id: str = ""
    effect_evidence_id: str = ""
    feasibility_lost: bool = False

    def __post_init__(self) -> None:
        _integer(self.now_ns, "now_ns", minimum=0)
        _integer(self.stage_ends_at_ns, "stage_ends_at_ns", minimum=0)
        _integer(self.map_version, "map_version", minimum=0)
        _integer(self.topology_version, "topology_version", minimum=0)
        _nonempty(self.stage_id, "stage_id")
        _nonempty(self.clock_epoch, "clock_epoch")
        _nonempty(self.localization_epoch, "localization_epoch")
        object.__setattr__(self, "stage_phase", _enum(StagePhase, self.stage_phase, "stage_phase"))
        object.__setattr__(self, "role", _enum(Role, self.role, "role"))
        for value, field_name in (
            (self.motion_authorized, "motion_authorized"),
            (self.lease_valid, "lease_valid"),
            (self.safety_stop, "safety_stop"),
            (self.path_valid, "path_valid"),
            (self.effect_satisfied, "effect_satisfied"),
            (self.feasibility_lost, "feasibility_lost"),
        ):
            if not isinstance(value, bool):
                raise ValueError(f"{field_name} must be boolean")
        if not isinstance(self.effect_instance_id, str) or not isinstance(
            self.effect_evidence_id, str
        ):
            raise ValueError("effect identity fields must be strings")
        if self.effect_satisfied:
            if not self.effect_instance_id.strip() or not self.effect_evidence_id.strip():
                raise ValueError("satisfied effects require an instance and evidence ID")
        elif self.effect_instance_id or self.effect_evidence_id:
            raise ValueError("effect identity fields require effect_satisfied")
        if not isinstance(self.accepted_goal_zone_ids, (tuple, list)):
            raise ValueError("accepted_goal_zone_ids must be a sequence")
        zone_ids = tuple(self.accepted_goal_zone_ids)
        if any(not isinstance(zone_id, str) or not zone_id.strip() for zone_id in zone_ids):
            raise ValueError("accepted_goal_zone_ids must contain non-empty IDs")
        if len(set(zone_ids)) != len(zone_ids):
            raise ValueError("accepted_goal_zone_ids must be unique")
        object.__setattr__(self, "accepted_goal_zone_ids", zone_ids)


@dataclass(frozen=True)
class PredicateDecision:
    accepted: bool
    reason: str

    def __post_init__(self) -> None:
        if not isinstance(self.accepted, bool):
            raise ValueError("accepted must be boolean")
        _nonempty(self.reason, "reason")


@dataclass(frozen=True)
class OptionDefinition:
    kind: OptionKind
    allowed_roles: tuple[Role, ...]
    target_required: bool
    target_forbidden: bool
    goal_zone_required: bool


@dataclass(frozen=True)
class ExecutionState:
    """The sole core-owned projection corresponding to ExecutionState.msg."""

    active_option_instance_id: str
    phase: OptionPhase
    candidate_authorized: bool
    sequence: int
    lease_generation: int
    stage_id: str
    clock_epoch: str
    localization_epoch: str
    updated_at_ns: int
    reason: str


@dataclass(frozen=True)
class OptionResult:
    action_id: str
    option_instance_id: str
    outcome: OptionOutcome
    reason: str
    started_at_ns: int
    finished_at_ns: int

    @property
    def elapsed_ns(self) -> int:
        return self.finished_at_ns - self.started_at_ns


@dataclass(frozen=True)
class OptionAuditRecord:
    sequence: int
    time_ns: int
    event: str
    action_id: str
    option_instance_id: str
    phase: OptionPhase
    candidate_authorized: bool
    lease_generation: int
    reason: str


@dataclass(frozen=True)
class ActionAdmission:
    accepted: bool
    reason: str
    execution_state: ExecutionState


@dataclass
class _ActionRecord:
    goal: OptionGoal
    admission: ActionAdmission
    instance: "_OptionInstance | None"
    result: OptionResult | None = None


@dataclass(frozen=True)
class _OptionInstance:
    action_id: str
    goal: OptionGoal
    phase: OptionPhase
    started_at_ns: int
    lease_generation: int
    cancel_deadline_ns: int = 0


class OptionRegistry:
    """Single source of truth for option role, target and precondition semantics."""

    def __init__(self, definitions: Iterable[OptionDefinition] | None = None) -> None:
        if definitions is None:
            definitions = _default_definitions()
        by_kind: dict[OptionKind, OptionDefinition] = {}
        for definition in definitions:
            if not isinstance(definition, OptionDefinition):
                raise ValueError("definitions must contain OptionDefinition values")
            kind = _enum(OptionKind, definition.kind, "definition.kind")
            if kind in by_kind:
                raise ValueError(f"duplicate option definition: {kind.name}")
            if not isinstance(definition.allowed_roles, (tuple, list)):
                raise ValueError("allowed_roles must be a sequence")
            roles = tuple(_enum(Role, role, "allowed_role") for role in definition.allowed_roles)
            if not roles or len(set(roles)) != len(roles):
                raise ValueError("each option definition needs unique allowed roles")
            if any(
                not isinstance(value, bool)
                for value in (
                    definition.target_required,
                    definition.target_forbidden,
                    definition.goal_zone_required,
                )
            ):
                raise ValueError("option definition flags must be boolean")
            if definition.target_required and definition.target_forbidden:
                raise ValueError("a target cannot be both required and forbidden")
            by_kind[kind] = replace(definition, kind=kind, allowed_roles=roles)
        if set(by_kind) != set(OptionKind):
            raise ValueError("registry must define every OptionKind exactly once")
        self._definitions = MappingProxyType(by_kind)

    @property
    def definitions(self) -> tuple[OptionDefinition, ...]:
        return tuple(self._definitions[kind] for kind in OptionKind)

    def definition(self, kind: OptionKind | int) -> OptionDefinition:
        return self._definitions[_enum(OptionKind, kind, "kind")]

    def check_initiation(self, goal: OptionGoal, context: OptionContext) -> PredicateDecision:
        if not isinstance(goal, OptionGoal) or not isinstance(context, OptionContext):
            raise ValueError("goal and context must be validated option contracts")
        definition = self.definition(goal.kind)
        if goal.role not in definition.allowed_roles:
            return PredicateDecision(False, "role_not_allowed")
        if goal.stage_id != context.stage_id:
            return PredicateDecision(False, "stage_mismatch")
        if goal.clock_epoch != context.clock_epoch:
            return PredicateDecision(False, "clock_epoch_mismatch")
        if goal.localization_epoch != context.localization_epoch:
            return PredicateDecision(False, "localization_epoch_mismatch")
        if goal.kind == OptionKind.HOLD_SAFE:
            if goal.deadline_ns <= context.now_ns:
                return PredicateDecision(False, "goal_deadline_expired")
            if goal.role != context.role:
                return PredicateDecision(False, "role_mismatch")
            if (
                goal.has_target
                or goal.goal_zone_id
                or goal.target_yaw_required
                or goal.position_tolerance_m != 0.0
                or goal.yaw_tolerance_rad != 0.0
            ):
                return PredicateDecision(False, "hold_safe_must_not_have_target")
            return PredicateDecision(True, "admissible_hold_safe")
        if goal.deadline_ns <= context.now_ns:
            return PredicateDecision(False, "goal_deadline_expired")
        if context.stage_ends_at_ns <= context.now_ns:
            return PredicateDecision(False, "stage_deadline_expired")
        if goal.deadline_ns > context.stage_ends_at_ns:
            return PredicateDecision(False, "goal_deadline_exceeds_stage")
        if goal.role != context.role:
            return PredicateDecision(False, "role_mismatch")
        if context.stage_phase != StagePhase.ACTIVE:
            return PredicateDecision(False, "stage_not_active")
        if not context.lease_valid or not context.motion_authorized:
            return PredicateDecision(False, "motion_authority_unavailable")
        if context.safety_stop:
            return PredicateDecision(False, "safety_stop_active")
        if goal.map_version != context.map_version:
            return PredicateDecision(False, "map_version_mismatch")
        if goal.topology_version != context.topology_version:
            return PredicateDecision(False, "topology_version_mismatch")
        if not goal.parameters_id.strip():
            return PredicateDecision(False, "parameters_id_missing")
        if goal.position_tolerance_m <= 0.0 or goal.yaw_tolerance_rad <= 0.0:
            return PredicateDecision(False, "target_tolerance_invalid")
        if definition.target_required and not goal.has_target:
            return PredicateDecision(False, "target_required")
        if definition.target_forbidden and goal.has_target:
            return PredicateDecision(False, "target_forbidden")
        if definition.goal_zone_required and not goal.goal_zone_id:
            return PredicateDecision(False, "goal_zone_required")
        if goal.goal_zone_id and goal.goal_zone_id not in context.accepted_goal_zone_ids:
            return PredicateDecision(False, "goal_zone_unresolved")
        return PredicateDecision(True, "admissible")

    def check_invariant(
        self, goal: OptionGoal, context: OptionContext, *, executing: bool
    ) -> PredicateDecision:
        if not isinstance(executing, bool):
            raise ValueError("executing must be boolean")
        definition = self.definition(goal.kind)
        if goal.stage_id != context.stage_id:
            return PredicateDecision(False, "stage_mismatch")
        if goal.clock_epoch != context.clock_epoch:
            return PredicateDecision(False, "clock_epoch_mismatch")
        if goal.localization_epoch != context.localization_epoch:
            return PredicateDecision(False, "localization_epoch_mismatch")
        if goal.role != context.role or goal.role not in definition.allowed_roles:
            return PredicateDecision(False, "role_mismatch")
        if goal.kind == OptionKind.HOLD_SAFE:
            return PredicateDecision(True, "hold_safe_invariant")
        if context.stage_phase != StagePhase.ACTIVE:
            return PredicateDecision(False, "stage_not_active")
        if context.now_ns >= goal.deadline_ns or context.now_ns >= context.stage_ends_at_ns:
            return PredicateDecision(False, "deadline_reached")
        if not context.lease_valid or not context.motion_authorized:
            return PredicateDecision(False, "motion_authority_unavailable")
        if context.safety_stop:
            return PredicateDecision(False, "safety_stop_active")
        if goal.map_version != context.map_version:
            return PredicateDecision(False, "map_version_mismatch")
        if goal.topology_version != context.topology_version:
            return PredicateDecision(False, "topology_version_mismatch")
        if goal.goal_zone_id and goal.goal_zone_id not in context.accepted_goal_zone_ids:
            return PredicateDecision(False, "goal_zone_unresolved")
        if executing and not context.path_valid:
            return PredicateDecision(False, "path_invalid")
        if context.feasibility_lost:
            return PredicateDecision(False, "feasibility_lost")
        return PredicateDecision(True, "invariants_hold")

    def check_termination(
        self, goal: OptionGoal, context: OptionContext, *, executing: bool
    ) -> tuple[OptionOutcome | None, str]:
        if not isinstance(executing, bool):
            raise ValueError("executing must be boolean")
        if goal.stage_id != context.stage_id or context.stage_phase in (
            StagePhase.INIT,
            StagePhase.TERMINAL,
        ):
            return OptionOutcome.STAGE_ENDED, "stage_ended"
        if context.now_ns >= goal.deadline_ns or context.now_ns >= context.stage_ends_at_ns:
            return OptionOutcome.TIMEOUT, "deadline_reached"
        if goal.kind == OptionKind.HOLD_SAFE:
            return None, "running"
        if context.safety_stop:
            return OptionOutcome.SAFETY_STOP, "safety_stop_active"
        if context.feasibility_lost:
            return OptionOutcome.FEASIBILITY_LOST, "feasibility_lost"
        if (
            executing
            and context.effect_satisfied
            and context.effect_instance_id == goal.option_instance_id
        ):
            return OptionOutcome.SUCCESS, "effect_satisfied"
        return None, "running"


class OptionAuthority:
    """Serializes action admission, option transitions and candidate authority."""

    def __init__(
        self,
        registry: OptionRegistry | None = None,
        *,
        lease: ExecutionLease | None = None,
        max_action_records: int = 4096,
        max_audit_records: int = 16384,
    ) -> None:
        _integer(max_action_records, "max_action_records", minimum=1)
        _integer(max_audit_records, "max_audit_records", minimum=1)
        self.registry = registry or OptionRegistry()
        self.lease = lease or ExecutionLease()
        self._max_action_records = max_action_records
        self._audit: deque[OptionAuditRecord] = deque(maxlen=max_audit_records)
        self._audit_sequence = 0
        self._audit_dropped_count = 0
        self._lock = threading.RLock()
        self._records: dict[str, _ActionRecord] = {}
        self._used_instance_ids: set[str] = set()
        self._active_action_id = ""
        self._last_now_ns = 0
        self._state = ExecutionState(
            active_option_instance_id="",
            phase=OptionPhase.IDLE,
            candidate_authorized=False,
            sequence=0,
            lease_generation=0,
            stage_id="",
            clock_epoch="",
            localization_epoch="",
            updated_at_ns=0,
            reason="startup",
        )

    @property
    def execution_state(self) -> ExecutionState:
        with self._lock:
            return self._state

    @property
    def journal(self) -> tuple[OptionAuditRecord, ...]:
        with self._lock:
            return tuple(self._audit)

    @property
    def audit_dropped_count(self) -> int:
        with self._lock:
            return self._audit_dropped_count

    def result(self, action_id: str) -> OptionResult | None:
        with self._lock:
            record = self._records.get(action_id)
            return None if record is None else record.result

    def submit(
        self, action_id: str, goal: OptionGoal, context: OptionContext
    ) -> ActionAdmission:
        _nonempty(action_id, "action_id")
        if not isinstance(goal, OptionGoal) or not isinstance(context, OptionContext):
            raise ValueError("goal and context must be validated option contracts")
        with self._lock:
            prior = self._records.get(action_id)
            if prior is not None:
                if prior.goal != goal:
                    return ActionAdmission(False, "action_id_reused_with_different_goal", self._state)
                return prior.admission
            self._observe_time(context.now_ns)
            if len(self._records) >= self._max_action_records:
                self._record_event(
                    "admission_rejected", action_id, goal, context.now_ns,
                    "action_record_capacity", None,
                )
                return ActionAdmission(False, "action_record_capacity", self._state)
            if self._active_action_id:
                admission = ActionAdmission(False, "another_option_is_active", self._state)
                self._record_event(
                    "admission_rejected", action_id, goal, context.now_ns,
                    admission.reason, None,
                )
                self._records[action_id] = _ActionRecord(goal, admission, None)
                return admission
            if goal.option_instance_id in self._used_instance_ids:
                admission = ActionAdmission(False, "option_instance_id_reused", self._state)
                self._record_event(
                    "admission_rejected", action_id, goal, context.now_ns,
                    admission.reason, None,
                )
                self._records[action_id] = _ActionRecord(goal, admission, None)
                return admission
            decision = self.registry.check_initiation(goal, context)
            if not decision.accepted:
                admission = ActionAdmission(False, decision.reason, self._state)
                self._record_event(
                    "admission_rejected", action_id, goal, context.now_ns,
                    decision.reason, None,
                )
                self._records[action_id] = _ActionRecord(goal, admission, None)
                return admission
            generation = self.lease.activate(
                goal.option_instance_id,
                candidate_authorized=False,
                expected_map_version=goal.map_version,
                expected_topology_version=goal.topology_version,
                expected_localization_epoch=goal.localization_epoch,
            )
            self._used_instance_ids.add(goal.option_instance_id)
            self._active_action_id = action_id
            instance = _OptionInstance(
                action_id=action_id,
                goal=goal,
                phase=OptionPhase.PLANNING,
                started_at_ns=context.now_ns,
                lease_generation=generation,
            )
            self._publish(instance, context, "goal_accepted")
            admission = ActionAdmission(True, "accepted", self._state)
            self._records[action_id] = _ActionRecord(goal, admission, instance)
            return admission

    def mark_executing(self, action_id: str, context: OptionContext) -> ExecutionState:
        with self._lock:
            self._observe_time(context.now_ns)
            instance = self._require_active(action_id)
            if instance.phase not in (OptionPhase.PLANNING, OptionPhase.REPLANNING):
                raise ValueError("option is not awaiting a validated plan")
            outcome, reason = self.registry.check_termination(
                instance.goal, context, executing=False
            )
            if outcome is not None:
                self._finish(instance, outcome, reason, context)
                return self._state
            decision = self.registry.check_invariant(instance.goal, context, executing=False)
            if not decision.accepted:
                self._finish_from_invariant(instance, decision.reason, context)
                return self._state
            if not context.path_valid:
                self._finish(
                    instance, OptionOutcome.FEASIBILITY_LOST, "path_invalid", context
                )
                return self._state
            candidate_authorized = instance.goal.kind != OptionKind.HOLD_SAFE
            self.lease.set_candidate_authorized(
                instance.goal.option_instance_id,
                instance.lease_generation,
                candidate_authorized,
            )
            instance = replace(instance, phase=OptionPhase.EXECUTING)
            self._replace_instance(instance)
            self._publish(instance, context, "plan_validated")
            return self._state

    def begin_replan(self, action_id: str, context: OptionContext) -> ExecutionState:
        with self._lock:
            self._observe_time(context.now_ns)
            instance = self._require_active(action_id)
            if instance.phase != OptionPhase.EXECUTING:
                raise ValueError("only an executing option can enter replanning")
            self.lease.set_candidate_authorized(
                instance.goal.option_instance_id, instance.lease_generation, False
            )
            generation = self.lease.advance_generation(
                instance.goal.option_instance_id, instance.lease_generation
            )
            instance = replace(
                instance, phase=OptionPhase.REPLANNING, lease_generation=generation
            )
            self._replace_instance(instance)
            self._publish(instance, context, "replanning")
            return self._state

    def request_cancel(
        self,
        action_id: str,
        context: OptionContext,
        *,
        cancel_timeout_ns: int,
        reason: str = "cancel_requested",
    ) -> ExecutionState:
        _integer(cancel_timeout_ns, "cancel_timeout_ns", minimum=1)
        _nonempty(reason, "reason")
        with self._lock:
            self._observe_time(context.now_ns)
            instance = self._require_active(action_id)
            if instance.phase == OptionPhase.CANCELING:
                return self._state
            self.lease.cancel(instance.goal.option_instance_id)
            instance = replace(
                instance,
                phase=OptionPhase.CANCELING,
                cancel_deadline_ns=context.now_ns + cancel_timeout_ns,
            )
            self._replace_instance(instance)
            self._publish(instance, context, reason)
            return self._state

    def acknowledge_cancel(self, action_id: str, context: OptionContext) -> OptionResult:
        with self._lock:
            self._observe_time(context.now_ns)
            instance = self._require_active(action_id)
            if instance.phase != OptionPhase.CANCELING:
                raise ValueError("option is not awaiting cancellation")
            result = self._finish(
                instance, OptionOutcome.CANCELED, "cancel_acknowledged", context
            )
            return result

    def tick(self, context: OptionContext) -> ExecutionState:
        if not isinstance(context, OptionContext):
            raise ValueError("context must be an OptionContext")
        with self._lock:
            self._observe_time(context.now_ns)
            if not self._active_action_id:
                return self._state
            instance = self._require_active(self._active_action_id)
            if instance.phase == OptionPhase.CANCELING:
                if context.now_ns >= instance.cancel_deadline_ns:
                    self._finish(
                        instance,
                        OptionOutcome.INTERNAL_ERROR,
                        "cancel_timeout_authority_revoked",
                        context,
                    )
                return self._state
            outcome, reason = self.registry.check_termination(
                instance.goal,
                context,
                executing=instance.phase == OptionPhase.EXECUTING,
            )
            if outcome is not None:
                self._finish(instance, outcome, reason, context)
                return self._state
            decision = self.registry.check_invariant(
                instance.goal,
                context,
                executing=instance.phase == OptionPhase.EXECUTING,
            )
            if not decision.accepted:
                self._finish_from_invariant(instance, decision.reason, context)
            return self._state

    def fail(
        self,
        action_id: str,
        context: OptionContext,
        *,
        outcome: OptionOutcome,
        reason: str,
    ) -> OptionResult:
        outcome = _enum(OptionOutcome, outcome, "outcome")
        if outcome not in (OptionOutcome.FEASIBILITY_LOST, OptionOutcome.INTERNAL_ERROR):
            raise ValueError("explicit failure must be FEASIBILITY_LOST or INTERNAL_ERROR")
        _nonempty(reason, "reason")
        with self._lock:
            self._observe_time(context.now_ns)
            instance = self._require_active(action_id)
            return self._finish(instance, outcome, reason, context)

    def _observe_time(self, now_ns: int) -> None:
        _integer(now_ns, "now_ns", minimum=0)
        if now_ns < self._last_now_ns:
            raise ValueError("option authority time must be monotonic")
        self._last_now_ns = now_ns

    def _require_active(self, action_id: str) -> _OptionInstance:
        if action_id != self._active_action_id:
            raise ValueError("action is not the active option")
        record = self._records.get(action_id)
        if record is None or record.instance is None:
            raise ValueError("active option record is missing")
        return record.instance

    def _replace_instance(self, instance: _OptionInstance) -> None:
        self._records[instance.action_id].instance = instance

    def _publish(self, instance: _OptionInstance, context: OptionContext, reason: str) -> None:
        candidate_authorized = (
            instance.phase == OptionPhase.EXECUTING
            and instance.goal.kind != OptionKind.HOLD_SAFE
            and context.motion_authorized
            and context.lease_valid
            and not context.safety_stop
        )
        if not candidate_authorized and instance.phase == OptionPhase.EXECUTING:
            self.lease.set_candidate_authorized(
                instance.goal.option_instance_id, instance.lease_generation, False
            )
        self._state = ExecutionState(
            active_option_instance_id=instance.goal.option_instance_id,
            phase=instance.phase,
            candidate_authorized=candidate_authorized,
            sequence=self._state.sequence + 1,
            lease_generation=instance.lease_generation,
            stage_id=context.stage_id,
            clock_epoch=context.clock_epoch,
            localization_epoch=context.localization_epoch,
            updated_at_ns=context.now_ns,
            reason=reason,
        )
        self._record_event(
            "state_transition", instance.action_id, instance.goal, context.now_ns,
            reason, instance,
        )

    def _finish_from_invariant(
        self, instance: _OptionInstance, reason: str, context: OptionContext
    ) -> OptionResult:
        if reason == "safety_stop_active":
            outcome = OptionOutcome.SAFETY_STOP
        elif reason in ("stage_mismatch", "stage_not_active"):
            outcome = OptionOutcome.STAGE_ENDED
        else:
            outcome = OptionOutcome.FEASIBILITY_LOST
        return self._finish(instance, outcome, reason, context)

    def _finish(
        self,
        instance: _OptionInstance,
        outcome: OptionOutcome,
        reason: str,
        context: OptionContext,
    ) -> OptionResult:
        self.lease.cancel(instance.goal.option_instance_id)
        outcome = _enum(OptionOutcome, outcome, "outcome")
        now_ns = context.now_ns
        if now_ns < instance.started_at_ns:
            raise ValueError("finish time precedes option start")
        record = self._records[instance.action_id]
        if record.result is not None:
            return record.result
        result = OptionResult(
            action_id=instance.action_id,
            option_instance_id=instance.goal.option_instance_id,
            outcome=outcome,
            reason=reason,
            started_at_ns=instance.started_at_ns,
            finished_at_ns=now_ns,
        )
        record.result = result
        record.instance = replace(instance, phase=OptionPhase.FINISHED)
        if self._active_action_id == instance.action_id:
            self._active_action_id = ""
        self._state = ExecutionState(
            active_option_instance_id=instance.goal.option_instance_id,
            phase=OptionPhase.FINISHED,
            candidate_authorized=False,
            sequence=self._state.sequence + 1,
            lease_generation=instance.lease_generation,
            stage_id=context.stage_id,
            clock_epoch=context.clock_epoch,
            localization_epoch=context.localization_epoch,
            updated_at_ns=now_ns,
            reason=reason,
        )
        self._record_event(
            "terminal_result", instance.action_id, instance.goal, now_ns,
            reason, replace(instance, phase=OptionPhase.FINISHED),
        )
        return result

    def _record_event(
        self,
        event: str,
        action_id: str,
        goal: OptionGoal,
        time_ns: int,
        reason: str,
        instance: _OptionInstance | None,
    ) -> None:
        if len(self._audit) == self._audit.maxlen:
            self._audit_dropped_count += 1
        self._audit_sequence += 1
        self._audit.append(
            OptionAuditRecord(
                sequence=self._audit_sequence,
                time_ns=time_ns,
                event=event,
                action_id=action_id,
                option_instance_id=goal.option_instance_id,
                phase=self._state.phase if instance is None else instance.phase,
                candidate_authorized=(
                    instance is not None
                    and self._state.candidate_authorized
                    and self._state.active_option_instance_id == goal.option_instance_id
                ),
                lease_generation=0 if instance is None else instance.lease_generation,
                reason=reason,
            )
        )


def _default_definitions() -> tuple[OptionDefinition, ...]:
    guardian = (Role.GUARDIAN,)
    explorer = (Role.EXPLORER,)
    both = (Role.EXPLORER, Role.GUARDIAN)
    return (
        OptionDefinition(OptionKind.HOLD_SAFE, both, False, True, False),
        OptionDefinition(OptionKind.SEARCH_PORTAL, guardian, True, False, False),
        OptionDefinition(OptionKind.INTERCEPT_PORTAL, guardian, True, False, False),
        OptionDefinition(OptionKind.PRESSURE_ROUTE, guardian, True, False, False),
        OptionDefinition(OptionKind.APPROACH_CAPTURE, guardian, True, False, False),
        OptionDefinition(OptionKind.RECOVER_VIEW, guardian, True, False, False),
        OptionDefinition(OptionKind.FALLBACK_DEFEND_BASE, guardian, True, False, True),
        OptionDefinition(OptionKind.ADVANCE_BASE, explorer, True, False, True),
        OptionDefinition(OptionKind.BREAK_LOS, explorer, True, False, False),
        OptionDefinition(OptionKind.TAKE_ALTERNATE_PORTAL, explorer, True, False, False),
        OptionDefinition(OptionKind.KEEP_ESCAPE_ROUTE, explorer, True, False, False),
        OptionDefinition(OptionKind.OBSERVE_SAFE, explorer, False, False, False),
    )


def _enum(enum_type, value, field_name: str):
    if isinstance(value, enum_type):
        return value
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"{field_name} must be a {enum_type.__name__}")
    try:
        return enum_type(value)
    except ValueError as error:
        raise ValueError(f"{field_name} must be a {enum_type.__name__}") from error


def _integer(value: int, field_name: str, *, minimum: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < minimum:
        raise ValueError(f"{field_name} must be an integer >= {minimum}")


def _nonempty(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string")


def _nonnegative_finite(value: float, field_name: str) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0.0
    ):
        raise ValueError(f"{field_name} must be finite and non-negative")
