"""Pure stage lifecycle, semantic-zone and event-adjudication contracts."""

from dataclasses import dataclass
from collections import deque
from enum import IntEnum
import re
import threading
import uuid
import math

from .contracts import ContractHeader, validate_polygon


class StagePhase(IntEnum):
    INIT = 0
    FREEZE = 1
    ACTIVE = 2
    TERMINAL = 3


class Role(IntEnum):
    EXPLORER = 0
    GUARDIAN = 1


class TerminalKind(IntEnum):
    NONE = 0
    CAPTURE = 1
    ARRIVAL = 2
    TIMEOUT = 3
    OFFICIAL_ABORT = 4
    AMBIGUOUS = 5


class EventEvidence(IntEnum):
    ESTIMATE = 0
    SIM_TRUTH = 1
    OFFICIAL = 2


class EventPredicate(IntEnum):
    UNKNOWN = 0
    FALSE = 1
    TRUE = 2


class ZonePurpose(IntEnum):
    OWN_START = 0
    TARGET = 1


class ZoneStatus(IntEnum):
    UNRESOLVED = 0
    CANDIDATE = 1
    ACCEPTED = 2
    INVALID = 3


class EventResolution(IntEnum):
    DISMISS_ESTIMATE = 0
    CONFIRM_OFFICIAL = 1
    SET_OFFICIAL_TERMINAL = 2


_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_RESET_ACKS = frozenset(("actions", "execution", "plans", "candidates", "tracks", "memory"))
_EPOCH_ACKS = frozenset(("actions", "plans", "candidates", "tracks", "map", "world"))


def _enum_value(enum_type, value, name: str):
    if isinstance(value, enum_type):
        return value
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"{name} must be a {enum_type.__name__}")
    try:
        return enum_type(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a {enum_type.__name__}") from error


def _nonempty(value: str, name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")


def _nonnegative_int(value: int, name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")


def _sha256(value: str, name: str) -> None:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        raise ValueError(f"{name} must be a lowercase SHA-256 hex digest")


@dataclass(frozen=True)
class StageProfile:
    profile_id: str
    score_profile_id: str
    freeze_duration_ns: int
    stage_duration_ns: int
    lease_duration_ns: int
    zone_update_window_ns: int
    max_request_records: int
    max_event_records: int
    max_audit_records: int
    config_hash: str
    source_id: str
    source_session: str
    clock_epoch: str
    localization_epoch: str
    organizer_authorization_ref: str
    approved_authorization_refs: tuple[str, ...]
    approved_zone_frames: tuple[str, ...]
    approved_memory_profile_ids: tuple[str, ...] = ()
    allow_sim_truth_terminal: bool = False

    def __post_init__(self) -> None:
        for value, name in (
            (self.profile_id, "profile_id"),
            (self.score_profile_id, "score_profile_id"),
            (self.source_id, "source_id"),
            (self.source_session, "source_session"),
            (self.clock_epoch, "clock_epoch"),
            (self.localization_epoch, "localization_epoch"),
            (self.organizer_authorization_ref, "organizer_authorization_ref"),
        ):
            _nonempty(value, name)
        _sha256(self.config_hash, "config_hash")
        for value, name in (
            (self.freeze_duration_ns, "freeze_duration_ns"),
            (self.stage_duration_ns, "stage_duration_ns"),
            (self.lease_duration_ns, "lease_duration_ns"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.freeze_duration_ns >= self.stage_duration_ns:
            raise ValueError("freeze duration must be shorter than stage duration")
        if (
            not isinstance(self.zone_update_window_ns, int)
            or isinstance(self.zone_update_window_ns, bool)
            or not 0 <= self.zone_update_window_ns <= self.freeze_duration_ns
        ):
            raise ValueError("zone_update_window_ns must be within [0, freeze_duration_ns]")
        for value, name in (
            (self.max_request_records, "max_request_records"),
            (self.max_event_records, "max_event_records"),
            (self.max_audit_records, "max_audit_records"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if not isinstance(self.approved_authorization_refs, (tuple, list)):
            raise ValueError("approved_authorization_refs must be a sequence")
        refs = tuple(self.approved_authorization_refs)
        if not refs or any(not isinstance(ref, str) or not ref.strip() for ref in refs):
            raise ValueError("approved_authorization_refs must contain non-empty refs")
        if len(set(refs)) != len(refs):
            raise ValueError("approved_authorization_refs must be unique")
        if self.organizer_authorization_ref not in refs:
            raise ValueError("organizer authorization must be in the approved refs")
        if not isinstance(self.approved_zone_frames, (tuple, list)):
            raise ValueError("approved_zone_frames must be a sequence")
        frames = tuple(self.approved_zone_frames)
        if not frames or any(not isinstance(frame, str) or not frame.strip() for frame in frames):
            raise ValueError("approved_zone_frames must contain non-empty frame IDs")
        if len(set(frames)) != len(frames):
            raise ValueError("approved_zone_frames must be unique")
        if not isinstance(self.approved_memory_profile_ids, (tuple, list)):
            raise ValueError("approved_memory_profile_ids must be a sequence")
        memory_ids = tuple(self.approved_memory_profile_ids)
        if any(not isinstance(item, str) or not item.strip() for item in memory_ids):
            raise ValueError("approved memory profile IDs must be non-empty")
        if len(set(memory_ids)) != len(memory_ids):
            raise ValueError("approved memory profile IDs must be unique")
        if not isinstance(self.allow_sim_truth_terminal, bool):
            raise ValueError("allow_sim_truth_terminal must be boolean")
        object.__setattr__(self, "approved_authorization_refs", refs)
        object.__setattr__(self, "approved_zone_frames", frames)
        object.__setattr__(self, "approved_memory_profile_ids", memory_ids)


@dataclass(frozen=True)
class GoalZone:
    zone_id: str
    purpose: ZonePurpose
    status: ZoneStatus
    boundary: tuple[tuple[float, float], ...] = ()
    frame_id: str = ""
    position_error_bound_m: float = 0.0
    provider: str = ""
    approval_ref: str = ""
    provenance_hash: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "purpose", _enum_value(ZonePurpose, self.purpose, "purpose"))
        object.__setattr__(self, "status", _enum_value(ZoneStatus, self.status, "status"))
        if (
            isinstance(self.position_error_bound_m, bool)
            or not isinstance(self.position_error_bound_m, (int, float))
            or not math.isfinite(self.position_error_bound_m)
            or self.position_error_bound_m < 0.0
        ):
            raise ValueError("position_error_bound_m must be finite and non-negative")
        if not isinstance(self.boundary, (tuple, list)):
            raise ValueError("boundary must be a sequence of points")
        if any(not isinstance(point, (tuple, list)) or len(point) != 2 for point in self.boundary):
            raise ValueError("boundary points must contain exactly two coordinates")
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for point in self.boundary
            for value in point
        ):
            raise ValueError("boundary coordinates must be numeric")
        boundary = tuple(tuple(point) for point in self.boundary)
        object.__setattr__(self, "boundary", boundary)
        if self.status == ZoneStatus.UNRESOLVED:
            if (
                self.zone_id or self.boundary or self.frame_id
                or self.position_error_bound_m != 0.0 or self.provider
                or self.approval_ref or self.provenance_hash
            ):
                raise ValueError("unresolved zone must not carry guessed geometry or provenance")
            return
        if self.status == ZoneStatus.INVALID:
            if self.boundary:
                raise ValueError("invalid zone must not expose an accepted boundary")
            return
        _nonempty(self.zone_id, "zone_id")
        _nonempty(self.frame_id, "frame_id")
        boundary = validate_polygon(self.boundary)
        if self.status == ZoneStatus.ACCEPTED:
            _nonempty(self.provider, "provider")
            _nonempty(self.approval_ref, "approval_ref")
            _sha256(self.provenance_hash, "provenance_hash")
        else:
            if self.provider and not self.provider.strip():
                raise ValueError("provider must be non-empty when supplied")
            if self.approval_ref and not self.approval_ref.strip():
                raise ValueError("approval_ref must be non-empty when supplied")
            if self.provenance_hash:
                _sha256(self.provenance_hash, "provenance_hash")
        object.__setattr__(self, "boundary", boundary)


@dataclass(frozen=True)
class RuleEvent:
    event_id: str
    stage_id: str
    clock_epoch: str
    localization_epoch: str
    kind: TerminalKind
    evidence: EventEvidence
    predicate: EventPredicate
    event_time_lower_ns: int
    event_time_upper_ns: int
    input_ids: tuple[str, ...]
    authorization_ref: str = ""
    zone_id: str = ""
    reason: str = ""

    def __post_init__(self) -> None:
        _nonempty(self.event_id, "event_id")
        _nonempty(self.stage_id, "stage_id")
        _nonempty(self.clock_epoch, "clock_epoch")
        _nonempty(self.localization_epoch, "localization_epoch")
        object.__setattr__(self, "kind", _enum_value(TerminalKind, self.kind, "kind"))
        object.__setattr__(self, "evidence", _enum_value(EventEvidence, self.evidence, "evidence"))
        object.__setattr__(self, "predicate", _enum_value(EventPredicate, self.predicate, "predicate"))
        if not isinstance(self.input_ids, (tuple, list)):
            raise ValueError("input_ids must be a sequence")
        input_ids = tuple(self.input_ids)
        object.__setattr__(self, "input_ids", input_ids)
        _nonnegative_int(self.event_time_lower_ns, "event_time_lower_ns")
        _nonnegative_int(self.event_time_upper_ns, "event_time_upper_ns")
        if self.event_time_lower_ns > self.event_time_upper_ns:
            raise ValueError("event interval lower bound must not exceed upper bound")
        if self.kind in (TerminalKind.NONE, TerminalKind.TIMEOUT, TerminalKind.AMBIGUOUS):
            raise ValueError("input event kind must be CAPTURE, ARRIVAL or OFFICIAL_ABORT")
        if not self.input_ids or any(not isinstance(value, str) or not value.strip() for value in self.input_ids):
            raise ValueError("event input_ids must contain non-empty provenance IDs")
        if len(set(self.input_ids)) != len(self.input_ids):
            raise ValueError("event input_ids must be unique")
        for value, name in (
            (self.authorization_ref, "authorization_ref"),
            (self.zone_id, "zone_id"),
            (self.reason, "reason"),
        ):
            if not isinstance(value, str):
                raise ValueError(f"{name} must be a string")


@dataclass(frozen=True)
class MatchState:
    meta: ContractHeader
    stage_number: int
    role: Role
    phase: StagePhase
    stage_started_at_ns: int
    freeze_ends_at_ns: int
    stage_ends_at_ns: int
    motion_authorized: bool
    event_hold: bool
    terminal_kind: TerminalKind
    terminal_evidence: EventEvidence
    score_profile_id: str
    own_start_zone_id: str
    target_zone_id: str
    config_hash: str

    def __post_init__(self) -> None:
        _nonnegative_int(self.stage_number, "stage_number")
        if self.stage_number > 255:
            raise ValueError("stage_number must be in [0, 255]")
        object.__setattr__(self, "role", _enum_value(Role, self.role, "role"))
        object.__setattr__(self, "phase", _enum_value(StagePhase, self.phase, "phase"))
        object.__setattr__(
            self,
            "terminal_kind",
            _enum_value(TerminalKind, self.terminal_kind, "terminal_kind"),
        )
        object.__setattr__(
            self,
            "terminal_evidence",
            _enum_value(EventEvidence, self.terminal_evidence, "terminal_evidence"),
        )
        for value, name in (
            (self.stage_started_at_ns, "stage_started_at_ns"),
            (self.freeze_ends_at_ns, "freeze_ends_at_ns"),
            (self.stage_ends_at_ns, "stage_ends_at_ns"),
        ):
            _nonnegative_int(value, name)
        if not isinstance(self.motion_authorized, bool) or not isinstance(self.event_hold, bool):
            raise ValueError("motion_authorized and event_hold must be booleans")
        if not isinstance(self.meta, ContractHeader):
            raise ValueError("MatchState requires a validated ContractHeader")
        if not self.meta.stage_id:
            raise ValueError("MatchState requires a stage ID")
        if self.phase == StagePhase.INIT:
            if (self.stage_started_at_ns, self.freeze_ends_at_ns, self.stage_ends_at_ns) != (0, 0, 0):
                raise ValueError("INIT MatchState must not carry active stage times")
        elif not (
            self.stage_started_at_ns < self.freeze_ends_at_ns < self.stage_ends_at_ns
        ):
            raise ValueError("stage, freeze and deadline times must be strictly ordered")
        if self.phase == StagePhase.TERMINAL:
            if self.terminal_kind == TerminalKind.NONE:
                raise ValueError("TERMINAL MatchState requires a terminal kind")
            if self.event_hold and self.terminal_kind != TerminalKind.AMBIGUOUS:
                raise ValueError("terminal event_hold is reserved for AMBIGUOUS results")
            if self.terminal_kind == TerminalKind.AMBIGUOUS and not self.event_hold:
                raise ValueError("AMBIGUOUS terminal result requires event_hold")
        elif self.terminal_kind != TerminalKind.NONE:
            raise ValueError("nonterminal MatchState must not carry a terminal result")
        if self.event_hold and not (
            self.phase == StagePhase.ACTIVE
            or (self.phase == StagePhase.TERMINAL and self.terminal_kind == TerminalKind.AMBIGUOUS)
        ):
            raise ValueError("event_hold is only valid in ACTIVE or ambiguous TERMINAL")
        if self.motion_authorized and (
            self.phase != StagePhase.ACTIVE or self.event_hold or self.meta.validity != 1
        ):
            raise ValueError("motion authority requires a valid ACTIVE state without event hold")
        _nonempty(self.score_profile_id, "score_profile_id")
        _sha256(self.config_hash, "config_hash")
        if not isinstance(self.own_start_zone_id, str) or not isinstance(self.target_zone_id, str):
            raise ValueError("zone IDs must be strings")


@dataclass(frozen=True)
class RequestResult:
    accepted: bool
    reason: str
    stage_id: str
    reset_effects_required: frozenset[str] = frozenset()
    memory_profile_id: str = ""


@dataclass(frozen=True)
class StageAuditRecord:
    sequence: int
    time_ns: int
    operation: str
    request_id: str
    stage_id: str
    prior_stage_id: str
    phase: StagePhase
    accepted: bool
    reason: str
    evidence: EventEvidence | None = None
    event_kind: TerminalKind | None = None
    event_id: str = ""
    event_time_lower_ns: int = 0
    event_time_upper_ns: int = 0
    input_ids: tuple[str, ...] = ()
    authorization_ref: str = ""
    provenance_hash: str = ""
    profile_id: str = ""
    config_hash: str = ""
    stage_number: int = 0
    role: Role = Role.EXPLORER
    stage_started_at_ns: int = 0
    freeze_ends_at_ns: int = 0
    stage_ends_at_ns: int = 0
    memory_profile_id: str = ""


@dataclass
class _OperationRecord:
    request: object
    result: RequestResult


class StageManager:
    """Single-owner pure-core state machine; ROS callbacks adapt at the edge."""

    def __init__(
        self,
        profile: StageProfile,
        *,
        initial_stage_id: str,
        stage_number: int,
        role: Role,
        stage_id_factory=None,
    ) -> None:
        _nonempty(initial_stage_id, "initial_stage_id")
        if not isinstance(profile, StageProfile):
            raise ValueError("profile must be a validated StageProfile")
        self._profile = profile
        self._stage_id = initial_stage_id
        self._stage_number = self._validate_stage_number(stage_number)
        self._role = _enum_value(Role, role, "role")
        self._stage_id_factory = (
            stage_id_factory if stage_id_factory is not None else lambda: str(uuid.uuid4())
        )
        if not callable(self._stage_id_factory):
            raise ValueError("stage_id_factory must be callable")
        self._lock = threading.RLock()
        self._phase = StagePhase.INIT
        self._stage_started_at_ns = 0
        self._freeze_ends_at_ns = 0
        self._stage_ends_at_ns = 0
        self._event_hold = False
        self._held_event: RuleEvent | None = None
        self._pending_estimates: dict[str, RuleEvent] = {}
        self._terminal_kind = TerminalKind.NONE
        self._terminal_evidence = EventEvidence.ESTIMATE
        self._official_events: list[RuleEvent] = []
        self._zones: dict[ZonePurpose, GoalZone] = {}
        self._last_now_ns = 0
        self._seq = 0
        self._reset_barrier_pending = False
        self._reset_acks: set[str] = set()
        self._epoch_barrier_pending = False
        self._epoch_acks: set[str] = set()
        self._operations: dict[tuple[str, str], _OperationRecord] = {}
        self._event_ids: dict[str, RuleEvent] = {}
        self._used_stage_ids = {initial_stage_id}
        self._audit: deque[StageAuditRecord] = deque(maxlen=profile.max_audit_records)
        self._audit_seq = 0
        self._audit_dropped_count = 0
        self._authorization_refs = frozenset(profile.approved_authorization_refs)
        self._approved_zone_frames = frozenset(profile.approved_zone_frames)

    @staticmethod
    def _validate_stage_number(value: int) -> int:
        if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 255:
            raise ValueError("stage_number must be an integer in [0, 255]")
        return value

    @property
    def stage_id(self) -> str:
        with self._lock:
            return self._stage_id

    @property
    def profile(self) -> StageProfile:
        return self._profile

    @property
    def phase(self) -> StagePhase:
        with self._lock:
            return self._phase

    @property
    def journal(self) -> tuple[StageAuditRecord, ...]:
        with self._lock:
            return tuple(self._audit)

    @property
    def audit_dropped_count(self) -> int:
        with self._lock:
            return self._audit_dropped_count

    def _record(
        self,
        operation: str,
        request_id: str,
        result: RequestResult,
        *,
        evidence: EventEvidence | None = None,
        event_kind: TerminalKind | None = None,
        event_id: str = "",
        event_time_lower_ns: int = 0,
        event_time_upper_ns: int = 0,
        input_ids: tuple[str, ...] = (),
        authorization_ref: str = "",
        provenance_hash: str = "",
        prior_stage_id: str = "",
    ) -> None:
        if len(self._audit) == self._audit.maxlen:
            self._audit_dropped_count += 1
        self._audit_seq += 1
        self._audit.append(
            StageAuditRecord(
                sequence=self._audit_seq,
                time_ns=self._last_now_ns,
                operation=operation,
                request_id=request_id,
                stage_id=result.stage_id,
                prior_stage_id=prior_stage_id,
                phase=self._phase,
                accepted=result.accepted,
                reason=result.reason,
                evidence=evidence,
                event_kind=event_kind,
                event_id=event_id,
                event_time_lower_ns=event_time_lower_ns,
                event_time_upper_ns=event_time_upper_ns,
                input_ids=tuple(input_ids),
                authorization_ref=authorization_ref,
                provenance_hash=provenance_hash,
                profile_id=self.profile.profile_id,
                config_hash=self.profile.config_hash,
                stage_number=self._stage_number,
                role=self._role,
                stage_started_at_ns=self._stage_started_at_ns,
                freeze_ends_at_ns=self._freeze_ends_at_ns,
                stage_ends_at_ns=self._stage_ends_at_ns,
                memory_profile_id=result.memory_profile_id,
            )
        )

    def _check_now(self, now_ns: int) -> None:
        _nonnegative_int(now_ns, "now_ns")
        if now_ns < self._last_now_ns:
            raise ValueError("stage manager time must be monotonic")

    def _advance_time(self, now_ns: int) -> None:
        self._check_now(now_ns)
        self._last_now_ns = now_ns
        if self._phase == StagePhase.FREEZE:
            if now_ns >= self._stage_ends_at_ns:
                self._set_timeout()
            elif now_ns >= self._freeze_ends_at_ns:
                self._phase = StagePhase.ACTIVE
                self._record(
                    "phase_transition",
                    "",
                    RequestResult(True, "freeze_elapsed", self._stage_id),
                )
        elif self._phase == StagePhase.ACTIVE and now_ns >= self._stage_ends_at_ns:
            self._set_timeout()

    def _set_timeout(self) -> None:
        if self._phase == StagePhase.TERMINAL:
            return
        self._phase = StagePhase.TERMINAL
        self._event_hold = False
        self._held_event = None
        self._terminal_kind = TerminalKind.TIMEOUT
        self._terminal_evidence = EventEvidence.ESTIMATE
        self._record(
            "deadline_transition",
            "",
            RequestResult(True, "timeout_deadline_reached", self._stage_id),
            evidence=EventEvidence.ESTIMATE,
            event_kind=TerminalKind.TIMEOUT,
            event_id=f"timeout:{self._stage_id}",
            event_time_lower_ns=self._stage_ends_at_ns,
            event_time_upper_ns=self._stage_ends_at_ns,
        )

    def _authorized(self, reference: str) -> bool:
        return isinstance(reference, str) and reference == self.profile.organizer_authorization_ref

    def _approved_reference(self, reference: str) -> bool:
        return isinstance(reference, str) and reference in self._authorization_refs

    def _cached(
        self, operation: str, request_id: str, request: object
    ) -> RequestResult | None:
        _nonempty(request_id, "request_id")
        record = self._operations.get((operation, request_id))
        if record is None:
            if len(self._operations) >= self.profile.max_request_records:
                result = RequestResult(False, "request_capacity_exhausted", self._stage_id)
                self._record(operation, request_id, result)
                return result
            return None
        if record.request != request:
            result = RequestResult(False, "request_id_reused_with_different_payload", self._stage_id)
            self._record(operation, request_id, result)
            return result
        return record.result

    def _remember(
        self, operation: str, request_id: str, request: object, result: RequestResult
    ) -> RequestResult:
        self._operations[(operation, request_id)] = _OperationRecord(request, result)
        authorization_ref = ""
        provenance_hash = ""
        if operation == "zone":
            authorization_ref = request[2]
            if isinstance(request[1], GoalZone):
                provenance_hash = request[1].provenance_hash
        elif operation in ("start", "reset"):
            authorization_ref = request[-1]
        elif operation == "resolve":
            authorization_ref = request[-1]
        event_id = request[1] if operation == "resolve" else ""
        event_kind = request[3] if operation == "resolve" else None
        evidence = (
            EventEvidence.OFFICIAL
            if operation == "resolve"
            and request[2] in (EventResolution.CONFIRM_OFFICIAL, EventResolution.SET_OFFICIAL_TERMINAL)
            else None
        )
        self._record(
            operation,
            request_id,
            result,
            evidence=evidence,
            event_kind=event_kind,
            event_id=event_id,
            authorization_ref=authorization_ref,
            provenance_hash=provenance_hash,
            prior_stage_id=request[0] if operation == "reset" else "",
        )
        return result

    def start_stage(
        self,
        *,
        request_id: str,
        expected_stage_id: str,
        official_start_stamp_ns: int,
        organizer_authorization_ref: str,
        now_ns: int,
    ) -> RequestResult:
        request = (expected_stage_id, official_start_stamp_ns, organizer_authorization_ref)
        with self._lock:
            self._advance_time(now_ns)
            cached = self._cached("start", request_id, request)
            if cached is not None:
                return cached
            if expected_stage_id != self._stage_id:
                result = RequestResult(False, "stale_expected_stage_id", self._stage_id)
            elif self._phase != StagePhase.INIT:
                result = RequestResult(False, "stage_not_in_INIT", self._stage_id)
            elif self._reset_barrier_pending or self._epoch_barrier_pending:
                result = RequestResult(False, "reset_effects_not_acknowledged", self._stage_id)
            elif not self._authorized(organizer_authorization_ref):
                result = RequestResult(False, "organizer_authorization_not_approved", self._stage_id)
            elif (
                not isinstance(official_start_stamp_ns, int)
                or isinstance(official_start_stamp_ns, bool)
                or official_start_stamp_ns < 0
                or official_start_stamp_ns > now_ns
            ):
                result = RequestResult(False, "official_start_stamp_invalid", self._stage_id)
            else:
                end = official_start_stamp_ns + self.profile.stage_duration_ns
                freeze_end = official_start_stamp_ns + self.profile.freeze_duration_ns
                if end > (1 << 63) - 1 or freeze_end > (1 << 63) - 1:
                    result = RequestResult(False, "stage_deadline_overflow", self._stage_id)
                else:
                    self._stage_started_at_ns = official_start_stamp_ns
                    self._freeze_ends_at_ns = freeze_end
                    self._stage_ends_at_ns = end
                    self._phase = StagePhase.FREEZE
                    self._terminal_kind = TerminalKind.NONE
                    self._terminal_evidence = EventEvidence.ESTIMATE
                    self._event_hold = False
                    self._held_event = None
                    self._pending_estimates.clear()
                    self._official_events.clear()
                    self._event_ids.clear()
                    result = RequestResult(True, "stage_start_latched", self._stage_id)
            remembered = self._remember("start", request_id, request, result)
            if result.accepted:
                self._advance_time(now_ns)
            return remembered

    def set_goal_zone(
        self,
        *,
        request_id: str,
        expected_stage_id: str,
        zone: GoalZone,
        organizer_authorization_ref: str,
        now_ns: int,
    ) -> RequestResult:
        request = (expected_stage_id, zone, organizer_authorization_ref)
        with self._lock:
            self._advance_time(now_ns)
            cached = self._cached("zone", request_id, request)
            if cached is not None:
                return cached
            if expected_stage_id != self._stage_id:
                result = RequestResult(False, "stale_expected_stage_id", self._stage_id)
            elif not isinstance(zone, GoalZone):
                result = RequestResult(False, "zone_payload_invalid", self._stage_id)
            elif self._reset_barrier_pending or self._epoch_barrier_pending:
                result = RequestResult(False, "reset_effects_not_acknowledged", self._stage_id)
            elif self._phase not in (StagePhase.INIT, StagePhase.FREEZE):
                result = RequestResult(False, "zone_updates_closed_after_FREEZE", self._stage_id)
            elif (
                self._phase == StagePhase.FREEZE
                and now_ns > self._stage_started_at_ns + self.profile.zone_update_window_ns
            ):
                result = RequestResult(False, "zone_permission_window_closed", self._stage_id)
            elif not self._authorized(organizer_authorization_ref):
                result = RequestResult(False, "organizer_authorization_not_approved", self._stage_id)
            elif zone.status == ZoneStatus.ACCEPTED and not self._approved_reference(zone.approval_ref):
                result = RequestResult(False, "zone_approval_ref_not_approved", self._stage_id)
            elif zone.status == ZoneStatus.ACCEPTED and zone.frame_id not in self._approved_zone_frames:
                result = RequestResult(False, "zone_frame_not_approved", self._stage_id)
            else:
                self._zones[zone.purpose] = zone
                result = RequestResult(True, "zone_recorded", self._stage_id)
            return self._remember("zone", request_id, request, result)

    def _zones_in_state(self) -> tuple[str, str]:
        own = self._zones.get(ZonePurpose.OWN_START)
        target = self._zones.get(ZonePurpose.TARGET)
        return (
            own.zone_id if own is not None and own.status == ZoneStatus.ACCEPTED else "",
            target.zone_id if target is not None and target.status == ZoneStatus.ACCEPTED else "",
        )

    def goal_zone(self, purpose: ZonePurpose) -> GoalZone:
        purpose = _enum_value(ZonePurpose, purpose, "purpose")
        with self._lock:
            return self._zones.get(
                purpose,
                GoalZone("", purpose, ZoneStatus.UNRESOLVED),
            )

    def _terminalize_official_events(self) -> None:
        events = sorted(
            self._official_events,
            key=lambda event: (event.event_time_lower_ns, event.event_time_upper_ns, event.event_id),
        )
        comparisons = sorted(
            (*self._official_events, *self._pending_estimates.values()),
            key=lambda event: (event.event_time_lower_ns, event.event_time_upper_ns, event.event_id),
        )
        for index, first in enumerate(comparisons):
            for second in comparisons[index + 1 :]:
                if first.kind == second.kind:
                    continue
                if first.event_time_upper_ns >= second.event_time_lower_ns:
                    if events:
                        self._phase = StagePhase.TERMINAL
                        self._event_hold = True
                        self._terminal_kind = TerminalKind.AMBIGUOUS
                        self._terminal_evidence = max(
                            (event.evidence for event in (first, second)),
                            key=int,
                        )
                    elif self._phase == StagePhase.ACTIVE:
                        self._event_hold = True
                        self._terminal_kind = TerminalKind.NONE
                    self._held_event = None
                    return
        if events:
            earliest = events[0]
            self._phase = StagePhase.TERMINAL
            self._event_hold = False
            self._held_event = None
            self._terminal_kind = earliest.kind
            self._terminal_evidence = max((event.evidence for event in events), key=int)
        elif self._phase != StagePhase.TERMINAL:
            self._event_hold = bool(self._pending_estimates)
            self._held_event = (
                next(iter(self._pending_estimates.values()))
                if len(self._pending_estimates) == 1
                else None
            )
            self._terminal_kind = TerminalKind.NONE

    def resolve_event(self, event: RuleEvent, *, now_ns: int) -> RequestResult:
        if not isinstance(event, RuleEvent):
            raise ValueError("event must be a validated RuleEvent")
        with self._lock:
            self._check_now(now_ns)
            prior_event = self._event_ids.get(event.event_id)
            if prior_event is not None:
                self._advance_time(now_ns)
                if prior_event == event:
                    return RequestResult(True, "duplicate_event_idempotent", self._stage_id)
                result = RequestResult(False, "event_id_reused_with_different_payload", self._stage_id)
                self._record("rule_event", event.event_id, result, event_id=event.event_id)
                return result
            if len(self._event_ids) >= self.profile.max_event_records:
                result = RequestResult(False, "event_capacity_exhausted", self._stage_id)
                self._record(
                    "rule_event",
                    event.event_id,
                    result,
                    evidence=event.evidence,
                    event_kind=event.kind,
                    event_id=event.event_id,
                    event_time_lower_ns=event.event_time_lower_ns,
                    event_time_upper_ns=event.event_time_upper_ns,
                    input_ids=event.input_ids,
                    authorization_ref=event.authorization_ref,
                )
                return result
            result = self._resolve_event_checked(event, now_ns)
            self._advance_time(now_ns)
            if result.accepted:
                self._event_ids[event.event_id] = event
            self._record(
                "rule_event",
                event.event_id,
                result,
                evidence=event.evidence,
                event_kind=event.kind,
                event_id=event.event_id,
                event_time_lower_ns=event.event_time_lower_ns,
                event_time_upper_ns=event.event_time_upper_ns,
                input_ids=event.input_ids,
                authorization_ref=event.authorization_ref,
            )
            return result

    def _resolve_event_checked(self, event: RuleEvent, now_ns: int) -> RequestResult:
        if event.stage_id != self._stage_id:
            return RequestResult(False, "stale_event_stage_id", self._stage_id)
        if event.clock_epoch != self.profile.clock_epoch:
            return RequestResult(False, "stale_event_clock_epoch", self._stage_id)
        if event.localization_epoch != self.profile.localization_epoch:
            return RequestResult(False, "stale_event_localization_epoch", self._stage_id)
        if event.predicate != EventPredicate.TRUE:
            return RequestResult(False, "event_predicate_not_proven_true", self._stage_id)
        if self._phase == StagePhase.INIT:
            return RequestResult(False, "event_outside_started_stage", self._stage_id)
        if event.event_time_lower_ns < self._freeze_ends_at_ns:
            return RequestResult(False, "event_precedes_active_phase", self._stage_id)
        if event.event_time_lower_ns > now_ns:
            return RequestResult(False, "event_time_in_future", self._stage_id)
        if event.event_time_upper_ns > now_ns:
            return RequestResult(False, "event_interval_extends_into_future", self._stage_id)
        if (
            self._phase == StagePhase.FREEZE
            and now_ns >= self._freeze_ends_at_ns
            and now_ns < self._stage_ends_at_ns
        ):
            self._phase = StagePhase.ACTIVE
            self._record(
                "phase_transition",
                "",
                RequestResult(True, "freeze_elapsed_before_event_adjudication", self._stage_id),
            )
        if event.kind == TerminalKind.ARRIVAL:
            target = self._zones.get(ZonePurpose.TARGET)
            if target is None or target.status != ZoneStatus.ACCEPTED:
                return RequestResult(False, "arrival_target_zone_unresolved", self._stage_id)
            if event.zone_id != target.zone_id:
                return RequestResult(False, "arrival_zone_id_mismatch", self._stage_id)
        if event.evidence == EventEvidence.ESTIMATE:
            if event.kind == TerminalKind.OFFICIAL_ABORT:
                return RequestResult(False, "estimated_abort_cannot_change_stage_result", self._stage_id)
            if self._phase == StagePhase.TERMINAL and self._terminal_evidence == EventEvidence.OFFICIAL:
                return RequestResult(False, "estimate_after_terminal", self._stage_id)
            if event.event_time_upper_ns >= self._stage_ends_at_ns:
                return RequestResult(False, "estimate_interval_overlaps_stage_deadline", self._stage_id)
            self._pending_estimates[event.event_id] = event
            local_hold = self._phase == StagePhase.ACTIVE and now_ns < self._stage_ends_at_ns
            if local_hold:
                self._event_hold = True
                self._held_event = (
                    next(iter(self._pending_estimates.values()))
                    if len(self._pending_estimates) == 1
                    else None
                )
            else:
                self._held_event = None
            self._terminalize_official_events()
            return RequestResult(
                True,
                (
                    "estimate_latched_as_local_hold_only"
                    if local_hold
                    else "estimate_retained_for_adjudication_only"
                ),
                self._stage_id,
            )
        if event.evidence == EventEvidence.SIM_TRUTH and not self.profile.allow_sim_truth_terminal:
            return RequestResult(False, "simulation_truth_terminal_not_enabled", self._stage_id)
        if not self._authorized(event.authorization_ref):
            return RequestResult(False, "event_authorization_not_approved", self._stage_id)
        if event.event_time_upper_ns >= self._stage_ends_at_ns:
            return RequestResult(False, "event_not_proven_strictly_before_deadline", self._stage_id)
        self._official_events.append(event)
        self._terminalize_official_events()
        return RequestResult(True, "event_recorded_without_inventing_order", self._stage_id)

    def resolve_estimate(
        self,
        *,
        request_id: str,
        expected_stage_id: str,
        event_id: str,
        resolution: EventResolution,
        terminal_kind: TerminalKind,
        organizer_authorization_ref: str,
        now_ns: int,
    ) -> RequestResult:
        resolution = _enum_value(EventResolution, resolution, "resolution")
        terminal_kind = _enum_value(TerminalKind, terminal_kind, "terminal_kind")
        _nonempty(event_id, "event_id")
        request = (expected_stage_id, event_id, resolution, terminal_kind, organizer_authorization_ref)
        with self._lock:
            self._advance_time(now_ns)
            cached = self._cached("resolve", request_id, request)
            if cached is not None:
                return cached
            if expected_stage_id != self._stage_id:
                result = RequestResult(False, "stale_expected_stage_id", self._stage_id)
            elif not self._authorized(organizer_authorization_ref):
                result = RequestResult(False, "organizer_authorization_not_approved", self._stage_id)
            elif resolution == EventResolution.SET_OFFICIAL_TERMINAL:
                if self._phase == StagePhase.INIT:
                    result = RequestResult(False, "official_terminal_outside_started_stage", self._stage_id)
                elif (
                    self._phase == StagePhase.TERMINAL
                    and self._terminal_evidence == EventEvidence.OFFICIAL
                    and self._terminal_kind != TerminalKind.AMBIGUOUS
                ):
                    result = RequestResult(False, "official_terminal_already_latched", self._stage_id)
                elif terminal_kind in (TerminalKind.CAPTURE, TerminalKind.ARRIVAL):
                    evidence = self._event_ids.get(event_id)
                    if (
                        evidence is None
                        or evidence.kind != terminal_kind
                        or evidence.evidence != EventEvidence.OFFICIAL
                        or evidence.predicate != EventPredicate.TRUE
                        or evidence.event_time_upper_ns >= self._stage_ends_at_ns
                    ):
                        result = RequestResult(False, "matching_adjudication_evidence_required", self._stage_id)
                    else:
                        self._phase = StagePhase.TERMINAL
                        self._event_hold = False
                        self._held_event = None
                        self._terminal_kind = terminal_kind
                        self._terminal_evidence = EventEvidence.OFFICIAL
                        result = RequestResult(True, "official_terminal_decision_latched", self._stage_id)
                elif terminal_kind not in (
                    TerminalKind.TIMEOUT,
                    TerminalKind.OFFICIAL_ABORT,
                    TerminalKind.AMBIGUOUS,
                ):
                    result = RequestResult(False, "terminal_kind_requires_matching_event_evidence", self._stage_id)
                elif terminal_kind == TerminalKind.TIMEOUT and now_ns < self._stage_ends_at_ns:
                    result = RequestResult(False, "official_timeout_before_deadline", self._stage_id)
                elif (
                    terminal_kind == TerminalKind.OFFICIAL_ABORT
                    and self._phase == StagePhase.TERMINAL
                    and self._terminal_evidence == EventEvidence.OFFICIAL
                ):
                    result = RequestResult(False, "official_terminal_already_latched", self._stage_id)
                else:
                    self._phase = StagePhase.TERMINAL
                    self._event_hold = terminal_kind == TerminalKind.AMBIGUOUS
                    self._held_event = None
                    self._terminal_kind = terminal_kind
                    self._terminal_evidence = EventEvidence.OFFICIAL
                    result = RequestResult(True, "official_terminal_decision_latched", self._stage_id)
            elif event_id not in self._pending_estimates:
                result = RequestResult(False, "matching_estimate_not_held", self._stage_id)
            elif resolution == EventResolution.DISMISS_ESTIMATE:
                self._pending_estimates.pop(event_id)
                self._terminalize_official_events()
                result = RequestResult(True, "estimate_dismissed_no_score_assigned", self._stage_id)
            elif resolution == EventResolution.CONFIRM_OFFICIAL:
                held_event = self._pending_estimates[event_id]
                conflicts = any(
                    other.event_id != event_id
                    and other.kind != held_event.kind
                    and other.event_time_lower_ns <= held_event.event_time_upper_ns
                    and held_event.event_time_lower_ns <= other.event_time_upper_ns
                    for other in self._pending_estimates.values()
                )
                if conflicts:
                    result = RequestResult(False, "conflicting_estimate_requires_resolution", self._stage_id)
                elif terminal_kind != held_event.kind:
                    result = RequestResult(False, "official_kind_does_not_match_estimate", self._stage_id)
                elif held_event.event_time_upper_ns >= self._stage_ends_at_ns:
                    result = RequestResult(False, "estimate_not_proven_before_deadline", self._stage_id)
                else:
                    confirmed = RuleEvent(
                        event_id=event_id,
                        stage_id=self._stage_id,
                        clock_epoch=self.profile.clock_epoch,
                        localization_epoch=self.profile.localization_epoch,
                        kind=terminal_kind,
                        evidence=EventEvidence.OFFICIAL,
                        predicate=EventPredicate.TRUE,
                        event_time_lower_ns=held_event.event_time_lower_ns,
                        event_time_upper_ns=held_event.event_time_upper_ns,
                        input_ids=held_event.input_ids,
                        authorization_ref=organizer_authorization_ref,
                        zone_id=held_event.zone_id,
                        reason="organizer-confirmed estimate",
                    )
                    self._event_ids[event_id] = confirmed
                    self._official_events.append(confirmed)
                    self._pending_estimates.pop(event_id)
                    self._terminalize_official_events()
                    result = RequestResult(True, "estimate_confirmed_by_official_authority", self._stage_id)
            else:
                result = RequestResult(False, "unsupported_event_resolution", self._stage_id)
            return self._remember("resolve", request_id, request, result)

    def reset_stage(
        self,
        *,
        request_id: str,
        expected_stage_id: str,
        stage_number: int,
        role: Role,
        memory_profile_id: str,
        organizer_authorization_ref: str,
        now_ns: int,
    ) -> RequestResult:
        role = _enum_value(Role, role, "role")
        stage_number = self._validate_stage_number(stage_number)
        if not isinstance(memory_profile_id, str):
            raise ValueError("memory_profile_id must be a string")
        request = (expected_stage_id, stage_number, role, memory_profile_id, organizer_authorization_ref)
        with self._lock:
            self._advance_time(now_ns)
            cached = self._cached("reset", request_id, request)
            if cached is not None:
                return cached
            if expected_stage_id != self._stage_id:
                result = RequestResult(False, "stale_expected_stage_id", self._stage_id)
            elif self._reset_barrier_pending:
                result = RequestResult(False, "reset_effects_not_acknowledged", self._stage_id)
            elif self._phase not in (StagePhase.INIT, StagePhase.TERMINAL):
                result = RequestResult(False, "reset_requires_INIT_or_TERMINAL_phase", self._stage_id)
            elif not self._authorized(organizer_authorization_ref):
                result = RequestResult(False, "organizer_authorization_not_approved", self._stage_id)
            elif memory_profile_id and memory_profile_id not in self.profile.approved_memory_profile_ids:
                result = RequestResult(False, "memory_retention_profile_not_approved", self._stage_id)
            else:
                new_stage_id = self._stage_id_factory()
                _nonempty(new_stage_id, "generated_stage_id")
                if new_stage_id in self._used_stage_ids:
                    result = RequestResult(False, "stage_id_factory_reused_id", self._stage_id)
                else:
                    self._stage_id = new_stage_id
                    self._used_stage_ids.add(new_stage_id)
                    self._stage_number = stage_number
                    self._role = role
                    self._phase = StagePhase.INIT
                    self._stage_started_at_ns = 0
                    self._freeze_ends_at_ns = 0
                    self._stage_ends_at_ns = 0
                    self._event_hold = False
                    self._held_event = None
                    self._pending_estimates.clear()
                    self._terminal_kind = TerminalKind.NONE
                    self._terminal_evidence = EventEvidence.ESTIMATE
                    self._official_events.clear()
                    self._event_ids.clear()
                    self._zones.clear()
                    self._reset_barrier_pending = True
                    self._reset_acks.clear()
                    self._epoch_barrier_pending = False
                    self._epoch_acks.clear()
                    required = _RESET_ACKS
                    result = RequestResult(
                        True,
                        "stage_reset_pending_downstream_revocation",
                        self._stage_id,
                        required,
                        memory_profile_id,
                    )
            return self._remember("reset", request_id, request, result)

    def acknowledge_reset(self, *, stage_id: str, acknowledgements: frozenset[str]) -> bool:
        with self._lock:
            if stage_id != self._stage_id:
                return False
            if not isinstance(acknowledgements, frozenset):
                raise ValueError("acknowledgements must be a frozenset")
            if not acknowledgements <= _RESET_ACKS:
                raise ValueError("unknown reset acknowledgement")
            if not self._reset_barrier_pending:
                return self._reset_acks == _RESET_ACKS and acknowledgements <= self._reset_acks
            self._reset_acks.update(acknowledgements)
            if self._reset_acks != _RESET_ACKS:
                self._record(
                    "reset_ack",
                    f"reset-ack:{stage_id}",
                    RequestResult(False, "downstream_effects_still_pending", self._stage_id),
                )
                return False
            self._reset_barrier_pending = False
            self._record(
                "reset_ack",
                f"reset-ack:{stage_id}",
                RequestResult(True, "all_downstream_effects_acknowledged", self._stage_id),
            )
            return True

    def change_localization_epoch(self, *, new_epoch: str) -> RequestResult:
        _nonempty(new_epoch, "new_epoch")
        with self._lock:
            if new_epoch == self.profile.localization_epoch:
                raise ValueError("new localization epoch must differ")
            self._profile = StageProfile(
                profile_id=self.profile.profile_id,
                score_profile_id=self.profile.score_profile_id,
                freeze_duration_ns=self.profile.freeze_duration_ns,
                stage_duration_ns=self.profile.stage_duration_ns,
                lease_duration_ns=self.profile.lease_duration_ns,
                zone_update_window_ns=self.profile.zone_update_window_ns,
                max_request_records=self.profile.max_request_records,
                max_event_records=self.profile.max_event_records,
                max_audit_records=self.profile.max_audit_records,
                config_hash=self.profile.config_hash,
                source_id=self.profile.source_id,
                source_session=self.profile.source_session,
                clock_epoch=self.profile.clock_epoch,
                localization_epoch=new_epoch,
                organizer_authorization_ref=self.profile.organizer_authorization_ref,
                approved_authorization_refs=self.profile.approved_authorization_refs,
                approved_zone_frames=self.profile.approved_zone_frames,
                approved_memory_profile_ids=self.profile.approved_memory_profile_ids,
                allow_sim_truth_terminal=self.profile.allow_sim_truth_terminal,
            )
            stale_estimate_ids = [
                event_id
                for event_id, event in self._event_ids.items()
                if event.evidence == EventEvidence.ESTIMATE
            ]
            for event_id in stale_estimate_ids:
                self._event_ids.pop(event_id, None)
            self._pending_estimates.clear()
            if self._held_event is not None and self._held_event.evidence == EventEvidence.ESTIMATE:
                self._held_event = None
                self._event_hold = False
            self._epoch_barrier_pending = True
            self._epoch_acks.clear()
            result = RequestResult(
                True,
                "localization_epoch_changed_authority_barrier_pending",
                self._stage_id,
                _EPOCH_ACKS,
            )
            self._record(
                "localization_epoch_change",
                "",
                result,
            )
            return result

    def acknowledge_localization_epoch(
        self, *, stage_id: str, acknowledgements: frozenset[str]
    ) -> bool:
        with self._lock:
            if stage_id != self._stage_id:
                return False
            if not isinstance(acknowledgements, frozenset):
                raise ValueError("acknowledgements must be a frozenset")
            if not acknowledgements <= _EPOCH_ACKS:
                raise ValueError("unknown localization-epoch acknowledgement")
            if not self._epoch_barrier_pending:
                return self._epoch_acks == _EPOCH_ACKS and acknowledgements <= self._epoch_acks
            self._epoch_acks.update(acknowledgements)
            if self._epoch_acks != _EPOCH_ACKS:
                self._record(
                    "localization_epoch_ack",
                    f"epoch-ack:{stage_id}",
                    RequestResult(False, "epoch_effects_still_pending", self._stage_id),
                )
                return False
            self._epoch_barrier_pending = False
            self._record(
                "localization_epoch_ack",
                f"epoch-ack:{stage_id}",
                RequestResult(True, "epoch_products_invalidated", self._stage_id),
            )
            return True

    def change_clock_epoch(self, *, new_epoch: str, now_ns: int) -> RequestResult:
        _nonempty(new_epoch, "new_epoch")
        _nonnegative_int(now_ns, "now_ns")
        with self._lock:
            if new_epoch == self.profile.clock_epoch:
                raise ValueError("new clock epoch must differ")
            prior_stage_id = self._stage_id
            new_stage_id = self._stage_id_factory()
            _nonempty(new_stage_id, "generated_stage_id")
            if new_stage_id in self._used_stage_ids:
                raise ValueError("stage ID factory reused an earlier ID")
            self._used_stage_ids.add(new_stage_id)
            self._profile = StageProfile(
                profile_id=self.profile.profile_id,
                score_profile_id=self.profile.score_profile_id,
                freeze_duration_ns=self.profile.freeze_duration_ns,
                stage_duration_ns=self.profile.stage_duration_ns,
                lease_duration_ns=self.profile.lease_duration_ns,
                zone_update_window_ns=self.profile.zone_update_window_ns,
                max_request_records=self.profile.max_request_records,
                max_event_records=self.profile.max_event_records,
                max_audit_records=self.profile.max_audit_records,
                config_hash=self.profile.config_hash,
                source_id=self.profile.source_id,
                source_session=self.profile.source_session,
                clock_epoch=new_epoch,
                localization_epoch=self.profile.localization_epoch,
                organizer_authorization_ref=self.profile.organizer_authorization_ref,
                approved_authorization_refs=self.profile.approved_authorization_refs,
                approved_zone_frames=self.profile.approved_zone_frames,
                approved_memory_profile_ids=self.profile.approved_memory_profile_ids,
                allow_sim_truth_terminal=self.profile.allow_sim_truth_terminal,
            )
            self._stage_id = new_stage_id
            self._phase = StagePhase.INIT
            self._stage_started_at_ns = 0
            self._freeze_ends_at_ns = 0
            self._stage_ends_at_ns = 0
            self._event_hold = False
            self._held_event = None
            self._terminal_kind = TerminalKind.NONE
            self._terminal_evidence = EventEvidence.ESTIMATE
            self._official_events.clear()
            self._event_ids.clear()
            self._pending_estimates.clear()
            self._zones.clear()
            self._operations.clear()
            self._last_now_ns = now_ns
            self._reset_barrier_pending = True
            self._reset_acks.clear()
            self._epoch_barrier_pending = False
            self._epoch_acks.clear()
            result = RequestResult(
                True,
                "clock_epoch_changed_stage_disarmed",
                self._stage_id,
                _RESET_ACKS,
            )
            self._record(
                "clock_epoch_change",
                "",
                result,
                prior_stage_id=prior_stage_id,
            )
            return result

    def snapshot(
        self,
        *,
        now_ns: int,
        map_version: int = 0,
        topology_version: int = 0,
    ) -> MatchState:
        with self._lock:
            self._advance_time(now_ns)
            _nonnegative_int(map_version, "map_version")
            _nonnegative_int(topology_version, "topology_version")
            if now_ns + self.profile.lease_duration_ns > (1 << 63) - 1:
                raise ValueError("MatchState lease overflows supported timestamp range")
            self._seq += 1
            header = ContractHeader(
                schema_version=2,
                source_id=self.profile.source_id,
                source_session=self.profile.source_session,
                seq=self._seq,
                stage_id=self._stage_id,
                clock_epoch=self.profile.clock_epoch,
                localization_epoch=self.profile.localization_epoch,
                frame_id="match",
                observation_stamp_ns=now_ns,
                state_stamp_ns=now_ns,
                publication_stamp_ns=now_ns,
                valid_until_ns=now_ns + self.profile.lease_duration_ns,
                map_version=map_version,
                topology_version=topology_version,
                validity=1,
            )
            own_zone, target_zone = self._zones_in_state()
            return MatchState(
                meta=header,
                stage_number=self._stage_number,
                role=self._role,
                phase=self._phase,
                stage_started_at_ns=self._stage_started_at_ns,
                freeze_ends_at_ns=self._freeze_ends_at_ns,
                stage_ends_at_ns=self._stage_ends_at_ns,
                motion_authorized=(
                    self._phase == StagePhase.ACTIVE
                    and not self._event_hold
                    and not self._reset_barrier_pending
                    and not self._epoch_barrier_pending
                ),
                event_hold=self._event_hold,
                terminal_kind=self._terminal_kind,
                terminal_evidence=self._terminal_evidence,
                score_profile_id=self.profile.score_profile_id,
                own_start_zone_id=own_zone,
                target_zone_id=target_zone,
                config_hash=self.profile.config_hash,
            )
