"""Immutable, provenance-bound records and split contracts for P6.1."""

from dataclasses import dataclass
from enum import Enum
import hashlib
import json
import math
import re
from bisect import bisect_right
from typing import Iterable

from ..match import EventEvidence, Role
from ..tactics import OptionKind, OptionOutcome


SCHEMA_VERSION = 1
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_FORBIDDEN_FEATURE_TOKENS = (
    "truth",
    "ground_truth",
    "oracle",
    "referee",
    "official_event",
    "hidden_pose",
)
_SPLITS = frozenset(("training", "validation", "held_out"))


def _text(value: str, field: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")


def _hash(value: str, field: str) -> None:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ValueError(f"{field} must be a lowercase SHA-256 hex digest")


def _finite(value: float, field: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be finite")
    try:
        finite = math.isfinite(float(value))
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f"{field} must be finite")


def _integer(value: int, field: str, *, minimum: int = 0) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < minimum:
        raise ValueError(f"{field} must be an integer >= {minimum}")


def _canonical_hash(value: object) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class FeatureSchema:
    schema_id: str
    feature_names: tuple[str, ...]
    version: int = SCHEMA_VERSION
    bin_edges: tuple[tuple[str, tuple[float, ...]], ...] = ()

    def __post_init__(self) -> None:
        _text(self.schema_id, "feature schema ID")
        _integer(self.version, "feature schema version", minimum=1)
        names = tuple(self.feature_names)
        if not names or any(not isinstance(name, str) or not name.strip() for name in names):
            raise ValueError("feature schema must define non-empty feature names")
        if len(names) != len(set(names)):
            raise ValueError("feature names must be unique")
        if tuple(sorted(names)) != names:
            raise ValueError("feature names must be in canonical sorted order")
        for name in names:
            lowered = name.lower()
            if any(token in lowered for token in _FORBIDDEN_FEATURE_TOKENS):
                raise ValueError(f"feature schema includes a prohibited truth field: {name}")
        edges = tuple(
            (name, tuple(boundaries)) for name, boundaries in self.bin_edges
        )
        if tuple(name for name, _ in edges) != names:
            raise ValueError("bin edges must define every feature in schema order")
        for name, boundaries in edges:
            for boundary in boundaries:
                _finite(boundary, f"{name} bin boundary")
                if not 0.0 < boundary < 1.0:
                    raise ValueError("bin boundaries must be strictly inside (0, 1)")
            if tuple(sorted(set(boundaries))) != boundaries:
                raise ValueError("bin boundaries must be strictly increasing")
        object.__setattr__(self, "feature_names", names)
        object.__setattr__(self, "bin_edges", edges)

    @property
    def fingerprint(self) -> str:
        return _canonical_hash(
            {
                "schema_id": self.schema_id,
                "version": self.version,
                "feature_names": self.feature_names,
                "bin_edges": self.bin_edges,
            }
        )

    def bin_values(
        self, values: tuple[tuple[str, float], ...]
    ) -> tuple[tuple[str, int], ...]:
        edge_map = dict(self.bin_edges)
        return tuple(
            (name, bisect_right(edge_map[name], value))
            for name, value in values
        )


@dataclass(frozen=True)
class FeatureState:
    schema_id: str
    schema_sha256: str
    role: Role
    features: tuple[tuple[str, float], ...]
    feature_bins: tuple[tuple[str, int], ...]

    def __post_init__(self) -> None:
        _text(self.schema_id, "feature schema ID")
        _hash(self.schema_sha256, "feature schema fingerprint")
        if not isinstance(self.role, Role):
            raise ValueError("feature state role must be a Role")
        entries = tuple(self.features)
        names: list[str] = []
        for entry in entries:
            if not isinstance(entry, (tuple, list)) or len(entry) != 2:
                raise ValueError("each feature must be a name/value pair")
            name, value = entry
            _text(name, "feature name")
            _finite(value, f"feature {name}")
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"feature {name} must be normalized within [0, 1]")
            if any(token in name.lower() for token in _FORBIDDEN_FEATURE_TOKENS):
                raise ValueError(f"feature state contains a prohibited truth field: {name}")
            names.append(name)
        if len(names) != len(set(names)):
            raise ValueError("feature names must be unique")
        if names != sorted(names):
            raise ValueError("feature values must use canonical feature-name order")
        object.__setattr__(
            self,
            "features",
            tuple((name, float(value)) for name, value in entries),
        )
        bins = tuple(self.feature_bins)
        if tuple(name for name, _ in bins) != tuple(names):
            raise ValueError("feature bins must match feature names and canonical order")
        if any(
            not isinstance(index, int) or isinstance(index, bool) or index < 0
            for _, index in bins
        ):
            raise ValueError("feature-bin indices must be non-negative integers")
        object.__setattr__(self, "feature_bins", bins)

    def validate(self, schema: FeatureSchema) -> None:
        if self.schema_id != schema.schema_id:
            raise ValueError("feature state schema ID mismatch")
        if self.schema_sha256 != schema.fingerprint:
            raise ValueError("feature state schema fingerprint mismatch")
        if tuple(name for name, _ in self.features) != schema.feature_names:
            raise ValueError("feature state does not match the declared feature schema")
        if schema.bin_values(self.features) != self.feature_bins:
            raise ValueError("feature-bin indices do not match schema boundaries")

    @property
    def state_id(self) -> str:
        return _canonical_hash(
            {
                "schema_id": self.schema_id,
                "schema_sha256": self.schema_sha256,
                "role": self.role.name,
                "feature_bins": self.feature_bins,
            }
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_id": self.schema_id,
            "schema_sha256": self.schema_sha256,
            "role": self.role.name,
            "state_id": self.state_id,
            "features": {name: value for name, value in self.features},
            "feature_bins": {name: index for name, index in self.feature_bins},
        }


@dataclass(frozen=True)
class RewardRateSegment:
    """Piecewise-constant reward-rate interval relative to option start."""

    start_ns: int
    end_ns: int
    rate_per_s: float

    def __post_init__(self) -> None:
        _integer(self.start_ns, "reward segment start")
        _integer(self.end_ns, "reward segment end", minimum=1)
        if self.end_ns <= self.start_ns:
            raise ValueError("reward segment must have positive duration")
        _finite(self.rate_per_s, "reward rate")


@dataclass(frozen=True)
class RewardComponent:
    component_id: str
    rate_segments: tuple[RewardRateSegment, ...] = ()
    terminal_impulse: float = 0.0

    def __post_init__(self) -> None:
        _text(self.component_id, "reward component ID")
        segments = tuple(self.rate_segments)
        if any(not isinstance(item, RewardRateSegment) for item in segments):
            raise ValueError("reward component contains an invalid rate segment")
        ordered = sorted(segments, key=lambda item: (item.start_ns, item.end_ns))
        if tuple(ordered) != segments:
            raise ValueError("reward rate segments must be sorted by start time")
        if any(
            current.start_ns < previous.end_ns
            for previous, current in zip(segments, segments[1:])
        ):
            raise ValueError("reward rate segments in a component must not overlap")
        _finite(self.terminal_impulse, "terminal reward impulse")
        object.__setattr__(self, "rate_segments", segments)


class CensoringStatus(str, Enum):
    COMPLETE = "complete"
    CENSORED = "censored"
    TRUNCATED = "truncated"


@dataclass(frozen=True)
class OptionTransition:
    """One immutable option sample; it does not certify its evidence source."""

    episode_id: str
    stage_id: str
    role: Role
    input_state: FeatureState
    option_instance_id: str
    option_kind: OptionKind
    parameters_sha256: str
    start_ns: int
    end_ns: int
    outcome: OptionOutcome | None
    terminal: bool
    next_state: FeatureState | None
    available_next_options: tuple[OptionKind, ...]
    reward_components: tuple[RewardComponent, ...]
    safety_intervention_ids: tuple[str, ...]
    official_event_id: str
    official_event_evidence: EventEvidence | None
    policy_sha256: str
    sensor_fidelity_id: str
    map_fidelity_id: str
    scenario_id: str
    opponent_id: str
    seed: int
    map_bank_id: str
    opponent_bank_id: str
    seed_bank_id: str
    source_profile_id: str
    censoring: CensoringStatus = CensoringStatus.COMPLETE
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        for value, field in (
            (self.episode_id, "episode_id"),
            (self.stage_id, "stage_id"),
            (self.option_instance_id, "option_instance_id"),
            (self.sensor_fidelity_id, "sensor_fidelity_id"),
            (self.map_fidelity_id, "map_fidelity_id"),
            (self.scenario_id, "scenario_id"),
            (self.opponent_id, "opponent_id"),
            (self.map_bank_id, "map_bank_id"),
            (self.opponent_bank_id, "opponent_bank_id"),
            (self.seed_bank_id, "seed_bank_id"),
            (self.source_profile_id, "source_profile_id"),
        ):
            _text(value, field)
        _hash(self.parameters_sha256, "parameters_sha256")
        _hash(self.policy_sha256, "policy_sha256")
        _integer(self.schema_version, "option transition schema version", minimum=1)
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError("unsupported option transition schema version")
        if not isinstance(self.role, Role):
            raise ValueError("role must be a Role")
        if not isinstance(self.input_state, FeatureState):
            raise ValueError("input_state must be a FeatureState")
        if self.input_state.role != self.role:
            raise ValueError("input state role must match transition role")
        if not isinstance(self.option_kind, OptionKind):
            raise ValueError("option_kind must be an OptionKind")
        if self.outcome is not None and not isinstance(self.outcome, OptionOutcome):
            raise ValueError("outcome must be an OptionOutcome or None")
        if not isinstance(self.terminal, bool):
            raise ValueError("terminal must be boolean")
        _integer(self.start_ns, "start_ns")
        _integer(self.end_ns, "end_ns", minimum=1)
        if self.end_ns <= self.start_ns:
            raise ValueError("option transition duration must be positive")
        _integer(self.seed, "seed")
        try:
            censoring = CensoringStatus(self.censoring)
        except (TypeError, ValueError) as error:
            raise ValueError("invalid censoring status") from error
        object.__setattr__(self, "censoring", censoring)

        if censoring == CensoringStatus.COMPLETE:
            if self.outcome is None:
                raise ValueError("complete transition requires a registered outcome")
            if self.terminal:
                if self.next_state is not None or self.available_next_options:
                    raise ValueError("terminal transition must not bootstrap from a next state")
            else:
                if not isinstance(self.next_state, FeatureState):
                    raise ValueError("nonterminal transition requires a next state")
                if self.next_state.role != self.role:
                    raise ValueError("next state role must match transition role")
                if self.next_state.schema_id != self.input_state.schema_id:
                    raise ValueError("input and next feature schemas must match")
                if not self.available_next_options:
                    raise ValueError("nonterminal transition requires available next options")
        else:
            if self.outcome is not None or self.terminal or self.next_state is not None:
                raise ValueError(
                    "censored/truncated records must not invent an outcome or next state"
                )
            if self.available_next_options:
                raise ValueError("censored/truncated records must not claim next options")
        options = tuple(self.available_next_options)
        if any(not isinstance(item, OptionKind) for item in options):
            raise ValueError("available_next_options must contain OptionKind values")
        if len(options) != len(set(options)):
            raise ValueError("available next option kinds must be unique")
        object.__setattr__(
            self,
            "available_next_options",
            tuple(sorted(options, key=int)),
        )

        rewards = tuple(self.reward_components)
        if any(not isinstance(item, RewardComponent) for item in rewards):
            raise ValueError("reward_components must contain RewardComponent values")
        component_ids = tuple(item.component_id for item in rewards)
        if len(component_ids) != len(set(component_ids)):
            raise ValueError("reward component IDs must be unique")
        duration_ns = self.end_ns - self.start_ns
        for component in rewards:
            for segment in component.rate_segments:
                if segment.end_ns > duration_ns:
                    raise ValueError("reward segment exceeds observed option duration")
        object.__setattr__(self, "reward_components", rewards)

        interventions = tuple(self.safety_intervention_ids)
        if any(not isinstance(item, str) or not item.strip() for item in interventions):
            raise ValueError("safety intervention IDs must be non-empty strings")
        if len(interventions) != len(set(interventions)):
            raise ValueError("safety intervention IDs must be unique")
        object.__setattr__(
            self, "safety_intervention_ids", tuple(sorted(interventions))
        )

        if self.official_event_evidence is None:
            if not isinstance(self.official_event_id, str) or self.official_event_id:
                raise ValueError("official event ID requires its evidence classification")
        else:
            if not isinstance(self.official_event_id, str) or not self.official_event_id.strip():
                raise ValueError("event evidence requires an event ID")
            if not isinstance(self.official_event_evidence, EventEvidence):
                raise ValueError("official_event_evidence must be an EventEvidence")

    @property
    def duration_ns(self) -> int:
        return self.end_ns - self.start_ns

    @property
    def duration_s(self) -> float:
        return self.duration_ns / 1_000_000_000.0

    @property
    def input_state_id(self) -> str:
        return self.input_state.state_id

    @property
    def next_state_id(self) -> str | None:
        return None if self.next_state is None else self.next_state.state_id

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "episode_id": self.episode_id,
            "stage_id": self.stage_id,
            "role": self.role.name,
            "input_state": self.input_state.to_dict(),
            "option_instance_id": self.option_instance_id,
            "option_kind": self.option_kind.name,
            "parameters_sha256": self.parameters_sha256,
            "start_ns": self.start_ns,
            "end_ns": self.end_ns,
            "duration_s": self.duration_s,
            "outcome": None if self.outcome is None else self.outcome.name,
            "terminal": self.terminal,
            "next_state": None if self.next_state is None else self.next_state.to_dict(),
            "available_next_options": [
                item.name for item in self.available_next_options
            ],
            "reward_components": [
                {
                    "component_id": component.component_id,
                    "rate_segments": [
                        {
                            "start_ns": segment.start_ns,
                            "end_ns": segment.end_ns,
                            "rate_per_s": segment.rate_per_s,
                        }
                        for segment in component.rate_segments
                    ],
                    "terminal_impulse": component.terminal_impulse,
                }
                for component in self.reward_components
            ],
            "official_event": (
                None
                if self.official_event_evidence is None
                else {
                    "event_id": self.official_event_id,
                    "evidence": self.official_event_evidence.name,
                }
            ),
            "safety_intervention_ids": list(self.safety_intervention_ids),
            "policy_sha256": self.policy_sha256,
            "sensor_fidelity_id": self.sensor_fidelity_id,
            "map_fidelity_id": self.map_fidelity_id,
            "scenario_id": self.scenario_id,
            "opponent_id": self.opponent_id,
            "seed": self.seed,
            "map_bank_id": self.map_bank_id,
            "opponent_bank_id": self.opponent_bank_id,
            "seed_bank_id": self.seed_bank_id,
            "source_profile_id": self.source_profile_id,
            "censoring": self.censoring.value,
        }

    @property
    def record_sha256(self) -> str:
        return _canonical_hash(self.to_dict())


@dataclass(frozen=True)
class BaselineEvidence:
    profile_id: str
    g4_status: str
    acceptance_record_sha256: str
    map_bank_approval_sha256: str
    opponent_bank_approval_sha256: str
    seed_bank_approval_sha256: str

    def __post_init__(self) -> None:
        _text(self.profile_id, "baseline profile ID")
        if not isinstance(self.g4_status, str) or self.g4_status not in (
            "PASS",
            "BLOCKED",
            "BLOCKED_NOT_RUN",
            "NOT_RUN",
            "FAIL",
        ):
            raise ValueError("unsupported G4 status")
        for value, field in (
            (self.acceptance_record_sha256, "acceptance record hash"),
            (self.map_bank_approval_sha256, "map bank approval hash"),
            (self.opponent_bank_approval_sha256, "opponent bank approval hash"),
            (self.seed_bank_approval_sha256, "seed bank approval hash"),
        ):
            _hash(value, field)


@dataclass(frozen=True)
class SplitAssignment:
    """Independent bank IDs assigned once to training/validation/held-out."""

    map_banks: tuple[tuple[str, str], ...]
    opponent_banks: tuple[tuple[str, str], ...]
    seed_banks: tuple[tuple[str, str], ...]

    def __post_init__(self) -> None:
        for field, pairs in (
            ("map_banks", self.map_banks),
            ("opponent_banks", self.opponent_banks),
            ("seed_banks", self.seed_banks),
        ):
            entries = tuple(pairs)
            ids: set[str] = set()
            normalized: list[tuple[str, str]] = []
            for entry in entries:
                if not isinstance(entry, (tuple, list)) or len(entry) != 2:
                    raise ValueError(f"{field} must contain bank/split pairs")
                bank_id, split = entry
                _text(bank_id, f"{field} bank ID")
                if not isinstance(split, str) or split not in _SPLITS:
                    raise ValueError(f"{field} has unsupported split: {split}")
                if bank_id in ids:
                    raise ValueError(f"{field} bank IDs must be assigned exactly once")
                ids.add(bank_id)
                normalized.append((bank_id, split))
            object.__setattr__(self, field, tuple(sorted(normalized)))

    def split_for(self, transition: OptionTransition) -> str:
        assignments = (
            (self.map_banks, transition.map_bank_id, "map"),
            (self.opponent_banks, transition.opponent_bank_id, "opponent"),
            (self.seed_banks, transition.seed_bank_id, "seed"),
        )
        result: set[str] = set()
        for pairs, bank_id, kind in assignments:
            mapping = dict(pairs)
            if bank_id not in mapping:
                raise ValueError(f"transition references unassigned {kind} bank {bank_id}")
            result.add(mapping[bank_id])
        if len(result) != 1:
            raise ValueError("map/opponent/seed banks for a transition cross dataset splits")
        return next(iter(result))


@dataclass(frozen=True)
class DatasetManifest:
    schema_version: int
    feature_schema_id: str
    feature_schema_sha256: str
    record_count: int
    completed_count: int
    censored_count: int
    truncated_count: int
    outcome_counts: tuple[tuple[str, int], ...]
    dataset_sha256: str
    map_bank_splits: tuple[tuple[str, str], ...]
    opponent_bank_splits: tuple[tuple[str, str], ...]
    seed_bank_splits: tuple[tuple[str, str], ...]
    split_record_counts: tuple[tuple[str, int], ...]
    profile_ids: tuple[str, ...]
    g4_status: str
    baseline_profile_id: str | None
    baseline_acceptance_record_sha256: str | None
    bank_approval_sha256: tuple[tuple[str, str], ...]
    contract_valid: bool
    promotion_eligible: bool
    ineligibility_reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "feature_schema_id": self.feature_schema_id,
            "feature_schema_sha256": self.feature_schema_sha256,
            "record_count": self.record_count,
            "completed_count": self.completed_count,
            "censored_count": self.censored_count,
            "truncated_count": self.truncated_count,
            "outcome_counts": dict(self.outcome_counts),
            "dataset_sha256": self.dataset_sha256,
            "split_assignments": {
                "map_banks": dict(self.map_bank_splits),
                "opponent_banks": dict(self.opponent_bank_splits),
                "seed_banks": dict(self.seed_bank_splits),
            },
            "split_record_counts": dict(self.split_record_counts),
            "profile_ids": list(self.profile_ids),
            "g4_status": self.g4_status,
            "baseline_profile_id": self.baseline_profile_id,
            "baseline_acceptance_record_sha256": self.baseline_acceptance_record_sha256,
            "bank_approval_sha256": dict(self.bank_approval_sha256),
            "contract_valid": self.contract_valid,
            "promotion_eligible": self.promotion_eligible,
            "ineligibility_reasons": list(self.ineligibility_reasons),
        }


class TransitionDataset:
    """Contract validator and deterministic summary; no implicit data approval."""

    def __init__(
        self,
        feature_schema: FeatureSchema,
        records: Iterable[OptionTransition],
        *,
        splits: SplitAssignment | None = None,
        baseline: BaselineEvidence | None = None,
    ) -> None:
        if not isinstance(feature_schema, FeatureSchema):
            raise ValueError("feature_schema must be a FeatureSchema")
        if splits is not None and not isinstance(splits, SplitAssignment):
            raise TypeError("splits must be a SplitAssignment or None")
        if baseline is not None and not isinstance(baseline, BaselineEvidence):
            raise TypeError("baseline must be BaselineEvidence or None")
        self.feature_schema = feature_schema
        self.records = tuple(records)
        self.splits = splits
        self.baseline = baseline
        if any(not isinstance(record, OptionTransition) for record in self.records):
            raise ValueError("dataset records must be OptionTransition values")
        self._validate_records()

    def _validate_records(self) -> None:
        instance_keys: set[tuple[str, str, Role, str]] = set()
        episode_splits: dict[str, str] = {}
        for record in self.records:
            record.input_state.validate(self.feature_schema)
            if record.next_state is not None:
                record.next_state.validate(self.feature_schema)
            key = (
                record.episode_id,
                record.stage_id,
                record.role,
                record.option_instance_id,
            )
            if key in instance_keys:
                raise ValueError("dataset contains a duplicate option transition record")
            instance_keys.add(key)
            if self.splits is not None:
                split = self.splits.split_for(record)
                prior_split = episode_splits.setdefault(record.episode_id, split)
                if prior_split != split:
                    raise ValueError("one episode must not cross dataset splits")
            if (
                self.baseline is not None
                and record.source_profile_id != self.baseline.profile_id
            ):
                raise ValueError("transition source profile does not match baseline profile")

    def manifest(self) -> DatasetManifest:
        reasons: list[str] = []
        if self.splits is None:
            reasons.append("independent_bank_split_manifest_missing")
        elif not {
            split for _, split in self.splits.map_banks
        } >= _SPLITS or not {
            split for _, split in self.splits.opponent_banks
        } >= _SPLITS or not {
            split for _, split in self.splits.seed_banks
        } >= _SPLITS:
            reasons.append("one_or_more_bank_dimensions_lack_independent_splits")
        if self.baseline is None:
            reasons.append("accepted_g4_baseline_evidence_missing")
        elif self.baseline.g4_status != "PASS":
            reasons.append("g4_baseline_not_accepted")
        counts = {
            status.value: sum(record.censoring == status for record in self.records)
            for status in CensoringStatus
        }
        outcomes: dict[str, int] = {}
        for record in self.records:
            if record.outcome is not None:
                outcomes[record.outcome.name] = outcomes.get(record.outcome.name, 0) + 1
        split_counts: dict[str, int] = {}
        if self.splits is not None:
            for record in self.records:
                split = self.splits.split_for(record)
                split_counts[split] = split_counts.get(split, 0) + 1
            if not _SPLITS <= set(split_counts):
                reasons.append("training_validation_or_held_out_records_missing")
        profile_ids = tuple(sorted({record.source_profile_id for record in self.records}))
        if len(profile_ids) > 1:
            reasons.append("mixed_source_profiles")
        reasons.append("dataset_quality_and_promotion_review_not_assessed")
        record_hashes = sorted(record.record_sha256 for record in self.records)
        structural_reasons = tuple(
            reason
            for reason in reasons
            if reason != "dataset_quality_and_promotion_review_not_assessed"
        )
        dataset_hash = _canonical_hash(
            {
                "schema_version": SCHEMA_VERSION,
                "feature_schema_sha256": self.feature_schema.fingerprint,
                "record_sha256": record_hashes,
                "splits": (
                    None
                    if self.splits is None
                    else {
                        "map_banks": self.splits.map_banks,
                        "opponent_banks": self.splits.opponent_banks,
                        "seed_banks": self.splits.seed_banks,
                    }
                ),
                "baseline": (
                    None
                    if self.baseline is None
                    else {
                        "profile_id": self.baseline.profile_id,
                        "g4_status": self.baseline.g4_status,
                        "acceptance_record_sha256": self.baseline.acceptance_record_sha256,
                        "map_bank_approval_sha256": self.baseline.map_bank_approval_sha256,
                        "opponent_bank_approval_sha256": self.baseline.opponent_bank_approval_sha256,
                        "seed_bank_approval_sha256": self.baseline.seed_bank_approval_sha256,
                    }
                ),
            }
        )
        return DatasetManifest(
            schema_version=SCHEMA_VERSION,
            feature_schema_id=self.feature_schema.schema_id,
            feature_schema_sha256=self.feature_schema.fingerprint,
            record_count=len(self.records),
            completed_count=counts[CensoringStatus.COMPLETE.value],
            censored_count=counts[CensoringStatus.CENSORED.value],
            truncated_count=counts[CensoringStatus.TRUNCATED.value],
            outcome_counts=tuple(sorted(outcomes.items())),
            dataset_sha256=dataset_hash,
            map_bank_splits=(
                () if self.splits is None else self.splits.map_banks
            ),
            opponent_bank_splits=(
                () if self.splits is None else self.splits.opponent_banks
            ),
            seed_bank_splits=(
                () if self.splits is None else self.splits.seed_banks
            ),
            split_record_counts=tuple(sorted(split_counts.items())),
            profile_ids=profile_ids,
            g4_status=(
                "NOT_RUN" if self.baseline is None else self.baseline.g4_status
            ),
            baseline_profile_id=(
                None if self.baseline is None else self.baseline.profile_id
            ),
            baseline_acceptance_record_sha256=(
                None
                if self.baseline is None
                else self.baseline.acceptance_record_sha256
            ),
            bank_approval_sha256=(
                ()
                if self.baseline is None
                else (
                    ("map", self.baseline.map_bank_approval_sha256),
                    ("opponent", self.baseline.opponent_bank_approval_sha256),
                    ("seed", self.baseline.seed_bank_approval_sha256),
                )
            ),
            contract_valid=not structural_reasons,
            promotion_eligible=not reasons,
            ineligibility_reasons=tuple(reasons),
        )

    def to_jsonl(self) -> str:
        ordered = sorted(
            self.records,
            key=lambda record: (
                record.episode_id,
                record.stage_id,
                int(record.role),
                record.option_instance_id,
            ),
        )
        return "".join(
            json.dumps(
                record.to_dict(),
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=True,
            )
            + "\n"
            for record in ordered
        )


def feature_state(
    schema: FeatureSchema, role: Role, values: dict[str, float]
) -> FeatureState:
    """Create a schema-ordered state and reject missing/extra feature inputs."""

    if not isinstance(schema, FeatureSchema):
        raise ValueError("schema must be a FeatureSchema")
    if set(values) != set(schema.feature_names):
        raise ValueError("feature values must exactly match the declared schema")
    state = FeatureState(
        schema_id=schema.schema_id,
        schema_sha256=schema.fingerprint,
        role=role,
        features=tuple((name, values[name]) for name in schema.feature_names),
        feature_bins=schema.bin_values(
            tuple((name, values[name]) for name in schema.feature_names)
        ),
    )
    state.validate(schema)
    return state
