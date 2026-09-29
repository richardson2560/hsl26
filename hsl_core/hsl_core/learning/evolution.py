"""Bounded tactical-genome search and paired SIL evaluation contracts.

This module selects only among existing utility profiles. It cannot alter
motion limits, option realizers, safety predicates, or runtime authority.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import hashlib
import json
import math
from random import Random
from typing import Iterable

from scipy.stats import t as student_t

from ..match import Role
from ..tactics import ROLE_FEATURES, UtilityProfile


_BENEFIT_FEATURES = frozenset(
    {
        "capture_opportunity",
        "portal_time_advantage",
        "observation_gain",
        "pursuit_value",
        "base_progress",
        "visibility_loss",
        "alternative_exits",
        "escape_safety",
    }
)
_COST_FEATURES = frozenset({"capture_risk", "duration_cost"})
_ROLES = (Role.GUARDIAN, Role.EXPLORER)
_SPLITS = frozenset({"training", "validation", "held_out"})


class EvaluationSplit(str, Enum):
    TRAINING = "training"
    VALIDATION = "validation"
    HELD_OUT = "held_out"


@dataclass(frozen=True, slots=True)
class TacticalGenome:
    """Approved normalized utility weights plus bounded switching parameters."""

    weights: tuple[tuple[Role, str, float], ...]
    hysteresis_delta_u: float
    minimum_dwell_ns: int
    schema_version: int = 1

    def __post_init__(self) -> None:
        if self.schema_version != 1 or isinstance(self.schema_version, bool):
            raise ValueError("unsupported tactical genome schema version")
        if isinstance(self.hysteresis_delta_u, bool) or not isinstance(
            self.hysteresis_delta_u, (int, float)
        ):
            raise TypeError("hysteresis_delta_u must be numeric")
        if not math.isfinite(self.hysteresis_delta_u) or not (
            0.0 <= self.hysteresis_delta_u <= 1.0
        ):
            raise ValueError("hysteresis_delta_u must be within [0, 1]")
        if (
            not isinstance(self.minimum_dwell_ns, int)
            or isinstance(self.minimum_dwell_ns, bool)
            or not 0 <= self.minimum_dwell_ns <= 60_000_000_000
        ):
            raise ValueError("minimum_dwell_ns must be an integer in [0, 60 s]")
        parsed: dict[tuple[Role, str], float] = {}
        for role, feature, weight in self.weights:
            if not isinstance(role, Role):
                raise ValueError("genome roles must be Role values")
            if not isinstance(feature, str) or not feature:
                raise ValueError("genome feature names must be non-empty")
            if isinstance(weight, bool) or not isinstance(weight, (int, float)):
                raise TypeError("genome weights must be numeric")
            if not math.isfinite(float(weight)) or abs(float(weight)) > 1.0:
                raise ValueError("genome weights must be finite and in [-1, 1]")
            key = (role, feature)
            if key in parsed:
                raise ValueError("genome contains a duplicate role/feature gene")
            if feature not in ROLE_FEATURES[role]:
                raise ValueError(f"feature {feature} is not registered for {role.name}")
            if feature in _COST_FEATURES and weight > 0.0:
                raise ValueError(f"cost feature {feature} must have a non-positive weight")
            if feature in _BENEFIT_FEATURES and weight < 0.0:
                raise ValueError(f"benefit feature {feature} must have a non-negative weight")
            parsed[key] = float(weight)
        expected = {
            (role, feature)
            for role in _ROLES
            for feature in ROLE_FEATURES[role]
        }
        if set(parsed) != expected:
            raise ValueError("genome must define every registered feature for both roles")
        for role in _ROLES:
            mass = math.fsum(
                abs(parsed[(role, feature)]) for feature in ROLE_FEATURES[role]
            )
            if mass > 1.0 + 1e-12:
                raise ValueError(f"{role.name} utility coefficient mass exceeds one")
        object.__setattr__(
            self,
            "weights",
            tuple(
                (role, feature, parsed[(role, feature)])
                for role in _ROLES
                for feature in sorted(ROLE_FEATURES[role])
            ),
        )

    @classmethod
    def from_profiles(
        cls,
        guardian: UtilityProfile,
        explorer: UtilityProfile,
        *,
        hysteresis_delta_u: float = 0.0,
        minimum_dwell_ns: int = 0,
    ) -> "TacticalGenome":
        if guardian.role is not Role.GUARDIAN or explorer.role is not Role.EXPLORER:
            raise ValueError("baseline profiles must cover Guardian and Explorer roles")
        return cls(
            tuple(
                (profile.role, feature, weight)
                for profile in (guardian, explorer)
                for feature, weight in profile.weights
            ),
            hysteresis_delta_u,
            minimum_dwell_ns,
        )

    def utility_profile(self, role: Role, profile_id: str) -> UtilityProfile:
        if role not in _ROLES:
            raise ValueError("utility profile role is unsupported")
        return UtilityProfile(
            profile_id,
            role,
            tuple(
                (feature, weight)
                for item_role, feature, weight in self.weights
                if item_role is role
            ),
            self.schema_version,
        )

    @property
    def sha256(self) -> str:
        canonical = {
            "schema_version": self.schema_version,
            "weights": [
                [role.name, feature, value]
                for role, feature, value in self.weights
            ],
            "hysteresis_delta_u": self.hysteresis_delta_u,
            "minimum_dwell_ns": self.minimum_dwell_ns,
        }
        payload = json.dumps(
            canonical, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()


def mutate_genome(
    genome: TacticalGenome,
    *,
    sigma: float,
    seed: int,
    max_hysteresis_delta_u: float = 1.0,
    max_dwell_ns: int = 60_000_000_000,
) -> TacticalGenome:
    """Projected Gaussian mutation; projection preserves role/sign contracts."""
    if not math.isfinite(sigma) or sigma < 0.0:
        raise ValueError("mutation sigma must be finite and non-negative")
    if not math.isfinite(max_hysteresis_delta_u) or not (
        0.0 <= max_hysteresis_delta_u <= 1.0
    ):
        raise ValueError("maximum hysteresis must be within [0, 1]")
    if (
        not isinstance(seed, int)
        or isinstance(seed, bool)
        or seed < 0
        or not isinstance(max_dwell_ns, int)
        or isinstance(max_dwell_ns, bool)
        or not 0 <= max_dwell_ns <= 60_000_000_000
    ):
        raise ValueError("mutation seed or dwell bound is invalid")
    random = Random(seed)
    old = {(role, feature): value for role, feature, value in genome.weights}
    candidate: dict[tuple[Role, str], float] = {}
    for role in _ROLES:
        role_values: dict[str, float] = {}
        for feature in sorted(ROLE_FEATURES[role]):
            sign = -1.0 if feature in _COST_FEATURES else 1.0
            value = min(
                1.0,
                max(0.0, sign * old[(role, feature)] + random.gauss(0.0, sigma)),
            )
            role_values[feature] = sign * value
        mass = math.fsum(abs(value) for value in role_values.values())
        if mass > 1.0:
            role_values = {
                feature: value / mass for feature, value in role_values.items()
            }
        candidate.update({(role, feature): value for feature, value in role_values.items()})
    if max_hysteresis_delta_u == 0.0:
        hysteresis = 0.0
    else:
        hysteresis_fraction = min(
            1.0,
            max(
                0.0,
                genome.hysteresis_delta_u / max_hysteresis_delta_u
                + random.gauss(0.0, sigma),
            ),
        )
        hysteresis = hysteresis_fraction * max_hysteresis_delta_u
    if max_dwell_ns == 0:
        dwell = 0
    else:
        dwell_fraction = min(
            1.0,
            max(
                0.0,
                genome.minimum_dwell_ns / max_dwell_ns
                + random.gauss(0.0, sigma),
            ),
        )
        dwell = round(dwell_fraction * max_dwell_ns)
    return TacticalGenome(
        tuple(
            (role, feature, candidate[(role, feature)])
            for role in _ROLES
            for feature in ROLE_FEATURES[role]
        ),
        hysteresis,
        dwell,
    )


@dataclass(frozen=True, slots=True)
class EpisodeResult:
    policy_id: str
    scenario_id: str
    split: EvaluationSplit
    role: Role
    seed: int
    map_bank_id: str
    opponent_bank_id: str
    seed_bank_id: str
    training_return: float
    safety_overrides: int
    safety_violations: int
    collisions: int
    completed: bool
    evaluation_profile_id: str = "unversioned-profile"

    def __post_init__(self) -> None:
        for value, name in (
            (self.policy_id, "policy_id"),
            (self.scenario_id, "scenario_id"),
            (self.map_bank_id, "map_bank_id"),
            (self.opponent_bank_id, "opponent_bank_id"),
            (self.seed_bank_id, "seed_bank_id"),
            (self.evaluation_profile_id, "evaluation_profile_id"),
        ):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be non-empty")
        if not isinstance(self.split, EvaluationSplit):
            raise TypeError("split must be an EvaluationSplit")
        if not isinstance(self.role, Role):
            raise TypeError("role must be a Role")
        if not isinstance(self.seed, int) or isinstance(self.seed, bool) or self.seed < 0:
            raise ValueError("seed must be a non-negative integer")
        if isinstance(self.training_return, bool) or not isinstance(
            self.training_return, (int, float)
        ) or not math.isfinite(float(self.training_return)):
            raise ValueError("training_return must be finite")
        for value, name in (
            (self.safety_overrides, "safety_overrides"),
            (self.safety_violations, "safety_violations"),
            (self.collisions, "collisions"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if not isinstance(self.completed, bool):
            raise TypeError("completed must be boolean")

    @property
    def pair_key(self) -> tuple[str, int, str, str, str, str]:
        return (
            self.scenario_id,
            self.seed,
            self.map_bank_id,
            self.opponent_bank_id,
            self.seed_bank_id,
            self.evaluation_profile_id,
        )


@dataclass(frozen=True, slots=True)
class EvaluationPlan:
    plan_id: str
    confidence_level: float
    minimum_pairs: int
    maximum_safety_overrides_per_episode: int

    def __post_init__(self) -> None:
        if not isinstance(self.plan_id, str) or not self.plan_id.strip():
            raise ValueError("plan_id must be non-empty")
        if (
            isinstance(self.confidence_level, bool)
            or not isinstance(self.confidence_level, (int, float))
            or not math.isfinite(float(self.confidence_level))
            or not 0.0 < self.confidence_level < 1.0
        ):
            raise ValueError("confidence_level must be within (0, 1)")
        if (
            not isinstance(self.minimum_pairs, int)
            or isinstance(self.minimum_pairs, bool)
            or self.minimum_pairs < 2
        ):
            raise ValueError("minimum_pairs must be at least two")
        if (
            not isinstance(self.maximum_safety_overrides_per_episode, int)
            or isinstance(self.maximum_safety_overrides_per_episode, bool)
            or self.maximum_safety_overrides_per_episode < 0
        ):
            raise ValueError("maximum safety overrides must be non-negative")


@dataclass(frozen=True, slots=True)
class CandidateAssessment:
    candidate_id: str
    split: EvaluationSplit
    eligible: bool
    reason: str
    pair_count: int
    guardian_mean_delta: float | None
    explorer_mean_delta: float | None
    balanced_mean_delta: float | None
    confidence_interval: tuple[float, float] | None
    official_score_available: bool = False


@dataclass(frozen=True, slots=True)
class ValidationDecision:
    selected_policy_id: str
    candidate_selected: bool
    reason: str
    assessment: CandidateAssessment


@dataclass(frozen=True, slots=True)
class EvolutionConfig:
    population_size: int
    elite_count: int
    mutation_sigma: float
    seed: int
    maximum_generations: int

    def __post_init__(self) -> None:
        for value, field in (
            (self.population_size, "population_size"),
            (self.elite_count, "elite_count"),
            (self.seed, "seed"),
            (self.maximum_generations, "maximum_generations"),
        ):
            if not isinstance(value, int) or isinstance(value, bool):
                raise TypeError(f"{field} must be an integer")
        if self.population_size < 2 or not 1 <= self.elite_count < self.population_size:
            raise ValueError("population and elite counts are inconsistent")
        if self.seed < 0 or self.maximum_generations < 1:
            raise ValueError("seed and maximum_generations must be non-negative/positive")
        if (
            isinstance(self.mutation_sigma, bool)
            or not isinstance(self.mutation_sigma, (int, float))
            or not math.isfinite(float(self.mutation_sigma))
            or self.mutation_sigma <= 0.0
        ):
            raise ValueError("mutation_sigma must be finite and positive")


@dataclass(frozen=True, slots=True)
class EvolutionGeneration:
    generation_index: int
    genomes: tuple[TacticalGenome, ...]
    elite_sha256: tuple[str, ...]
    excluded_candidate_reasons: tuple[tuple[str, str], ...]


def create_next_generation(
    *,
    baseline_genome: TacticalGenome,
    evaluated_genomes: Iterable[tuple[TacticalGenome, CandidateAssessment]],
    config: EvolutionConfig,
    generation_index: int,
) -> EvolutionGeneration:
    """Rank only safety-eligible paired training results, retain baseline."""
    if (
        not isinstance(generation_index, int)
        or isinstance(generation_index, bool)
        or not 0 <= generation_index < config.maximum_generations
    ):
        raise ValueError("generation_index is outside the configured search budget")
    evaluated = tuple(evaluated_genomes)
    by_id: dict[str, tuple[TacticalGenome, CandidateAssessment]] = {}
    excluded: list[tuple[str, str]] = []
    for genome, assessment in evaluated:
        if not isinstance(genome, TacticalGenome) or not isinstance(
            assessment, CandidateAssessment
        ):
            raise TypeError("generation evaluations require genome/assessment pairs")
        if assessment.candidate_id != genome.sha256:
            raise ValueError("candidate assessment ID must equal genome SHA-256")
        if assessment.split is not EvaluationSplit.TRAINING:
            raise ValueError("generation fitness may use training data only")
        if genome.sha256 in by_id:
            raise ValueError("duplicate genome in generation evaluations")
        by_id[genome.sha256] = (genome, assessment)
        if not assessment.eligible:
            excluded.append((genome.sha256, assessment.reason))
    eligible = [
        (genome, assessment)
        for genome, assessment in by_id.values()
        if assessment.eligible and assessment.balanced_mean_delta is not None
    ]
    eligible.sort(
        key=lambda item: (
            -item[1].balanced_mean_delta,
            item[0].sha256,
        )
    )
    elites = tuple(genome for genome, _ in eligible[: config.elite_count])
    parents = elites or (baseline_genome,)
    next_genomes = [baseline_genome]
    child_index = 0
    while len(next_genomes) < config.population_size:
        parent = parents[child_index % len(parents)]
        child_seed = config.seed + generation_index * config.population_size + child_index
        next_genomes.append(
            mutate_genome(
                parent,
                sigma=float(config.mutation_sigma),
                seed=child_seed,
            )
        )
        child_index += 1
    unique: dict[str, TacticalGenome] = {}
    for genome in next_genomes:
        unique.setdefault(genome.sha256, genome)
    while len(unique) < config.population_size:
        child_seed = (
            config.seed
            + generation_index * config.population_size
            + child_index
        )
        child = mutate_genome(
            baseline_genome,
            sigma=float(config.mutation_sigma),
            seed=child_seed,
        )
        unique.setdefault(child.sha256, child)
        child_index += 1
    return EvolutionGeneration(
        generation_index,
        tuple(unique.values()),
        tuple(genome.sha256 for genome in elites),
        tuple(sorted(excluded)),
    )


def select_on_validation(
    baseline_id: str, assessment: CandidateAssessment
) -> ValidationDecision:
    """Use validation for selection only; does not accept/promote or test G5."""
    if not isinstance(baseline_id, str) or not baseline_id.strip():
        raise ValueError("baseline_id must be non-empty")
    if assessment.split is not EvaluationSplit.VALIDATION:
        raise ValueError("candidate selection requires validation-split results")
    if (
        assessment.eligible
        and assessment.confidence_interval is not None
        and assessment.confidence_interval[0] > 0.0
        and assessment.guardian_mean_delta is not None
        and assessment.guardian_mean_delta >= 0.0
        and assessment.explorer_mean_delta is not None
        and assessment.explorer_mean_delta >= 0.0
    ):
        return ValidationDecision(
            assessment.candidate_id,
            True,
            "validation_lower_confidence_bound_positive_review_required",
            assessment,
        )
    if (
        assessment.eligible
        and (
            (assessment.guardian_mean_delta is not None and assessment.guardian_mean_delta < 0.0)
            or (assessment.explorer_mean_delta is not None and assessment.explorer_mean_delta < 0.0)
        )
    ):
        return ValidationDecision(
            baseline_id,
            False,
            "baseline_retained_role_regression",
            assessment,
        )
    return ValidationDecision(
        baseline_id,
        False,
        "baseline_retained_candidate_not_supported_by_validation",
        assessment,
    )


def assess_paired_candidate(
    baseline: Iterable[EpisodeResult],
    candidate: Iterable[EpisodeResult],
    *,
    plan: EvaluationPlan,
    split: EvaluationSplit,
) -> CandidateAssessment:
    """Compare exact paired episode keys; never mixes splits or role counts."""
    if split is EvaluationSplit.HELD_OUT:
        raise ValueError("held-out episodes cannot be used for candidate selection")
    base_rows = tuple(baseline)
    candidate_rows = tuple(candidate)
    if not base_rows or not candidate_rows:
        return _ineligible("", split, "paired_results_missing")
    base_ids = {row.policy_id for row in base_rows}
    candidate_ids = {row.policy_id for row in candidate_rows}
    if len(base_ids) != 1 or len(candidate_ids) != 1:
        return _ineligible("", split, "mixed_policy_ids")
    candidate_id = next(iter(candidate_ids))
    if len(base_ids & candidate_ids):
        return _ineligible(candidate_id, split, "baseline_and_candidate_ids_must_differ")
    if any(row.split is not split for row in (*base_rows, *candidate_rows)):
        return _ineligible(candidate_id, split, "evaluation_split_mismatch")

    def index_rows(
        rows: tuple[EpisodeResult, ...]
    ) -> dict[tuple[tuple[str, int, str, str, str, str], Role], EpisodeResult] | None:
        indexed = {}
        for row in rows:
            key = (row.pair_key, row.role)
            if key in indexed:
                return None
            indexed[key] = row
        return indexed

    base_index = index_rows(base_rows)
    candidate_index = index_rows(candidate_rows)
    if base_index is None or candidate_index is None:
        return _ineligible(candidate_id, split, "duplicate_paired_episode_role")
    if set(base_index) != set(candidate_index):
        return _ineligible(candidate_id, split, "baseline_candidate_pairing_mismatch")
    pair_keys = sorted({key for key, _role in base_index})
    if len(pair_keys) < plan.minimum_pairs:
        return _ineligible(candidate_id, split, "minimum_paired_scenarios_not_met", len(pair_keys))

    balanced_deltas: list[float] = []
    role_deltas: dict[Role, list[float]] = {role: [] for role in _ROLES}
    for pair_key in pair_keys:
        deltas: dict[Role, float] = {}
        for role in _ROLES:
            key = (pair_key, role)
            if key not in base_index:
                return _ineligible(candidate_id, split, "both_roles_required_per_pair", len(pair_keys))
            base = base_index[key]
            trial = candidate_index[key]
            if (
                not base.completed
                or not trial.completed
                or trial.collisions > 0
                or trial.safety_violations > 0
                or trial.safety_overrides
                > plan.maximum_safety_overrides_per_episode
                or trial.safety_overrides > base.safety_overrides
            ):
                return _ineligible(
                    candidate_id,
                    split,
                    "hard_safety_or_episode_completion_exclusion",
                    len(pair_keys),
                )
            delta = float(trial.training_return) - float(base.training_return)
            deltas[role] = delta
            role_deltas[role].append(delta)
        balanced_deltas.append(math.fsum(deltas.values()) / len(_ROLES))

    balanced_mean = math.fsum(balanced_deltas) / len(balanced_deltas)
    interval = paired_student_t_interval(
        balanced_deltas, confidence_level=float(plan.confidence_level)
    )
    return CandidateAssessment(
        candidate_id=candidate_id,
        split=split,
        eligible=True,
        reason="paired_both_roles_safety_constraints_pass",
        pair_count=len(pair_keys),
        guardian_mean_delta=math.fsum(role_deltas[Role.GUARDIAN])
        / len(role_deltas[Role.GUARDIAN]),
        explorer_mean_delta=math.fsum(role_deltas[Role.EXPLORER])
        / len(role_deltas[Role.EXPLORER]),
        balanced_mean_delta=balanced_mean,
        confidence_interval=interval,
    )


def paired_student_t_interval(
    differences: Iterable[float], *, confidence_level: float
) -> tuple[float, float]:
    """Two-sided paired Student-t interval over independent scenario keys."""
    values = tuple(float(value) for value in differences)
    if len(values) < 2 or any(not math.isfinite(value) for value in values):
        raise ValueError("at least two finite paired differences are required")
    if (
        isinstance(confidence_level, bool)
        or not isinstance(confidence_level, (int, float))
        or not math.isfinite(float(confidence_level))
        or not 0.0 < confidence_level < 1.0
    ):
        raise ValueError("confidence_level must be within (0, 1)")
    mean = math.fsum(values) / len(values)
    variance = math.fsum((value - mean) ** 2 for value in values) / (len(values) - 1)
    standard_error = math.sqrt(variance / len(values))
    critical = float(
        student_t.ppf((1.0 + confidence_level) / 2.0, df=len(values) - 1)
    )
    if not math.isfinite(critical):
        raise ArithmeticError("Student-t critical value is not finite")
    half_width = critical * standard_error
    return mean - half_width, mean + half_width


def _ineligible(
    candidate_id: str,
    split: EvaluationSplit,
    reason: str,
    pair_count: int = 0,
) -> CandidateAssessment:
    return CandidateAssessment(
        candidate_id,
        split,
        False,
        reason,
        pair_count,
        None,
        None,
        None,
        None,
    )
