"""Exact continuous-time SMDP option returns and tabular value backups."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable

from .transitions import (
    CensoringStatus,
    FeatureSchema,
    OptionKind,
    OptionTransition,
)


class KernelEvidenceKind(str, Enum):
    EXACT_SPECIFICATION = "exact_specification"
    EMPIRICAL_ESTIMATE = "empirical_estimate"


@dataclass(frozen=True, slots=True)
class DiscountedReturn:
    reward: float
    duration_s: float
    discount: float


@dataclass(frozen=True, slots=True)
class ValueEntry:
    state_id: str
    option: OptionKind
    value: float
    visit_count: int


@dataclass(frozen=True, slots=True)
class ValueUpdate:
    state_id: str
    option: OptionKind
    old_value: float
    target: float
    new_value: float
    eta: float
    eta_schedule_id: str


class OptionValueTable:
    """In-memory tabular Q(S, option); no policy is inferred from censored data."""

    def __init__(self, schema: FeatureSchema, options: Iterable[OptionKind]) -> None:
        if not isinstance(schema, FeatureSchema):
            raise TypeError("schema must be a FeatureSchema")
        self.schema = schema
        supplied_options = tuple(options)
        if any(not isinstance(option, OptionKind) for option in supplied_options):
            raise TypeError("registered options must be OptionKind values")
        self.options = tuple(sorted(set(supplied_options), key=lambda option: option.value))
        if not self.options:
            raise ValueError("at least one option must be registered")
        self._values: dict[tuple[str, OptionKind], float] = {}
        self._visits: dict[tuple[str, OptionKind], int] = {}

    def value(self, state_id: str, option: OptionKind) -> float:
        if option not in self.options:
            raise ValueError("option is not registered in this value table")
        return self._values.get((state_id, option), 0.0)

    def snapshot(self) -> tuple[ValueEntry, ...]:
        return tuple(
            ValueEntry(state_id, option, value, self._visits[(state_id, option)])
            for (state_id, option), value in sorted(
                self._values.items(),
                key=lambda item: (item[0][0], item[0][1].value),
            )
        )

    def update(
        self,
        transition: OptionTransition,
        *,
        beta_per_s: float,
        eta: float,
        eta_schedule_id: str,
    ) -> ValueUpdate:
        """Apply one Q-learning backup; eta is an explicit value-step size."""
        _validate_learning_parameters(beta_per_s, eta, eta_schedule_id)
        if transition.censoring is not CensoringStatus.COMPLETE:
            raise ValueError("censored transitions cannot be used for Bellman updates")
        transition.input_state.validate(self.schema)
        if transition.option_kind not in self.options:
            raise ValueError("transition option is not registered in value table")

        discounted = discounted_return(transition, beta_per_s=beta_per_s)
        bootstrap = 0.0
        if not transition.terminal:
            if transition.next_state is None:
                raise ValueError("nonterminal transition requires a next state")
            transition.next_state.validate(self.schema)
            if not transition.available_next_options:
                raise ValueError("nonterminal transition requires available next options")
            bootstrap = discounted.discount * max(
                self.value(transition.next_state.state_id, option)
                for option in transition.available_next_options
            )

        target = discounted.reward + bootstrap
        if not math.isfinite(target):
            raise ArithmeticError("Bellman target is not finite")
        key = (transition.input_state.state_id, transition.option_kind)
        old_value = self._values.get(key, 0.0)
        new_value = old_value + eta * (target - old_value)
        if not math.isfinite(new_value):
            raise ArithmeticError("updated option value is not finite")
        self._values[key] = new_value
        self._visits[key] = self._visits.get(key, 0) + 1
        return ValueUpdate(
            state_id=key[0],
            option=key[1],
            old_value=old_value,
            target=target,
            new_value=new_value,
            eta=eta,
            eta_schedule_id=eta_schedule_id,
        )


def discounted_return(
    transition: OptionTransition, *, beta_per_s: float
) -> DiscountedReturn:
    """Integrate piecewise-constant reward rates exactly over an option."""
    beta = _finite_float(beta_per_s, "discount rate beta")
    if beta < 0.0:
        raise ValueError("discount rate beta must be finite and non-negative")
    if transition.censoring is not CensoringStatus.COMPLETE:
        raise ValueError("censored transitions have no complete option return")

    duration = transition.duration_ns / 1_000_000_000.0
    result_terms: list[float] = []
    for component in transition.reward_components:
        for segment in component.rate_segments:
            start = segment.start_ns / 1_000_000_000.0
            end = segment.end_ns / 1_000_000_000.0
            if beta == 0.0:
                factor = end - start
            else:
                # expm1 avoids cancellation for short intervals or small beta.
                factor = (
                    math.exp(-beta * start)
                    * (-math.expm1(-beta * (end - start)))
                    / beta
                )
            result_terms.append(segment.rate_per_s * factor)
        if component.terminal_impulse != 0.0:
            result_terms.append(
                math.exp(-beta * duration) * component.terminal_impulse
            )
    reward = math.fsum(result_terms)
    discount = math.exp(-beta * duration)
    if not math.isfinite(reward) or not math.isfinite(discount):
        raise ArithmeticError("discounted return overflowed")
    return DiscountedReturn(reward, duration, discount)


def _validate_learning_parameters(
    beta_per_s: float, eta: float, eta_schedule_id: str
) -> None:
    beta = _finite_float(beta_per_s, "discount rate beta")
    step = _finite_float(eta, "eta")
    if beta < 0.0:
        raise ValueError("discount rate beta must be finite and non-negative")
    if not 0.0 < step <= 1.0:
        raise ValueError("eta must be finite and in (0, 1]")
    if not isinstance(eta_schedule_id, str) or not eta_schedule_id.strip():
        raise ValueError("eta schedule must have a non-empty identifier")


def _finite_float(value: int | float, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{field} must be numeric")
    try:
        result = float(value)
    except OverflowError as error:
        raise ValueError(f"{field} must be finite") from error
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _is_finite(value: int | float) -> bool:
    try:
        return math.isfinite(float(value))
    except OverflowError:
        return False


@dataclass(frozen=True, slots=True)
class KernelAtom:
    probability: float
    duration_s: float
    discounted_reward: float
    next_class_id: str

    def __post_init__(self) -> None:
        values = (self.probability, self.duration_s, self.discounted_reward)
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
            raise TypeError("kernel atom values must be numeric")
        if any(not _is_finite(value) for value in values):
            raise ValueError("kernel atom values must be finite")
        if not 0.0 <= self.probability <= 1.0 or self.duration_s < 0.0:
            raise ValueError("kernel probability or duration is outside its valid range")
        if not isinstance(self.next_class_id, str) or not self.next_class_id.strip():
            raise ValueError("kernel next-class ID must be non-empty")


@dataclass(frozen=True, slots=True)
class ExactOptionModel:
    """A declared finite SMDP model; empirical samples are never exact evidence."""

    state_id: str
    available_options: tuple[OptionKind, ...]
    option_kernels: tuple[tuple[OptionKind, tuple[KernelAtom, ...]], ...]
    model_id: str
    evidence_kind: KernelEvidenceKind

    def __post_init__(self) -> None:
        available_options = tuple(self.available_options)
        option_kernels = tuple(
            (option, tuple(atoms)) for option, atoms in self.option_kernels
        )
        if not isinstance(self.state_id, str) or not self.state_id.strip():
            raise ValueError("model state ID must be non-empty")
        if not isinstance(self.model_id, str) or not self.model_id.strip():
            raise ValueError("model ID must be non-empty")
        if not isinstance(self.evidence_kind, KernelEvidenceKind):
            raise TypeError("evidence_kind must be a KernelEvidenceKind")
        if not available_options or any(
            not isinstance(option, OptionKind) for option in available_options
        ):
            raise ValueError("model must declare available options")
        if len(set(available_options)) != len(available_options):
            raise ValueError("available options must not contain duplicates")
        kernels = dict(option_kernels)
        if len(kernels) != len(option_kernels):
            raise ValueError("each option must have exactly one kernel")
        if set(kernels) != set(available_options):
            raise ValueError("kernel options must equal the available option set")
        for atoms in kernels.values():
            if not atoms:
                raise ValueError("each option kernel must contain at least one atom")
            if any(not isinstance(atom, KernelAtom) for atom in atoms):
                raise TypeError("option kernels must contain KernelAtom values")
            total = math.fsum(atom.probability for atom in atoms)
            if not math.isclose(total, 1.0, rel_tol=0.0, abs_tol=1e-12):
                raise ValueError("kernel probabilities must sum to one")
        ordered_options = tuple(sorted(available_options, key=lambda item: item.value))
        object.__setattr__(self, "available_options", ordered_options)
        object.__setattr__(
            self,
            "option_kernels",
            tuple((option, tuple(kernels[option])) for option in ordered_options),
        )


@dataclass(frozen=True, slots=True)
class FusionAssessment:
    equivalent: bool
    reason: str
    max_reward_difference: float | None
    max_kernel_difference: float | None


def assess_exact_option_fusion(
    left: ExactOptionModel,
    right: ExactOptionModel,
    *,
    beta_per_s: float,
    tolerance: float = 1e-12,
) -> FusionAssessment:
    """Check reward and discounted successor kernels, not mean duration alone."""
    beta = _finite_float(beta_per_s, "discount rate beta")
    tolerance_value = _finite_float(tolerance, "tolerance")
    if beta < 0.0:
        raise ValueError("discount rate beta must be finite and non-negative")
    if tolerance_value < 0.0:
        raise ValueError("tolerance must be finite and non-negative")
    if (
        left.evidence_kind is not KernelEvidenceKind.EXACT_SPECIFICATION
        or right.evidence_kind is not KernelEvidenceKind.EXACT_SPECIFICATION
    ):
        return FusionAssessment(False, "empirical_estimates_cannot_prove_exact_fusion", None, None)
    if set(left.available_options) != set(right.available_options):
        return FusionAssessment(False, "available_option_sets_differ", None, None)

    reward_difference = 0.0
    kernel_difference = 0.0
    left_kernels = dict(left.option_kernels)
    right_kernels = dict(right.option_kernels)
    for option in left.available_options:
        l_atoms = left_kernels[option]
        r_atoms = right_kernels[option]
        left_reward = math.fsum(atom.probability * atom.discounted_reward for atom in l_atoms)
        right_reward = math.fsum(atom.probability * atom.discounted_reward for atom in r_atoms)
        reward_difference = max(reward_difference, abs(left_reward - right_reward))
        targets = {atom.next_class_id for atom in l_atoms} | {
            atom.next_class_id for atom in r_atoms
        }
        for target in targets:
            left_mass = math.fsum(
                atom.probability * math.exp(-beta * atom.duration_s)
                for atom in l_atoms
                if atom.next_class_id == target
            )
            right_mass = math.fsum(
                atom.probability * math.exp(-beta * atom.duration_s)
                for atom in r_atoms
                if atom.next_class_id == target
            )
            kernel_difference = max(kernel_difference, abs(left_mass - right_mass))

    equivalent = max(reward_difference, kernel_difference) <= tolerance_value
    return FusionAssessment(
        equivalent,
        "discounted_reward_and_successor_kernels_match"
        if equivalent
        else "discounted_reward_or_successor_kernel_differs",
        reward_difference,
        kernel_difference,
    )
