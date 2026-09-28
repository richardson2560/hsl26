"""Immutable Dirichlet posterior rows for offline option outcomes."""

from dataclasses import dataclass
import math
from typing import Iterable

from ..tactics import OptionOutcome


UINT64_MAX = (1 << 64) - 1
REQUIRED_FAILURE_OUTCOMES = frozenset(
    (
        OptionOutcome.CANCELED.name,
        OptionOutcome.TIMEOUT.name,
        OptionOutcome.PRECONDITION_FAILED.name,
        OptionOutcome.FEASIBILITY_LOST.name,
        OptionOutcome.SAFETY_STOP.name,
        OptionOutcome.STAGE_ENDED.name,
        OptionOutcome.INTERNAL_ERROR.name,
    )
)


def _identifier(value: str, field: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")


def _probability_parameters(
    alpha_prior: tuple[float, ...], counts: tuple[int, ...]
) -> tuple[float, ...]:
    if not alpha_prior or len(alpha_prior) != len(counts):
        raise ValueError("prior and count vectors must have the same nonzero length")
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not _finite_numeric(value)
        or value <= 0.0
        for value in alpha_prior
    ):
        raise ValueError("every Dirichlet prior concentration must be finite and positive")
    if any(
        not isinstance(value, int)
        or isinstance(value, bool)
        or value < 0
        or value > UINT64_MAX
        for value in counts
    ):
        raise ValueError("counts must be non-negative uint64 integers")
    posterior = tuple(float(alpha) + count for alpha, count in zip(alpha_prior, counts))
    try:
        total = math.fsum(posterior)
    except OverflowError as error:
        raise ValueError("posterior concentration sum is not finite") from error
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError("posterior concentration sum must be finite and positive")
    return posterior


def _finite_numeric(value: int | float) -> bool:
    try:
        return math.isfinite(float(value))
    except OverflowError:
        return False


@dataclass(frozen=True)
class TransitionPosterior:
    outcome_schema_id: str
    outcome_ids: tuple[str, ...]
    alpha: tuple[float, ...]
    mean: tuple[float, ...]
    variance: tuple[float, ...]
    counts: tuple[int, ...]


@dataclass(frozen=True)
class DirichletBeliefTransition:
    """One `(abstract state, option)` row with an immutable prior and counts."""

    state_id: str
    option_id: str
    outcome_schema_id: str
    outcome_ids: tuple[str, ...]
    alpha_prior: tuple[float, ...]
    outcome_counts: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        for value, field in (
            (self.state_id, "state_id"),
            (self.option_id, "option_id"),
            (self.outcome_schema_id, "outcome_schema_id"),
        ):
            _identifier(value, field)
        outcomes = tuple(self.outcome_ids)
        prior = tuple(self.alpha_prior)
        counts = (
            (0,) * len(outcomes)
            if not self.outcome_counts
            else tuple(self.outcome_counts)
        )
        if any(not isinstance(item, str) or not item.strip() for item in outcomes):
            raise ValueError("outcome IDs must be non-empty strings")
        if len(outcomes) != len(set(outcomes)):
            raise ValueError("outcome IDs must be unique")
        missing = REQUIRED_FAILURE_OUTCOMES - set(outcomes)
        if missing:
            raise ValueError(
                f"outcome schema must register failure/cancellation outcomes: {sorted(missing)}"
            )
        _probability_parameters(prior, counts)
        object.__setattr__(self, "outcome_ids", outcomes)
        object.__setattr__(self, "alpha_prior", prior)
        object.__setattr__(self, "outcome_counts", counts)

    @property
    def posterior_alpha(self) -> tuple[float, ...]:
        return _probability_parameters(self.alpha_prior, self.outcome_counts)

    def observe(self, outcome_id: str) -> "DirichletBeliefTransition":
        _identifier(outcome_id, "outcome_id")
        try:
            index = self.outcome_ids.index(outcome_id)
        except ValueError as error:
            raise ValueError(f"outcome_id is not registered: {outcome_id}") from error
        counts = list(self.outcome_counts)
        if counts[index] == UINT64_MAX:
            raise ValueError("outcome count would overflow uint64")
        counts[index] += 1
        return DirichletBeliefTransition(
            state_id=self.state_id,
            option_id=self.option_id,
            outcome_schema_id=self.outcome_schema_id,
            outcome_ids=self.outcome_ids,
            alpha_prior=self.alpha_prior,
            outcome_counts=tuple(counts),
        )

    def posterior_mean(self) -> tuple[float, ...]:
        alpha = self.posterior_alpha
        total = math.fsum(alpha)
        return tuple(value / total for value in alpha)

    def posterior_variance(self) -> tuple[float, ...]:
        alpha = self.posterior_alpha
        total = math.fsum(alpha)
        return tuple(
            (value / total) * (1.0 - value / total) / (total + 1.0)
            for value in alpha
        )

    def snapshot(self) -> TransitionPosterior:
        return TransitionPosterior(
            outcome_schema_id=self.outcome_schema_id,
            outcome_ids=self.outcome_ids,
            alpha=self.posterior_alpha,
            mean=self.posterior_mean(),
            variance=self.posterior_variance(),
            counts=self.outcome_counts,
        )


def make_dirichlet_row(
    *,
    state_id: str,
    option_id: str,
    outcome_schema_id: str,
    outcome_ids: Iterable[str],
    alpha_prior: Iterable[float],
) -> DirichletBeliefTransition:
    """Normalize caller sequences once and construct a validated immutable row."""

    return DirichletBeliefTransition(
        state_id=state_id,
        option_id=option_id,
        outcome_schema_id=outcome_schema_id,
        outcome_ids=tuple(outcome_ids),
        alpha_prior=tuple(alpha_prior),
    )
