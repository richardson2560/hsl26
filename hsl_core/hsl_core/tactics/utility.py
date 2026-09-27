"""Bounded, role-specific tactical utility with explicit feature semantics."""

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping

from ..match import Role


ROLE_FEATURES = MappingProxyType(
    {
        Role.GUARDIAN: frozenset(
            (
                "capture_opportunity",
                "portal_time_advantage",
                "observation_gain",
                "pursuit_value",
                "duration_cost",
            )
        ),
        Role.EXPLORER: frozenset(
            (
                "base_progress",
                "visibility_loss",
                "alternative_exits",
                "escape_safety",
                "observation_gain",
                "capture_risk",
                "duration_cost",
            )
        ),
    }
)

_COST_FEATURES = frozenset(("capture_risk", "duration_cost"))


@dataclass(frozen=True)
class UtilityProfile:
    """Immutable versioned coefficients; absolute coefficient mass is at most one."""

    profile_id: str
    role: Role
    weights: tuple[tuple[str, float], ...]
    schema_version: int

    def __post_init__(self) -> None:
        if not isinstance(self.profile_id, str) or not self.profile_id.strip():
            raise ValueError("profile_id must be non-empty")
        object.__setattr__(self, "role", _role(self.role))
        if (
            not isinstance(self.schema_version, int)
            or isinstance(self.schema_version, bool)
            or self.schema_version != 1
        ):
            raise ValueError("unsupported utility profile schema_version")
        if not isinstance(self.weights, (tuple, list)):
            raise ValueError("weights must be a sequence of name/value pairs")
        parsed: dict[str, float] = {}
        for entry in self.weights:
            if not isinstance(entry, (tuple, list)) or len(entry) != 2:
                raise ValueError("each weight must contain a feature name and coefficient")
            name, weight = entry
            if not isinstance(name, str) or not name:
                raise ValueError("weight feature names must be non-empty")
            if isinstance(weight, bool) or not isinstance(weight, (int, float)):
                raise ValueError(f"weight {name} must be numeric")
            if not math.isfinite(weight):
                raise ValueError(f"weight {name} must be finite")
            if name in parsed:
                raise ValueError(f"duplicate feature weight: {name}")
            parsed[name] = float(weight)
        expected = ROLE_FEATURES[self.role]
        if set(parsed) != expected:
            missing = sorted(expected - set(parsed))
            extra = sorted(set(parsed) - expected)
            raise ValueError(f"utility feature schema mismatch; missing={missing}, extra={extra}")
        for name, weight in parsed.items():
            if name in _COST_FEATURES and weight > 0.0:
                raise ValueError(f"cost feature {name} must have a non-positive weight")
            if name not in _COST_FEATURES and weight < 0.0:
                raise ValueError(f"benefit feature {name} must have a non-negative weight")
        if math.fsum(abs(weight) for weight in parsed.values()) > 1.0 + 1e-12:
            raise ValueError("sum of absolute utility weights must not exceed one")
        object.__setattr__(self, "weights", tuple(sorted(parsed.items())))


@dataclass(frozen=True)
class UtilityResult:
    value: float
    contributions: tuple[tuple[str, float], ...]
    profile_id: str
    schema_version: int

    def __post_init__(self) -> None:
        if (
            isinstance(self.value, bool)
            or not isinstance(self.value, (int, float))
            or not math.isfinite(self.value)
            or not -1.0 - 1e-12 <= self.value <= 1.0 + 1e-12
        ):
            raise ValueError("utility must be finite and within [-1, 1]")
        if not self.profile_id:
            raise ValueError("profile_id must be non-empty")
        if (
            not isinstance(self.schema_version, int)
            or isinstance(self.schema_version, bool)
            or self.schema_version != 1
        ):
            raise ValueError("unsupported utility result schema_version")
        if not isinstance(self.contributions, (tuple, list)):
            raise ValueError("contributions must be a sequence")
        names: set[str] = set()
        total: list[float] = []
        for entry in self.contributions:
            if not isinstance(entry, (tuple, list)) or len(entry) != 2:
                raise ValueError("each contribution must contain a name and value")
            name, value = entry
            if not isinstance(name, str) or not name or name in names:
                raise ValueError("contribution names must be non-empty and unique")
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError("utility contributions must be finite numbers")
            names.add(name)
            total.append(float(value))
        if not math.isclose(
            math.fsum(total), float(self.value), rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError("utility value does not equal the sum of contributions")
        object.__setattr__(self, "contributions", tuple(self.contributions))


def score(features: Mapping[str, float], parameters: UtilityProfile) -> UtilityResult:
    """Return a dimensionless dot product over complete normalized [0,1] features."""
    if not isinstance(parameters, UtilityProfile):
        raise ValueError("parameters must be a UtilityProfile")
    if not isinstance(features, Mapping):
        raise ValueError("features must be a mapping")
    expected = ROLE_FEATURES[parameters.role]
    actual = set(features)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise ValueError(f"utility feature set mismatch; missing={missing}, extra={extra}")
    weights = dict(parameters.weights)
    terms: list[tuple[str, float]] = []
    for name in sorted(expected):
        feature = features[name]
        if (
            isinstance(feature, bool)
            or not isinstance(feature, (int, float))
            or not math.isfinite(feature)
            or not 0.0 <= feature <= 1.0
        ):
            raise ValueError(f"feature {name} must be finite and normalized to [0, 1]")
        terms.append((name, weights[name] * float(feature)))
    value = math.fsum(contribution for _, contribution in terms)
    if not -1.0 - 1e-12 <= value <= 1.0 + 1e-12:
        raise ArithmeticError("bounded utility escaped its proven range")
    return UtilityResult(
        value=value,
        contributions=tuple(terms),
        profile_id=parameters.profile_id,
        schema_version=parameters.schema_version,
    )


def _role(value: Role | int) -> Role:
    if isinstance(value, Role):
        return value
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError("role must be a Role")
    try:
        return Role(value)
    except ValueError as error:
        raise ValueError("role must be a Role") from error
