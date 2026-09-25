# hsl_core/hsl_core/perception/implicit_surface.py
"""Compactly supported Hermite-GPIS-W prior.

The implementation follows the normative HSL26 kernel contract:
Wendland C2 in three dimensions, value and directional-derivative
observations, explicit support checks, and Cholesky-backed prediction.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


KERNEL_ID = "wendland_c4_d3_unit_center_v1"
SCHEMA_VERSION = "hsl26.opponent-gpis.v1"


def _array(values: Sequence[float], shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    return result


def _unit(vector: Sequence[float], name: str) -> np.ndarray:
    value = _array(vector, (3,), name)
    norm = float(np.linalg.norm(value))
    if norm <= 1e-12:
        raise ValueError(f"{name} must be non-zero")
    return value / norm


def _kernel_profile(u: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the TS §8.1 Wendland C4 profile and its first two derivatives."""
    if np.any(~np.isfinite(u)):
        raise ValueError("kernel arguments must be finite")
    phi = np.zeros_like(u)
    first = np.zeros_like(u)
    second = np.zeros_like(u)
    inside = (u >= 0.0) & (u < 1.0)
    v = u[inside]
    one_minus = 1.0 - v
    # phi(u) = (1/3) (1-u)^6 (35u^2 + 18u + 3).
    phi[inside] = (one_minus**6 * (35.0 * v**2 + 18.0 * v + 3.0)) / 3.0
    first[inside] = (-56.0 / 3.0) * v * (5.0 * v + 1.0) * one_minus**5
    second[inside] = (56.0 / 3.0) * one_minus**4 * (35.0 * v**2 - 4.0 * v - 1.0)
    phi[u == 0.0] = 1.0
    first[u == 0.0] = 0.0
    second[u == 0.0] = -56.0 / 3.0
    return phi, first, second


def wendland_c4(
    points_a: Sequence[Sequence[float]],
    points_b: Sequence[Sequence[float]],
    *,
    support_radius_m: float,
    amplitude: float = 1.0,
) -> np.ndarray:
    """Evaluate the compact positive-definite scalar covariance block."""
    a = np.asarray(points_a, dtype=float)
    b = np.asarray(points_b, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3 or b.ndim != 2 or b.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise ValueError("points must be finite")
    if not math.isfinite(support_radius_m) or support_radius_m <= 0.0:
        raise ValueError("support_radius_m must be positive and finite")
    if not math.isfinite(amplitude) or amplitude <= 0.0:
        raise ValueError("amplitude must be positive and finite")
    distances = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    phi, _, _ = _kernel_profile(distances / support_radius_m)
    return amplitude * phi


def _radial_derivatives(
    difference: np.ndarray,
    *,
    support_radius_m: float,
    amplitude: float,
) -> tuple[float, np.ndarray, np.ndarray]:
    """Return k, grad_x k and Hessian_x k for one pair of 3-D points."""
    distance = float(np.linalg.norm(difference))
    u = np.asarray([distance / support_radius_m])
    phi, first, second = _kernel_profile(u)
    value = float(amplitude * phi[0])
    if distance <= 1e-12:
        gradient = np.zeros(3)
        hessian = np.eye(3) * ((-56.0 / 3.0) * amplitude / support_radius_m**2)
        return value, gradient, hessian
    if distance >= support_radius_m:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    direction = difference / distance
    radial_first = amplitude * first[0] / support_radius_m
    radial_second = amplitude * second[0] / support_radius_m**2
    gradient = radial_first * direction
    hessian = (
        radial_second * np.outer(direction, direction)
        + (radial_first / distance) * (np.eye(3) - np.outer(direction, direction))
    )
    return value, gradient, hessian


@dataclass(frozen=True)
class HermiteObservation:
    """One value or directional derivative observation in model coordinates."""

    point_m: tuple[float, float, float]
    kind: str
    direction: tuple[float, float, float]
    value: float
    noise_variance: float

    def __post_init__(self) -> None:
        point = _array(self.point_m, (3,), "point_m")
        object.__setattr__(self, "point_m", tuple(float(v) for v in point))
        if self.kind not in ("value", "derivative"):
            raise ValueError("kind must be 'value' or 'derivative'")
        direction = _unit(self.direction, "direction")
        object.__setattr__(self, "direction", tuple(float(v) for v in direction))
        if not math.isfinite(self.value):
            raise ValueError("value must be finite")
        if not math.isfinite(self.noise_variance) or self.noise_variance <= 0.0:
            raise ValueError("noise_variance must be positive and finite")
        if self.kind == "value" and np.linalg.norm(direction) <= 0.0:
            raise ValueError("direction must remain valid for schema consistency")


def hermite_covariance(
    observations: Sequence[HermiteObservation],
    *,
    support_radius_m: float,
    amplitude: float,
) -> np.ndarray:
    """Build K + diagonal observation noise using both Hermite operators."""
    if not observations:
        raise ValueError("at least one observation is required")
    matrix = np.zeros((len(observations), len(observations)), dtype=float)
    for row, first in enumerate(observations):
        x = np.asarray(first.point_m)
        na = np.asarray(first.direction)
        for column, second in enumerate(observations):
            xp = np.asarray(second.point_m)
            nb = np.asarray(second.direction)
            value, gradient, hessian = _radial_derivatives(
                x - xp,
                support_radius_m=support_radius_m,
                amplitude=amplitude,
            )
            if first.kind == "value" and second.kind == "value":
                block = value
            elif first.kind == "derivative" and second.kind == "value":
                block = float(na @ gradient)
            elif first.kind == "value" and second.kind == "derivative":
                block = float(-gradient @ nb)
            else:
                # grad_x grad_x' k = - Hessian_x k for a stationary kernel.
                block = float(-na @ hessian @ nb)
            matrix[row, column] = block
        matrix[row, row] += first.noise_variance
    matrix = 0.5 * (matrix + matrix.T)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("Hermite covariance contains non-finite values")
    return matrix


class HermiteGPIS:
    """Fitted compact-support implicit field with conservative support checks."""

    def __init__(
        self,
        observations: Sequence[HermiteObservation],
        *,
        support_radius_m: float,
        amplitude: float = 1.0,
        regularization: float = 0.0,
        model_frame: str = "opponent_model",
    ) -> None:
        if not observations:
            raise ValueError("at least one observation is required")
        if not math.isfinite(regularization) or regularization < 0.0:
            raise ValueError("regularization must be finite and non-negative")
        self.observations = tuple(observations)
        self.support_radius_m = float(support_radius_m)
        self.amplitude = float(amplitude)
        self.regularization = float(regularization)
        self.model_frame = model_frame
        if not model_frame:
            raise ValueError("model_frame must not be empty")
        covariance = hermite_covariance(
            self.observations,
            support_radius_m=self.support_radius_m,
            amplitude=self.amplitude,
        )
        if regularization:
            covariance += np.eye(len(covariance)) * regularization
        try:
            self._cholesky = np.linalg.cholesky(covariance)
        except np.linalg.LinAlgError as exc:
            raise ValueError("regularized Hermite covariance is not positive definite") from exc
        values = np.asarray([item.value for item in self.observations], dtype=float)
        self._alpha = np.linalg.solve(
            self._cholesky.T,
            np.linalg.solve(self._cholesky, values),
        )
        self._covariance = covariance

    @property
    def coefficients(self) -> np.ndarray:
        return self._alpha.copy()

    @property
    def condition_number(self) -> float:
        return float(np.linalg.cond(self._covariance))

    def _query_covariance(self, point_m: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        point = _array(point_m, (3,), "point_m")
        vector = np.zeros(len(self.observations), dtype=float)
        gradients = np.zeros((len(self.observations), 3), dtype=float)
        for index, observation in enumerate(self.observations):
            value, gradient, _ = _radial_derivatives(
                point - np.asarray(observation.point_m),
                support_radius_m=self.support_radius_m,
                amplitude=self.amplitude,
            )
            if observation.kind == "value":
                vector[index] = value
                gradients[index] = gradient
            else:
                direction = np.asarray(observation.direction)
                vector[index] = -float(gradient @ direction)
                # Derivative of a query-observation covariance is evaluated
                # analytically by the model's finite-difference-free gradient.
                _, _, hessian = _radial_derivatives(
                    point - np.asarray(observation.point_m),
                    support_radius_m=self.support_radius_m,
                    amplitude=self.amplitude,
                )
                gradients[index] = -hessian @ direction
        return vector, gradients

    def evaluate(self, point_m: Sequence[float]) -> tuple[float, float, np.ndarray]:
        """Return posterior mean, variance and spatial gradient at a query."""
        point = _array(point_m, (3,), "point_m")
        covariance_vector, covariance_gradients = self._query_covariance(point)
        mean = float(covariance_vector @ self._alpha)
        solved = np.linalg.solve(self._cholesky, covariance_vector)
        variance = float(self.amplitude - solved @ solved)
        tolerance = 1e-10 * max(1.0, self.amplitude)
        if variance < -tolerance:
            raise ValueError("posterior variance is materially negative")
        variance = max(0.0, variance)
        gradient = covariance_gradients.T @ self._alpha
        return mean, variance, gradient

    def support_fraction(self, points_m: Iterable[Sequence[float]]) -> float:
        points = np.asarray(list(points_m), dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        if len(points) == 0:
            return 0.0
        centers = np.asarray([item.point_m for item in self.observations])
        distances = np.linalg.norm(points[:, None, :] - centers[None, :, :], axis=2)
        return float(np.mean(np.any(distances < self.support_radius_m, axis=1)))

    def to_arrays(self) -> dict[str, np.ndarray]:
        return {
            "support_points_m": np.asarray([item.point_m for item in self.observations]),
            "operator_kind": np.asarray([0 if item.kind == "value" else 1 for item in self.observations], dtype=np.int8),
            "operator_direction": np.asarray([item.direction for item in self.observations]),
            "observations": np.asarray([item.value for item in self.observations]),
            "noise_variance": np.asarray([item.noise_variance for item in self.observations]),
            "alpha": self._alpha.copy(),
            "cholesky": self._cholesky.copy(),
            "support_radius_m": np.asarray([self.support_radius_m]),
            "amplitude": np.asarray([self.amplitude]),
            "regularization": np.asarray([self.regularization]),
            "base_from_model": np.eye(4),
            "normalization_center_m": np.zeros(3),
            "normalization_scale": np.ones(3),
        }

    def save(self, directory: str | Path, *, source_hashes: dict[str, str] | None = None) -> dict:
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        model_path = path / "model.npz"
        np.savez_compressed(model_path, **self.to_arrays())
        digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "model_id": f"opponent-gpis-{digest[:12]}",
            "kernel_id": KERNEL_ID,
            "field_units": "metres",
            "coordinate_units": "metres",
            "support_radius_m": self.support_radius_m,
            "model_frame": self.model_frame,
            "base_from_model": np.eye(4).tolist(),
            "source_hashes": source_hashes or {},
            "training_config_hash": hashlib.sha256(
                json.dumps(
                    {"support_radius_m": self.support_radius_m, "amplitude": self.amplitude,
                     "regularization": self.regularization},
                    sort_keys=True,
                ).encode("utf-8")
            ).hexdigest(),
            "array_shapes": {key: list(value.shape) for key, value in self.to_arrays().items()},
            "artifact_sha256": digest,
            "uncertainty_method": "exact_cholesky",
            "supported_view_domain": {"support_radius_m": self.support_radius_m},
            "created_by_version": "hsl26-p4.1",
            "condition_number": self.condition_number,
        }
        (path / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        return manifest
