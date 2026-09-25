# hsl_core/hsl_core/perception/registration.py
"""Bounded planar registration of a point candidate against a Hermite GPIS."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .implicit_surface import HermiteGPIS


def _finite_vector(values: Sequence[float], size: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite with shape ({size},)")
    return result


def _pose_transform(points_m: np.ndarray, pose: np.ndarray) -> np.ndarray:
    cosine = math.cos(float(pose[2]))
    sine = math.sin(float(pose[2]))
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    relative = points_m[:, :2] - pose[:2]
    model_xy = relative @ rotation
    return np.column_stack((model_xy, points_m[:, 2]))


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class RegistrationConfig:
    max_iterations: int = 20
    max_translation_step_m: float = 0.25
    max_yaw_step_rad: float = 0.35
    damping: float = 1e-4
    huber_delta: float = 2.5
    min_support_fraction: float = 0.5
    min_points: int = 6
    min_position_information: float = 1e-6
    min_yaw_information: float = 1e-6
    max_residual_rms: float = 5.0
    variance_floor: float = 1e-10

    def __post_init__(self) -> None:
        if self.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        if self.max_translation_step_m <= 0 or self.max_yaw_step_rad <= 0:
            raise ValueError("step bounds must be positive")
        if self.damping <= 0 or self.huber_delta <= 0:
            raise ValueError("damping and huber_delta must be positive")
        if not 0.0 < self.min_support_fraction <= 1.0:
            raise ValueError("min_support_fraction must be in (0, 1]")
        if self.min_points < 3 or self.min_position_information <= 0:
            raise ValueError("invalid minimum information configuration")
        if self.min_yaw_information < 0 or self.max_residual_rms <= 0:
            raise ValueError("invalid residual/information configuration")


@dataclass(frozen=True)
class RegistrationResult:
    status: str
    pose_xyyaw: tuple[float, float, float]
    covariance: tuple[float, ...]
    support_fraction: float
    inlier_count: int
    residual_rms: float
    position_valid: bool
    yaw_valid: bool
    iterations: int
    reason: str

    def __post_init__(self) -> None:
        if self.status not in ("ACCEPTED", "REJECTED", "TIMEOUT"):
            raise ValueError("invalid registration status")
        pose = _finite_vector(self.pose_xyyaw, 3, "pose_xyyaw")
        if not 0.0 <= self.support_fraction <= 1.0:
            raise ValueError("support_fraction must be in [0, 1]")
        if self.inlier_count < 0 or self.iterations < 0:
            raise ValueError("counts must be non-negative")
        if not math.isfinite(self.residual_rms) or self.residual_rms < 0:
            raise ValueError("residual_rms must be finite and non-negative")
        covariance = np.asarray(self.covariance, dtype=float)
        if covariance.shape != (9,) or not np.all(np.isfinite(covariance)):
            raise ValueError("covariance must contain 9 finite values")
        if not self.reason:
            raise ValueError("reason must not be empty")
        object.__setattr__(self, "pose_xyyaw", tuple(float(v) for v in pose))


class PlanarRegistrar:
    """IRLS/Gauss-Newton registrar with bounded steps and typed rejection."""

    def __init__(self, model: HermiteGPIS, config: RegistrationConfig = RegistrationConfig()) -> None:
        self.model = model
        self.config = config

    def register(
        self,
        points_m: Sequence[Sequence[float]],
        *,
        initial_pose_xyyaw: Sequence[float],
        point_variance_m2: float = 1e-4,
    ) -> RegistrationResult:
        points = np.asarray(points_m, dtype=float)
        pose = _finite_vector(initial_pose_xyyaw, 3, "initial_pose_xyyaw")
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise ValueError("points_m must have shape (N, 3) and be finite")
        if len(points) < self.config.min_points:
            return self._rejected(pose, "INSUFFICIENT_POINTS")
        if not math.isfinite(point_variance_m2) or point_variance_m2 <= 0:
            raise ValueError("point_variance_m2 must be positive and finite")
        support = self.model.support_fraction(_pose_transform(points, pose))
        if support < self.config.min_support_fraction:
            return self._rejected(pose, "INSUFFICIENT_SUPPORT", support)
        accepted = False
        iterations = 0
        last = None
        for iterations in range(1, self.config.max_iterations + 1):
            model_points = _pose_transform(points, pose)
            means = []
            variances = []
            gradients = []
            for point in model_points:
                mean, variance, gradient = self.model.evaluate(point)
                means.append(mean)
                variances.append(variance)
                gradients.append(gradient)
            means_array = np.asarray(means)
            gradients_array = np.asarray(gradients)
            sigma_sq = np.asarray(variances) + point_variance_m2
            if np.any(sigma_sq <= 0) or not np.all(np.isfinite(sigma_sq)):
                return self._rejected(pose, "INVALID_FIELD_UNCERTAINTY", support, iterations)
            residual = means_array / np.sqrt(sigma_sq)
            abs_residual = np.abs(residual)
            weights = np.ones(len(points))
            outlier = abs_residual > self.config.huber_delta
            weights[outlier] = self.config.huber_delta / abs_residual[outlier]
            jacobian = np.zeros((len(points), 3))
            for index, (model_point, gradient) in enumerate(zip(model_points, gradients_array)):
                cosine = math.cos(float(pose[2]))
                sine = math.sin(float(pose[2]))
                rotation_transpose = np.array([[cosine, sine], [-sine, cosine]])
                dz_dp = np.zeros((3, 2))
                dz_dp[:2] = -rotation_transpose
                jz = np.array([-model_point[1], model_point[0], 0.0])
                dz_dyaw = -jz
                jacobian[index, :2] = gradient @ dz_dp / math.sqrt(sigma_sq[index])
                jacobian[index, 2] = float(gradient @ dz_dyaw) / math.sqrt(sigma_sq[index])
            weighted = weights[:, None]
            normal = jacobian.T @ (weighted * jacobian)
            normal += np.eye(3) * self.config.damping
            rhs = -jacobian.T @ (weights * residual)
            try:
                step = np.linalg.solve(normal, rhs)
            except np.linalg.LinAlgError:
                return self._rejected(pose, "SINGULAR_INFORMATION", support, iterations)
            step[:2] = np.clip(step[:2], -self.config.max_translation_step_m, self.config.max_translation_step_m)
            step[2] = float(np.clip(step[2], -self.config.max_yaw_step_rad, self.config.max_yaw_step_rad))
            pose_candidate = pose + step
            pose_candidate[2] = _wrap_angle(float(pose_candidate[2]))
            candidate_points = _pose_transform(points, pose_candidate)
            candidate_support = self.model.support_fraction(candidate_points)
            if candidate_support < self.config.min_support_fraction:
                step *= 0.5
                pose_candidate = pose + step
                pose_candidate[2] = _wrap_angle(float(pose_candidate[2]))
            pose = pose_candidate
            support = self.model.support_fraction(_pose_transform(points, pose))
            last = (normal, residual, weights)
            if float(np.linalg.norm(step)) < 1e-6:
                accepted = True
                break
        if last is None:
            return self._rejected(pose, "NO_ITERATION", support, iterations)
        normal, residual, weights = last
        rms = float(math.sqrt(np.average(residual**2, weights=weights)))
        information = normal - np.eye(3) * self.config.damping
        position_information = float(np.min(np.linalg.eigvalsh(information[:2, :2])))
        yaw_information = float(information[2, 2])
        yaw_valid = yaw_information >= self.config.min_yaw_information
        position_valid = (
            accepted
            and support >= self.config.min_support_fraction
            and position_information >= self.config.min_position_information
            and rms <= self.config.max_residual_rms
        )
        try:
            covariance = np.linalg.pinv(normal).reshape(-1)
        except np.linalg.LinAlgError:
            covariance = np.full(9, math.inf)
        status = "ACCEPTED" if position_valid else "REJECTED"
        reason = "OK" if position_valid else "WEAK_GEOMETRY_OR_RESIDUAL"
        if not yaw_valid:
            reason = "POSITION_ONLY_YAW_UNOBSERVABLE" if position_valid else reason
        return RegistrationResult(
            status,
            tuple(pose),
            tuple(float(v) for v in covariance),
            support,
            int(np.sum(weights > 0.25)),
            rms,
            position_valid,
            yaw_valid,
            iterations,
            reason,
        )

    def _rejected(
        self,
        pose: np.ndarray,
        reason: str,
        support: float = 0.0,
        iterations: int = 0,
    ) -> RegistrationResult:
        return RegistrationResult(
            "REJECTED", tuple(pose), tuple(float(v) for v in np.full(9, 1e12)),
            support, 0, 0.0,
            False, False, iterations, reason,
        )
