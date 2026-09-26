# hsl_core/hsl_core/perception/ekf_opponent.py
"""P4.3 constant-velocity opponent filter and association contract."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from ..types import TrackState


def _finite_vector(values: Sequence[float], size: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite with shape ({size},)")
    return result


def _finite_matrix(values: Sequence[float] | np.ndarray, shape: tuple[int, int], name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape == (shape[0] * shape[1],):
        result = result.reshape(shape)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    return result


@dataclass(frozen=True)
class FilterConfig:
    acceleration_noise_m2ps3: float = 0.5
    innovation_gate: float = 5.991464547
    confirmation_count: int = 2
    coasting_timeout_s: float = 0.5
    belief_timeout_s: float = 2.0
    lost_timeout_s: float = 5.0
    ambiguity_margin: float = 1.0
    max_prediction_dt_s: float = 1.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.acceleration_noise_m2ps3) or self.acceleration_noise_m2ps3 <= 0:
            raise ValueError("acceleration_noise_m2ps3 must be positive and finite")
        if not math.isfinite(self.innovation_gate) or self.innovation_gate <= 0:
            raise ValueError("innovation_gate must be positive and finite")
        if self.confirmation_count < 1:
            raise ValueError("confirmation_count must be positive")
        for name in ("coasting_timeout_s", "belief_timeout_s", "lost_timeout_s", "max_prediction_dt_s"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not 0.0 < self.ambiguity_margin <= 1.0:
            raise ValueError("ambiguity_margin must be in (0, 1]")
        if not (
            self.coasting_timeout_s < self.belief_timeout_s < self.lost_timeout_s
        ):
            raise ValueError("lifecycle timeouts must be strictly ordered")


@dataclass(frozen=True)
class OpponentDetection:
    """Position-only candidate with explicit source and temporal metadata."""

    position_xy_m: tuple[float, float]
    covariance_xy_m2: tuple[float, float, float, float]
    stamp_s: float
    source_id: str
    map_version: int
    localization_epoch: str
    yaw_valid: bool = False

    def __post_init__(self) -> None:
        _finite_vector(self.position_xy_m, 2, "position_xy_m")
        covariance = _finite_matrix(self.covariance_xy_m2, (2, 2), "covariance_xy_m2")
        covariance = 0.5 * (covariance + covariance.T)
        if np.min(np.linalg.eigvalsh(covariance)) <= 0.0:
            raise ValueError("covariance_xy_m2 must be positive definite")
        object.__setattr__(self, "covariance_xy_m2", tuple(float(v) for v in covariance.reshape(-1)))
        if not math.isfinite(self.stamp_s):
            raise ValueError("stamp_s must be finite")
        if not self.source_id or not self.localization_epoch:
            raise ValueError("source_id and localization_epoch must not be empty")
        if self.map_version < 0:
            raise ValueError("map_version must be non-negative")


@dataclass(frozen=True)
class AssociationResult:
    accepted: bool
    ambiguous: bool
    candidate_index: int | None
    mahalanobis_distance: float | None
    reason: str


@dataclass(frozen=True)
class FilterOutput:
    state: TrackState
    state_vector: tuple[float, float, float, float]
    covariance: tuple[float, ...]
    last_measurement_s: float
    prediction_stamp_s: float
    publication_stamp_s: float
    association: AssociationResult


def _transition(dt: float) -> np.ndarray:
    return np.array(
        [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )


def _process_noise(dt: float, acceleration_noise: float) -> np.ndarray:
    q = acceleration_noise
    block = q * np.array([[dt**3 / 3.0, dt**2 / 2.0], [dt**2 / 2.0, dt]])
    return np.array(
        [[block[0, 0], 0.0, block[0, 1], 0.0],
         [0.0, block[0, 0], 0.0, block[0, 1]],
         [block[1, 0], 0.0, block[1, 1], 0.0],
         [0.0, block[1, 0], 0.0, block[1, 1]]]
    )


class OpponentFilter:
    """Linear CV Kalman filter with conservative lifecycle and association."""

    def __init__(self, config: FilterConfig = FilterConfig()) -> None:
        self.config = config
        self._state = np.zeros(4)
        self._covariance = np.eye(4)
        self._initialized = False
        self._state_name = TrackState.SEARCHING
        self._last_measurement_s: float | None = None
        self._prediction_stamp_s: float | None = None
        self._publication_stamp_s: float | None = None
        self._map_version: int | None = None
        self._localization_epoch: str | None = None
        self._source_id = ""
        self._confirmation = 0

    @property
    def state(self) -> np.ndarray:
        return self._state.copy()

    @property
    def covariance(self) -> np.ndarray:
        return self._covariance.copy()

    def reset(self) -> None:
        self.__init__(self.config)

    def initialize(self, detection: OpponentDetection) -> FilterOutput:
        self._state[:2] = detection.position_xy_m
        self._state[2:] = 0.0
        self._covariance = np.zeros((4, 4))
        self._covariance[:2, :2] = np.asarray(detection.covariance_xy_m2).reshape(2, 2)
        self._covariance[2:, 2:] = np.eye(2) * 1.0
        self._initialized = True
        self._state_name = TrackState.TRACKED if self.config.confirmation_count == 1 else TrackState.SEARCHING
        self._confirmation = 1
        self._last_measurement_s = detection.stamp_s
        self._prediction_stamp_s = detection.stamp_s
        self._publication_stamp_s = detection.stamp_s
        self._map_version = detection.map_version
        self._localization_epoch = detection.localization_epoch
        self._source_id = detection.source_id
        return self._output(detection.stamp_s, AssociationResult(True, False, 0, 0.0, "INITIALIZED"))

    def predict(self, stamp_s: float) -> FilterOutput:
        self._require_initialized()
        if not math.isfinite(stamp_s) or self._prediction_stamp_s is None:
            raise ValueError("stamp_s must be finite and filter initialized")
        dt = stamp_s - self._prediction_stamp_s
        if dt < 0.0 or dt > self.config.max_prediction_dt_s:
            raise ValueError("prediction timestamp is out of order or too far ahead")
        if dt > 0.0:
            transition = _transition(dt)
            self._state = transition @ self._state
            self._covariance = transition @ self._covariance @ transition.T + _process_noise(
                dt, self.config.acceleration_noise_m2ps3
            )
            self._covariance = self._symmetrize_psd(self._covariance)
            self._prediction_stamp_s = stamp_s
        self._advance_lifecycle(stamp_s)
        self._publication_stamp_s = stamp_s
        return self._output(stamp_s, AssociationResult(False, False, None, None, "PREDICTED"))

    def update(self, detections: Sequence[OpponentDetection], stamp_s: float) -> FilterOutput:
        self._require_initialized()
        if not math.isfinite(stamp_s) or self._prediction_stamp_s is None:
            raise ValueError("stamp_s must be finite and filter initialized")
        if stamp_s < self._prediction_stamp_s:
            raise ValueError("measurement timestamp must not precede prediction timestamp")
        if stamp_s > self._prediction_stamp_s:
            self.predict(stamp_s)
        association = self.associate(detections)
        if not association.accepted or association.candidate_index is None:
            self._advance_lifecycle(stamp_s)
            self._publication_stamp_s = stamp_s
            return self._output(stamp_s, association)
        detection = detections[association.candidate_index]
        measurement = np.asarray(detection.position_xy_m)
        measurement_covariance = np.asarray(detection.covariance_xy_m2).reshape(2, 2)
        measurement_matrix = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        innovation = measurement - measurement_matrix @ self._state
        innovation_covariance = (
            measurement_matrix @ self._covariance @ measurement_matrix.T + measurement_covariance
        )
        try:
            gain = np.linalg.solve(innovation_covariance, measurement_matrix @ self._covariance).T
        except np.linalg.LinAlgError as exc:
            raise ValueError("innovation covariance is not solvable") from exc
        identity = np.eye(4)
        residual_projector = identity - gain @ measurement_matrix
        self._state = self._state + gain @ innovation
        # Joseph form preserves PSD under finite precision and correlated R.
        self._covariance = (
            residual_projector @ self._covariance @ residual_projector.T
            + gain @ measurement_covariance @ gain.T
        )
        self._covariance = self._symmetrize_psd(self._covariance)
        self._last_measurement_s = detection.stamp_s
        self._source_id = detection.source_id
        self._advance_lifecycle(stamp_s, accepted=True)
        self._publication_stamp_s = stamp_s
        return self._output(stamp_s, association)

    def associate(self, detections: Sequence[OpponentDetection]) -> AssociationResult:
        if not detections:
            return AssociationResult(False, False, None, None, "NO_CANDIDATES")
        if self._map_version is None or self._localization_epoch is None:
            raise RuntimeError("filter metadata is not initialized")
        measurement_matrix = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        scored: list[tuple[float, int]] = []
        for index, detection in enumerate(detections):
            if detection.map_version != self._map_version or detection.localization_epoch != self._localization_epoch:
                continue
            innovation = np.asarray(detection.position_xy_m) - measurement_matrix @ self._state
            innovation_covariance = (
                measurement_matrix @ self._covariance @ measurement_matrix.T
                + np.asarray(detection.covariance_xy_m2).reshape(2, 2)
            )
            try:
                distance = float(innovation @ np.linalg.solve(innovation_covariance, innovation))
            except np.linalg.LinAlgError:
                continue
            if math.isfinite(distance) and distance <= self.config.innovation_gate:
                scored.append((distance, index))
        if not scored:
            return AssociationResult(False, False, None, None, "GATED_OUT")
        scored.sort()
        if len(scored) > 1 and scored[1][0] - scored[0][0] < self.config.ambiguity_margin:
            return AssociationResult(False, True, None, scored[0][0], "AMBIGUOUS_ASSOCIATION")
        return AssociationResult(True, False, scored[0][1], scored[0][0], "ACCEPTED")

    def _advance_lifecycle(self, stamp_s: float, accepted: bool = False) -> None:
        if accepted:
            self._confirmation += 1
            if self._confirmation >= self.config.confirmation_count:
                self._state_name = TrackState.TRACKED
            return
        if self._last_measurement_s is None:
            return
        age = stamp_s - self._last_measurement_s
        if age < self.config.coasting_timeout_s:
            self._state_name = TrackState.COASTING
        elif age < self.config.belief_timeout_s:
            self._state_name = TrackState.OCCLUDED_BELIEF
        elif age < self.config.lost_timeout_s:
            self._state_name = TrackState.LOST
        else:
            self._state_name = TrackState.SEARCHING

    def _output(self, stamp_s: float, association: AssociationResult) -> FilterOutput:
        return FilterOutput(
            self._state_name,
            tuple(float(v) for v in self._state),
            tuple(float(v) for v in self._covariance.reshape(-1)),
            self._last_measurement_s if self._last_measurement_s is not None else -1.0,
            self._prediction_stamp_s if self._prediction_stamp_s is not None else -1.0,
            stamp_s,
            association,
        )

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise RuntimeError("filter must be initialized before prediction or update")

    @staticmethod
    def _symmetrize_psd(covariance: np.ndarray) -> np.ndarray:
        covariance = 0.5 * (covariance + covariance.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        if np.min(eigenvalues) < -1e-8:
            raise ValueError("filter covariance became materially non-PSD")
        return eigenvectors @ np.diag(np.maximum(eigenvalues, 0.0)) @ eigenvectors.T
