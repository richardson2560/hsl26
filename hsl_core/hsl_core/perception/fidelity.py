# hsl_core/hsl_core/perception/fidelity.py
"""Evaluation-only comparison of cloud and synthetic-detection profiles."""

from __future__ import annotations

import math
from typing import Mapping, Sequence

import numpy as np


_PROFILE_NAMES = ("real_cloud", "mvsim_cloud", "synthetic_detection")


def _profile_metrics(samples: Sequence[Mapping], profile: str) -> dict:
    attempts = len(samples)
    detections = 0
    false_positives = 0
    misses = 0
    errors = []
    latencies = []
    covered_95 = []
    eligible = 0
    for sample in samples:
        truth = sample.get("truth_xy_m")
        result = sample.get(profile)
        if result is None:
            if truth is not None:
                misses += 1
            continue
        detections += 1
        position = np.asarray(result.get("position_xy_m"), dtype=float)
        if position.shape != (2,) or not np.all(np.isfinite(position)):
            raise ValueError(f"{profile} position must contain two finite values")
        latency = float(result.get("latency_ms"))
        if not math.isfinite(latency) or latency < 0.0:
            raise ValueError(f"{profile} latency_ms must be finite and non-negative")
        latencies.append(latency)
        if truth is None:
            false_positives += 1
            continue
        truth_array = np.asarray(truth, dtype=float)
        if truth_array.shape != (2,) or not np.all(np.isfinite(truth_array)):
            raise ValueError("truth_xy_m must contain two finite values or be null")
        error = position - truth_array
        errors.append(float(np.linalg.norm(error)))
        if "covariance_xy_m2" in result:
            covariance = np.asarray(result["covariance_xy_m2"], dtype=float)
            if covariance.shape == (4,):
                covariance = covariance.reshape(2, 2)
            if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
                raise ValueError(f"{profile} covariance must be finite 2x2")
            covariance = 0.5 * (covariance + covariance.T)
            eigenvalues = np.linalg.eigvalsh(covariance)
            if np.min(eigenvalues) <= 0.0:
                raise ValueError(f"{profile} covariance must be positive definite")
            mahalanobis_sq = float(error @ np.linalg.solve(covariance, error))
            covered_95.append(mahalanobis_sq <= 5.991464547)
        eligible += 1
    errors_array = np.asarray(errors, dtype=float)
    latency_array = np.asarray(latencies, dtype=float)
    return {
        "attempts": attempts,
        "detections": detections,
        "misses": misses,
        "false_positives": false_positives,
        "false_positive_rate_per_attempt": false_positives / attempts if attempts else None,
        "position_error_count": len(errors),
        "position_error_mean_m": float(np.mean(errors_array)) if len(errors_array) else None,
        "position_error_rmse_m": float(np.sqrt(np.mean(errors_array**2))) if len(errors_array) else None,
        "position_error_p95_m": float(np.quantile(errors_array, 0.95)) if len(errors_array) else None,
        "covariance_95_coverage": float(np.mean(covered_95)) if covered_95 else None,
        "latency_count": len(latency_array),
        "latency_mean_ms": float(np.mean(latency_array)) if len(latency_array) else None,
        "latency_p95_ms": float(np.quantile(latency_array, 0.95)) if len(latency_array) else None,
        "calibrated_error_samples": eligible,
    }


def evaluate_profiles(
    samples: Sequence[Mapping],
    *,
    minimum_labeled_samples: int = 20,
) -> dict:
    """Compute paired test metrics; never treat missing data as a pass."""
    if (
        isinstance(minimum_labeled_samples, bool)
        or not isinstance(minimum_labeled_samples, int)
        or minimum_labeled_samples <= 0
    ):
        raise ValueError("minimum_labeled_samples must be positive")
    if not samples:
        return {
            "status": "BLOCKED_NO_DATA",
            "evaluation_only": True,
            "profiles": {profile: _profile_metrics((), profile) for profile in _PROFILE_NAMES},
            "paired_rmse_delta_real_minus_synthetic_m": None,
        }
    ids = []
    for sample in samples:
        if not isinstance(sample, Mapping) or not sample.get("case_id"):
            raise ValueError("every evaluation sample needs a non-empty case_id")
        truth = sample.get("truth_xy_m")
        if truth is not None:
            truth_array = np.asarray(truth, dtype=float)
            if truth_array.shape != (2,) or not np.all(np.isfinite(truth_array)):
                raise ValueError("truth_xy_m must contain two finite values or be null")
        ids.append(sample["case_id"])
        for profile in _PROFILE_NAMES:
            if profile not in sample:
                raise ValueError(f"evaluation sample is missing profile {profile}")
    if len(ids) != len(set(ids)):
        raise ValueError("case_id values must be unique")
    metrics = {profile: _profile_metrics(samples, profile) for profile in _PROFILE_NAMES}
    common_errors = {profile: {} for profile in _PROFILE_NAMES}
    for sample in samples:
        truth = sample.get("truth_xy_m")
        if truth is None:
            continue
        truth_array = np.asarray(truth, dtype=float)
        for profile in _PROFILE_NAMES:
            result = sample[profile]
            if result is not None:
                position = np.asarray(result["position_xy_m"], dtype=float)
                common_errors[profile][sample["case_id"]] = float(np.linalg.norm(position - truth_array))
    paired_ids = set(common_errors["real_cloud"]) & set(common_errors["synthetic_detection"])
    if paired_ids:
        real_rmse = math.sqrt(np.mean([common_errors["real_cloud"][case] ** 2 for case in paired_ids]))
        synthetic_rmse = math.sqrt(
            np.mean([common_errors["synthetic_detection"][case] ** 2 for case in paired_ids])
        )
        delta = real_rmse - synthetic_rmse
    else:
        delta = None
    enough = min(metrics[p]["position_error_count"] for p in ("real_cloud", "synthetic_detection"))
    return {
        "status": "METRICS_COMPUTED" if enough >= minimum_labeled_samples else "BLOCKED_INSUFFICIENT_LABELED_DATA",
        "evaluation_only": True,
        "minimum_labeled_samples": minimum_labeled_samples,
        "paired_case_count": len(paired_ids),
        "profiles": metrics,
        "paired_rmse_delta_real_minus_synthetic_m": delta,
        "interpretation": "Metrics are descriptive; no acceptance threshold is inferred or declared.",
    }
