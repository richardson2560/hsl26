# hsl_core/tests/test_p41_gpis.py
"""P4.1 adversarial tests for the compact Hermite-GPIS-W prior."""

import json
import hashlib
from pathlib import Path

import numpy as np
import pytest

from hsl_core.perception.implicit_surface import (
    HermiteGPIS,
    HermiteObservation,
    KERNEL_ID,
    _kernel_profile,
    hermite_covariance,
    wendland_c4,
)


def _observations():
    return [
        HermiteObservation((0.0, 0.0, 0.0), "value", (1.0, 0.0, 0.0), 0.0, 1e-4),
        HermiteObservation((0.2, 0.0, 0.0), "derivative", (1.0, 0.0, 0.0), 1.0, 1e-4),
        HermiteObservation((0.0, 0.2, 0.0), "derivative", (0.0, 1.0, 0.0), 1.0, 1e-4),
        HermiteObservation((0.0, 0.0, 0.2), "derivative", (0.0, 0.0, 1.0), 1.0, 1e-4),
    ]


def test_kernel_is_symmetric_compact_and_unit_centered():
    points = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]])
    matrix = wendland_c4(points, points, support_radius_m=1.0)
    assert matrix[0, 0] == pytest.approx(1.0)
    assert matrix[0, 1] == matrix[1, 0]
    assert wendland_c4([[0.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]], support_radius_m=1.0)[0, 0] == 0.0


def test_c4_profile_limits_and_closed_form_derivatives_match():
    values, first, second = _kernel_profile(np.array([0.0, 0.3, 1.0]))
    assert values[0] == pytest.approx(1.0)
    assert first[0] == pytest.approx(0.0)
    assert second[0] == pytest.approx(-56.0 / 3.0)
    u = 0.3
    assert first[1] == pytest.approx((-56.0 / 3.0) * u * (5.0 * u + 1.0) * (1.0 - u) ** 5)
    assert second[1] == pytest.approx((56.0 / 3.0) * (1.0 - u) ** 4 * (35.0 * u**2 - 4.0 * u - 1.0))
    assert values[2] == pytest.approx(0.0)
    assert first[2] == pytest.approx(0.0)
    assert second[2] == pytest.approx(0.0)


def test_hermite_covariance_is_symmetric_positive_definite():
    covariance = hermite_covariance(_observations(), support_radius_m=1.0, amplitude=1.0)
    assert np.allclose(covariance, covariance.T, atol=1e-12)
    assert np.min(np.linalg.eigvalsh(covariance)) > 0.0


def test_derivative_signs_and_origin_hessian_are_finite():
    observations = [
        HermiteObservation((0.0, 0.0, 0.0), "derivative", (1.0, 0.0, 0.0), 0.0, 1e-3),
        HermiteObservation((0.1, 0.0, 0.0), "derivative", (1.0, 0.0, 0.0), 0.0, 1e-3),
    ]
    matrix = hermite_covariance(observations, support_radius_m=1.0, amplitude=1.0)
    assert np.all(np.isfinite(matrix))
    assert matrix[0, 1] == pytest.approx(matrix[1, 0])


def test_directional_derivative_block_matches_finite_difference():
    point = np.array([[0.17, -0.08, 0.11]])
    other = np.array([[0.03, 0.06, -0.04]])
    direction = np.array([0.6, -0.8, 0.0])
    epsilon = 1e-6
    plus = wendland_c4(point, other + epsilon * direction, support_radius_m=1.0)[0, 0]
    minus = wendland_c4(point, other - epsilon * direction, support_radius_m=1.0)[0, 0]
    numerical = (plus - minus) / (2.0 * epsilon)
    observation = HermiteObservation(tuple(other[0]), "derivative", tuple(direction), 0.0, 1e-3)
    block = hermite_covariance(
        [
            HermiteObservation(tuple(point[0]), "value", (1.0, 0.0, 0.0), 0.0, 1e-3),
            observation,
        ],
        support_radius_m=1.0,
        amplitude=1.0,
    )[0, 1]
    assert block == pytest.approx(numerical, rel=1e-5, abs=1e-7)


def test_kernel_and_gradient_are_zero_at_and_beyond_support_boundary():
    model = HermiteGPIS(_observations(), support_radius_m=0.5, regularization=1e-8)
    inside = model.evaluate((0.699999, 0.0, 0.0))
    boundary = model.evaluate((0.7, 0.0, 0.0))
    outside = model.evaluate((0.700001, 0.0, 0.0))
    assert abs(inside[0]) < 1e-15
    assert np.allclose(inside[2], 0.0, atol=1e-10)
    assert boundary[0] == pytest.approx(0.0)
    assert np.allclose(boundary[2], 0.0)
    assert outside[0] == pytest.approx(0.0)


def test_model_rejects_unsupported_query_as_surface_match():
    model = HermiteGPIS(_observations(), support_radius_m=0.5, regularization=1e-8)
    mean, variance, gradient = model.evaluate((10.0, 10.0, 10.0))
    assert mean == pytest.approx(0.0)
    assert variance == pytest.approx(1.0)
    assert np.allclose(gradient, 0.0)
    assert model.support_fraction([(10.0, 10.0, 10.0)]) == 0.0


def test_model_prediction_and_support_fraction_are_finite():
    model = HermiteGPIS(_observations(), support_radius_m=1.0, regularization=1e-8)
    mean, variance, gradient = model.evaluate((0.05, 0.0, 0.0))
    assert np.isfinite(mean)
    assert 0.0 <= variance <= 1.0
    assert np.all(np.isfinite(gradient))
    assert model.support_fraction([(0.05, 0.0, 0.0), (3.0, 0.0, 0.0)]) == pytest.approx(0.5)


def test_invalid_observations_and_kernel_parameters_are_rejected():
    with pytest.raises(ValueError):
        HermiteObservation((0.0, 0.0, 0.0), "value", (0.0, 0.0, 0.0), 0.0, 1e-3)
    with pytest.raises(ValueError):
        HermiteObservation((0.0, 0.0, 0.0), "unknown", (1.0, 0.0, 0.0), 0.0, 1e-3)
    with pytest.raises(ValueError):
        wendland_c4([[0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0]], support_radius_m=0.0)
    with pytest.raises(ValueError):
        HermiteGPIS([], support_radius_m=1.0)


def test_artifact_round_trip_has_non_object_arrays_and_hash_manifest(tmp_path: Path):
    model = HermiteGPIS(_observations(), support_radius_m=1.0, regularization=1e-8)
    manifest = model.save(tmp_path)
    assert manifest["kernel_id"] == KERNEL_ID
    with np.load(tmp_path / "model.npz", allow_pickle=False) as arrays:
        assert "alpha" in arrays.files
        assert all(arrays[name].dtype != object for name in arrays.files)
    loaded_manifest = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))
    assert loaded_manifest["artifact_sha256"] == manifest["artifact_sha256"]
    assert hashlib.sha256((tmp_path / "model.npz").read_bytes()).hexdigest() == manifest["artifact_sha256"]
