# hsl_core/tests/test_p42_registration.py
"""Adversarial P4.2 tests for segmentation and bounded planar registration."""

import math

import numpy as np
import pytest

from hsl_core.perception.implicit_surface import HermiteGPIS, HermiteObservation
from hsl_core.perception.registration import PlanarRegistrar, RegistrationConfig
from hsl_core.perception.segmenter import CandidateSegmenter, SegmentationConfig


def _model():
    observations = []
    angles = np.linspace(0.0, 2.0 * math.pi, 12, endpoint=False)
    for angle in angles:
        normal = (math.cos(angle), math.sin(angle), 0.0)
        point = tuple(0.3 * np.asarray(normal))
        observations.append(HermiteObservation(point, "value", normal, 0.0, 1e-4))
        observations.append(HermiteObservation(point, "derivative", normal, 1.0, 1e-4))
    observations.append(HermiteObservation((0.0, 0.0, 0.3), "value", (0.0, 0.0, 1.0), 0.0, 1e-4))
    observations.append(HermiteObservation((0.0, 0.0, -0.3), "value", (0.0, 0.0, -1.0), 0.0, 1e-4))
    return HermiteGPIS(observations, support_radius_m=0.8, regularization=1e-8)


def _surface_points():
    angles = np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)
    return np.column_stack((0.3 * np.cos(angles), 0.3 * np.sin(angles), np.zeros_like(angles)))


def test_segmenter_excludes_invalid_and_static_points_without_touching_input():
    points = np.vstack((_surface_points(), [[3.0, 3.0, 3.0], [4.0, 4.0, 4.0]]))
    original = points.copy()
    valid = np.ones(len(points), dtype=bool)
    valid[-1] = False
    static = np.zeros(len(points), dtype=bool)
    static[-2] = True
    clusters = CandidateSegmenter(SegmentationConfig(connectivity_radius_m=0.1, min_points=5)).segment(
        points, valid_mask=valid, static_mask=static
    )
    assert len(clusters) == 1
    assert len(clusters[0].points_m) == 32
    assert np.array_equal(points, original)


def test_segmenter_rejects_bad_shapes_nonfinite_and_bad_config():
    with pytest.raises(ValueError):
        CandidateSegmenter().segment([[0.0, 0.0]])
    with pytest.raises(ValueError):
        CandidateSegmenter().segment([[math.nan, 0.0, 0.0]])
    with pytest.raises(ValueError):
        SegmentationConfig(connectivity_radius_m=0.0)


def test_registration_recovers_translation_and_wraps_yaw():
    model = _model()
    registrar = PlanarRegistrar(model, RegistrationConfig(max_iterations=40))
    result = registrar.register(_surface_points() + np.array([0.1, -0.05, 0.0]), initial_pose_xyyaw=(0.1, -0.05, 0.0))
    assert result.status == "ACCEPTED"
    assert result.position_valid is True
    assert np.all(np.isfinite(result.pose_xyyaw))
    assert -math.pi <= result.pose_xyyaw[2] < math.pi


def test_registration_rejects_insufficient_support_and_points():
    registrar = PlanarRegistrar(_model())
    few = registrar.register([[0.0, 0.0, 0.0]] * 3, initial_pose_xyyaw=(0.0, 0.0, 0.0))
    assert few.status == "REJECTED"
    assert few.reason == "INSUFFICIENT_POINTS"
    unsupported = registrar.register(
        np.ones((8, 3)) * 10.0, initial_pose_xyyaw=(0.0, 0.0, 0.0)
    )
    assert unsupported.status == "REJECTED"
    assert unsupported.reason == "INSUFFICIENT_SUPPORT"


def test_registration_rejects_nonfinite_and_invalid_noise():
    registrar = PlanarRegistrar(_model())
    with pytest.raises(ValueError):
        registrar.register([[math.nan, 0.0, 0.0]] * 8, initial_pose_xyyaw=(0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        registrar.register(_surface_points(), initial_pose_xyyaw=(0.0, 0.0, 0.0), point_variance_m2=0.0)


def test_axisymmetric_like_flat_view_does_not_claim_yaw():
    config = RegistrationConfig(min_yaw_information=1e-3)
    flat_view = np.column_stack((
        np.full(12, 0.3),
        np.zeros(12),
        np.linspace(-0.15, 0.15, 12),
    ))
    result = PlanarRegistrar(_model(), config).register(
        flat_view, initial_pose_xyyaw=(0.0, 0.0, 0.0)
    )
    assert result.yaw_valid is False or result.position_valid is False
