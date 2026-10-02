"""Tests for constructing an experimental HGW prior from extracted clouds."""

import numpy as np
import pytest

from tools.build_gpis_from_robot_cloud import build_observations


def test_build_observations_produces_oriented_hermite_surface_samples():
    x, y = np.meshgrid(np.linspace(-0.1, 0.1, 11), np.linspace(-0.1, 0.1, 11))
    points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    observations, transform, report = build_observations(
        points,
        max_surface_points=24,
        normal_radius_m=0.05,
        normal_max_neighbors=30,
        sensor_origin_base_link_m=(0.0, 0.0, 1.0),
    )
    assert report["surface_sample_count"] == 24
    assert report["hermite_observation_count"] == 48
    assert np.allclose(transform[:3, 3], np.median(points, axis=0))
    value_observations = [item for item in observations if item.kind == "value"]
    derivative_observations = [item for item in observations if item.kind == "derivative"]
    assert len(value_observations) == len(derivative_observations) == 24
    assert all(item.value == 0.0 for item in value_observations)
    assert all(item.value == 1.0 for item in derivative_observations)
    assert all(item.direction[2] > 0.99 for item in derivative_observations)
    assert not report["surface_constraints"]["off_surface_anchors"]


def test_build_observations_rejects_nonfinite_cloud():
    with pytest.raises(ValueError, match="non-finite"):
        build_observations(
            np.array(
                [
                    [0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [1.0, 1.0, 0.0],
                    [0.5, 0.5, 0.0],
                    [np.nan, 0.0, 0.0],
                ]
            )
        )
