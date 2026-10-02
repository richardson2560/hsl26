"""Tests for the dependency-free GPIS isosurface extraction fallback."""

import numpy as np

from tools.visualize_gpis_field import _filter_supported_surface, _marching_tetrahedra


def test_marching_tetrahedra_extracts_a_closed_zero_level_surface():
    axis = np.linspace(-1.0, 1.0, 15)
    x, y, z = np.meshgrid(axis, axis, axis, indexing="ij")
    volume = x * x + y * y + z * z - 0.45**2
    vertices, triangles = _marching_tetrahedra((axis, axis, axis), volume)
    assert len(vertices) > 0
    assert len(triangles) > 0
    assert np.max(np.abs(np.linalg.norm(vertices, axis=1) - 0.45)) < 0.04
    assert np.max(triangles) < len(vertices)


def test_unsupported_zero_level_triangles_are_not_rendered():
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [1.0, 1.0, 1.0],
        ]
    )
    triangles = np.array([[0, 1, 2], [0, 1, 3]])
    support_points = np.array([[0.0, 0.0, 0.0]])
    kept_vertices, kept_triangles = _filter_supported_surface(
        vertices, triangles, support_points, support_radius_m=0.2
    )
    assert len(kept_triangles) == 1
    assert np.all(np.linalg.norm(kept_vertices[kept_triangles], axis=2) < 0.2)
