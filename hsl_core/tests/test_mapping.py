# hsl_core/tests/test_mapping.py
"""Adversarial SIL tests for the Phase-2 local obstacle/coverage contract."""

import math

import numpy as np
import pytest

from hsl_core.mapping import (
    OCCUPIED,
    OBSERVED_FREE,
    UNKNOWN,
    CoverageGrid,
    LocalObstacleBuilder,
    LocalObstacleConfig,
    LocalObstacleSnapshot,
    _central_corridor_distance,
)


def _builder() -> LocalObstacleBuilder:
    return LocalObstacleBuilder(
        LocalObstacleConfig(
            resolution_m=0.05,
            origin_x_m=-0.5,
            origin_y_m=-1.0,
            width=80,
            height=40,
        )
    )


def test_height_and_self_masks_keep_low_obstacle_and_remove_ground_roof_body():
    points = np.array([
        [1.5, 0.0, 0.10],
        [1.5, 0.1, 0.15],
        [1.5, 0.0, -0.20],
        [1.5, 0.0, 1.50],
        [0.10, 0.0, 0.10],
    ])
    result = _builder().build(points)
    assert result.free_distance_m == pytest.approx(1.5)
    assert result.obstacles_xyz.shape == (2, 3)
    assert np.all(result.obstacles_xyz[:, 2] >= -0.10)
    assert np.all(result.obstacles_xyz[:, 2] <= 0.40)
    assert np.all(np.hypot(result.obstacles_xyz[:, 0], result.obstacles_xyz[:, 1]) >= 0.18)


def test_corridor_metric_is_forward_x_and_excludes_side_returns():
    result = _builder().build(np.array([
        [0.8, 0.201, 0.1],
        [1.5, 0.0, 0.1],
        [-0.2, 0.0, 0.1],
    ]))
    assert result.free_distance_m == pytest.approx(1.5)
    assert result.complete is True
    assert result.frontal_coverage_valid is True


def test_side_only_or_out_of_grid_returns_do_not_certify_forward_free_space():
    side_only = _builder().build(np.array([[1.0, 0.5, 0.1]]))
    out_of_grid = _builder().build(np.array([[10.0, 0.0, 0.1]]))
    assert side_only.free_distance_m == 0.0
    assert side_only.complete is True
    assert side_only.frontal_coverage_valid is False
    assert out_of_grid.free_distance_m == 0.0
    assert out_of_grid.complete is True
    assert out_of_grid.frontal_coverage_valid is False


def test_ray_trace_marks_free_before_hit_and_never_clears_behind_hit():
    result = _builder().build(np.array([[1.5, 0.0, 0.1]]))
    hit = result.coverage.cells[20, 40]
    behind = result.coverage.cells[20, 50]
    assert hit == OCCUPIED
    assert behind == UNKNOWN
    before = result.coverage.cells[20, 35]
    assert before == OBSERVED_FREE


def test_no_return_leaves_grid_unknown_and_empty_corridor_unbounded():
    result = _builder().build(np.empty((0, 3)))
    assert result.free_distance_m == 0.0
    assert np.all(result.coverage.cells == UNKNOWN)
    assert result.complete is True
    assert result.frontal_coverage_valid is False


def test_near_hit_is_processed_before_far_hit_on_same_ray():
    result = _builder().build(np.array([
        [2.5, 0.0, 0.1],
        [1.5, 0.0, 0.1],
    ]))
    near_cell = result.coverage.cells[20, 40]
    behind_near = result.coverage.cells[20, 50]
    assert near_cell == OCCUPIED
    assert behind_near == UNKNOWN


def test_observed_centreline_free_space_is_reported_until_first_unknown():
    result = _builder().build(np.array([[1.5, 0.0, 0.1]]))
    assert result.free_distance_m == pytest.approx(1.5)
    assert result.complete is True
    assert result.frontal_coverage_valid is True


def test_unknown_after_observed_free_segment_remains_usable_with_bounded_clearance():
    result = _builder().build(np.array([[1.5, 0.0, 0.1]]))
    assert result.free_distance_m == pytest.approx(1.5)
    assert result.frontal_coverage_valid is True


def test_immediate_unknown_without_observed_margin_is_not_valid_coverage():
    result = _builder().build(np.array([[0.18, 0.5, 0.1]]))
    assert result.free_distance_m == pytest.approx(0.0)
    assert result.frontal_coverage_valid is False


def test_fully_observed_grid_uses_metric_origin_without_attribute_error():
    config = LocalObstacleConfig(
        resolution_m=0.05,
        origin_x_m=-0.5,
        origin_y_m=-1.0,
        width=40,
        height=40,
    )
    cells = np.full((config.height, config.width), OBSERVED_FREE, dtype=np.uint8)
    distance, valid = _central_corridor_distance(cells, config)
    assert distance == pytest.approx(1.5)
    assert valid is True


def test_occupied_return_at_corridor_edge_stops_forward_distance():
    result = _builder().build(np.array([
        [1.5, 0.0, 0.1],
        [1.5, 0.19, 0.1],
    ]))
    assert result.free_distance_m == pytest.approx(1.5)


def test_bounded_measurement_noise_preserves_a_clear_low_obstacle_fixture():
    rng = np.random.default_rng(20260925)
    nominal = np.array([[1.5, y, 0.15] for y in (-0.12, 0.0, 0.12)], dtype=float)
    noisy = nominal + rng.normal(0.0, (0.005, 0.005, 0.003), nominal.shape)
    result = _builder().build(noisy)
    assert result.complete is True
    assert 0.0 < result.free_distance_m <= float(np.min(noisy[:, 0]))
    assert result.obstacles_xyz.shape == nominal.shape
    assert np.all(result.obstacles_xyz[:, 2] > -0.10)
    assert np.all(result.obstacles_xyz[:, 2] < 0.40)


@pytest.mark.parametrize("z_value", [-0.100001, 0.400001])
def test_threshold_noise_outside_height_contract_is_rejected(z_value):
    result = _builder().build(np.array([[1.5, 0.0, z_value]]))
    assert result.obstacles_xyz.shape == (0, 3)
    assert result.complete is True
    assert np.all(result.coverage.cells == UNKNOWN)


def test_multiple_rays_preserve_occupied_returns_and_unknown_space():
    result = _builder().build(np.array([[1.0, 0.0, 0.1], [1.0, 0.5, 0.1]]))
    assert np.count_nonzero(result.coverage.cells == OCCUPIED) >= 2
    assert np.count_nonzero(result.coverage.cells == UNKNOWN) > 0


@pytest.mark.parametrize(
    "points, message",
    [
        (np.ones((2, 2)), "shape"),
        (np.array([[1.0, math.nan, 0.1]]), "finite"),
        (np.ones((5, 3)), "bound"),
    ],
)
def test_builder_rejects_malformed_or_unbounded_clouds(points, message):
    config = LocalObstacleConfig(max_points=4)
    with pytest.raises(ValueError, match=message):
        LocalObstacleBuilder(config).build(points)


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"z_min_m": 1.0, "z_max_m": 0.0}, "z_min"),
        ({"resolution_m": 0.0}, "resolution"),
        ({"width": True}, "width"),
        ({"height": 0}, "height"),
        ({"self_radius_m": -1.0}, "self radius"),
    ],
)
def test_configuration_rejects_invalid_geometry_and_bounds(kwargs, message):
    with pytest.raises(ValueError, match=message):
        LocalObstacleConfig(**kwargs)


def test_contract_records_copy_arrays_and_validate_states():
    cells = np.zeros((2, 2), dtype=np.uint8)
    grid = CoverageGrid((0.0, 0.0), 0.1, 2, 2, cells)
    cells[0, 0] = OCCUPIED
    assert grid.cells[0, 0] == UNKNOWN
    with pytest.raises(ValueError, match="invalid state"):
        CoverageGrid((0.0, 0.0), 0.1, 2, 2, np.full((2, 2), 3))
    with pytest.raises(ValueError, match="finite"):
        LocalObstacleSnapshot(np.array([[math.nan, 0.0, 0.0]]), grid, 1.0, True, False)


def test_builder_does_not_mutate_input():
    points = np.array([[1.5, 0.0, 0.1]])
    original = points.copy()
    _builder().build(points)
    assert np.array_equal(points, original)
