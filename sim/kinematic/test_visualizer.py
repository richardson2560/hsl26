# sim/kinematic/test_visualizer.py
"""Headless visualizer tests; no GUI backend is required."""

import math

import numpy as np
import pytest

from hsl_core.types import Pose2D

from sim.kinematic.common import CircleTarget, Segment, SensorObservation, WorldGeometry
from sim.kinematic.visualizer import DoomConfig, TopViewConfig, render_doom, render_top_view


def _observation(valid=True):
    return SensorObservation(0.0, "lidar_link", (2.0, 2.0, 2.0, 2.0), (valid,) * 4)


def test_top_view_is_deterministic_and_has_expected_shape():
    geometry = WorldGeometry(
        (Segment((2.0, -1.0), (2.0, 1.0)),),
        (CircleTarget((1.0, 0.0), 0.2, "explorer"),),
    )
    config = TopViewConfig(width_px=80, height_px=60, world_bounds=(-2.0, -2.0, 3.0, 2.0))
    first = render_top_view(Pose2D(0.0, 0.0, 0.0), geometry, _observation(), config)
    second = render_top_view(Pose2D(0.0, 0.0, 0.0), geometry, _observation(), config)
    assert first.shape == (60, 80, 3)
    assert first.dtype == np.uint8
    assert np.array_equal(first, second)


def test_top_view_marks_invalid_beams_red_and_valid_beams_green():
    geometry = WorldGeometry(())
    image = render_top_view(
        Pose2D(0.0, 0.0, 0.0),
        geometry,
        SensorObservation(0.0, "lidar_link", (1.0, 1.0), (True, False)),
        TopViewConfig(width_px=40, height_px=40, world_bounds=(-2.0, -2.0, 2.0, 2.0)),
    )
    assert np.any(np.all(image == (35, 170, 70), axis=2))
    assert np.any(np.all(image == (210, 60, 60), axis=2))


def test_doom_projection_shape_and_invalid_beam_is_not_rendered_as_wall():
    config = DoomConfig(width_px=64, height_px=32)
    image = render_doom(_observation(), config)
    invalid = render_doom(
        SensorObservation(0.0, "lidar_link", (2.0,), (False,)),
        DoomConfig(width_px=16, height_px=16),
    )
    assert image.shape == (32, 64, 3)
    assert image.dtype == np.uint8
    assert np.all(invalid[7] == (92, 128, 170))
    assert np.all(invalid[8] == (70, 70, 70))


def test_doom_uses_only_front_fov_and_does_not_project_rear_beams():
    observation = SensorObservation(
        0.0,
        "lidar_link",
        (1.0, 5.0, 5.0, 1.0),
        (False, True, True, False),
    )
    image = render_doom(
        observation,
        DoomConfig(width_px=32, height_px=32, focal_px=16.0, fov_rad=math.pi / 2.0),
    )
    # The front FOV uses the central beams; rear beams cannot create giant
    # columns because they are never selected by the projection.
    assert np.all(image[0] == (92, 128, 170))
    gray_pixels = (image[:, :, 0] == image[:, :, 1]) & (
        image[:, :, 1] == image[:, :, 2]
    )
    assert np.any(gray_pixels & (image[:, :, 0] > 0) & (image[:, :, 0] < 220))


def test_visualizer_rejects_empty_observations_and_invalid_configurations():
    with pytest.raises(ValueError):
        render_doom(SensorObservation(0.0, "lidar_link", (), ()))
    with pytest.raises(ValueError):
        render_top_view(Pose2D(0.0, 0.0, 0.0), WorldGeometry(()), SensorObservation(0.0, "lidar_link", (), ()))
    with pytest.raises(ValueError):
        TopViewConfig(world_bounds=(0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        DoomConfig(wall_height_m=0.0)
    with pytest.raises(ValueError):
        DoomConfig(fov_rad=0.0)
    with pytest.raises(ValueError):
        DoomConfig(fov_rad=math.pi)
