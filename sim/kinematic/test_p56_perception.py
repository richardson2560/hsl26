"""Adversarial tests for observation-only synthetic opponent extraction."""

import math

import pytest

from hsl_core.types import Pose2D
from sim.kinematic.common import SensorObservation, Segment
from sim.kinematic.perception import (
    ExtractorConfig,
    extract_synthetic_opponent,
    line_of_sight_clear,
)
from sim.kinematic.raycaster import FirstHitRaycaster


def _scan(*, target=True, blind=False):
    from sim.kinematic.common import CircleTarget, WorldGeometry
    from sim.kinematic.sensors import LidarSensor

    sensor = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=5.0),
        beam_count=720,
        max_range_m=5.0,
        seed=17,
        blind_sectors=((0.0, 0.05),) if blind else (),
    )
    geometry = WorldGeometry(
        (Segment((4.0, -3.0), (4.0, 3.0)),),
        (CircleTarget((2.0, 0.0), 0.15, "private-truth"),) if target else (),
    )
    pose = Pose2D(0.0, 0.0, 0.0)
    return sensor.observe(pose, 1.0, geometry), pose, geometry.static_segments


def test_static_wall_returns_are_not_misclassified_as_opponent():
    observation, pose, walls = _scan(target=False)
    assert extract_synthetic_opponent(
        observation,
        pose,
        walls,
        stamp_s=1.0,
        source_id="scan-a",
        map_version=1,
        localization_epoch="loc-a",
        opponent_radius_m=0.15,
    ) == ()


def test_dynamic_cluster_is_detected_without_target_identity_or_truth_input():
    observation, pose, walls = _scan(target=True)
    detections = extract_synthetic_opponent(
        observation,
        pose,
        walls,
        stamp_s=1.0,
        source_id="scan-a",
        map_version=1,
        localization_epoch="loc-a",
        opponent_radius_m=0.15,
    )
    assert len(detections) == 1
    assert detections[0].position_xy_m[0] == pytest.approx(2.0, abs=0.04)
    assert detections[0].position_xy_m[1] == pytest.approx(0.0, abs=0.04)
    assert detections[0].source_id == "scan-a"
    assert not hasattr(detections[0], "target_id")


def test_multiple_residual_clusters_remain_separate_for_ambiguous_association():
    from sim.kinematic.common import CircleTarget, WorldGeometry
    from sim.kinematic.sensors import LidarSensor

    pose = Pose2D(0.0, 0.0, 0.0)
    walls = (Segment((4.0, -3.0), (4.0, 3.0)),)
    observation = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=5.0),
        beam_count=720,
        max_range_m=5.0,
    ).observe(
        pose,
        1.0,
        WorldGeometry(
            walls,
            (
                CircleTarget((2.0, 0.0), 0.15, "private-a"),
                CircleTarget((0.0, 2.0), 0.15, "private-b"),
            ),
        ),
    )
    detections = extract_synthetic_opponent(
        observation,
        pose,
        walls,
        stamp_s=1.0,
        source_id="scan-a",
        map_version=1,
        localization_epoch="loc-a",
        opponent_radius_m=0.15,
    )
    assert len(detections) == 2
    assert detections[0].position_xy_m != detections[1].position_xy_m


def test_incomplete_coverage_and_invalid_cluster_fail_closed():
    observation, pose, walls = _scan(target=True, blind=True)
    # Partial coverage cannot be promoted to a complete circular scan.
    assert extract_synthetic_opponent(
        observation,
        pose,
        walls,
        stamp_s=1.0,
        source_id="scan-a",
        map_version=1,
        localization_epoch="loc-a",
        opponent_radius_m=0.15,
    ) == ()
    observation, pose, walls = _scan(target=True)
    with pytest.raises(ValueError, match="stamp"):
        extract_synthetic_opponent(
            observation,
            pose,
            walls,
            stamp_s=0.9,
            source_id="scan-a",
            map_version=1,
            localization_epoch="loc-a",
            opponent_radius_m=0.15,
        )


@pytest.mark.parametrize(
    "changes",
    [
        {"static_residual_threshold_m": math.nan},
        {"minimum_cluster_beams": 1},
        {"maximum_cluster_gap_beams": -1},
    ],
)
def test_extractor_config_rejects_invalid_thresholds(changes):
    with pytest.raises(ValueError):
        ExtractorConfig(**changes)


def test_los_reports_walls_and_keeps_clear_or_degenerate_rays_valid():
    wall = (Segment((1.0, -1.0), (1.0, 1.0)),)
    assert not line_of_sight_clear((0.0, 0.0), (2.0, 0.0), wall)
    assert line_of_sight_clear((0.0, 2.0), (2.0, 2.0), wall)
    assert line_of_sight_clear((0.0, 0.0), (0.0, 0.0), wall)
    assert not line_of_sight_clear(
        (0.0, 0.0), (2.0, 0.0), (Segment((0.5, 0.0), (1.5, 0.0)),)
    )
