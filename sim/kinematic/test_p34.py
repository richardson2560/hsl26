# sim/kinematic/test_p34.py
"""Adversarial P3.4 tests for deterministic plant, sensors and truth boundary."""

from dataclasses import asdict
import math
from pathlib import Path

import pytest

from hsl_core.types import Pose2D

from sim.kinematic.common import (
    Actuation,
    CircleTarget,
    Segment,
    SensorObservation,
    WorldGeometry,
)
from sim.kinematic.plant import KinematicPlant
from sim.kinematic.raycaster import FirstHitRaycaster
from sim.kinematic.referee import Referee
from sim.kinematic.scenario import load_scenario
from sim.kinematic.sensors import LidarSensor
from sim.kinematic.trace import DeterministicTrace


def _sensor(seed: int = 4, blind=()):
    return LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=8.0),
        beam_count=8,
        max_range_m=8.0,
        noise_std_m=0.02,
        seed=seed,
        blind_sectors=blind,
    )


def test_raycaster_returns_nearest_static_or_dynamic_first_hit():
    raycaster = FirstHitRaycaster(max_range_m=10.0)
    geometry = WorldGeometry(
        (Segment((2.0, -1.0), (2.0, 1.0)),),
        (CircleTarget((4.0, 0.0), 0.5, "opponent"),),
    )
    assert raycaster.cast((0.0, 0.0), 0.0, geometry.static_segments, geometry.dynamic_targets) == pytest.approx(2.0)


def test_raycaster_does_not_report_parallel_or_behind_geometry():
    raycaster = FirstHitRaycaster(max_range_m=5.0)
    segment = Segment((1.0, 1.0), (2.0, 1.0))
    assert raycaster.cast((0.0, 0.0), 0.0, (segment,)) is None
    assert raycaster.cast((0.0, 0.0), math.pi, (segment,)) is None


def test_sensor_blind_sector_is_invalid_not_a_max_range_hit():
    sensor = _sensor(blind=((0.0, 0.2),))
    geometry = WorldGeometry((Segment((2.0, -0.1), (2.0, 0.1)),))
    observation = sensor.observe(Pose2D(0.0, 0.0, 0.0), 1.0, geometry)
    assert observation.valid_mask[4] is False
    assert observation.ranges_m[4] == 8.0


def test_sensor_observation_contains_no_truth_identity_or_pose():
    sensor = LidarSensor(raycaster=FirstHitRaycaster(), beam_count=4)
    observation = sensor.observe(
        Pose2D(0.0, 0.0, 0.0),
        0.0,
        WorldGeometry((), (CircleTarget((1.0, 0.0), 0.2, "secret"),)),
    )
    fields = asdict(observation)
    assert set(fields) == {"stamp_s", "frame_id", "ranges_m", "valid_mask"}
    assert "secret" not in repr(observation)


def test_plant_straight_step_is_exact_and_monotonic_in_time():
    plant = KinematicPlant(Pose2D(0.0, 0.0, 0.0))
    state = plant.step(Actuation(0.5, 0.0), 2.0)
    assert state.pose.x_m == pytest.approx(1.0)
    assert state.pose.y_m == pytest.approx(0.0)
    assert state.stamp_s == pytest.approx(2.0)


def test_plant_arc_step_matches_unicycle_solution():
    plant = KinematicPlant(Pose2D(0.0, 0.0, 0.0))
    state = plant.step(Actuation(1.0, 1.0), math.pi / 2.0)
    assert state.pose.x_m == pytest.approx(1.0)
    assert state.pose.y_m == pytest.approx(1.0)
    assert state.pose.theta_rad == pytest.approx(math.pi / 2.0)


def test_plant_rejects_invalid_dt_and_overlimit_commands():
    plant = KinematicPlant(Pose2D(0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        plant.step(Actuation(0.0, 0.0), 0.0)
    with pytest.raises(ValueError):
        plant.step(Actuation(2.0, 0.0), 0.1)


def test_referee_alone_can_report_collision_truth():
    geometry = WorldGeometry((), (CircleTarget((0.3, 0.0), 0.2, "opponent"),))
    result = Referee(geometry, 0.2).evaluate(
        KinematicPlant(Pose2D(0.0, 0.0, 0.0)).state
    )
    assert result.collision is True
    assert result.collision_target_ids == ("opponent",)


def test_sensor_seed_replay_is_byte_stable_and_seeds_are_isolated():
    pose = Pose2D(0.0, 0.0, 0.0)
    geometry = WorldGeometry((Segment((2.0, -1.0), (2.0, 1.0)),))
    first = _sensor(seed=7).observe(pose, 0.0, geometry)
    second = _sensor(seed=7).observe(pose, 0.0, geometry)
    changed = _sensor(seed=8).observe(pose, 0.0, geometry)
    assert first == second
    assert first != changed


def test_trace_digest_is_deterministic_for_same_event_sequence():
    command = Actuation(0.1, 0.2)
    observation = SensorObservation(0.1, "lidar_link", (1.0,), (True,))
    first = DeterministicTrace()
    second = DeterministicTrace()
    for trace in (first, second):
        trace.record_command(0.0, command)
        trace.record_observation(observation)
    assert first.digest() == second.digest()
    assert first.to_json() == second.to_json()


def test_scenario_manifest_is_strict_and_rejects_physical_claims():
    path = Path(__file__).with_name("scenarios") / "parallel_corridors.json"
    spec = load_scenario(path)
    assert spec.profile == "observed_map"
    assert spec.physical_authority is False

    path_data = path.read_text(encoding="utf-8").replace('"physical_authority": false', '"physical_authority": true')
    invalid = path.with_name("_invalid_p34_scenario.json")
    invalid.write_text(path_data, encoding="utf-8")
    try:
        with pytest.raises(ValueError, match="physical authority"):
            load_scenario(invalid)
    finally:
        invalid.unlink()


def test_contracts_reject_invalid_sensor_observation_shapes_and_values():
    with pytest.raises(ValueError):
        SensorObservation(0.0, "lidar_link", (1.0,), ())
    with pytest.raises(ValueError):
        SensorObservation(0.0, "lidar_link", (-1.0,), (True,))
