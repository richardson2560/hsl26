"""Adversarial tests for deterministic, fixture-only P6.3 episode execution."""

from dataclasses import replace
import math

import pytest

from hsl_core.learning.evolution import TacticalGenome, mutate_genome
from hsl_core.match import Role, TerminalKind
from hsl_core.tactics import UtilityProfile
from sim.kinematic.benchmark import (
    BenchmarkConfig,
    PairedSILBenchmarkEpisode,
    SILBenchmarkRunner,
    _segment_distance,
    _swept_robot_contact,
    _swept_wall_contact,
)
from sim.kinematic.maze_bank import load_maze_bank
from hsl_core.types import Pose2D
from sim.kinematic.common import Actuation, Segment
from sim.kinematic.common import WorldGeometry
from sim.kinematic.raycaster import FirstHitRaycaster
from sim.kinematic.sensors import LidarSensor


_BASELINE = TacticalGenome.from_profiles(
    UtilityProfile(
        "baseline-guardian",
        Role.GUARDIAN,
        (
            ("capture_opportunity", 0.40),
            ("portal_time_advantage", 0.25),
            ("observation_gain", 0.15),
            ("pursuit_value", 0.10),
            ("duration_cost", -0.10),
        ),
        1,
    ),
    UtilityProfile(
        "baseline-explorer",
        Role.EXPLORER,
        (
            ("base_progress", 0.35),
            ("visibility_loss", 0.20),
            ("alternative_exits", 0.15),
            ("escape_safety", 0.15),
            ("observation_gain", 0.05),
            ("capture_risk", -0.05),
            ("duration_cost", -0.05),
        ),
        1,
    ),
    hysteresis_delta_u=0.1,
    minimum_dwell_ns=500_000_000,
)
_CANDIDATE = mutate_genome(_BASELINE, sigma=0.1, seed=91)
_BANK_PATH = "sim/kinematic/scenarios/phase6_maze_bank.json"


def _fixture(split="training"):
    bank = load_maze_bank(_BANK_PATH)
    return {
        "training": bank.training_fixtures(),
        "validation": bank.validation_fixtures(),
        "held_out": bank.held_out_fixtures(),
    }[split][0]


def test_benchmark_config_rejects_invalid_period_timeout_and_sensor_inputs():
    with pytest.raises(ValueError, match="duration must exceed"):
        BenchmarkConfig(episode_duration_s=0.15)
    with pytest.raises(ValueError, match="cannot reach"):
        BenchmarkConfig(episode_duration_s=2.0, max_episode_steps=2)
    with pytest.raises(ValueError, match="discount_rate"):
        BenchmarkConfig(discount_rate_per_s=math.nan)
    with pytest.raises(ValueError, match="lidar_beam_count"):
        BenchmarkConfig(lidar_beam_count=3)
    with pytest.raises(ValueError, match="curved-path scan minimum"):
        BenchmarkConfig(lidar_beam_count=180)
    with pytest.raises(ValueError, match="candidate lease"):
        BenchmarkConfig(control_period_s=0.11)
    with pytest.raises(ValueError, match="P5.6 policy limit"):
        BenchmarkConfig(robot_max_speed_mps=0.21)


def test_episode_runner_is_deterministic_role_paired_and_fixture_only():
    runner = SILBenchmarkRunner(
        BenchmarkConfig(episode_duration_s=0.5, max_episode_steps=11)
    )
    fixture = _fixture()
    first = runner.run_episode(_BASELINE, fixture, seed=711)
    second = runner.run_episode(_BASELINE, fixture, seed=711)

    assert first == second
    assert first.terminal_kind is TerminalKind.TIMEOUT
    assert first.step_count == 10
    assert first.evidence_class == "SIL_DEVELOPMENT_FIXTURE_NOT_PROMOTION_ELIGIBLE"
    assert len(first.scenario_sha256) == 64
    assert len(first.evaluation_profile_id) == 64
    assert first.official_score is None
    rows = {result.role: result for result in first.episode_results}
    assert set(rows) == {Role.GUARDIAN, Role.EXPLORER}
    assert rows[Role.GUARDIAN].training_return == pytest.approx(
        -math.exp(-0.01 * 0.35)
    )
    assert rows[Role.EXPLORER].training_return == pytest.approx(
        math.exp(-0.01 * 0.35)
    )
    assert all(row.completed for row in rows.values())
    assert all(row.policy_id == _BASELINE.sha256 for row in rows.values())
    with pytest.raises(ValueError, match="cannot claim promotion-eligible"):
        replace(first, evidence_class="G4_ACCEPTED")


def test_paired_episode_uses_exact_same_fixture_seed_and_both_roles():
    runner = SILBenchmarkRunner(
        BenchmarkConfig(episode_duration_s=0.5, max_episode_steps=11)
    )
    pair = runner.run_paired_episode(
        _CANDIDATE, _BASELINE, _fixture(), seed=712
    )
    assert pair.candidate.scenario_id == pair.baseline.scenario_id
    assert pair.candidate.seed == pair.baseline.seed == 712
    assert pair.candidate.policy_id == _CANDIDATE.sha256
    assert pair.baseline.policy_id == _BASELINE.sha256
    assert {
        row.pair_key for row in pair.candidate.episode_results
    } == {
        row.pair_key for row in pair.baseline.episode_results
    }
    assert {
        row.role for row in pair.candidate.episode_results
    } == {Role.GUARDIAN, Role.EXPLORER}


def test_runner_rejects_heldout_and_invalid_pair_inputs():
    runner = SILBenchmarkRunner(
        BenchmarkConfig(episode_duration_s=0.5, max_episode_steps=11)
    )
    with pytest.raises(ValueError, match="held-out fixtures are reserved"):
        runner.run_episode(
            _BASELINE, _fixture("held_out"), seed=1
        )
    with pytest.raises(ValueError, match="must differ"):
        runner.run_paired_episode(
            _BASELINE, _BASELINE, _fixture(), seed=1
        )
    with pytest.raises(ValueError, match="non-negative"):
        runner.run_episode(_BASELINE, _fixture(), seed=True)


def test_swept_wall_check_accounts_for_robot_radius_and_arc_chord_error():
    wall = (Segment((0.0, 0.2), (2.0, 0.2)),)
    start = Pose2D(0.5, 0.0, 0.0)
    clear_end = Pose2D(1.5, 0.0, 0.0)
    contact_end = Pose2D(1.5, 0.1, 0.0)
    assert not _swept_wall_contact(
        start, clear_end, Actuation(0.2, 0.0), 0.05, 0.15, 0.0, wall
    )
    assert _swept_wall_contact(
        start, contact_end, Actuation(0.2, 0.0), 0.05, 0.15, 0.0, wall
    )
    assert _segment_distance(
        (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)
    ) == pytest.approx(1.0)


def test_swept_robot_contact_uses_relative_motion_and_footprint_radius():
    guardian_start = Pose2D(0.0, 0.0, 0.0)
    guardian_end = Pose2D(1.0, 0.0, 0.0)
    explorer_start = Pose2D(0.0, 0.29, math.pi)
    explorer_end = Pose2D(1.0, 0.29, math.pi)
    assert _swept_robot_contact(
        guardian_start,
        guardian_end,
        Actuation(0.2, 0.0),
        explorer_start,
        explorer_end,
        Actuation(0.2, 0.0),
        0.05,
        0.15,
    )
    assert not _swept_robot_contact(
        guardian_start,
        guardian_end,
        Actuation(0.2, 0.0),
        Pose2D(0.0, 0.31, math.pi),
        Pose2D(1.0, 0.31, math.pi),
        Actuation(0.2, 0.0),
        0.05,
        0.15,
    )


def test_lidar_noise_is_common_random_per_beam_across_paired_worlds():
    raycaster = FirstHitRaycaster(max_range_m=5.0)
    sensor_a = LidarSensor(
        raycaster=raycaster,
        beam_count=360,
        max_range_m=5.0,
        noise_std_m=0.05,
        seed=814,
    )
    sensor_b = LidarSensor(
        raycaster=raycaster,
        beam_count=360,
        max_range_m=5.0,
        noise_std_m=0.05,
        seed=814,
    )
    geometry_a = WorldGeometry(
        (
            Segment((2.0, -3.0), (2.0, 3.0)),
            Segment((-3.0, 2.0), (3.0, 2.0)),
        )
    )
    geometry_b = WorldGeometry((Segment((3.0, -3.0), (3.0, 3.0)),))
    pose = Pose2D(0.0, 0.0, 0.0)
    observation_a = sensor_a.observe(pose, 0.0, geometry_a)
    observation_b = sensor_b.observe(pose, 0.0, geometry_b)
    common_valid = [
        index
        for index in range(360)
        if observation_a.valid_mask[index] and observation_b.valid_mask[index]
    ]
    assert common_valid
    for index in common_valid:
        angle = -math.pi + 2.0 * math.pi * index / 360
        distance_a = raycaster.cast((0.0, 0.0), angle, geometry_a.static_segments)
        distance_b = raycaster.cast((0.0, 0.0), angle, geometry_b.static_segments)
        assert distance_a is not None and distance_b is not None
        assert observation_a.ranges_m[index] - distance_a == pytest.approx(
            observation_b.ranges_m[index] - distance_b,
            abs=1e-12,
        )


def test_pair_result_rejects_different_scenario_seed_or_split():
    runner = SILBenchmarkRunner(
        BenchmarkConfig(episode_duration_s=0.5, max_episode_steps=11)
    )
    result = runner.run_episode(_BASELINE, _fixture(), seed=713)
    candidate = runner.run_episode(_CANDIDATE, _fixture(), seed=714)

    with pytest.raises(ValueError, match="share scenario"):
        PairedSILBenchmarkEpisode(candidate, result)
