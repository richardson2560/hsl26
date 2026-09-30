"""Deterministic, fixture-only episode runner for P6.3 SIL contracts."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Iterable, Mapping

from hsl_core.learning.evolution import (
    EpisodeResult,
    EvaluationSplit,
    TacticalGenome,
)
from hsl_core.match import (
    EventEvidence,
    Role,
    StageManager,
    StagePhase,
    StageProfile,
    TerminalKind,
)
from hsl_core.types import Pose2D

from .autonomous import KinematicAutonomousPolicy, P56Profile
from .common import Actuation, PoseEstimate
from .match import (
    MatchRole,
    RoleEndpoint,
    SILPolicyInput,
    TwoRobotMatch,
    capture_rule_event,
)
from .maze_bank import MazeFixture
from .plant import KinematicPlant, SilDeadReckoningEstimator
from .raycaster import FirstHitRaycaster
from .sensors import LidarSensor


_AUTHORIZATION_REF = "p63-synthetic-fixture-only"
_FREEZE_DURATION_NS = 150_000_000
_LEASE_DURATION_NS = 500_000_000
_SCORE_PROFILE_ID = "synthetic-zero-sum-win-loss-surrogate-v1"
_EVIDENCE_CLASS = "SIL_DEVELOPMENT_FIXTURE_NOT_PROMOTION_ELIGIBLE"
_ROLES = (Role.GUARDIAN, Role.EXPLORER)


@dataclass(frozen=True, slots=True)
class BenchmarkConfig:
    episode_duration_s: float = 30.0
    control_period_s: float = 0.05
    discount_rate_per_s: float = 0.01
    lidar_beam_count: int = 360
    lidar_max_range_m: float = 5.0
    lidar_noise_std_m: float = 0.0
    robot_radius_m: float = 0.15
    robot_max_speed_mps: float = 0.20
    max_episode_steps: int = 100_000

    def __post_init__(self) -> None:
        for value, name in (
            (self.episode_duration_s, "episode_duration_s"),
            (self.control_period_s, "control_period_s"),
            (self.lidar_max_range_m, "lidar_max_range_m"),
            (self.robot_radius_m, "robot_radius_m"),
            (self.robot_max_speed_mps, "robot_max_speed_mps"),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if (
            isinstance(self.discount_rate_per_s, bool)
            or not isinstance(self.discount_rate_per_s, (int, float))
            or not math.isfinite(self.discount_rate_per_s)
            or self.discount_rate_per_s < 0.0
        ):
            raise ValueError("discount_rate_per_s must be finite and non-negative")
        if (
            not isinstance(self.lidar_beam_count, int)
            or isinstance(self.lidar_beam_count, bool)
            or self.lidar_beam_count < 4
        ):
            raise ValueError("lidar_beam_count must be an integer >= 4")
        if (
            isinstance(self.lidar_noise_std_m, bool)
            or not isinstance(self.lidar_noise_std_m, (int, float))
            or not math.isfinite(self.lidar_noise_std_m)
            or self.lidar_noise_std_m < 0.0
        ):
            raise ValueError("lidar_noise_std_m must be finite and non-negative")
        if (
            not isinstance(self.max_episode_steps, int)
            or isinstance(self.max_episode_steps, bool)
            or self.max_episode_steps < 1
        ):
            raise ValueError("max_episode_steps must be a positive integer")
        duration_ns = round(self.episode_duration_s * 1_000_000_000)
        period_ns = round(self.control_period_s * 1_000_000_000)
        if duration_ns <= _FREEZE_DURATION_NS:
            raise ValueError("episode duration must exceed the stage freeze")
        if period_ns <= 0:
            raise ValueError("control period must resolve to at least one nanosecond")
        p56 = P56Profile()
        if self.control_period_s > p56.candidate_lease_s:
            raise ValueError("control period must not exceed the P5.6 candidate lease")
        if self.lidar_beam_count < p56.curvature_minimum_scan_beams:
            raise ValueError(
                "benchmark LiDAR must meet the P5.6 curved-path scan minimum"
            )
        if not math.isclose(
            self.robot_radius_m,
            p56.robot_radius_m,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("benchmark robot radius must match the P5.6 policy profile")
        if self.robot_max_speed_mps > p56.speed_max_mps:
            raise ValueError("benchmark plant speed must not exceed the P5.6 policy limit")
        required_steps = math.ceil(duration_ns / period_ns) + 1
        if self.max_episode_steps < required_steps:
            raise ValueError("max_episode_steps cannot reach the configured timeout")

    @property
    def episode_duration_ns(self) -> int:
        return round(self.episode_duration_s * 1_000_000_000)

    @property
    def control_period_ns(self) -> int:
        return round(self.control_period_s * 1_000_000_000)


@dataclass(frozen=True, slots=True)
class SILBenchmarkEpisode:
    """A synthetic episode outcome, explicitly ineligible for promotion evidence."""

    scenario_id: str
    seed: int
    split: EvaluationSplit
    policy_id: str
    scenario_sha256: str
    evaluation_profile_id: str
    terminal_kind: TerminalKind
    active_duration_s: float
    step_count: int
    guardian_wall_contacts: int
    explorer_wall_contacts: int
    episode_results: tuple[EpisodeResult, EpisodeResult]
    role_policy_ids: tuple[tuple[Role, str], tuple[Role, str]]
    robot_identity_by_role: tuple[tuple[Role, str], tuple[Role, str]]
    telemetry: tuple["SILEpisodeSample", ...] = ()
    evidence_class: str = _EVIDENCE_CLASS
    official_score: None = None

    def __post_init__(self) -> None:
        if not self.scenario_id or not self.policy_id:
            raise ValueError("scenario_id and policy_id must be non-empty")
        if len(self.policy_id) != 64 or any(
            char not in "0123456789abcdef" for char in self.policy_id
        ):
            raise ValueError("policy_id must be a lowercase SHA-256 genome digest")
        for value, name in (
            (self.scenario_sha256, "scenario_sha256"),
            (self.evaluation_profile_id, "evaluation_profile_id"),
        ):
            if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
                raise ValueError(f"{name} must be a lowercase SHA-256 digest")
        if (
            not isinstance(self.seed, int)
            or isinstance(self.seed, bool)
            or self.seed < 0
        ):
            raise ValueError("seed must be a non-negative integer")
        if not isinstance(self.split, EvaluationSplit):
            raise ValueError("split must be an EvaluationSplit")
        if not isinstance(self.terminal_kind, TerminalKind) or self.terminal_kind not in (
            TerminalKind.CAPTURE,
            TerminalKind.TIMEOUT,
        ):
            raise ValueError("benchmark episode must end by capture or timeout")
        if (
            isinstance(self.active_duration_s, bool)
            or not isinstance(self.active_duration_s, (int, float))
            or not math.isfinite(self.active_duration_s)
            or self.active_duration_s < 0.0
            or not isinstance(self.step_count, int)
            or isinstance(self.step_count, bool)
            or self.step_count < 1
            or not isinstance(self.guardian_wall_contacts, int)
            or isinstance(self.guardian_wall_contacts, bool)
            or self.guardian_wall_contacts < 0
            or not isinstance(self.explorer_wall_contacts, int)
            or isinstance(self.explorer_wall_contacts, bool)
            or self.explorer_wall_contacts < 0
        ):
            raise ValueError("episode duration, steps, or wall-contact counts are invalid")
        if (
            not isinstance(self.episode_results, tuple)
            or len(self.episode_results) != 2
            or any(
                not isinstance(result, EpisodeResult)
                for result in self.episode_results
            )
            or {
                result.role for result in self.episode_results
            }
            != {Role.GUARDIAN, Role.EXPLORER}
        ):
            raise ValueError("episode results must contain exactly one row per role")
        if (
            not isinstance(self.role_policy_ids, tuple)
            or len(self.role_policy_ids) != 2
            or any(
                not isinstance(item, tuple)
                or len(item) != 2
                or not isinstance(item[0], Role)
                or not isinstance(item[1], str)
                for item in self.role_policy_ids
            )
        ):
            raise ValueError("role_policy_ids must contain one role and digest per role")
        expected_policy_ids = dict(self.role_policy_ids)
        if (
            set(expected_policy_ids) != {Role.GUARDIAN, Role.EXPLORER}
            or any(
                len(policy_id) != 64
                or any(char not in "0123456789abcdef" for char in policy_id)
                for policy_id in expected_policy_ids.values()
            )
            or any(
                result.policy_id != expected_policy_ids[result.role]
                or result.scenario_id != self.scenario_id
                or result.seed != self.seed
                or result.split is not self.split
                or result.evaluation_profile_id != self.evaluation_profile_id
                for result in self.episode_results
            )
        ):
            raise ValueError("episode result provenance must match its SIL run")
        if (
            not isinstance(self.robot_identity_by_role, tuple)
            or len(self.robot_identity_by_role) != 2
            or any(
                not isinstance(item, tuple)
                or len(item) != 2
                or not isinstance(item[0], Role)
                or not isinstance(item[1], str)
                or not item[1].strip()
                for item in self.robot_identity_by_role
            )
            or {item[0] for item in self.robot_identity_by_role}
            != {Role.GUARDIAN, Role.EXPLORER}
            or len({item[1] for item in self.robot_identity_by_role}) != 2
        ):
            raise ValueError(
                "robot_identity_by_role must uniquely assign two virtual robot IDs"
            )
        if self.official_score is not None:
            raise ValueError("synthetic SIL episodes cannot contain official scores")
        if self.evidence_class != _EVIDENCE_CLASS:
            raise ValueError("SIL fixture output cannot claim promotion-eligible evidence")
        if not isinstance(self.telemetry, tuple) or any(
            not isinstance(sample, SILEpisodeSample) for sample in self.telemetry
        ):
            raise ValueError("telemetry must be a tuple of validated SIL samples")


@dataclass(frozen=True, slots=True)
class SILEpisodeSample:
    stamp_ns: int
    guardian_xy_m: tuple[float, float]
    explorer_xy_m: tuple[float, float]
    guardian_heading_rad: float
    explorer_heading_rad: float
    guardian_command: tuple[float, float]
    explorer_command: tuple[float, float]
    guardian_option: str
    explorer_option: str

    def __post_init__(self) -> None:
        if not isinstance(self.stamp_ns, int) or isinstance(self.stamp_ns, bool) or self.stamp_ns < 0:
            raise ValueError("telemetry timestamp must be a non-negative integer")
        for values, name in (
            (self.guardian_xy_m, "guardian_xy_m"),
            (self.explorer_xy_m, "explorer_xy_m"),
            (self.guardian_command, "guardian_command"),
            (self.explorer_command, "explorer_command"),
        ):
            if (
                not isinstance(values, tuple)
                or len(values) != 2
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    for value in values
                )
            ):
                raise ValueError(f"{name} must contain two finite numeric values")
        for value, name in (
            (self.guardian_heading_rad, "guardian_heading_rad"),
            (self.explorer_heading_rad, "explorer_heading_rad"),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if (
            not isinstance(self.guardian_option, str)
            or not self.guardian_option
            or not isinstance(self.explorer_option, str)
            or not self.explorer_option
        ):
            raise ValueError("telemetry option names must not be empty")


@dataclass(frozen=True, slots=True)
class PairedSILBenchmarkEpisode:
    candidate: SILBenchmarkEpisode
    baseline: SILBenchmarkEpisode

    def __post_init__(self) -> None:
        if not isinstance(self.candidate, SILBenchmarkEpisode) or not isinstance(
            self.baseline, SILBenchmarkEpisode
        ):
            raise ValueError("paired outcome requires two SIL benchmark episodes")
        if self.candidate.policy_id == self.baseline.policy_id:
            raise ValueError("paired candidate and baseline genomes must differ")
        if (
            self.candidate.scenario_id != self.baseline.scenario_id
            or self.candidate.seed != self.baseline.seed
            or self.candidate.split is not self.baseline.split
            or self.candidate.scenario_sha256 != self.baseline.scenario_sha256
            or self.candidate.evaluation_profile_id
            != self.baseline.evaluation_profile_id
        ):
            raise ValueError("paired SIL runs must share scenario, seed, and split")
        candidate_keys = {
            result.pair_key for result in self.candidate.episode_results
        }
        baseline_keys = {
            result.pair_key for result in self.baseline.episode_results
        }
        if candidate_keys != baseline_keys:
            raise ValueError("paired SIL result provenance does not match")


@dataclass(frozen=True, slots=True)
class RoleSwappedSILEvaluation:
    """Paired role-wise candidate-vs-baseline results from separate matches."""

    candidate_results: tuple[EpisodeResult, EpisodeResult]
    baseline_results: tuple[EpisodeResult, EpisodeResult]
    scenario_sha256: str
    evidence_class: str = _EVIDENCE_CLASS

    def __post_init__(self) -> None:
        if self.evidence_class != _EVIDENCE_CLASS:
            raise ValueError("role-swapped fixture results cannot be promotion evidence")
        for rows in (self.candidate_results, self.baseline_results):
            if (
                not isinstance(rows, tuple)
                or len(rows) != 2
                or {row.role for row in rows} != {Role.GUARDIAN, Role.EXPLORER}
            ):
                raise ValueError("role-swapped evaluation requires both policy roles")
            if any(not isinstance(row, EpisodeResult) for row in rows):
                raise ValueError("role-swapped results must be EpisodeResult records")
        candidate_by_role = {row.role: row for row in self.candidate_results}
        baseline_by_role = {row.role: row for row in self.baseline_results}
        all_rows = (*self.candidate_results, *self.baseline_results)
        if (
            len({row.pair_key for row in all_rows}) != 1
            or len({row.split for row in all_rows}) != 1
            or len({row.policy_id for row in self.candidate_results}) != 1
            or len({row.policy_id for row in self.baseline_results}) != 1
            or len(self.scenario_sha256) != 64
            or any(char not in "0123456789abcdef" for char in self.scenario_sha256)
            or any(
                not row.completed
                or row.split is EvaluationSplit.HELD_OUT
                for row in all_rows
            )
            or any(
            candidate_by_role[role].pair_key != baseline_by_role[role].pair_key
            or candidate_by_role[role].policy_id == baseline_by_role[role].policy_id
            for role in _ROLES
            )
        ):
            raise ValueError("role-wise baseline and candidate provenance must pair exactly")


class SILBenchmarkRunner:
    """Runs one genome against the fixture-only Guardian/Explorer policies."""

    def __init__(self, config: BenchmarkConfig = BenchmarkConfig()) -> None:
        if not isinstance(config, BenchmarkConfig):
            raise ValueError("config must be a BenchmarkConfig")
        self.config = config

    def run_episode(
        self,
        genome: TacticalGenome,
        fixture: MazeFixture,
        *,
        seed: int,
        explorer_genome: TacticalGenome | None = None,
        robot_identity_by_role: Mapping[Role, str] | None = None,
        collect_telemetry: bool = False,
    ) -> SILBenchmarkEpisode:
        if not isinstance(genome, TacticalGenome):
            raise ValueError("genome must be a TacticalGenome")
        if explorer_genome is not None and not isinstance(
            explorer_genome, TacticalGenome
        ):
            raise ValueError("explorer_genome must be a TacticalGenome")
        if not isinstance(collect_telemetry, bool):
            raise ValueError("collect_telemetry must be boolean")
        if robot_identity_by_role is None:
            robot_identity_by_role = {
                Role.GUARDIAN: "sil-robot-guardian",
                Role.EXPLORER: "sil-robot-explorer",
            }
        if (
            not isinstance(robot_identity_by_role, Mapping)
            or set(robot_identity_by_role) != {Role.GUARDIAN, Role.EXPLORER}
            or any(
                not isinstance(identity, str) or not identity.strip()
                for identity in robot_identity_by_role.values()
            )
            or len(set(robot_identity_by_role.values())) != 2
        ):
            raise ValueError(
                "robot_identity_by_role must uniquely assign both SIL robot identities"
            )
        identity_rows = (
            (Role.GUARDIAN, robot_identity_by_role[Role.GUARDIAN]),
            (Role.EXPLORER, robot_identity_by_role[Role.EXPLORER]),
        )
        explorer_genome = explorer_genome or genome
        split = _fixture_split(fixture)
        _validate_seed(seed)
        match, managers, policies, profile_id = self._build_match(
            genome,
            explorer_genome,
            fixture,
            seed,
            robot_identity_by_role,
        )
        collision = False
        wall_contacts = {Role.GUARDIAN: False, Role.EXPLORER: False}
        overrides = {Role.GUARDIAN: 0, Role.EXPLORER: 0}
        violations = {Role.GUARDIAN: 0, Role.EXPLORER: 0}
        terminal_kind: TerminalKind | None = None
        terminal_stamp_ns = 0
        steps = 0
        telemetry: list[SILEpisodeSample] = []
        if collect_telemetry:
            telemetry.append(
                SILEpisodeSample(
                    stamp_ns=match.stamp_ns,
                    guardian_xy_m=(
                        match.guardian.plant.state.pose.x_m,
                        match.guardian.plant.state.pose.y_m,
                    ),
                    explorer_xy_m=(
                        match.explorer.plant.state.pose.x_m,
                        match.explorer.plant.state.pose.y_m,
                    ),
                    guardian_heading_rad=match.guardian.plant.state.pose.theta_rad,
                    explorer_heading_rad=match.explorer.plant.state.pose.theta_rad,
                    guardian_command=(0.0, 0.0),
                    explorer_command=(0.0, 0.0),
                    guardian_option="INITIAL",
                    explorer_option="INITIAL",
                )
            )

        for _ in range(self.config.max_episode_steps):
            remaining_ns = self.config.episode_duration_ns - match.stamp_ns
            if remaining_ns <= 0:
                states = _stage_states(managers, match.stamp_ns, fixture)
                terminal_kind = _shared_terminal_kind(states)
                terminal_stamp_ns = match.stamp_ns
                break
            dt_ns = min(self.config.control_period_ns, remaining_ns)
            start_guardian = match.guardian.plant.state.pose
            start_explorer = match.explorer.plant.state.pose
            tick = match.step(dt_ns / 1_000_000_000.0)
            steps += 1
            if collect_telemetry:
                guardian_trace = policies[Role.GUARDIAN].last_trace
                explorer_trace = policies[Role.EXPLORER].last_trace
                telemetry.append(
                    SILEpisodeSample(
                        stamp_ns=tick.stamp_ns,
                        guardian_xy_m=(
                            match.guardian.plant.state.pose.x_m,
                            match.guardian.plant.state.pose.y_m,
                        ),
                        explorer_xy_m=(
                            match.explorer.plant.state.pose.x_m,
                            match.explorer.plant.state.pose.y_m,
                        ),
                        guardian_heading_rad=match.guardian.plant.state.pose.theta_rad,
                        explorer_heading_rad=match.explorer.plant.state.pose.theta_rad,
                        guardian_command=(
                            tick.guardian_command.linear_mps,
                            tick.guardian_command.angular_rps,
                        ),
                        explorer_command=(
                            tick.explorer_command.linear_mps,
                            tick.explorer_command.angular_rps,
                        ),
                        guardian_option=(
                            guardian_trace.selected_kind.name
                            if guardian_trace.selected_kind is not None
                            else "NONE"
                        ),
                        explorer_option=(
                            explorer_trace.selected_kind.name
                            if explorer_trace.selected_kind is not None
                            else "NONE"
                        ),
                    )
                )
            collision = collision or tick.truth.robot_collision or _swept_robot_contact(
                start_guardian,
                match.guardian.plant.state.pose,
                tick.guardian_command,
                start_explorer,
                match.explorer.plant.state.pose,
                tick.explorer_command,
                dt_ns / 1_000_000_000.0,
                self.config.robot_radius_m,
            )
            wall_contacts[Role.GUARDIAN] |= _swept_wall_contact(
                start_guardian,
                match.guardian.plant.state.pose,
                tick.guardian_command,
                dt_ns / 1_000_000_000.0,
                self.config.robot_radius_m,
                match.guardian.pose_estimator.estimate.position_error_bound_m,
                fixture.geometry.static_segments,
            )
            wall_contacts[Role.EXPLORER] |= _swept_wall_contact(
                start_explorer,
                match.explorer.plant.state.pose,
                tick.explorer_command,
                dt_ns / 1_000_000_000.0,
                self.config.robot_radius_m,
                match.explorer.pose_estimator.estimate.position_error_bound_m,
                fixture.geometry.static_segments,
            )
            for role, policy, command in (
                (Role.GUARDIAN, policies[Role.GUARDIAN], tick.guardian_command),
                (Role.EXPLORER, policies[Role.EXPLORER], tick.explorer_command),
            ):
                decision = policy.last_trace.safety_decision
                if decision in (0, 2):
                    overrides[role] += 1
                if decision == 0 and (
                    command.linear_mps != 0.0 or command.angular_rps != 0.0
                ):
                    violations[role] += 1

            if (
                tick.truth.capture_interval is not None
                and tick.guardian_command.angular_rps == 0.0
                and tick.explorer_command.angular_rps == 0.0
                and tick.truth.capture_interval.lower_ns >= _FREEZE_DURATION_NS
                and tick.truth.capture_interval.upper_ns
                < self.config.episode_duration_ns
            ):
                terminal_kind = TerminalKind.CAPTURE
                terminal_stamp_ns = tick.stamp_ns
                self._resolve_synthetic_capture(tick, managers, fixture, seed)
                break

            states = _stage_states(managers, tick.stamp_ns, fixture)
            terminal_kind = _shared_terminal_kind(states, allow_active=True)
            if terminal_kind is TerminalKind.TIMEOUT:
                terminal_stamp_ns = tick.stamp_ns
                break
            terminal_kind = None

        if terminal_kind not in (TerminalKind.CAPTURE, TerminalKind.TIMEOUT):
            raise RuntimeError(
                "SIL episode exhausted its step budget before a valid terminal outcome"
            )

        active_start_ns = _FREEZE_DURATION_NS
        active_duration_s = max(
            0.0, (terminal_stamp_ns - active_start_ns) / 1_000_000_000.0
        )
        guardian_outcome = (
            1.0 if terminal_kind is TerminalKind.CAPTURE else -1.0
        ) * math.exp(
            -self.config.discount_rate_per_s * active_duration_s
        )
        rows = []
        for role in (Role.GUARDIAN, Role.EXPLORER):
            role_genome = genome if role is Role.GUARDIAN else explorer_genome
            role_return = (
                guardian_outcome if role is Role.GUARDIAN else -guardian_outcome
            )
            role_collision = collision or wall_contacts[role]
            rows.append(
                EpisodeResult(
                    policy_id=role_genome.sha256,
                    scenario_id=fixture.scenario_id,
                    split=split,
                    role=role,
                    seed=seed,
                    map_bank_id=fixture.map_bank_id,
                    opponent_bank_id=fixture.opponent_bank_id,
                    seed_bank_id=fixture.seed_bank_id,
                    training_return=role_return,
                    safety_overrides=overrides[role],
                    safety_violations=violations[role],
                    collisions=int(role_collision),
                    completed=True,
                    evaluation_profile_id=profile_id,
                )
            )
        return SILBenchmarkEpisode(
            scenario_id=fixture.scenario_id,
            seed=seed,
            split=split,
            policy_id=_match_policy_id(genome, explorer_genome),
            scenario_sha256=_fixture_sha256(fixture),
            evaluation_profile_id=profile_id,
            terminal_kind=terminal_kind,
            active_duration_s=active_duration_s,
            step_count=steps,
            guardian_wall_contacts=int(wall_contacts[Role.GUARDIAN]),
            explorer_wall_contacts=int(wall_contacts[Role.EXPLORER]),
            episode_results=(rows[0], rows[1]),
            role_policy_ids=(
                (Role.GUARDIAN, genome.sha256),
                (Role.EXPLORER, explorer_genome.sha256),
            ),
            robot_identity_by_role=identity_rows,
            telemetry=tuple(telemetry),
        )

    def run_paired_episode(
        self,
        candidate_genome: TacticalGenome,
        baseline_genome: TacticalGenome,
        fixture: MazeFixture,
        *,
        seed: int,
    ) -> PairedSILBenchmarkEpisode:
        """Pair genome self-play runs; use role-swapped evaluation for fitness."""
        if not isinstance(candidate_genome, TacticalGenome) or not isinstance(
            baseline_genome, TacticalGenome
        ):
            raise ValueError("candidate and baseline must be TacticalGenomes")
        if candidate_genome.sha256 == baseline_genome.sha256:
            raise ValueError("candidate and baseline genomes must differ")
        candidate = self.run_episode(candidate_genome, fixture, seed=seed)
        baseline = self.run_episode(baseline_genome, fixture, seed=seed)
        return PairedSILBenchmarkEpisode(candidate, baseline)

    def run_role_swapped_evaluation(
        self,
        candidate_genome: TacticalGenome,
        baseline_genome: TacticalGenome,
        fixture: MazeFixture,
        *,
        seed: int,
        baseline_reference: SILBenchmarkEpisode | None = None,
    ) -> RoleSwappedSILEvaluation:
        """Compare each candidate role against the fixed baseline opponent.

        The baseline-vs-baseline run supplies paired reference outcomes. Two
        further matches put only the candidate's Guardian or Explorer policy
        against the corresponding baseline policy, avoiding self-play
        confounding and the identically-zero balanced reward of a single
        zero-sum self-play comparison.
        """
        if not isinstance(candidate_genome, TacticalGenome) or not isinstance(
            baseline_genome, TacticalGenome
        ):
            raise ValueError("candidate and baseline must be TacticalGenomes")
        if candidate_genome.sha256 == baseline_genome.sha256:
            raise ValueError("candidate and baseline genomes must differ")
        if baseline_reference is not None and not isinstance(
            baseline_reference, SILBenchmarkEpisode
        ):
            raise ValueError("baseline_reference must be a SILBenchmarkEpisode")
        baseline_match = (
            self.run_episode(baseline_genome, fixture, seed=seed)
            if baseline_reference is None
            else baseline_reference
        )
        if (
            baseline_match.policy_id != baseline_genome.sha256
            or baseline_match.scenario_id != fixture.scenario_id
            or baseline_match.seed != seed
            or baseline_match.split.value != fixture.split
            or baseline_match.scenario_sha256 != _fixture_sha256(fixture)
            or baseline_match.evaluation_profile_id
            != _config_hash(self.config, fixture, seed)
            or dict(baseline_match.role_policy_ids)
            != {
                Role.GUARDIAN: baseline_genome.sha256,
                Role.EXPLORER: baseline_genome.sha256,
            }
        ):
            raise ValueError("baseline reference does not match evaluation provenance")
        candidate_guardian_match = self.run_episode(
            candidate_genome,
            fixture,
            seed=seed,
            explorer_genome=baseline_genome,
        )
        candidate_explorer_match = self.run_episode(
            baseline_genome,
            fixture,
            seed=seed,
            explorer_genome=candidate_genome,
        )
        candidate_by_role = {
            Role.GUARDIAN: next(
                row
                for row in candidate_guardian_match.episode_results
                if row.role is Role.GUARDIAN
            ),
            Role.EXPLORER: next(
                row
                for row in candidate_explorer_match.episode_results
                if row.role is Role.EXPLORER
            ),
        }
        baseline_by_role = {
            row.role: row for row in baseline_match.episode_results
        }
        return RoleSwappedSILEvaluation(
            candidate_results=(
                candidate_by_role[Role.GUARDIAN],
                candidate_by_role[Role.EXPLORER],
            ),
            baseline_results=(
                baseline_by_role[Role.GUARDIAN],
                baseline_by_role[Role.EXPLORER],
            ),
            scenario_sha256=_fixture_sha256(fixture),
        )

    def _build_match(
        self,
        guardian_genome,
        explorer_genome,
        fixture,
        seed,
        robot_identity_by_role,
    ):
        config_hash = _config_hash(
            self.config,
            fixture,
            seed,
            robot_identity_by_role,
        )
        stage_id = f"p63-{fixture.scenario_id}-{seed}"
        localization_epoch = fixture.topology.localization_epoch
        guardian_policy = _policy(guardian_genome, Role.GUARDIAN)
        explorer_policy = _policy(explorer_genome, Role.EXPLORER)
        goal_node = (
            fixture.synthetic_goal_rc[0] * len(fixture.rows[0])
            + fixture.synthetic_goal_rc[1]
        )

        def endpoint(role, cell, yaw, policy, goal):
            core_role = (
                Role.GUARDIAN if role is MatchRole.GUARDIAN else Role.EXPLORER
            )
            pose = Pose2D(*fixture.cell_pose(cell), yaw)
            return RoleEndpoint(
                role=role,
                namespace=f"/robot_{role.value}",
                plant=KinematicPlant(
                    pose,
                    max_linear_mps=self.config.robot_max_speed_mps,
                ),
                sensor=LidarSensor(
                    raycaster=FirstHitRaycaster(
                        max_range_m=self.config.lidar_max_range_m
                    ),
                    beam_count=self.config.lidar_beam_count,
                    max_range_m=self.config.lidar_max_range_m,
                    noise_std_m=self.config.lidar_noise_std_m,
                    seed=_sensor_seed(seed, role),
                ),
                policy=policy,
                radius_m=self.config.robot_radius_m,
                pose_estimator=SilDeadReckoningEstimator(
                    PoseEstimate(
                        0,
                        "map",
                        f"p63-clock:{stage_id}",
                        localization_epoch,
                        pose,
                        0.0,
                        0.0,
                    )
                ),
                topology_graph=fixture.topology,
                synthetic_goal_node_id=goal,
                synthetic_goal_zone_id=(
                    "sil-fixture:p63-goal" if role is MatchRole.EXPLORER else ""
                ),
                robot_identity=robot_identity_by_role[core_role],
            )

        guardian = endpoint(
            MatchRole.GUARDIAN,
            fixture.guardian_start_rc,
            0.0,
            guardian_policy,
            None,
        )
        explorer = endpoint(
            MatchRole.EXPLORER,
            fixture.explorer_start_rc,
            math.pi,
            explorer_policy,
            goal_node,
        )
        managers = tuple(
            _stage_manager(
                role,
                fixture,
                stage_id,
                config_hash,
                self.config.episode_duration_ns,
            )
            for role in (Role.GUARDIAN, Role.EXPLORER)
        )
        match = TwoRobotMatch(
            guardian,
            explorer,
            fixture.geometry,
            duel_stage_managers=managers,
        )
        return (
            match,
            managers,
            {
                Role.GUARDIAN: guardian_policy,
                Role.EXPLORER: explorer_policy,
            },
            config_hash,
        )

    @staticmethod
    def _resolve_synthetic_capture(tick, managers, fixture, seed) -> None:
        guardian_state, explorer_state = tick.role_match_states or (None, None)
        if guardian_state is None or explorer_state is None:
            raise RuntimeError("duel capture requires role-scoped stage snapshots")
        event = capture_rule_event(
            tick.truth,
            event_id=f"p63-capture-{fixture.scenario_id}-{seed}-{tick.stamp_ns}",
            stage_id=guardian_state.meta.stage_id,
            clock_epoch=guardian_state.meta.clock_epoch,
            localization_epoch=guardian_state.meta.localization_epoch,
            input_ids=(
                f"synthetic-referee:{fixture.scenario_id}:{seed}",
                f"synthetic-scan-pair:{tick.stamp_ns}",
            ),
            authorization_ref=_AUTHORIZATION_REF,
        )
        if event is None or event.evidence is not EventEvidence.SIM_TRUTH:
            raise RuntimeError("referee capture could not be encoded as fixture evidence")
        for manager in managers:
            result = manager.resolve_event(event, now_ns=tick.stamp_ns)
            if not result.accepted:
                raise RuntimeError(
                    f"synthetic capture was not terminalized: {result.reason}"
                )


def _policy(genome: TacticalGenome, role: Role) -> KinematicAutonomousPolicy:
    match_role = (
        MatchRole.GUARDIAN if role is Role.GUARDIAN else MatchRole.EXPLORER
    )
    return KinematicAutonomousPolicy(
        match_role,
        utility_profile=genome.utility_profile(
            role, f"p63:{genome.sha256}:{role.name.lower()}"
        ),
        hysteresis_delta_u=genome.hysteresis_delta_u,
        minimum_dwell_ns=genome.minimum_dwell_ns,
    )


def _stage_manager(role, fixture, stage_id, config_hash, duration_ns):
    profile = StageProfile(
        profile_id="p63-sil-development-fixture-v1",
        score_profile_id=_SCORE_PROFILE_ID,
        freeze_duration_ns=_FREEZE_DURATION_NS,
        stage_duration_ns=duration_ns,
        lease_duration_ns=_LEASE_DURATION_NS,
        zone_update_window_ns=_FREEZE_DURATION_NS,
        max_request_records=16,
        max_event_records=16,
        max_audit_records=64,
        config_hash=config_hash,
        source_id="p63-sil-benchmark",
        source_session=stage_id,
        clock_epoch=f"p63-clock:{stage_id}",
        localization_epoch=fixture.topology.localization_epoch,
        organizer_authorization_ref=_AUTHORIZATION_REF,
        approved_authorization_refs=(_AUTHORIZATION_REF,),
        approved_zone_frames=("map",),
        allow_sim_truth_terminal=True,
    )
    manager = StageManager(
        profile,
        initial_stage_id=stage_id,
        stage_number=1,
        role=role,
    )
    started = manager.start_stage(
        request_id=f"start:{stage_id}:{role.name.lower()}",
        expected_stage_id=stage_id,
        official_start_stamp_ns=0,
        organizer_authorization_ref=_AUTHORIZATION_REF,
        now_ns=0,
    )
    if not started.accepted:
        raise RuntimeError(f"SIL stage failed to start: {started.reason}")
    return manager


def _fixture_split(fixture: MazeFixture) -> EvaluationSplit:
    if not isinstance(fixture, MazeFixture):
        raise ValueError("fixture must be a MazeFixture")
    try:
        split = EvaluationSplit(fixture.split)
    except ValueError as error:
        raise ValueError("fixture split is unsupported") from error
    if split is EvaluationSplit.HELD_OUT:
        raise ValueError(
            "held-out fixtures are reserved and cannot enter the SIL benchmark runner"
        )
    return split


def _validate_seed(seed: int) -> None:
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ValueError("episode seed must be a non-negative integer")


def _stage_states(managers, now_ns, fixture):
    return tuple(
        manager.snapshot(
            now_ns=now_ns,
            map_version=fixture.topology.map_version,
            topology_version=fixture.topology.topology_version,
        )
        for manager in managers
    )


def _shared_terminal_kind(states, *, allow_active=False):
    if len(states) != 2 or states[0].phase is not states[1].phase:
        raise RuntimeError("role stage managers diverged during benchmark episode")
    if states[0].role is not Role.GUARDIAN or states[1].role is not Role.EXPLORER:
        raise RuntimeError("benchmark terminal states lost role ownership")
    if states[0].phase in (StagePhase.FREEZE, StagePhase.ACTIVE) and allow_active:
        return TerminalKind.NONE
    if states[0].phase is not StagePhase.TERMINAL:
        raise RuntimeError("benchmark stage ended without terminal state")
    if states[0].terminal_kind != states[1].terminal_kind:
        raise RuntimeError("role stage managers disagree on terminal kind")
    return states[0].terminal_kind


def _config_hash(
    config: BenchmarkConfig,
    fixture: MazeFixture,
    seed: int,
    robot_identity_by_role: Mapping[Role, str] | None = None,
) -> str:
    payload = {
        "config": {
            "episode_duration_s": config.episode_duration_s,
            "control_period_s": config.control_period_s,
            "discount_rate_per_s": config.discount_rate_per_s,
            "lidar_beam_count": config.lidar_beam_count,
            "lidar_max_range_m": config.lidar_max_range_m,
            "lidar_noise_std_m": config.lidar_noise_std_m,
            "robot_radius_m": config.robot_radius_m,
            "robot_max_speed_mps": config.robot_max_speed_mps,
        },
        "scenario_id": fixture.scenario_id,
        "scenario_sha256": _fixture_sha256(fixture),
        "seed": seed,
        "robot_identity_by_role": {
            role.name: identity
            for role, identity in sorted(
                (
                    robot_identity_by_role
                    or {
                        Role.GUARDIAN: "sil-robot-guardian",
                        Role.EXPLORER: "sil-robot-explorer",
                    }
                ).items(),
                key=lambda item: item[0].name,
            )
        },
        "topology_version": fixture.topology.topology_version,
        "map_version": fixture.topology.map_version,
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _match_policy_id(
    guardian_genome: TacticalGenome, explorer_genome: TacticalGenome
) -> str:
    if guardian_genome.sha256 == explorer_genome.sha256:
        return guardian_genome.sha256
    payload = json.dumps(
        [guardian_genome.sha256, explorer_genome.sha256],
        separators=(",", ":"),
    ).encode("ascii")
    return hashlib.sha256(payload).hexdigest()


def _fixture_sha256(fixture: MazeFixture) -> str:
    payload = {
        "scenario_id": fixture.scenario_id,
        "split": fixture.split,
        "map_bank_id": fixture.map_bank_id,
        "opponent_bank_id": fixture.opponent_bank_id,
        "seed_bank_id": fixture.seed_bank_id,
        "cell_size_m": fixture.cell_size_m,
        "rows": fixture.rows,
        "guardian_start_rc": fixture.guardian_start_rc,
        "explorer_start_rc": fixture.explorer_start_rc,
        "synthetic_goal_rc": fixture.synthetic_goal_rc,
        "map_version": fixture.topology.map_version,
        "topology_version": fixture.topology.topology_version,
        "localization_epoch": fixture.topology.localization_epoch,
        "walls": tuple(
            (segment.start_xy, segment.end_xy)
            for segment in fixture.geometry.static_segments
        ),
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _sensor_seed(seed: int, role: Role) -> int:
    digest = hashlib.sha256(f"{seed}:{role.name}:lidar".encode("ascii")).digest()
    return int.from_bytes(digest[:8], "big")


def _swept_wall_contact(
    start: Pose2D,
    end: Pose2D,
    command: Actuation,
    dt_s: float,
    radius_m: float,
    pose_error_bound_m: float,
    walls: Iterable,
) -> bool:
    """Conservatively detect body contact using the exact arc's chord bound."""
    sagitta = _arc_sagitta(command, dt_s)
    contact_bound = radius_m + pose_error_bound_m + sagitta
    for wall in walls:
        if _segment_distance(
            (start.x_m, start.y_m),
            (end.x_m, end.y_m),
            wall.start_xy,
            wall.end_xy,
        ) <= contact_bound:
            return True
    return False


def _swept_robot_contact(
    guardian_start: Pose2D,
    guardian_end: Pose2D,
    guardian_command: Actuation,
    explorer_start: Pose2D,
    explorer_end: Pose2D,
    explorer_command: Actuation,
    dt_s: float,
    radius_m: float,
) -> bool:
    relative_start = (
        explorer_start.x_m - guardian_start.x_m,
        explorer_start.y_m - guardian_start.y_m,
    )
    relative_end = (
        explorer_end.x_m - guardian_end.x_m,
        explorer_end.y_m - guardian_end.y_m,
    )
    minimum_chord_distance = _point_segment_distance(
        (0.0, 0.0), relative_start, relative_end
    )
    contact_bound = (
        2.0 * radius_m
        + _arc_sagitta(guardian_command, dt_s)
        + _arc_sagitta(explorer_command, dt_s)
    )
    return minimum_chord_distance <= contact_bound


def _arc_sagitta(command: Actuation, dt_s: float) -> float:
    angle = abs(command.angular_rps * dt_s)
    if abs(command.angular_rps) <= 1e-12:
        return 0.0
    turn_radius = abs(command.linear_mps / command.angular_rps)
    return (
        2.0 * turn_radius
        if angle > math.pi
        else turn_radius * (1.0 - math.cos(angle / 2.0))
    )


def _segment_distance(a, b, c, d) -> float:
    if _segments_intersect(a, b, c, d):
        return 0.0
    return min(
        _point_segment_distance(a, c, d),
        _point_segment_distance(b, c, d),
        _point_segment_distance(c, a, b),
        _point_segment_distance(d, a, b),
    )


def _segments_intersect(a, b, c, d) -> bool:
    def cross(first, second, third):
        return (second[0] - first[0]) * (third[1] - first[1]) - (
            second[1] - first[1]
        ) * (third[0] - first[0])

    def on_segment(first, point, second):
        return (
            min(first[0], second[0]) <= point[0] <= max(first[0], second[0])
            and min(first[1], second[1]) <= point[1] <= max(first[1], second[1])
        )

    ab_c = cross(a, b, c)
    ab_d = cross(a, b, d)
    cd_a = cross(c, d, a)
    cd_b = cross(c, d, b)
    epsilon = 1e-12
    if (
        abs(ab_c) <= epsilon
        and on_segment(a, c, b)
        or abs(ab_d) <= epsilon
        and on_segment(a, d, b)
        or abs(cd_a) <= epsilon
        and on_segment(c, a, d)
        or abs(cd_b) <= epsilon
        and on_segment(c, b, d)
    ):
        return True
    return (ab_c < 0.0) != (ab_d < 0.0) and (cd_a < 0.0) != (cd_b < 0.0)


def _point_segment_distance(point, start, end) -> float:
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    length_sq = delta_x * delta_x + delta_y * delta_y
    if length_sq == 0.0:
        return math.dist(point, start)
    fraction = max(
        0.0,
        min(
            1.0,
            ((point[0] - start[0]) * delta_x + (point[1] - start[1]) * delta_y)
            / length_sq,
        ),
    )
    closest = (start[0] + fraction * delta_x, start[1] + fraction * delta_y)
    return math.dist(point, closest)
