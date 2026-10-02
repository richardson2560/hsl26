"""Paired fixture benchmarks for a P6.3 policy versus the fixed baseline."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
from pathlib import Path
import sys
from time import perf_counter
from typing import Any

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))
CORE_DIR = ROOT_DIR / "hsl_core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from hsl_core.learning.evolution import (  # noqa: E402
    EpisodeResult,
    EvaluationPlan,
    EvaluationSplit,
    TacticalGenome,
    assess_paired_candidate,
)
from hsl_core.match import Role  # noqa: E402
from sim.kinematic.benchmark import BenchmarkConfig, SILBenchmarkRunner  # noqa: E402
from sim.kinematic.maze_bank import load_maze_bank  # noqa: E402
from tools.p63_parallel import ordered_process_map, validate_worker_count  # noqa: E402
from tools.p63_policy import (  # noqa: E402
    EVIDENCE_CLASS,
    build_default_baseline,
    canonical_json,
    load_development_policy,
)


_DEFAULT_BANK = (
    ROOT_DIR / "sim" / "kinematic" / "scenarios"
    / "phase6_interior_tactics_bank.json"
)


def _episode_summary(episode) -> dict[str, Any]:
    return {
        "terminal_kind": episode.terminal_kind.name,
        "active_duration_s": episode.active_duration_s,
        "step_count": episode.step_count,
        "guardian_wall_contacts": episode.guardian_wall_contacts,
        "explorer_wall_contacts": episode.explorer_wall_contacts,
        "roles": {
            row.role.name: {
                "return": row.training_return,
                "safety_overrides": row.safety_overrides,
                "safety_violations": row.safety_violations,
                "collisions": row.collisions,
                "completed": row.completed,
                "policy_sha256": row.policy_id,
                "evaluation_profile_id": row.evaluation_profile_id,
            }
            for row in episode.episode_results
        },
        "evidence_class": episode.evidence_class,
        "promotion_eligible": False,
    }


def _paired_job(job):
    bank_path, scenario_id, config, candidate, baseline, seed = job
    fixture = next(
        item
        for item in load_maze_bank(bank_path).fixtures
        if item.scenario_id == scenario_id
    )
    runner = SILBenchmarkRunner(config)
    base = runner.run_episode(baseline, fixture, seed=seed)
    candidate_guardian = runner.run_episode(
        candidate,
        fixture,
        seed=seed,
        explorer_genome=baseline,
    )
    candidate_explorer = runner.run_episode(
        baseline,
        fixture,
        seed=seed,
        explorer_genome=candidate,
    )
    candidate_rows = (
        next(row for row in candidate_guardian.episode_results if row.role is Role.GUARDIAN),
        next(row for row in candidate_explorer.episode_results if row.role is Role.EXPLORER),
    )
    baseline_rows = tuple(base.episode_results)
    return {
        "scenario_id": scenario_id,
        "split": fixture.split,
        "seed": seed,
        "scenario_sha256": base.scenario_sha256,
        "baseline": _episode_summary(base),
        "candidate_guardian_vs_baseline_explorer": _episode_summary(
            candidate_guardian
        ),
        "baseline_guardian_vs_candidate_explorer": _episode_summary(
            candidate_explorer
        ),
        "candidate_results": candidate_rows,
        "baseline_results": baseline_rows,
        "pair_key": candidate_rows[0].pair_key,
    }


def _serialize_assessment(assessment) -> dict[str, Any]:
    return {
        "candidate_id": assessment.candidate_id,
        "split": assessment.split.value,
        "eligible": assessment.eligible,
        "reason": assessment.reason,
        "pair_count": assessment.pair_count,
        "guardian_mean_delta": assessment.guardian_mean_delta,
        "explorer_mean_delta": assessment.explorer_mean_delta,
        "balanced_mean_delta": assessment.balanced_mean_delta,
        "confidence_interval": assessment.confidence_interval,
        "official_score_available": False,
        "promotion_eligible": False,
    }


def run_p63_benchmark(
    *,
    policy_path: str | Path,
    output_path: str | Path,
    bank_path: str | Path = _DEFAULT_BANK,
    seed: int = 20260930,
    replicates: int = 1,
    episode_duration_s: float = 60.0,
    workers: int = 1,
) -> dict[str, Any]:
    validate_worker_count(workers)
    if (
        not isinstance(replicates, int)
        or isinstance(replicates, bool)
        or replicates < 1
    ):
        raise ValueError("replicates must be a positive integer")
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ValueError("seed must be a non-negative integer")
    destination = Path(output_path).resolve()
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite benchmark output: {destination}")
    policy, payload = load_development_policy(policy_path)
    baseline = build_default_baseline()
    bank = load_maze_bank(bank_path)
    fixtures = (*bank.training_fixtures(), *bank.validation_fixtures())
    if len(fixtures) < 2:
        raise ValueError("benchmark requires at least two non-held-out fixtures")
    config = BenchmarkConfig(
        episode_duration_s=episode_duration_s,
        control_period_s=0.05,
        discount_rate_per_s=0.01,
        lidar_beam_count=360,
        lidar_max_range_m=6.0,
        lidar_noise_std_m=0.0,
        robot_radius_m=0.15,
        robot_max_speed_mps=0.20,
    )
    started_at = perf_counter()
    jobs = (
        (
            str(Path(bank_path).resolve()),
            fixture.scenario_id,
            config,
            policy,
            baseline,
            seed + fixture.seed + replicate * 1_000_003,
        )
        for fixture in fixtures
        for replicate in range(replicates)
    )
    evaluations = ordered_process_map(_paired_job, jobs, workers=workers)
    candidate_rows: dict[EvaluationSplit, list[EpisodeResult]] = {
        EvaluationSplit.TRAINING: [],
        EvaluationSplit.VALIDATION: [],
    }
    baseline_rows: dict[EvaluationSplit, list[EpisodeResult]] = {
        EvaluationSplit.TRAINING: [],
        EvaluationSplit.VALIDATION: [],
    }
    details = []
    for evaluation in evaluations:
        split = EvaluationSplit(evaluation["split"])
        candidate_rows[split].extend(evaluation.pop("candidate_results"))
        baseline_rows[split].extend(evaluation.pop("baseline_results"))
        evaluation.pop("pair_key")
        details.append(evaluation)

    assessments = {}
    if policy.sha256 != baseline.sha256:
        for split in (EvaluationSplit.TRAINING, EvaluationSplit.VALIDATION):
            pair_count = len(candidate_rows[split]) // 2
            if pair_count < 2:
                continue
            plan = EvaluationPlan(
                plan_id=f"p63-diagnostic-{split.value}-v1",
                confidence_level=0.95,
                minimum_pairs=pair_count,
                maximum_safety_overrides_per_episode=150,
            )
            assessments[split.value] = _serialize_assessment(
                assess_paired_candidate(
                    baseline_rows[split],
                    candidate_rows[split],
                    plan=plan,
                    split=split,
                )
            )
    elif policy.sha256 == baseline.sha256:
        assessments["selection"] = {
            "eligible": False,
            "reason": "selected_policy_is_baseline_no_candidate_comparison",
            "candidate_id": policy.sha256,
            "promotion_eligible": False,
        }

    report = {
        "schema": "hsl26.p63-paired-benchmark.v1",
        "evidence_class": EVIDENCE_CLASS,
        "physical_authority": False,
        "official_score_available": False,
        "promotion_eligible": False,
        "held_out_fixtures_evaluated": 0,
        "run_id": payload["run_id"],
        "policy_sha256": policy.sha256,
        "baseline_sha256": baseline.sha256,
        "bank_id": bank.bank_id,
        "bank_sha256": hashlib.sha256(Path(bank_path).read_bytes()).hexdigest(),
        "benchmark_config": asdict(config),
        "seed": seed,
        "replicates_per_fixture": replicates,
        "workers": workers,
        "wall_clock_s": perf_counter() - started_at,
        "assessments": assessments,
        "episodes": details,
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(canonical_json(report), encoding="utf-8")
    return report


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--bank", type=Path, default=_DEFAULT_BANK)
    parser.add_argument("--seed", type=int, default=20260930)
    parser.add_argument("--replicates", type=int, default=1)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="new JSON report path; existing files are never overwritten",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    result = run_p63_benchmark(
        policy_path=args.policy,
        output_path=args.output,
        bank_path=args.bank,
        seed=args.seed,
        replicates=args.replicates,
        episode_duration_s=args.duration_s,
        workers=args.workers,
    )
    print(
        f"P6.3 paired fixture benchmark complete: {len(result['episodes'])} "
        f"episodes; promotion_eligible={result['promotion_eligible']}; "
        f"report={args.output}"
    )
