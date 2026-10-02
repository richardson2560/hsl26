"""Run bounded, fixture-only P6.3 genome search (never a release promotion)."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import platform
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
    CandidateAssessment,
    EvaluationPlan,
    EvaluationSplit,
    EvolutionConfig,
    TacticalGenome,
    assess_paired_candidate,
    create_next_generation,
    mutate_genome,
    select_on_validation,
)
from sim.kinematic.benchmark import (  # noqa: E402
    BenchmarkConfig,
    SILBenchmarkRunner,
)
from sim.kinematic.maze_bank import load_maze_bank  # noqa: E402
from tools.p63_parallel import ordered_process_map, validate_worker_count  # noqa: E402
from tools.p63_policy import (  # noqa: E402
    EVIDENCE_CLASS,
    build_default_baseline,
    canonical_json,
    policy_payload,
)


_DEFAULT_BANK = ROOT_DIR / "sim" / "kinematic" / "scenarios" / "phase6_maze_bank.json"


def _baseline_fixture_job(job):
    bank_path, scenario_id, bench_config, genome, seed = job
    fixture = next(
        item
        for item in load_maze_bank(bank_path).fixtures
        if item.scenario_id == scenario_id
    )
    episode = SILBenchmarkRunner(bench_config).run_episode(
        genome, fixture, seed=seed
    )
    return scenario_id, episode


def _role_swapped_fixture_job(job):
    bank_path, scenario_id, bench_config, candidate, baseline, seed, baseline_ref = job
    fixture = next(
        item
        for item in load_maze_bank(bank_path).fixtures
        if item.scenario_id == scenario_id
    )
    evaluation = SILBenchmarkRunner(bench_config).run_role_swapped_evaluation(
        candidate,
        baseline,
        fixture,
        seed=seed,
        baseline_reference=baseline_ref,
    )
    return candidate.sha256, scenario_id, evaluation


def train_p63_evolution(
    *,
    generations: int = 3,
    population_size: int = 6,
    elite_count: int = 2,
    mutation_sigma: float = 0.08,
    seed: int = 2026,
    episode_duration_s: float = 8.0,
    workers: int = 1,
    bank_path: str | Path = _DEFAULT_BANK,
    output_dir: str | Path,
    development_fixtures: bool = False,
) -> dict[str, Any]:
    """Search only synthetic train fixtures; validate one training-selected finalist.

    Output policy parameters carry an immutable fixture-only/non-promotion
    label. This function does not read held-out fixtures or change phase gates.
    """
    if development_fixtures is not True:
        raise ValueError(
            "fixture search requires explicit development_fixtures=True; "
            "results are not eligible P6.3 evidence"
        )
    validate_worker_count(workers)
    started_at = perf_counter()
    config = EvolutionConfig(
        population_size=population_size,
        elite_count=elite_count,
        mutation_sigma=mutation_sigma,
        seed=seed,
        maximum_generations=generations,
    )
    if (
        isinstance(episode_duration_s, bool)
        or not isinstance(episode_duration_s, (int, float))
        or not math.isfinite(float(episode_duration_s))
        or episode_duration_s <= 0.0
    ):
        raise ValueError("episode_duration_s must be finite and positive")
    bank = load_maze_bank(bank_path)
    train_fixtures = bank.training_fixtures()
    validation_fixtures = bank.validation_fixtures()
    if len(train_fixtures) < 2 or len(validation_fixtures) < 2:
        raise ValueError("P6.3 fixture search requires at least two train and validation maps")
    destination = Path(output_dir).resolve()
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite existing run directory: {destination}")

    baseline = build_default_baseline()
    bench_config = BenchmarkConfig(
        episode_duration_s=float(episode_duration_s),
        control_period_s=0.05,
        discount_rate_per_s=0.01,
        lidar_beam_count=360,
        lidar_max_range_m=5.0,
        lidar_noise_std_m=0.0,
        robot_max_speed_mps=0.20,
    )
    runner = SILBenchmarkRunner(bench_config)
    train_plan = EvaluationPlan(
        plan_id="p63-development-train-v1",
        confidence_level=0.95,
        minimum_pairs=len(train_fixtures),
        maximum_safety_overrides_per_episode=150,
    )
    validation_plan = EvaluationPlan(
        plan_id="p63-development-validation-v1",
        confidence_level=0.95,
        minimum_pairs=len(validation_fixtures),
        maximum_safety_overrides_per_episode=150,
    )
    evo_config = config

    population = _initial_population(
        baseline,
        population_size=population_size,
        mutation_sigma=mutation_sigma,
        seed=seed,
    )

    generation_records: list[dict[str, Any]] = []
    pairing_provenance: dict[tuple[str, int], dict[str, Any]] = {}
    finalist: TacticalGenome | None = None
    finalist_training_mean = 0.0

    for generation_index in range(generations):
        evaluated: list[tuple[TacticalGenome, CandidateAssessment]] = []
        candidate_records: list[dict[str, Any]] = []
        train_seeds = {
            fixture.scenario_id: fixture.seed + generation_index * 1_000_003
            for fixture in train_fixtures
        }
        baseline_results = ordered_process_map(
            _baseline_fixture_job,
            (
                (
                    str(Path(bank_path).resolve()),
                    fixture.scenario_id,
                    bench_config,
                    baseline,
                    train_seeds[fixture.scenario_id],
                )
                for fixture in train_fixtures
            ),
            workers=workers,
        )
        baseline_references = dict(baseline_results)
        candidate_jobs = (
            (
                str(Path(bank_path).resolve()),
                fixture.scenario_id,
                bench_config,
                candidate,
                baseline,
                train_seeds[fixture.scenario_id],
                baseline_references[fixture.scenario_id],
            )
            for candidate in population
            if candidate.sha256 != baseline.sha256
            for fixture in train_fixtures
        )
        role_evaluations = ordered_process_map(
            _role_swapped_fixture_job,
            candidate_jobs,
            workers=workers,
        )
        evaluations_by_candidate: dict[str, dict[str, Any]] = {}
        for candidate_id, scenario_id, role_pair in role_evaluations:
            evaluations_by_candidate.setdefault(candidate_id, {})[
                scenario_id
            ] = role_pair
        for candidate in population:
            if candidate.sha256 == baseline.sha256:
                assessment = _baseline_assessment(baseline, len(train_fixtures))
            else:
                baseline_rows = []
                candidate_rows = []
                for fixture in train_fixtures:
                    role_pair = evaluations_by_candidate[candidate.sha256][
                        fixture.scenario_id
                    ]
                    _record_pair_provenance(
                        pairing_provenance,
                        fixture,
                        role_pair.candidate_results[0],
                    )
                    baseline_rows.extend(role_pair.baseline_results)
                    candidate_rows.extend(role_pair.candidate_results)
                assessment = assess_paired_candidate(
                    baseline_rows,
                    candidate_rows,
                    plan=train_plan,
                    split=EvaluationSplit.TRAINING,
                )
                if (
                    assessment.eligible
                    and assessment.balanced_mean_delta is not None
                    and assessment.balanced_mean_delta > finalist_training_mean
                    and assessment.guardian_mean_delta is not None
                    and assessment.guardian_mean_delta >= 0.0
                    and assessment.explorer_mean_delta is not None
                    and assessment.explorer_mean_delta >= 0.0
                ):
                    finalist = candidate
                    finalist_training_mean = assessment.balanced_mean_delta
            evaluated.append((candidate, assessment))
            candidate_records.append(_assessment_payload(candidate, assessment))

        generation_records.append(
            {
                "generation_index": generation_index,
                "candidate_assessments": candidate_records,
            }
        )
        if generation_index + 1 < generations:
            next_generation = create_next_generation(
                baseline_genome=baseline,
                evaluated_genomes=evaluated,
                config=evo_config,
                generation_index=generation_index,
            )
            population = list(next_generation.genomes)
            generation_records[-1]["elite_sha256"] = list(
                next_generation.elite_sha256
            )
            generation_records[-1]["excluded_candidate_reasons"] = [
                {"genome_sha256": genome_id, "reason": reason}
                for genome_id, reason in next_generation.excluded_candidate_reasons
            ]

    validation_assessment = None
    selected = baseline
    selection_reason = "baseline_retained_no_training_supported_candidate"
    if finalist is not None:
        base_rows = []
        candidate_rows = []
        validation_seeds = {
            fixture.scenario_id: fixture.seed + 9_000_001
            for fixture in validation_fixtures
        }
        baseline_references = dict(
            ordered_process_map(
                _baseline_fixture_job,
                (
                    (
                        str(Path(bank_path).resolve()),
                        fixture.scenario_id,
                        bench_config,
                        baseline,
                        validation_seeds[fixture.scenario_id],
                    )
                    for fixture in validation_fixtures
                ),
                workers=workers,
            )
        )
        validation_pairs = ordered_process_map(
            _role_swapped_fixture_job,
            (
                (
                    str(Path(bank_path).resolve()),
                    fixture.scenario_id,
                    bench_config,
                    finalist,
                    baseline,
                    validation_seeds[fixture.scenario_id],
                    baseline_references[fixture.scenario_id],
                )
                for fixture in validation_fixtures
            ),
            workers=workers,
        )
        validation_by_scenario = {
            scenario_id: role_pair
            for _candidate_id, scenario_id, role_pair in validation_pairs
        }
        for fixture in validation_fixtures:
            role_pair = validation_by_scenario[fixture.scenario_id]
            _record_pair_provenance(
                pairing_provenance,
                fixture,
                role_pair.candidate_results[0],
            )
            base_rows.extend(role_pair.baseline_results)
            candidate_rows.extend(role_pair.candidate_results)
        validation_assessment = assess_paired_candidate(
            base_rows,
            candidate_rows,
            plan=validation_plan,
            split=EvaluationSplit.VALIDATION,
        )
        decision = select_on_validation(baseline.sha256, validation_assessment)
        selection_reason = decision.reason
        if decision.candidate_selected:
            selected = finalist

    destination.mkdir(parents=True, exist_ok=False)
    run_id = destination.name
    policy = policy_payload(
        selected,
        selection_reason=selection_reason,
        baseline_sha256=baseline.sha256,
        run_id=run_id,
    )
    report = {
        "schema": "hsl26.p63-development-run.v1",
        "run_id": run_id,
        "traceability": {
            "plan_document": "docs/HSL26_COEVOLUTION_PLAN.md",
            "plan_revision": "2.1",
            "step": "signal_diagnostics_and_registration",
            "phase": "P6.3",
            "generated_at_utc": __import__("datetime").datetime.now(__import__("datetime").timezone.utc).isoformat(),
            "source_file_sha256": _source_hashes(),
            "maze_bank_sha256": hashlib.sha256(
                Path(bank_path).read_bytes()
            ).hexdigest(),
            "selected_policy_sha256": selected.sha256,
        },
        "invocation": sys.argv if __name__ == "__main__" else "programmatic API",
        "python_version": platform.python_version(),
        "evidence_class": EVIDENCE_CLASS,
        "fixture_search_performed": True,
        "eligible_empirical_training": False,
        "promotion_eligible": False,
        "official_score_available": False,
        "gates": {"G4": "BLOCKED_NOT_RUN", "G5": "NOT_RUN", "G6": "BLOCKED"},
        "seed": seed,
        "evolution": {
            "generations": generations,
            "population_size": population_size,
            "elite_count": elite_count,
            "mutation_sigma": mutation_sigma,
            "workers": workers,
        },
        "wall_clock_s": perf_counter() - started_at,
        "benchmark_config": asdict(bench_config),
        "evaluation_plans": {
            "training": asdict(train_plan),
            "validation": asdict(validation_plan),
        },
        "maze_bank_sha256": hashlib.sha256(
            Path(bank_path).read_bytes()
        ).hexdigest(),
        "source_file_sha256": _source_hashes(),
        "training_scenarios": [
            _fixture_reference(fixture) for fixture in train_fixtures
        ],
        "validation_scenarios": [
            _fixture_reference(fixture) for fixture in validation_fixtures
        ],
        "held_out_episodes_evaluated": 0,
        "pairing_provenance": [
            pairing_provenance[key] for key in sorted(pairing_provenance)
        ],
        "baseline_sha256": baseline.sha256,
        "training_finalist_sha256": finalist.sha256 if finalist else None,
        "training_finalist_mean_delta": (
            finalist_training_mean if finalist is not None else None
        ),
        "validation_assessment": (
            _assessment_to_json(validation_assessment)
            if validation_assessment is not None
            else None
        ),
        "selected_policy_sha256": selected.sha256,
        "selection_reason": selection_reason,
        "generation_records": generation_records,
    }
    (destination / "run_report.json").write_text(
        canonical_json(report), encoding="utf-8"
    )
    (destination / "policy.json").write_text(canonical_json(policy), encoding="utf-8")
    return report


def _baseline_assessment(
    baseline: TacticalGenome, pair_count: int
) -> CandidateAssessment:
    return CandidateAssessment(
        candidate_id=baseline.sha256,
        split=EvaluationSplit.TRAINING,
        eligible=True,
        reason="baseline_reference",
        pair_count=pair_count,
        guardian_mean_delta=0.0,
        explorer_mean_delta=0.0,
        balanced_mean_delta=0.0,
        confidence_interval=(0.0, 0.0),
    )


def _initial_population(
    baseline: TacticalGenome,
    *,
    population_size: int,
    mutation_sigma: float,
    seed: int,
) -> list[TacticalGenome]:
    population = [baseline]
    seen = {baseline.sha256}
    attempt = 1
    maximum_attempts = max(100, population_size * 100)
    while len(population) < population_size:
        if attempt > maximum_attempts:
            raise RuntimeError(
                "bounded mutation could not initialize a unique genome population"
            )
        child = mutate_genome(
            baseline,
            sigma=mutation_sigma,
            seed=seed + attempt,
        )
        attempt += 1
        if child.sha256 not in seen:
            seen.add(child.sha256)
            population.append(child)
    return population


def _assessment_payload(
    genome: TacticalGenome, assessment: CandidateAssessment
) -> dict[str, Any]:
    return {
        "genome_sha256": genome.sha256,
        **_assessment_to_json(assessment),
    }


def _assessment_to_json(assessment: CandidateAssessment) -> dict[str, Any]:
    return {
        "split": assessment.split.value,
        "eligible": assessment.eligible,
        "reason": assessment.reason,
        "pair_count": assessment.pair_count,
        "guardian_mean_delta": assessment.guardian_mean_delta,
        "explorer_mean_delta": assessment.explorer_mean_delta,
        "balanced_mean_delta": assessment.balanced_mean_delta,
        "confidence_interval": assessment.confidence_interval,
        "official_score_available": assessment.official_score_available,
    }


def _record_pair_provenance(records, fixture, result) -> None:
    records.setdefault(
        (fixture.scenario_id, result.seed),
        {
            "scenario_id": fixture.scenario_id,
            "scenario_sha256": _fixture_reference(fixture)["scenario_sha256"],
            "seed": result.seed,
            "split": result.split.value,
            "map_bank_id": result.map_bank_id,
            "opponent_bank_id": result.opponent_bank_id,
            "seed_bank_id": result.seed_bank_id,
            "evaluation_profile_id": result.evaluation_profile_id,
        },
    )


def _fixture_reference(fixture) -> dict[str, Any]:
    rows = tuple("".join(row) for row in fixture.rows)
    payload = {
        "scenario_id": fixture.scenario_id,
        "split": fixture.split,
        "map_bank_id": fixture.map_bank_id,
        "opponent_bank_id": fixture.opponent_bank_id,
        "seed_bank_id": fixture.seed_bank_id,
        "rows": rows,
        "guardian_start_rc": fixture.guardian_start_rc,
        "explorer_start_rc": fixture.explorer_start_rc,
        "synthetic_goal_rc": fixture.synthetic_goal_rc,
        "cell_size_m": fixture.cell_size_m,
        "wall_segments_m": tuple(
            (segment.start_xy, segment.end_xy)
            for segment in fixture.geometry.static_segments
        ),
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    payload["scenario_sha256"] = hashlib.sha256(encoded).hexdigest()
    return payload


def _source_hashes() -> dict[str, str]:
    paths = (
        ROOT_DIR / "tools" / "train_evolution.py",
        ROOT_DIR / "tools" / "p63_policy.py",
        ROOT_DIR / "sim" / "kinematic" / "benchmark.py",
        ROOT_DIR / "sim" / "kinematic" / "autonomous.py",
        ROOT_DIR / "sim" / "kinematic" / "maze_bank.py",
        ROOT_DIR / "sim" / "kinematic" / "scenarios" / "phase6_maze_bank.json",
        ROOT_DIR / "hsl_core" / "hsl_core" / "learning" / "evolution.py",
    )
    return {
        str(path.relative_to(ROOT_DIR)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in paths
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--development-fixtures",
        action="store_true",
        help="required acknowledgement that results are synthetic and non-promotable",
    )
    parser.add_argument("--generations", type=int, default=3)
    parser.add_argument("--population-size", type=int, default=6)
    parser.add_argument("--elite-count", type=int, default=2)
    parser.add_argument("--mutation-sigma", type=float, default=0.08)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--episode-duration-s", type=float, default=8.0)
    parser.add_argument("--bank", type=Path, default=_DEFAULT_BANK)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT_DIR / "artifacts" / "reports" / "phase6" / "p63_development_run",
    )
    args = parser.parse_args()
    if not args.development_fixtures:
        parser.error(
            "training is fixture-only; pass --development-fixtures to acknowledge "
            "the run is not empirical evidence or policy promotion"
        )
    return args


if __name__ == "__main__":
    cli_args = _parse_args()
    result = train_p63_evolution(
        generations=cli_args.generations,
        population_size=cli_args.population_size,
        elite_count=cli_args.elite_count,
        mutation_sigma=cli_args.mutation_sigma,
        seed=cli_args.seed,
        workers=cli_args.workers,
        episode_duration_s=cli_args.episode_duration_s,
        bank_path=cli_args.bank,
        output_dir=cli_args.output_dir,
        development_fixtures=True,
    )
    print(
        f"Fixture search complete: selected {result['selected_policy_sha256']}; "
        f"promotion_eligible={result['promotion_eligible']}; "
        f"report={cli_args.output_dir / 'run_report.json'}"
    )
