"""End-to-end contracts for fixture-only P6.3 training and match tools."""

import json
from pathlib import Path

import pytest

from hsl_core.learning.evolution import (
    EvaluationPlan,
    EvaluationSplit,
    assess_paired_candidate,
)
from hsl_core.match import Role
from hsl_core.learning.evolution import mutate_genome
from sim.kinematic.benchmark import BenchmarkConfig, SILBenchmarkRunner
from sim.kinematic.maze_bank import load_maze_bank
from tools.benchmark_p63 import run_p63_benchmark
from tools.demo_competition_match import run_ideal_competition_match
from tools.demo_p63_match import run_p63_match_viewer
from tools.p63_policy import (
    EVIDENCE_CLASS,
    build_default_baseline,
    load_development_policy,
    policy_payload,
)
from tools.train_evolution import train_p63_evolution


_BANK = "sim/kinematic/scenarios/phase6_maze_bank.json"
_INTERIOR_BANK = "sim/kinematic/scenarios/phase6_interior_tactics_bank.json"


def test_role_swapped_assessment_pairs_candidate_roles_against_fixed_baseline():
    baseline = build_default_baseline()
    candidate = mutate_genome(baseline, sigma=0.08, seed=471)
    fixture = load_maze_bank(_BANK).training_fixtures()[0]
    runner = SILBenchmarkRunner(
        BenchmarkConfig(episode_duration_s=0.5, max_episode_steps=11)
    )
    baseline_reference = runner.run_episode(baseline, fixture, seed=31)

    comparison = runner.run_role_swapped_evaluation(
        candidate,
        baseline,
        fixture,
        seed=31,
        baseline_reference=baseline_reference,
    )

    candidate_rows = {row.role: row for row in comparison.candidate_results}
    baseline_rows = {row.role: row for row in comparison.baseline_results}
    assert set(candidate_rows) == {Role.GUARDIAN, Role.EXPLORER}
    assert all(row.policy_id == candidate.sha256 for row in candidate_rows.values())
    assert all(row.policy_id == baseline.sha256 for row in baseline_rows.values())
    assert all(row.split is EvaluationSplit.TRAINING for row in candidate_rows.values())
    assert {
        role: candidate_rows[role].pair_key for role in candidate_rows
    } == {role: baseline_rows[role].pair_key for role in baseline_rows}
    assert comparison.evidence_class == EVIDENCE_CLASS
    bad_reference = runner.run_episode(candidate, fixture, seed=31)
    with pytest.raises(ValueError, match="baseline reference does not match"):
        runner.run_role_swapped_evaluation(
            candidate,
            baseline,
            fixture,
            seed=31,
            baseline_reference=bad_reference,
        )


def test_zero_sum_self_play_is_not_used_as_balanced_genome_fitness():
    baseline = build_default_baseline()
    candidate = mutate_genome(baseline, sigma=0.08, seed=472)
    fixtures = load_maze_bank(_BANK).training_fixtures()[:2]
    runner = SILBenchmarkRunner(
        BenchmarkConfig(episode_duration_s=0.5, max_episode_steps=11)
    )
    baseline_rows = []
    candidate_rows = []
    for fixture in fixtures:
        pair = runner.run_paired_episode(
            candidate, baseline, fixture, seed=fixture.seed
        )
        baseline_rows.extend(pair.baseline.episode_results)
        candidate_rows.extend(pair.candidate.episode_results)
    self_play_assessment = assess_paired_candidate(
        baseline_rows,
        candidate_rows,
        plan=EvaluationPlan("self-play-diagnostic", 0.95, 2, 150),
        split=EvaluationSplit.TRAINING,
    )

    assert self_play_assessment.eligible
    assert self_play_assessment.balanced_mean_delta == pytest.approx(0.0)
    for episode in (pair.candidate, pair.baseline):
        assert sum(
            result.training_return for result in episode.episode_results
        ) == pytest.approx(0.0)


def test_role_swapped_evaluator_rejects_self_comparison_and_heldout():
    baseline = build_default_baseline()
    fixture = load_maze_bank(_BANK).training_fixtures()[0]
    held_out = load_maze_bank(_BANK).held_out_fixtures()[0]
    runner = SILBenchmarkRunner(
        BenchmarkConfig(episode_duration_s=0.5, max_episode_steps=11)
    )
    with pytest.raises(ValueError, match="must differ"):
        runner.run_role_swapped_evaluation(
            baseline, baseline, fixture, seed=8
        )
    candidate = mutate_genome(baseline, sigma=0.05, seed=94)
    with pytest.raises(ValueError, match="held-out fixtures are reserved"):
        runner.run_role_swapped_evaluation(
            candidate, baseline, held_out, seed=8
        )


def test_development_policy_loader_checks_boundary_and_genome_digest(tmp_path):
    genome = build_default_baseline()
    payload = policy_payload(
        genome,
        selection_reason="fixture-test",
        baseline_sha256=genome.sha256,
        run_id="unit-test",
    )
    path = tmp_path / "policy.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    loaded, _ = load_development_policy(path)
    assert loaded == genome

    payload["promotion_eligible"] = True
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="fixture-only boundary"):
        load_development_policy(path)

    payload["promotion_eligible"] = False
    payload["genome_sha256"] = "0" * 64
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="digest does not match"):
        load_development_policy(path)


def test_one_generation_smoke_search_writes_nonpromotable_report_and_policy(tmp_path):
    output_dir = tmp_path / "p63-run"
    with pytest.raises(ValueError, match="explicit development_fixtures=True"):
        train_p63_evolution(
            generations=1,
            population_size=2,
            elite_count=1,
            episode_duration_s=0.5,
            bank_path=_BANK,
            output_dir=output_dir,
        )
    report = train_p63_evolution(
        generations=2,
        population_size=3,
        elite_count=1,
        mutation_sigma=0.04,
        seed=20260929,
        workers=2,
        episode_duration_s=0.5,
        bank_path=_BANK,
        output_dir=output_dir,
        development_fixtures=True,
    )

    assert report["fixture_search_performed"] is True
    assert report["eligible_empirical_training"] is False
    assert report["promotion_eligible"] is False
    assert report["official_score_available"] is False
    assert report["gates"] == {
        "G4": "BLOCKED_NOT_RUN",
        "G5": "NOT_RUN",
        "G6": "BLOCKED",
    }
    assert report["held_out_episodes_evaluated"] == 0
    assert report["evolution"]["workers"] == 2
    assert len(report["generation_records"]) == 2
    assert all(
        len(generation["candidate_assessments"]) == 3
        for generation in report["generation_records"]
    )
    assert len(report["generation_records"][0]["elite_sha256"]) <= 1
    assert (output_dir / "run_report.json").is_file()
    policy, payload = load_development_policy(output_dir / "policy.json")
    assert policy.sha256 == report["selected_policy_sha256"]
    assert payload["evidence_class"] == EVIDENCE_CLASS
    with pytest.raises(FileExistsError):
        train_p63_evolution(
            generations=1,
            population_size=2,
            elite_count=1,
            episode_duration_s=0.5,
            bank_path=_BANK,
            output_dir=output_dir,
            development_fixtures=True,
        )


def test_serial_and_parallel_p63_searches_have_identical_selection_and_scores(
    tmp_path,
):
    common = {
        "generations": 1,
        "population_size": 2,
        "elite_count": 1,
        "mutation_sigma": 0.08,
        "seed": 44551,
        "episode_duration_s": 0.5,
        "bank_path": _INTERIOR_BANK,
        "development_fixtures": True,
    }
    serial = train_p63_evolution(
        **common,
        workers=1,
        output_dir=tmp_path / "serial",
    )
    parallel = train_p63_evolution(
        **common,
        workers=2,
        output_dir=tmp_path / "parallel",
    )

    assert serial["selected_policy_sha256"] == parallel["selected_policy_sha256"]
    assert serial["selection_reason"] == parallel["selection_reason"]
    assert serial["generation_records"] == parallel["generation_records"]
    assert serial["pairing_provenance"] == parallel["pairing_provenance"]
    assert serial["held_out_episodes_evaluated"] == 0
    assert parallel["held_out_episodes_evaluated"] == 0


def test_parallel_paired_benchmark_excludes_heldout_and_emits_metrics(tmp_path):
    genome = build_default_baseline()
    policy_file = tmp_path / "policy.json"
    policy_file.write_text(
        json.dumps(
            policy_payload(
                genome,
                selection_reason="fixture-benchmark",
                baseline_sha256=genome.sha256,
                run_id="benchmark-test",
            )
        ),
        encoding="utf-8",
    )
    output = tmp_path / "benchmark.json"
    report = run_p63_benchmark(
        policy_path=policy_file,
        output_path=output,
        bank_path=_INTERIOR_BANK,
        seed=763,
        replicates=1,
        episode_duration_s=0.5,
        workers=2,
    )

    assert len(report["episodes"]) == 4
    assert report["held_out_fixtures_evaluated"] == 0
    assert report["promotion_eligible"] is False
    assert report["assessments"]["selection"]["reason"] == (
        "selected_policy_is_baseline_no_candidate_comparison"
    )
    assert all(
        "terminal_kind" in episode["baseline"]
        and "safety_violations" in episode["baseline"]["roles"]["GUARDIAN"]
        for episode in report["episodes"]
    )
    assert json.loads(output.read_text(encoding="utf-8"))["episodes"]


def test_interactive_match_viewer_uses_selected_policy_and_labels_synthetic_run(
    tmp_path,
):
    genome = build_default_baseline()
    policy_file = tmp_path / "policy.json"
    policy_file.write_text(
        json.dumps(
            policy_payload(
                genome,
                selection_reason="fixture-viewer",
                baseline_sha256=genome.sha256,
                run_id="viewer-test",
            )
        ),
        encoding="utf-8",
    )
    result = run_p63_match_viewer(
        policy_path=policy_file,
        output_dir=tmp_path / "viewer",
        bank_path=_INTERIOR_BANK,
        scenario_id="maze_interior_loops_train_c",
        seed=187,
        duration_s=1.0,
        frame_stride=2,
    )
    html = Path(result["viewer_path"]).read_text(encoding="utf-8")

    assert result["policy_sha256"] == genome.sha256
    assert result["promotion_eligible"] is False
    assert result["rendered_frame_stride"] == 2
    assert 'id="top"' in html and 'id="doom"' in html
    assert "SYNTHETIC SIL VISUALIZATION" in html
    assert "maze_interior_loops_train_c" in html
    assert "const top=" not in html and "const frames=" not in html
    assert "const topCanvas=" in html and "const frameList=scene.frames" in html
    assert 'document.getElementById("scenario").textContent=scene.scenario_id' in html
    assert "show();" in html


def test_demo_runs_learned_policy_and_exports_truth_scoped_telemetry(tmp_path):
    baseline = build_default_baseline()
    policy_file = tmp_path / "policy.json"
    policy_file.write_text(
        json.dumps(
            policy_payload(
                baseline,
                selection_reason="fixture-demo",
                baseline_sha256=baseline.sha256,
                run_id="fixture-demo",
            )
        ),
        encoding="utf-8",
    )
    output_dir = tmp_path / "demo"

    result = run_ideal_competition_match(
        scenario_id="maze_multiring_7x7_train_a",
        max_episode_duration_s=0.5,
        seed=913,
        bank_path=_BANK,
        policy_path=policy_file,
        output_dir=output_dir,
    )

    assert result["policy_sha256"] == baseline.sha256
    assert result["evidence_class"] == EVIDENCE_CLASS
    assert result["promotion_eligible"] is False
    assert result["official_score_available"] is False
    assert result["sensor_profile"]["hardware_or_sensor_validation"] is False
    assert result["telemetry_samples"] > 0
    assert (output_dir / "match.svg").read_text(encoding="utf-8").startswith(
        "<svg "
    )
    written = json.loads((output_dir / "match_report.json").read_text(encoding="utf-8"))
    assert len(written["trajectory"]) == result["telemetry_samples"]
    with pytest.raises(FileExistsError):
        run_ideal_competition_match(
            max_episode_duration_s=0.5,
            bank_path=_BANK,
            policy_path=policy_file,
            output_dir=output_dir,
        )
