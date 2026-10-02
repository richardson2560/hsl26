"""Bounded development audit; no validation/held-out use or gate acceptance."""
from pathlib import Path
import sys
import cProfile
import hashlib
import json
import platform
import pstats
import io
from collections import Counter, defaultdict
from dataclasses import asdict
from time import perf_counter

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / "hsl_core")]
from hsl_core.tactics.fsm import TacticalSelector
from sim.kinematic.autonomous import P56Profile
from sim.kinematic.benchmark import BenchmarkConfig, SILBenchmarkRunner
from sim.kinematic.maze_bank import load_maze_bank
from tools.p63_policy import build_default_baseline

DEST = Path(__file__).resolve().parent
bank = load_maze_bank(ROOT / "sim/kinematic/scenarios/phase6_maze_bank.json")
fixture = next(f for f in bank.training_fixtures() if f.scenario_id == "maze_multiring_7x7_train_a")
genome = build_default_baseline()
short_runner = SILBenchmarkRunner(BenchmarkConfig(episode_duration_s=8.0))
timings = []
for _ in range(3):
    start = perf_counter()
    episode = short_runner.run_episode(genome, fixture, seed=20260931)
    timings.append({"wall_s": perf_counter() - start, "active_s": episode.active_duration_s,
                    "steps": episode.step_count, "terminal": episode.terminal_kind.name})

counts = defaultdict(Counter)
original_select = TacticalSelector.select
def measured_select(self, snapshot, *args, **kwargs):
    result = original_select(self, snapshot, *args, **kwargs)
    role = self.utility_profile.role.name
    eligible = [a for a in result.alternatives if a.applicable and a.utility is not None]
    counts[role]["calls"] += 1
    counts[role][f"eligible_count_{len(eligible)}"] += 1
    if eligible:
        priority = min(a.priority for a in eligible)
        cohort = [a for a in eligible if a.priority == priority]
        urgency = max((a.urgency_rank or 0) for a in cohort)
        cohort = [a for a in cohort if (a.urgency_rank or 0) == urgency]
        counts[role]["utility_choice_calls"] += int(len(cohort) >= 2)
    return result

profiler = cProfile.Profile()
try:
    TacticalSelector.select = measured_select
    profiler.enable()
    diagnostic = SILBenchmarkRunner(BenchmarkConfig(episode_duration_s=60.0)).run_episode(genome, fixture, seed=20260931)
    profiler.disable()
finally:
    TacticalSelector.select = original_select
stream = io.StringIO()
pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats("cumulative").print_stats(28)
(DEST / "profile_cumulative.txt").write_text(stream.getvalue(), encoding="utf-8")

margin_result = {}
for margin in (0.06, 0.08, 0.20):
    try:
        P56Profile(clearance_margin_m=margin)
        margin_result[str(margin)] = "accepted_by_fixture_schema"
    except ValueError as error:
        margin_result[str(margin)] = str(error)

source_paths = ["sim/kinematic/autonomous.py", "sim/kinematic/benchmark.py",
                "hsl_core/hsl_core/tactics/fsm.py", "hsl_core/hsl_core/learning/evolution.py",
                "hsl_core/hsl_core/perception/registration.py", "hsl_core/hsl_core/perception/implicit_surface.py",
                "tools/train_evolution.py", "docs/Регламент HSL26 - v06092026.md"]
report = {
    "schema": "hsl26.coevolution.document-audit.v1", "date": "2026-10-02",
    "evidence_class": "development_fixture_diagnostic", "promotion_eligible": False,
    "gate_acceptance": False, "python": platform.python_version(), "host": platform.platform(),
    "scenario_id": fixture.scenario_id, "seed": 20260931, "policy_sha256": genome.sha256,
    "benchmark_configs": {"unprofiled": asdict(short_runner.config),
                          "instrumented": asdict(BenchmarkConfig(episode_duration_s=60.0))},
    "synthetic_freeze_s": 0.15,
    "unprofiled_short_runs": timings, "instrumented_selector_counts": dict(counts),
    "instrumented_terminal": diagnostic.terminal_kind.name,
    "instrumented_active_s": diagnostic.active_duration_s,
    "instrumented_collisions_by_role": {r.role.name: r.collisions for r in diagnostic.episode_results},
    "margin_schema_checks": margin_result,
    "source_sha256": {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in source_paths},
    "limitations": ["One deterministic training fixture only; no statistical claim of improvement.",
                    "Profiler and selector wrapper add overhead; profiled timing is not throughput.",
                    "No ROS, hardware, real-cloud, validation or held-out execution."],
}
(DEST / "review_checks.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
print(json.dumps(report, indent=2))
