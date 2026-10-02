"""Package A differential check against HEAD's pre-instrumentation controller.

Development training fixtures only; not package C, training, or gate evidence.
Produces a fresh report beside the historical first-step audit.
"""
from collections import Counter, defaultdict
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from time import perf_counter
import types

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / "hsl_core")]

import sim.kinematic.benchmark as benchmark
from sim.kinematic.autonomous import KinematicAutonomousPolicy
from sim.kinematic.maze_bank import load_maze_bank
from tools.p63_policy import build_default_baseline

REFERENCE_COMMIT = "8d1c6a96d1517bd953cb6d16da1dfe4d653d69e3"


def reference_policy():
    source = subprocess.check_output(
        ["git", "show", f"{REFERENCE_COMMIT}:sim/kinematic/autonomous.py"], cwd=ROOT
    )
    name = "sim.kinematic._package_a_reference"
    module = types.ModuleType(name)
    module.__file__ = str(ROOT / "sim/kinematic/autonomous.py")
    sys.modules[name] = module
    exec(compile(source, module.__file__, "exec"), module.__dict__)
    return module.KinematicAutonomousPolicy, hashlib.sha256(source).hexdigest()


def run(policy_class, fixture, baseline):
    original_call = policy_class.__call__
    original_policy = benchmark.KinematicAutonomousPolicy
    rows = []
    totals = defaultdict(Counter)
    errors = []

    def measured(self, frame):
        command = original_call(self, frame)
        trace = self.last_trace
        state = self.authority.execution_state
        rows.append({
            "role": self.role.value, "stamp_ns": frame.stamp_ns,
            "status": trace.status, "kind": trace.selected_kind,
            "reason": trace.selection_reason, "command": asdict(command),
            "authority_phase": state.phase, "authority_reason": state.reason,
            "active_instance": state.active_option_instance_id,
            "lease_generation": state.lease_generation,
            "candidate_authorized": state.candidate_authorized,
            "safety_decision": trace.safety_decision, "safety_reason": trace.safety_reason,
        })
        if hasattr(trace, "trace_schema"):
            role = self.role.value
            totals[role]["policy_calls"] += 1
            totals[role]["selector_invocations"] += int(trace.selector_invoked)
            totals[role]["active_before_selector"] += int(trace.phase == "ACTIVE" and not trace.selector_invoked)
            totals[role]["non_active_calls"] += int(trace.phase != "ACTIVE")
            if not trace.diagnostic_valid:
                errors.append({"role": role, "stamp_ns": frame.stamp_ns, "error": trace.diagnostic_error})
            else:
                for key in ("n_active", "n_available", "n_choice", "n_forced", "n_empty"):
                    totals[role][key] += getattr(trace, key)
                if trace.n_choice + trace.n_forced + trace.n_empty != trace.n_active:
                    raise ValueError("Invalid per-cycle selection partition")
                if trace.proposal_choice != len(set(signature for _, signature in trace.behavior_signatures)):
                    raise ValueError("Behavior cardinality does not match signatures")
        return command

    try:
        policy_class.__call__ = measured
        benchmark.KinematicAutonomousPolicy = policy_class
        start = perf_counter()
        episode = benchmark.SILBenchmarkRunner(
            benchmark.BenchmarkConfig(episode_duration_s=8.0)
        ).run_episode(baseline, fixture, seed=20261002)
        wall_s = perf_counter() - start
    finally:
        policy_class.__call__ = original_call
        benchmark.KinematicAutonomousPolicy = original_policy
    if errors:
        raise ValueError(f"Incomplete nominal diagnostics: {errors[:3]}")
    for counts in totals.values():
        if counts["n_choice"] + counts["n_forced"] + counts["n_empty"] != counts["n_active"]:
            raise ValueError("Invalid aggregate partition")
    outcome = {
        "terminal_kind": episode.terminal_kind.name,
        "active_duration_s": episode.active_duration_s, "step_count": episode.step_count,
        "role_results": [asdict(row) for row in episode.episode_results],
    }
    return {"rows": rows, "outcome": outcome, "wall_s": wall_s, "totals": dict(totals)}


def main():
    baseline = build_default_baseline()
    bank = load_maze_bank(ROOT / "sim/kinematic/scenarios/phase6_maze_bank.json")
    reference, reference_hash = reference_policy()
    cases = []
    for fixture in bank.training_fixtures():
        before = run(reference, fixture, baseline)
        after = run(KinematicAutonomousPolicy, fixture, baseline)
        if before["rows"] != after["rows"] or before["outcome"] != after["outcome"]:
            raise ValueError(f"Package A changed reference behavior in {fixture.scenario_id}")
        digest = hashlib.sha256(json.dumps(after["rows"], sort_keys=True, allow_nan=False).encode()).hexdigest()
        cases.append({
            "scenario_id": fixture.scenario_id, "seed": 20261002,
            "split": fixture.split, "behavior_equal": True,
            "decision_command_authority_trace_sha256": digest,
            "reference_wall_s": before["wall_s"], "instrumented_wall_s": after["wall_s"],
            "outcome": after["outcome"], "selection_totals": after["totals"],
        })
    files = (
        "sim/kinematic/autonomous.py", "sim/kinematic/selection_diagnostics.py",
        "sim/kinematic/test_selection_diagnostics.py", "sim/kinematic/test_p56_autonomous.py",
        "sim/kinematic/benchmark.py", "hsl_core/hsl_core/tactics/fsm.py",
        "hsl_core/hsl_core/tactics/options.py", "hsl_core/hsl_core/planning/astar.py",
        "sim/kinematic/scenarios/phase6_maze_bank.json",
        "artifacts/reports/coevolution_package_a_20261002/verify_runtime.py",
        "docs/HSL26_COEVOLUTION_IMPLEMENTATION_REVIEW_20261002.md",
    )
    test_path = Path(__file__).with_name("tests.txt")
    test_bytes = test_path.read_bytes()
    test_text = test_bytes.decode("utf-16" if test_bytes[:2] in (b"\xff\xfe", b"\xfe\xff") else "utf-8-sig")
    test_summaries = re.findall(r"(\d+) passed in ([0-9.]+)s", test_text)
    if not test_summaries or " failed" in test_text:
        raise ValueError("A passing regression log is required beside this verifier")
    passed, elapsed = test_summaries[-1]
    report = {
        "schema": "hsl26.coevolution.package-a-verification.v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "evidence_class": "DEVELOPMENT_ONLY", "promotion_eligible": False, "gate_acceptance": False,
        "baseline_genome_sha256": baseline.sha256,
        "reference_commit": REFERENCE_COMMIT,
        "reference_autonomous_source_sha256": reference_hash,
        "regression": {
            "command": "python -m pytest hsl_core/tests/test_p53_tactics.py hsl_core/tests/test_p63_evolution.py sim/kinematic/test_selection_diagnostics.py sim/kinematic/test_p56_autonomous.py sim/kinematic/test_p63_benchmark.py sim/kinematic/test_p63_tools.py -q -p no:cacheprovider --tb=short",
            "passed": int(passed), "elapsed_s": float(elapsed),
            "log_sha256": hashlib.sha256(test_bytes).hexdigest(),
            "note": "Existing log is validated and hashed; this script does not rerun pytest.",
        },
        "source_sha256": {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in files},
        "cases": cases,
        "limitations": [
            "Four deterministic training fixtures, 8 s horizon, one seed; no learning/generalization claim.",
            "Reference is source from the fixed reference commit; local initial trace additions changed telemetry only.",
            "Wall times include wrappers and order/warm-up effects; not a sustained overhead benchmark.",
            "No validation/held-out or G4/G5/G6 acceptance; package B dwell and C collector remain pending.",
        ],
    }
    destination = Path(__file__).with_name("runtime_verification.json")
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(destination), "equivalent_training_fixtures": len(cases),
                      "selection_totals": {r["scenario_id"]: r["selection_totals"] for r in cases}}, indent=2))


if __name__ == "__main__":
    main()
