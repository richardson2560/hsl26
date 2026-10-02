"""B verification: fixed A snapshot, zero-dwell equivalence and real start ages.

Development training fixtures only. No evolution, validation or held-out data.
"""
from collections import Counter, defaultdict
from dataclasses import asdict, replace
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

import pytest
import sim.kinematic.benchmark as benchmark
from hsl_core.match import Role
from sim.kinematic.autonomous import KinematicAutonomousPolicy
from sim.kinematic.maze_bank import load_maze_bank
from sim.kinematic.test_active_option_lifecycle import _fixture, _frame, START_NS, DWELL_NS
from tools.p63_policy import build_default_baseline

REFERENCE_SHA256 = "0d7b1a748de5839b5c36aa32597bfbc5d5afabc6f6a0b89a07024c5e0d9e37ab"
REGRESSION_COMMAND = (
    "python -m pytest hsl_core/tests/test_p53_tactics.py hsl_core/tests/test_p63_evolution.py "
    "hsl_core/tests/test_p52_option_authority.py hsl_core/tests/test_p35_planning_control.py "
    "hsl_core/tests/test_safety.py hsl_core/tests/test_p51_match.py "
    "sim/kinematic/test_selection_diagnostics.py sim/kinematic/test_active_option_lifecycle.py "
    "sim/kinematic/test_p56_autonomous.py sim/kinematic/test_p63_benchmark.py "
    "sim/kinematic/test_p63_tools.py -q -p no:cacheprovider --tb=short"
)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reference_policy():
    path = Path(__file__).with_name("reference_a_autonomous.py")
    if sha256(path) != REFERENCE_SHA256:
        raise ValueError("The frozen pre-B package A source has changed")
    name = "sim.kinematic._package_b_reference"
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
    return module.KinematicAutonomousPolicy


def run(policy_class, fixture, genome, *, verify_start=False):
    original_call = policy_class.__call__
    original_policy = benchmark.KinematicAutonomousPolicy
    rows, ages = [], []
    totals = defaultdict(Counter)
    starts = {}

    def measured(self, frame):
        original_select = self.selector.select

        def observed(snapshot, **kwargs):
            if verify_start and self._active is not None:
                active = self._active
                if (kwargs["current_started_at_ns"] != active.started_at_ns
                        or kwargs["current_option_instance_id"] != active.option_instance_id
                        or kwargs["current_stable_key"] != active.stable_key
                        or active.lease_generation != self.authority.execution_state.lease_generation):
                    raise ValueError("Caller/active record/authority disagree")
                age = snapshot.context.now_ns - active.started_at_ns
                key = (self.role.value, active.option_instance_id)
                if age < 0 or starts.setdefault(key, active.started_at_ns) != active.started_at_ns:
                    raise ValueError("Start reset or cross-domain/negative age")
                ages.append({"role": self.role.value, "instance_id": active.option_instance_id,
                             "started_at_ns": active.started_at_ns, "stamp_ns": frame.stamp_ns,
                             "age_ns": age})
            return original_select(snapshot, **kwargs)

        try:
            self.selector.select = observed
            command = original_call(self, frame)
        finally:
            self.selector.select = original_select
        trace = self.last_trace
        state = self.authority.execution_state
        rows.append({
            "role": self.role.value, "stamp_ns": frame.stamp_ns, "status": trace.status,
            "kind": trace.selected_kind, "reason": trace.selection_reason, "command": asdict(command),
            "authority_phase": state.phase, "authority_reason": state.reason,
            "active_instance": state.active_option_instance_id, "lease_generation": state.lease_generation,
            "candidate_authorized": state.candidate_authorized,
            "safety_decision": trace.safety_decision, "safety_reason": trace.safety_reason,
        })
        if not trace.diagnostic_valid:
            raise ValueError(f"Incomplete nominal diagnostic: {trace.diagnostic_error}")
        if trace.n_choice + trace.n_forced + trace.n_empty != trace.n_active:
            raise ValueError("Invalid per-cycle selection partition")
        totals[self.role.value]["policy_calls"] += 1
        for key in ("n_active", "n_available", "n_choice", "n_forced", "n_empty"):
            totals[self.role.value][key] += getattr(trace, key)
        return command

    try:
        policy_class.__call__ = measured
        benchmark.KinematicAutonomousPolicy = policy_class
        start = perf_counter()
        episode = benchmark.SILBenchmarkRunner(
            benchmark.BenchmarkConfig(episode_duration_s=8.0)
        ).run_episode(genome, fixture, seed=20261002)
        wall_s = perf_counter() - start
    finally:
        policy_class.__call__ = original_call
        benchmark.KinematicAutonomousPolicy = original_policy
    outcome = {"terminal_kind": episode.terminal_kind.name,
               "active_duration_s": episode.active_duration_s, "step_count": episode.step_count,
               "role_results": [asdict(row) for row in episode.episode_results]}
    age_summary = {}
    if verify_start:
        for role in ("guardian", "explorer"):
            role_ages = [r["age_ns"] for r in ages if r["role"] == role]
            if not role_ages or max(role_ages) < genome.minimum_dwell_ns or min(role_ages) <= 0:
                raise ValueError(f"No non-vacuous dwell age observations for {role}")
            age_summary[role] = {
                "observations": len(role_ages), "min_age_ns": min(role_ages), "max_age_ns": max(role_ages),
                "at_or_beyond_dwell": sum(a >= genome.minimum_dwell_ns for a in role_ages),
                "start_resets_within_instance": 0,
            }
    return {"rows": rows, "outcome": outcome, "wall_s": wall_s,
            "totals": dict(totals), "age_summary": age_summary}


def dwell_control(role, *, emulate_old_clock):
    with pytest.MonkeyPatch.context() as patch:
        policy, manager, graph, preferences, *_ = _fixture(patch, role)
        original = policy.selector.select
        ages = []

        def observed(snapshot, **kwargs):
            if kwargs["current_started_at_ns"] is not None:
                if emulate_old_clock:
                    kwargs["current_started_at_ns"] = snapshot.context.now_ns
                ages.append(snapshot.context.now_ns - kwargs["current_started_at_ns"])
            return original(snapshot, **kwargs)

        patch.setattr(policy.selector, "select", observed)
        policy(_frame(policy, manager, graph, START_NS))
        first = policy._active
        preferences[2] = 0.75
        policy(_frame(policy, manager, graph, START_NS + DWELL_NS))
        row = {"role": role.name, "emulate_old_clock": emulate_old_clock,
               "caller_ages_ns": ages, "selected_target": policy._active.goal.target_node_id,
               "reason": policy.last_trace.selection_reason,
               "original_started_at_ns": first.started_at_ns,
               "active_started_at_ns": policy._active.started_at_ns}
        expected = ((1, "minimum_dwell", [0]) if emulate_old_clock
                    else (2, "utility_improvement_exceeds_hysteresis", [DWELL_NS]))
        if (row["selected_target"], row["reason"], ages) != expected:
            raise ValueError("Controlled counterfactual did not distinguish the old clock defect")
        return row


def endpoint_control(policy_class, mismatch):
    with pytest.MonkeyPatch.context() as patch:
        patch.setitem(_fixture.__globals__, "KinematicAutonomousPolicy", policy_class)
        policy, manager, graph, *_ = _fixture(patch)
        first_frame = _frame(policy, manager, graph, START_NS)
        policy(first_frame)
        goal = policy._active.goal if hasattr(policy._active, "goal") else policy._active[2]
        path = policy._active.path if hasattr(policy._active, "path") else policy._active[3]
        if mismatch == "target":
            bad_path = policy._path_to_node(first_frame, graph, 2)
        elif mismatch == "versions":
            bad_path = replace(path, map_version=1)
        else:
            bad_path = replace(path, polyline_xy_m=((0.0, 0.0), (0.0, 2.0)))

        def propose(frame, *args):
            from hsl_core.tactics import ROLE_FEATURES, TacticalGuard
            proposal = policy._make_proposal(frame, goal.kind, target_node_id=1,
                guards=(TacticalGuard.GRAPH_SEARCH_VIEWPOINT,), evidence="b-endpoint-counterfactual",
                features={f: float(f == "pursuit_value") for f in ROLE_FEATURES[Role.GUARDIAN]})
            return [proposal], {1: bad_path}

        patch.setattr(policy, "_proposals", propose)
        from hsl_core.types import Pose2D
        x, y = bad_path.polyline_xy_m[-1]
        policy(_frame(policy, manager, graph, START_NS + 50_000_000, pose=Pose2D(x, y, 0.0)))
        reference = policy_class is not KinematicAutonomousPolicy
        expected = "option_effect_satisfied" if reference else "authority_or_planning_rejected"
        if policy.last_trace.status != expected or (1 in policy._completed_nodes) != reference:
            raise ValueError("Endpoint negative/positive control did not reproduce/reject the false success")
        return {"source": "frozen_package_A" if reference else "package_B",
                "mismatch": mismatch, "status": policy.last_trace.status,
                "target_marked_complete": 1 in policy._completed_nodes,
                "diagnostic_valid": policy.last_trace.diagnostic_valid,
                "candidate_authorized": policy.authority.execution_state.candidate_authorized}


def main():
    baseline = build_default_baseline()
    zero_dwell = replace(baseline, minimum_dwell_ns=0)
    reference = reference_policy()
    bank = load_maze_bank(ROOT / "sim/kinematic/scenarios/phase6_maze_bank.json")
    files = (
        "sim/kinematic/autonomous.py", "sim/kinematic/selection_diagnostics.py",
        "sim/kinematic/test_active_option_lifecycle.py", "sim/kinematic/test_p56_autonomous.py",
        "sim/kinematic/test_selection_diagnostics.py", "sim/kinematic/benchmark.py",
        "sim/kinematic/match.py", "sim/kinematic/referee.py", "sim/kinematic/sensors.py",
        "hsl_core/hsl_core/tactics/fsm.py", "hsl_core/hsl_core/tactics/options.py",
        "hsl_core/hsl_core/tactics/utility.py", "hsl_core/hsl_core/planning/execution.py",
        "hsl_core/hsl_core/planning/astar.py", "hsl_core/hsl_core/control/safety.py",
        "hsl_core/hsl_core/match.py", "tools/p63_policy.py",
        "sim/kinematic/scenarios/phase6_maze_bank.json",
        "artifacts/reports/coevolution_package_b_20261002/verify_runtime.py",
        "docs/HSL26_COEVOLUTION_IMPLEMENTATION_REVIEW_20261002.md",
    )
    hashes = {p: sha256(ROOT / p) for p in files}
    cases = []
    for fixture in bank.training_fixtures():
        before = run(reference, fixture, zero_dwell)
        after = run(KinematicAutonomousPolicy, fixture, zero_dwell)
        if before["rows"] != after["rows"] or before["outcome"] != after["outcome"]:
            raise ValueError(f"Zero-dwell regression against package A in {fixture.scenario_id}")
        positive_dwell = run(KinematicAutonomousPolicy, fixture, baseline, verify_start=True)
        cases.append({
            "scenario_id": fixture.scenario_id, "split": fixture.split, "seed": 20261002,
            "zero_dwell_behavior_equal": True, "zero_dwell_outcome": after["outcome"],
            "zero_dwell_decision_command_authority_sha256": hashlib.sha256(
                json.dumps(after["rows"], sort_keys=True, allow_nan=False).encode()).hexdigest(),
            "reference_wall_s": before["wall_s"], "zero_dwell_wall_s": after["wall_s"],
            "positive_dwell_wall_s": positive_dwell["wall_s"],
            "positive_dwell_outcome": positive_dwell["outcome"],
            "positive_dwell_caller_ages": positive_dwell["age_summary"],
            "positive_dwell_selection_totals": positive_dwell["totals"],
        })
        print(f"Verified {fixture.scenario_id}: zero-dwell equality and real positive ages", flush=True)
    controls = [dwell_control(role, emulate_old_clock=old)
                for role in (Role.GUARDIAN, Role.EXPLORER) for old in (False, True)]
    endpoints = [endpoint_control(policy_class, mismatch)
                 for policy_class in (reference, KinematicAutonomousPolicy)
                 for mismatch in ("target", "versions", "geometry")]
    test_bytes = Path(__file__).with_name("tests.txt").read_bytes()
    test_text = test_bytes.decode("utf-16" if test_bytes[:2] in (b"\xff\xfe", b"\xfe\xff") else "utf-8-sig")
    summaries = re.findall(r"(\d+) passed in ([0-9.]+)s", test_text)
    if not summaries or " failed" in test_text:
        raise ValueError("Passing regression log required beside the verifier")
    if hashes != {p: sha256(ROOT / p) for p in files}:
        raise ValueError("Verification sources changed during the run")
    report = {
        "schema": "hsl26.coevolution.package-b-verification.v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "evidence_class": "DEVELOPMENT_ONLY", "promotion_eligible": False, "gate_acceptance": False,
        "reference_a_autonomous_sha256": REFERENCE_SHA256,
        "baseline_genome_sha256": baseline.sha256, "zero_dwell_genome_sha256": zero_dwell.sha256,
        "regression": {"command": REGRESSION_COMMAND, "passed": int(summaries[-1][0]),
                       "elapsed_s": float(summaries[-1][1]),
                       "log_sha256": hashlib.sha256(test_bytes).hexdigest(),
                       "note": "Validates/hashes the completed regression log; does not rerun pytest."},
        "source_sha256": hashes, "cases": cases, "controlled_dwell_counterfactuals": controls,
        "controlled_endpoint_counterfactuals": endpoints,
        "limitations": [
            "Four deterministic training fixtures, 8 s, one seed; not learned or physical performance.",
            "Equivalence against A is asserted with dwell=0; positive dwell intentionally repairs behavior.",
            "Controlled proposals isolate utility/dwell; production selector/authority/control/safety remain active.",
            "Clock epoch change revokes at last old-clock authority stamp and requires a fresh policy instance.",
            "Wall times include wrapper/warm-up effects; not a sustained overhead benchmark.",
            "Package C, sensitivity, training eligibility and G4/G5/G6 acceptance remain pending.",
        ],
    }
    path = Path(__file__).with_name("runtime_verification.json")
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(path), "equivalent_zero_dwell_fixtures": len(cases),
                      "controlled_counterfactuals": len(controls), "regression_passed": report["regression"]["passed"]}))


if __name__ == "__main__":
    main()
