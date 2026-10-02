"""Read-only runtime audit of the attempted coevolution step, development only.

Uses existing test builders for small adversarial snapshots. Wrappers are
process-local, restored in finally, and never change a decision or command.
This is evidence collection, not the proposed production diagnostic runner.
"""
from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / "hsl_core"), str(ROOT / "hsl_core/tests")]

from hsl_core.match import Role, StagePhase
from hsl_core.tactics import OptionKind
from hsl_core.tactics.fsm import TacticalSelector
from test_p53_tactics import (
    _fallback, _features, _proposal, _selector, _snapshot,
)
from sim.kinematic.autonomous import KinematicAutonomousPolicy
from sim.kinematic.benchmark import BenchmarkConfig, SILBenchmarkRunner
from sim.kinematic.match import MatchRole
from sim.kinematic.maze_bank import load_maze_bank
from sim.kinematic.test_p56_autonomous import _graph, _runner
from tools.p63_policy import build_default_baseline


def attempted_counts(snapshot):
    policy = KinematicAutonomousPolicy(MatchRole(snapshot.context.role.name.lower()))
    values = policy._selection_trace_state_for(
        SimpleNamespace(phase=snapshot.context.stage_phase), snapshot,
        system_ready=snapshot.system_ready,
    )
    keys = (
        "phase", "system_ready", "proposal_total", "proposal_feasible",
        "proposal_choice", "n_active", "n_available", "n_choice",
        "n_forced", "n_empty",
    )
    return dict(zip(keys, values))


def evaluated_counts(snapshot, result):
    """Selector admissibility and cohort cardinality; no route dedup claim."""
    eligible = [
        e for e in result.alternatives
        if e.applicable and e.kind != OptionKind.HOLD_SAFE
    ]
    cohort = []
    if eligible:
        priority = min(e.priority for e in eligible)
        same_priority = [e for e in eligible if e.priority == priority]
        urgency = max(e.urgency_rank or 0 for e in same_priority)
        cohort = [e for e in same_priority if (e.urgency_rank or 0) == urgency]
    return {
        "selector_invoked_active": int(snapshot.context.stage_phase == StagePhase.ACTIVE),
        "applicable_non_hold": len(eligible),
        "winning_cohort_cardinality_before_behavior_dedup": len(cohort),
        "winning_cohort_keys": [e.stable_key for e in cohort],
        "selection_reason": result.reason,
        "rejections": {e.stable_key: e.reason for e in result.alternatives if not e.applicable},
    }


def synthetic_cases():
    role = Role.GUARDIAN
    four = tuple(_proposal(OptionKind.SEARCH_PORTAL, role, target_node_id=i) for i in range(1, 5))
    single = four[:1]
    cases = {
        "empty_non_hold_set": _snapshot(role, (_fallback(role),)),
        "single_applicable_proposal": _snapshot(role, (*single, _fallback(role))),
        "four_applicable_proposals": _snapshot(role, (*four, _fallback(role))),
        "same_target_different_instance_ids": _snapshot(role, (
            _proposal(OptionKind.SEARCH_PORTAL, role, target_node_id=1, stable_key="a", instance_id="a-id"),
            _proposal(OptionKind.SEARCH_PORTAL, role, target_node_id=1, stable_key="b", instance_id="b-id"),
            _fallback(role),
        )),
        "different_priorities_single_winning_candidate": _snapshot(role, (
            _proposal(OptionKind.PRESSURE_ROUTE, role, target_node_id=1),
            _proposal(OptionKind.SEARCH_PORTAL, role, target_node_id=2),
            _fallback(role),
        )),
        "different_urgencies_single_winning_candidate": _snapshot(Role.EXPLORER, (
            _proposal(OptionKind.BREAK_LOS, Role.EXPLORER, target_node_id=1, urgency_rank=200),
            _proposal(OptionKind.KEEP_ESCAPE_ROUTE, Role.EXPLORER, target_node_id=2, urgency_rank=10),
            _fallback(Role.EXPLORER),
        )),
        "missing_required_guard": _snapshot(role, (
            _proposal(OptionKind.SEARCH_PORTAL, role, target_node_id=1, guards=()),
            _fallback(role),
        )),
    }
    base = _snapshot(role, (*single, _fallback(role)))
    cases["not_ready"] = replace(base, system_ready=False, health_reason="audit-no-ready")
    cases["safety_stop"] = replace(base, context=replace(base.context, safety_stop=True))
    cases["invalid_lease"] = replace(base, context=replace(base.context, lease_valid=False))
    cases["motion_not_authorized"] = replace(base, context=replace(base.context, motion_authorized=False))
    rows = {}
    for name, snapshot in cases.items():
        result = _selector(snapshot.context.role).select(snapshot)
        counts = attempted_counts(snapshot)
        rows[name] = {
            "attempted": counts,
            "selector": evaluated_counts(snapshot, result),
            "partition_sum": counts["n_choice"] + counts["n_forced"] + counts["n_empty"],
        }
    return rows


def dwell_controlled_case():
    role = Role.GUARDIAN
    a = _proposal(OptionKind.SEARCH_PORTAL, role, target_node_id=1,
                  features=dict(_features(role, pursuit_value=0.0)))
    b = _proposal(OptionKind.SEARCH_PORTAL, role, target_node_id=2,
                  features=dict(_features(role, pursuit_value=1.0)))
    snapshot = _snapshot(role, (a, b, _fallback(role)))
    kwargs = {"current_stable_key": a.stable_key, "current_option_instance_id": a.goal.option_instance_id}
    results = {}
    for name, start in (("actual_old_start", snapshot.context.now_ns - 1_000_000_000),
                        ("start_recomputed_as_now", snapshot.context.now_ns)):
        result = _selector(role, hysteresis=0.0, dwell_ns=250_000_000).select(
            snapshot, current_started_at_ns=start, **kwargs,
        )
        results[name] = {"reason": result.reason, "selected_key": result.selected.stable_key,
                         "elapsed_ns": snapshot.context.now_ns - start}
    return results


def runtime_cases():
    original_call = KinematicAutonomousPolicy.__call__
    original_select = TacticalSelector.select
    pending = {}
    status_counts = defaultdict(Counter)
    totals = defaultdict(Counter)
    examples = {}
    last_frames = {}

    def measured_select(self, snapshot, *args, **kwargs):
        result = original_select(self, snapshot, *args, **kwargs)
        row = evaluated_counts(snapshot, result)
        row["attempted"] = attempted_counts(snapshot)
        start = kwargs.get("current_started_at_ns")
        row["current_age_ns"] = None if start is None else snapshot.context.now_ns - start
        pending[id(self)] = row
        return result

    def measured_call(self, frame):
        pending.pop(id(self.selector), None)
        last_frames[self.role.value] = frame
        command = original_call(self, frame)
        row = pending.pop(id(self.selector), None)
        trace = self.last_trace
        role = self.role.value
        status_counts[role][trace.status] += 1
        if row:
            totals[role]["actual_active_selector_calls"] += row["selector_invoked_active"]
            totals[role]["active_calls_with_applicable_proposal"] += int(
                row["selector_invoked_active"] and row["applicable_non_hold"] > 0
            )
            totals[role]["active_calls_with_multiple_cohort_entries_before_dedup"] += int(
                row["selector_invoked_active"] and row["winning_cohort_cardinality_before_behavior_dedup"] >= 2
            )
            if row["current_age_ns"] is not None:
                totals[role]["current_start_supplied"] += 1
                totals[role]["current_age_zero"] += int(row["current_age_ns"] == 0)
        for name in ("n_active", "n_available", "n_choice", "n_forced", "n_empty"):
            totals[role]["trace_sum_" + name] += getattr(trace, name)
        key = role + ":" + trace.status
        if key not in examples:
            examples[key] = {"frame_stamp_ns": frame.stamp_ns,
                             "frame_phase": frame.match_state.phase.name if frame.match_state else None,
                             "trace": asdict(trace), "selector": row}
        return command

    try:
        KinematicAutonomousPolicy.__call__ = measured_call
        TacticalSelector.select = measured_select
        match, guardian, _ = _runner(Role.GUARDIAN, _graph())
        for _ in range(3):
            match.step(0.05)
        good_frame = last_frames["guardian"]
        # Inject a legitimate next-call missing input to expose trace carry-over.
        missing_frame = replace(good_frame, stamp_ns=good_frame.stamp_ns + 1, topology_graph=None)
        guardian(missing_frame)
        match, _, _ = _runner(Role.EXPLORER, _graph())
        for _ in range(3):
            match.step(0.05)
        bank = load_maze_bank(ROOT / "sim/kinematic/scenarios/phase6_maze_bank.json")
        fixture = next(f for f in bank.training_fixtures() if f.scenario_id == "maze_multiring_7x7_train_a")
        config = BenchmarkConfig(episode_duration_s=8.0)
        episode = SILBenchmarkRunner(config).run_episode(build_default_baseline(), fixture, seed=20261002)
    finally:
        KinematicAutonomousPolicy.__call__ = original_call
        TacticalSelector.select = original_select
    return {
        "status_counts_including_small_regressions": dict(status_counts),
        "totals_including_small_regressions": dict(totals),
        "examples": examples,
        "short_training_fixture": {
            "scenario_id": fixture.scenario_id, "seed": episode.seed,
            "config": asdict(config), "terminal": episode.terminal_kind.name,
            "active_duration_s": episode.active_duration_s,
            "limitation": "8 s horizon is a development fixture timeout, not a competition survival victory.",
        },
    }


def main():
    files = [
        "sim/kinematic/autonomous.py", "sim/kinematic/benchmark.py",
        "hsl_core/hsl_core/tactics/fsm.py", "hsl_core/hsl_core/tactics/options.py",
        "hsl_core/hsl_core/learning/evolution.py", "tools/train_evolution.py",
        "tools/p63_policy.py", "docs/HSL26_COEVOLUTION_PLAN.md",
        "sim/kinematic/scenarios/phase6_maze_bank.json",
        "hsl_core/tests/test_p53_tactics.py", "sim/kinematic/test_p56_autonomous.py",
        "artifacts/reports/coevolution_step1_audit_20261002/reproduce.py",
    ]
    hashes = {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in files}
    report = {
        "schema": "hsl26.coevolution.step1-audit.v1", "audit_date": "2026-10-02",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_sha256": hashes, "python": platform.python_version(),
        "promotion_eligible": False, "gate_acceptance": False,
        "synthetic_cases": synthetic_cases(), "controlled_dwell": dwell_controlled_case(),
        "runtime": runtime_cases(),
        "limitations": [
            "Audit wrappers observe existing decisions; no production fix is applied.",
            "Cohort counts are cardinalities before route/effect deduplication; not N_choice rates.",
            "Runtime totals mix three small regression cases and one training fixture; no statistical claim.",
            "No training search, validation, held-out, ROS, hardware or HGW acceptance is run.",
            "Synthetic builders are imported from existing tests; this script needs pytest installed.",
        ],
    }
    report["source_unchanged_during_run"] = all(
        hashes[p] == hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in files
    )
    dest = Path(__file__).with_name("findings.json")
    dest.write_text(json.dumps(report, indent=2, default=lambda x: x.name) + "\n", encoding="utf-8")
    print(json.dumps({"report": str(dest), "source_unchanged": report["source_unchanged_during_run"],
                      "synthetic_cases": len(report["synthetic_cases"]),
                      "runtime_totals": report["runtime"]["totals_including_small_regressions"]}, indent=2))


if __name__ == "__main__":
    main()
