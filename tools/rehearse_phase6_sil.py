"""Run a bounded two-stage, role-swapped P6.5 SIL release rehearsal."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any

from hsl_core.match import Role
from sim.kinematic.benchmark import BenchmarkConfig, SILBenchmarkRunner
from sim.kinematic.maze_bank import load_maze_bank
from tools.p63_policy import load_development_policy
from tools.preflight_check import verify_dossier
from tools.release_freeze import canonical_sha256


def _fault_injection_checks(
    manifest: dict[str, Any],
    *,
    root: Path,
    policy_path: Path,
    expected_policy_sha256: str,
) -> dict[str, Any]:
    mutated_manifest = deepcopy(manifest)
    mutated_manifest["frozen_inputs"][0]["sha256"] = "0" * 64
    mutated_manifest["manifest_sha256"] = canonical_sha256(mutated_manifest)
    tamper_report = verify_dossier(mutated_manifest, root=root)
    tamper_detected = (
        tamper_report["status"] == "FAIL_PREFLIGHT"
        and any(
            item["status"] == "FAIL"
            for item in tamper_report["frozen_input_checks"]
        )
    )
    if not tamper_detected:
        raise RuntimeError("fault injection failed: changed frozen input was not detected")

    baseline, payload = load_development_policy(policy_path)
    corrupted_payload = deepcopy(payload)
    corrupted_payload["genome_sha256"] = "0" * 64
    with tempfile.TemporaryDirectory(prefix="hsl26-p65-policy-tamper-") as temp_dir:
        corrupt_file = Path(temp_dir) / "tampered_policy.json"
        corrupt_file.write_text(
            json.dumps(corrupted_payload, allow_nan=False),
            encoding="utf-8",
        )
        try:
            load_development_policy(corrupt_file)
        except ValueError as error:
            if "digest does not match" not in str(error):
                raise RuntimeError(
                    "tampered policy was rejected for an unexpected reason"
                ) from error
            policy_tamper_detected = True
        else:
            raise RuntimeError("fault injection failed: tampered policy was accepted")

    restored, restored_payload = load_development_policy(policy_path)
    rollback_verified = (
        restored.sha256 == expected_policy_sha256
        and baseline.sha256 == expected_policy_sha256
        and restored_payload["promotion_eligible"] is False
    )
    if not rollback_verified:
        raise RuntimeError("rollback failed to restore the hash-pinned SIL fallback")
    return {
        "changed_frozen_input": {
            "injected": True,
            "detected": tamper_detected,
            "expected_response": "preflight failure; no stage starts",
        },
        "tampered_policy_genome_digest": {
            "injected": True,
            "detected": policy_tamper_detected,
            "expected_response": "policy rejected before execution",
        },
        "rollback_to_fixture_baseline": {
            "verified": rollback_verified,
            "policy_sha256": restored.sha256,
            "accepted_g4_baseline": False,
            "note": "Pinned development fallback only; this is not a last accepted release.",
        },
    }


def _fresh_process_check(root: Path, policy_path: Path, expected_sha256: str) -> dict[str, Any]:
    source = (
        "import json,sys; "
        "from tools.p63_policy import load_development_policy; "
        "genome,payload=load_development_policy(sys.argv[1]); "
        "print(json.dumps({'sha256':genome.sha256,"
        "'promotion_eligible':payload['promotion_eligible']}))"
    )
    environment = os.environ.copy()
    python_paths = [str(root / "hsl_core"), str(root)]
    existing = environment.get("PYTHONPATH")
    if existing:
        python_paths.append(existing)
    environment["PYTHONPATH"] = os.pathsep.join(python_paths)
    completed = subprocess.run(
        [sys.executable, "-c", source, str(policy_path)],
        cwd=root,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "fresh-process SIL cold-start check failed: "
            f"exit={completed.returncode}; stderr={completed.stderr.strip()}"
        )
    try:
        payload = json.loads(completed.stdout.strip())
    except json.JSONDecodeError as error:
        raise RuntimeError("fresh-process check emitted malformed JSON") from error
    if payload.get("sha256") != expected_sha256 or payload.get("promotion_eligible") is not False:
        raise RuntimeError("fresh-process check loaded the wrong or promotable policy")
    return {
        "status": "PASS_FRESH_SIL_PROCESS",
        "process_exit_code": completed.returncode,
        "loaded_policy_sha256": payload["sha256"],
        "network_isolation_verified": False,
        "final_image_cold_start_verified": False,
        "note": "A local subprocess start is not an offline final-image or hardware cold boot.",
    }


def _stage_record(
    episode,
    *,
    stage_number: int,
    freeze_duration_ns: int,
) -> dict[str, Any]:
    samples = episode.telemetry
    if len(samples) < 2:
        raise RuntimeError(f"stage {stage_number} did not produce sufficient telemetry")
    stamps = [sample.stamp_ns for sample in samples]
    monotonic = all(left < right for left, right in zip(stamps, stamps[1:]))
    freeze_samples = tuple(sample for sample in samples if sample.stamp_ns < freeze_duration_ns)
    freeze_zero = bool(freeze_samples) and all(
        sample.guardian_command == (0.0, 0.0)
        and sample.explorer_command == (0.0, 0.0)
        for sample in freeze_samples
    )
    roles = {result.role for result in episode.episode_results}
    roles_complete = roles == {Role.GUARDIAN, Role.EXPLORER} and all(
        result.completed for result in episode.episode_results
    )
    safety_violations = sum(
        result.safety_violations for result in episode.episode_results
    )
    collisions = sum(result.collisions for result in episode.episode_results)
    no_contacts = (
        episode.guardian_wall_contacts == 0
        and episode.explorer_wall_contacts == 0
    )
    passed = (
        monotonic
        and freeze_zero
        and roles_complete
        and safety_violations == 0
        and collisions == 0
        and no_contacts
    )
    return {
        "stage_number": stage_number,
        "scenario_id": episode.scenario_id,
        "scenario_sha256": episode.scenario_sha256,
        "seed": episode.seed,
        "virtual_robot_identity_by_role": {
            role.name: identity
            for role, identity in episode.robot_identity_by_role
        },
        "identity_mapping_is_virtual_only": True,
        "role_policy_ids": {
            role.name: digest for role, digest in episode.role_policy_ids
        },
        "terminal_kind": episode.terminal_kind.name,
        "active_duration_s": episode.active_duration_s,
        "step_count": episode.step_count,
        "telemetry_samples": len(samples),
        "telemetry_clock_strictly_monotonic": monotonic,
        "freeze_samples": len(freeze_samples),
        "freeze_commands_zero": freeze_zero,
        "roles_completed": roles_complete,
        "safety_violations": safety_violations,
        "collisions": collisions,
        "guardian_wall_contacts": episode.guardian_wall_contacts,
        "explorer_wall_contacts": episode.explorer_wall_contacts,
        "checks": {
            "telemetry_clock_strictly_monotonic": monotonic,
            "freeze_commands_zero": freeze_zero,
            "both_roles_completed": roles_complete,
            "zero_safety_violations": safety_violations == 0,
            "zero_robot_collisions": collisions == 0,
            "zero_wall_contacts": no_contacts,
        },
        "passed": passed,
        "official_score": None,
        "surrogate_terminal_outcome": episode.terminal_kind.name,
        "evidence_class": episode.evidence_class,
    }


def run_phase6_sil_rehearsal(
    *,
    manifest_path: str | Path,
    output_dir: str | Path,
    root: str | Path,
    duration_s: float = 30.0,
    seed: int = 20260930,
) -> dict[str, Any]:
    repository = Path(root).resolve()
    manifest_file = Path(manifest_path).resolve()
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    preflight = verify_dossier(manifest, root=repository)
    if not preflight["sil_integrity_pass"]:
        raise ValueError("P6.4 preflight integrity failed; refusing to start SIL stages")
    if preflight["release_eligible"] is not False:
        raise RuntimeError("SIL preflight unexpectedly claims release eligibility")
    if (
        isinstance(duration_s, bool)
        or not isinstance(duration_s, (int, float))
        or not math.isfinite(duration_s)
        or duration_s <= 0.15
    ):
        raise ValueError("duration_s must be greater than the 0.15 s fixture freeze")
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ValueError("seed must be a non-negative integer")

    authority = manifest["authority"]
    bank_file = repository / authority["scenario_bank_path"]
    policy_file = repository / authority["policy_path"]
    bank = load_maze_bank(bank_file)
    fixtures_by_id = {fixture.scenario_id: fixture for fixture in bank.training_fixtures()}
    stage_scenarios = ("maze_interior_loops_train_c", "maze_interior_loops_train_d")
    if any(scenario not in fixtures_by_id for scenario in stage_scenarios):
        raise ValueError("rehearsal stage fixtures are missing from the training split")
    profile_file = repository / "config/schema/phase6_sil_release_profile.json"
    profile = json.loads(profile_file.read_text(encoding="utf-8"))
    stage_config = profile["synthetic_stage"]
    if (
        stage_config.get("freeze_duration_ns") != 150_000_000
        or stage_config.get("control_period_s") != 0.05
    ):
        raise ValueError("SIL profile timing does not match the supported P6.3 runner")
    if duration_s > stage_config["stage_duration_s"]:
        raise ValueError("duration_s cannot exceed the versioned synthetic stage duration")

    genome, policy_payload = load_development_policy(policy_file)
    if genome.sha256 != authority["policy_sha256"]:
        raise ValueError("policy digest does not match the frozen release dossier")
    fault_checks = _fault_injection_checks(
        manifest,
        root=repository,
        policy_path=policy_file,
        expected_policy_sha256=authority["policy_sha256"],
    )
    cold_start = _fresh_process_check(
        repository,
        policy_file,
        authority["policy_sha256"],
    )

    runner = SILBenchmarkRunner(
        BenchmarkConfig(
            episode_duration_s=float(duration_s),
            control_period_s=float(stage_config["control_period_s"]),
        )
    )
    stages = []
    for index, scenario_id in enumerate(stage_scenarios):
        role_swap = index == 1
        identities = (
            {Role.GUARDIAN: "sil-robot-A", Role.EXPLORER: "sil-robot-B"}
            if not role_swap
            else {Role.GUARDIAN: "sil-robot-B", Role.EXPLORER: "sil-robot-A"}
        )
        episode = runner.run_episode(
            genome,
            fixtures_by_id[scenario_id],
            seed=seed + index,
            robot_identity_by_role=identities,
            collect_telemetry=True,
        )
        stages.append(
            _stage_record(
                episode,
                stage_number=index + 1,
                freeze_duration_ns=stage_config["freeze_duration_ns"],
            )
        )

    role_swap_verified = (
        stages[0]["virtual_robot_identity_by_role"]["GUARDIAN"]
        == stages[1]["virtual_robot_identity_by_role"]["EXPLORER"]
        and stages[0]["virtual_robot_identity_by_role"]["EXPLORER"]
        == stages[1]["virtual_robot_identity_by_role"]["GUARDIAN"]
    )
    report = {
        "schema": "hsl26.phase6.p65-sil-rehearsal.v1",
        "status": "PASS_SIL_FIXTURE_REHEARSAL",
        "created_from_manifest_sha256": manifest["manifest_sha256"],
        "preflight": preflight,
        "cold_start": cold_start,
        "run_configuration": {
            "duration_s_per_stage": float(duration_s),
            "control_period_s": float(stage_config["control_period_s"]),
            "freeze_duration_ns": stage_config["freeze_duration_ns"],
            "seed": seed,
            "scenario_bank_sha256": next(
                item["sha256"]
                for item in manifest["frozen_inputs"]
                if item["path"] == authority["scenario_bank_path"]
            ),
            "policy_sha256": genome.sha256,
            "selected_policy_reason": policy_payload["selection_reason"],
            "official_score_available": False,
            "held_out_episodes_evaluated": 0,
        },
        "fault_injection_and_recovery": fault_checks,
        "stages": stages,
        "role_swap_verified": role_swap_verified,
        "inter_stage_memory": {
            "sil_behavior": "RESET_BETWEEN_STAGES",
            "organizer_retention_policy": "UNKNOWN",
            "accepted_competition_behavior": False,
        },
        "rollback": fault_checks["rollback_to_fixture_baseline"],
        "gates": {
            "P6.4": "PASS_SIL_INPUT_INTEGRITY_ONLY",
            "P6.5": "PASS_SIL_FIXTURE_REHEARSAL_ONLY",
            **preflight["gate_matrix"],
            "formal_g6_acceptance": "BLOCKED",
        },
        "release_decision": "BLOCKED_FOR_RELEASE",
        "limits": [
            "No robot, final release image, ROS graph, MVSim, or physical calibration was available.",
            "Robot identities are virtual labels; these runs do not verify physical robot or memory retention behavior.",
            "Fixture timing and synthetic map/goal/score assumptions are not official inputs.",
            "A passing fixture rehearsal is not G4, G5, or G6 acceptance and cannot arm hardware.",
            "The selected policy is the retained development baseline, not a learned or accepted competition policy.",
        ],
    }
    if len(stages) != 2 or not role_swap_verified or any(
        not stage["passed"] for stage in stages
    ):
        report["status"] = "FAIL_SIL_FIXTURE_REHEARSAL"

    destination = Path(output_dir).resolve()
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite rehearsal directory: {destination}")
    destination.mkdir(parents=True, exist_ok=False)
    report_path = destination / "rehearsal_report.json"
    report["report_path"] = str(report_path)
    temporary_path = destination / ".rehearsal_report.json.tmp"
    temporary_path.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary_path.replace(report_path)
    return report


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=20260930)
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    result = run_phase6_sil_rehearsal(
        manifest_path=args.manifest,
        output_dir=args.output_dir,
        root=args.root,
        duration_s=args.duration_s,
        seed=args.seed,
    )
    print(
        f"{result['status']}; G6={result['gates']['formal_g6_acceptance']}; "
        f"report={result['report_path']}"
    )
    if result["status"] != "PASS_SIL_FIXTURE_REHEARSAL":
        raise SystemExit(2)
