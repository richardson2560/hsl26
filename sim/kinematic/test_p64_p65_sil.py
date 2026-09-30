"""Adversarial checks for P6.4 dossier integrity and P6.5 SIL rehearsal."""

from copy import deepcopy
from dataclasses import fields
import json
from pathlib import Path

import pytest

from hsl_core.match import Role
from hsl_core.learning.evolution import EvaluationSplit
from sim.kinematic.benchmark import BenchmarkConfig, SILBenchmarkRunner
from sim.kinematic.maze_bank import load_maze_bank
from sim.kinematic.match import SILPolicyInput
from tools.preflight_check import verify_dossier
from tools.release_freeze import (
    DEFAULT_BANK_PATH,
    DEFAULT_POLICY_PATH,
    build_release_dossier,
    canonical_sha256,
    write_release_dossier,
)
from tools.rehearse_phase6_sil import run_phase6_sil_rehearsal
from tools.validate_config import validate_sil_profile


_ROOT = Path(__file__).resolve().parents[2]


def _build_dossier():
    return build_release_dossier(
        root=_ROOT,
        release_id="p64-test",
        policy_path=DEFAULT_POLICY_PATH,
        bank_path=DEFAULT_BANK_PATH,
    )


def test_sil_config_validator_rejects_authority_leak_and_unknown_official_input():
    profile = json.loads(
        (_ROOT / "config/schema/phase6_sil_release_profile.json").read_text(
            encoding="utf-8"
        )
    )
    assert validate_sil_profile(profile)["status"] == "VALID_SIL_FIXTURE_ONLY"

    profile["physical_devices_enabled"] = True
    profile["official_inputs"]["zone_and_goal_provenance"] = "assumed"
    report = validate_sil_profile(profile)

    assert report["status"] == "INVALID_SIL_PROFILE"
    assert report["release_eligible"] is False
    assert {check["check"] for check in report["checks"] if check["status"] == "FAIL"} == {
        "execution_isolation",
        "official_inputs_explicitly_unresolved",
    }


def test_release_dossier_hashes_inputs_but_never_claims_release_authority():
    dossier = _build_dossier()
    report = verify_dossier(dossier, root=_ROOT)

    assert dossier["manifest_sha256"] == canonical_sha256(dossier)
    assert report["status"] == "PASS_SIL_INTEGRITY_RELEASE_BLOCKED"
    assert report["release_eligible"] is False
    assert dossier["release_decision"]["status"] == "BLOCKED_FOR_RELEASE"
    assert dossier["authority"]["physical_authority"] is False
    assert dossier["authority"]["official_scoring_authority"] is False
    assert dossier["authority"]["held_out_evaluated"] is False
    assert dossier["gate_matrix"][-1]["status"] == "BLOCKED"
    assert dossier["official_inputs"]["status"] == "INCOMPLETE"
    assert "final_target_image_digest" in dossier["official_inputs"]["unresolved"]
    assert any(
        item["path"].endswith("MatchState.msg")
        for item in dossier["source_inventory"]
    )


def test_preflight_detects_tampering_even_when_manifest_digest_is_recomputed():
    dossier = _build_dossier()
    changed = deepcopy(dossier)
    changed["frozen_inputs"][0]["sha256"] = "0" * 64
    changed["manifest_sha256"] = canonical_sha256(changed)

    report = verify_dossier(changed, root=_ROOT)

    assert report["status"] == "FAIL_PREFLIGHT"
    assert report["release_eligible"] is False
    assert any(
        finding["status"] == "FAIL"
        for finding in report["frozen_input_checks"]
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("physical_authority", True),
        ("official_scoring_authority", True),
        ("simulator_truth_available_to_policy", True),
    ],
)
def test_preflight_rejects_authority_boundary_regression(field, value):
    dossier = _build_dossier()
    dossier["authority"][field] = value
    dossier["manifest_sha256"] = canonical_sha256(dossier)

    report = verify_dossier(dossier, root=_ROOT)

    assert report["status"] == "FAIL_PREFLIGHT"
    boundary = next(
        check for check in report["checks"]
        if check["check"] == "authority_and_gate_boundary"
    )
    assert boundary["status"] == "FAIL"


def test_preflight_rejects_claimed_live_graph_pass_without_runtime_evidence():
    dossier = _build_dossier()
    dossier["static_authority_review"]["live_ros_graph_probe"] = "PASS"
    dossier["manifest_sha256"] = canonical_sha256(dossier)

    report = verify_dossier(dossier, root=_ROOT)

    assert report["status"] == "FAIL_PREFLIGHT"
    graph_check = next(
        check for check in report["checks"]
        if check["check"] == "runtime_authority_scope"
    )
    assert graph_check["status"] == "FAIL"


def test_preflight_rejects_runtime_drift_after_manifest_rehash():
    dossier = _build_dossier()
    dossier["runtime"]["python"] = "different interpreter"
    dossier["manifest_sha256"] = canonical_sha256(dossier)

    report = verify_dossier(dossier, root=_ROOT)

    assert report["status"] == "FAIL_PREFLIGHT"
    runtime_check = next(
        check for check in report["checks"]
        if check["check"] == "runtime_fingerprint"
    )
    assert runtime_check["status"] == "FAIL"


def test_freeze_refuses_to_overwrite_existing_dossier(tmp_path):
    output = tmp_path / "existing"
    output.mkdir()
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        write_release_dossier(
            root=_ROOT,
            output_dir=output,
            release_id="p64-overwrite-test",
        )


def test_p65_two_stage_sil_rehearsal_role_swaps_and_keeps_g6_blocked(tmp_path):
    dossier = _build_dossier()
    manifest_dir = tmp_path / "manifest"
    manifest_dir.mkdir()
    manifest_path = manifest_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(dossier, indent=2, allow_nan=False),
        encoding="utf-8",
    )

    result = run_phase6_sil_rehearsal(
        manifest_path=manifest_path,
        output_dir=tmp_path / "rehearsal",
        root=_ROOT,
        duration_s=0.5,
        seed=2046,
    )

    assert result["status"] == "PASS_SIL_FIXTURE_REHEARSAL"
    assert result["release_decision"] == "BLOCKED_FOR_RELEASE"
    assert result["gates"]["P6.4"] == "PASS_SIL_INPUT_INTEGRITY_ONLY"
    assert result["gates"]["P6.5"] == "PASS_SIL_FIXTURE_REHEARSAL_ONLY"
    assert result["gates"]["formal_g6_acceptance"] == "BLOCKED"
    assert result["role_swap_verified"] is True
    assert result["cold_start"]["status"] == "PASS_FRESH_SIL_PROCESS"
    assert result["cold_start"]["final_image_cold_start_verified"] is False
    assert result["rollback"]["accepted_g4_baseline"] is False
    assert result["fault_injection_and_recovery"]["changed_frozen_input"]["detected"]
    assert result["fault_injection_and_recovery"]["tampered_policy_genome_digest"]["detected"]
    assert len(result["stages"]) == 2
    assert all(stage["freeze_commands_zero"] for stage in result["stages"])
    assert all(stage["telemetry_clock_strictly_monotonic"] for stage in result["stages"])
    assert all(stage["roles_completed"] for stage in result["stages"])
    assert all(stage["official_score"] is None for stage in result["stages"])
    assert all(stage["collisions"] == 0 for stage in result["stages"])
    assert Path(result["report_path"]).is_file()
    saved_report = json.loads(
        Path(result["report_path"]).read_text(encoding="utf-8")
    )
    assert saved_report["report_path"] == result["report_path"]


def test_p65_caps_episode_duration_at_declared_sil_profile_limit(tmp_path):
    dossier = _build_dossier()
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(dossier), encoding="utf-8")

    with pytest.raises(ValueError, match="cannot exceed"):
        run_phase6_sil_rehearsal(
            manifest_path=manifest_path,
            output_dir=tmp_path / "overlong",
            root=_ROOT,
            duration_s=30.1,
            seed=2048,
        )
    assert not (tmp_path / "overlong").exists()


def test_sil_runner_binds_role_swap_identity_into_endpoint_provenance():
    from tools.p63_policy import build_default_baseline

    fixture = load_maze_bank(DEFAULT_BANK_PATH).training_fixtures()[0]
    runner = SILBenchmarkRunner(
        BenchmarkConfig(episode_duration_s=0.5, max_episode_steps=20)
    )
    genome = build_default_baseline()
    stage_one = runner.run_episode(
        genome,
        fixture,
        seed=2049,
        robot_identity_by_role={
            Role.GUARDIAN: "sil-A",
            Role.EXPLORER: "sil-B",
        },
    )
    stage_two = runner.run_episode(
        genome,
        fixture,
        seed=2050,
        robot_identity_by_role={
            Role.GUARDIAN: "sil-B",
            Role.EXPLORER: "sil-A",
        },
    )

    assert dict(stage_one.robot_identity_by_role) == {
        Role.GUARDIAN: "sil-A",
        Role.EXPLORER: "sil-B",
    }
    assert dict(stage_two.robot_identity_by_role) == {
        Role.GUARDIAN: "sil-B",
        Role.EXPLORER: "sil-A",
    }
    assert stage_one.evaluation_profile_id != stage_two.evaluation_profile_id
    assert "robot_identity" not in {
        field.name for field in fields(SILPolicyInput)
    }
    assert all(result.split is EvaluationSplit.TRAINING for result in stage_two.episode_results)
    with pytest.raises(ValueError, match="uniquely assign"):
        runner.run_episode(
            genome,
            fixture,
            seed=2051,
            robot_identity_by_role={
                Role.GUARDIAN: "duplicate",
                Role.EXPLORER: "duplicate",
            },
        )


def test_p65_refuses_to_start_stages_when_preflight_integrity_fails(tmp_path):
    dossier = _build_dossier()
    dossier["frozen_inputs"][0]["sha256"] = "0" * 64
    dossier["manifest_sha256"] = canonical_sha256(dossier)
    manifest_path = tmp_path / "tampered-manifest.json"
    manifest_path.write_text(json.dumps(dossier), encoding="utf-8")

    with pytest.raises(ValueError, match="preflight integrity failed"):
        run_phase6_sil_rehearsal(
            manifest_path=manifest_path,
            output_dir=tmp_path / "must-not-start",
            root=_ROOT,
            duration_s=0.5,
            seed=2047,
        )
    assert not (tmp_path / "must-not-start").exists()
