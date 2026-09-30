"""Verify P6.4 dossier integrity and report release blockers without arming."""

from __future__ import annotations

import argparse
import hashlib
import json
import importlib.metadata
import platform
from pathlib import Path
import sys
from typing import Any

from tools.p63_policy import EVIDENCE_CLASS
from tools.release_freeze import SCHEMA, _source_inventory, canonical_sha256
from tools.validate_config import validate_sil_profile


def _check(name: str, passed: bool, detail: str) -> dict[str, Any]:
    return {"check": name, "status": "PASS" if passed else "FAIL", "detail": detail}


def verify_dossier(
    manifest: dict[str, Any],
    *,
    root: str | Path,
) -> dict[str, Any]:
    repository = Path(root).resolve()
    if not isinstance(manifest, dict):
        raise TypeError("manifest must be a JSON object")
    if manifest.get("schema") != SCHEMA:
        raise ValueError("unsupported Phase-6 release dossier schema")
    checks: list[dict[str, Any]] = []
    recorded_digest = manifest.get("manifest_sha256")
    checks.append(
        _check(
            "manifest_digest",
            isinstance(recorded_digest, str)
            and recorded_digest == canonical_sha256(manifest),
            "Canonical manifest SHA-256 must match its recorded digest.",
        )
    )

    frozen_inputs = manifest.get("frozen_inputs")
    source_inventory = manifest.get("source_inventory")
    if not isinstance(frozen_inputs, list) or not frozen_inputs:
        raise ValueError("manifest frozen_inputs must be a non-empty list")
    if not isinstance(source_inventory, list) or not source_inventory:
        raise ValueError("manifest source_inventory must be a non-empty list")
    expected_inventory = _source_inventory(repository)
    inventory_coverage = source_inventory == expected_inventory
    checks.append(
        _check(
            "source_inventory_coverage",
            inventory_coverage,
            "Manifest inventory must cover the complete declared release source set.",
        )
    )
    input_findings = []
    seen_paths: dict[str, tuple[Any, Any]] = {}
    for item in (*frozen_inputs, *source_inventory):
        if not isinstance(item, dict):
            raise ValueError("manifest frozen input entry must be an object")
        relative = item.get("path")
        expected_digest = item.get("sha256")
        expected_size = item.get("size_bytes")
        if (
            not isinstance(relative, str)
            or not relative
            or Path(relative).is_absolute()
            or ".." in Path(relative).parts
        ):
            raise ValueError("frozen input path must be a safe repository-relative path")
        recorded_pair = (expected_digest, expected_size)
        if relative in seen_paths:
            if seen_paths[relative] != recorded_pair:
                input_findings.append(
                    {
                        "path": relative,
                        "status": "FAIL",
                        "detail": "duplicate file entries disagree",
                    }
                )
            continue
        seen_paths[relative] = recorded_pair
        target = (repository / Path(relative)).resolve()
        if not target.is_relative_to(repository) or not target.is_file():
            input_findings.append(
                {
                    "path": relative,
                    "status": "FAIL",
                    "detail": "missing or outside repository",
                }
            )
            continue
        digest = hashlib.sha256(target.read_bytes()).hexdigest()
        size = target.stat().st_size
        valid_digest = (
            isinstance(expected_digest, str)
            and len(expected_digest) == 64
            and digest == expected_digest
        )
        valid_size = (
            isinstance(expected_size, int)
            and not isinstance(expected_size, bool)
            and size == expected_size
        )
        input_findings.append(
            {
                "path": relative,
                "status": "PASS" if valid_digest and valid_size else "FAIL",
                "detail": (
                    "size and SHA-256 match"
                    if valid_digest and valid_size
                    else "size or SHA-256 mismatch"
                ),
            }
        )
    checks.append(
        _check(
            "frozen_inputs",
            all(item["status"] == "PASS" for item in input_findings),
            "All referenced profile, policy, fixture, authority and evidence inputs must match.",
        )
    )

    authority = manifest.get("authority")
    constraints = manifest.get("environment_constraints")
    release_decision = manifest.get("release_decision")
    gates = manifest.get("gate_matrix")
    if not all(
        isinstance(value, dict)
        for value in (authority, constraints, release_decision)
    ):
        raise ValueError("manifest authority, constraints, or decision is malformed")
    if not isinstance(gates, list) or not gates:
        raise ValueError("manifest gate_matrix must be a non-empty list")
    gate_status = {
        entry.get("gate"): entry.get("status")
        for entry in gates
        if isinstance(entry, dict)
    }
    gates_well_formed = (
        len(gate_status) == 7
        and set(gate_status) == {f"G{index}" for index in range(7)}
    )
    gates_blocked = (
        gates_well_formed
        and gate_status.get("G4") == "BLOCKED_NOT_RUN"
        and gate_status.get("G5") == "NOT_RUN"
        and gate_status.get("G6") == "BLOCKED"
    )
    safe_boundary = (
        authority.get("physical_authority") is False
        and authority.get("official_scoring_authority") is False
        and authority.get("simulator_truth_available_to_policy") is False
        and constraints.get("physical_devices_enabled") is False
        and release_decision.get("promotion_eligible") is False
        and release_decision.get("status") == "BLOCKED_FOR_RELEASE"
        and bool(release_decision.get("blocking_reasons"))
        and gates_blocked
    )
    checks.append(
        _check(
            "authority_and_gate_boundary",
            safe_boundary,
            "Fixture dossier must stay non-promotable and keep G4/G5/G6 unresolved.",
        )
    )
    static_review = manifest.get("static_authority_review")
    static_authority_valid = (
        isinstance(static_review, dict)
        and static_review.get("expected_velocity_writer") == "cmd_vel_mux"
        and static_review.get("expected_velocity_topic") == "/commands/velocity"
        and static_review.get("live_ros_graph_probe") == "NOT_RUN"
        and static_review.get("tf_ownership_runtime_probe") == "NOT_RUN"
        and static_review.get("configuration_calibration_acceptance") == "BLOCKED"
    )
    checks.append(
        _check(
            "runtime_authority_scope",
            static_authority_valid,
            "Expected static writer is pinned, while live ROS/TF ownership remains explicitly NOT_RUN.",
        )
    )
    policy_reference = authority.get("policy_path")
    policy_digest = authority.get("policy_sha256")
    policy_path = (
        repository / policy_reference
        if isinstance(policy_reference, str)
        else None
    )
    policy_recorded = any(
        item.get("path") == policy_reference
        and item.get("kind") == "development_policy"
        for item in frozen_inputs
    )
    policy_valid = False
    if (
        policy_recorded
        and isinstance(policy_path, Path)
        and policy_path.is_file()
        and policy_path.resolve().is_relative_to(repository)
        and isinstance(policy_digest, str)
    ):
        try:
            policy_payload = json.loads(policy_path.read_text(encoding="utf-8"))
            policy_valid = (
                policy_payload.get("evidence_class") == EVIDENCE_CLASS
                and policy_payload.get("promotion_eligible") is False
                and policy_payload.get("official_score_available") is False
                and policy_payload.get("genome_sha256") == policy_digest
            )
        except (json.JSONDecodeError, AttributeError):
            policy_valid = False
    checks.append(
        _check(
            "serialized_policy_boundary",
            policy_valid,
            "Pinned policy must match the genome digest and remain fixture-only/non-promotable.",
        )
    )
    profile_reference = manifest.get("frozen_inputs")
    profile_record = next(
        (
            item for item in profile_reference
            if item.get("kind") == "sil_profile"
        ),
        None,
    )
    profile_valid = False
    profile_reference_valid = False
    if isinstance(profile_record, dict):
        profile_path = repository / profile_record["path"]
        profile_reference_valid = (
            profile_path.resolve().is_relative_to(repository)
            and profile_path.is_file()
        )
        if profile_reference_valid:
            try:
                profile_payload = json.loads(profile_path.read_text(encoding="utf-8"))
                profile_valid = validate_sil_profile(profile_payload)[
                    "sil_profile_valid"
                ]
                profile_valid = (
                    profile_valid
                    and manifest.get("sil_profile_sha256")
                    == profile_record.get("sha256")
                    and manifest.get("profile") == "KINEMATIC_SIL_ONLY"
                    and constraints.get("network_downloads_allowed") is False
                    and constraints.get("editable_bind_mounts_allowed") is False
                    and profile_payload.get("fixture_bank")
                    == authority.get("scenario_bank_path")
                    and profile_payload.get("development_policy")
                    == authority.get("policy_path")
                )
            except (json.JSONDecodeError, AttributeError, KeyError):
                profile_valid = False
    checks.append(
        _check(
            "sil_profile_constraints",
            profile_reference_valid and profile_valid,
            "The hashed profile must identify SIL-only use with downloads, mounts, and hardware disabled.",
        )
    )
    runtime = manifest.get("runtime")
    runtime_valid = isinstance(runtime, dict)
    if runtime_valid:
        current_packages = {}
        for package in ("numpy", "scipy", "pytest"):
            try:
                current_packages[package] = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                current_packages[package] = None
        runtime_valid = (
            runtime.get("os") == platform.platform()
            and runtime.get("python") == sys.version
            and runtime.get("python_implementation")
            == platform.python_implementation()
            and runtime.get("packages") == current_packages
        )
    checks.append(
        _check(
            "runtime_fingerprint",
            runtime_valid,
            "Current OS/interpreter/package versions must match the locally frozen SIL environment.",
        )
    )
    passed = all(item["status"] == "PASS" for item in checks)
    return {
        "schema": "hsl26.phase6.preflight-report.v1",
        "status": (
            "PASS_SIL_INTEGRITY_RELEASE_BLOCKED" if passed else "FAIL_PREFLIGHT"
        ),
        "sil_integrity_pass": passed,
        "release_eligible": False,
        "physical_authority": False,
        "checks": checks,
        "frozen_input_checks": input_findings,
        "gate_matrix": gate_status,
        "must_not_arm_or_deploy": True,
    }


def verify_dossier_file(
    manifest_path: str | Path,
    *,
    root: str | Path,
) -> dict[str, Any]:
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    return verify_dossier(manifest, root=root)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    report = verify_dossier_file(args.manifest, root=args.root)
    encoded = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output is None:
        print(encoded, end="")
    else:
        output = args.output.resolve()
        if output.exists():
            raise FileExistsError(f"refusing to overwrite preflight report: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded, encoding="utf-8")
    if report["status"] == "FAIL_PREFLIGHT":
        raise SystemExit(2)
