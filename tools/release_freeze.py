"""Create a hash-verified P6.4 SIL release dossier, never a release approval."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any

from sim.kinematic.maze_bank import load_maze_bank
from tools.p63_policy import load_development_policy


SCHEMA = "hsl26.phase6.sil-release-dossier.v1"
PROFILE_PATH = Path("config/schema/phase6_sil_release_profile.json")
DEFAULT_BANK_PATH = Path(
    "sim/kinematic/scenarios/phase6_interior_tactics_bank.json"
)
DEFAULT_POLICY_PATH = Path(
    "artifacts/reports/phase6/p63_interior_dev_seed20260930/policy.json"
)
SOURCE_ROOTS = (
    Path("hsl_core/hsl_core"),
    Path("sim/kinematic"),
    Path("ros_ws/src"),
    Path("config/schema"),
    Path("kobuki/workspace/src/cmd_vel_mux/config"),
)
SOURCE_EXTENSIONS = {
    ".py",
    ".msg",
    ".srv",
    ".action",
    ".json",
    ".yaml",
    ".yml",
    ".xml",
    ".toml",
    ".txt",
}
SOURCE_FILES = (
    Path("tools/release_freeze.py"),
    Path("tools/preflight_check.py"),
    Path("tools/validate_config.py"),
    Path("tools/check_ros_graph_authority.py"),
    Path("tools/rehearse_phase6_sil.py"),
    Path("tools/p63_policy.py"),
    Path("tools/train_evolution.py"),
    Path("tools/benchmark_p63.py"),
    Path("tools/demo_p63_match.py"),
    Path("docker/Dockerfile"),
    Path("docker/entrypoint.bash"),
    Path("hsl_core/pyproject.toml"),
    Path("ros_ws/src/hsl_interfaces/CMakeLists.txt"),
    Path("ros_ws/src/hsl_safety/package.xml"),
    Path("docs/HSL26_TECHNICAL_SPECIFICATION.md"),
    Path("docs/HSL26_FINAL_ARCHITECTURE.md"),
    Path("docs/HSL26_PHASE6_LEARNING_AND_RELEASE.md"),
    Path("docs/runbook_competition.md"),
    Path("artifacts/reports/phase5/P5.6_autonomous_integration_report.json"),
    Path("artifacts/reports/phase6/P6.3_interior_fixture_development_closeout.json"),
)
REQUIRED_INPUTS = (
    "accepted_g4_baseline",
    "official_zone_goal_and_scoring_provenance",
    "approved_timing_trigger_and_memory_policy",
    "physical_robot_identity_and_calibration",
    "final_target_image_digest",
)
GATE_MATRIX = {
    "G0": "BLOCKED",
    "G1": "BLOCKED",
    "G2": "BLOCKED",
    "G3": "BLOCKED",
    "G4": "BLOCKED_NOT_RUN",
    "G5": "NOT_RUN",
    "G6": "BLOCKED",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(payload: dict[str, Any]) -> str:
    unsigned = dict(payload)
    unsigned.pop("manifest_sha256", None)
    encoded = json.dumps(
        unsigned,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _git_state(root: Path) -> dict[str, Any]:
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=root,
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    status = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=root,
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    if head.returncode != 0 or status.returncode != 0:
        return {
            "head": None,
            "working_tree": "UNKNOWN",
            "git_probe_error": (head.stderr + status.stderr).strip(),
        }
    return {
        "head": head.stdout.strip(),
        "working_tree": "CLEAN" if not status.stdout.strip() else "DIRTY",
        "dirty_path_count": len(status.stdout.splitlines()),
    }


def _source_inventory(root: Path) -> list[dict[str, Any]]:
    paths: set[Path] = set(SOURCE_FILES)
    for source_root in SOURCE_ROOTS:
        directory = root / source_root
        if not directory.is_dir():
            raise FileNotFoundError(
                f"required source directory is missing: {source_root}"
            )
        paths.update(
            path.relative_to(root)
            for path in directory.rglob("*")
            if path.is_file()
            and path.suffix.lower() in SOURCE_EXTENSIONS
            and "__pycache__" not in path.parts
            and ".pytest_cache" not in path.parts
        )
    inventory = []
    for path in sorted(paths, key=lambda item: item.as_posix()):
        resolved = root / path
        if not resolved.resolve().is_relative_to(root.resolve()):
            raise ValueError(f"source inventory path escapes repository root: {path}")
        if not resolved.is_file():
            raise FileNotFoundError(f"required release input is missing: {path}")
        inventory.append(
            {
                "path": path.as_posix(),
                "size_bytes": resolved.stat().st_size,
                "sha256": sha256_file(resolved),
            }
        )
    return inventory


def _input_record(root: Path, path: Path, kind: str) -> dict[str, Any]:
    resolved = (root / path).resolve()
    if not resolved.is_relative_to(root.resolve()):
        raise ValueError(f"release input escapes repository root: {path}")
    if not resolved.is_file():
        raise FileNotFoundError(f"release input does not exist: {path}")
    return {
        "kind": kind,
        "path": path.as_posix(),
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
    }


def _package_versions() -> dict[str, str | None]:
    versions: dict[str, str | None] = {}
    for package in ("numpy", "scipy", "pytest"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    return versions


def build_release_dossier(
    *,
    root: str | Path,
    release_id: str,
    policy_path: str | Path = DEFAULT_POLICY_PATH,
    bank_path: str | Path = DEFAULT_BANK_PATH,
) -> dict[str, Any]:
    repository = Path(root).resolve()
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_."
    if not release_id or any(char not in allowed for char in release_id):
        raise ValueError("release_id may contain only letters, digits, '.', '_' and '-'")
    policy_file = Path(policy_path)
    bank_file = Path(bank_path)
    if (
        policy_file.is_absolute()
        or bank_file.is_absolute()
        or ".." in policy_file.parts
        or ".." in bank_file.parts
    ):
        raise ValueError("policy and bank paths must be repository-relative")
    for relative in (policy_file, bank_file, PROFILE_PATH):
        if not (repository / relative).resolve().is_relative_to(repository):
            raise ValueError(f"release input escapes repository root: {relative}")
    profile = json.loads((repository / PROFILE_PATH).read_text(encoding="utf-8"))
    if (
        policy_file.as_posix() != profile.get("development_policy")
        or bank_file.as_posix() != profile.get("fixture_bank")
    ):
        raise ValueError("policy and fixture bank must match the versioned SIL profile")
    from tools.validate_config import validate_sil_profile

    profile_report = validate_sil_profile(profile)
    if not profile_report["sil_profile_valid"]:
        raise ValueError("SIL profile failed typed configuration validation")

    policy, policy_payload = load_development_policy(repository / policy_file)
    if policy_payload["promotion_eligible"] is not False:
        raise ValueError("P6.4 SIL dossier accepts only explicitly non-promotable policy")
    bank = load_maze_bank(repository / bank_file)
    if not bank.training_fixtures() or not bank.validation_fixtures():
        raise ValueError("release fixtures must include training and validation splits")

    inputs = [
        _input_record(repository, PROFILE_PATH, "sil_profile"),
        _input_record(repository, bank_file, "synthetic_fixture_bank"),
        _input_record(repository, policy_file, "development_policy"),
    ]
    for source in (
        Path("docs/HSL26_TECHNICAL_SPECIFICATION.md"),
        Path("docs/HSL26_FINAL_ARCHITECTURE.md"),
        Path("docs/HSL26_PHASE6_LEARNING_AND_RELEASE.md"),
        Path("docs/runbook_competition.md"),
        Path("artifacts/reports/phase5/P5.6_autonomous_integration_report.json"),
        Path("artifacts/reports/phase6/P6.3_interior_fixture_development_closeout.json"),
    ):
        inputs.append(_input_record(repository, source, "authority_or_evidence"))

    dossier: dict[str, Any] = {
        "schema": SCHEMA,
        "release_id": release_id,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "profile": "KINEMATIC_SIL_ONLY",
        "release_decision": {
            "status": "BLOCKED_FOR_RELEASE",
            "sil_dossier_status": "HASHES_VERIFIED_NO_RELEASE_AUTHORITY",
            "promotion_eligible": False,
            "official_score_available": False,
            "physical_authority": False,
            "blocking_reasons": [
                "G4 has no accepted deterministic baseline.",
                "Organizer-approved objective, score, stage timing, trigger, and memory policy are unavailable.",
                "No physical robot identity, calibration, or stopping-response evidence is available.",
                "No final target image digest, offline cold-start evidence, or immutable release environment is available.",
                "This worktree is not an immutable signed release source.",
            ],
        },
        "signature": {
            "status": "NOT_SIGNED",
            "signer_id": None,
            "reason": "No trusted release signing authority or key was supplied.",
        },
        "source_revision": _git_state(repository),
        "runtime": {
            "os": platform.platform(),
            "python": sys.version,
            "python_implementation": platform.python_implementation(),
            "python_executable": sys.executable,
            "packages": _package_versions(),
            "docker_image_digest": None,
            "ros_distribution": None,
            "mvsim_version": None,
        },
        "authority": {
            "physical_authority": False,
            "official_scoring_authority": False,
            "simulator_truth_available_to_policy": False,
            "held_out_evaluated": False,
            "policy_sha256": policy.sha256,
            "policy_path": policy_file.as_posix(),
            "scenario_bank_path": bank_file.as_posix(),
            "scenario_bank_split_counts": {
                "training": len(bank.training_fixtures()),
                "validation": len(bank.validation_fixtures()),
                "held_out_reserved_not_run": len(bank.held_out_fixtures()),
            },
            "model": {"status": "NOT_USED_IN_DECLARED_SIL_PROFILE", "sha256": None},
        },
        "official_inputs": {
            "status": "INCOMPLETE",
            "unresolved": list(REQUIRED_INPUTS),
            "values": profile["official_inputs"],
        },
        "frozen_inputs": inputs,
        "source_inventory": _source_inventory(repository),
        "gate_matrix": [
            {"gate": gate, "status": status, "evidence": None}
            for gate, status in GATE_MATRIX.items()
        ],
        "sil_profile_sha256": inputs[0]["sha256"],
        "config_validation": profile_report,
        "environment_constraints": {
            "network_downloads_allowed": False,
            "editable_bind_mounts_allowed": False,
            "physical_devices_enabled": False,
            "note": "These are intended constraints; this local dossier does not prove container or host isolation.",
        },
        "static_authority_review": {
            "expected_velocity_writer": "cmd_vel_mux",
            "expected_velocity_topic": "/commands/velocity",
            "evidence_inputs_hashed": [
                "kobuki/workspace/src/cmd_vel_mux/config/cmd_vel_mux_params.yaml",
                "ros_ws/src/hsl_interfaces",
                "ros_ws/src/hsl_safety",
            ],
            "live_ros_graph_probe": "NOT_RUN",
            "tf_ownership_runtime_probe": "NOT_RUN",
            "configuration_calibration_acceptance": "BLOCKED",
        },
    }
    dossier["manifest_sha256"] = canonical_sha256(dossier)
    return dossier


def write_release_dossier(
    *,
    root: str | Path,
    output_dir: str | Path,
    release_id: str,
    policy_path: str | Path = DEFAULT_POLICY_PATH,
    bank_path: str | Path = DEFAULT_BANK_PATH,
) -> Path:
    destination = Path(output_dir).resolve()
    if destination.exists():
        raise FileExistsError(
            f"refusing to overwrite release dossier directory: {destination}"
        )
    dossier = build_release_dossier(
        root=root,
        release_id=release_id,
        policy_path=policy_path,
        bank_path=bank_path,
    )
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "manifest.json"
    temporary_path = destination / ".manifest.json.tmp"
    temporary_path.write_text(
        json.dumps(dossier, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary_path.replace(manifest_path)
    return manifest_path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release-id", required=True)
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY_PATH)
    parser.add_argument("--bank", type=Path, default=DEFAULT_BANK_PATH)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    output = write_release_dossier(
        root=Path.cwd(),
        output_dir=args.output_dir,
        release_id=args.release_id,
        policy_path=args.policy,
        bank_path=args.bank,
    )
    print(f"SIL dossier written: {output}; release_decision=BLOCKED_FOR_RELEASE")
