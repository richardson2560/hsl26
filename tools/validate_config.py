"""Validate the typed boundary of the P6 kinematic-SIL release profile."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


SCHEMA = "hsl26.phase6.sil-release-profile.v1"
REQUIRED_OFFICIAL_INPUTS = {
    "accepted_g4_baseline",
    "zone_and_goal_provenance",
    "stage_timing_and_trigger",
    "score_profile",
    "memory_retention_policy",
    "robot_identity_and_calibration",
    "final_target_image_digest",
}


def validate_sil_profile(profile: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(profile, dict):
        raise TypeError("SIL profile must be a JSON object")
    checks: list[dict[str, Any]] = []

    def add(name: str, passed: bool, detail: str) -> None:
        checks.append(
            {"check": name, "status": "PASS" if passed else "FAIL", "detail": detail}
        )

    add(
        "schema",
        profile.get("schema") == SCHEMA and bool(profile.get("profile_id")),
        "Versioned P6 SIL profile schema and ID are required.",
    )
    add(
        "fixture_authority_boundary",
        profile.get("profile_kind") == "SOFTWARE_FIXTURE_ONLY"
        and profile.get("physical_authority") is False
        and profile.get("official_scoring_authority") is False
        and profile.get("simulator_truth_available_to_policy") is False,
        "Profile must not grant physical, scoring, or policy truth authority.",
    )
    add(
        "execution_isolation",
        profile.get("physical_devices_enabled") is False
        and profile.get("network_downloads_allowed") is False
        and profile.get("editable_bind_mounts_allowed") is False,
        "Fixture execution must disable physical devices, downloads, and editable mounts.",
    )
    official = profile.get("official_inputs")
    add(
        "official_inputs_explicitly_unresolved",
        isinstance(official, dict)
        and set(official) == REQUIRED_OFFICIAL_INPUTS
        and all(value is None for value in official.values()),
        "Every organizer and physical input is explicitly null until authorized evidence exists.",
    )
    timing = profile.get("synthetic_stage")
    valid_timing = False
    if isinstance(timing, dict):
        duration = timing.get("stage_duration_s")
        period = timing.get("control_period_s")
        valid_timing = (
            timing.get("freeze_duration_ns") == 150_000_000
            and isinstance(duration, (int, float))
            and not isinstance(duration, bool)
            and math.isfinite(duration)
            and duration > 0.15
            and period == 0.05
            and timing.get("score_profile_id")
            == "synthetic-zero-sum-win-loss-surrogate-v1"
            and timing.get("memory_between_stages") == "RESET_FOR_SIL_ONLY"
        )
    add(
        "supported_synthetic_timing",
        valid_timing,
        "Synthetic timing, surrogate, and reset behavior must match the tested runner profile.",
    )
    references_valid = all(
        isinstance(profile.get(name), str)
        and bool(profile[name])
        and not Path(profile[name]).is_absolute()
        and ".." not in Path(profile[name]).parts
        for name in ("fixture_bank", "development_policy")
    )
    add(
        "repository_relative_artifact_references",
        references_valid,
        "Fixture and policy references must be repository-relative and traversal-free.",
    )
    passed = all(check["status"] == "PASS" for check in checks)
    return {
        "schema": "hsl26.phase6.config-validation-report.v1",
        "status": "VALID_SIL_FIXTURE_ONLY" if passed else "INVALID_SIL_PROFILE",
        "sil_profile_valid": passed,
        "release_eligible": False,
        "physical_authority": False,
        "checks": checks,
    }


def validate_sil_profile_file(path: str | Path) -> dict[str, Any]:
    profile = json.loads(Path(path).read_text(encoding="utf-8"))
    return validate_sil_profile(profile)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "profile",
        nargs="?",
        type=Path,
        default=Path("config/schema/phase6_sil_release_profile.json"),
    )
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    report = validate_sil_profile_file(args.profile)
    encoded = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output is None:
        print(encoded, end="")
    else:
        output = args.output.resolve()
        if output.exists():
            raise FileExistsError(f"refusing to overwrite config report: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded, encoding="utf-8")
    if not report["sil_profile_valid"]:
        raise SystemExit(2)
