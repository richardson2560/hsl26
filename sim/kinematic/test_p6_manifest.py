"""Contract checks for the Phase-6 no-hardware preparation manifest."""

import json
from pathlib import Path


MANIFEST_PATH = Path(__file__).parent / "scenarios" / "phase6_learning_release.json"


def _manifest():
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def test_p6_manifest_is_preparation_only_and_keeps_gates_open():
    manifest = _manifest()
    authority = manifest["authority"]
    packages = manifest["work_packages"]

    assert manifest["schema"] == "hsl26.phase6.environment-manifest.v1"
    assert authority["physical_authority"] is False
    assert authority["official_scoring_authority"] is False
    assert authority["learning_enabled"] is False
    assert authority["g4_status"] == "BLOCKED_NOT_RUN"
    assert authority["g5_status"] == "NOT_RUN"
    assert authority["g6_status"] == "BLOCKED"
    assert manifest["profiles"]["offline_learning"]["pure_algorithm_development"] == (
        "ALLOWED_WITH_NORMATIVE_FIXTURES"
    )
    assert manifest["profiles"]["offline_learning"]["status"] == (
        "EMPIRICAL_EVALUATION_BLOCKED_UNTIL_ACCEPTED_BASELINE_AND_BANKS"
    )
    assert packages["P6.0"] == "PREPARED"
    assert packages["P5.6"] == (
        "PARTIAL_SIMULTANEOUS_SIL_FIXTURE_G4_AND_AUTONOMOUS_BASELINE_BLOCKED"
    )
    assert packages["P6.1"] == (
        "SCHEMA_WORK_ALLOWED_DATASET_BLOCKED_UNTIL_ACCEPTED_BASELINE"
    )
    assert packages["P6.2"] == (
        "PURE_IMPLEMENTATION_ALLOWED_EMPIRICAL_EVALUATION_BLOCKED"
    )
    assert packages["P6.3"] == (
        "FIXTURE_SEARCH_IMPLEMENTED_ELIGIBLE_EVALUATION_BLOCKED_UNTIL_G4_AND_ACCEPTED_DATA"
    )
    assert packages["P6.4"] == (
        "SIL_DOSSIER_PREFLIGHT_IMPLEMENTED_FORMAL_RELEASE_BLOCKED_UNTIL_G4_AND_APPROVED_INPUTS"
    )
    assert packages["P6.5"] == (
        "TWO_STAGE_SIL_REHEARSAL_IMPLEMENTED_FORMAL_G6_BLOCKED_UNTIL_FINAL_TARGET"
    )


def test_p6_fixture_cannot_be_promoted_to_training_data():
    manifest = _manifest()

    assert all(
        not split
        for split in manifest["dataset_splits"].values()
        if isinstance(split, list)
    )
    assert manifest["dataset_splits"]["status"] == "NOT_AVAILABLE"
    assert all(
        fixture["eligible_as_training_episode"] is False
        for fixture in manifest["fixture_references"]
    )


def test_p6_policy_boundary_excludes_referee_truth_and_is_hardware_free():
    manifest = _manifest()
    boundary = manifest["policy_boundary"]

    assert not set(boundary["referee_only_fields"]) & set(
        boundary["permitted_feature_sources"]
    )
    assert boundary["truth_access_by_policy"] is False
    assert manifest["profiles"]["kinematic_sil_preparation"]["physical_authority"] is False
    assert manifest["profiles"]["hardware"]["status"] == "DISARMED_NO_ROBOT_OR_CALIBRATION"


def test_p6_manifest_keeps_phase5_external_inputs_explicit():
    manifest = _manifest()

    assert manifest["p5_evidence"]["accepted_g4_baseline"]["status"] == "BLOCKED_NOT_RUN"
    assert manifest["dataset_splits"]["training"] == []
    assert manifest["dataset_splits"]["validation"] == []
    assert manifest["dataset_splits"]["held_out"] == []
    assert manifest["software_dependencies"]["new_required"] == []
