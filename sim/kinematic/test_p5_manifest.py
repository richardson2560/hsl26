"""Preparation checks for the Phase-5 case manifest; no match runner is implied."""

import json
from pathlib import Path

from sim.kinematic.scenario import load_scenario


MANIFEST_PATH = Path(__file__).parent / "scenarios" / "match_tactics.json"


def _manifest():
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def test_p5_manifest_uses_the_existing_kinematic_scenario_contract():
    manifest = _manifest()
    scenario = load_scenario(MANIFEST_PATH)

    assert scenario.scenario_id == "phase5-match-tactics-sil"
    assert scenario.profile == "observed_map"
    assert scenario.physical_authority is False
    assert manifest["execution_status"] == "MANIFEST_PREPARED_RUNNER_NOT_IMPLEMENTED"
    assert all(Path(path).is_file() for path in manifest["base_fixtures"])


def test_p5_manifest_keeps_external_zone_and_stage_inputs_unresolved():
    stage = _manifest()["stage_profile"]

    assert stage["freeze_duration_s"] is None
    assert stage["stage_duration_s"] is None
    assert stage["goal_zone_provider"] == "UNRESOLVED"
    assert stage["goal_zone_coordinates"] is None
    assert stage["start_trigger"] == "UNAPPROVED_EXTERNAL_INPUT"
    assert stage["memory_retention"] == "DISABLED_UNTIL_APPROVED"


def test_p5_case_catalog_covers_both_roles_without_exposing_truth():
    manifest = _manifest()
    cases = {case["id"]: set(case["tests"]) for case in manifest["case_catalog"]}
    boundary = manifest["truth_boundary"]

    assert set(manifest["roles"]) == {"guardian", "explorer"}
    assert {"capture_boundary", "unresolved_goal_and_stale_match_state",
            "two_robot_truth_isolation"} <= set(cases)
    assert "T02" in cases["capture_boundary"]
    assert "I14" in cases["two_robot_truth_isolation"]
    assert not set(boundary["referee_only_fields"]) & set(boundary["policy_visible_fields"])
