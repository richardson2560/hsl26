"""Contract and case-inventory checks for the Phase-5 SIL manifest."""

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
    assert manifest["execution_status"] == (
        "KINEMATIC_LIFECYCLE_REHEARSAL_TESTED_G4_AND_RUNTIME_INTEGRATION_BLOCKED"
    )
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


def test_p5_rehearsal_fixture_labels_assumed_timing_and_external_zone_as_unapproved():
    manifest = _manifest()
    rehearsal = manifest["rehearsal_profile"]
    assert rehearsal["kind"] == "DEVELOPMENT_TEST_FIXTURE_ONLY"
    assert rehearsal["freeze_duration_ns"] == 100_000_000
    assert rehearsal["stage_duration_ns"] == 300_000_000
    assert rehearsal["official_authority"] is False
    assert rehearsal["accepted_goal_zone"] is None
    assert rehearsal["memory_retention"] == "DISABLED"
    case_ids = {case["id"] for case in manifest["case_catalog"]}
    assert {
        "cold_start_stage_lifecycle_and_role_swap",
        "injected_stage_planner_sensor_and_watchdog_faults",
        "deterministic_replay_and_truth_adjudication_boundary",
    } <= case_ids
