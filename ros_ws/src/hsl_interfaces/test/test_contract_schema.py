# ros_ws/src/hsl_interfaces/test/test_contract_schema.py
"""Static contract checks that run without a sourced ROS environment."""

from pathlib import Path
import re


PACKAGE = Path(__file__).resolve().parents[1]
MSG = PACKAGE / "msg"
SRV = PACKAGE / "srv"
ACTION = PACKAGE / "action"


REVISION_2_MESSAGES = {
    "ContractHeader",
    "Polygon2",
    "Point2",
    "EgoState",
    "CoverageGrid",
    "Obstacle2",
    "LocalObstacleSnapshot",
    "OpponentTrack",
    "GoalZone",
    "WorldSnapshot",
    "TopologyNode",
    "TopologyEdge",
    "MatchState",
    "OptionGoal",
    "OptionFeedback",
    "OptionResult",
    "ExecutionState",
    "PathPlan",
    "MotionCandidate",
    "SafetyStatus",
    "SupervisorHeartbeat",
    "WatchdogHealth",
    "RuleEvent",
}


def _fields(path: Path) -> list[str]:
    fields = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" in line:
            continue
        fields.append(line.split()[1])
    return fields


def test_revision_2_message_inventory_and_cmake_registration():
    actual = {path.stem for path in MSG.glob("*.msg")}
    assert REVISION_2_MESSAGES <= actual
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    for name in REVISION_2_MESSAGES:
        assert f'"msg/{name}.msg"' in cmake


def test_top_level_records_use_contract_header_first():
    records = {
        "EgoState",
        "LocalObstacleSnapshot",
        "MatchState",
        "MotionCandidate",
        "OpponentTrack",
        "OptionFeedback",
        "OptionGoal",
        "OptionResult",
        "PathPlan",
        "RuleEvent",
        "SafetyStatus",
        "SupervisorHeartbeat",
        "WatchdogHealth",
        "WorldSnapshot",
    }
    for name in records:
        assert _fields(MSG / f"{name}.msg")[0] == "meta"


def test_header_field_order_and_serialized_constants():
    fields = _fields(MSG / "ContractHeader.msg")
    assert fields == [
        "schema_version",
        "source_id",
        "source_session",
        "seq",
        "stage_id",
        "clock_epoch",
        "localization_epoch",
        "frame_id",
        "observation_stamp",
        "state_stamp",
        "publication_stamp",
        "valid_until",
        "map_version",
        "topology_version",
        "validity",
    ]
    text = (MSG / "ContractHeader.msg").read_text(encoding="utf-8")
    assert "uint16 REVISION_2 = 2" in text
    assert "uint8 INVALID = 0" in text
    assert "uint8 VALID = 1" in text
    assert "uint8 DEGRADED = 2" in text


def test_fixed_covariance_order_and_no_bootstrap_aliases():
    assert _fields(MSG / "EgoState.msg")[4:6] == [
        "pose_covariance",
        "twist_covariance",
    ]
    assert _fields(MSG / "OpponentTrack.msg")[7] == "covariance"
    all_idl = "\n".join(
        path.read_text(encoding="utf-8")
        for path in list(MSG.glob("*.msg")) + list(SRV.glob("*.srv"))
    )
    assert not re.search(r"\b(header|linear_velocity|angular_velocity|option_id)\b", all_idl)


def test_transactional_interfaces_are_registered():
    services = {
        "StartStage",
        "SetGoalZone",
        "ResetStage",
        "ResolveEvent",
        "RearmSafety",
    }
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    for name in services:
        assert (SRV / f"{name}.srv").exists()
        assert f'"srv/{name}.srv"' in cmake
    action = (ACTION / "ExecuteOption.action").read_text(encoding="utf-8")
    assert "OptionGoal goal" in action
    assert "OptionResult result" in action
    assert "OptionFeedback feedback" in action
