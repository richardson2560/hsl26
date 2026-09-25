# sim/kinematic/scenario.py
"""Strict validation for versioned Phase-3 scenario manifests."""

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ScenarioSpec:
    scenario_id: str
    seed: int
    profile: str
    map_version: int
    localization_epoch: str
    physical_authority: bool


def load_scenario(path: str | Path) -> ScenarioSpec:
    with Path(path).open("r", encoding="utf-8") as stream:
        data: Any = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("scenario manifest must be an object")
    required = (
        "scenario_id", "seed", "profile", "physical_authority",
        "map_version", "localization_epoch", "grid", "truth_boundary",
    )
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError(f"scenario manifest missing fields: {missing}")
    if not isinstance(data["scenario_id"], str) or not data["scenario_id"]:
        raise ValueError("scenario_id must be a non-empty string")
    if not isinstance(data["seed"], int) or isinstance(data["seed"], bool):
        raise ValueError("seed must be an integer")
    if data["profile"] not in ("observed_map", "approved_map", "oracle"):
        raise ValueError("unsupported scenario profile")
    if not isinstance(data["physical_authority"], bool):
        raise ValueError("physical_authority must be boolean")
    if data["physical_authority"]:
        raise ValueError("kinematic scenarios cannot claim physical authority")
    if not isinstance(data["map_version"], int) or data["map_version"] < 0:
        raise ValueError("map_version must be a non-negative integer")
    if not isinstance(data["localization_epoch"], str) or not data["localization_epoch"]:
        raise ValueError("localization_epoch must be non-empty")
    grid = data["grid"]
    if not isinstance(grid, dict) or grid.get("unknown_is_blocking") is not True:
        raise ValueError("grid must explicitly block unknown cells")
    boundary = data["truth_boundary"]
    if not isinstance(boundary, dict):
        raise ValueError("truth_boundary must be an object")
    for field in ("referee_only_fields", "policy_visible_fields"):
        if not isinstance(boundary.get(field), list):
            raise ValueError(f"truth_boundary.{field} must be a list")
    if set(boundary["referee_only_fields"]) & set(boundary["policy_visible_fields"]):
        raise ValueError("truth fields must not be policy-visible")
    return ScenarioSpec(
        data["scenario_id"],
        data["seed"],
        data["profile"],
        data["map_version"],
        data["localization_epoch"],
        data["physical_authority"],
    )
