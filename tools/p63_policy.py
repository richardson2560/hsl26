"""Serialization helpers for explicitly non-promotable P6.3 fixture policies."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from hsl_core.learning.evolution import TacticalGenome
from hsl_core.match import Role
from hsl_core.tactics import UtilityProfile


SCHEMA = "hsl26.p63-development-policy.v1"
EVIDENCE_CLASS = "SIL_DEVELOPMENT_FIXTURE_NOT_PROMOTION_ELIGIBLE"


def build_default_baseline() -> TacticalGenome:
    guardian = UtilityProfile(
        "baseline-guardian-v1",
        Role.GUARDIAN,
        (
            ("capture_opportunity", 0.40),
            ("portal_time_advantage", 0.25),
            ("observation_gain", 0.15),
            ("pursuit_value", 0.10),
            ("duration_cost", -0.10),
        ),
        1,
    )
    explorer = UtilityProfile(
        "baseline-explorer-v1",
        Role.EXPLORER,
        (
            ("base_progress", 0.35),
            ("visibility_loss", 0.20),
            ("alternative_exits", 0.15),
            ("escape_safety", 0.15),
            ("observation_gain", 0.05),
            ("capture_risk", -0.05),
            ("duration_cost", -0.05),
        ),
        1,
    )
    return TacticalGenome.from_profiles(
        guardian,
        explorer,
        hysteresis_delta_u=0.08,
        minimum_dwell_ns=250_000_000,
    )


def policy_payload(
    genome: TacticalGenome,
    *,
    selection_reason: str,
    baseline_sha256: str,
    run_id: str,
) -> dict[str, Any]:
    if not isinstance(genome, TacticalGenome):
        raise TypeError("genome must be a TacticalGenome")
    if not selection_reason or not run_id:
        raise ValueError("selection_reason and run_id must be non-empty")
    if not _is_sha256(baseline_sha256):
        raise ValueError("baseline_sha256 must be a lowercase SHA-256 digest")
    return {
        "schema": SCHEMA,
        "evidence_class": EVIDENCE_CLASS,
        "promotion_eligible": False,
        "official_score_available": False,
        "run_id": run_id,
        "selection_reason": selection_reason,
        "baseline_sha256": baseline_sha256,
        "genome_sha256": genome.sha256,
        "genome": {
            "schema_version": genome.schema_version,
            "hysteresis_delta_u": genome.hysteresis_delta_u,
            "minimum_dwell_ns": genome.minimum_dwell_ns,
            "weights": [
                {
                    "role": role.name,
                    "feature": feature,
                    "weight": weight,
                }
                for role, feature, weight in genome.weights
            ],
        },
    }


def load_development_policy(path: str | Path) -> tuple[TacticalGenome, dict[str, Any]]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schema") != SCHEMA:
        raise ValueError("unsupported development policy schema")
    if (
        payload.get("evidence_class") != EVIDENCE_CLASS
        or payload.get("promotion_eligible") is not False
        or payload.get("official_score_available") is not False
    ):
        raise ValueError("policy artifact does not carry the required fixture-only boundary")
    raw = payload.get("genome")
    if not isinstance(raw, dict) or not isinstance(raw.get("weights"), list):
        raise ValueError("policy genome payload is malformed")
    weights = []
    for item in raw["weights"]:
        if not isinstance(item, dict):
            raise ValueError("policy weight entry must be an object")
        try:
            role = Role[item["role"]]
            feature = item["feature"]
            weight = item["weight"]
        except (KeyError, TypeError) as error:
            raise ValueError("policy weight entry lacks role, feature, or weight") from error
        if (
            not isinstance(feature, str)
            or isinstance(weight, bool)
            or not isinstance(weight, (int, float))
            or not math.isfinite(weight)
        ):
            raise ValueError("policy weight entry is invalid")
        weights.append((role, feature, float(weight)))
    try:
        genome = TacticalGenome(
            tuple(weights),
            raw["hysteresis_delta_u"],
            raw["minimum_dwell_ns"],
            raw["schema_version"],
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("policy genome violates tactical genome contracts") from error
    if payload.get("genome_sha256") != genome.sha256:
        raise ValueError("policy genome digest does not match its serialized values")
    if not _is_sha256(payload.get("baseline_sha256", "")):
        raise ValueError("policy baseline digest is invalid")
    return genome, payload


def canonical_json(payload: dict[str, Any]) -> str:
    return json.dumps(
        payload,
        sort_keys=True,
        indent=2,
        allow_nan=False,
    ) + "\n"


def _is_sha256(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(char in "0123456789abcdef" for char in value)
    )
