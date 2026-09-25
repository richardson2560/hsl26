# sim/kinematic/trace.py
"""Canonical trace recording for deterministic replay evidence."""

from dataclasses import dataclass
import hashlib
import json
from typing import Any

from .common import Actuation, SensorObservation


@dataclass
class DeterministicTrace:
    events: list[dict[str, Any]]

    def __init__(self) -> None:
        self.events = []

    def record_command(self, stamp_s: float, command: Actuation) -> None:
        self.events.append({
            "kind": "command",
            "stamp_s": float(stamp_s),
            "linear_mps": float(command.linear_mps),
            "angular_rps": float(command.angular_rps),
        })

    def record_observation(self, observation: SensorObservation) -> None:
        self.events.append({
            "kind": "observation",
            "stamp_s": float(observation.stamp_s),
            "frame_id": observation.frame_id,
            "ranges_m": list(observation.ranges_m),
            "valid_mask": list(observation.valid_mask),
        })

    def digest(self) -> str:
        encoded = json.dumps(
            self.events, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def to_json(self) -> str:
        return json.dumps(self.events, sort_keys=True, separators=(",", ":"))
