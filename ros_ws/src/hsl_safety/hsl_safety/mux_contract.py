# ros_ws/src/hsl_safety/hsl_safety/mux_contract.py
"""ROS-free command-mux contract and deterministic arbitration model."""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class MuxInput:
    """One received command with receiver-local freshness information."""

    name: str
    priority: int
    received_steady_ns: int
    timeout_ns: int
    linear_x_mps: float
    angular_z_rps: float


@dataclass(frozen=True)
class MuxDecision:
    """The only command and source that may be sent to the physical output."""

    source: str
    linear_x_mps: float
    angular_z_rps: float
    stop_asserted: bool


class DeterministicMux:
    """Select the freshest highest-priority input, otherwise output zero."""

    def __init__(self, *, physical_output: str = "/commands/velocity") -> None:
        if not physical_output:
            raise ValueError("physical_output must be non-empty")
        self.physical_output = physical_output
        self._inputs: Dict[str, MuxInput] = {}

    def receive(self, command: MuxInput) -> None:
        if (
            not command.name
            or command.priority < 0
            or command.received_steady_ns < 0
            or command.timeout_ns <= 0
            or not math.isfinite(command.linear_x_mps)
            or not math.isfinite(command.angular_z_rps)
        ):
            raise ValueError("invalid mux input contract")
        if command.name == "watchdog_stop" and (
            command.linear_x_mps != 0.0 or command.angular_z_rps != 0.0
        ):
            raise ValueError("watchdog_stop must be zero-only")
        self._inputs[command.name] = command

    def decide(self, *, now_steady_ns: int) -> MuxDecision:
        if now_steady_ns < 0:
            raise ValueError("now_steady_ns must be non-negative")
        fresh = [
            command for command in self._inputs.values()
            if 0 <= now_steady_ns - command.received_steady_ns <= command.timeout_ns
        ]
        if not fresh:
            return MuxDecision("idle", 0.0, 0.0, True)
        selected = max(fresh, key=lambda command: command.priority)
        return MuxDecision(
            selected.name,
            selected.linear_x_mps,
            selected.angular_z_rps,
            selected.name == "watchdog_stop",
        )


MUX_INPUT_CONTRACT: Tuple[Tuple[str, str, int, int], ...] = (
    ("watchdog_stop", "/hsl/cmd_vel_stop", 200, 100_000_000),
    ("emergency_teleop", "/teleop/cmd_vel", 100, 200_000_000),
    ("autonomous_supervisor", "/hsl/cmd_vel_final", 50, 150_000_000),
)


def configured_mux_names() -> Tuple[str, ...]:
    """Return the canonical names in descending safety priority."""

    return tuple(item[0] for item in MUX_INPUT_CONTRACT)
