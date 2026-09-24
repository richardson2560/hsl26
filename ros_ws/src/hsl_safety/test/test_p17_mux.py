# ros_ws/src/hsl_safety/test/test_p17_mux.py
"""Adversarial, ROS-free P1.7 mux and authority tests."""

from pathlib import Path

import pytest

from hsl_safety.mux_contract import DeterministicMux, MuxInput
from tools.check_ros_graph_authority import unauthorized_publishers, validate_graph


def _input(name, priority, received, timeout, linear=0.0, angular=0.0):
    return MuxInput(name, priority, received, timeout, linear, angular)


def test_cold_start_is_zero_only():
    mux = DeterministicMux()
    assert mux.decide(now_steady_ns=0).stop_asserted
    assert mux.decide(now_steady_ns=0).linear_x_mps == 0.0


def test_stop_overrides_teleop_and_autonomy():
    mux = DeterministicMux()
    mux.receive(_input("autonomous_supervisor", 50, 1000, 150, linear=0.4))
    mux.receive(_input("emergency_teleop", 100, 1000, 200, linear=0.2))
    mux.receive(_input("watchdog_stop", 200, 1000, 100))
    decision = mux.decide(now_steady_ns=1050)
    assert decision.source == "watchdog_stop"
    assert decision.stop_asserted
    assert decision.linear_x_mps == 0.0


def test_autonomy_requires_freshness_and_never_replays_after_expiry():
    mux = DeterministicMux()
    mux.receive(_input("autonomous_supervisor", 50, 1000, 150, linear=0.4))
    assert mux.decide(now_steady_ns=1150).linear_x_mps == 0.4
    expired = mux.decide(now_steady_ns=1151)
    assert expired.source == "idle"
    assert expired.linear_x_mps == 0.0


def test_watchdog_input_is_zero_only():
    mux = DeterministicMux()
    with pytest.raises(ValueError, match="zero-only"):
        mux.receive(_input("watchdog_stop", 200, 1, 100, linear=0.1))


def test_graph_requires_exactly_one_mux_physical_publisher():
    validate_graph([
        {"node": "/cmd_vel_mux", "topic": "/commands/velocity"},
    ])
    with pytest.raises(ValueError):
        validate_graph([
            {"node": "/cmd_vel_mux", "topic": "/commands/velocity"},
            {"node": "/legacy_driver", "topic": "/commands/velocity"},
        ])
    with pytest.raises(ValueError):
        validate_graph([])
    assert unauthorized_publishers([
        {"node": "/legacy_driver", "topic": "/commands/velocity"},
    ])


def test_mux_configuration_has_required_priority_and_topic_contract():
    config = Path(
        "kobuki/workspace/src/cmd_vel_mux/config/cmd_vel_mux_params.yaml"
    ).read_text(encoding="utf-8")
    expected = (
        ('topic: "/hsl/cmd_vel_stop"', "priority: 200", "timeout: 0.10"),
        ('topic: "/teleop/cmd_vel"', "priority: 100", "timeout: 0.2"),
        ('topic: "/hsl/cmd_vel_final"', "priority: 50", "timeout: 0.15"),
    )
    positions = []
    for topic, priority, timeout in expected:
        position = config.index(topic)
        assert priority in config[position:position + 180]
        assert timeout in config[position:position + 180]
        positions.append(position)
    assert positions == sorted(positions)
    assert 'output: "/commands/velocity"' in config
