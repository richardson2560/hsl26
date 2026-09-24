# ros_ws/src/hsl_safety/test/test_adapters.py
# ros_ws/src/hsl_safety/test/test_adapters.py
"""Strict revision-2 adapter and cache tests."""

from types import SimpleNamespace

import pytest

from hsl_safety.adapters import (
    AdapterContext,
    ValidatedCache,
    decode_safety_snapshot,
    validate_authority_context,
)
from fixtures import DeterministicMockScenario, MockOutputSink


def _context(scenario, option="option-a"):
    return AdapterContext(
        now_ros_ns=scenario.clock.ros_ns,
        now_steady_ns=scenario.clock.steady_ns,
        expected_clock_epoch=scenario.clock_epoch,
        expected_localization_epoch=scenario.localization_epoch,
        expected_candidate_frame_id="base_link",
        expected_ego_frame_id="odom",
        expected_obstacle_frame_id="odom",
        expected_option_instance_id=option,
    )


def _records(scenario, option="option-a", complete=True):
    candidate = SimpleNamespace(
        meta=scenario.header(frame_id="base_link", lease_ns=200_000_000),
        option_instance_id=option,
        v=0.3,
        omega=0.0,
    )
    ego_meta = scenario.header(frame_id="odom", lease_ns=200_000_000)
    ego_meta.state_stamp = ego_meta.publication_stamp
    ego = SimpleNamespace(meta=ego_meta, v=0.0, omega=0.0)
    obstacle_meta = scenario.header(frame_id="odom", lease_ns=200_000_000)
    obstacle = SimpleNamespace(meta=obstacle_meta, complete=complete)
    return candidate, ego, obstacle


def test_decode_preserves_revision2_identity_and_exact_nanoseconds():
    scenario = DeterministicMockScenario()
    scenario.clock.advance(ros_ns=1_234_567_890, steady_ns=50_000)
    candidate, ego, obstacle = _records(scenario)
    result = decode_safety_snapshot(
        candidate,
        ego,
        obstacle,
        free_distance_m=0.8,
        context=_context(scenario),
    )
    assert result.candidate_stamp_ns == 1_234_567_890
    assert result.obstacle_stamp_ns == 1_234_567_890
    assert result.ego_stamp_ns == 1_234_567_890
    assert result.candidate_seq == 0
    assert result.received_steady_ns == 50_000
    assert result.option_instance_id == "option-a"


@pytest.mark.parametrize(
    "mutation",
    [
        lambda candidate, ego, obstacle: setattr(candidate.meta, "schema_version", 1),
        lambda candidate, ego, obstacle: setattr(candidate.meta, "validity", 0),
        lambda candidate, ego, obstacle: setattr(candidate.meta, "clock_epoch", "old"),
        lambda candidate, ego, obstacle: setattr(candidate.meta, "frame_id", "map"),
        lambda candidate, ego, obstacle: setattr(
            candidate.meta,
            "publication_stamp",
            SimpleNamespace(sec=2, nanosec=0),
        ),
        lambda candidate, ego, obstacle: setattr(obstacle, "complete", 1),
    ],
)
def test_decode_rejects_invalid_revision2_contract_before_use(mutation):
    scenario = DeterministicMockScenario()
    candidate, ego, obstacle = _records(scenario)
    mutation(candidate, ego, obstacle)
    with pytest.raises(ValueError):
        decode_safety_snapshot(
            candidate,
            ego,
            obstacle,
            free_distance_m=0.8,
            context=_context(scenario),
        )


def test_cache_does_not_replace_with_replayed_or_stale_records():
    scenario = DeterministicMockScenario()
    first, _, _ = _records(scenario)
    cache = ValidatedCache()
    cache.replace(first, observation_stamp_ns=0)
    original = cache.record
    with pytest.raises(ValueError, match="stale"):
        cache.replace(first, observation_stamp_ns=0)
    assert cache.record == original
    scenario.clock.advance(ros_ns=1, steady_ns=1)
    second, _, _ = _records(scenario)
    cache.replace(second, observation_stamp_ns=1)
    assert cache.seq > 0


def test_authority_context_requires_execution_match_and_watchdog_health():
    scenario = DeterministicMockScenario()
    meta = scenario.header(frame_id="", lease_ns=200_000_000)
    execution = SimpleNamespace(meta=meta, candidate_authorized=True)
    match = SimpleNamespace(meta=meta, motion_authorized=True)
    health = SimpleNamespace(meta=meta, ready=True, stop_latched=False)
    validate_authority_context(execution, match, health, context=_context(scenario))
    health.stop_latched = True
    with pytest.raises(ValueError, match="latched"):
        validate_authority_context(execution, match, health, context=_context(scenario))

    metadata_meta = scenario.header(frame_id="", lease_ns=200_000_000)
    validate_authority_context(
        SimpleNamespace(meta=metadata_meta, candidate_authorized=True),
        SimpleNamespace(meta=metadata_meta, motion_authorized=True),
        SimpleNamespace(meta=metadata_meta, ready=True, stop_latched=False),
        context=_context(scenario),
    )


def test_incomplete_obstacle_coverage_decodes_but_is_not_authorized():
    scenario = DeterministicMockScenario()
    candidate, ego, obstacle = _records(scenario, complete=False)
    result = decode_safety_snapshot(
        candidate,
        ego,
        obstacle,
        free_distance_m=0.8,
        context=_context(scenario),
    )
    assert result.coverage_valid is False


def test_decode_rejects_mixed_stage_identity():
    scenario = DeterministicMockScenario()
    candidate, ego, obstacle = _records(scenario)
    ego.meta.stage_id = "other-stage"
    with pytest.raises(ValueError, match="stage identities"):
        decode_safety_snapshot(
            candidate,
            ego,
            obstacle,
            free_distance_m=0.8,
            context=_context(scenario),
        )


def test_deterministic_scenario_controls_all_phase1_sources_and_sinks():
    scenario = DeterministicMockScenario()
    candidate = scenario.candidate(
        option_instance_id="option-a", v=0.2, omega=0.0, lease_ns=100
    )
    ego = scenario.ego_local(v=0.0, omega=0.0, lease_ns=100)
    obstacles = scenario.obstacles(complete=True, lease_ns=100)
    match = scenario.match(motion_authorized=True, lease_ns=100)
    execution = scenario.execution(candidate_authorized=True, lease_ns=100)
    health = scenario.watchdog_health(ready=True, stop_latched=False, lease_ns=100)
    assert candidate.meta.seq < ego.meta.seq < obstacles.meta.seq
    assert match.meta.frame_id == ""
    assert execution.meta.frame_id == ""
    assert health.meta.frame_id == ""

    sink = MockOutputSink()
    command = SimpleNamespace(v=0.0, omega=0.0)
    sink.publish_autonomy(command)
    sink.publish_watchdog_stop(command)
    sink.observe_mux(command)
    command.v = 1.0
    assert sink.autonomy.messages[0].v == 0.0
    assert len(sink.watchdog_stop.messages) == 1
    assert len(sink.mux_observation.messages) == 1
