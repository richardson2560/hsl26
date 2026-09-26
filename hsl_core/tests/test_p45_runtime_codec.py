# hsl_core/tests/test_p45_runtime_codec.py
"""Strict contract and bounded-worker tests for Phase 4.5 runtime integration."""

import threading
import time
from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pytest
from tools.validate_gpis_prior import validate as validate_gpis_artifact

from hsl_core.perception.bounded_worker import BoundedWorker, WorkerBusyError
from hsl_core.perception.ekf_opponent import FilterConfig, OpponentDetection, OpponentFilter
from hsl_core.perception.fidelity import evaluate_profiles
from hsl_core.perception.implicit_surface import HermiteGPIS, HermiteObservation
from hsl_core.perception.runtime_codec import (
    ContractValidity,
    SensorProfile,
    TrackEnvelope,
    decode_belief,
    decode_track,
    decode_ros_belief,
    decode_ros_track,
    encode_belief,
    encode_track,
    populate_ros_belief,
    populate_ros_track,
    track_from_filter_output,
)
from hsl_core.perception.topological_belief import BeliefCell, TopologicalBelief
from hsl_core.topology import NodeKind, TopologyEdge, TopologyGraph, TopologyNode
from hsl_core.types import TrackState


def _track():
    return TrackEnvelope(
        track_id="opponent",
        state=TrackState.TRACKED,
        x=1.0,
        y=-2.0,
        vx=0.2,
        vy=0.1,
        covariance=tuple(np.eye(4).reshape(-1)),
        position_valid=True,
        yaw=0.0,
        yaw_variance=0.0,
        yaw_valid=False,
        last_measurement_stamp=1.25,
        sensor_profile=SensorProfile.SYNTHETIC_DETECTION,
        model_id="fixture-model",
        association_status="ACCEPTED",
        source_id="tracker",
        source_session="session-1",
        seq=7,
        stage_id="stage-1",
        clock_epoch="clock-1",
        localization_epoch="loc-1",
        frame_id="map",
        observation_stamp=1.25,
        state_stamp=1.3,
        publication_stamp=1.31,
        valid_until=1.5,
        map_version=4,
        topology_version=9,
        validity=ContractValidity.VALID,
    )


def _belief():
    graph = TopologyGraph(
        4,
        9,
        "loc-1",
        (
            TopologyNode(0, 0.0, 0.0, NodeKind.ANCHOR, 1.0),
            TopologyNode(1, 2.0, 0.0, NodeKind.DEAD_END, 1.0),
        ),
        (TopologyEdge(42, 0, 1, ((0.0, 0.0), (2.0, 0.0)), 0.5),),
    )
    return TopologicalBelief(
        graph,
        (BeliefCell(42, 0.0, 1.0, 0.2, 0.7),),
        0.3,
        1.0,
        1.2,
        0.5,
    )


def test_track_codec_round_trip_preserves_all_contract_fields():
    encoded = encode_track(_track())
    decoded = decode_track(encoded)
    assert decoded == _track()
    assert encoded["meta"]["schema_version"] == 2
    assert encoded["meta"]["map_version"] == 4
    assert encoded["meta"]["topology_version"] == 9
    assert encoded["yaw"] == 0.0
    assert encoded["yaw_valid"] is False


def test_filter_output_adapter_does_not_fabricate_yaw_or_measurement_stamp():
    tracker = OpponentFilter(FilterConfig(confirmation_count=1))
    output = tracker.initialize(
        OpponentDetection(
            (0.4, -0.2), (0.01, 0.0, 0.0, 0.01), 2.0,
            "synthetic", 4, "loc-1",
        )
    )
    envelope = track_from_filter_output(
        output,
        track_id="opponent",
        sensor_profile=SensorProfile.SYNTHETIC_DETECTION,
        model_id="synthetic-bypass-v1",
        source_id="tracker",
        source_session="test-session",
        seq=1,
        stage_id="stage-1",
        clock_epoch="clock-1",
        localization_epoch="loc-1",
        frame_id="map",
        map_version=4,
        topology_version=9,
        publication_stamp=2.01,
        valid_until=2.2,
        validity=ContractValidity.DEGRADED,
    )
    assert envelope.position_valid
    assert envelope.yaw_valid is False
    assert envelope.yaw == 0.0
    assert envelope.last_measurement_stamp == 2.0
    assert envelope.sensor_profile == SensorProfile.SYNTHETIC_DETECTION


def test_track_codec_rejects_unknown_schema_missing_fields_and_bad_stamp():
    encoded = encode_track(_track())
    encoded["extra"] = 1
    with pytest.raises(ValueError):
        decode_track(encoded)
    encoded = encode_track(_track())
    encoded["meta"]["schema_version"] = 1
    with pytest.raises(ValueError):
        decode_track(encoded)
    encoded = encode_track(_track())
    encoded["meta"]["observation_stamp"]["nanosec"] = 1_000_000_000
    with pytest.raises(ValueError):
        decode_track(encoded)


def _ros_header():
    return SimpleNamespace(
        schema_version=0,
        source_id="",
        source_session="",
        seq=0,
        stage_id="",
        clock_epoch="",
        localization_epoch="",
        frame_id="",
        observation_stamp=SimpleNamespace(sec=0, nanosec=0),
        state_stamp=SimpleNamespace(sec=0, nanosec=0),
        publication_stamp=SimpleNamespace(sec=0, nanosec=0),
        valid_until=SimpleNamespace(sec=0, nanosec=0),
        map_version=0,
        topology_version=0,
        validity=0,
    )


def test_generated_ros_track_object_round_trip_without_ros_installation():
    message = SimpleNamespace(
        meta=_ros_header(),
        last_measurement_stamp=SimpleNamespace(sec=0, nanosec=0),
    )
    populate_ros_track(message, _track())
    assert decode_ros_track(message) == _track()


def test_generated_ros_belief_object_round_trip_without_ros_installation():
    message = SimpleNamespace(
        meta=_ros_header(),
        last_measurement_stamp=SimpleNamespace(sec=0, nanosec=0),
        cells=[],
    )

    class CellMessage:
        pass

    populate_ros_belief(
        message,
        _belief(),
        belief_cell_factory=CellMessage,
        track_id="opponent",
        transition_model_id="transition-v1",
        visibility_model_id="visibility-v1",
        sensor_profile=SensorProfile.SYNTHETIC_DETECTION,
        source_id="belief-node",
        source_session="session-1",
        seq=11,
        stage_id="stage-1",
        clock_epoch="clock-1",
        frame_id="map",
        publication_stamp=1.21,
        valid_until=1.5,
        validity=ContractValidity.DEGRADED,
    )
    decoded = decode_ros_belief(message)
    assert decoded.cells == _belief().cells
    assert decoded.unknown_mass == pytest.approx(0.3)
    assert decoded.topology_version == 9


@pytest.mark.parametrize(
    "changes",
    [
        {"yaw": 1.0},
        {"valid_until": 1.0},
        {"last_measurement_stamp": 1.4},
        {"state": TrackState.LOST},
        {"position_valid": 1},
        {"covariance": tuple(np.diag([1.0, -1.0, 1.0, 1.0]).reshape(-1))},
    ],
)
def test_track_envelope_rejects_semantically_inconsistent_payloads(changes):
    values = _track().__dict__ | changes
    with pytest.raises(ValueError):
        TrackEnvelope(**values)


def test_belief_codec_round_trip_preserves_mass_versions_and_fidelity():
    belief = _belief()
    encoded = encode_belief(
        belief,
        track_id="opponent",
        transition_model_id="cv-reachability-v1",
        visibility_model_id="coverage-pd-v1",
        sensor_profile=SensorProfile.SYNTHETIC_DETECTION,
        source_id="belief-node",
        source_session="session-1",
        seq=11,
        stage_id="stage-1",
        clock_epoch="clock-1",
        frame_id="map",
        publication_stamp=1.21,
        valid_until=1.5,
        validity=ContractValidity.DEGRADED,
    )
    decoded = decode_belief(encoded)
    assert decoded.cells == belief.cells
    assert decoded.unknown_mass == pytest.approx(0.3)
    assert decoded.topology_version == 9
    assert decoded.sensor_profile == SensorProfile.SYNTHETIC_DETECTION
    assert decoded.validity == ContractValidity.DEGRADED


def test_belief_codec_rejects_bad_mass_cell_and_metadata():
    encoded = encode_belief(
        _belief(), track_id="opponent", transition_model_id="t1",
        visibility_model_id="v1", sensor_profile=SensorProfile.REPLAY_CLOUD,
        source_id="s", source_session="ss", seq=1, stage_id="st",
        clock_epoch="c", frame_id="map", publication_stamp=1.2,
        valid_until=1.5, validity=ContractValidity.VALID,
    )
    encoded["unknown_mass"] = 0.8
    with pytest.raises(ValueError):
        decode_belief(encoded)
    encoded["unknown_mass"] = 0.3
    encoded["cells"][0]["s_end_m"] = -1.0
    with pytest.raises(ValueError):
        decode_belief(encoded)


def test_static_idl_declares_belief_contract_and_cmake_registration():
    root = Path(__file__).resolve().parents[2]
    package = root / "ros_ws" / "src" / "hsl_interfaces"
    belief_fields = (package / "msg" / "OpponentBelief.msg").read_text(encoding="utf-8")
    cell_fields = (package / "msg" / "BeliefCell.msg").read_text(encoding="utf-8")
    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "hsl_interfaces/ContractHeader meta" in belief_fields
    assert "hsl_interfaces/BeliefCell[] cells" in belief_fields
    assert "float64 unknown_mass" in belief_fields
    assert "float64 mass" in cell_fields
    assert '"msg/OpponentBelief.msg"' in cmake
    assert '"msg/BeliefCell.msg"' in cmake


def test_p45_model_validator_checks_normative_kernel_shapes_and_hash(tmp_path: Path):
    model = HermiteGPIS(
        [
            HermiteObservation((0.0, 0.0, 0.0), "value", (1.0, 0.0, 0.0), 0.0, 1e-4),
            HermiteObservation((0.2, 0.0, 0.0), "derivative", (1.0, 0.0, 0.0), 1.0, 1e-4),
        ],
        support_radius_m=1.0,
        regularization=1e-8,
    )
    manifest = model.save(tmp_path)
    assert validate_gpis_artifact(tmp_path)["status"] == "PASS"
    manifest["kernel_id"] = "wendland_c2_d3_unit_center_v1"
    (tmp_path / "manifest.json").write_text(
        __import__("json").dumps(manifest), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="kernel_id"):
        validate_gpis_artifact(tmp_path)


class _FakeClock:
    def __init__(self):
        self.value = 10.0

    def __call__(self):
        return self.value


def test_worker_returns_completion_and_surfaces_task_exception():
    clock = _FakeClock()
    worker = BoundedWorker(timeout_s=1.0, clock=clock)
    try:
        request = worker.submit(lambda value: value * 2, 3, map_version=1, topology_version=2, localization_epoch="e")
        deadline = time.monotonic() + 2.0
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(map_version=1, topology_version=2, localization_epoch="e")
            time.sleep(0.001)
        assert result is not None
        assert result.request_id == request
        assert result.status == "COMPLETED"
        assert result.value == 6

        def fail(_):
            raise RuntimeError("visible failure")

        worker.submit(fail, None, map_version=1, topology_version=2, localization_epoch="e")
        result = None
        while result is None and time.monotonic() < deadline + 2.0:
            result = worker.poll(map_version=1, topology_version=2, localization_epoch="e")
            time.sleep(0.001)
        assert result is not None
        assert result.status == "FAILED"
        assert "visible failure" in result.reason
    finally:
        worker.close()


def test_worker_times_out_without_spawning_unbounded_replacements():
    clock = _FakeClock()
    started = threading.Event()
    release = threading.Event()
    worker = BoundedWorker(timeout_s=0.5, clock=clock)
    try:
        worker.submit(
            lambda _: (started.set(), release.wait(2.0), "late")[-1],
            None,
            map_version=1,
            topology_version=2,
            localization_epoch="e",
        )
        assert started.wait(1.0)
        clock.value += 0.5
        timed_out = worker.poll(map_version=1, topology_version=2, localization_epoch="e")
        assert timed_out.status == "TIMEOUT"
        with pytest.raises(WorkerBusyError):
            worker.submit(lambda _: "second", None, map_version=1, topology_version=2, localization_epoch="e")
        release.set()
    finally:
        release.set()
        worker.close()


def test_worker_discards_results_after_version_or_generation_change():
    clock = _FakeClock()
    started = threading.Event()
    release = threading.Event()
    worker = BoundedWorker(timeout_s=1.0, clock=clock)
    try:
        worker.submit(
            lambda _: (started.set(), release.wait(1.0), "computed")[-1],
            None,
            map_version=1,
            topology_version=2,
            localization_epoch="e",
        )
        assert started.wait(1.0)
        stale = worker.poll(map_version=2, topology_version=2, localization_epoch="e")
        assert stale.status == "DISCARDED"
        assert stale.value is None

        release.set()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            try:
                worker.submit(lambda _: "computed", None, map_version=1, topology_version=2, localization_epoch="e")
                break
            except WorkerBusyError:
                time.sleep(0.001)
        else:
            pytest.fail("worker did not release completed stale task")
        event = threading.Event()
        worker.close()
        worker = BoundedWorker(timeout_s=1.0, clock=clock)
        worker.submit(
            lambda _: (event.set(), "computed")[-1],
            None,
            map_version=1,
            topology_version=2,
            localization_epoch="e",
        )
        assert event.wait(1.0)
        worker.invalidate()
        deadline = time.monotonic() + 1.0
        stale_generation = None
        while stale_generation is None and time.monotonic() < deadline:
            stale_generation = worker.poll(map_version=1, topology_version=2, localization_epoch="e")
            if stale_generation is None:
                time.sleep(0.001)
        assert stale_generation is not None
        assert stale_generation.status == "DISCARDED"
    finally:
        release.set()
        worker.close()


def test_profile_evaluator_reports_metrics_without_promoting_insufficient_data():
    cases = [
        {
            "case_id": f"case-{index}",
            "truth_xy_m": [float(index), 0.0],
            "real_cloud": {
                "position_xy_m": [float(index) + 0.1, 0.0],
                "covariance_xy_m2": [0.04, 0.0, 0.0, 0.04],
                "latency_ms": 10.0 + index,
            },
            "mvsim_cloud": None,
            "synthetic_detection": {
                "position_xy_m": [float(index) + 0.2, 0.0],
                "covariance_xy_m2": [0.04, 0.0, 0.0, 0.04],
                "latency_ms": 2.0,
            },
        }
        for index in range(3)
    ]
    report = evaluate_profiles(cases, minimum_labeled_samples=4)
    assert report["status"] == "BLOCKED_INSUFFICIENT_LABELED_DATA"
    assert report["evaluation_only"] is True
    assert report["profiles"]["real_cloud"]["position_error_rmse_m"] == pytest.approx(0.1)
    assert report["profiles"]["mvsim_cloud"]["misses"] == 3
    assert report["paired_rmse_delta_real_minus_synthetic_m"] == pytest.approx(-0.1)


def test_profile_evaluator_handles_false_positives_and_rejects_bad_case_ids():
    sample = {
        "case_id": "negative",
        "truth_xy_m": None,
        "real_cloud": {"position_xy_m": [0.0, 0.0], "latency_ms": 2.0},
        "mvsim_cloud": None,
        "synthetic_detection": None,
    }
    report = evaluate_profiles([sample], minimum_labeled_samples=1)
    assert report["profiles"]["real_cloud"]["false_positives"] == 1
    assert report["profiles"]["synthetic_detection"]["false_positives"] == 0
    with pytest.raises(ValueError):
        evaluate_profiles([sample, sample])
