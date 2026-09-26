# ros_ws/src/hsl_safety/hsl_safety/adapters.py
"""Revision-2 safety-boundary adapters without ROS runtime imports.

The functions accept generated-message-shaped objects so contract tests can
run without a sourced ROS environment. They never create timestamps, mutate
messages, or replace a cache on a rejected record.
"""

from copy import deepcopy
from dataclasses import dataclass
import math
from typing import Any

from hsl_core.control.safety import SafetySnapshot
from hsl_core.control.safety import SafetyEvaluation
from hsl_core.contracts import validate_covariance
from hsl_core.types import EgoState, Pose2D, Twist2D


@dataclass(frozen=True)
class AdapterContext:
    """Expected identities and receiver-local times for one decode operation."""

    now_ros_ns: int
    now_steady_ns: int
    expected_stage_id: str
    expected_clock_epoch: str
    expected_localization_epoch: str
    expected_candidate_frame_id: str
    expected_ego_frame_id: str
    expected_obstacle_frame_id: str
    expected_map_version: int
    expected_topology_version: int
    expected_lease_generation: int
    expected_option_instance_id: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.expected_stage_id, str) or not self.expected_stage_id.strip():
            raise ValueError("expected_stage_id must be non-empty")
        for value, name in (
            (self.expected_map_version, "expected_map_version"),
            (self.expected_topology_version, "expected_topology_version"),
            (self.expected_lease_generation, "expected_lease_generation"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if self.expected_lease_generation == 0:
            raise ValueError("expected_lease_generation must be positive")


@dataclass
class ValidatedCache:
    """Replace-on-success cache that rejects replayed sequence/time pairs."""

    record: Any = None
    source_session: str = ""
    seq: int = -1
    observation_stamp_ns: int = -1

    def replace(self, message: Any, *, observation_stamp_ns: int) -> None:
        meta = _meta(message)
        source_session = _required(meta, "source_session")
        seq = _required(meta, "seq")
        if not isinstance(source_session, str) or not source_session:
            raise ValueError("source_session must be non-empty")
        if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
            raise ValueError("seq must be a non-negative integer")
        if not isinstance(observation_stamp_ns, int) or observation_stamp_ns < 0:
            raise ValueError("observation_stamp_ns must be non-negative")
        if self.record is not None and source_session == self.source_session:
            if seq <= self.seq or observation_stamp_ns <= self.observation_stamp_ns:
                raise ValueError("stale or replayed record")
        self.record = deepcopy(message)
        self.source_session = source_session
        self.seq = seq
        self.observation_stamp_ns = observation_stamp_ns


@dataclass(frozen=True)
class StaleDiagnostic:
    """Non-authoritative diagnostic re-emission preserving source provenance."""

    message: Any
    observation_stamp_ns: int
    publication_stamp_ns: int
    valid_until_ns: int
    authoritative: bool
    reason: str


def republish_stale_diagnostic(message: Any, *, reason: str = "STALE_OBSERVATION") -> StaleDiagnostic:
    """Prepare a stale record for diagnostics without changing its lease."""

    if not isinstance(reason, str) or not reason:
        raise ValueError("stale diagnostic reason must be non-empty")
    meta = _meta(message)
    observation_stamp_ns = _stamp_ns(_required(meta, "observation_stamp"), "observation_stamp")
    publication_stamp_ns = _stamp_ns(_required(meta, "publication_stamp"), "publication_stamp")
    valid_until_ns = _stamp_ns(_required(meta, "valid_until"), "valid_until")
    return StaleDiagnostic(
        message=deepcopy(message),
        observation_stamp_ns=observation_stamp_ns,
        publication_stamp_ns=publication_stamp_ns,
        valid_until_ns=valid_until_ns,
        authoritative=False,
        reason=reason,
    )


def encode_ego_state_message(
    state: EgoState,
    message: Any,
    *,
    meta: Any,
    twist_covariance: tuple[float, ...],
    calibration_id: str,
    localization_valid: bool = True,
) -> Any:
    """Encode all revision-2 EgoState fields without manufacturing metadata."""

    if not isinstance(localization_valid, bool) or not localization_valid:
        raise ValueError("encoded ego localization must be valid")
    if not isinstance(calibration_id, str) or not calibration_id:
        raise ValueError("calibration_id must be non-empty")
    twist_covariance = validate_covariance(tuple(twist_covariance), 2)
    message.meta = deepcopy(meta)
    message.pose.x = state.pose.x_m
    message.pose.y = state.pose.y_m
    message.pose.theta = state.pose.theta_rad
    message.v = state.twist.linear_mps
    message.omega = state.twist.angular_rps
    message.pose_covariance = list(state.pose_covariance)
    message.twist_covariance = list(twist_covariance)
    message.healthy = state.healthy
    message.slip = state.slip
    message.localization_valid = localization_valid
    message.calibration_id = calibration_id
    return message


def decode_ego_state_message(message: Any, *, context: AdapterContext) -> EgoState:
    """Decode an exact revision-2 EgoState without creating provenance."""

    meta = _meta(message)
    _validate_meta(
        meta,
        context,
        require_frame=True,
        expected_frame_id=context.expected_ego_frame_id,
    )
    pose = _required(message, "pose")
    pose_covariance = tuple(float(value) for value in _required(message, "pose_covariance"))
    twist_covariance = tuple(float(value) for value in _required(message, "twist_covariance"))
    validate_covariance(pose_covariance, 3)
    validate_covariance(twist_covariance, 2)
    if not isinstance(_required(message, "healthy"), bool) or not isinstance(_required(message, "slip"), bool):
        raise ValueError("ego health flags must be boolean")
    if not isinstance(_required(message, "localization_valid"), bool):
        raise ValueError("localization_valid must be boolean")
    if not _required(message, "localization_valid"):
        raise ValueError("ego localization is invalid")
    calibration_id = _required(message, "calibration_id")
    if not isinstance(calibration_id, str) or not calibration_id:
        raise ValueError("calibration_id must be non-empty")
    twist = Twist2D(float(_required(message, "v")), float(_required(message, "omega")))
    state = EgoState(
        pose=Pose2D(
            float(_required(pose, "x")),
            float(_required(pose, "y")),
            float(_required(pose, "theta")),
        ),
        twist=twist,
        pose_covariance=pose_covariance,
        localization_epoch=_required(meta, "localization_epoch"),
        healthy=_required(message, "healthy"),
        slip=_required(message, "slip"),
        observation_time_s=_stamp_ns(
            _required(meta, "observation_stamp"), "observation_stamp"
        ) / 1_000_000_000.0,
        frame_id=_required(meta, "frame_id"),
    )
    return state


def validate_obstacle_payload(
    message: Any,
    *,
    context: AdapterContext,
    max_cells: int = 200_000,
    max_obstacles: int = 512,
) -> None:
    """Validate bounded obstacle/coverage payloads before safety use."""

    meta = _meta(message)
    _validate_meta(
        meta,
        context,
        require_frame=True,
        expected_frame_id=context.expected_obstacle_frame_id,
    )
    coverage = _required(message, "coverage")
    width = _required(coverage, "width")
    height = _required(coverage, "height")
    resolution = _required(coverage, "resolution_m")
    origin = _required(coverage, "origin")
    cells = _required(coverage, "cells")
    observed_stamps = _required(coverage, "observed_stamps")
    obstacles = _required(message, "obstacles")
    snapshot_stamp_ns = _stamp_ns(_required(meta, "observation_stamp"), "observation_stamp")
    if (
        not isinstance(width, int) or isinstance(width, bool) or width <= 0
        or not isinstance(height, int) or isinstance(height, bool) or height <= 0
        or isinstance(resolution, bool) or not isinstance(resolution, (int, float))
        or not math.isfinite(float(resolution)) or resolution <= 0.0
        or not math.isfinite(float(_required(origin, "x")))
        or not math.isfinite(float(_required(origin, "y")))
        or len(cells) != width * height
        or len(observed_stamps) != width * height
        or width * height > max_cells
    ):
        raise ValueError("coverage dimensions and cells are inconsistent")
    if len(obstacles) > max_obstacles:
        raise ValueError("obstacle payload exceeds configured bound")
    if any(value not in (0, 1, 2) for value in cells):
        raise ValueError("coverage contains an invalid state")
    for stamp in observed_stamps:
        if _stamp_ns(stamp, "observed_stamp") > snapshot_stamp_ns:
            raise ValueError("observed_stamp cannot be newer than snapshot")
    for name in ("pose_error_bound_m", "map_error_bound_m"):
        value = _required(message, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
    if not isinstance(_required(message, "complete"), bool):
        raise ValueError("obstacle completeness must be boolean")
    if not isinstance(_required(message, "frontal_coverage_valid"), bool):
        raise ValueError("frontal coverage validity must be boolean")
    bounds_id = _required(message, "bounds_id")
    if not isinstance(bounds_id, str) or not bounds_id:
        raise ValueError("bounds_id must be non-empty")
    for obstacle in obstacles:
        obstacle_id = _required(obstacle, "obstacle_id")
        if not isinstance(obstacle_id, int) or isinstance(obstacle_id, bool) or obstacle_id < 0:
            raise ValueError("obstacle_id must be a non-negative integer")
        for name in ("vx", "vy", "speed_bound_mps", "position_error_bound_m"):
            value = _required(obstacle, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise ValueError(f"obstacle {name} must be finite")
        if obstacle.speed_bound_mps < 0.0 or obstacle.position_error_bound_m < 0.0:
            raise ValueError("obstacle bounds must be non-negative")
        last_observed_ns = _stamp_ns(_required(obstacle, "last_observed_stamp"), "last_observed_stamp")
        if last_observed_ns > snapshot_stamp_ns:
            raise ValueError("last_observed_stamp cannot be newer than snapshot")
        polygon = _required(obstacle, "polygon")
        vertices = _required(polygon, "vertices")
        if len(vertices) < 3:
            raise ValueError("obstacle polygon requires at least three vertices")
        for vertex in vertices:
            for coordinate in ("x", "y", "z"):
                value = _required(vertex, coordinate)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                    raise ValueError("obstacle polygon coordinates must be finite")
        for name in ("motion_estimate_valid", "is_opponent"):
            if not isinstance(_required(obstacle, name), bool):
                raise ValueError(f"obstacle {name} must be boolean")


def validate_observation_progress(
    previous: Any,
    current: Any,
    *,
    max_position_jump_m: float,
    max_time_gap_ns: int,
) -> None:
    """Reject dropout-sized gaps and discontinuous ego observations."""

    if max_position_jump_m <= 0.0 or max_time_gap_ns <= 0:
        raise ValueError("observation progress bounds must be positive")
    previous_stamp = _stamp_ns(_required(_meta(previous), "observation_stamp"), "previous observation_stamp")
    current_stamp = _stamp_ns(_required(_meta(current), "observation_stamp"), "current observation_stamp")
    if current_stamp <= previous_stamp or current_stamp - previous_stamp > max_time_gap_ns:
        raise ValueError("observation dropout or non-monotonic timestamp")
    previous_pose = _required(previous, "pose")
    current_pose = _required(current, "pose")
    distance = math.hypot(
        float(_required(current_pose, "x")) - float(_required(previous_pose, "x")),
        float(_required(current_pose, "y")) - float(_required(previous_pose, "y")),
    )
    if not math.isfinite(distance) or distance > max_position_jump_m:
        raise ValueError("ego observation jump exceeds configured bound")


def _set_time(stamp: Any, stamp_ns: int) -> None:
    stamp.sec = stamp_ns // 1_000_000_000
    stamp.nanosec = stamp_ns % 1_000_000_000


def encode_safety_status(
    evaluation: SafetyEvaluation,
    message: Any,
    *,
    meta: Any,
    mode: int,
    limits_id: str,
    evaluation_time_ns: int,
) -> Any:
    """Encode every SafetyStatus field without manufacturing authority metadata."""

    if not isinstance(limits_id, str) or not limits_id:
        raise ValueError("limits_id must be non-empty")
    if not isinstance(evaluation_time_ns, int) or evaluation_time_ns < 0:
        raise ValueError("evaluation_time_ns must be non-negative")
    if not isinstance(mode, int) or isinstance(mode, bool) or mode not in range(5):
        raise ValueError("mode must be a valid SafetyMode value")
    message.meta = deepcopy(meta)
    message.mode = mode
    message.decision = evaluation.decision
    message.primary_reason = evaluation.primary_reason
    message.reasons = list(evaluation.reasons)
    message.candidate_seq = evaluation.candidate_seq
    message.proposed_v = evaluation.proposed_v_mps
    message.proposed_omega = evaluation.proposed_omega_rps
    message.applied_v = evaluation.applied_v_mps
    message.applied_omega = evaluation.applied_omega_rps
    message.checked_clearance_m = evaluation.checked_clearance_m
    message.required_stop_distance_m = evaluation.required_stop_distance_m
    message.response_bound_s = evaluation.response_bound_s
    message.clearance_valid = evaluation.decision != 0
    message.limits_id = limits_id
    _set_time(message.evaluation_time, evaluation_time_ns)
    return message


def encode_supervisor_heartbeat(
    *,
    message: Any,
    meta: Any,
    decision_seq: int,
    mode: int,
    permit_motion: bool,
    config_hash: str,
    active_option_instance_id: str,
) -> Any:
    """Encode a completed decision heartbeat after the decision is published."""

    if not isinstance(decision_seq, int) or isinstance(decision_seq, bool) or decision_seq < 0:
        raise ValueError("decision_seq must be a non-negative integer")
    if not isinstance(config_hash, str) or not config_hash:
        raise ValueError("config_hash must be non-empty")
    if not isinstance(permit_motion, bool):
        raise ValueError("permit_motion must be boolean")
    message.meta = deepcopy(meta)
    message.decision_seq = decision_seq
    message.mode = mode
    message.permit_motion = permit_motion
    message.config_hash = config_hash
    message.active_option_instance_id = active_option_instance_id
    return message


def _required(message: Any, name: str) -> Any:
    if not hasattr(message, name):
        raise ValueError(f"revision-2 message missing field: {name}")
    return getattr(message, name)


def _stamp_ns(stamp: Any, name: str) -> int:
    sec = _required(stamp, "sec")
    nanosec = _required(stamp, "nanosec")
    if (
        not isinstance(sec, int)
        or isinstance(sec, bool)
        or not isinstance(nanosec, int)
        or isinstance(nanosec, bool)
        or nanosec < 0
        or nanosec >= 1_000_000_000
    ):
        raise ValueError(f"{name} must contain normalized integer time")
    if sec < 0:
        raise ValueError(f"{name} cannot be negative")
    return sec * 1_000_000_000 + nanosec


def _meta(message: Any) -> Any:
    return _required(message, "meta")


def _validate_meta(
    meta: Any,
    context: AdapterContext,
    *,
    require_frame: bool,
    expected_frame_id: str,
) -> None:
    if (
        not isinstance(_required(meta, "schema_version"), int)
        or isinstance(_required(meta, "schema_version"), bool)
        or _required(meta, "schema_version") != 2
    ):
        raise ValueError("schema_version must be revision 2")
    if _required(meta, "clock_epoch") != context.expected_clock_epoch:
        raise ValueError("clock_epoch mismatch")
    if _required(meta, "stage_id") != context.expected_stage_id:
        raise ValueError("stage_id mismatch")
    if _required(meta, "localization_epoch") != context.expected_localization_epoch:
        raise ValueError("localization_epoch mismatch")
    for name, expected in (
        ("map_version", context.expected_map_version),
        ("topology_version", context.expected_topology_version),
    ):
        version = _required(meta, name)
        if not isinstance(version, int) or isinstance(version, bool) or version < 0:
            raise ValueError(f"{name} must be a non-negative integer")
        if version != expected:
            raise ValueError(f"{name} mismatch")
    frame_id = _required(meta, "frame_id")
    if require_frame and frame_id != expected_frame_id:
        raise ValueError("frame_id mismatch")
    if not require_frame and frame_id not in ("", expected_frame_id):
        raise ValueError("metadata frame_id must be empty or expected")
    validity = _required(meta, "validity")
    if (
        not isinstance(validity, int)
        or isinstance(validity, bool)
        or validity not in (1, 2)
    ):
        raise ValueError("record validity is INVALID")
    publication_ns = _stamp_ns(_required(meta, "publication_stamp"), "publication_stamp")
    valid_until_ns = _stamp_ns(_required(meta, "valid_until"), "valid_until")
    if publication_ns > context.now_ros_ns:
        raise ValueError("publication_stamp is in the future")
    if context.now_ros_ns >= valid_until_ns:
        raise ValueError("record lease has expired")


def decode_safety_snapshot(
    candidate: Any,
    ego_local: Any,
    obstacles: Any,
    *,
    free_distance_m: float,
    context: AdapterContext,
) -> SafetySnapshot:
    """Decode a coherent supervisor snapshot from revision-2 records."""

    _validate_meta(
        _meta(candidate),
        context,
        require_frame=True,
        expected_frame_id=context.expected_candidate_frame_id,
    )
    _validate_meta(
        _meta(ego_local),
        context,
        require_frame=True,
        expected_frame_id=context.expected_ego_frame_id,
    )
    _validate_meta(
        _meta(obstacles),
        context,
        require_frame=True,
        expected_frame_id=context.expected_obstacle_frame_id,
    )
    candidate_meta = _meta(candidate)
    ego_meta = _meta(ego_local)
    obstacle_meta = _meta(obstacles)
    stage_ids = {
        _required(candidate_meta, "stage_id"),
        _required(ego_meta, "stage_id"),
        _required(obstacle_meta, "stage_id"),
    }
    if len(stage_ids) != 1 or "" in stage_ids:
        raise ValueError("candidate, ego and obstacle stage identities must match")
    candidate_generation = _required(candidate, "lease_generation")
    if (
        not isinstance(candidate_generation, int)
        or isinstance(candidate_generation, bool)
        or candidate_generation <= 0
    ):
        raise ValueError("candidate lease_generation must be positive")
    if candidate_generation != context.expected_lease_generation:
        raise ValueError("candidate lease_generation mismatch")
    candidate_stamp_ns = _stamp_ns(
        _required(candidate_meta, "publication_stamp"), "candidate publication_stamp"
    )
    candidate_valid_until_ns = _stamp_ns(
        _required(candidate_meta, "valid_until"), "candidate valid_until"
    )
    obstacle_stamp_ns = _stamp_ns(
        _required(obstacle_meta, "observation_stamp"), "obstacle observation_stamp"
    )
    ego_stamp_ns = _stamp_ns(
        _required(ego_meta, "state_stamp"), "ego state_stamp"
    )
    if (
        isinstance(free_distance_m, bool)
        or not isinstance(free_distance_m, (int, float))
        or not math.isfinite(float(free_distance_m))
        or free_distance_m < 0.0
    ):
        raise ValueError("free_distance_m must be finite and non-negative")
    if not isinstance(_required(obstacles, "complete"), bool):
        raise ValueError("obstacle completeness must be boolean")
    if not isinstance(_required(obstacles, "frontal_coverage_valid"), bool):
        raise ValueError("frontal coverage validity must be boolean")
    return SafetySnapshot(
        candidate_seq=_required(candidate_meta, "seq"),
        candidate_v_mps=_required(candidate, "v"),
        candidate_omega_rps=_required(candidate, "omega"),
        candidate_stamp_ns=candidate_stamp_ns,
        candidate_valid_until_ns=candidate_valid_until_ns,
        obstacle_stamp_ns=obstacle_stamp_ns,
        ego_stamp_ns=ego_stamp_ns,
        now_ros_ns=context.now_ros_ns,
        frame_id=_required(candidate_meta, "frame_id"),
        expected_frame_id=context.expected_candidate_frame_id,
        clock_epoch=_required(candidate_meta, "clock_epoch"),
        expected_clock_epoch=context.expected_clock_epoch,
        localization_epoch=_required(candidate_meta, "localization_epoch"),
        expected_localization_epoch=context.expected_localization_epoch,
        coverage_valid=(
            _required(obstacles, "complete")
            and _required(obstacles, "frontal_coverage_valid")
        ),
        free_distance_m=float(free_distance_m),
        ego_speed_mps=_required(ego_local, "v"),
        candidate_map_version=_required(candidate_meta, "map_version"),
        expected_map_version=context.expected_map_version,
        candidate_topology_version=_required(candidate_meta, "topology_version"),
        expected_topology_version=context.expected_topology_version,
        candidate_lease_generation=candidate_generation,
        expected_lease_generation=context.expected_lease_generation,
        option_instance_id=_required(candidate, "option_instance_id"),
        expected_option_instance_id=context.expected_option_instance_id,
        received_steady_ns=context.now_steady_ns,
    )


def validate_authority_context(
    execution_state: Any,
    match_state: Any,
    watchdog_health: Any,
    *,
    context: AdapterContext,
) -> None:
    """Reject authority inputs that cannot support a nonzero decision."""

    for message in (execution_state, match_state, watchdog_health):
        _validate_meta(
            _meta(message),
            context,
            require_frame=False,
            expected_frame_id=context.expected_candidate_frame_id,
        )
    candidate_authorized = _required(execution_state, "candidate_authorized")
    if not isinstance(candidate_authorized, bool):
        raise ValueError("candidate_authorized must be boolean")
    if not candidate_authorized:
        raise ValueError("candidate authority is not granted")
    lease_generation = _required(execution_state, "lease_generation")
    if (
        not isinstance(lease_generation, int)
        or isinstance(lease_generation, bool)
        or lease_generation <= 0
        or lease_generation != context.expected_lease_generation
    ):
        raise ValueError("execution lease_generation mismatch")
    phase = _required(execution_state, "phase")
    if not isinstance(phase, int) or isinstance(phase, bool) or phase != 2:
        raise ValueError("execution phase is not EXECUTING")
    if not _required(match_state, "motion_authorized"):
        raise ValueError("match motion authority is not granted")
    if not _required(watchdog_health, "ready"):
        raise ValueError("watchdog is not ready")
    if _required(watchdog_health, "stop_latched"):
        raise ValueError("watchdog stop is latched")
