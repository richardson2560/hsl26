# hsl_core/hsl_core/control/safety.py
"""Bounded, ROS-independent forward safety evaluation.

This Phase-1 evaluator deliberately supports only a conservative scalar
forward corridor. It never treats a distance as evidence without a matching
frame, timestamp, coverage flag, and epoch identity.
"""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Tuple

from .braking import (
    admissible_speed,
    admissible_speed_accelerating,
    stopping_distance_accelerating,
    wheel_rates_within_limits,
)


STOP = 0
ADMIT = 1
LIMIT = 2


class ControlMode(IntEnum):
    HOLD = 0
    TRACK_PATH = 1
    ALIGN = 2
    APPROACH_CAPTURE = 3


def _finite(value: float, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _nonnegative(value: float, name: str) -> float:
    value = _finite(value, name)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return value


@dataclass(frozen=True)
class LimitsProfile:
    """Calibrated limits required to certify a forward command."""

    limits_id: str
    calibration_id: str
    wheel_separation_m: float
    wheel_radius_m: float
    max_wheel_rate_radps: float
    speed_max_mps: float
    yaw_rate_max_rps: float
    b_forward_min_mps2: float
    response_bound_s: float
    clearance_margin_m: float
    acceleration_delay_mps2: float = 0.0
    rotation_radius_m: float = 0.0
    rotation_clearance_margin_m: float = 0.0
    linear_rest_tolerance_mps: float = 0.0
    angular_acceleration_max_rps2: float = 0.0
    lateral_acceleration_max_mps2: float = 0.0
    align_yaw_rate_max_rps: float = 0.0

    def __post_init__(self) -> None:
        if (
            not isinstance(self.limits_id, str)
            or not isinstance(self.calibration_id, str)
            or not self.limits_id
            or not self.calibration_id
        ):
            raise ValueError("limits_id and calibration_id must not be empty")
        for value, name in (
            (self.wheel_separation_m, "wheel_separation_m"),
            (self.wheel_radius_m, "wheel_radius_m"),
            (self.speed_max_mps, "speed_max_mps"),
            (self.yaw_rate_max_rps, "yaw_rate_max_rps"),
            (self.response_bound_s, "response_bound_s"),
        ):
            _finite(value, name)
        if self.wheel_separation_m <= 0.0 or self.wheel_radius_m <= 0.0:
            raise ValueError("wheel geometry must be positive")
        for value, name in (
            (self.max_wheel_rate_radps, "max_wheel_rate_radps"),
            (self.speed_max_mps, "speed_max_mps"),
            (self.yaw_rate_max_rps, "yaw_rate_max_rps"),
            (self.b_forward_min_mps2, "b_forward_min_mps2"),
            (self.response_bound_s, "response_bound_s"),
            (self.clearance_margin_m, "clearance_margin_m"),
            (self.acceleration_delay_mps2, "acceleration_delay_mps2"),
            (self.rotation_radius_m, "rotation_radius_m"),
            (self.rotation_clearance_margin_m, "rotation_clearance_margin_m"),
            (self.linear_rest_tolerance_mps, "linear_rest_tolerance_mps"),
            (self.angular_acceleration_max_rps2, "angular_acceleration_max_rps2"),
            (self.lateral_acceleration_max_mps2, "lateral_acceleration_max_mps2"),
            (self.align_yaw_rate_max_rps, "align_yaw_rate_max_rps"),
        ):
            _nonnegative(value, name)
        if self.b_forward_min_mps2 <= 0.0:
            raise ValueError("b_forward_min_mps2 must be positive")


@dataclass(frozen=True)
class TimingProfile:
    """Lease and computation bounds expressed in seconds."""

    candidate_lease_s: float
    obstacle_lease_s: float
    ego_lease_s: float
    processing_budget_s: float
    accepted_clock_skew_s: float = 0.0

    def __post_init__(self) -> None:
        for value, name in (
            (self.candidate_lease_s, "candidate_lease_s"),
            (self.obstacle_lease_s, "obstacle_lease_s"),
            (self.ego_lease_s, "ego_lease_s"),
            (self.processing_budget_s, "processing_budget_s"),
            (self.accepted_clock_skew_s, "accepted_clock_skew_s"),
        ):
            _nonnegative(value, name)
        if not self.candidate_lease_s or not self.obstacle_lease_s or not self.ego_lease_s:
            raise ValueError("input leases must be positive")
        if not self.processing_budget_s:
            raise ValueError("processing_budget_s must be positive")


@dataclass(frozen=True)
class SafetySnapshot:
    """Immutable, version-bound data consumed by one safety cycle."""

    candidate_seq: int
    candidate_v_mps: float
    candidate_omega_rps: float
    candidate_stamp_ns: int
    candidate_valid_until_ns: int
    obstacle_stamp_ns: int
    ego_stamp_ns: int
    now_ros_ns: int
    frame_id: str
    expected_frame_id: str
    clock_epoch: str
    expected_clock_epoch: str
    localization_epoch: str
    expected_localization_epoch: str
    coverage_valid: bool
    free_distance_m: float
    ego_speed_mps: float
    candidate_map_version: int
    expected_map_version: int
    candidate_topology_version: int
    expected_topology_version: int
    candidate_lease_generation: int
    expected_lease_generation: int
    option_instance_id: str = ""
    expected_option_instance_id: str = ""
    received_steady_ns: int = 0
    control_mode: ControlMode = ControlMode.TRACK_PATH
    free_distance_360_m: float = 0.0
    coverage_360_valid: bool = False
    position_error_bound_m: float = 0.0
    control_period_s: float = 0.0
    ego_angular_velocity_rps: float = 0.0
    curved_path_clearance_m: float = 0.0
    curved_path_coverage_valid: bool = False

    def __post_init__(self) -> None:
        if (
            not isinstance(self.candidate_seq, int)
            or isinstance(self.candidate_seq, bool)
            or self.candidate_seq < 0
        ):
            raise ValueError("candidate_seq must be a non-negative integer")
        for value, name in (
            (self.candidate_v_mps, "candidate_v_mps"),
            (self.candidate_omega_rps, "candidate_omega_rps"),
            (self.free_distance_m, "free_distance_m"),
            (self.free_distance_360_m, "free_distance_360_m"),
            (self.ego_speed_mps, "ego_speed_mps"),
            (self.position_error_bound_m, "position_error_bound_m"),
            (self.control_period_s, "control_period_s"),
            (self.ego_angular_velocity_rps, "ego_angular_velocity_rps"),
            (self.curved_path_clearance_m, "curved_path_clearance_m"),
        ):
            _finite(value, name)
        for value, name in (
            (self.candidate_stamp_ns, "candidate_stamp_ns"),
            (self.candidate_valid_until_ns, "candidate_valid_until_ns"),
            (self.obstacle_stamp_ns, "obstacle_stamp_ns"),
            (self.ego_stamp_ns, "ego_stamp_ns"),
            (self.now_ros_ns, "now_ros_ns"),
            (self.received_steady_ns, "received_steady_ns"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        for value, name in (
            (self.candidate_map_version, "candidate_map_version"),
            (self.expected_map_version, "expected_map_version"),
            (self.candidate_topology_version, "candidate_topology_version"),
            (self.expected_topology_version, "expected_topology_version"),
            (self.candidate_lease_generation, "candidate_lease_generation"),
            (self.expected_lease_generation, "expected_lease_generation"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if self.candidate_lease_generation == 0 or self.expected_lease_generation == 0:
            raise ValueError("lease generations must be positive")
        if self.candidate_valid_until_ns <= self.candidate_stamp_ns:
            raise ValueError("candidate lease must be positive")
        if self.free_distance_m < 0.0:
            raise ValueError("free_distance_m must be non-negative")
        if (
            self.free_distance_360_m < 0.0
            or self.position_error_bound_m < 0.0
            or self.curved_path_clearance_m < 0.0
        ):
            raise ValueError("rotation clearance and position bound must be non-negative")
        if self.control_period_s < 0.0:
            raise ValueError("control_period_s must be non-negative")
        try:
            mode = ControlMode(self.control_mode)
        except (TypeError, ValueError) as error:
            raise ValueError("invalid control mode") from error
        object.__setattr__(self, "control_mode", mode)
        if not isinstance(self.coverage_valid, bool):
            raise ValueError("coverage_valid must be boolean")
        if not isinstance(self.coverage_360_valid, bool):
            raise ValueError("coverage_360_valid must be boolean")
        if not isinstance(self.curved_path_coverage_valid, bool):
            raise ValueError("curved_path_coverage_valid must be boolean")
        for value, name in (
            (self.frame_id, "frame_id"),
            (self.expected_frame_id, "expected_frame_id"),
            (self.clock_epoch, "clock_epoch"),
            (self.expected_clock_epoch, "expected_clock_epoch"),
            (self.localization_epoch, "localization_epoch"),
            (self.expected_localization_epoch, "expected_localization_epoch"),
        ):
            if not value:
                raise ValueError(f"{name} must not be empty")


@dataclass(frozen=True)
class SafetyEvaluation:
    """One auditable supervisor result."""

    decision: int
    primary_reason: str
    reasons: Tuple[str, ...]
    candidate_seq: int
    proposed_v_mps: float
    proposed_omega_rps: float
    applied_v_mps: float
    applied_omega_rps: float
    checked_clearance_m: float
    required_stop_distance_m: float
    response_bound_s: float
    processing_time_s: float

    def __post_init__(self) -> None:
        if (
            not isinstance(self.decision, int)
            or isinstance(self.decision, bool)
            or self.decision not in (STOP, ADMIT, LIMIT)
        ):
            raise ValueError("invalid safety decision")
        if not self.primary_reason or not self.reasons:
            raise ValueError("safety reasons must not be empty")
        for value, name in (
            (self.proposed_v_mps, "proposed_v_mps"),
            (self.proposed_omega_rps, "proposed_omega_rps"),
            (self.applied_v_mps, "applied_v_mps"),
            (self.applied_omega_rps, "applied_omega_rps"),
            (self.checked_clearance_m, "checked_clearance_m"),
            (self.required_stop_distance_m, "required_stop_distance_m"),
            (self.response_bound_s, "response_bound_s"),
            (self.processing_time_s, "processing_time_s"),
        ):
            _finite(value, name)
        if self.applied_v_mps < 0.0 or self.processing_time_s < 0.0:
            raise ValueError("applied speed and processing time must be non-negative")


class SafetySupervisor:
    """Evaluate forward commands and separately evidenced constant-curvature arcs."""

    def __init__(self, limits: LimitsProfile, timing: TimingProfile) -> None:
        self._limits = limits
        self._timing = timing

    def evaluate(
        self,
        snapshot: SafetySnapshot,
        now_ros_ns: int,
        now_steady_ns: int,
    ) -> SafetyEvaluation:
        if not isinstance(now_ros_ns, int) or isinstance(now_ros_ns, bool) or now_ros_ns < 0:
            raise ValueError("now_ros_ns must be non-negative")
        if not isinstance(now_steady_ns, int) or now_steady_ns < 0:
            raise ValueError("now_steady_ns must be a non-negative integer")
        required_ego = (
            stopping_distance_accelerating(
                snapshot.ego_speed_mps,
                self._limits.acceleration_delay_mps2,
                self._limits.b_forward_min_mps2,
                self._limits.response_bound_s,
            )
            if snapshot.ego_speed_mps >= 0.0
            else 0.0
        )
        if now_steady_ns < snapshot.received_steady_ns:
            return self._result(
                snapshot, STOP, "COMPUTE_OVERRUN", ("COMPUTE_OVERRUN",),
                0.0, required_ego, 0
            )
        processing_ns = now_steady_ns - snapshot.received_steady_ns
        if processing_ns > int(self._timing.processing_budget_s * 1e9):
            return self._result(
                snapshot,
                STOP,
                "COMPUTE_OVERRUN",
                ("COMPUTE_OVERRUN",),
                0.0,
                required_ego,
                processing_ns,
            )
        reasons = self._validation_reasons(snapshot, now_ros_ns)
        required = required_ego
        if reasons:
            return self._result(
                snapshot, STOP, reasons[0], reasons, 0.0, required, processing_ns
            )
        if snapshot.control_mode is ControlMode.ALIGN:
            return self._evaluate_align(snapshot, processing_ns)
        if snapshot.control_mode not in (
            ControlMode.HOLD,
            ControlMode.TRACK_PATH,
            ControlMode.APPROACH_CAPTURE,
        ):
            return self._result(
                snapshot, STOP, "LIMITS_INVALID", ("LIMITS_INVALID",),
                0.0, required, processing_ns
            )
        if snapshot.candidate_v_mps < 0.0 or snapshot.ego_speed_mps < 0.0:
            reasons = ("LIMITS_INVALID",)
            return self._result(
                snapshot, STOP, reasons[0], reasons, 0.0, required, processing_ns
            )
        curved = snapshot.candidate_omega_rps != 0.0
        if curved and (
            snapshot.candidate_v_mps <= 0.0
            or self._limits.lateral_acceleration_max_mps2 <= 0.0
        ):
            return self._result(
                snapshot, STOP, "LIMITS_INVALID", ("LIMITS_INVALID",),
                0.0, required, processing_ns
            )
        if curved and not snapshot.curved_path_coverage_valid:
            return self._result(
                snapshot, STOP, "CURVED_PATH_UNCERTIFIED",
                ("CURVED_PATH_UNCERTIFIED",), 0.0, required, processing_ns,
                checked_clearance_m=snapshot.curved_path_clearance_m,
            )
        if not curved and abs(snapshot.candidate_omega_rps) > self._limits.yaw_rate_max_rps:
            return self._result(
                snapshot, STOP, "LIMITS_INVALID", ("LIMITS_INVALID",),
                0.0, required, processing_ns
            )
        if not curved and not wheel_rates_within_limits(
            snapshot.candidate_v_mps,
            snapshot.candidate_omega_rps,
            self._limits.wheel_separation_m,
            self._limits.wheel_radius_m,
            self._limits.max_wheel_rate_radps,
        ):
            reasons = ("LIMITS_INVALID",)
            return self._result(
                snapshot, STOP, reasons[0], reasons, 0.0, required, processing_ns
            )
        checked_clearance = (
            snapshot.curved_path_clearance_m
            if curved
            else snapshot.free_distance_m
        )
        safe_speed = admissible_speed_accelerating(
            checked_clearance,
            self._limits.clearance_margin_m,
            self._limits.acceleration_delay_mps2,
            self._limits.b_forward_min_mps2,
            self._limits.response_bound_s,
            self._limits.speed_max_mps,
        )
        if required > max(0.0, checked_clearance - self._limits.clearance_margin_m):
            reasons = ("BRAKING_INFEASIBLE",)
            return self._result(
                snapshot, STOP, reasons[0], reasons, 0.0, required, processing_ns,
                checked_clearance_m=checked_clearance,
            )
        if curved:
            curvature = abs(
                snapshot.candidate_omega_rps / snapshot.candidate_v_mps
            )
            lateral_speed_limit = math.sqrt(
                self._limits.lateral_acceleration_max_mps2 / curvature
            )
            yaw_speed_limit = (
                self._limits.yaw_rate_max_rps / curvature
            )
            wheel_rate_per_speed = max(
                abs(
                    1.0
                    - curvature * self._limits.wheel_separation_m / 2.0
                ),
                abs(
                    1.0
                    + curvature * self._limits.wheel_separation_m / 2.0
                ),
            ) / self._limits.wheel_radius_m
            wheel_speed_limit = (
                self._limits.max_wheel_rate_radps / wheel_rate_per_speed
            )
            safe_speed = min(
                safe_speed,
                lateral_speed_limit,
                yaw_speed_limit,
                wheel_speed_limit,
            )
        if snapshot.candidate_v_mps <= safe_speed:
            required_candidate = stopping_distance_accelerating(
                snapshot.candidate_v_mps,
                self._limits.acceleration_delay_mps2,
                self._limits.b_forward_min_mps2,
                self._limits.response_bound_s,
            )
            return self._result(
                snapshot, ADMIT, "NONE", ("NONE",),
                snapshot.candidate_v_mps, required_candidate, processing_ns,
                applied_omega=snapshot.candidate_omega_rps,
                checked_clearance_m=checked_clearance,
            )
        limited = min(safe_speed, snapshot.candidate_v_mps)
        limited_omega = (
            snapshot.candidate_omega_rps * limited / snapshot.candidate_v_mps
            if curved
            else 0.0
        )
        return self._result(
            snapshot, LIMIT, "COMMAND_LIMITED", ("COMMAND_LIMITED",),
            limited,
            stopping_distance_accelerating(
                limited,
                self._limits.acceleration_delay_mps2,
                self._limits.b_forward_min_mps2,
                self._limits.response_bound_s,
            ),
            processing_ns,
            applied_omega=limited_omega,
            checked_clearance_m=checked_clearance,
        )

    def _evaluate_align(
        self, snapshot: SafetySnapshot, processing_ns: int
    ) -> SafetyEvaluation:
        clearance = snapshot.free_distance_360_m
        required_clearance = (
            self._limits.rotation_radius_m
            + snapshot.position_error_bound_m
            + self._limits.rotation_clearance_margin_m
        )
        if snapshot.candidate_v_mps != 0.0:
            return self._result(
                snapshot, STOP, "LIMITS_INVALID", ("LIMITS_INVALID",),
                0.0, 0.0, processing_ns, checked_clearance_m=clearance
            )
        if abs(snapshot.ego_speed_mps) > self._limits.linear_rest_tolerance_mps:
            return self._result(
                snapshot, STOP, "WAITING_LINEAR_REST", ("WAITING_LINEAR_REST",),
                0.0, 0.0, processing_ns, checked_clearance_m=clearance
            )
        if not snapshot.coverage_360_valid:
            return self._result(
                snapshot, STOP, "OUTSIDE_COVERAGE", ("OUTSIDE_COVERAGE",),
                0.0, 0.0, processing_ns, checked_clearance_m=clearance
            )
        if (
            self._limits.rotation_radius_m <= 0.0
            or self._limits.rotation_clearance_margin_m <= 0.0
            or self._limits.linear_rest_tolerance_mps <= 0.0
            or self._limits.angular_acceleration_max_rps2 <= 0.0
            or snapshot.control_period_s <= 0.0
        ):
            return self._result(
                snapshot, STOP, "ROTATION_PROFILE_INVALID", ("ROTATION_PROFILE_INVALID",),
                0.0, 0.0, processing_ns, checked_clearance_m=clearance
            )
        if clearance < required_clearance:
            return self._result(
                snapshot, STOP, "ROTATION_CLEARANCE_INSUFFICIENT",
                ("ROTATION_CLEARANCE_INSUFFICIENT",), 0.0, 0.0,
                processing_ns, checked_clearance_m=clearance
            )
        align_yaw_limit = (
            self._limits.align_yaw_rate_max_rps
            if self._limits.align_yaw_rate_max_rps > 0.0
            else self._limits.yaw_rate_max_rps
        )
        if abs(snapshot.candidate_omega_rps) > min(
            self._limits.yaw_rate_max_rps, align_yaw_limit
        ):
            return self._result(
                snapshot, STOP, "LIMITS_INVALID", ("LIMITS_INVALID",),
                0.0, 0.0, processing_ns, checked_clearance_m=clearance
            )
        if not wheel_rates_within_limits(
            0.0,
            snapshot.candidate_omega_rps,
            self._limits.wheel_separation_m,
            self._limits.wheel_radius_m,
            self._limits.max_wheel_rate_radps,
        ):
            return self._result(
                snapshot, STOP, "LIMITS_INVALID", ("LIMITS_INVALID",),
                0.0, 0.0, processing_ns, checked_clearance_m=clearance
            )
        max_delta_omega = (
            self._limits.angular_acceleration_max_rps2
            * snapshot.control_period_s
        )
        if abs(snapshot.candidate_omega_rps - snapshot.ego_angular_velocity_rps) > (
            max_delta_omega + 1e-12
        ):
            return self._result(
                snapshot, STOP, "ANGULAR_ACCELERATION_EXCEEDED",
                ("ANGULAR_ACCELERATION_EXCEEDED",), 0.0, 0.0,
                processing_ns, checked_clearance_m=clearance
            )
        return self._result(
            snapshot, ADMIT, "NONE", ("NONE",),
            0.0, 0.0, processing_ns,
            applied_omega=snapshot.candidate_omega_rps,
            checked_clearance_m=clearance,
        )

    def _validation_reasons(
        self, snapshot: SafetySnapshot, now_ros_ns: int
    ) -> Tuple[str, ...]:
        reasons = []
        allowed_skew_ns = int(self._timing.accepted_clock_skew_s * 1e9)
        candidate_lease_ns = int(self._timing.candidate_lease_s * 1e9)
        if snapshot.now_ros_ns > now_ros_ns + allowed_skew_ns:
            reasons.append("STALE_EGO")
        if (
            snapshot.candidate_stamp_ns > now_ros_ns
            or now_ros_ns - snapshot.candidate_stamp_ns > candidate_lease_ns
            or now_ros_ns >= snapshot.candidate_valid_until_ns
        ):
            reasons.append("STALE_CANDIDATE")
        if (
            snapshot.obstacle_stamp_ns > now_ros_ns
            or now_ros_ns - snapshot.obstacle_stamp_ns
            > int(self._timing.obstacle_lease_s * 1e9)
        ):
            reasons.append("STALE_OBSTACLES")
        if (
            snapshot.ego_stamp_ns > now_ros_ns
            or now_ros_ns - snapshot.ego_stamp_ns
            > int(self._timing.ego_lease_s * 1e9)
        ):
            reasons.append("STALE_EGO")
        if snapshot.frame_id != snapshot.expected_frame_id:
            reasons.append("TF_UNAVAILABLE")
        if snapshot.clock_epoch != snapshot.expected_clock_epoch:
            reasons.append("EPOCH_MISMATCH")
        if snapshot.localization_epoch != snapshot.expected_localization_epoch:
            reasons.append("EPOCH_MISMATCH")
        if snapshot.candidate_v_mps < 0.0 or snapshot.ego_speed_mps < 0.0:
            reasons.append("LIMITS_INVALID")
        if not snapshot.coverage_valid:
            reasons.append("OUTSIDE_COVERAGE")
        if snapshot.control_mode is ControlMode.ALIGN and not snapshot.coverage_360_valid:
            reasons.append("OUTSIDE_COVERAGE")
        if (
            snapshot.expected_option_instance_id
            and snapshot.option_instance_id != snapshot.expected_option_instance_id
        ):
            reasons.append("OPTION_REVOKED")
        if snapshot.candidate_lease_generation != snapshot.expected_lease_generation:
            reasons.append("OPTION_REVOKED")
        if (
            snapshot.candidate_map_version != snapshot.expected_map_version
            or snapshot.candidate_topology_version != snapshot.expected_topology_version
        ):
            reasons.append("STALE_CANDIDATE")
        return tuple(dict.fromkeys(reasons))

    def _result(
        self,
        snapshot: SafetySnapshot,
        decision: int,
        primary_reason: str,
        reasons: Tuple[str, ...],
        applied_v: float,
        required: float,
        processing_time_ns: int,
        *,
        applied_omega: float = 0.0,
        checked_clearance_m: float | None = None,
    ) -> SafetyEvaluation:
        return SafetyEvaluation(
            decision=decision,
            primary_reason=primary_reason,
            reasons=reasons,
            candidate_seq=snapshot.candidate_seq,
            proposed_v_mps=snapshot.candidate_v_mps,
            proposed_omega_rps=snapshot.candidate_omega_rps,
            applied_v_mps=applied_v,
            applied_omega_rps=applied_omega,
            checked_clearance_m=(
                snapshot.free_distance_m
                if checked_clearance_m is None
                else checked_clearance_m
            ),
            required_stop_distance_m=required,
            response_bound_s=self._limits.response_bound_s,
            processing_time_s=processing_time_ns * 1e-9,
        )
