# ros_ws/src/hsl_safety/hsl_safety/conversions.py
"""Conversions between ROS messages and immutable HSL26 domain objects.

This module is the ROS boundary.  The mathematical core remains importable
without ROS 2, while malformed or incomplete messages fail explicitly.
"""

from hsl_core.types import (
    EgoState,
    MotionCandidate,
    TrackState,
    OpponentTrack,
    Pose2D,
    SafetyStatus,
    Twist2D,
)


def _time_to_seconds(stamp) -> float:
    if stamp is None or not hasattr(stamp, "sec") or not hasattr(stamp, "nanosec"):
        raise ValueError("ROS time must contain sec and nanosec")
    if stamp.nanosec < 0 or stamp.nanosec >= 1_000_000_000:
        raise ValueError("ROS nanosec must be in [0, 1e9)")
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _seconds_to_time(seconds: float, time_type):
    if seconds < 0:
        raise ValueError("ROS time cannot represent negative domain time")
    result = time_type()
    result.sec = int(seconds)
    result.nanosec = int(round((seconds - result.sec) * 1_000_000_000))
    if result.nanosec == 1_000_000_000:
        result.sec += 1
        result.nanosec = 0
    return result


def _pose(message) -> Pose2D:
    return Pose2D(message.x, message.y, message.theta)


def _twist(message) -> Twist2D:
    return Twist2D(message.linear.x, message.angular.z)


def ego_state_from_ros(message) -> EgoState:
    return EgoState(
        pose=_pose(message.pose),
        twist=_twist(message.twist),
        pose_covariance=tuple(message.pose_covariance),
        localization_epoch=message.localization_epoch,
        healthy=message.healthy,
        slip=message.slip,
        observation_time_s=_time_to_seconds(message.header.stamp),
        frame_id=message.header.frame_id,
    )


def opponent_track_from_ros(message) -> OpponentTrack:
    return OpponentTrack(
        track_id=message.id,
        pose=_pose(message.pose),
        velocity_x_mps=message.twist.linear.x,
        velocity_y_mps=message.twist.linear.y,
        covariance=tuple(message.covariance),
        yaw_valid=message.yaw_valid,
        last_measurement_s=_time_to_seconds(message.last_measurement),
        valid_until_s=_time_to_seconds(message.valid_until),
        localization_epoch=message.localization_epoch,
        map_version=message.map_version,
        state=TrackState(message.state),
        source_id=message.source_id,
        frame_id=message.header.frame_id,
    )


def motion_candidate_from_ros(message) -> MotionCandidate:
    return MotionCandidate(
        linear_velocity_mps=message.linear_velocity,
        angular_velocity_rps=message.angular_velocity,
        observation_time_s=_time_to_seconds(message.header.stamp),
        valid_until_s=_time_to_seconds(message.valid_until),
        horizon_s=message.horizon,
        source_id=message.source_id,
        map_version=message.map_version,
        localization_epoch=message.localization_epoch,
    )


def safety_status_to_ros(domain: SafetyStatus, message, time_type):
    """Populate a generated SafetyStatus message and return it."""
    message.header.stamp = _seconds_to_time(domain.publication_time_s, time_type)
    message.vetoed = domain.vetoed
    message.reason = domain.reason
    message.free_distance = domain.free_distance_m
    message.measured_latency = domain.measured_latency_s
    message.admissible_velocity = domain.admissible_velocity_mps
    message.healthy = domain.healthy
    message.source_id = domain.source_id
    return message
