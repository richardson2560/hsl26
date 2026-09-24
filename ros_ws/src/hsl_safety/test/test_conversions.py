# ros_ws/src/hsl_safety/test/test_conversions.py
from types import SimpleNamespace

import pytest

from hsl_safety.conversions import (
    ego_state_from_ros,
    motion_candidate_from_ros,
    opponent_track_from_ros,
)
from hsl_core.types import OpponentState


def _time(sec, nanosec=0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def _header(frame_id, stamp):
    return SimpleNamespace(frame_id=frame_id, stamp=stamp)


def _pose(x, y, theta):
    return SimpleNamespace(x=x, y=y, theta=theta)


def _twist(linear_x, linear_y=0.0, angular_z=0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=linear_x, y=linear_y),
        angular=SimpleNamespace(z=angular_z),
    )


def test_ego_conversion_preserves_time_frame_and_covariance():
    message = SimpleNamespace(
        header=_header("base_link", _time(12, 500_000_000)),
        pose=_pose(1.0, 2.0, 0.3),
        twist=_twist(0.4, angular_z=0.2),
        pose_covariance=[0.0] * 9,
        localization_epoch="epoch-2",
        healthy=True,
        slip=False,
    )

    result = ego_state_from_ros(message)

    assert result.observation_time_s == 12.5
    assert result.frame_id == "base_link"
    assert result.pose_covariance == (0.0,) * 9


def test_motion_candidate_conversion_requires_new_epoch_contract():
    message = SimpleNamespace(
        header=_header("map", _time(20)),
        linear_velocity=0.3,
        angular_velocity=-0.1,
        valid_until=_time(20, 200_000_000),
        horizon=0.4,
        source_id="navigation",
        map_version=7,
        localization_epoch="epoch-2",
    )

    result = motion_candidate_from_ros(message)

    assert result.valid_until_s == 20.2
    assert result.map_version == 7
    assert result.localization_epoch == "epoch-2"


def test_opponent_conversion_maps_named_lifecycle_state():
    message = SimpleNamespace(
        header=_header("map", _time(30)),
        id="opponent-1",
        pose=_pose(2.0, 3.0, 0.0),
        twist=_twist(0.1, linear_y=-0.2),
        covariance=[0.0] * 16,
        yaw_valid=False,
        last_measurement=_time(29),
        valid_until=_time(31),
        localization_epoch="epoch-2",
        map_version=7,
        state=OpponentState.COASTING.value,
        source_id="tracker",
    )

    result = opponent_track_from_ros(message)

    assert result.state is OpponentState.COASTING
    assert result.valid_until_s == 31.0


def test_invalid_ros_time_is_rejected():
    message = SimpleNamespace(
        header=_header("base_link", _time(1, 1_000_000_000)),
        pose=_pose(0.0, 0.0, 0.0),
        twist=_twist(0.0),
        pose_covariance=[0.0] * 9,
        localization_epoch="epoch",
        healthy=True,
        slip=False,
    )

    with pytest.raises(ValueError, match="nanosec"):
        ego_state_from_ros(message)
