# hsl_core/tests/test_types.py
import pytest

from hsl_core.types import (
    EgoState,
    MotionCandidate,
    OpponentState,
    OpponentTrack,
    Pose2D,
    SafetyStatus,
    Twist2D,
)


def _ego() -> EgoState:
    return EgoState(
        pose=Pose2D(1.0, 2.0, 0.5),
        twist=Twist2D(0.2, 0.1),
        pose_covariance=(0.0,) * 9,
        localization_epoch="epoch-1",
        healthy=True,
        slip=False,
        observation_time_s=10.0,
        frame_id="base_link",
    )


def test_domain_objects_are_immutable_and_validate_shape():
    state = _ego()

    with pytest.raises(AttributeError):
        state.healthy = False

    with pytest.raises(ValueError, match="exactly 9"):
        EgoState(
            pose=state.pose,
            twist=state.twist,
            pose_covariance=(0.0,) * 8,
            localization_epoch="epoch-1",
            healthy=True,
            slip=False,
            observation_time_s=10.0,
            frame_id="base_link",
        )


def test_temporal_and_spatial_metadata_are_required():
    with pytest.raises(ValueError, match="valid_until_s"):
        MotionCandidate(
            linear_velocity_mps=0.2,
            angular_velocity_rps=0.0,
            observation_time_s=5.0,
            valid_until_s=4.0,
            horizon_s=0.2,
            source_id="planner",
            map_version=3,
            localization_epoch="epoch-1",
        )

    track = OpponentTrack(
        track_id="opponent-1",
        pose=Pose2D(1.0, 1.0, 0.0),
        velocity_x_mps=0.1,
        velocity_y_mps=0.0,
        covariance=(0.0,) * 16,
        yaw_valid=False,
        last_measurement_s=1.0,
        valid_until_s=2.0,
        localization_epoch="epoch-1",
        map_version=3,
        state=OpponentState.TRACKED,
        source_id="tracker",
        frame_id="map",
    )
    assert track.state is OpponentState.TRACKED


def test_safety_status_rejects_empty_audit_fields():
    with pytest.raises(ValueError, match="reason"):
        SafetyStatus(False, "", 1.0, 0.01, 0.2, True, "supervisor", 2.0)
