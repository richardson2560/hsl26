"""Adversarial tests for P5.4 continuous referee geometry and truth isolation."""

from dataclasses import fields
import math

import pytest

from hsl_core.match import EventEvidence, TerminalKind
from hsl_core.rules import (
    TimedPose,
    capture_predicate,
    first_capture_interval,
    first_footprint_arrival,
)
from hsl_core.types import Pose2D

from sim.kinematic.common import Actuation, Segment, WorldGeometry
from sim.kinematic.match import (
    MatchRole,
    PolicyInput,
    RoleEndpoint,
    TwoRobotMatch,
    capture_rule_event,
)
from sim.kinematic.plant import KinematicPlant
from sim.kinematic.raycaster import FirstHitRaycaster
from sim.kinematic.sensors import LidarSensor


SQUARE = ((1.0, -1.0), (2.0, -1.0), (2.0, 1.0), (1.0, 1.0))


def _sensor(seed):
    return LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=5.0),
        beam_count=16,
        max_range_m=5.0,
        seed=seed,
    )


def _timed(stamp, x, y=0.0, heading=0.0):
    return TimedPose(stamp, Pose2D(x, y, heading))


def test_footprint_first_contact_detects_enter_and_exit_between_samples():
    event = first_footprint_arrival(
        ((0.2, 0.0), (2.8, 0.0)),
        SQUARE,
        0.2,
        start_time_s=10.0,
        sample_period_s=2.0,
    )
    assert event is not None
    assert event.fraction == pytest.approx(0.6 / 2.6)
    assert event.time_s == pytest.approx(10.0 + 2.0 * 0.6 / 2.6)
    assert event.point == pytest.approx((1.0, 0.0))


def test_footprint_contact_handles_vertex_tangency_and_zero_radius():
    tangent = first_footprint_arrival(
        ((0.5, 1.2), (2.5, 1.2)), SQUARE, 0.2
    )
    assert tangent is not None
    assert tangent.fraction == pytest.approx(0.25)
    assert tangent.point == pytest.approx((1.0, 1.0))

    center_only = first_footprint_arrival(
        ((0.5, 0.0), (2.5, 0.0)), SQUARE, 0.0
    )
    assert center_only is not None
    assert center_only.fraction == pytest.approx(0.25)


def test_footprint_arrival_rejects_initial_overlap_and_does_not_count_near_miss():
    with pytest.raises(ValueError, match="initial footprint"):
        first_footprint_arrival(((0.9, 0.0), (1.5, 0.0)), SQUARE, 0.1)
    assert first_footprint_arrival(
        ((0.2, 1.2001), (2.8, 1.2001)), SQUARE, 0.2
    ) is None
    endpoint_contact = first_footprint_arrival(
        ((0.2, 0.0), (0.8, 0.0)), SQUARE, 0.2
    )
    assert endpoint_contact is not None
    assert endpoint_contact.fraction == pytest.approx(1.0)
    assert first_footprint_arrival(
        ((0.2, 0.0), (0.79, 0.0)), SQUARE, 0.2
    ) is None


@pytest.mark.parametrize(
    "radius,start_time,period",
    [(-0.1, 0.0, 1.0), (math.nan, 0.0, 1.0), (0.1, math.inf, 1.0), (0.1, 0.0, 0.0)],
)
def test_footprint_arrival_rejects_invalid_parameters(radius, start_time, period):
    with pytest.raises(ValueError):
        first_footprint_arrival(
            ((0.0, 0.0), (3.0, 0.0)),
            SQUARE,
            radius,
            start_time_s=start_time,
            sample_period_s=period,
        )


def test_footprint_arrival_rejects_invalid_polygons_and_malformed_points():
    with pytest.raises(ValueError, match="counterclockwise"):
        first_footprint_arrival(
            ((0.0, 0.0), (2.0, 0.0)),
            tuple(reversed(SQUARE)),
            0.1,
        )
    with pytest.raises(ValueError, match="2D points"):
        first_footprint_arrival(((0.0,), (2.0, 0.0)), SQUARE, 0.1)
    diamond = ((0.0, 1.0), (1.0, 0.0), (2.0, 1.0), (1.0, 2.0))
    assert first_footprint_arrival(
        ((0.25, 0.72), (0.30, 0.69)), diamond, 0.0
    ) is None


def test_capture_interval_detects_between_samples_and_brackets_time():
    event = first_capture_interval(
        (_timed(100, 0.0), _timed(200, 0.0)),
        (_timed(100, 0.8), _timed(200, 0.4)),
        (),
    )
    assert event is not None
    assert (event.segment_index, event.lower_ns, event.upper_ns) == (0, 100, 200)


def test_capture_requires_distance_bearing_and_clear_los_concurrently():
    guardian = _timed(0, 0.0).pose
    assert not capture_predicate(
        guardian, Pose2D(0.45, 0.0, 0.0), line_of_sight=True
    )
    assert not capture_predicate(
        guardian, Pose2D(0.4, 0.0, 0.0), line_of_sight=None
    )
    assert first_capture_interval(
        (_timed(10, 0.0, heading=math.pi), _timed(20, 0.0, heading=math.pi)),
        (_timed(10, 0.4), _timed(20, 0.4)),
        (),
    ) is None
    assert first_capture_interval(
        (_timed(10, 0.0), _timed(20, 0.0)),
        (_timed(10, 0.4), _timed(20, 0.4)),
        (((0.2, -0.1), (0.2, 0.1)),),
    ) is None
    assert first_capture_interval(
        (_timed(10, 0.0), _timed(20, 0.0)),
        (_timed(10, 0.4), _timed(20, 0.4)),
        (),
        circle_obstacles=(((0.2, 0.0), 0.01),),
    ) is None
    assert first_capture_interval(
        (_timed(10, 0.0), _timed(20, 0.0)),
        (_timed(10, 0.4), _timed(20, 0.4)),
        (),
        circle_obstacles=(((0.2, 0.1), 0.1),),
    ) is None


def test_capture_boundary_and_coincident_origins_never_create_false_positive():
    assert first_capture_interval(
        (_timed(1, 0.0), _timed(2, 0.0)),
        (_timed(1, 0.45), _timed(2, 0.45)),
        (),
    ) is None
    assert first_capture_interval(
        (_timed(1, 0.0), _timed(2, 0.0)),
        (_timed(1, 0.0), _timed(2, 0.0)),
        (),
    ) is None


def test_capture_contract_rejects_mismatched_or_nonmonotonic_samples():
    with pytest.raises(ValueError, match="matching strictly increasing"):
        first_capture_interval(
            (_timed(1, 0.0), _timed(2, 0.0)),
            (_timed(1, 0.4), _timed(3, 0.4)),
            (),
        )
    with pytest.raises(ValueError, match="matching strictly increasing"):
        first_capture_interval(
            (_timed(2, 0.0), _timed(1, 0.0)),
            (_timed(2, 0.4), _timed(1, 0.4)),
            (),
        )


def test_two_role_runner_exposes_only_role_scoped_sensor_inputs_to_policies():
    seen: dict[MatchRole, PolicyInput] = {}

    def policy(role):
        def decide(frame):
            assert isinstance(frame, PolicyInput)
            seen[role] = frame
            assert {field.name for field in fields(frame)} == {
                "role", "namespace", "stamp_ns", "observation", "match_state"
            }
            assert frame.role is role
            assert frame.namespace == f"/robot_{role.value}"
            assert not hasattr(frame, "truth")
            assert not hasattr(frame, "other_robot_pose")
            assert not hasattr(frame, "world_geometry")
            return Actuation(0.0, 0.0)

        return decide

    guardian = RoleEndpoint(
        MatchRole.GUARDIAN,
        "/robot_guardian",
        KinematicPlant(Pose2D(0.0, 0.0, 0.0)),
        _sensor(1),
        policy(MatchRole.GUARDIAN),
        0.17,
    )
    explorer = RoleEndpoint(
        MatchRole.EXPLORER,
        "/robot_explorer",
        KinematicPlant(Pose2D(0.4, 0.0, math.pi)),
        _sensor(2),
        policy(MatchRole.EXPLORER),
        0.17,
    )
    tick = TwoRobotMatch(guardian, explorer, WorldGeometry(())).step(0.01)

    assert set(seen) == {MatchRole.GUARDIAN, MatchRole.EXPLORER}
    assert seen[MatchRole.GUARDIAN].observation != seen[MatchRole.EXPLORER].observation
    assert tick.truth.capture_interval is not None
    assert tick.truth.robot_collision is False
    event = capture_rule_event(
        tick.truth,
        event_id="capture-1",
        stage_id="stage-sil",
        clock_epoch="clock-sil",
        localization_epoch="loc-sil",
        input_ids=("trace-1", "guardian-sensor-1", "explorer-sensor-1"),
        authorization_ref="sil-profile-approval",
    )
    assert event is not None
    assert event.kind is TerminalKind.CAPTURE
    assert event.evidence is EventEvidence.SIM_TRUTH
    assert event.event_time_lower_ns <= event.event_time_upper_ns


def test_two_role_runner_detects_swept_collision_and_prevalidates_both_commands():
    guardian_plant = KinematicPlant(Pose2D(-1.0, 0.0, 0.0))
    explorer_plant = KinematicPlant(Pose2D(1.0, 0.0, math.pi))
    guardian = RoleEndpoint(
        MatchRole.GUARDIAN, "/robot_guardian", guardian_plant, _sensor(3),
        lambda _: Actuation(1.0, 0.0), 0.2,
    )
    explorer = RoleEndpoint(
        MatchRole.EXPLORER, "/robot_explorer", explorer_plant, _sensor(4),
        lambda _: Actuation(1.0, 0.0), 0.2,
    )
    tick = TwoRobotMatch(guardian, explorer, WorldGeometry(())).step(1.0)
    assert tick.truth.robot_collision is True

    guarded_plant = KinematicPlant(Pose2D(-1.0, 0.0, 0.0))
    exploratory_plant = KinematicPlant(Pose2D(1.0, 0.0, math.pi))
    before_guardian = guarded_plant.state
    before_explorer = exploratory_plant.state
    failing = TwoRobotMatch(
        RoleEndpoint(
            MatchRole.GUARDIAN, "/robot_guardian", guarded_plant, _sensor(5),
            lambda _: Actuation(0.0, 0.0), 0.2,
        ),
        RoleEndpoint(
            MatchRole.EXPLORER, "/robot_explorer", exploratory_plant, _sensor(6),
            lambda _: Actuation(2.0, 0.0), 0.2,
        ),
        WorldGeometry(()),
    )
    with pytest.raises(ValueError, match="plant limit"):
        failing.step(0.1)
    assert guarded_plant.state == before_guardian
    assert exploratory_plant.state == before_explorer
    assert failing.stamp_ns == 0


def test_two_role_runner_rejects_role_namespace_mismatch():
    with pytest.raises(ValueError, match="role-specific"):
        RoleEndpoint(
            MatchRole.GUARDIAN,
            "/robot_explorer",
            KinematicPlant(Pose2D(0.0, 0.0, 0.0)),
            _sensor(7),
            lambda _: Actuation(0.0, 0.0),
            0.17,
        )


def test_two_role_runner_rejects_overlapping_or_clock_misaligned_initial_state():
    guardian = RoleEndpoint(
        MatchRole.GUARDIAN, "/robot_guardian",
        KinematicPlant(Pose2D(0.0, 0.0, 0.0)), _sensor(10),
        lambda _: Actuation(0.0, 0.0), 0.2,
    )
    overlapping = RoleEndpoint(
        MatchRole.EXPLORER, "/robot_explorer",
        KinematicPlant(Pose2D(0.3, 0.0, math.pi)), _sensor(11),
        lambda _: Actuation(0.0, 0.0), 0.2,
    )
    with pytest.raises(ValueError, match="overlap"):
        TwoRobotMatch(guardian, overlapping, WorldGeometry(()))

    advanced_plant = KinematicPlant(Pose2D(1.0, 0.0, math.pi))
    advanced_plant.step(Actuation(0.0, 0.0), 0.1)
    misaligned = RoleEndpoint(
        MatchRole.EXPLORER, "/robot_explorer",
        advanced_plant, _sensor(12),
        lambda _: Actuation(0.0, 0.0), 0.2,
    )
    aligned_guardian = RoleEndpoint(
        MatchRole.GUARDIAN, "/robot_guardian",
        KinematicPlant(Pose2D(-1.0, 0.0, 0.0)), _sensor(13),
        lambda _: Actuation(0.0, 0.0), 0.2,
    )
    with pytest.raises(ValueError, match="same simulation clock"):
        TwoRobotMatch(aligned_guardian, misaligned, WorldGeometry(()))


def test_two_role_runner_has_no_physical_authority_and_no_implicit_goal_event():
    guardian = RoleEndpoint(
        MatchRole.GUARDIAN, "/robot_guardian",
        KinematicPlant(Pose2D(0.0, 0.0, 0.0)), _sensor(8),
        lambda _: Actuation(0.0, 0.0), 0.17,
    )
    explorer = RoleEndpoint(
        MatchRole.EXPLORER, "/robot_explorer",
        KinematicPlant(Pose2D(0.8, 0.0, math.pi)), _sensor(9),
        lambda _: Actuation(0.0, 0.0), 0.17,
    )
    tick = TwoRobotMatch(guardian, explorer, WorldGeometry(())).step(0.01)
    assert not hasattr(tick.truth, "arrival")
    assert tick.truth.capture_interval is None
