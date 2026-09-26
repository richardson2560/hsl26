"""Adversarial P5.1 tests for stage authority, zones and event adjudication."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace

import pytest

from hsl_core.contracts import validate_header
from hsl_core.match import (
    EventEvidence,
    EventPredicate,
    EventResolution,
    GoalZone,
    MatchState,
    Role,
    RuleEvent,
    StageManager,
    StagePhase,
    StageProfile,
    TerminalKind,
    ZonePurpose,
    ZoneStatus,
)


AUTH = "organizer-approval-2026"
HASH = "a" * 64
ZONE_HASH = "b" * 64
SQUARE = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
RESET_ACKS = frozenset(("actions", "execution", "plans", "candidates", "tracks", "memory"))
EPOCH_ACKS = frozenset(("actions", "plans", "candidates", "tracks", "map", "world"))


def profile(**overrides):
    values = dict(
        profile_id="sil-profile",
        score_profile_id="score-profile-fixture",
        freeze_duration_ns=20,
        stage_duration_ns=100,
        lease_duration_ns=25,
        zone_update_window_ns=20,
        max_request_records=100,
        max_event_records=32,
        max_audit_records=256,
        config_hash=HASH,
        source_id="hsl_match",
        source_session="session-1",
        clock_epoch="clock-1",
        localization_epoch="localization-1",
        organizer_authorization_ref=AUTH,
        approved_authorization_refs=(AUTH, "zone-approval"),
        approved_zone_frames=("map",),
        approved_memory_profile_ids=("retention-approved",),
    )
    values.update(overrides)
    return StageProfile(**values)


def manager(**profile_overrides):
    ids = iter(("stage-2", "stage-3", "stage-4"))
    return StageManager(
        profile(**profile_overrides),
        initial_stage_id="stage-1",
        stage_number=1,
        role=Role.EXPLORER,
        stage_id_factory=lambda: next(ids),
    )


def start(mgr, *, now=100, stamp=100, request_id="start", auth=AUTH):
    return mgr.start_stage(
        request_id=request_id,
        expected_stage_id=mgr.stage_id,
        official_start_stamp_ns=stamp,
        organizer_authorization_ref=auth,
        now_ns=now,
    )


def target_zone(status=ZoneStatus.ACCEPTED, *, zone_id="target-A", auth_ref="zone-approval"):
    return GoalZone(
        zone_id=zone_id,
        purpose=ZonePurpose.TARGET,
        status=status,
        boundary=SQUARE,
        frame_id="map",
        position_error_bound_m=0.02,
        provider="organizer",
        approval_ref=auth_ref,
        provenance_hash=ZONE_HASH,
    )


def event(
    mgr,
    *,
    event_id="event-1",
    kind=TerminalKind.CAPTURE,
    evidence=EventEvidence.OFFICIAL,
    predicate=EventPredicate.TRUE,
    lower=120,
    upper=125,
    authorization=AUTH,
    zone_id="",
):
    return RuleEvent(
        event_id=event_id,
        stage_id=mgr.stage_id,
        clock_epoch=mgr.profile.clock_epoch,
        localization_epoch=mgr.profile.localization_epoch,
        kind=kind,
        evidence=evidence,
        predicate=predicate,
        event_time_lower_ns=lower,
        event_time_upper_ns=upper,
        input_ids=("evidence-1",),
        authorization_ref=authorization,
        zone_id=zone_id,
    )


@pytest.mark.parametrize(
    "changes, message",
    [
        ({"freeze_duration_ns": 0}, "freeze_duration_ns"),
        ({"stage_duration_ns": 0}, "stage_duration_ns"),
        ({"lease_duration_ns": 0}, "lease_duration_ns"),
        ({"freeze_duration_ns": 100, "stage_duration_ns": 100}, "shorter"),
        ({"config_hash": "not-a-hash"}, "SHA-256"),
        ({"approved_authorization_refs": ()}, "approved_authorization_refs"),
        ({"allow_sim_truth_terminal": 1}, "must be boolean"),
        ({"stage_duration_ns": True}, "stage_duration_ns"),
    ],
)
def test_profile_rejects_missing_or_invalid_authority_parameters(changes, message):
    with pytest.raises(ValueError, match=message):
        profile(**changes)


def test_profile_requires_finite_positive_resource_caps():
    with pytest.raises(ValueError, match="max_request_records"):
        profile(max_request_records=0)
    with pytest.raises(ValueError, match="max_event_records"):
        profile(max_event_records=True)
    with pytest.raises(ValueError, match="max_audit_records"):
        profile(max_audit_records=0)


def test_zone_window_must_be_a_boolean_free_subinterval_of_freeze():
    with pytest.raises(ValueError, match="zone_update_window_ns"):
        profile(zone_update_window_ns=True)
    with pytest.raises(ValueError, match="zone_update_window_ns"):
        profile(zone_update_window_ns=21)


def test_stage_number_and_identity_are_strict():
    with pytest.raises(ValueError, match="stage_number"):
        StageManager(profile(), initial_stage_id="x", stage_number=True, role=Role.EXPLORER)
    with pytest.raises(ValueError, match="initial_stage_id"):
        StageManager(profile(), initial_stage_id="", stage_number=0, role=Role.EXPLORER)


def test_start_latches_once_and_duplicate_does_not_restart_timers():
    mgr = manager()
    first = start(mgr)
    duplicate = start(mgr, now=115)
    state = mgr.snapshot(now_ns=120)

    assert first.accepted and duplicate == first
    assert state.stage_started_at_ns == 100
    assert state.freeze_ends_at_ns == 120
    assert state.stage_ends_at_ns == 200
    assert state.phase == StagePhase.ACTIVE
    assert state.motion_authorized


def test_concurrent_retries_of_one_start_request_are_serialized_and_idempotent():
    mgr = manager()

    def submit(_):
        return start(mgr)

    with ThreadPoolExecutor(max_workers=8) as pool:
        results = list(pool.map(submit, range(32)))

    assert all(result.accepted for result in results)
    assert {result.stage_id for result in results} == {"stage-1"}
    assert mgr.snapshot(now_ns=100).phase == StagePhase.FREEZE
    assert sum(record.operation == "start" for record in mgr.journal) == 1


def test_request_event_and_journal_storage_are_explicitly_bounded():
    mgr = manager(max_request_records=1, max_event_records=1, max_audit_records=4)
    start(mgr)
    mgr.snapshot(now_ns=120)
    assert mgr.resolve_event(event(mgr), now_ns=130).accepted
    assert mgr.resolve_event(event(mgr, event_id="event-2"), now_ns=131).reason == (
        "event_capacity_exhausted"
    )
    assert mgr.start_stage(
        request_id="extra",
        expected_stage_id="stage-1",
        official_start_stamp_ns=100,
        organizer_authorization_ref=AUTH,
        now_ns=131,
    ).reason == "request_capacity_exhausted"
    assert len(mgr.journal) == 4
    assert mgr.audit_dropped_count > 0


def test_request_id_reuse_with_different_payload_is_rejected():
    mgr = manager()
    accepted = start(mgr)
    conflict = start(mgr, stamp=101)

    assert accepted.accepted
    assert not conflict.accepted
    assert conflict.reason == "request_id_reused_with_different_payload"
    assert mgr.snapshot(now_ns=110).stage_started_at_ns == 100


@pytest.mark.parametrize(
    "kwargs, reason",
    [
        ({"expected_stage_id": "old"}, "stale_expected_stage_id"),
        ({"auth": "unapproved"}, "organizer_authorization_not_approved"),
        ({"stamp": 101, "now": 100}, "official_start_stamp_invalid"),
        ({"stamp": -1, "now": 100}, "official_start_stamp_invalid"),
    ],
)
def test_invalid_stage_start_inputs_do_not_grant_authority(kwargs, reason):
    mgr = manager()
    if "expected_stage_id" in kwargs:
        result = mgr.start_stage(
            request_id="start",
            expected_stage_id=kwargs["expected_stage_id"],
            official_start_stamp_ns=100,
            organizer_authorization_ref=AUTH,
            now_ns=100,
        )
    else:
        result = start(mgr, **kwargs)
    assert not result.accepted
    assert result.reason == reason
    assert mgr.snapshot(now_ns=100).phase == StagePhase.INIT


def test_zone_approval_reference_cannot_authorize_stage_start():
    mgr = manager()
    result = start(mgr, auth="zone-approval")
    assert not result.accepted
    assert result.reason == "organizer_authorization_not_approved"


def test_delayed_start_skips_freeze_or_times_out_without_transient_active():
    active = manager()
    assert start(active, now=150).accepted
    assert active.snapshot(now_ns=150).phase == StagePhase.ACTIVE
    active_records = active.journal
    start_record = next(record for record in active_records if record.operation == "start")
    transition_record = next(
        record for record in active_records if record.reason == "freeze_elapsed"
    )
    assert start_record.sequence < transition_record.sequence

    expired = manager()
    assert start(expired, now=200).accepted
    state = expired.snapshot(now_ns=200)
    assert state.phase == StagePhase.TERMINAL
    assert state.terminal_kind == TerminalKind.TIMEOUT
    assert not state.motion_authorized


def test_deadline_arithmetic_overflow_fails_closed_without_latching_start():
    mgr = manager()
    now = (1 << 63) - 10
    result = start(mgr, now=now, stamp=now)
    assert not result.accepted
    assert result.reason == "stage_deadline_overflow"
    assert mgr.phase == StagePhase.INIT


def test_exact_freeze_and_deadline_boundaries_are_deterministic():
    mgr = manager()
    start(mgr)
    assert mgr.snapshot(now_ns=119).phase == StagePhase.FREEZE
    assert mgr.snapshot(now_ns=120).phase == StagePhase.ACTIVE
    state = mgr.snapshot(now_ns=200)
    assert state.phase == StagePhase.TERMINAL
    assert state.terminal_kind == TerminalKind.TIMEOUT
    assert state.terminal_evidence == EventEvidence.ESTIMATE
    assert not state.motion_authorized


def test_time_reversal_and_invalid_snapshot_versions_are_rejected():
    mgr = manager()
    mgr.snapshot(now_ns=10)
    with pytest.raises(ValueError, match="monotonic"):
        mgr.snapshot(now_ns=9)
    with pytest.raises(ValueError, match="map_version"):
        mgr.snapshot(now_ns=10, map_version=True)


def test_match_state_header_is_leased_and_motion_authority_is_derived():
    mgr = manager()
    start(mgr)
    state = mgr.snapshot(now_ns=120, map_version=7, topology_version=9)

    assert state.motion_authorized is True
    assert state.meta.valid_until_ns == 145
    assert state.meta.map_version == 7
    assert state.meta.topology_version == 9
    validate_header(
        state.meta,
        now_ns=144,
        expected_clock_epoch="clock-1",
        expected_localization_epoch="localization-1",
        expected_stage_id="stage-1",
        require_stage=True,
    )
    with pytest.raises(ValueError, match="expired"):
        validate_header(
            state.meta,
            now_ns=145,
            expected_clock_epoch="clock-1",
            expected_localization_epoch="localization-1",
            expected_stage_id="stage-1",
        )


def test_match_state_contract_rejects_forged_authority_and_inconsistent_phase():
    mgr = manager()
    start(mgr)
    freeze_state = mgr.snapshot(now_ns=110)
    with pytest.raises(ValueError, match="motion authority"):
        replace(freeze_state, motion_authorized=True)

    active_state = mgr.snapshot(now_ns=120)
    with pytest.raises(ValueError, match="strictly ordered"):
        replace(active_state, freeze_ends_at_ns=active_state.stage_ends_at_ns)
    with pytest.raises(ValueError, match="nonterminal"):
        replace(active_state, terminal_kind=TerminalKind.CAPTURE)
    with pytest.raises(ValueError, match="TERMINAL"):
        replace(active_state, phase=StagePhase.TERMINAL)
    with pytest.raises(ValueError, match="stage ID"):
        replace(active_state, meta=replace(active_state.meta, stage_id=""))


def test_unresolved_zone_cannot_smuggle_geometry_or_semantic_identity():
    assert GoalZone("", ZonePurpose.TARGET, ZoneStatus.UNRESOLVED).zone_id == ""
    with pytest.raises(ValueError, match="must not carry"):
        GoalZone("guessed", ZonePurpose.TARGET, ZoneStatus.UNRESOLVED)
    with pytest.raises(ValueError, match="counterclockwise|simple|non-collinear"):
        GoalZone(
            "bad",
            ZonePurpose.TARGET,
            ZoneStatus.ACCEPTED,
            boundary=tuple(reversed(SQUARE)),
            frame_id="map",
            position_error_bound_m=0.02,
            provider="organizer",
            approval_ref="zone-approval",
            provenance_hash=ZONE_HASH,
        )


def test_zone_requires_valid_polygon_and_provenance():
    with pytest.raises(ValueError, match="simple|non-collinear"):
        GoalZone(
            zone_id="bad",
            purpose=ZonePurpose.TARGET,
            status=ZoneStatus.ACCEPTED,
            boundary=((0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)),
            frame_id="map",
            position_error_bound_m=0.02,
            provider="organizer",
            approval_ref="zone-approval",
            provenance_hash=ZONE_HASH,
        )
    with pytest.raises(ValueError, match="SHA-256"):
        GoalZone(
            "target-A",
            ZonePurpose.TARGET,
            ZoneStatus.ACCEPTED,
            SQUARE,
            frame_id="map",
            position_error_bound_m=0.02,
            provider="organizer",
            approval_ref="zone-approval",
            provenance_hash="bad",
        )


def test_zone_requests_are_stage_scoped_idempotent_and_require_approved_refs():
    mgr = manager()
    request = dict(
        request_id="zone",
        expected_stage_id="stage-1",
        zone=target_zone(),
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    assert mgr.set_goal_zone(**request).accepted
    assert mgr.set_goal_zone(**request).reason == "zone_recorded"
    state = mgr.snapshot(now_ns=0)
    assert state.target_zone_id == "target-A"
    bad = mgr.set_goal_zone(
        request_id="bad-zone",
        expected_stage_id="stage-1",
        zone=target_zone(zone_id="target-B", auth_ref="unapproved"),
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    assert not bad.accepted
    assert bad.reason == "zone_approval_ref_not_approved"
    wrong_frame = mgr.set_goal_zone(
        request_id="wrong-frame",
        expected_stage_id="stage-1",
        zone=GoalZone(
            zone_id="target-C",
            purpose=ZonePurpose.TARGET,
            status=ZoneStatus.ACCEPTED,
            boundary=SQUARE,
            frame_id="unregistered",
            position_error_bound_m=0.01,
            provider="organizer",
            approval_ref="zone-approval",
            provenance_hash=ZONE_HASH,
        ),
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    assert wrong_frame.reason == "zone_frame_not_approved"


def test_zone_uncertainty_must_be_finite_nonnegative_and_preserved():
    with pytest.raises(ValueError, match="position_error_bound_m"):
        GoalZone(
            zone_id="bad-error",
            purpose=ZonePurpose.TARGET,
            status=ZoneStatus.ACCEPTED,
            boundary=SQUARE,
            frame_id="map",
            position_error_bound_m=float("nan"),
            provider="organizer",
            approval_ref="zone-approval",
            provenance_hash=ZONE_HASH,
        )
    mgr = manager()
    zone = target_zone()
    mgr.set_goal_zone(
        request_id="zone",
        expected_stage_id="stage-1",
        zone=zone,
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    assert mgr.goal_zone(ZonePurpose.TARGET) == zone


def test_candidate_zone_keeps_geometry_but_never_becomes_authoritative_target():
    candidate = GoalZone(
        zone_id="candidate-target",
        purpose=ZonePurpose.TARGET,
        status=ZoneStatus.CANDIDATE,
        boundary=SQUARE,
        frame_id="map",
        position_error_bound_m=0.2,
    )
    mgr = manager()
    result = mgr.set_goal_zone(
        request_id="candidate",
        expected_stage_id="stage-1",
        zone=candidate,
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    assert result.accepted
    assert mgr.goal_zone(ZonePurpose.TARGET).status == ZoneStatus.CANDIDATE
    assert mgr.snapshot(now_ns=0).target_zone_id == ""


def test_candidate_zone_requires_frame_geometry_and_valid_uncertainty():
    with pytest.raises(ValueError, match="frame_id"):
        GoalZone(
            zone_id="candidate",
            purpose=ZonePurpose.TARGET,
            status=ZoneStatus.CANDIDATE,
            boundary=SQUARE,
            position_error_bound_m=0.0,
        )
    with pytest.raises(ValueError, match="finite and non-negative"):
        GoalZone(
            zone_id="candidate",
            purpose=ZonePurpose.TARGET,
            status=ZoneStatus.CANDIDATE,
            boundary=SQUARE,
            frame_id="map",
            position_error_bound_m=-0.01,
        )


def test_zone_permission_window_is_an_explicit_profile_bound():
    mgr = manager(zone_update_window_ns=10)
    start(mgr)
    result = mgr.set_goal_zone(
        request_id="too-late",
        expected_stage_id="stage-1",
        zone=target_zone(),
        organizer_authorization_ref=AUTH,
        now_ns=111,
    )
    assert result.reason == "zone_permission_window_closed"

    at_boundary = manager(zone_update_window_ns=10)
    start(at_boundary)
    accepted = at_boundary.set_goal_zone(
        request_id="at-boundary",
        expected_stage_id="stage-1",
        zone=target_zone(),
        organizer_authorization_ref=AUTH,
        now_ns=110,
    )
    assert accepted.accepted


def test_zone_can_be_updated_during_freeze_but_not_after_active():
    mgr = manager()
    start(mgr)
    assert mgr.set_goal_zone(
        request_id="zone",
        expected_stage_id="stage-1",
        zone=target_zone(),
        organizer_authorization_ref=AUTH,
        now_ns=110,
    ).accepted
    result = mgr.set_goal_zone(
        request_id="late-zone",
        expected_stage_id="stage-1",
        zone=target_zone(zone_id="late"),
        organizer_authorization_ref=AUTH,
        now_ns=120,
    )
    assert not result.accepted
    assert result.reason == "zone_updates_closed_after_FREEZE"


def test_missing_target_zone_blocks_arrival_event_without_invented_zone():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    result = mgr.resolve_event(
        event(mgr, kind=TerminalKind.ARRIVAL, zone_id="fixture-farthest-node"),
        now_ns=130,
    )
    assert not result.accepted
    assert result.reason == "arrival_target_zone_unresolved"
    assert mgr.snapshot(now_ns=130).phase == StagePhase.ACTIVE


def test_estimated_event_causes_only_local_hold_and_never_official_score():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    estimate = event(mgr, evidence=EventEvidence.ESTIMATE, lower=122, upper=124)
    result = mgr.resolve_event(estimate, now_ns=125)
    state = mgr.snapshot(now_ns=125)

    assert result.accepted
    assert result.reason == "estimate_latched_as_local_hold_only"
    assert state.phase == StagePhase.ACTIVE
    assert state.event_hold
    assert not state.motion_authorized
    assert state.terminal_kind == TerminalKind.NONE


def test_estimate_at_deadline_is_not_reported_as_active_hold():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    estimate = event(mgr, evidence=EventEvidence.ESTIMATE, lower=190, upper=199)
    result = mgr.resolve_event(estimate, now_ns=200)
    state = mgr.snapshot(now_ns=200)

    assert result.reason == "estimate_retained_for_adjudication_only"
    assert state.phase == StagePhase.TERMINAL
    assert state.terminal_kind == TerminalKind.TIMEOUT
    assert not state.event_hold
    assert not state.motion_authorized


def test_estimate_arriving_after_timeout_can_be_officially_confirmed_by_event_time():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=200)
    estimate = event(mgr, evidence=EventEvidence.ESTIMATE, lower=150, upper=160)
    result = mgr.resolve_event(estimate, now_ns=210)

    assert result.reason == "estimate_retained_for_adjudication_only"
    assert mgr.snapshot(now_ns=210).terminal_kind == TerminalKind.TIMEOUT
    confirmed = mgr.resolve_estimate(
        request_id="late-estimate-confirm",
        expected_stage_id="stage-1",
        event_id=estimate.event_id,
        resolution=EventResolution.CONFIRM_OFFICIAL,
        terminal_kind=TerminalKind.CAPTURE,
        organizer_authorization_ref=AUTH,
        now_ns=211,
    )
    assert confirmed.accepted
    state = mgr.snapshot(now_ns=211)
    assert state.terminal_kind == TerminalKind.CAPTURE
    assert state.terminal_evidence == EventEvidence.OFFICIAL


@pytest.mark.parametrize("predicate", [EventPredicate.FALSE, EventPredicate.UNKNOWN])
def test_false_or_unknown_predicate_cannot_certify_event(predicate):
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    result = mgr.resolve_event(event(mgr, predicate=predicate), now_ns=130)
    state = mgr.snapshot(now_ns=130)
    assert not result.accepted
    assert result.reason == "event_predicate_not_proven_true"
    assert state.motion_authorized
    assert state.terminal_kind == TerminalKind.NONE


def test_untrusted_official_event_and_stale_stage_epoch_are_rejected():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    assert not mgr.resolve_event(event(mgr, authorization="attacker"), now_ns=130).accepted
    stale = replace(event(mgr), stage_id="old-stage")
    assert mgr.resolve_event(stale, now_ns=130).reason == "stale_event_stage_id"

    stale_epoch = replace(
        event(mgr, event_id="old-epoch"),
        localization_epoch="old-localization",
    )
    assert mgr.resolve_event(stale_epoch, now_ns=130).reason == "stale_event_localization_epoch"


def test_official_event_requires_approved_provenance_and_interval_in_active_time():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    before_freeze = mgr.resolve_event(event(mgr, lower=119, upper=119), now_ns=130)
    future = mgr.resolve_event(event(mgr, event_id="future", lower=131, upper=132), now_ns=130)
    deadline = mgr.resolve_event(event(mgr, event_id="deadline", lower=198, upper=200), now_ns=200)

    assert before_freeze.reason == "event_precedes_active_phase"
    assert future.reason in ("event_time_in_future", "event_interval_extends_into_future")
    assert deadline.reason == "event_not_proven_strictly_before_deadline"
    assert mgr.snapshot(now_ns=200).terminal_kind == TerminalKind.TIMEOUT


def test_arrival_requires_accepted_matching_target_zone():
    mgr = manager()
    mgr.set_goal_zone(
        request_id="target",
        expected_stage_id="stage-1",
        zone=target_zone(),
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    start(mgr)
    mgr.snapshot(now_ns=120)
    wrong = mgr.resolve_event(
        event(mgr, kind=TerminalKind.ARRIVAL, zone_id="wrong-zone"),
        now_ns=130,
    )
    accepted = mgr.resolve_event(
        event(mgr, event_id="arrival", kind=TerminalKind.ARRIVAL, zone_id="target-A"),
        now_ns=130,
    )
    assert wrong.reason == "arrival_zone_id_mismatch"
    assert accepted.accepted
    assert mgr.snapshot(now_ns=130).terminal_kind == TerminalKind.ARRIVAL


def test_nonoverlapping_official_events_are_ordered_by_evidence_time():
    mgr = manager()
    start(mgr)
    mgr.set_goal_zone(
        request_id="target",
        expected_stage_id="stage-1",
        zone=target_zone(),
        organizer_authorization_ref=AUTH,
        now_ns=110,
    )
    mgr.snapshot(now_ns=120)
    later = event(mgr, event_id="arrival", kind=TerminalKind.CAPTURE, lower=150, upper=155)
    earlier = event(mgr, event_id="capture", kind=TerminalKind.ARRIVAL, lower=125, upper=130)
    earlier = RuleEvent(
        **(earlier.__dict__ | {"zone_id": "target-A"})
    )
    assert mgr.resolve_event(later, now_ns=160).accepted
    assert mgr.resolve_event(earlier, now_ns=160).accepted
    state = mgr.snapshot(now_ns=160)
    assert state.terminal_kind == TerminalKind.ARRIVAL
    assert state.terminal_evidence == EventEvidence.OFFICIAL


def test_overlapping_terminal_intervals_remain_ambiguous_not_arbitrarily_scored():
    mgr = manager()
    mgr.set_goal_zone(
        request_id="target",
        expected_stage_id="stage-1",
        zone=target_zone(),
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    start(mgr)
    mgr.snapshot(now_ns=120)
    capture = event(mgr, event_id="capture", lower=130, upper=140)
    arrival = event(
        mgr,
        event_id="arrival",
        kind=TerminalKind.ARRIVAL,
        lower=139,
        upper=145,
        zone_id="target-A",
    )
    assert mgr.resolve_event(capture, now_ns=150).accepted
    assert mgr.resolve_event(arrival, now_ns=150).accepted
    state = mgr.snapshot(now_ns=150)
    assert state.phase == StagePhase.TERMINAL
    assert state.terminal_kind == TerminalKind.AMBIGUOUS
    assert state.event_hold
    assert not state.motion_authorized


def test_terminal_event_order_is_invariant_to_delivery_order():
    def run(reverse):
        mgr = manager()
        mgr.set_goal_zone(
            request_id="target",
            expected_stage_id="stage-1",
            zone=target_zone(),
            organizer_authorization_ref=AUTH,
            now_ns=0,
        )
        start(mgr)
        mgr.snapshot(now_ns=120)
        events = [
            event(mgr, event_id="capture", lower=130, upper=140),
            event(
                mgr,
                event_id="arrival",
                kind=TerminalKind.ARRIVAL,
                lower=139,
                upper=145,
                zone_id="target-A",
            ),
        ]
        for item in reversed(events) if reverse else events:
            assert mgr.resolve_event(item, now_ns=150).accepted
        state = mgr.snapshot(now_ns=150)
        return state.terminal_kind, state.event_hold, state.terminal_evidence

    assert run(False) == run(True) == (
        TerminalKind.AMBIGUOUS,
        True,
        EventEvidence.OFFICIAL,
    )


def test_conflicting_estimates_keep_local_hold_until_both_are_resolved():
    mgr = manager()
    mgr.set_goal_zone(
        request_id="target",
        expected_stage_id="stage-1",
        zone=target_zone(),
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    start(mgr)
    mgr.snapshot(now_ns=120)
    capture = event(mgr, event_id="capture", evidence=EventEvidence.ESTIMATE, lower=130, upper=140)
    arrival = event(
        mgr,
        event_id="arrival",
        kind=TerminalKind.ARRIVAL,
        evidence=EventEvidence.ESTIMATE,
        lower=139,
        upper=145,
        zone_id="target-A",
    )
    assert mgr.resolve_event(capture, now_ns=145).accepted
    assert mgr.resolve_event(arrival, now_ns=145).accepted
    state = mgr.snapshot(now_ns=145)
    assert state.phase == StagePhase.ACTIVE
    assert state.event_hold
    assert state.terminal_kind == TerminalKind.NONE
    assert not state.motion_authorized

    first = mgr.resolve_estimate(
        request_id="confirm-overlapping",
        expected_stage_id="stage-1",
        event_id="capture",
        resolution=EventResolution.CONFIRM_OFFICIAL,
        terminal_kind=TerminalKind.CAPTURE,
        organizer_authorization_ref=AUTH,
        now_ns=146,
    )
    assert first.reason == "conflicting_estimate_requires_resolution"
    dismissed = mgr.resolve_estimate(
        request_id="dismiss-arrival",
        expected_stage_id="stage-1",
        event_id="arrival",
        resolution=EventResolution.DISMISS_ESTIMATE,
        terminal_kind=TerminalKind.ARRIVAL,
        organizer_authorization_ref=AUTH,
        now_ns=147,
    )
    assert dismissed.accepted
    assert mgr.resolve_estimate(
        request_id="confirm-capture",
        expected_stage_id="stage-1",
        event_id="capture",
        resolution=EventResolution.CONFIRM_OFFICIAL,
        terminal_kind=TerminalKind.CAPTURE,
        organizer_authorization_ref=AUTH,
        now_ns=148,
    ).accepted
    assert mgr.snapshot(now_ns=148).terminal_kind == TerminalKind.CAPTURE


def test_timeout_then_delayed_predeadline_official_event_reconciles_by_event_time():
    mgr = manager()
    start(mgr)
    assert mgr.snapshot(now_ns=200).terminal_kind == TerminalKind.TIMEOUT
    delayed = event(mgr, lower=150, upper=160)
    result = mgr.resolve_event(delayed, now_ns=220)
    state = mgr.snapshot(now_ns=220)

    assert result.accepted
    assert state.terminal_kind == TerminalKind.CAPTURE
    assert state.terminal_evidence == EventEvidence.OFFICIAL
    assert not state.motion_authorized


def test_official_predeadline_event_is_processed_before_delayed_timeout_transition():
    mgr = manager()
    start(mgr)
    delayed = event(mgr, lower=150, upper=160)
    result = mgr.resolve_event(delayed, now_ns=200)
    state = mgr.snapshot(now_ns=200)

    assert result.accepted
    assert state.phase == StagePhase.TERMINAL
    assert state.terminal_kind == TerminalKind.CAPTURE
    assert state.terminal_evidence == EventEvidence.OFFICIAL


def test_overlapping_deadline_interval_does_not_become_capture():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=200)
    result = mgr.resolve_event(event(mgr, lower=195, upper=200), now_ns=210)
    state = mgr.snapshot(now_ns=210)

    assert not result.accepted
    assert state.terminal_kind == TerminalKind.TIMEOUT
    assert state.terminal_evidence == EventEvidence.ESTIMATE


def test_event_idempotency_and_payload_conflict_do_not_duplicate_terminal():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    original = event(mgr)
    assert mgr.resolve_event(original, now_ns=130).accepted
    assert mgr.resolve_event(original, now_ns=140).reason == "duplicate_event_idempotent"
    conflicting = event(mgr, lower=121, upper=121)
    assert mgr.resolve_event(conflicting, now_ns=140).reason == "event_id_reused_with_different_payload"


def test_estimate_can_be_dismissed_or_confirmed_only_by_authorized_resolution():
    dismissed = manager()
    start(dismissed)
    dismissed.snapshot(now_ns=120)
    estimate = event(dismissed, evidence=EventEvidence.ESTIMATE)
    dismissed.resolve_event(estimate, now_ns=130)
    rejected = dismissed.resolve_estimate(
        request_id="dismiss-unauthorized",
        expected_stage_id="stage-1",
        event_id="event-1",
        resolution=EventResolution.DISMISS_ESTIMATE,
        terminal_kind=TerminalKind.CAPTURE,
        organizer_authorization_ref="bad",
        now_ns=131,
    )
    assert not rejected.accepted
    result = dismissed.resolve_estimate(
        request_id="dismiss",
        expected_stage_id="stage-1",
        event_id="event-1",
        resolution=EventResolution.DISMISS_ESTIMATE,
        terminal_kind=TerminalKind.CAPTURE,
        organizer_authorization_ref=AUTH,
        now_ns=132,
    )
    assert result.accepted
    assert dismissed.snapshot(now_ns=132).motion_authorized

    confirmed = manager()
    start(confirmed)
    confirmed.snapshot(now_ns=120)
    confirmed.resolve_event(event(confirmed, evidence=EventEvidence.ESTIMATE), now_ns=130)
    result = confirmed.resolve_estimate(
        request_id="confirm",
        expected_stage_id="stage-1",
        event_id="event-1",
        resolution=EventResolution.CONFIRM_OFFICIAL,
        terminal_kind=TerminalKind.CAPTURE,
        organizer_authorization_ref=AUTH,
        now_ns=131,
    )
    state = confirmed.snapshot(now_ns=131)
    assert result.accepted
    assert state.phase == StagePhase.TERMINAL
    assert state.terminal_kind == TerminalKind.CAPTURE
    assert state.terminal_evidence == EventEvidence.OFFICIAL


def test_confirmed_predeadline_estimate_survives_timeout_delivery_delay():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    mgr.resolve_event(event(mgr, evidence=EventEvidence.ESTIMATE, lower=150, upper=160), now_ns=160)
    assert mgr.snapshot(now_ns=200).terminal_kind == TerminalKind.TIMEOUT
    result = mgr.resolve_estimate(
        request_id="late-confirm",
        expected_stage_id="stage-1",
        event_id="event-1",
        resolution=EventResolution.CONFIRM_OFFICIAL,
        terminal_kind=TerminalKind.CAPTURE,
        organizer_authorization_ref=AUTH,
        now_ns=220,
    )
    assert result.accepted
    assert mgr.snapshot(now_ns=220).terminal_kind == TerminalKind.CAPTURE


def test_authorized_official_terminal_cannot_overwrite_final_official_result():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    mgr.resolve_event(event(mgr), now_ns=130)
    result = mgr.resolve_estimate(
        request_id="override",
        expected_stage_id="stage-1",
        event_id="unknown-event",
        resolution=EventResolution.SET_OFFICIAL_TERMINAL,
        terminal_kind=TerminalKind.OFFICIAL_ABORT,
        organizer_authorization_ref=AUTH,
        now_ns=131,
    )
    assert not result.accepted
    assert result.reason == "official_terminal_already_latched"
    assert mgr.snapshot(now_ns=131).terminal_kind == TerminalKind.CAPTURE


def test_direct_terminal_capture_resolution_requires_official_event_evidence():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    estimate = event(mgr, evidence=EventEvidence.ESTIMATE)
    mgr.resolve_event(estimate, now_ns=130)
    result = mgr.resolve_estimate(
        request_id="forge-capture",
        expected_stage_id="stage-1",
        event_id=estimate.event_id,
        resolution=EventResolution.SET_OFFICIAL_TERMINAL,
        terminal_kind=TerminalKind.CAPTURE,
        organizer_authorization_ref=AUTH,
        now_ns=131,
    )
    assert result.reason == "matching_adjudication_evidence_required"
    assert mgr.snapshot(now_ns=131).phase == StagePhase.ACTIVE
    assert mgr.snapshot(now_ns=131).event_hold


def test_sim_truth_terminal_requires_an_explicit_test_profile():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    denied = mgr.resolve_event(event(mgr, evidence=EventEvidence.SIM_TRUTH), now_ns=130)
    assert denied.reason == "simulation_truth_terminal_not_enabled"

    sim = manager(allow_sim_truth_terminal=True)
    start(sim)
    sim.snapshot(now_ns=120)
    accepted = sim.resolve_event(event(sim, evidence=EventEvidence.SIM_TRUTH), now_ns=130)
    assert accepted.accepted
    assert sim.snapshot(now_ns=130).terminal_evidence == EventEvidence.SIM_TRUTH


def test_reset_requires_terminal_authority_and_approved_retention():
    init_manager = manager()
    init_reset = init_manager.reset_stage(
        request_id="init-reset",
        expected_stage_id="stage-1",
        stage_number=1,
        role=Role.EXPLORER,
        memory_profile_id="",
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    assert init_reset.accepted

    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=200)
    bad_memory = mgr.reset_stage(
        request_id="bad-memory",
        expected_stage_id="stage-1",
        stage_number=2,
        role=Role.GUARDIAN,
        memory_profile_id="unapproved",
        organizer_authorization_ref=AUTH,
        now_ns=200,
    )
    assert not bad_memory.accepted
    result = mgr.reset_stage(
        request_id="reset",
        expected_stage_id="stage-1",
        stage_number=2,
        role=Role.GUARDIAN,
        memory_profile_id="retention-approved",
        organizer_authorization_ref=AUTH,
        now_ns=200,
    )
    assert result.accepted
    assert result.stage_id == "stage-2"
    assert result.reset_effects_required == RESET_ACKS
    state = mgr.snapshot(now_ns=200)
    assert state.phase == StagePhase.INIT
    assert state.role == Role.GUARDIAN
    assert not state.motion_authorized
    assert state.own_start_zone_id == state.target_zone_id == ""


def test_reset_barrier_blocks_start_until_all_downstream_effects_acknowledged():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=200)
    reset = mgr.reset_stage(
        request_id="reset",
        expected_stage_id="stage-1",
        stage_number=2,
        role=Role.GUARDIAN,
        memory_profile_id="",
        organizer_authorization_ref=AUTH,
        now_ns=200,
    )
    assert mgr.acknowledge_reset(
        stage_id=reset.stage_id,
        acknowledgements=frozenset(("actions", "execution", "plans")),
    ) is False
    assert not start(mgr, request_id="new-start", now=201).accepted
    with pytest.raises(ValueError, match="unknown"):
        mgr.acknowledge_reset(
            stage_id=reset.stage_id,
            acknowledgements=frozenset(("actions", "execution", "plans", "motor")),
        )
    assert mgr.acknowledge_reset(
        stage_id=reset.stage_id,
        acknowledgements=frozenset(("candidates", "tracks", "memory")),
    ) is True
    assert start(mgr, request_id="new-start-after-ack", now=201).accepted


def test_reset_request_retry_is_idempotent_and_cannot_reuse_id():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=200)
    kwargs = dict(
        request_id="reset",
        expected_stage_id="stage-1",
        stage_number=2,
        role=Role.GUARDIAN,
        memory_profile_id="",
        organizer_authorization_ref=AUTH,
        now_ns=200,
    )
    first = mgr.reset_stage(**kwargs)
    assert mgr.reset_stage(**kwargs) == first
    conflict = mgr.reset_stage(**(kwargs | {"stage_number": 3}))
    assert not conflict.accepted
    assert conflict.reason == "request_id_reused_with_different_payload"


def test_stage_ids_must_never_be_reused_even_after_multiple_resets():
    identifiers = iter(("stage-2", "stage-1"))
    mgr = StageManager(
        profile(),
        initial_stage_id="stage-1",
        stage_number=1,
        role=Role.EXPLORER,
        stage_id_factory=lambda: next(identifiers),
    )
    first = mgr.reset_stage(
        request_id="reset-init",
        expected_stage_id="stage-1",
        stage_number=1,
        role=Role.EXPLORER,
        memory_profile_id="",
        organizer_authorization_ref=AUTH,
        now_ns=0,
    )
    assert first.accepted
    assert mgr.acknowledge_reset(
        stage_id="stage-2", acknowledgements=RESET_ACKS
    )
    start(mgr, request_id="start-2", now=10, stamp=10)
    mgr.snapshot(now_ns=110)
    second = mgr.reset_stage(
        request_id="reset-terminal",
        expected_stage_id="stage-2",
        stage_number=2,
        role=Role.GUARDIAN,
        memory_profile_id="",
        organizer_authorization_ref=AUTH,
        now_ns=110,
    )
    assert not second.accepted
    assert second.reason == "stage_id_factory_reused_id"
    assert mgr.stage_id == "stage-2"


def test_localization_epoch_change_invalidates_old_events_and_clears_estimate_hold():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    old = event(mgr, evidence=EventEvidence.ESTIMATE, lower=125, upper=125)
    mgr.resolve_event(old, now_ns=130)
    change = mgr.change_localization_epoch(new_epoch="localization-2")
    assert change.reset_effects_required == EPOCH_ACKS
    state = mgr.snapshot(now_ns=130)
    assert state.meta.localization_epoch == "localization-2"
    assert not state.motion_authorized
    assert not state.event_hold
    assert mgr.resolve_event(old, now_ns=131).reason == "stale_event_localization_epoch"
    assert mgr.acknowledge_localization_epoch(
        stage_id=change.stage_id,
        acknowledgements=frozenset(("actions", "plans", "candidates")),
    ) is False
    assert mgr.acknowledge_localization_epoch(
        stage_id=change.stage_id, acknowledgements=frozenset(("tracks", "map", "world"))
    ) is True
    assert mgr.snapshot(now_ns=131).motion_authorized


def test_journal_preserves_requests_transition_and_estimated_vs_official_evidence():
    mgr = manager()
    start(mgr)
    mgr.snapshot(now_ns=120)
    estimate = event(mgr, evidence=EventEvidence.ESTIMATE)
    mgr.resolve_event(estimate, now_ns=130)
    mgr.resolve_estimate(
        request_id="confirm",
        expected_stage_id="stage-1",
        event_id=estimate.event_id,
        resolution=EventResolution.CONFIRM_OFFICIAL,
        terminal_kind=TerminalKind.CAPTURE,
        organizer_authorization_ref=AUTH,
        now_ns=131,
    )
    journal = mgr.journal
    rule_events = [record for record in journal if record.operation == "rule_event"]
    resolution = [record for record in journal if record.operation == "resolve"]

    assert [record.sequence for record in journal] == sorted(record.sequence for record in journal)
    assert rule_events[0].evidence == EventEvidence.ESTIMATE
    assert rule_events[0].input_ids == ("evidence-1",)
    assert resolution[0].evidence == EventEvidence.OFFICIAL
    assert mgr.snapshot(now_ns=132).terminal_kind == TerminalKind.CAPTURE


def test_clock_epoch_change_disarms_stage_and_requires_new_stage_and_reset_barrier():
    mgr = manager()
    start(mgr)
    changed = mgr.change_clock_epoch(new_epoch="clock-2", now_ns=0)
    state = mgr.snapshot(now_ns=0)
    assert changed.accepted
    assert state.meta.clock_epoch == "clock-2"
    assert state.phase == StagePhase.INIT
    assert not state.motion_authorized
    assert state.meta.stage_id == "stage-2"
    assert not start(mgr, request_id="blocked", now=1).accepted
    assert mgr.acknowledge_reset(
        stage_id="stage-2",
        acknowledgements=frozenset(("actions", "execution", "plans", "candidates")),
    ) is False
    assert mgr.acknowledge_reset(
        stage_id="stage-2", acknowledgements=frozenset(("tracks", "memory"))
    ) is True
    assert start(mgr, request_id="new", now=1, stamp=1).accepted


def test_no_safety_fault_is_fabricated_into_official_abort():
    mgr = manager()
    start(mgr)
    active = mgr.snapshot(now_ns=120)
    assert active.phase == StagePhase.ACTIVE
    assert active.terminal_kind == TerminalKind.NONE
    assert active.motion_authorized
