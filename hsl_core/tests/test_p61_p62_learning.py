"""Adversarial contract and mathematical tests for offline learning P6.1/P6.2."""

from dataclasses import FrozenInstanceError, replace
import json
import math

import pytest

from hsl_core.learning.dirichlet import (
    REQUIRED_FAILURE_OUTCOMES,
    DirichletBeliefTransition,
    make_dirichlet_row,
)
from hsl_core.learning.option_value import (
    ExactOptionModel,
    KernelAtom,
    KernelEvidenceKind,
    OptionValueTable,
    assess_exact_option_fusion,
    discounted_return,
)
from hsl_core.learning.transitions import (
    BaselineEvidence,
    CensoringStatus,
    FeatureSchema,
    OptionTransition,
    RewardComponent,
    RewardRateSegment,
    SplitAssignment,
    TransitionDataset,
    feature_state,
)
from hsl_core.match import Role
from hsl_core.tactics import OptionKind, OptionOutcome


_HASH = "a" * 64
_SCHEMA = FeatureSchema(
    "features-v1",
    ("confidence", "distance"),
    bin_edges=(("confidence", (0.5,)), ("distance", (0.5,))),
)


def _transition(
    *,
    episode_id="episode-1",
    instance_id="option-1",
    start_ns=10_000_000_000,
    end_ns=12_000_000_000,
    terminal=False,
    censoring=CensoringStatus.COMPLETE,
    outcome=OptionOutcome.SUCCESS,
    option_kind=OptionKind.SEARCH_PORTAL,
    next_state=None,
    available=(OptionKind.HOLD_SAFE,),
    rewards=(),
    banks=("map-training", "opponent-training", "seed-training"),
    schema_version=1,
):
    state = feature_state(
        _SCHEMA, Role.GUARDIAN, {"confidence": 0.25, "distance": 0.5}
    )
    if censoring is not CensoringStatus.COMPLETE:
        outcome = None
        terminal = False
        next_state = None
        available = ()
    elif terminal:
        next_state = None
        available = ()
    elif next_state is None:
        next_state = feature_state(
            _SCHEMA, Role.GUARDIAN, {"confidence": 0.75, "distance": 0.25}
        )
    return OptionTransition(
        episode_id=episode_id,
        stage_id="stage-1",
        role=Role.GUARDIAN,
        input_state=state,
        option_instance_id=instance_id,
        option_kind=option_kind,
        parameters_sha256=_HASH,
        start_ns=start_ns,
        end_ns=end_ns,
        outcome=outcome,
        terminal=terminal,
        next_state=next_state,
        available_next_options=available,
        reward_components=tuple(rewards),
        safety_intervention_ids=(),
        official_event_id="",
        official_event_evidence=None,
        policy_sha256=_HASH,
        sensor_fidelity_id="sensor-sim",
        map_fidelity_id="map-sim",
        scenario_id="scenario-1",
        opponent_id="opponent-1",
        seed=1,
        map_bank_id=banks[0],
        opponent_bank_id=banks[1],
        seed_bank_id=banks[2],
        source_profile_id="profile-1",
        censoring=censoring,
        schema_version=schema_version,
    )


def _required_outcomes():
    return tuple(sorted(REQUIRED_FAILURE_OUTCOMES | {"SUCCESS"}))


def test_dirichlet_posterior_mean_variance_and_immutable_update():
    row = make_dirichlet_row(
        state_id="s0",
        option_id="search",
        outcome_schema_id="outcomes-v1",
        outcome_ids=_required_outcomes(),
        alpha_prior=(1.0,) * len(_required_outcomes()),
    )
    updated = row.observe("SUCCESS")

    assert row.outcome_counts == (0,) * len(_required_outcomes())
    assert sum(updated.outcome_counts) == 1
    assert updated.posterior_mean() == pytest.approx(
        tuple((2.0 if item == "SUCCESS" else 1.0) / 9.0 for item in row.outcome_ids)
    )
    alpha = updated.posterior_alpha
    total = math.fsum(alpha)
    expected_variance = tuple(
        value * (total - value) / (total**2 * (total + 1.0)) for value in alpha
    )
    assert updated.posterior_variance() == pytest.approx(expected_variance)
    assert math.fsum(updated.posterior_mean()) == pytest.approx(1.0)
    with pytest.raises(FrozenInstanceError):
        row.state_id = "mutated"
    large_prior = make_dirichlet_row(
        state_id="large",
        option_id="search",
        outcome_schema_id="outcomes-v1",
        outcome_ids=_required_outcomes(),
        alpha_prior=(1e200,) * len(_required_outcomes()),
    )
    assert all(math.isfinite(value) for value in large_prior.posterior_variance())


@pytest.mark.parametrize(
    "changes",
    [
        {"outcome_ids": ("SUCCESS",), "alpha_prior": (1.0,)},
        {"alpha_prior": (0.0,) * len(_required_outcomes())},
        {"outcome_counts": (-1,) + (0,) * (len(_required_outcomes()) - 1)},
        {"outcome_counts": (True,) + (0,) * (len(_required_outcomes()) - 1)},
    ],
)
def test_dirichlet_rejects_invalid_schema_prior_and_counts(changes):
    args = dict(
        state_id="s0",
        option_id="search",
        outcome_schema_id="outcomes-v1",
        outcome_ids=_required_outcomes(),
        alpha_prior=(1.0,) * len(_required_outcomes()),
    )
    args.update(changes)
    with pytest.raises(ValueError):
        DirichletBeliefTransition(**args)


def test_dirichlet_rejects_unregistered_outcomes_and_count_overflow():
    row = make_dirichlet_row(
        state_id="s0",
        option_id="search",
        outcome_schema_id="outcomes-v1",
        outcome_ids=_required_outcomes(),
        alpha_prior=(1.0,) * len(_required_outcomes()),
    )
    with pytest.raises(ValueError, match="not registered"):
        row.observe("NOT_REGISTERED")
    counts = list(row.outcome_counts)
    counts[0] = (1 << 64) - 1
    saturated = DirichletBeliefTransition(
        row.state_id, row.option_id, row.outcome_schema_id,
        row.outcome_ids, row.alpha_prior, tuple(counts)
    )
    with pytest.raises(ValueError, match="overflow"):
        saturated.observe(row.outcome_ids[0])


def test_feature_schema_rejects_truth_oracle_and_noncanonical_inputs():
    with pytest.raises(ValueError, match="prohibited"):
        FeatureSchema("unsafe", ("ground_truth_pose",))
    with pytest.raises(ValueError, match="canonical"):
        FeatureSchema("unordered", ("z", "a"))
    with pytest.raises(ValueError, match="exactly match"):
        feature_state(_SCHEMA, Role.GUARDIAN, {"confidence": 0.2})
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        feature_state(_SCHEMA, Role.GUARDIAN, {"confidence": 1.1, "distance": 0.2})


def test_binned_abstraction_is_schema_bound_and_deterministic():
    first = feature_state(
        _SCHEMA, Role.GUARDIAN, {"confidence": 0.1, "distance": 0.3}
    )
    second = feature_state(
        _SCHEMA, Role.GUARDIAN, {"confidence": 0.4, "distance": 0.49}
    )
    changed_bin = feature_state(
        _SCHEMA, Role.GUARDIAN, {"confidence": 0.5, "distance": 0.3}
    )
    assert first.feature_bins == second.feature_bins
    assert first.state_id == second.state_id
    assert first.state_id != changed_bin.state_id
    assert first.to_dict()["features"] != second.to_dict()["features"]
    different_edges = FeatureSchema(
        "features-v1",
        ("confidence", "distance"),
        bin_edges=(("confidence", (0.25,)), ("distance", (0.5,))),
    )
    with pytest.raises(ValueError, match="fingerprint"):
        first.validate(different_edges)


@pytest.mark.parametrize(
    "changes",
    [
        {"end_ns": 10_000_000_000},
        {"parameters_sha256": "not-a-hash"},
        {"policy_sha256": "A" * 64},
        {"seed": -1},
        {"schema_version": True},
    ],
)
def test_transition_rejects_invalid_duration_hashes_and_state(changes):
    with pytest.raises((ValueError, TypeError)):
        _transition(**changes)


def test_transition_rejects_terminal_bootstrap_and_value_table_schema_collision():
    terminal = _transition(terminal=True)
    with pytest.raises(ValueError, match="terminal transition"):
        replace(terminal, next_state=terminal.input_state)

    conflicting_schema = FeatureSchema(
        _SCHEMA.schema_id,
        ("confidence", "different_feature"),
        bin_edges=(("confidence", (0.5,)), ("different_feature", (0.5,))),
    )
    table = OptionValueTable(conflicting_schema, (OptionKind.SEARCH_PORTAL,))
    with pytest.raises(ValueError, match="mismatch"):
        table.update(
            _transition(terminal=True),
            beta_per_s=0.0,
            eta=1.0,
            eta_schedule_id="schedule",
        )


@pytest.mark.parametrize(
    "censoring",
    [CensoringStatus.CENSORED, CensoringStatus.TRUNCATED],
)
def test_censored_and_truncated_samples_are_not_fabricated_as_complete(censoring):
    transition = _transition(censoring=censoring)
    assert transition.outcome is None
    assert transition.next_state is None
    assert not transition.terminal
    assert transition.available_next_options == ()
    with pytest.raises(ValueError, match="complete option return"):
        discounted_return(transition, beta_per_s=0.2)
    with pytest.raises(ValueError, match="censored"):
        OptionValueTable(_SCHEMA, (OptionKind.SEARCH_PORTAL,)).update(
            transition, beta_per_s=0.2, eta=0.5, eta_schedule_id="eta-v1"
        )


def test_reward_segments_are_bounded_nonoverlapping_and_exactly_integrated():
    with pytest.raises(ValueError, match="overlap"):
        RewardComponent(
            "bad",
            (
                RewardRateSegment(0, 1_000_000_000, 1.0),
                RewardRateSegment(500_000_000, 1_500_000_000, 1.0),
            ),
        )
    with pytest.raises(ValueError, match="exceeds"):
        _transition(
            rewards=(RewardComponent("late", (RewardRateSegment(0, 3_000_000_000, 1.0),)),)
        )

    transition = _transition(
        terminal=True,
        rewards=(
            RewardComponent(
                "progress",
                (RewardRateSegment(0, 1_000_000_000, 2.0),),
                terminal_impulse=3.0,
            ),
            RewardComponent(
                "penalty",
                (RewardRateSegment(1_000_000_000, 2_000_000_000, -1.0),),
                terminal_impulse=-0.5,
            ),
        ),
    )
    undiscounted = discounted_return(transition, beta_per_s=0.0)
    assert undiscounted.reward == pytest.approx(3.5)
    assert undiscounted.discount == 1.0
    beta = 0.7
    result = discounted_return(transition, beta_per_s=beta)
    expected = (
        2.0 * (1.0 - math.exp(-beta)) / beta
        - (math.exp(-beta) - math.exp(-2.0 * beta)) / beta
        + 2.5 * math.exp(-2.0 * beta)
    )
    assert result.reward == pytest.approx(expected, rel=1e-13)
    assert result.discount == pytest.approx(math.exp(-2.0 * beta))
    assert result.duration_s == pytest.approx(2.0)


def test_smdp_backup_terminal_nonterminal_eta_and_parameter_guards():
    option = OptionKind.SEARCH_PORTAL
    table = OptionValueTable(_SCHEMA, (option, OptionKind.HOLD_SAFE))
    next_state = feature_state(
        _SCHEMA, Role.GUARDIAN, {"confidence": 0.25, "distance": 0.5}
    )
    seed_value = _transition(
        episode_id="seed-value",
        instance_id="seed-hold",
        terminal=True,
        option_kind=OptionKind.HOLD_SAFE,
        rewards=(RewardComponent("seed", terminal_impulse=4.0),),
    )
    table.update(seed_value, beta_per_s=0.0, eta=1.0, eta_schedule_id="seed")
    transition = _transition(
        rewards=(RewardComponent("reward", terminal_impulse=2.0),),
        next_state=next_state,
        available=(OptionKind.HOLD_SAFE,),
    )
    update = table.update(
        transition, beta_per_s=math.log(2.0) / 2.0, eta=0.25, eta_schedule_id="schedule-A"
    )
    assert update.target == pytest.approx(3.0)
    assert update.new_value == pytest.approx(0.75)
    assert update.eta_schedule_id == "schedule-A"

    terminal = _transition(
        episode_id="episode-terminal",
        instance_id="terminal-option",
        terminal=True,
        rewards=(RewardComponent("terminal", terminal_impulse=3.0),),
    )
    terminal_update = table.update(
        terminal, beta_per_s=0.0, eta=1.0, eta_schedule_id="schedule-B"
    )
    assert terminal_update.target == pytest.approx(3.0)
    assert terminal_update.new_value == pytest.approx(3.0)
    assert table.snapshot()

    with pytest.raises(ValueError, match="eta"):
        table.update(terminal, beta_per_s=0.0, eta=0.0, eta_schedule_id="schedule")
    with pytest.raises(ValueError, match="identifier"):
        table.update(terminal, beta_per_s=0.0, eta=0.5, eta_schedule_id=" ")
    with pytest.raises(ValueError, match="discount rate"):
        discounted_return(terminal, beta_per_s=-1.0)


def test_dataset_rejects_duplicate_transition_and_episode_split_leakage():
    record = _transition()
    changed_same_identity = _transition(
        end_ns=13_000_000_000,
        rewards=(RewardComponent("different", terminal_impulse=1.0),),
    )
    with pytest.raises(ValueError, match="duplicate"):
        TransitionDataset(_SCHEMA, (record, changed_same_identity))

    records = (
        _transition(episode_id="shared-episode", instance_id="one"),
        _transition(
            episode_id="shared-episode",
            instance_id="two",
            banks=("map-validation", "opponent-validation", "seed-validation"),
        ),
    )
    splits = SplitAssignment(
        map_banks=(("map-training", "training"), ("map-validation", "validation")),
        opponent_banks=(("opponent-training", "training"), ("opponent-validation", "validation")),
        seed_banks=(("seed-training", "training"), ("seed-validation", "validation")),
    )
    with pytest.raises(ValueError, match="episode"):
        TransitionDataset(_SCHEMA, records, splits=splits)


def test_split_assignment_normalizes_nested_mutable_input():
    map_assignments = [["map-training", "training"]]
    splits = SplitAssignment(
        map_banks=map_assignments,
        opponent_banks=[["opponent-training", "training"]],
        seed_banks=[["seed-training", "training"]],
    )
    map_assignments[0][0] = "mutated"
    assert splits.map_banks == (("map-training", "training"),)


def test_dataset_manifest_never_promotes_empty_or_incomplete_evidence():
    empty = TransitionDataset(_SCHEMA, ())
    assert not empty.manifest().promotion_eligible
    assert "accepted_g4_baseline_evidence_missing" in empty.manifest().ineligibility_reasons

    baseline = BaselineEvidence(
        "profile-1", "PASS", _HASH, _HASH, _HASH, _HASH
    )
    three_records = tuple(
        _transition(
            episode_id=f"episode-{split}",
            instance_id=f"option-{split}",
            banks=(f"map-{split}", f"opponent-{split}", f"seed-{split}"),
        )
        for split in ("training", "validation", "held_out")
    )
    assignments = tuple((f"{kind}-{split}", split) for kind in ("map", "opponent", "seed") for split in ("training", "validation", "held_out"))
    splits = SplitAssignment(
        map_banks=tuple(pair for pair in assignments if pair[0].startswith("map-")),
        opponent_banks=tuple(pair for pair in assignments if pair[0].startswith("opponent-")),
        seed_banks=tuple(pair for pair in assignments if pair[0].startswith("seed-")),
    )
    manifest = TransitionDataset(
        _SCHEMA, three_records, splits=splits, baseline=baseline
    ).manifest()
    assert manifest.split_record_counts == (("held_out", 1), ("training", 1), ("validation", 1))
    assert manifest.contract_valid
    assert not manifest.promotion_eligible
    assert "dataset_quality_and_promotion_review_not_assessed" in manifest.ineligibility_reasons
    serialized = json.loads(json.dumps(manifest.to_dict(), sort_keys=True))
    assert serialized["feature_schema_sha256"] == _SCHEMA.fingerprint
    reordered = TransitionDataset(
        _SCHEMA, reversed(three_records), splits=splits, baseline=baseline
    )
    assert reordered.manifest().dataset_sha256 == manifest.dataset_sha256
    assert reordered.to_jsonl() == TransitionDataset(
        _SCHEMA, three_records, splits=splits, baseline=baseline
    ).to_jsonl()

    blocked_baseline = BaselineEvidence(
        "profile-1", "BLOCKED_NOT_RUN", _HASH, _HASH, _HASH, _HASH
    )
    blocked = TransitionDataset(
        _SCHEMA, three_records, splits=splits, baseline=blocked_baseline
    ).manifest()
    assert not blocked.promotion_eligible
    assert "g4_baseline_not_accepted" in blocked.ineligibility_reasons


def _exact_model(model_id, durations, *, evidence=KernelEvidenceKind.EXACT_SPECIFICATION):
    atoms = tuple(
        KernelAtom(1.0 / len(durations), duration, 2.0, "next")
        for duration in durations
    )
    return ExactOptionModel(
        state_id=model_id,
        available_options=(OptionKind.SEARCH_PORTAL,),
        option_kernels=((OptionKind.SEARCH_PORTAL, atoms),),
        model_id=model_id,
        evidence_kind=evidence,
    )


def test_fusion_uses_discounted_successor_kernel_not_mean_duration_t27():
    deterministic = _exact_model("s1", (1.0,))
    same_mean_different_duration_law = _exact_model("s2", (0.5, 1.5))
    result = assess_exact_option_fusion(
        deterministic, same_mean_different_duration_law, beta_per_s=0.8
    )
    assert not result.equivalent
    assert result.max_kernel_difference is not None
    assert result.max_kernel_difference > 0.0

    same_kernel = assess_exact_option_fusion(
        deterministic, _exact_model("s3", (1.0,)), beta_per_s=0.8
    )
    assert same_kernel.equivalent
    empirical = assess_exact_option_fusion(
        deterministic,
        _exact_model("s4", (1.0,), evidence=KernelEvidenceKind.EMPIRICAL_ESTIMATE),
        beta_per_s=0.8,
    )
    assert not empirical.equivalent
    assert "cannot_prove_exact" in empirical.reason


def test_model_rejects_non_normalized_probability_and_incompatible_options():
    with pytest.raises(ValueError, match="sum to one"):
        ExactOptionModel(
            "s", (OptionKind.SEARCH_PORTAL,),
            ((OptionKind.SEARCH_PORTAL, (KernelAtom(0.9, 1.0, 0.0, "n"),)),),
            "m", KernelEvidenceKind.EXACT_SPECIFICATION
        )
    left = _exact_model("s1", (1.0,))
    right = ExactOptionModel(
        "s2",
        (OptionKind.HOLD_SAFE,),
        ((OptionKind.HOLD_SAFE, (KernelAtom(1.0, 1.0, 2.0, "next"),)),),
        "m2",
        KernelEvidenceKind.EXACT_SPECIFICATION,
    )
    assert assess_exact_option_fusion(left, right, beta_per_s=0.0).reason == (
        "available_option_sets_differ"
    )

    multi_option_left = ExactOptionModel(
        "left",
        (OptionKind.SEARCH_PORTAL, OptionKind.HOLD_SAFE),
        (
            (OptionKind.SEARCH_PORTAL, (KernelAtom(1.0, 1.0, 2.0, "next"),)),
            (OptionKind.HOLD_SAFE, (KernelAtom(1.0, 0.5, -1.0, "same"),)),
        ),
        "left-model",
        KernelEvidenceKind.EXACT_SPECIFICATION,
    )
    multi_option_reordered = ExactOptionModel(
        "right",
        (OptionKind.HOLD_SAFE, OptionKind.SEARCH_PORTAL),
        (
            (OptionKind.HOLD_SAFE, (KernelAtom(1.0, 0.5, -1.0, "same"),)),
            (OptionKind.SEARCH_PORTAL, (KernelAtom(1.0, 1.0, 2.0, "next"),)),
        ),
        "right-model",
        KernelEvidenceKind.EXACT_SPECIFICATION,
    )
    assert assess_exact_option_fusion(
        multi_option_left, multi_option_reordered, beta_per_s=0.3
    ).equivalent
