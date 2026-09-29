"""Adversarial tests for bounded P6.3 genome and paired-evaluation contracts."""

from dataclasses import replace
import math

import pytest

from hsl_core.learning.evolution import (
    CandidateAssessment,
    EpisodeResult,
    EvolutionConfig,
    EvaluationPlan,
    EvaluationSplit,
    TacticalGenome,
    assess_paired_candidate,
    create_next_generation,
    mutate_genome,
    paired_student_t_interval,
    select_on_validation,
)
from hsl_core.match import Role
from hsl_core.tactics import ROLE_FEATURES, UtilityProfile
from sim.kinematic.autonomous import KinematicAutonomousPolicy
from sim.kinematic.match import MatchRole


_GUARDIAN = UtilityProfile(
    "guardian-base",
    Role.GUARDIAN,
    (
        ("capture_opportunity", 0.40),
        ("portal_time_advantage", 0.25),
        ("observation_gain", 0.15),
        ("pursuit_value", 0.10),
        ("duration_cost", -0.10),
    ),
    1,
)
_EXPLORER = UtilityProfile(
    "explorer-base",
    Role.EXPLORER,
    (
        ("base_progress", 0.35),
        ("visibility_loss", 0.20),
        ("alternative_exits", 0.15),
        ("escape_safety", 0.15),
        ("observation_gain", 0.05),
        ("capture_risk", -0.05),
        ("duration_cost", -0.05),
    ),
    1,
)
_GENOME = TacticalGenome.from_profiles(
    _GUARDIAN,
    _EXPLORER,
    hysteresis_delta_u=0.1,
    minimum_dwell_ns=100_000_000,
)
_PLAN = EvaluationPlan("plan-v1", 0.95, 2, 2)


def _episode(
    policy_id,
    pair,
    role,
    value,
    *,
    split=EvaluationSplit.VALIDATION,
    safety_overrides=0,
    safety_violations=0,
    collisions=0,
    completed=True,
):
    scenario, seed = pair
    return EpisodeResult(
        policy_id,
        scenario,
        split,
        role,
        seed,
        f"map-{scenario}-{split.value}",
        f"opponent-{split.value}",
        f"seed-{split.value}-{seed}",
        value,
        safety_overrides,
        safety_violations,
        collisions,
        completed,
    )


def _paired_rows(candidate_values, *, split=EvaluationSplit.VALIDATION):
    baseline = []
    candidate = []
    for index, (guardian_delta, explorer_delta) in enumerate(candidate_values):
        pair = (f"scenario-{index}", 100 + index)
        for role, delta in (
            (Role.GUARDIAN, guardian_delta),
            (Role.EXPLORER, explorer_delta),
        ):
            baseline.append(
                _episode("baseline", pair, role, 1.0, split=split)
            )
            candidate.append(
                _episode("candidate", pair, role, 1.0 + delta, split=split)
            )
    return baseline, candidate


def test_genome_matches_role_feature_contracts_and_produces_stable_hashes():
    assert _GENOME.utility_profile(Role.GUARDIAN, "g").weights == _GUARDIAN.weights
    assert _GENOME.utility_profile(Role.EXPLORER, "e").weights == _EXPLORER.weights
    assert len(_GENOME.sha256) == 64
    assert _GENOME.sha256 == replace(_GENOME).sha256
    assert {(role, feature) for role, feature, _ in _GENOME.weights} == {
        (role, feature) for role in (Role.GUARDIAN, Role.EXPLORER)
        for feature in ROLE_FEATURES[role]
    }


@pytest.mark.parametrize(
    "changes",
    [
        {"hysteresis_delta_u": -0.1},
        {"hysteresis_delta_u": math.inf},
        {"minimum_dwell_ns": -1},
        {"minimum_dwell_ns": True},
        {"schema_version": 2},
        {"weights": _GENOME.weights[:-1]},
    ],
)
def test_genome_rejects_invalid_bounds_schema_and_nonfinite_values(changes):
    args = {
        "weights": _GENOME.weights,
        "hysteresis_delta_u": _GENOME.hysteresis_delta_u,
        "minimum_dwell_ns": _GENOME.minimum_dwell_ns,
        "schema_version": _GENOME.schema_version,
    }
    args.update(changes)
    with pytest.raises((ValueError, TypeError)):
        TacticalGenome(**args)


def test_gaussian_mutation_is_seeded_projected_and_does_not_touch_safety_genes():
    first = mutate_genome(_GENOME, sigma=0.8, seed=78)
    second = mutate_genome(_GENOME, sigma=0.8, seed=78)
    assert first == second
    assert first.sha256 == second.sha256
    assert first.sha256 != _GENOME.sha256
    assert first.hysteresis_delta_u <= 1.0
    assert first.minimum_dwell_ns <= 60_000_000_000
    for role in (Role.GUARDIAN, Role.EXPLORER):
        profile = first.utility_profile(role, f"candidate-{role.name}")
        assert math.fsum(abs(value) for _, value in profile.weights) <= 1.0 + 1e-12
        assert dict(profile.weights)["duration_cost"] <= 0.0
    with pytest.raises(ValueError):
        mutate_genome(_GENOME, sigma=-0.1, seed=1)
    with pytest.raises(ValueError):
        mutate_genome(_GENOME, sigma=0.1, seed=True)


def test_genome_utility_parameters_are_wired_to_role_policy_without_safety_authority():
    candidate = mutate_genome(_GENOME, sigma=0.2, seed=9)
    guardian_policy = KinematicAutonomousPolicy(
        MatchRole.GUARDIAN,
        utility_profile=candidate.utility_profile(Role.GUARDIAN, "candidate-g"),
        hysteresis_delta_u=candidate.hysteresis_delta_u,
        minimum_dwell_ns=candidate.minimum_dwell_ns,
    )
    explorer_policy = KinematicAutonomousPolicy(
        MatchRole.EXPLORER,
        utility_profile=candidate.utility_profile(Role.EXPLORER, "candidate-e"),
        hysteresis_delta_u=candidate.hysteresis_delta_u,
        minimum_dwell_ns=candidate.minimum_dwell_ns,
    )
    assert guardian_policy.selector.utility_profile.weights == (
        candidate.utility_profile(Role.GUARDIAN, "candidate-g").weights
    )
    assert explorer_policy.selector.utility_profile.weights == (
        candidate.utility_profile(Role.EXPLORER, "candidate-e").weights
    )
    assert guardian_policy.supervisor is not explorer_policy.supervisor
    with pytest.raises(ValueError, match="role"):
        KinematicAutonomousPolicy(
            MatchRole.GUARDIAN,
            utility_profile=candidate.utility_profile(Role.EXPLORER, "wrong-role"),
        )


def test_paired_assessment_balances_both_roles_and_reports_paired_t_interval():
    baseline, candidate = _paired_rows(
        ((2.0, 0.0), (0.0, 2.0), (1.0, 1.0))
    )
    result = assess_paired_candidate(
        baseline,
        candidate,
        plan=EvaluationPlan("valid", 0.95, 2, 2),
        split=EvaluationSplit.VALIDATION,
    )
    assert result.eligible
    assert result.pair_count == 3
    assert result.guardian_mean_delta == pytest.approx(1.0)
    assert result.explorer_mean_delta == pytest.approx(1.0)
    assert result.balanced_mean_delta == pytest.approx(1.0)
    assert result.confidence_interval is not None
    assert result.confidence_interval[0] <= 1.0 <= result.confidence_interval[1]
    assert result.official_score_available is False
    assert select_on_validation("baseline", result).candidate_selected


def test_paired_assessment_does_not_let_one_role_mask_regression_in_the_other():
    baseline, candidate = _paired_rows(((10.0, -9.0), (10.0, -9.0)))
    result = assess_paired_candidate(
        baseline, candidate, plan=_PLAN, split=EvaluationSplit.VALIDATION
    )
    assert result.eligible
    assert result.guardian_mean_delta == pytest.approx(10.0)
    assert result.explorer_mean_delta == pytest.approx(-9.0)
    assert result.balanced_mean_delta == pytest.approx(0.5)
    decision = select_on_validation("baseline", result)
    assert not decision.candidate_selected
    assert "role_regression" in decision.reason


@pytest.mark.parametrize(
    "mutation, reason",
    [
        ("collision", "hard_safety_or_episode_completion_exclusion"),
        ("violation", "hard_safety_or_episode_completion_exclusion"),
        ("override", "hard_safety_or_episode_completion_exclusion"),
        ("incomplete", "hard_safety_or_episode_completion_exclusion"),
    ],
)
def test_any_hard_safety_or_completion_failure_excludes_candidate(mutation, reason):
    baseline, candidate = _paired_rows(((1.0, 1.0), (1.0, 1.0)))
    index = next(
        index for index, row in enumerate(candidate) if row.role is Role.GUARDIAN
    )
    changes = {
        "collision": {"collisions": 1},
        "violation": {"safety_violations": 1},
        "override": {"safety_overrides": 3},
        "incomplete": {"completed": False},
    }[mutation]
    candidate[index] = replace(candidate[index], **changes)
    result = assess_paired_candidate(
        baseline, candidate, plan=_PLAN, split=EvaluationSplit.VALIDATION
    )
    assert not result.eligible
    assert result.reason == reason
    assert not select_on_validation("baseline", result).candidate_selected


def test_pairing_requires_same_scenarios_seeds_banks_roles_and_split():
    baseline, candidate = _paired_rows(((1.0, 1.0), (1.0, 1.0)))
    candidate.pop()
    result = assess_paired_candidate(
        baseline, candidate, plan=_PLAN, split=EvaluationSplit.VALIDATION
    )
    assert not result.eligible
    assert result.reason == "baseline_candidate_pairing_mismatch"

    baseline, candidate = _paired_rows(((1.0, 1.0), (1.0, 1.0)))
    candidate[0] = replace(candidate[0], map_bank_id="leaked-map-bank")
    result = assess_paired_candidate(
        baseline, candidate, plan=_PLAN, split=EvaluationSplit.VALIDATION
    )
    assert not result.eligible
    assert result.reason == "baseline_candidate_pairing_mismatch"

    baseline, candidate = _paired_rows(((1.0, 1.0), (1.0, 1.0)))
    candidate[0] = replace(candidate[0], evaluation_profile_id="changed-profile")
    result = assess_paired_candidate(
        baseline, candidate, plan=_PLAN, split=EvaluationSplit.VALIDATION
    )
    assert not result.eligible
    assert result.reason == "baseline_candidate_pairing_mismatch"

    baseline, candidate = _paired_rows(((1.0, 1.0), (1.0, 1.0)))
    candidate[0] = replace(candidate[0], split=EvaluationSplit.TRAINING)
    result = assess_paired_candidate(
        baseline, candidate, plan=_PLAN, split=EvaluationSplit.VALIDATION
    )
    assert not result.eligible
    assert result.reason == "evaluation_split_mismatch"


def test_heldout_is_never_consumed_for_selection_and_requires_explicit_final_path():
    baseline, candidate = _paired_rows(
        ((1.0, 1.0), (1.0, 1.0)), split=EvaluationSplit.HELD_OUT
    )
    with pytest.raises(ValueError, match="held-out"):
        assess_paired_candidate(
            baseline,
            candidate,
            plan=_PLAN,
            split=EvaluationSplit.HELD_OUT,
        )
    baseline, candidate = _paired_rows(
        ((1.0, 1.0),), split=EvaluationSplit.VALIDATION
    )
    result = assess_paired_candidate(
        baseline, candidate, plan=_PLAN, split=EvaluationSplit.VALIDATION
    )
    assert not result.eligible
    assert result.reason == "minimum_paired_scenarios_not_met"


def test_duplicate_pairs_are_rejected_instead_of_inflating_sample_size():
    baseline, candidate = _paired_rows(((1.0, 1.0), (1.0, 1.0)))
    baseline.append(baseline[0])
    result = assess_paired_candidate(
        baseline, candidate, plan=_PLAN, split=EvaluationSplit.VALIDATION
    )
    assert not result.eligible
    assert result.reason == "duplicate_paired_episode_role"


def test_student_interval_rejects_degenerate_input_contracts():
    with pytest.raises(ValueError):
        paired_student_t_interval((1.0,), confidence_level=0.95)
    with pytest.raises(ValueError):
        paired_student_t_interval((1.0, math.nan), confidence_level=0.95)
    with pytest.raises(ValueError):
        paired_student_t_interval((1.0, 2.0), confidence_level=1.0)


def test_validation_selection_retains_baseline_without_positive_lower_bound():
    assessment = CandidateAssessment(
        "candidate",
        EvaluationSplit.VALIDATION,
        True,
        "paired_both_roles_safety_constraints_pass",
        5,
        1.0,
        1.0,
        1.0,
        (-0.2, 2.2),
    )
    decision = select_on_validation("baseline", assessment)
    assert decision.selected_policy_id == "baseline"
    assert not decision.candidate_selected
    assert "baseline_retained" in decision.reason
    with pytest.raises(ValueError, match="validation"):
        select_on_validation(
            "baseline",
            replace(assessment, split=EvaluationSplit.TRAINING),
        )


def test_generation_uses_training_only_hard_exclusions_and_keeps_baseline():
    candidate = mutate_genome(_GENOME, sigma=0.05, seed=100)
    assessment = CandidateAssessment(
        candidate.sha256,
        EvaluationSplit.TRAINING,
        True,
        "paired_both_roles_safety_constraints_pass",
        4,
        0.5,
        0.25,
        0.375,
        (0.1, 0.65),
    )
    excluded = mutate_genome(_GENOME, sigma=0.05, seed=101)
    excluded_assessment = replace(
        assessment,
        candidate_id=excluded.sha256,
        eligible=False,
        reason="hard_safety_or_episode_completion_exclusion",
        balanced_mean_delta=None,
        confidence_interval=None,
    )
    config = EvolutionConfig(
        population_size=4,
        elite_count=1,
        mutation_sigma=0.1,
        seed=900,
        maximum_generations=3,
    )
    first = create_next_generation(
        baseline_genome=_GENOME,
        evaluated_genomes=((candidate, assessment), (excluded, excluded_assessment)),
        config=config,
        generation_index=0,
    )
    second = create_next_generation(
        baseline_genome=_GENOME,
        evaluated_genomes=((candidate, assessment), (excluded, excluded_assessment)),
        config=config,
        generation_index=0,
    )
    assert first == second
    assert len(first.genomes) == 4
    assert first.genomes[0] == _GENOME
    assert first.elite_sha256 == (candidate.sha256,)
    assert first.excluded_candidate_reasons == (
        (excluded.sha256, "hard_safety_or_episode_completion_exclusion"),
    )
    with pytest.raises(ValueError, match="training"):
        create_next_generation(
            baseline_genome=_GENOME,
            evaluated_genomes=(
                (candidate, replace(assessment, split=EvaluationSplit.HELD_OUT)),
            ),
            config=config,
            generation_index=0,
        )


def test_generation_stops_boundedly_when_mutation_cannot_make_unique_genomes():
    config = EvolutionConfig(
        population_size=2,
        elite_count=1,
        mutation_sigma=5e-324,
        seed=1,
        maximum_generations=1,
    )
    baseline_assessment = CandidateAssessment(
        _GENOME.sha256,
        EvaluationSplit.TRAINING,
        True,
        "baseline_reference",
        2,
        0.0,
        0.0,
        0.0,
        (0.0, 0.0),
    )
    with pytest.raises(RuntimeError, match="could not produce a unique"):
        create_next_generation(
            baseline_genome=_GENOME,
            evaluated_genomes=((_GENOME, baseline_assessment),),
            config=config,
            generation_index=0,
        )
