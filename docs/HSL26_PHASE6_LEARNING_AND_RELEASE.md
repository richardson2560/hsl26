# HSL26 — Phase 6: Optional Learning and Frozen Release

**Revision:** 1.0 · **Baseline:** architecture/blueprint revision 2.2; roadmap revision 1.0.  
**Objective:** assess bounded offline tactical improvement without changing runtime safety, then produce a reproducible, authorized two-stage release. **Exit:** G6; G5 is optional.

## 1. Entry and immutability

Requires a G4 deterministic baseline, validated G0–G3 prerequisites in the intended execution profile, and recorded organizer decisions for all information required by the chosen policy. Official score fields remain unavailable until the organizer supplies them; surrogate rewards are explicitly labeled. Learning never changes rule predicates, option realizers, geometry inflation, speed/braking bounds, ROS authority or truth access.

The current P5.5 evidence records G4 as `BLOCKED_NOT_RUN`. P5.6 now includes
an optional two-role interactive SIL fixture with synthetic scan-residual
tracking, route/escape policies, and supervised continuous-curvature motion;
this is not an accepted baseline or G4 evidence. The full role baseline and
G4 acceptance remain open. P6.1 schema/collector contracts and P6.2 algorithms may be implemented and tested
with normative fixtures while that work proceeds; they do not authorize
empirical training, a valid dataset, G5 promotion or a release before an
accepted baseline/profile and required provenance exist. The independent
handoff review and detailed prerequisite roadmap are recorded in
[`HSL26_PHASE6_INDEPENDENT_AUDIT.md`](HSL26_PHASE6_INDEPENDENT_AUDIT.md).

## 2. Work packages

| ID / owner role | Subtasks and target files | Observable deliverable | Verification |
|---|---|---|---|
| P6.1 Dataset and abstraction | **a** define role/feature/state/option/outcome schema IDs; **b** implement collection of durations, cancellations, safety interventions and censored episodes; **c** split independent map/opponent/seed banks without leakage. `learning/dirichlet.py`, transition log. | Versioned dataset manifest and valid transition counts. | §15.1/15.4; normalization and schema tests. Pure contract work may precede G4; empirical dataset acceptance requires an accepted baseline and provenance. |
| P6.2 Offline values | **a** positive immutable Dirichlet prior and posterior; **b** continuous-time discounted option return; **c** terminal/no-bootstrap and failure samples; **d** reject unsupported state fusion. `learning/option_value.py`. | Bounded/convergent value-table fixtures and interpretation report. | T18/T27 and §15.2 fixtures. Pure implementation may precede G4; empirical conclusions require eligible data. |
| P6.3 Bounded evolution and shadow | **a** bounded tactical genome; **b** paired training on both roles; **c** safety hard exclusion; **d** validation selection and once-only held-out report; **e** shadow logging before promotion. `learning/evolution.py`, `sim/kinematic/benchmark.py`, later `benchmark_policy.py`. | Fixture-only paired episode runner exists; population orchestration and eligible baseline/candidate evidence remain blocked. | I15; G5 PASS only if preset criterion met. |
| P6.4 Frozen release inputs | **a** verify official zone/timing/score/memory provenance; **b** pin source, image digest, IDL, config, calibration, model, baseline or accepted policy; **c** check config and topic/TF ownership; **d** exclude sim truth and hardware devices in sim/replay. `validate_config.py`, `preflight_check.py`, `release_freeze.py`. | Signed-off manifest and cold-start checklist, with every unresolved item explicit. | I01–I05/I14, static/runtime authority checks. |
| P6.5 Final rehearsal and rollback | **a** offline cold boot in final image; **b** freeze/active/terminal over two role-swapped stages; **c** fault injection and recovery with permitted workflow; **d** record scores as official only on official evidence; **e** verify rollback. `runbook_competition.md`, `artifacts/releases/*`. | Complete two-stage run, logs, final gate matrix and last accepted release. | I16 plus G0–G4 prerequisites; optional I15. |

## 2.1 Pure P6.1/P6.2 implementation status

The P6.1/P6.2 pure-core contracts are implemented in
`hsl_core/hsl_core/learning/` and covered by normative/adversarial tests. P6.1
now has versioned, schema-fingerprinted feature bins and abstract-state IDs,
immutable option-transition records, explicit censored/truncated outcomes,
bounded reward-rate intervals, deterministic JSONL/manifest serialization,
and map/opponent/seed split checks that prevent bank and episode leakage.
P6.2 implements immutable Dirichlet rows, analytic continuous-time discounted
returns, explicit terminal/nonterminal SMDP backups, and exact-model-only
discounted-kernel fusion checks (including T27).

`DatasetManifest.contract_valid` reports structural/schema/split validity only.
`promotion_eligible` remains false until dataset-quality thresholds and an
independent promotion review are assessed; neither field accepts G4 or G5.
No empirical transition dataset was collected for this implementation. The
machine-readable implementation/evidence boundary is
[`P6.1_P6.2_learning_contracts.json`](../artifacts/reports/phase6/P6.1_P6.2_learning_contracts.json).
P6.3 now has bounded genome, paired-assessment, validation-selection, mutation,
generation, synthetic maze fixtures, and a deterministic single/paired episode
runner. The runner feeds existing `EpisodeResult` contracts but labels each
run `SIL_DEVELOPMENT_FIXTURE_NOT_PROMOTION_ELIGIBLE`, refuses the reserved
held-out split, and emits no official score. This is executable fixture
preparation, not an empirical population run; it does not alter the existing
entry gates. Physical authority, G4/G5/G6 acceptance, and release acceptance
remain outside this implementation.

## 2.2 P6.3 SIL preparation boundary

The current P6.3 work is recorded in
[`P6.3_sil_preparation.json`](../artifacts/reports/phase6/P6.3_sil_preparation.json).
It implements bounded tactical-genome operations, role-aware paired candidate
assessment, hard safety/completion exclusions, Student-t uncertainty intervals,
training-only generation ranking, validation-only selection, and a bounded
SIL episode runner. A synthetic, connected orthogonal maze bank provides four
training maps and two validation maps, including 7-by-7 fixtures, plus a
reserved held-out fixture. The held-out fixture is rejected by the runner.

This does **not** authorize empirical evolution. The Phase-6 entry criterion
requires an accepted deterministic G4 baseline, while the current P5.6 status
is partial and G4 remains `BLOCKED_NOT_RUN`. The current SIL profile has fixture-only Explorer route/escape behavior and
synthetic opponent tracks, not a validated competition policy or perception
system. No population/generation orchestrator, held-out use-once ledger,
shadow logging, or promotion review has been implemented. Running these
episodes optimizes/evaluates only a synthetic fixture task and must not be
described as an eligible P6.3 result.

The runner's outcome is a terminal-only zero-sum surrogate,
`outcome * exp(-beta * active_duration)`: capture is a Guardian win and an
uncaptured timeout is an Explorer win. This follows §15.3 without inventing
an unapproved per-second reward rate or treating elapsed time as failure.
Capture terminalization is accepted only on straight-command intervals,
because the referee's trajectory interpolation is piecewise linear; curved
intervals are not promoted to capture evidence by this runner.

The SIL extension supports straight and bounded constant-curvature translation,
with stationary `ALIGN` available as a fallback. Curved movement requires
full-scan swept-arc clearance certification and explicit fixture error and
dynamic-speed bounds; the certificate applies only to its declared model and
does not prove arbitrary or physical swept-volume safety, Kobuki limits, G4,
or physical calibration. Continue to label the maze bank and all maneuver
parameters as synthetic software fixtures. No dependency was added; the
statistical interval uses the existing SciPy dependency.

## 3. Milestones and branches

| Milestone | Condition |
|---|---|
| M6.1 Reproducible dataset/model | P6.1/P6.2 with schema hashes, split and sanity fixtures; optional if using baseline only. |
| M6.2 Candidate promotion decision | P6.3; G5 may PASS, FAIL or NOT_RUN. FAIL/NOT_RUN selects the already accepted deterministic baseline. |
| M6.3 Release candidate | P6.4, accepted required gate matrix and frozen immutable hashes. |
| M6.4 G6 | P6.5 runs in final target environment and matches pinned identities for two complete stages. |

## 4. Gate G5 and G6 acceptance records

G5 compares baseline and candidate on paired held-out scenarios with the same observations, opponent bank and both roles. Record uncertainty intervals, official-score availability, surrogate return separately, safety overrides, collisions, latency and no regression on required safety constraints. Do not promote a policy for training fitness alone.

G6 requires I16, actual cold start without network downloads or editable bind mounts, two-stage role swap under approved retention, final hardware operating-condition scope, measured stop/braking response, rule/zone/timer provenance and immutable release hashes. A reported scaffold build or simulation-only pass is not a final-platform acceptance. If an organizer-dependent input remains unknown, preserve a BLOCKED release decision and the last safe accepted baseline.

**Required negative cases:** equal mean option duration with unequal discounted transition kernels fails abstraction/fusion; unsafe genome is excluded regardless of reward; a policy artifact with mismatched feature/option hash fails loading; a simulated truth capability cannot be obtained through a policy port; a changed release hash, unapproved zone provider or unmeasured physical bound fails preflight. Log each gate separately so a passing G5 never masks a failed G6.

## 5. Closure artifact

The final report lists each gate ID, evidence link, environment, PASS/FAIL/BLOCKED/NOT_RUN, owner/reviewer, accepted modes, unresolved limitations and rollback artifact. It records the actual source/image/config/model/policy digests. No evidence line is marked PASS solely because this plan specifies a test.
