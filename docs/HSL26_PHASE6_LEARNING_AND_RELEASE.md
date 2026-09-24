# HSL26 — Phase 6: Optional Learning and Frozen Release

**Revision:** 1.0 · **Baseline:** architecture/blueprint revision 2.2; roadmap revision 1.0.  
**Objective:** assess bounded offline tactical improvement without changing runtime safety, then produce a reproducible, authorized two-stage release. **Exit:** G6; G5 is optional.

## 1. Entry and immutability

Requires a G4 deterministic baseline, validated G0–G3 prerequisites in the intended execution profile, and recorded organizer decisions for all information required by the chosen policy. Official score fields remain unavailable until the organizer supplies them; surrogate rewards are explicitly labeled. Learning never changes rule predicates, option realizers, geometry inflation, speed/braking bounds, ROS authority or truth access.

## 2. Work packages

| ID / owner role | Subtasks and target files | Observable deliverable | Verification |
|---|---|---|---|
| P6.1 Dataset and abstraction | **a** define role/feature/state/option/outcome schema IDs; **b** collect durations, cancellations, safety interventions and censored episodes; **c** split map/opponent/seed banks without leakage. `learning/dirichlet.py`, transition log. | Versioned dataset manifest and valid transition counts. | §15.1/15.4; normalization and schema tests. |
| P6.2 Offline values | **a** positive Dirichlet prior and posterior; **b** continuous-time discounted option return; **c** terminal/no-bootstrap and failure samples; **d** reject unsupported state fusion. `learning/option_value.py`. | Convergent/bounded value-table fixtures and interpretation report. | T18/T27 and §15.2 fixtures. |
| P6.3 Bounded evolution and shadow | **a** bounded tactical genome; **b** paired training on both roles; **c** safety hard exclusion; **d** validation selection and once-only held-out report; **e** shadow logging before promotion. `learning/evolution.py`, `benchmark_policy.py`. | Baseline versus candidate paired outcomes, confidence intervals and safety metrics. | I15; G5 PASS only if preset criterion met. |
| P6.4 Frozen release inputs | **a** verify official zone/timing/score/memory provenance; **b** pin source, image digest, IDL, config, calibration, model, baseline or accepted policy; **c** check config and topic/TF ownership; **d** exclude sim truth and hardware devices in sim/replay. `validate_config.py`, `preflight_check.py`, `release_freeze.py`. | Signed-off manifest and cold-start checklist, with every unresolved item explicit. | I01–I05/I14, static/runtime authority checks. |
| P6.5 Final rehearsal and rollback | **a** offline cold boot in final image; **b** freeze/active/terminal over two role-swapped stages; **c** fault injection and recovery with permitted workflow; **d** record scores as official only on official evidence; **e** verify rollback. `runbook_competition.md`, `artifacts/releases/*`. | Complete two-stage run, logs, final gate matrix and last accepted release. | I16 plus G0–G4 prerequisites; optional I15. |

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
