# HSL26 — Independent audit of the Phase 6 handoff

**Date:** 2026-09-28  
**Scope:** supplied external-auditor text, current P5/P6 source and reports,
technical specification revision 2.2, repository contract, and existing
release-tool paths.  
**Disposition:** main integration-gap finding confirmed; proposed implementation
sequence requires corrections. This report is an engineering review, not a
gate acceptance.

**Implementation follow-up:** the original gap analysis predates the optional
P5.6 simultaneous-duel SIL profile. That profile now supplies role-scoped
duel stages, scan-residual tracking, fixture-only movement for both roles,
and a separately certified constant-curvature safety path. Its tests do not
change this audit's G4/G5/G6 dispositions or the need for a complete accepted
baseline; see the current P5.6 implementation/evidence report for scope.

## 1. Executive conclusion

The Phase 6 preparation was correct to keep learning disabled and to leave the
dataset splits empty: the P5.5 evidence explicitly records G4 as
`BLOCKED_NOT_RUN`, and P6 learning modules are placeholders. The external
review correctly identifies the missing decision-to-actuation integration as
a concrete work item. The current P5.5 runner accepts policy callbacks that
return `Actuation`; its own report explicitly excludes TacticalSelector,
OptionAuthority, candidate safety/mux and the production runtime from that
rehearsal. Consequently, fixed-command lifecycle/fault tests are not
autonomous-option episodes and cannot be relabeled as a complete baseline.

The review's proposed `agent.py` chain is not yet a valid implementation
specification. It skips several required data/provenance adapters, assumes
interfaces compose directly when they do not, and risks substituting
unverified values for evidence. A single match, 100 matches, an analytical
test on empirical frequencies, a Docker/MVSim run, or a generated report
cannot by itself close G4, establish a valid held-out dataset, or pass G6.
The safe next sequence is:

1. Implement the missing **P5.6 observation-to-actuation integration** against
   existing contracts and a declared kinematic/degraded profile.
2. Review all applicable upstream and G4 evidence separately by profile;
   preserve G4 as blocked until the profile's acceptance matrix is satisfied.
3. In parallel, implement P6.1/P6.2 pure schemas, algorithms and normative
   tests, but do not collect promotion-grade episodes or draw empirical
   conclusions before there is an accepted, traceable baseline.
4. Only then collect independent datasets, optionally evaluate G5, and pursue
   G6 on the actual authorized final target. Hardware and organizer blockers
   remain blockers.

No user-supplied source files were needed for this review: the attached text
and live repository provided enough context to verify the principal claims.

## 2. Claim-by-claim findings

| ID | External claim | Audit result | Evidence and qualification |
|---|---|---|---|
| A1 | P5.5 policies return fixed `Actuation`; no tactical brain is wired into that rehearsal. | **Confirmed, scoped to P5.5.** | `sim/kinematic/test_p55_rehearsal.py` uses callbacks/lambdas returning commands; `P5.5_rehearsal_report.json` says the callbacks return kinematic `Actuation` directly and excludes selector/authority/safety integration. This does not prove every simulator experiment in the repository uses fixed commands; it proves the cited rehearsal is not the requested closed-loop baseline. |
| A2 | `TacticalSelector` generates tactical proposals from observations. | **Incorrect; it is a consumer/ranker.** | `TacticalSnapshot` already contains validated `OptionProposal` objects. The caller must build proposals, contexts, guards, feature values and traceable evidence. Selector tests do not validate assessors or authenticate evidence producers. |
| A3 | The learning modules are completely empty. | **Functionally confirmed, wording corrected.** | `dirichlet.py`, `option_value.py`, and `evolution.py` contain only module-path comments (43/46/43 bytes at audit time); they have no learning implementation. They are not zero-byte files. |
| A4 | G4 cannot be closed without the autonomous decision loop. | **Confirmed as an integration gap; closure remains evidence-based.** | P5.4/P5.5 documents and report explicitly leave P5.3/OptionAuthority end-to-end wiring and G4 open. P5.6 is added as a required implementation package. A code merge or one successful match is not a gate pass. |
| A5 | No P6 dataset can be obtained until that integration is built. | **Substantially correct for this repository's planned SIL source, not universal.** | The current P5 rehearsal does not produce option-transition episodes. A separately validated replay/simulator source could be eligible only under the same observation, provenance, truth-isolation and acceptance contracts. Do not treat P5 fixed-command traces as learning data. |
| A6 | A single `KinematicTacticalAgent` can directly chain selector → authority → A* → pursuit → safety → `Actuation`. | **Not established; direct composition is invalid as stated.** | `PolicyInput` contains role, namespace, time, sensor observation and optional leased `MatchState`, not a `TopologyGraph`, validated opponent belief, option context, authenticated assessor evidence or safety snapshot. `astar()` consumes a versioned graph. `OptionAuthority` consumes a versioned `OptionGoal` and `OptionContext`, with lifecycle and lease transitions. `SafetySupervisor` consumes a provenance/version/lease-bound `SafetySnapshot` and only supports forward, curvature-free commands; its current core API is not a general angular-command filter. Explicit adapters/contracts and a supported profile are required. |
| A7 | G4 can be certified as `PASS_SIL` after one autonomous match. | **Rejected.** | Blueprint I11–I14 and the P5 gate require race/cancel, event, unresolved-goal, two-role truth-isolation and other applicable evidence, not merely nominal success. Record the profile and gate matrix; a profile-specific result cannot imply ROS/DDS, MVSim, physical safety or competition acceptance. Keep the existing P5.5 report immutable. |
| A8 | One hundred fast games are sufficient, and should take about ten seconds. | **Unsupported.** | No normative count/runtime appears in the supplied phase or technical specification. Runtime depends on step count and instrumented policy. Sample count must follow a preregistered evaluation design, independent split units, uncertainty/power needs and compute budget; report measured runtime rather than assume it. |
| A9 | Kemeny–Snell lumpability can be analytically validated from collected empirical transitions. | **Overstated.** | The specification requires compatible action availability, expected discounted reward and discounted transition kernels for all options/target classes. Exact equality is a model property; finite data can estimate and test it only with uncertainty and declared tolerances. The normative negative case T27 must reject equal-mean-duration but unequal discounted kernels. Never label finite counts an exact proof. |
| A10 | Fix the Dirichlet prior to uniform `alpha=1.0`. | **A possible test choice, not a mandated design.** | The specification requires positive prior concentrations and an immutable prior; it gives `alpha=(0.1,0.1)` as an example and explains its interpretation. Select and version prior values with schema/tests; do not silently hard-code the review's suggested value as normative. |
| A11 | Unsafe candidate gets `fitness=-infinity`. | **Use hard exclusion, but avoid non-finite artifact data.** | The specification explicitly separates safety disqualification from reward. Represent exclusion as a typed status/reason or filtered candidate, not a non-finite value that could leak into JSON, arrays or ordering. |
| A12 | Create `tools/release_freeze.py`. | **Path exists, implementation does not.** | `tools/release_freeze.py` is a 27-byte comment-only placeholder. `tools/preflight_check.py` is also a placeholder. Implement these existing targets under P6.4; do not create duplicate tools. |
| A13 | Generate `model.npz` from CAD as a P6 task. | **Misplaced and unsupported as stated.** | GPIS model construction/validation belongs to P4.1/P4.2 and has separate geometry, transform and held-out evidence requirements. The technical specification does not presume a complete CAD model. Phase 6 pins a model only if its selected deployment profile requires an accepted one; synthetic detections do not establish real-cloud fidelity. |
| A14 | Freeze `champion_params.yaml` and run two 10-minute Docker/MVSim stages for G6. | **Not normative and insufficient for G6.** | The policy contract specifies `manifest.json`, bounded `parameters.json`, non-executable `.npz` arrays and schema/hash checks; YAML is not required. No official 10-minute duration is supplied. G6 requires offline cold start and two stages in the final authorized target/operating conditions, matching immutable identities and physical evidence as applicable. Docker/MVSim cannot substitute for an unavailable robot/final target or organizer inputs. |

## 3. Integration constraints for P5.6

P5.6 is a kinematic/software integration package and must not alter the
normative architecture or weaken earlier safety contracts.

1. **Observation boundary:** policies receive only their role-scoped
   observation and leased stage state. A policy adapter may receive separately
   validated, immutable map/topology/ belief snapshots through an explicit
   test-profile interface; it must never receive `RefereeTruth`.
2. **Proposal assessment:** implement role-specific assessors that derive
   feasibility, guards, normalized features, urgency and evidence IDs from
   permitted observations. Missing/stale/unknown evidence means infeasible or
   `HOLD_SAFE`, not a fabricated guard. Evidence IDs provide traceability;
   they are not authentication.
3. **Authority lifecycle:** construct `OptionContext` from one coherent stage,
   epoch, version, lease and safety snapshot. Exercise submit, plan validation,
   execution, cancel/revoke, replan generation changes, effect evidence,
   timeout and failure. There must be one admitted active option and stale
   candidates must fail closed.
4. **Planning and control:** use a version-matched `TopologyGraph`, A* path and
   the existing path-to-candidate logic. A missing/unreachable target cannot
   be guessed. Keep map/topology/localization/option-instance and lease
   generation identities intact across each adapter.
5. **Safety boundary:** do not claim that the current
   `SafetySupervisor.evaluate()` certifies arbitrary `v, omega`: its documented
   implementation rejects nonzero angular candidate commands and negative
   forward speeds and validates timestamps, coverage, epochs, versions,
   candidate lease and calibration-bound limits. Define a supported
   curvature-free SIL profile or separately implement/specify the missing
   swept-trajectory safety contract through change control. Synthetic limit
   values must be labeled as fixtures and cannot become hardware calibration.
6. **Plant/referee transaction:** preserve `TwoRobotMatch.step()` as the
   owner of policy invocation, role gating, paired prediction/adjudication
   and plant commit. Faults must not partially advance one robot. Do not make
   the referee an input port to policy or promote simulated truth to official
   score.
7. **Evidence:** prove nontrivial options are selected and run by both roles
   across the declared sequential stages; record proposal/selection,
   authority, versioned plan, safety result, actuation, termination and
   reproducible seed. Exercise truth-leak negatives, stale lease/epoch,
   cancel/replan race, planner/assessor/sensor/watchdog faults, unreachable
   route, unknown zone and role swap. Fixed-command tests remain useful for
   runner tests but do not satisfy these items.

The `agent.py` filename in the external proposal is not an architectural
requirement. A small policy adapter may coordinate components, but typed
boundaries and testable assessors/adapters should remain independently
verifiable. Do not modify `TwoRobotMatch` to smuggle truth or undocumented
state into a policy.

## 4. Revised phase roadmap and exit criteria

| Order | Work | Entry/output | Exit/status rule |
|---:|---|---|---|
| 0 | Preserve P1–P5 and P6 entry snapshots; freeze interface/config/fixture provenance for the new work | Existing reports remain immutable | Record source hashes and exact profile; do not rewrite historical status |
| 1 | **P5.6a — integration design and input inventory** | Inspect actual P1 safety, P3 topology/planning, P4 belief, P5 authority/tactics and SIL observations; enumerate fields/versions/evidence available and missing | No code starts by assuming composition; unresolved capability is marked degraded/blocked |
| 2 | **P5.6b — observation/proposal/option adapters** | Add bounded role-specific assessors and adapt leased StageManager snapshots to option contexts | Typed contracts; hold-safe on missing evidence; unit and negative tests |
| 3 | **P5.6c — route, candidate and safety integration** | Use versioned graph, plan, execution lease and supported safety boundary | Stale/mixed versions and invalid coverage/limits reject or stop; no command bypass |
| 4 | **P5.6d — two-role closed-loop SIL regression** | Drive `TwoRobotMatch` via integrated policies, maintain separate referee output | Repeatable nontrivial option traces, both roles, authority races/faults/truth isolation, no partial tick; bounded SIL evidence report |
| 5 | **G4 profile review** | Applicable P5 T01–T05/T14/T21/T22/T28–T30 and I04/I11–I14 plus upstream prerequisites | Reviewer records PASS/FAIL/BLOCKED/NOT_RUN for each intended profile. G4 is not passed by the existence of a runner or one match. |
| 6 | **P6.1 schema and collector implementation** | May be developed/tested before G4 using clearly synthetic normative fixtures | Validate the §15.4 record, hashes, censoring, termination and immutable split manifest. Empirical dataset acceptance waits for the accepted baseline/profile. |
| 7 | **P6.2 Dirichlet and option-value implementation** | May be developed/tested before G4 with specification fixtures | Positive immutable prior; posterior normalization; continuous-time discount; integrated reward `dt` convention; terminal no-bootstrap; failures/censoring; T18/T27; no unsupported fusion |
| 8 | **P6.3 paired evaluation and optional G5** | Accepted baseline, eligible independent data, preregistered criterion and frozen candidate set | Compare both roles on paired held-out scenarios; uncertainty and safety metrics; evaluate held-out data once. Promote only on criterion; otherwise baseline |
| 9 | **P6.4 frozen release inputs** | Approved official inputs and accepted required gates; implement existing `tools/validate_config.py`, `tools/preflight_check.py`, and `tools/release_freeze.py` targets as appropriate | Strict provenance, compatible schemas, complete immutable hashes, no truth/device access in sim/replay; unresolved inputs block |
| 10 | **P6.5/G6** | Access to intended final target, approved retention/reset and operating conditions | Offline cold start without network fetch/editable mount; two role-swapped full stages; faults/recovery, same hashes and rollback; no robot means physical/final G6 remains BLOCKED |

Pure-core P6.1/P6.2 implementation can proceed in parallel with the P5.6
integration work. Dataset acquisition and promotion-quality analysis cannot
use P5.5 fixed-command episodes as baseline policy runs. No fixed number of
matches is required by the specification; preregister split units, paired
scenario design, minimum information/precision, confidence procedure,
stopping rule, and compute budget before running. Split by independent maps,
opponent banks and seed groups before fitting or selecting a policy, not by
randomly distributing correlated frames/episodes across splits.

## 5. Phase-specific model, artifact and release decisions

- **No new package by default.** Existing NumPy/SciPy and standard-library
  formats satisfy the planned numerical, JSON/JSONL and SHA-256 requirements;
  pytest remains the test runner. Add dependencies only for a demonstrated
  implementation need.
- **Dirichlet prior:** require finite positive concentrations and immutable
  schema identity; keep test values from §15.2. `alpha=1` is one possible
  symmetric prior, not an external-review mandate.
- **Reward/value:** keep `official_score`, `training_return` and
  `selection_fitness` distinct. Label surrogate outcomes. Integrate reward
  rates with the declared time-step convention; do not double count terminal
  impulses. Do not bootstrap terminal/censored records as though their
  successor were observed.
- **Abstraction:** exact Kemeny–Snell compatibility needs all applicable
  reward and discounted-kernel conditions plus action availability. Empirical
  data can support a stated statistical/tolerance assessment, never an
  unsupported exact equality claim.
- **Safety exclusion:** exclude failing candidates before reward ranking and
  report explicit failure reasons. Avoid `-inf` in serializable models.
- **Artifacts:** follow technical specification §15.4: JSON manifest and
  bounded parameters plus non-executable NumPy arrays (`allow_pickle=False`)
  with shape/dtype/schema/hash checks. Do not introduce a champion YAML
  contract merely because the pasted review proposes it.
- **Dataset path:** when P6.1 produces actual records, use
  `artifacts/learning/datasets/<dataset_id>/` for immutable transition data
  and manifests, and `artifacts/reports/phase6/` for derived evaluation
  reports. Record role, scenario/opponent/seed-bank identities, episode
  status, observation fidelity and policy/schema hashes. No dataset directory
  is populated by this preparation.
- **GPIS:** no P6 CAD-to-GPIS work is added. If the release profile consumes
  GPIS, require the P4 model's own valid provenance and held-out report, and
  pin its hashes; otherwise explicitly record the model as not used.
- **Release tools:** `tools/release_freeze.py` and `tools/preflight_check.py`
  already exist as comment-only stubs; continue those paths rather than
  duplicating them. `tools/validate_config.py` is also a planned target and
  its current implementation/readiness must be checked when P6.4 begins.
- **G6:** no assumed 10-minute official stage duration. Obtain authorized
  parameters; preserve unknowns as blocked. MVSim or Docker is useful only for
  the profile it validates, not as a proxy for final hardware acceptance.

## 6. Verified repository state and evidence limitations

Verified from the live checkout during this audit:

- P5.5 tests construct callbacks returning fixed `Actuation` values; the
  report's integration limitations match the implementation.
- `TacticalSnapshot` contains preconstructed proposals, while
  `TacticalSelector.select()` ranks and validates them.
- `dirichlet.py`, `option_value.py`, `evolution.py`,
  `tools/release_freeze.py`, and `tools/preflight_check.py` are comment-only
  placeholders at this audit point.
- The current P5.5 report says bounded kinematic rehearsal PASS and G4
  `BLOCKED_NOT_RUN`; it also records no accepted zone, no physical authority,
  and no ROS namespace/DDS or MVSim evidence.
- Existing P5.0/P5.5 report entries have historical test counts and source
  revisions. They describe their recorded run, not the current modified
  checkout and not a G4 acceptance.

No acceptance report was created or altered by this audit. The actual G4,
G5, and G6 status remains open/blocking as recorded until new executable
evidence is reviewed.

## 7. P5.6 implementation addendum

After the audit, a bounded P5.6 slice was implemented in
`sim/kinematic/autonomous.py`. The SIL input contract now optionally carries
an explicit estimated pose with time/frame/epoch/error bounds and a
versioned topology graph; LiDAR also distinguishes uncovered beams from
covered no-return beams. The fixture exercises Guardian
`SEARCH_PORTAL` through selector, authority, A*, rotating execution lease,
candidate and the existing zero-curvature safety supervisor. Explorer
`OBSERVE_SAFE` is a no-motion authority lifecycle only.

This does not resolve the original integration gap completely: there is no
validated opponent-belief assessor, rival-dependent role policy, full
Explorer navigation, turning safety profile, ROS/DDS integration or complete
T/I acceptance evidence. The report
[`P5.6_autonomous_integration_report.json`](../artifacts/reports/phase5/P5.6_autonomous_integration_report.json)
records the bounded evidence and unsupported claims. P5.6 remains PARTIAL,
and G4 remains `BLOCKED_NOT_RUN`; the historical P5.5 report is unchanged.
