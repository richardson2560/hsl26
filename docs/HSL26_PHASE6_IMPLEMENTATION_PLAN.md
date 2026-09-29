# HSL26 - Phase 6 implementation plan and no-hardware preparation

**Revision:** 1.0 · **Date:** 2026-09-28  
**Authority:** `HSL26_TECHNICAL_SPECIFICATION.md` §§14-17; `HSL26_FINAL_ARCHITECTURE.md`; `HSL26_IMPLEMENTATION_ROADMAP.md`; `HSL26_PHASE6_LEARNING_AND_RELEASE.md`.  
**Preparation decision:** `PREPARED_WITH_P5_G4_INTEGRATION_GAP`  
**Current scope:** P6.0 environment and contract preparation only; no physical robot.

## 1. Purpose, entry conditions and limits

This document prepares the repository and a reproducible software-in-the-loop
(SIL) workflow for Phase 6. It does not implement or claim completion of the
learning algorithm, G5 promotion, a frozen release, or G6. The Phase 6 entry
criterion is a deterministic G4 baseline. The latest P5 rehearsal report
records `g4_status: BLOCKED_NOT_RUN`; this blocks empirical data collection,
promotion conclusions, integrated gate closure and release acceptance. Pure
P6.1 schema/collector work and P6.2 algorithm/tests may proceed in parallel.
P5.6 is the concrete missing integration package and must be reviewed before
autonomous baseline episodes are accepted for learning. A bounded P5.6 slice
is now implemented; its report explicitly leaves the full baseline and G4
blocked.

The current P5 evidence supports bounded kinematic lifecycle, fault, replay,
and a limited decision-to-actuation integration slice. It does not establish
the complete accepted policy baseline, ROS/runtime integration, MVSim
fidelity, physical safety, or official scoring. P5.6 now includes a separate
simultaneous SIL duel profile, synthetic scan-to-CV-track processing,
Guardian pursuit, Explorer route/escape decisions, and certified
constant-curvature motion under the declared fixture bounds. The existing
exclusive-role runner remains unchanged. This interaction profile remains a
software fixture: it is not accepted baseline evidence, complete perception
validation, an official competition objective, or a G4 pass. It addresses the
previous lack of a dynamic two-role SIL interaction without relaxing the
Phase-6 entry gates. Pure-core P6.1 schema/collector
work and P6.2 algorithm/tests may proceed in parallel; dataset collection,
promotion conclusions, integrated gate closure and release acceptance remain
blocked on the accepted baseline/profile and their other inputs.
In particular:

- `sim/kinematic/scenarios/match_tactics.json` is a development case manifest,
  not a training/validation/held-out scenario bank.
- The P5 rehearsal's assumed timing, role poses, and seeds are fixtures, not
  approved competition parameters.
- No organizer-provided score, target-zone, stage-trigger, event-ordering, or
  memory-retention input is inferred or populated here.
- No hardware profile is armed. No P6 test in this plan can establish physical
  braking, stop response, calibration, or competition acceptance.

## 2. Environment prepared

| Asset | Purpose | Preparation status |
|---|---|---|
| `sim/kinematic/scenarios/phase6_learning_release.json` | P6 profile, evidence references, truth boundary, and empty split-bank declaration | Prepared; not a dataset |
| `sim/kinematic/test_p6_manifest.py` | Guards against accidental promotion of fixtures, truth leakage, or guessed external inputs | Prepared |
| `artifacts/reports/phase6/P6_environment_baseline.json` | Host/software inventory, P5 entry evidence, package decision, readiness and blockers | Prepared |
| `docs/HSL26_PHASE6_IMPLEMENTATION_PLAN.md` | Ordered P6 work packages, gates, commands, and evidence requirements | Prepared |
| `docs/HSL26_PHASE6_INDEPENDENT_AUDIT.md` | Independent review of the supplied audit and corrected P5.6 → P6 roadmap | Prepared |
| `docs/HSL26_PHASE6_LEARNING_AND_RELEASE.md` | Normative Phase 6 objectives and acceptance conditions | Governing specification |
| `hsl_core/hsl_core/learning/*.py` | P6.1/P6.2 contracts and bounded P6.3 evolution operators | Implemented and tested as contracts; no empirical dataset, population runner, or promotion |

### 2.1 Profiles

| Profile | Status | Permitted claims |
|---|---|---|
| Kinematic SIL preparation | Enabled for bounded fixture interaction and contract checks | Software behavior under declared synthetic assumptions only; no G4, eligible training data, or G5 result |
| Offline dataset/model | Blocked until G4 and independent, provenance-tagged banks are available | No learned-value or candidate-performance claim |
| Replay | Prepared in principle; blocked on immutable observations and validated runtime | No replay or sensor-fidelity claim |
| MVSim | Blocked pending native runtime and sensor-model inspection | No MVSim integration/fidelity claim |
| Hardware/real | Disarmed; no robot available | No physical or competition release claim |

The policy-facing boundary remains observation-only. Referee truth is available
only to evaluation code and is never an admitted policy feature. The duel
profile uses separate role-scoped `MatchState` snapshots, a static map fixture,
and a fixture-scoped Explorer destination; none is an official base/zone
decision. Synthetic observations, opponent-speed bounds, and scenario
provenance remain explicit fixture inputs.

## 3. Dependencies and setup

No new Python package is required for P6.0 or the planned pure-core work:

- `pytest` is already declared by `hsl_core/pyproject.toml` and runs the
  contract/regression suite.
- NumPy and SciPy are existing `hsl_core` dependencies and are available for
  the numerical learning code.
- JSON/JSONL, hashing, deterministic serialization, and basic reporting can
  use the Python standard library.
- Matplotlib is optional diagnostics only. Open3D is unrelated to Phase 6
  learning/release preparation. Neither is required.
- ROS 2, colcon, MVSim, and Docker are not required for P6.0 or pure-core
  numerical tests. Native ROS/release-image workflows are separate, later
  integration requirements; they do not remove hardware or organizer gates.

The recorded local host uses Python 3.14.7. The project baseline is Python
>=3.10; release reproducibility must additionally pin the actual interpreter,
dependency versions, and image digest for the selected target environment.

From the repository root in Windows PowerShell:

```powershell
$env:PYTHONPATH = "hsl_core"
python -m pytest sim\kinematic\test_p6_manifest.py -q
python -m pytest hsl_core\tests sim\kinematic ros_ws\src\hsl_interfaces\test ros_ws\src\hsl_safety\test -q
python -m compileall -q hsl_core\hsl_core sim
```

The first command checks only the prepared P6 manifest boundary. The broader
command is regression coverage, not proof of P6 learning behavior, G4, G5, or
G6. Do not install optional packages or change dependency declarations unless
a concrete implementation and measured need require them.

## 4. Ordered work packages

| Package | Implementation sequence | Core outputs | Exit evidence |
|---|---:|---|---|
| P6.0 Environment and contract preparation | Prepared | This plan, environment report, truth-isolated profile, no-leakage manifest tests | JSON validation and regression; entry blockers stay explicit |
| P5.6 G4 kinematic decision-loop prerequisite | Bounded SIL interaction profile implemented; G4 baseline remains blocked | Standard exclusive-role runner is preserved. Optional dual-manager duel has simultaneous role authorization gated by both ACTIVE leases, scan-residual EKF tracks, Guardian pursuit, Explorer fixture-route/escape navigation, and continuous curvature only under swept-scan, braking, lateral, yaw and wheel limits. | See P5.6 report; integrated cases are test evidence only. T21/T22/T28–T30, I11–I14 and profile review remain incomplete; no G4, G5 or physical pass inferred |
| P6.1 Dataset and abstraction | Schema/collector contract work may start in parallel; empirical dataset blocked until accepted baseline | Versioned role/feature/state/option/outcome schemas; immutable transition records including cancellation, safety interventions and censoring; independent map/opponent/seed bank IDs and split manifest | Schema/hash tests may pass before G4. Dataset validity, split integrity, and promotion-quality counts require accepted baseline episodes and provenance |
| P6.2 Offline values | Pure implementation and normative fixtures may start in parallel; empirical interpretation waits for P6.1 data | Positive immutable Dirichlet prior/posterior; continuous-time discounted option return and value update; terminal/no-bootstrap and failures | T18/T27 and §15.2 fixtures; fixture tests do not establish data quality, learned improvement, or G5 |
| P6.3 Bounded evolution and shadow | Optional; blocked until accepted baseline, eligible P6.1 data, and P6.2 | Bounded tactical parameters; paired training for both roles; safety hard exclusion; validation choice; once-only held-out report; shadow logging | I15/G5 with preregistered criterion, intervals, role balance and safety metrics; otherwise retain baseline |
| P6.4 Frozen release inputs | Static/preflight implementation may proceed in parallel; closure blocked on accepted gates and approved inputs | Provenance-validated config; source/image/IDL/config/calibration/model/policy hashes; ownership/preflight checks | Required gate matrix and complete immutable inputs; missing official or physical evidence remains BLOCKED |
| P6.5 Final rehearsal and rollback | Blocked on accepted release candidate and access to intended final target | Offline cold start; two role-swapped stages; permitted reset/retention; fault recovery; logs and rollback artifact | I16/G6 only in the specified final target/operating conditions; no physical target means G6 remains BLOCKED |

P6.1 and P6.2 must implement the normative §15 contracts, not a new or weaker
learning model. In particular, equal mean option duration is insufficient
evidence for state fusion; terminal transitions do not bootstrap; censoring is
not silently mapped to wins/losses; and safety failure is a hard exclusion,
not a reward penalty. Learning remains outside competition-critical ROS nodes.

## 5. Entry gates, blockers and handoff

Pure-core schema, numerical implementation, and negative-fixture tests are
not gated on physical hardware or a completed G4. The following are gated:
using P5 fixture output as an empirical baseline, producing a
promotion-quality dataset, drawing performance conclusions, promoting a
learned policy, and closing integrated/release gates. The G4 integration gap
must be resolved and reviewed before such evidence is collected. A future
verified replay or simulator profile could be a data source only if it meets
the same observation, provenance, truth-isolation, and acceptance contracts;
P5 direct-command fixture traces do not.

| Gate/input | Current status | Required before |
|---|---|---|
| P5.6 observation-to-actuation autonomous kinematic integration | `PARTIAL_BOUNDED_SLICE`; full role baseline not accepted | Eligible autonomous episode collection and profile-specific G4 review |
| G4 deterministic accepted baseline | `BLOCKED_NOT_RUN` in P5.5 evidence | Promotion-quality empirical P6 work and overall release progression |
| G0 physical stop, calibrated bounds, G1/G2 physical evidence | Blocked by unavailable robot/hardware | Any physical or competition motion claim |
| Organizer score, zone, stage, event-order and memory decisions | Unresolved/not supplied | Official scoring, authorized policy inputs, final release |
| Independent map/opponent/seed banks and transition logs | Not available in this preparation | P6.1 dataset/model claims and P6.3 held-out evaluation |
| Final target image/runtime and offline cold-start evidence | Not available | P6.4 release closure and P6.5/G6 |

The next integration action is to complete and review P5.6 against the actual
contracts, while pure P6.1/P6.2 implementation can proceed independently.
Collect eligible data only after review accepts the baseline/profile and data
provenance. Do not mark a P6 work package `PASS` because its schema, plan, or
fixture exists. If G5 is not run or fails its preregistered criterion, use
only an already accepted baseline; G5 failure does not waive any G6
requirement.

## 6. Evidence and status discipline

Store entry/preparation evidence in `artifacts/reports/phase6/`, per-package
results in separately named reports, datasets and policies only in their
specified artifact locations, and frozen manifests under
`artifacts/releases/`. Every result records source revision, exact command,
environment/profile, seed and bank identities, relevant hashes, observed and
expected result, status (`PASS`, `FAIL`, `BLOCKED`, or `NOT_RUN`), limitations,
and reviewer/signoff when applicable.

Keep preparation status distinct from acceptance. `P6_environment_baseline`
is an entry snapshot; do not overwrite it with later package results or edit
prior phase reports to imply a gate closed. A software/kinematic pass never
transfers automatically to replay, MVSim, hardware, or the final release
target.
