# HSL26 - Phase 5 implementation plan and SIL environment

**Revision:** 1.2 · **Date:** 2026-09-28
**Authority:** `HSL26_TECHNICAL_SPECIFICATION.md` §§10-11, 14, 17;
`HSL26_FINAL_ARCHITECTURE.md`; `HSL26_PHASE5_MATCH_AND_TACTICS.md`.  
**Entry decision:** `READY_FOR_SIL_IMPLEMENTATION_WITH_EXTERNAL_GATE_BLOCKERS`  
**Current scope:** deterministic software-in-the-loop (SIL); no physical robot.

## 1. Purpose, entry conditions and limits

This plan records the no-hardware SIL preparation and implementation sequence
for stage lifecycle, adjudication, option authority, both tactical roles and
the bounded P5.4 kinematic referee runner. It follows the evidence-led setup
used for Phases 3 and 4. Pure-core P5.1-P5.3 and the bounded P5.4/P5.5
kinematic referee/rehearsal slices are implemented and tested. A bounded
P5.6 slice now connects observation coverage, role selection, option authority,
versioned planning, lease-controlled candidates and the existing safety
supervisor. It does not complete both autonomous role policies or the profile
acceptance matrix. Existing evidence does not claim G4, ROS runtime
integration, MVSim fidelity or competition readiness.

The current repository provides useful SIL inputs:

- P1 pure-core and safety-boundary evidence, with G0 software contracts
  passing in the recorded report; native ROS, process fault and physical-stop
  evidence remain blocked.
- P3 kinematic world, route and cancellation contracts with a SIL handoff;
  physical G2 evidence and some route-execution/supervisor integration remain
  open.
- P4 tracking/belief contracts and a synthetic-detection profile; real-cloud,
  registered-scan, MVSim fidelity and physical G3 evidence remain blocked.
- Existing deterministic plant, sensor, referee, trace and scenario
  infrastructure under `sim/kinematic/`, extended by `match.py` for role-
  scoped two-robot SIL ticks and private capture/collision truth output.

Therefore P5 may start with the kinematic + `SYNTHETIC_DETECTION` profile for
contract and policy work. Entry to real motion, G4 release or an official
match result remains blocked until the relevant upstream gates and external
rule/zone inputs are accepted. Missing goal metadata must remain unresolved;
it must not be guessed from the map, farthest node or fixture coordinates.

## 2. Environment prepared

| Asset | Purpose | Preparation status |
|---|---|---|
| `sim/kinematic/scenarios/match_tactics.json` | Seeded P5 case inventory for stage races, adjudication boundaries, both roles and truth isolation | Consumed as case inventory; no inferred official timing or goal geometry |
| `sim/kinematic/README.md` | Declares P5 SIL profile, fixture limitations and truth boundary | Updated |
| `artifacts/reports/phase5/P5_environment_baseline.json` | Machine-readable environment, inherited gate evidence, dependencies and blockers | Prepared |
| `docs/HSL26_PHASE5_IMPLEMENTATION_PLAN.md` | Ordered P5.1-P5.6 implementation and verification plan | P5.6 partial; full integrated baseline blocked |
| `hsl_core/hsl_core/match.py` and `tactics/options.py` | Pure stage and option authority boundaries | P5.1/P5.2 core slices implemented; not connected to a ROS action server |
| `hsl_core/hsl_core/rules.py`, `sim/kinematic/match.py` | Continuous modeled footprint contact, capture interval and role-scoped policy/referee boundary | Bounded kinematic SIL slice; not ROS/MVSim/G4 evidence |
| P5.6, `sim/kinematic/autonomous.py` | Role-scoped selector/authority/plan/safety integration and SIL evidence | Bounded guardian search + explorer observation slice; see `P5.6_autonomous_integration_report.json`; not promotion evidence |

The P5 manifest references existing Phase 3/4 fixtures. Its coordinates, seed
and case inputs are software test data only. They are not competition layout,
calibration, organizer-approved zones, official stage timing or official
scoring. The private referee truth boundary is for evaluation only; policy
inputs must be observations and admitted commands.

### 2.1 Profiles

| Profile | Status | Permitted claims |
|---|---|---|
| `kinematic` + `SYNTHETIC_DETECTION` | Enabled for deterministic SIL preparation | Pure contract, option-selection, race, truth-isolation and regression evidence |
| `replay` | Prepared, data/runtime dependent | Estimator regression after immutable observations and bag-time handling are available |
| `mvsim` | Blocked pending sensor-model/runtime inspection | No sensor-fidelity or two-robot ROS integration claim yet |
| `hardware` / `real` | Disarmed and blocked | No physical motion, stop-response, calibration or competition acceptance |

## 3. Dependencies and setup commands

The current `hsl_core/pyproject.toml` already declares Python-compatible
NumPy, SciPy and pytest dependencies. **No additional library beyond pytest
is required for the planned P5 pure-core/kinematic SIL work.** NumPy and SciPy
are existing package requirements, not new P5 dependencies. JSON fixtures use
the Python standard library.

Open3D and Matplotlib remain optional diagnostics/point-cloud tooling. PyYAML
is not needed for the JSON SIL profile. ROS 2, colcon and MVSim are not
required for pure-core P5 tests; the repository's pinned ROS/Docker workflow
will be needed later for ROS action-server, launch, namespace and topic-graph
integration. No package should be added before a concrete P5 implementation
requires it.

From the repository root in Windows PowerShell:

```powershell
$env:PYTHONPATH = "hsl_core"
python -m pytest hsl_core\tests sim\kinematic -q
python -m compileall -q hsl_core\hsl_core sim
```

Focused P5 commands to use as the work packages add tests:

```powershell
$env:PYTHONPATH = "hsl_core"
python -m pytest sim\kinematic\test_p5_manifest.py -q
python -m pytest sim\kinematic\test_p54_referee.py hsl_core\tests\test_p13_foundation.py -q
python -m pytest sim\kinematic\test_p55_rehearsal.py sim\kinematic\test_p54_referee.py sim\kinematic\test_p5_manifest.py -q
python -m pytest sim\kinematic\test_p56_autonomous.py -q
python -m pytest hsl_core\tests\test_p51_match.py hsl_core\tests\test_p52_option_authority.py hsl_core\tests\test_p53_tactics.py ros_ws\src\hsl_interfaces\test\test_contract_schema.py -q
```

The first focused commands validate the manifest, P5.4 referee geometry and
runner, and P5.5 lifecycle/fault rehearsal. The pure-core command covers
P5.1-P5.3 contracts. There is no P5 ROS action-server adapter, DDS race test,
automatic referee-to-StageManager event wiring or complete architectural
baseline rehearsal.

## 4. Ordered work packages

| Package | Implementation sequence | Core outputs | Exit evidence |
|---|---:|---|---|
| P5.0 Environment and contract freeze | Prepared | This plan, baseline report, role/stage case manifest, profile and blockers | JSON parses; inputs and assumptions are explicit; inherited regression passes |
| P5.1 Stage and semantic zones | 1 | `hsl_core/hsl_core/match.py`: `ResetStage`, `SetGoalZone`, `StartStage`, `ResolveEvent`; stage ID/epoch; freeze/active/terminal timers; reset/epoch acknowledgement barriers; leased MatchState; append-only in-memory transition/request journal | T01-T05/T14/T28/T30; I04/I12/I13; duplicate-start, stale-state, delayed-event, ambiguity and reset-race traces |
| P5.2 Option authority | Implemented in pure core + message-shaped safety boundary | `tactics/options.py`: one `OptionRegistry`, strict goal/context contracts, `OptionAuthority` lifecycle, exact deadline/effect-instance checks, single `ExecutionState` projection, bounded causal journal; lease generation rotates on replan and is carried through `ExecutionState`/`MotionCandidate`; planning lease and safety adapter reject stale generation/stage/map/topology | Dedicated adversarial tests; ROS action server, DDS/QoS, process-failure and G4 evidence remain open |
| P5.3 Role tactics | Implemented in pure core | `tactics/fsm.py`: role/guard-constrained priority selection, evidence-bearing proposals, strict interception timing comparison, explorer ordinal urgency, fixed deterministic tie-break, <=256 proposal work bound, bounded utility/hysteresis/dwell, health/authority gates and mandatory HOLD_SAFE; `tactics/utility.py`: versioned normalized features and L1-bounded score | Dedicated adversarial selector/utility suite and full regression; ROS tactics node, evidence-source authentication, referee integration and G4 remain open |
| P5.4 Referee and simulation integration | Bounded kinematic SIL slice implemented | Piecewise-linear circular-footprint first contact; conservative capture/LOS interval; two injected role-scoped policy callbacks and separate referee truth output. TacticalSelector/OptionAuthority wiring, StageManager event wiring, accepted-zone result, ROS/MVSim and complete rehearsal remain open. | Dedicated P5.4 adversarial tests and existing P5.1 overlapping-event ambiguity tests; I12/I14 partial, G4 not run |
| P5.5 Bounded lifecycle rehearsal | Bounded kinematic lifecycle/fault rehearsal implemented | StageManager-gated snapshots and per-role zero veto; cold start/freeze, sequential role swap, timeout, reset barrier, deterministic trace and clock/planner/sensor/watchdog injections. Test timings are fixture assumptions. | Dedicated P5.5/P5.4/manifest tests; diagnostics, P5.3/OptionAuthority chain, ROS/DDS, MVSim, accepted goal and physical G4 remain BLOCKED/NOT_RUN |
| P5.6 Kinematic decision-loop integration | Bounded slice implemented; full baseline remains open | Role-scoped pose/coverage/topology adapters; Guardian `SEARCH_PORTAL` selector→authority→versioned plan/candidate→lease→supported safety; Explorer no-motion `OBSERVE_SAFE`; preserve the `TwoRobotMatch` referee/transaction boundary | `sim/kinematic/test_p56_autonomous.py` and report; missing opponent belief, Explorer navigation, ROS/MVSim and full T/I evidence keep G4 blocked |

Implement pure-core state/rule/option contracts before ROS wrappers or the
full simulator loop. Add small deterministic fixtures and negative tests at
each step. Do not build tactical policy on an unimplemented guessed start or
goal zone; exercise the unresolved-goal `OBSERVE_SAFE`/`HOLD_SAFE` paths first.

P5.2 core termination accepts an effect only when the evidence is bound to the
active `option_instance_id` and carries a non-empty evidence ID. The effect
producer still must validate the option-specific physical/semantic predicate;
the pure authority does not infer capture, arrival or LOS. The adapter must
translate the ROS action UUID and `ContractHeader` into validated core
contracts, publish the sole `ExecutionState`, and use the same
`ExecutionLease` as candidate generation.

P5.3 consumes proposals from role-specific perception/planning/referee
assessors; it does not derive visibility, capture, physical escape or route
feasibility from truth. Every guard and ordinal urgency input carries an
evidence ID for traceability, but this core cannot authenticate the producer.
Utility weights are explicit versioned inputs; test profiles are fixtures,
not tuned or competition-approved deployment policy. Tactics outputs an
`OptionGoal` proposal only; `OptionAuthority` still performs final admission.

## 5. Normative test and scenario sequence

Use the technical specification as the acceptance authority; the manifest
only indexes prepared cases and does not redefine rule values.

1. **Rule boundaries and event geometry:** T01-T05 and T14. Capture requires
   `distance < 0.45 m`, inclusive `abs(bearing) <= pi/4`, and LOS exactly
   `TRUE`; `UNKNOWN` blocks certification. The footprint's first zone contact
   is the event, not center entry. For the fixture in the specification, a
   0.22 m radius at an edge at x=1.0 first contacts at center x=0.78.
2. **Adjudication and semantic data:** T28/T30 and I04/I12/I13. Test missing
   or unresolved goal metadata, stage/epoch reset, stale transient-local
   state, timeout and concurrent event intervals. An estimate may request a
   local stop/hold without being promoted to official score. Overlapping
   terminal intervals without an approved ordering produce `AMBIGUOUS` and
   hold.
3. **Action authority and cancellation:** T21/T22 and I11. Revoke and cancel
   the active instance before replacement; a delayed candidate from an old
   instance, stage or localization epoch is rejected, never resurrected.
4. **Both role policies and shared collision constraints:** T28/T29 and
   I12/I13. Exercise guardian search/intercept/pressure/recover/capture/
   defense and explorer advance/break-LOS/alternate-route/escape/observe,
   including infeasible and unresolved-goal cases. An opponent blocks passage
   for either role; utility cannot override route or safety feasibility.
5. **Integrated rehearsal:** I04/I08/I11-I14. Run two isolated roles through
   cold start, preparation freeze, active behavior, role swap, terminal and
   reset. Inject stage, planner, sensor and watchdog faults. Inspect runtime
   topic/namespace graphs when the ROS integration profile becomes available.

For every test/evidence row record `PASS`, `FAIL`, `BLOCKED` or `NOT_RUN`,
profile/mode, exact command, seed, source/config hashes, inputs, expected and
observed result, event-time intervals and limitations. A mock or kinematic
PASS never transfers automatically to MVSim or hardware.

## 6. P5 gate and open decisions

The P5 implementation target is **G4 in a declared profile**, not real
competition readiness. The current P5.5 report records G4 as
`BLOCKED_NOT_RUN`. P5.6 implementation and all applicable tests/review are
still required for an integrated kinematic-policy claim. A kinematic profile
result cannot close ROS/runtime, MVSim/replay fidelity, physical safety or
organizer-approval gates; preserve each status separately.

Before claiming goal arrival, base defense or official result, obtain and
version the accepted zone provider, coordinates, frame, authorization and
uncertainty. Keep the following organizer-dependent values explicitly
unknown until approved: official stage-start trigger, stage/timeout scoring,
simultaneous-event ordering, tie handling, and permission for map or memory
retention across role/stage reset. Test-only timers and fixture values must be
labelled assumptions and must never silently become official configuration.

## 7. Baseline verification and handoff

On 2026-09-26, before adding this P5 preparation, the inherited regression
passed on the recorded Windows host:

```text
$env:PYTHONPATH = "hsl_core"
python -m pytest hsl_core\tests sim\kinematic -q
213 passed
```

The entry baseline is inherited core/kinematic evidence only. It did not
exercise P5 lifecycle, option races, role integration, G4 or an executable
scenario. P5.1 implementation and test results are reported separately in
[`P5.1_stage_lifecycle_report.json`](../artifacts/reports/phase5/P5.1_stage_lifecycle_report.json).

The P5.0 preparation regression was **216 passed in 2.87s**; the three new
checks validated only the manifest's schema envelope, explicit
unresolved inputs, role/case inventory and truth-field separation. The
manifest-focused command reports **3 passed**. `compileall` and `git diff
--check` also pass; these results are recorded in
`artifacts/reports/phase5/P5_environment_baseline.json`.

This is the entry snapshot, not the current implementation status. P5.4 and
the bounded kinematic P5.5 rehearsal evidence are recorded separately in
[`P5.4_referee_integration_report.json`](../artifacts/reports/phase5/P5.4_referee_integration_report.json)
and [`P5.5_rehearsal_report.json`](../artifacts/reports/phase5/P5.5_rehearsal_report.json).
Keep `P5_environment_baseline.json` as the entry snapshot; maintain separate
work-package result reports and a G4 acceptance record only when each result
is backed by complete profile-specific executable evidence. Do not overwrite
inherited P1-P4 reports or convert external/runtime blockers into P5 passes.
The autonomous integration work is P5.6, not an inferred property of the
P5.5 fixed-Actuation callbacks. The independent review and interface
constraints are documented in
[`HSL26_PHASE6_INDEPENDENT_AUDIT.md`](HSL26_PHASE6_INDEPENDENT_AUDIT.md).

### 7.1 P5.1 implementation boundary

The pure stage manager serializes requests under one lock and derives
`motion_authorized`; callers cannot set the permission flag. It uses monotonic
integer nanosecond timestamps, requires explicit start/freeze/deadline/lease
durations and an explicit zone permission window, and rejects clock reversal,
overflow, stale stage IDs, stale epochs and unapproved organizer references.
Reset returns a new never-reused stage ID and remains disarmed until action,
track and memory reset acknowledgements arrive. Localization epoch changes
invalidate estimate holds and block authority until the action and track
consumers acknowledge invalidation. Clock epoch changes create a new INIT
stage. Reset waits for actions, execution, plans, candidates, tracks and memory
invalidation acknowledgements; localization-epoch changes additionally wait
for map/world invalidation. Request-cache, per-stage event and audit-journal capacities are
explicit profile inputs; exhausted request/event capacity fails closed and
audit truncation increments a visible dropped-record counter.

Accepted zones require a validated simple counterclockwise polygon, explicit
approved frame, finite non-negative position-error bound, provider, approval
reference and SHA-256 provenance. Candidate zones retain their metadata but
never populate authoritative MatchState zone IDs. An unresolved zone carries
no guessed geometry. The service adapter must additionally authenticate
provenance and validate coordinate bounds/frame transforms before invoking
this core API.

Event adjudication consumes event-time intervals before applying the current
deadline transition. Only a proven event interval entirely before the deadline
can win over delayed timeout delivery. An interval touching the deadline is
not proven earlier. Competing distinct terminal kinds whose closed intervals
overlap become `AMBIGUOUS`, independent of event arrival order. Competing
estimates hold motion until all conflicting estimates are explicitly resolved;
an estimate arriving after timeout is retained only for adjudication and does
not re-open motion. Estimates create only a local active-stage hold; they
require an authorized explicit resolution to be dismissed or confirmed, and
never directly assign official score. Physical capture/LOS and swept-footprint
arrival predicates remain upstream evidence producers; this stage manager does
not infer geometry.

P5.1 evidence is pure-core pytest evidence only. It does not implement or
validate a ROS node, ROS service authentication, DDS QoS/transient-local
behavior, process-failure cancellation, two-robot simulation, or G4.

P5.1 validation completed with **64 dedicated adversarial tests passed**,
**77 passed** across P5.1, normative rule fixtures and the interface schema,
and **321 passed** in the combined core, kinematic, interface and safety
regression. `compileall`, JSON validation and `git diff --check` passed. Exact
commands and results are recorded in
[`P5.1_stage_lifecycle_report.json`](../artifacts/reports/phase5/P5.1_stage_lifecycle_report.json).

### 7.2 P5.4 bounded kinematic integration

The pure rule layer detects the first contact of a circular footprint against
a validated simple polygon along piecewise-linear trajectory segments,
including between-sample enter/exit and vertex tangency. Initial footprint
overlap is a setup error. Capture requires simultaneous strict range,
inclusive bearing and clear LOS. The interval checker certifies range/bearing
over a subinterval and rejects any subinterval whose conservative swept LOS
envelope intersects a static segment or circular occluder. Ambiguous
subdivisions fail closed at the fixed depth, so bounded SIL can miss very
narrow/isolated events; it does not widen rule thresholds or promote unknown
LOS.

`sim/kinematic/match.py` steps guardian and explorer plants independently,
uses distinct absolute role namespaces and gives each callback only its own
role/namespace/clock and observation. Referee truth is returned separately
for evaluation, never passed to either callback. Its explicit
`capture_rule_event` adapter emits only `SIM_TRUTH` and still requires the
caller to supply stage/epoch/provenance/authorization. This is not a ROS
namespace graph, automatic StageManager integration, approved target-zone/
arrival result, MVSim or G4 test. Exact P5.4 outcomes and limits are recorded
in [`P5.4_referee_integration_report.json`](../artifacts/reports/phase5/P5.4_referee_integration_report.json).

### 7.3 P5.5 bounded rehearsal

`TwoRobotMatch` can be given a `StageManager`. Each policy callback receives
the current leased `MatchState`; before plant integration the runner itself
vetoes motion unless the stage authorizes motion for that endpoint's role and
the complete tick ends no later than both the lease expiry and stage deadline.
A tick that straddles either boundary is zeroed in full. Watchdog-unhealthy
bypasses both policy callbacks and supplies zeros while simulation time
advances. Sensor, planner and referee exceptions propagate before either
plant commits; both predicted poses and truth evaluation complete before
paired plant integration.

The rehearsal tests cold start, freeze, active guardian, stage timeout,
terminal stop, acknowledged reset and explorer role swap; fault injections
cover stage clock reversal, sensor exception, planner exception, invalid
paired command and watchdog loss. Same-seed traces must compare exactly.
Stage durations and tick intervals are explicitly test-only. No accepted
zone is supplied, and simulated truth is rejected by a profile that has not
enabled simulation-truth adjudication.

This does not complete full-baseline P5.5/G4: the runner uses injected raw
`Actuation` callbacks, not the P5.3 selector/P5.2 authority/P1 safety and mux
chain. No diagnostics UI, ROS/DDS, MVSim, process watchdog or hardware was
available/verified. Results and a per-evidence status matrix are recorded in
[`P5.5_rehearsal_report.json`](../artifacts/reports/phase5/P5.5_rehearsal_report.json).
