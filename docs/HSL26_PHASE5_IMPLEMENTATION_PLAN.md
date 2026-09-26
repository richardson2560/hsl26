# HSL26 - Phase 5 implementation plan and SIL environment

**Revision:** 1.0 · **Date:** 2026-09-26  
**Authority:** `HSL26_TECHNICAL_SPECIFICATION.md` §§10-11, 14, 17;
`HSL26_FINAL_ARCHITECTURE.md`; `HSL26_PHASE5_MATCH_AND_TACTICS.md`.  
**Entry decision:** `READY_FOR_SIL_IMPLEMENTATION_WITH_EXTERNAL_GATE_BLOCKERS`  
**Current scope:** deterministic software-in-the-loop (SIL); no physical robot.

## 1. Purpose, entry conditions and limits

This plan records the no-hardware SIL preparation and implementation sequence
for stage lifecycle, adjudication, option authority and both tactical roles.
It follows the evidence-led setup used for Phases 3 and 4. P5.1 and the pure
core of P5.2 are implemented and tested; this does not claim G4, ROS runtime
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
  infrastructure under `sim/kinematic/`. It is not yet a two-robot P5 match
  runner, and the present referee only exposes its existing collision result.

Therefore P5 may start with the kinematic + `SYNTHETIC_DETECTION` profile for
contract and policy work. Entry to real motion, G4 release or an official
match result remains blocked until the relevant upstream gates and external
rule/zone inputs are accepted. Missing goal metadata must remain unresolved;
it must not be guessed from the map, farthest node or fixture coordinates.

## 2. Environment prepared

| Asset | Purpose | Preparation status |
|---|---|---|
| `sim/kinematic/scenarios/match_tactics.json` | Seeded P5 case inventory for stage races, adjudication boundaries, both roles and truth isolation | Prepared as a test fixture manifest; not yet consumed by a P5 runner |
| `sim/kinematic/README.md` | Declares P5 SIL profile, fixture limitations and truth boundary | Updated |
| `artifacts/reports/phase5/P5_environment_baseline.json` | Machine-readable environment, inherited gate evidence, dependencies and blockers | Prepared |
| `docs/HSL26_PHASE5_IMPLEMENTATION_PLAN.md` | Ordered P5.1-P5.5 implementation and verification plan | Prepared |
| `hsl_core/hsl_core/match.py` and `tactics/options.py` | Pure stage and option authority boundaries | P5.1/P5.2 core slices implemented; not connected to a ROS action server |
| Existing planning, control, perception and `sim/kinematic` code | Reusable pure-core and SIL contracts | Available; P5 end-to-end behavior is not yet implemented |

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
python -m pytest hsl_core\tests\test_p51_match.py hsl_core\tests\test_p52_option_authority.py ros_ws\src\hsl_interfaces\test\test_contract_schema.py -q
```

The first command validates only the prepared manifest and existing Phase-3
scenario envelope. The focused command covers P5.1/P5.2 pure-core contracts.
There is no P5 ROS action-server adapter, DDS race test or two-robot match
runner yet.

## 4. Ordered work packages

| Package | Implementation sequence | Core outputs | Exit evidence |
|---|---:|---|---|
| P5.0 Environment and contract freeze | Prepared | This plan, baseline report, role/stage case manifest, profile and blockers | JSON parses; inputs and assumptions are explicit; inherited regression passes |
| P5.1 Stage and semantic zones | 1 | `hsl_core/hsl_core/match.py`: `ResetStage`, `SetGoalZone`, `StartStage`, `ResolveEvent`; stage ID/epoch; freeze/active/terminal timers; reset/epoch acknowledgement barriers; leased MatchState; append-only in-memory transition/request journal | T01-T05/T14/T28/T30; I04/I12/I13; duplicate-start, stale-state, delayed-event, ambiguity and reset-race traces |
| P5.2 Option authority | Implemented in pure core + message-shaped safety boundary | `tactics/options.py`: one `OptionRegistry`, strict goal/context contracts, `OptionAuthority` lifecycle, exact deadline/effect-instance checks, single `ExecutionState` projection, bounded causal journal; lease generation rotates on replan and is carried through `ExecutionState`/`MotionCandidate`; planning lease and safety adapter reject stale generation/stage/map/topology | Dedicated adversarial tests; ROS action server, DDS/QoS, process-failure and G4 evidence remain open |
| P5.3 Role tactics | 3 | Guardian and explorer feasible-option selection, bounded utility, deterministic tie-break, hysteresis/dwell and safe fallback | T28/T29; I12/I13; per-role selected-option, alternatives and reason trace |
| P5.4 Referee and simulation integration | 4 | Continuous first-contact and capture/LOS event evaluation; interval ordering/ambiguity; two isolated robot namespaces and referee-only truth | T01-T05/T14; I12/I14; truth/evidence and isolation comparison |
| P5.5 Full-baseline rehearsal | 5 | Cold start, freeze, both roles, role swap, terminal/reset, injected faults, diagnostics and runbook | I04/I08/I11-I14; G4 matrix with profile-specific evidence |

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

The P5 implementation target is **G4 in a declared SIL profile**, not real
competition readiness. G4 remains `NOT_RUN` until the listed P5 implementation,
test and integration evidence exists. Preserve separate gate status for
software contracts, ROS runtime, MVSim/replay fidelity, physical safety and
organizer approval.

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

The next implementation task is P5.2 option authority, reusing P5.1 stage IDs,
clock/localization epochs, reset barrier and leases. Keep
`P5_environment_baseline.json` as the entry snapshot; append work-package
result reports under `artifacts/reports/phase5/` and a separate G4 acceptance
record only when each result is backed by executable evidence. Do not
overwrite inherited P1-P4 reports or convert their blockers into P5 passes.

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
