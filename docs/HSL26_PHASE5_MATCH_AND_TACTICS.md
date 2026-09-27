# HSL26 — Phase 5: Stage Lifecycle, Roles and End-to-End Baseline

**Revision:** 1.0 · **Baseline:** architecture/blueprint revision 2.2; roadmap revision 1.0.  
**Objective:** run the deterministic baseline through both guardian and explorer stages while preserving the adjudication, authority and safety boundaries. **Exit:** G4 in the declared profile; real competition readiness also requires organizer-dependent inputs.

Execution sequencing, the prepared no-hardware SIL profile, dependency decision
and evidence status are recorded in
[`HSL26_PHASE5_IMPLEMENTATION_PLAN.md`](HSL26_PHASE5_IMPLEMENTATION_PLAN.md)
and [`P5_environment_baseline.json`](../artifacts/reports/phase5/P5_environment_baseline.json).
P5.4's bounded kinematic evidence is recorded in
[`P5.4_referee_integration_report.json`](../artifacts/reports/phase5/P5.4_referee_integration_report.json).
P5.5's bounded lifecycle/fault rehearsal and blocked gate matrix are recorded
in [`P5.5_rehearsal_report.json`](../artifacts/reports/phase5/P5.5_rehearsal_report.json).
P5.1-P5.3 pure-core slices and the bounded kinematic P5.4 referee/runner are
implemented; P5.5 adds a StageManager-gated, fault-injected kinematic
rehearsal. These results do not claim TacticalSelector/OptionAuthority
end-to-end wiring, ROS integration, MVSim fidelity or G4 completion. See the
phase-specific evidence reports for exact scope and verification.

## 1. Entry and rule inputs

Requires G0 software and G2 route contracts plus a G3 track/belief profile or a clearly labeled degraded detector with equivalent schema. Real motion requires physical G0/G1. Before claiming goal arrival or base defense, the own start and target-zone provider, coordinates, frame, authorization and uncertainty must be accepted. Numerical official scoring, simultaneous events, retention permission and stage-start trigger remain external inputs until approved; simulation assumptions are labeled separately.

## 2. Work packages

| ID / owner role | Subtasks and target files | Observable deliverable | Verification |
|---|---|---|---|
| P5.1 Stage and semantic zones | **a** implement idempotent `ResetStage`, `SetGoalZone`, `StartStage`, `ResolveEvent`; **b** freeze/active/terminal timers; **c** atomic reset of actions/epochs with approved map retention; **d** leased `MatchState` and authenticated provenance. `stage_manager_node.py`, `rules.py`, memory/score profiles. | Transition/request journal with explicit estimated vs official event evidence. | T01–T05/T14/T28/T30; I04/I12/I13. |
| P5.2 Option authority | **Core implemented:** one `OptionRegistry` defines role/target/initiation/invariant/termination checks; `OptionAuthority` enforces admission, lifecycle, cancel/revoke, unique identities and one core `ExecutionState`; replan rotates `lease_generation`, propagated by `ExecutionState`/`MotionCandidate` and checked at the safety boundary with stage/map/topology identity. **Still required:** ROS action-server feedback/result adapter, `option_executor_node.py`, `navigation/adapters.py`, DDS/process-race integration. | Bounded causal core journal and adversarial cancellation/effect/lease traces; ROS-level traces remain open. | Pure-core/adapter-contract tests pass; T21/T22 and I11 integration evidence remains open. |
| P5.3 Role tactics | **Pure core implemented:** guardian/explorer option allowlists and required guard evidence; stage/health/safety gates; explicit per-role priority classes; discrete evidence-bearing explorer escape urgency; strict integer-nanosecond intercept-vs-base-defense interval test; normalized, versioned bounded utility; deterministic tie-break; utility-unit hysteresis and nanosecond dwell; mandatory `HOLD_SAFE`. `tactics/fsm.py`, `utility.py`. **Still required:** proposal assessors, `tactics_node.py`, authenticated evidence-source adapters and SIL/referee integration. | Each `SelectionResult` contains the chosen goal, priority, utility contributions, profile identity and ordered alternative evaluations with applicability, rejection reason and evidence IDs. | Pure-core tests pass; T28/T29 and I12/I13 end-to-end evidence remains open. |
| P5.4 Referee and simulation integration | **Implemented bounded SIL slice:** continuous first contact for circular footprints on piecewise-linear trajectories; strict capture/bearing/LOS evaluation with conservative swept-LOS checks; role-scoped two-robot runner with injected callbacks and separate referee result. **Still open:** wiring P5.3 TacticalSelector/OptionAuthority, StageManager events, accepted-zone arrival, replay/MVSim, ROS namespaces/DDS, complete I12/I14 and G4. `hsl_core/hsl_core/rules.py`, `sim/kinematic/match.py`. | Truth/evidence contracts and role-isolated kinematic tick; not an official result or complete match rehearsal. | `sim/kinematic/test_p54_referee.py`; existing P5.1 overlap-order tests; I12/I14 remain partial. |
| P5.5 Full-baseline rehearsal | **Implemented bounded SIL rehearsal:** cold start/freeze, exclusive guardian and explorer stage roles, timeout/terminal, reset acknowledgement barrier, deterministic replay and injected clock/planner/sensor/watchdog faults. Fixed-size outputs/traces are asserted without drawing official score from simulation truth. **Blocked:** P5.3/OptionAuthority end-to-end, ROS UI/topic graph, MVSim, accepted zone, physical stop and G4. `sim/kinematic/match.py`, `sim/kinematic/test_p55_rehearsal.py`. | Reproducible test-only lifecycle trace with explicit development timing assumptions; no competition result. | P5.5 matrix/report; integration I04/I08/I11–I14 and G4 remain BLOCKED/NOT_RUN where runtime/evidence is absent. |

## 3. Milestones

| Milestone | Required result |
|---|---|
| M5.1 Stage ownership | One stage identity and timer origin; repeated requests idempotent; old epochs/actions cannot command. |
| M5.2 Both role policies | Feasible option selection with complete effect/termination/failure semantics and no direct driver writes. |
| M5.3 G4 integrated baseline | Both stages, event order, restart and safety faults behave according to blueprint; log official unknowns as unknown. |

## 4. Gate G4 acceptance record

Run T01–T05/T14/T21/T22/T28–T30 and I04/I11–I14; attach inputs, event time intervals, actor identities, old/new stage/option IDs, command admission traces and role outcomes. Mark each profile separately. Capture requires strict distance, inclusive bearing and clear LOS at the same time. An estimated event may halt movement without becoming an official score. Missing accepted goal metadata leaves arrival/base-defense capability BLOCKED, not guessed from the farthest node. Record pending organizer decisions before any real competition release.

**Required negative cases:** distance exactly 0.45 m does not certify capture; UNKNOWN LOS blocks certification; a footprint crossing a zone between two samples produces the first-contact event; duplicate `StartStage` never resets elapsed time; old goals or transient-local match data cannot rearm a new stage; an ambiguous simultaneous terminal interval does not invent a winner. Exercise guardian and explorer separately, then the role swap with allowed memory retention.

### P5.4 implementation boundary

`first_footprint_arrival` treats each pair of supplied center samples as a
linear path segment and solves edge-offset and vertex-circle contact
continuously; starting with an intersecting footprint is rejected. The
referee's capture checker interpolates paired poses linearly, requires strict
range, inclusive directed bearing and unblocked LOS concurrently, and only
accepts a positive interval when its distance/bearing bounds and swept LOS
envelope certify the predicate against static segments and circular
occluders. It subdivides ambiguous intervals to a fixed depth and fails closed
at that bound; a sufficiently narrow or isolated capture may consequently be
missed rather than falsely certified.

`TwoRobotMatch` calls each policy with only its role, namespace, clock and
sensor observation. The separate returned referee truth record is evaluation
output and is not fed back into policy callbacks. This is an executable
kinematic SIL harness, not ROS namespace/DDS isolation evidence. The harness
does not guess or supply a goal zone, stage trigger, event scoring policy or
memory-retention rule, and does not feed simulated truth directly into
`StageManager`; the explicit `capture_rule_event` adapter emits only
`SIM_TRUTH` evidence and still requires a caller-configured simulation profile
and authorization. Event-order ambiguity remains covered by the separate
P5.1 adjudication tests. No P5.4 result closes G4 or transfers to MVSim/hardware.

### P5.5 bounded rehearsal procedure and result

Run from the repository root in Windows PowerShell:

```powershell
$env:PYTHONPATH = "hsl_core;ros_ws\src\hsl_safety"
python -m pytest sim\kinematic\test_p55_rehearsal.py sim\kinematic\test_p54_referee.py sim\kinematic\test_p5_manifest.py -q
```

The rehearsal fixture uses a **0.1 s freeze**, **0.3 s stage** and **0.05 s
tick** strictly as test parameters. They are not official rules or manifest
defaults. The runner supplies a leased `MatchState` snapshot to both isolated
policy callbacks and independently zeros both actuation commands unless the
snapshot authorizes motion for that tick and assigned role, and the whole
tick fits within both its lease and stage deadline. A tick spanning either
boundary is held in full rather than extrapolating authority past it; this is
conservative and can shorten motion near expiry.
Reset remains disarmed until every required downstream acknowledgement is
present; memory retention is disabled in this fixture.

The adversarial rehearsal covers cold start, preparation freeze, guardian
active motion, timeout, role swap to explorer, terminal-event stop, partial
and complete reset acknowledgements, lease/deadline-spanning ticks, clock
regression, sensor/planner failure, invalid paired commands, watchdog fault,
simulation-truth non-promotion, unresolved zones and byte-stable same-seed
replay. Sensor, callback and referee exceptions are surfaced; both next poses
are predicted and adjudicated before plant commit, so a failed tick does not
advance either plant or the runner clock. Watchdog zero holds pose while the
simulation clock continues, and does not alter stage result.

This remains a **bounded harness**, not the complete architectural baseline:
callbacks return kinematic `Actuation` directly, so P5.3 selection,
`OptionAuthority`, candidate safety/mux, production watchdog process,
diagnostics/HUD, ROS namespace/DDS, MVSim and physical-stop chains are not
exercised. The stage-role model is sequential and does not imply simultaneous
competition rules. Accepted goal-zone inputs remain unresolved; therefore no
arrival/base-defense claim is produced. See
[`P5.5_rehearsal_report.json`](../artifacts/reports/phase5/P5.5_rehearsal_report.json).

## 5. Handoff to Phase 6

Freeze a deterministic baseline and option/feature registries. Supply per-role complete transition logs, scenario/opponent bank, provenance and G0–G4 gate records. Learning may be skipped while the baseline proceeds to release acceptance.
