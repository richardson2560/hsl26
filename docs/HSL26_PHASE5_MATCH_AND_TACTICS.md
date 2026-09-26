# HSL26 — Phase 5: Stage Lifecycle, Roles and End-to-End Baseline

**Revision:** 1.0 · **Baseline:** architecture/blueprint revision 2.2; roadmap revision 1.0.  
**Objective:** run the deterministic baseline through both guardian and explorer stages while preserving the adjudication, authority and safety boundaries. **Exit:** G4 in the declared profile; real competition readiness also requires organizer-dependent inputs.

Execution sequencing, the prepared no-hardware SIL profile, dependency decision
and evidence status are recorded in
[`HSL26_PHASE5_IMPLEMENTATION_PLAN.md`](HSL26_PHASE5_IMPLEMENTATION_PLAN.md)
and [`P5_environment_baseline.json`](../artifacts/reports/phase5/P5_environment_baseline.json).
P5.1 and pure-core P5.2 slices are implemented; these do not change the
normative contracts below or claim ROS integration/G4 completion. See the
phase-specific evidence reports for exact scope and verification.

## 1. Entry and rule inputs

Requires G0 software and G2 route contracts plus a G3 track/belief profile or a clearly labeled degraded detector with equivalent schema. Real motion requires physical G0/G1. Before claiming goal arrival or base defense, the own start and target-zone provider, coordinates, frame, authorization and uncertainty must be accepted. Numerical official scoring, simultaneous events, retention permission and stage-start trigger remain external inputs until approved; simulation assumptions are labeled separately.

## 2. Work packages

| ID / owner role | Subtasks and target files | Observable deliverable | Verification |
|---|---|---|---|
| P5.1 Stage and semantic zones | **a** implement idempotent `ResetStage`, `SetGoalZone`, `StartStage`, `ResolveEvent`; **b** freeze/active/terminal timers; **c** atomic reset of actions/epochs with approved map retention; **d** leased `MatchState` and authenticated provenance. `stage_manager_node.py`, `rules.py`, memory/score profiles. | Transition/request journal with explicit estimated vs official event evidence. | T01–T05/T14/T28/T30; I04/I12/I13. |
| P5.2 Option authority | **Core implemented:** one `OptionRegistry` defines role/target/initiation/invariant/termination checks; `OptionAuthority` enforces admission, lifecycle, cancel/revoke, unique identities and one core `ExecutionState`; replan rotates `lease_generation`, propagated by `ExecutionState`/`MotionCandidate` and checked at the safety boundary with stage/map/topology identity. **Still required:** ROS action-server feedback/result adapter, `option_executor_node.py`, `navigation/adapters.py`, DDS/process-race integration. | Bounded causal core journal and adversarial cancellation/effect/lease traces; ROS-level traces remain open. | Pure-core/adapter-contract tests pass; T21/T22 and I11 integration evidence remains open. |
| P5.3 Role tactics | **a** guardian search/intercept/pressure/recover/capture/defense selection; **b** explorer advance/LOS/alternate-route/escape/observe selection; **c** capability and role guards; **d** bounded utility, hysteresis and dwell; **e** safe fallback on unresolved goal. `tactics/fsm.py`, `utility.py`, `tactics_node.py`. | Per-role decision trace with selected option, alternatives and reason. | T28/T29, I12/I13. |
| P5.4 Referee and simulation integration | **a** continuous first-contact arrival and strict capture/bearing/LOS predicate; **b** handle interval order ambiguity; **c** two robot namespaces with private truth referee; **d** replay/kinematic/MVSim declared fidelity; **e** separate safety stop from official result. `sim/common/referee.py`, `sim/kinematic/*`, launch files. | Truth/evidence comparison and both-role run logs. | T01–T05/T14, I12/I14. |
| P5.5 Full-baseline rehearsal | **a** cold start; **b** preparation freeze; **c** active role swap and terminal/reset; **d** inject sensor/planner/watchdog/stage faults; **e** capture UI/log and metrics without blocking motion. `hsl_diagnostics/terminal_hud_node.py`, runbook and report. | Reproducible two-stage baseline traces in the declared simulation profile. | I04/I08/I11–I14; G4 matrix. |

## 3. Milestones

| Milestone | Required result |
|---|---|
| M5.1 Stage ownership | One stage identity and timer origin; repeated requests idempotent; old epochs/actions cannot command. |
| M5.2 Both role policies | Feasible option selection with complete effect/termination/failure semantics and no direct driver writes. |
| M5.3 G4 integrated baseline | Both stages, event order, restart and safety faults behave according to blueprint; log official unknowns as unknown. |

## 4. Gate G4 acceptance record

Run T01–T05/T14/T21/T22/T28–T30 and I04/I11–I14; attach inputs, event time intervals, actor identities, old/new stage/option IDs, command admission traces and role outcomes. Mark each profile separately. Capture requires strict distance, inclusive bearing and clear LOS at the same time. An estimated event may halt movement without becoming an official score. Missing accepted goal metadata leaves arrival/base-defense capability BLOCKED, not guessed from the farthest node. Record pending organizer decisions before any real competition release.

**Required negative cases:** distance exactly 0.45 m does not certify capture; UNKNOWN LOS blocks certification; a footprint crossing a zone between two samples produces the first-contact event; duplicate `StartStage` never resets elapsed time; old goals or transient-local match data cannot rearm a new stage; an ambiguous simultaneous terminal interval does not invent a winner. Exercise guardian and explorer separately, then the role swap with allowed memory retention.

## 5. Handoff to Phase 6

Freeze a deterministic baseline and option/feature registries. Supply per-role complete transition logs, scenario/opponent bank, provenance and G0–G4 gate records. Learning may be skipped while the baseline proceeds to release acceptance.
