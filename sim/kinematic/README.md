# HSL26 Phase-3 kinematic testbed

This is the no-hardware baseline for Phase 3. It is deliberately separate from
ROS and from the policy: `KinematicPlant` accepts bounded actuation,
`LidarSensor` exposes observations only, and `Referee` owns ground truth.
The first-hit raycaster is deterministic and makes blind sectors explicit
invalid measurements rather than pretending they are free space.

## Profiles

- `observed_map`: only the supplied observed-free/occupied grid is available.
- `approved_map`: an immutable, provenance-tagged map profile is available.
- `oracle`: referee-only truth for assertions; it must never be imported by
  planning, control or policy code.

The initial scenario is `scenarios/parallel_corridors.json`. Its seed and
geometry are development fixtures, not competition coordinates or calibration.
Use it to implement P3.1/P3.2/P3.5 negative cases before adding MVSim.

## Phase-4 opponent-perception fixture

`scenarios/opponent_perception.json` extends the same deterministic boundary
for P4. It supplies labels and bounds for stationary, moving, partial-view,
blind-sector, occluded and reappearing opponent sequences. The hidden
opponent pose and edge progress remain referee-only. A generated measurement
must be labelled `SYNTHETIC_DETECTION` and must include the scenario seed,
map/topology versions, localization epoch, observation stamp and sensor frame.
This fixture validates association, covariance, finite-speed belief and
rejection behavior; it is not evidence for real-cloud GPIS accuracy.

The testbed must remain deterministic: use the scenario seed, keep truth in the
referee boundary, record map/topology/path versions, and emit traces that can
be replayed byte-for-byte. Sensor randomness has its own seed and is never
shared with plant or referee state. No physical authority is granted by this
testbed. The MVSim adapter remains a separate integration boundary and is not
claimed as validated by the kinematic tests.

## Phase-5 match and tactics preparation

`scenarios/match_tactics.json` indexes the planned stage, adjudication,
authority-race and both-role SIL cases. It reuses the Phase-3/4 fixtures and
is explicitly a case manifest, not yet an executable two-robot match runner.
Its role poses and seed are development inputs only. Stage durations, official
start trigger, goal-zone provider/coordinates and memory-retention permission
remain unresolved; null or unresolved values must not be inferred from the
fixture or map.

The policy must receive only its own observations, versioned topology, leased
match state and admitted commands. Opponent/goal truth and official event
truth remain referee/evaluator-only. Estimated capture/arrival may cause a
safe local hold, but must not be promoted to an official result. Hardware,
MVSim and G4 acceptance remain separate from this SIL preparation. See
[`HSL26_PHASE5_IMPLEMENTATION_PLAN.md`](../../docs/HSL26_PHASE5_IMPLEMENTATION_PLAN.md)
and [`P5_environment_baseline.json`](../../artifacts/reports/phase5/P5_environment_baseline.json).

## Optional visualizer

`visualizer.py` is a diagnostic satellite and is not imported by policy,
planning, control or the referee. It has two headless renderers:

- `render_top_view(...)` returns a deterministic RGB image with walls, dynamic
  targets, heading, valid beams and invalid/blind beams.
- `render_doom(...)` returns a deterministic 2.5D RGB projection using
  perpendicular distance correction, so oblique rays do not create fish-eye
  wall inflation.

`show_top_view(...)` and `show_doom(...)` provide optional Matplotlib display
helpers. Matplotlib is intentionally lazy-imported and is not a required
runtime dependency or safety-path dependency. The renderers consume the
observation contract only; they do not access the referee result or infer
target identity from a range.
