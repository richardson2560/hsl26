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

`scenarios/match_tactics.json` indexes the stage, adjudication, authority-race
and both-role SIL cases. `match.py` provides a deterministic two-role
kinematic runner with role-scoped inputs to injected policy callbacks, an
optional `StageManager` lease projection, a runner-level per-role command
veto and a separate referee result. The P5.5 fixture covers lifecycle and
fault injection, but does not wire `TacticalSelector`/`OptionAuthority`, the
P1 safety/mux chain or ROS/DDS; nor does it provide an accepted goal zone or
official scoring service. Its role poses and seed
are development inputs only. Stage durations, official start trigger,
goal-zone provider/coordinates and memory-retention permission remain
unresolved; null or unresolved values must not be inferred from the fixture
or map.

The policy must receive only its own observations, versioned topology, leased
match state and admitted commands. Opponent/goal truth and official event
truth remain referee/evaluator-only. The capture interval checker uses
piecewise-linear pose interpolation, conservative LOS swept envelopes and
bounded subdivision; unresolved intervals fail closed and can be missed at
the finite refinement limit. Arrival uses continuous first contact of a
circular footprint along piecewise-linear trajectory segments, and rejects
initial overlap. These are kinematic model contracts, not formal exact-
arithmetic or sensor-fidelity proofs. Estimated capture/arrival may cause a
safe local hold, but must not be promoted to an official result. Hardware,
ROS runtime, MVSim and G4 acceptance remain separate from this SIL runner. See
[`HSL26_PHASE5_IMPLEMENTATION_PLAN.md`](../../docs/HSL26_PHASE5_IMPLEMENTATION_PLAN.md)
and [`P5_environment_baseline.json`](../../artifacts/reports/phase5/P5_environment_baseline.json).
The bounded P5.5 evidence matrix is in
[`P5.5_rehearsal_report.json`](../../artifacts/reports/phase5/P5.5_rehearsal_report.json).

`autonomous.py` is the bounded P5.6 slice: it composes per-beam coverage,
estimated pose, versioned topology, tactics, authority, routing, control and
safety. The optional simultaneous SIL profile adds scan-residual opponent
extraction and a four-state filter, Guardian pursuit, and Explorer synthetic
goal/escape navigation. Curved motion requires the implemented swept-scan
coverage, braking and motion-limit evidence. The default exclusive-role
profile remains distinct. All parameters and inputs are synthetic fixtures,
not hardware calibration, accepted competition zones, real-cloud HGW or G4
acceptance. In the simultaneous profile, tracked Guardian pursuit and
Explorer navigation currently return one proposal; weight-search sensitivity
must be diagnosed before population-scale training. See
[`P5.6_autonomous_integration_report.json`](../../artifacts/reports/phase5/P5.6_autonomous_integration_report.json)
and [`test_p56_autonomous.py`](test_p56_autonomous.py).

The [coevolution plan rev. 2.0](../../docs/HSL26_COEVOLUTION_PLAN.md) retains
this backend, defers a separate light simulator and requires independent
evaluation before any candidate promotion. The current benchmark supports
CAPTURE/TIMEOUT only and uses a synthetic 0.15-second freeze; it does not
validate official arrival or the 600-second total/240-second preparation
timing profile.

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
