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

The testbed must remain deterministic: use the scenario seed, keep truth in the
referee boundary, record map/topology/path versions, and emit traces that can
be replayed byte-for-byte. Sensor randomness has its own seed and is never
shared with plant or referee state. No physical authority is granted by this
testbed. The MVSim adapter remains a separate integration boundary and is not
claimed as validated by the kinematic tests.

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
