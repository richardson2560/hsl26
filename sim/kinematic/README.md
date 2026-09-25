# HSL26 Phase-3 kinematic testbed

This is the no-hardware baseline for Phase 3. It is deliberately separate from
ROS and from the policy: the plant and sensor adapter expose observations and
accepted actuation only, while a future referee adapter owns ground truth.

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
be replayed byte-for-byte. No physical authority is granted by this testbed.
