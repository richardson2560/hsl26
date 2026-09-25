# HSL26 - Phase 3 execution plan

**Revision:** 1.1 · **Baseline:** Phase-3 document 1.0 and architecture/technical specification 2.2
**Mode:** SIL/kinematic first; no hardware is available  
**Entry decision:** `P3.0 READY_FOR_SIL_IMPLEMENTATION_WITH_HARDWARE_BLOCKERS`
**Closure decision:** `P3_CLOSED_SIL_WITH_G2_PHYSICAL_BLOCKERS`

## 1. Purpose and limits

This plan turns `HSL26_PHASE3_WORLD_AND_NAVIGATION.md` into an executable
work sequence without claiming physical acceptance. P2 provides the bounded
occupancy/coverage contract and deterministic deskew fixtures. The new
kinematic environment exposes only observations and admitted actuation; referee
truth stays outside policy, planner and controller interfaces.

No semantic start/goal zone is inferred from graph shape. No unknown cell is
free. Nonzero hardware motion remains disabled until the exact mode has G0
physical stop evidence and G1 restricted-envelope evidence.

## 2. Environment created

| Asset | Purpose | Status |
|---|---|---|
| `hsl_core/hsl_core/topology.py` | Versioned multigraph/node/edge/spectral contracts | Prepared |
| `hsl_core/hsl_core/occupancy.py` | Layered atomic occupancy mapper with bounded evidence | Implemented in SIL |
| `hsl_core/hsl_core/topology.py` | Deterministic grid-to-graph extraction and clearance masks | Implemented in SIL |
| `compute_spectrum` in `topology.py` | Optional normalized-Laplacian descriptor | Implemented in SIL, disabled for tactics |
| `sim/kinematic/README.md` | Profile and truth-boundary rules | Prepared |
| `sim/kinematic/scenarios/parallel_corridors.json` | Seeded maze fixture and negative cases | Prepared |
| `artifacts/reports/phase3/P3_environment_baseline.json` | Machine-readable baseline and blockers | Prepared |
| `sim/kinematic/{common,plant,raycaster,sensors,referee,scenario,trace}.py` | Seed-separated deterministic SIL testbed and truth boundary | Implemented in SIL |
| `sim/kinematic/test_p34.py` | P3.4 ray, blind-zone, plant, replay and isolation tests | 12 passed |
| `sim/kinematic/visualizer.py` | Optional headless Top-View and Doom 2.5D diagnostics | Implemented, 5 tests |
| `hsl_core/hsl_core/planning/{astar,execution}.py` | Versioned A*/Dijkstra paths, safe smoothing and leases | Implemented in SIL |
| `hsl_core/hsl_core/control/{regulated_pursuit,execution}.py` | Bounded pursuit candidates and option lease boundary | Implemented in SIL |
| `hsl_core/tests/test_p35_planning_control.py` | P3.5 planner/control/cancellation adversarial tests | 8 passed |

The fixture includes diagonal corner contact, a pure cycle, parallel corridors
and a transient opponent block. Its coordinates are development data only.

## 3. Ordered work packages

1. **P3.1 map layers and versions:** implemented in SIL in
   `hsl_core/hsl_core/occupancy.py`; maintain structural, collision, semantic
   and observed-free layers; preserve unknown; cap bounded evidence, decay only
   transient semantic evidence and commit atomic `map_version`. Physical
   sensor/replay calibration remains pending.
2. **P3.2 embedded graph:** implemented in SIL with footprint inflation,
   unknown-space exclusion, explicit diagonal corner-cut rejection,
   frontier/dead-end distinction, deterministic cycle anchors, metric edge
   polylines, canonical IDs and versioned graph snapshots. Canonical endpoint
   ordering reverses a traced polyline when required. Portal minima refinement,
   swept turnability and physical/replay validation remain pending. Physical
   Livox mounting orientation and support-bar blind sectors are not promoted
   from photographs to calibration; they remain P2 hardware evidence.
3. **P3.3 spectrum (optional):** implemented in SIL for explicitly bounded
   global or induced k-hop subgraphs, with symmetric normalized Laplacian,
   isolated-node convention, parallel-edge affinity summation and complete
   scope metadata. Keep it disabled until direct graph features beat the
   ablation baseline; it is never a safety authority.
4. **P3.4 simulator boundary:** implemented in SIL with seed-separated
   plant/raycaster/sensors/referee, strict scenario validation, observation-only
   sensor contracts and deterministic trace hashing. MVSim remains blocked until
   its sensor model is inspected and replay evidence exists.
5. **P3.5 planning/control:** implemented in SIL: metric A* is checked against
   Dijkstra, blocked overlays preserve structural IDs, smoothing requires an
   external swept-footprint validator, regulated pursuit is bounded by speed,
   yaw and lateral acceleration, and candidates cross a versioned
   cancellation/lease boundary before the existing supervisor/mux chain.

## 4. Verification sequence

Run from the repository root:

```powershell
Set-Location hsl_core
python -m pytest tests -q
python -m compileall -q hsl_core
Set-Location ..
python -m pytest hsl_core\tests -q
```

After each package, add a report under `artifacts/reports/phase3/` recording
status (`PASS`, `FAIL`, `BLOCKED` or `NOT_RUN`), exact command, seed, versions,
hashes, expected/observed results and limitations. Do not convert SIL PASS to
physical PASS.

## 5. G2 acceptance checklist

- [ ] Diagonal blocked corners never connect.
- [ ] Pure cycles receive a deterministic anchor.
- [ ] Parallel corridors retain distinct edge IDs and polylines.
- [ ] A transient opponent blocks traversal without deleting structural identity.
- [ ] A cancelled old plan cannot produce an admitted candidate.
- [ ] A* cost matches Dijkstra for the same graph/cost profile.
- [ ] Every commanded swept/stopping tube is valid in observed coverage.
- [ ] Map changes trigger bounded route revalidation; localization epoch changes
      invalidate map-dependent products.
- [ ] Truth-isolation and seed-replay evidence is complete.
- [ ] Hardware rows remain `BLOCKED_FOR_HARDWARE` until P2 G0/G1 closure.

The checklist above is the G2 acceptance record, not a claim that G2 passed.
The first six SIL/contract rows are evidenced by the phase reports; swept
stopping tubes, route execution through the supervisor, and physical rows
remain open or blocked as recorded in the closure report.

## 6. Dependencies

No new library is needed for the SIL environment beyond the existing
`hsl_core/pyproject.toml` baseline: Python 3.10+, NumPy, SciPy and pytest.
Open3D remains optional for later point-cloud tooling and must not enter the
safety path. PyYAML is unnecessary while profiles are JSON or ROS-managed YAML;
ROS 2, colcon and MVSim belong to their container/runtime profiles.

## 7. Closure and handoff to Phase 4

P3 is closed for the declared SIL/kinematic profile. The immutable handoff
contract is documented in
`artifacts/reports/phase3/P3_CLOSURE_AND_PHASE4_HANDOFF.json`. Phase 4 may use
the versioned `TopologyGraph`, `SensorObservation`, map/topology versions,
localization epochs, deterministic scenario seed and truth-isolated replay
boundary. It must not treat synthetic detections, the optional spectrum,
visualizer output or referee truth as physical sensing evidence.

Phase 4 entry prerequisites are the contracts in
`docs/HSL26_PHASE4_OPPONENT_PERCEPTION.md`, especially explicit labeling of
`SYNTHETIC_DETECTION`, verified model/base geometry and independent safety
returns. G3 and physical G2 remain separate gates.
