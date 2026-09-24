# HSL26 — Phase 3: World, Topology and Navigation

**Revision:** 1.0 · **Baseline:** architecture/blueprint revision 2.2; roadmap revision 1.0.  
**Objective:** produce a valid versioned world graph and track bounded routes through the existing safety chain. **Exit:** G2 for the accepted observation/motion profile; no opponent-recognition claim.

## 1. Entry and limits

Requires Phase-2 observation/coverage contracts. Work on pure graph and simulator components can proceed before physical G1; any nonzero robot trial requires G0 physical and G1 for its exact mode. Use an approved map profile or observed evidence with explicit provenance. Never treat an unexplored cell as free or assign a semantic start zone from graph shape.

## 2. Work packages

| ID / owner role | Subtasks and target files | Observable deliverable | Verification |
|---|---|---|---|
| P3.1 Occupancy and versions | **a** separate structural, collision, semantic and observed-free layers; **b** cap correlated log-odds/ray evidence; **c** persistence/decay under approved memory profile; **d** atomic map versions. `mapping.py`, `map_server_node.py`, `topology.yaml`. | Replayable grids including unknown, transient opponent and wall cases. | T07/T22/T29; I09. |
| P3.2 Embedded multigraph | **a** known-free inflated configuration space and metric distance transform; **b** skeleton/junction cluster/cycle anchor/frontier/portal extraction; **c** preserve parallel corridors and edge polylines; **d** validate swept footprint, node IDs and topology migration; **e** compute bridge/articulation. `topology.py`, `topology_node.py`. | Typed `WorldSnapshot`, visual graph overlays and topology-version trace. | T10/T17/T29; I09. |
| P3.3 Optional spectral descriptor | **a** choose scope/affinity in profile; **b** construct symmetric normalized Laplacian with isolated-node convention; **c** mark unavailable eigenvalues; **d** compare against direct graph features before enabling tactical use. `topology.compute_spectrum`. | Versioned `SpectralSignature` only on selected bounded subgraphs. | T15–T17. May remain disabled without blocking G2. |
| P3.4 Shared simulator and route testbed | **a** scenario validation; **b** seed-separated kinematic plant/raycaster/sensors; **c** restricted ports and truth-only referee; **d** declare observed-map, approved-map and oracle profiles. `sim/common/*`, `sim/kinematic/*`, `sim/mvsim/adapter/*`. | Deterministic recorded scenario/observation traces and truth-isolation report. | I09/I14; ray first hit, blind zone, no truth import/subscription. |
| P3.5 Planning and control | **a** A* metric cost with admissible unit-matched heuristic; **b** path versions and collision-checked smoothing; **c** regulated pursuit and optional bounded arcs; **d** option executor/planner/local-control typed boundary, cancellation and execution lease; **e** send only leased candidates to supervisor. `planning/*`, `control/{regulated_pursuit,dwa_local}.py`, navigation nodes. | Reproducible path/candidate traces and option-action race logs. | T10/T11/T13/T21/T29, I09/I11. |

The `GlobalPlannerNode` is a logical class. A separate process needs a versioned transport contract for planner requests/results; the baseline co-locates it with `OptionExecutorNode` as a composable component using the typed internal interface in blueprint §16.6. Do not improvise a public ROS service.

## 3. Milestones

| Milestone | Prerequisite | Acceptance |
|---|---|---|
| M3.1 Geometry-valid map | P3.1/P3.2 | No diagonal corner cutting, missing IDs, wall shortcuts or invented dead ends in maze fixtures. |
| M3.2 Simulator truth separation | P3.4 | Policy sees observations/actuation only; referee alone sees ground truth; seed replay reproduces traces. |
| M3.3 Admitted route following | M3.1/M3.2/P3.5 | Complete turns/junction/cycle and inserted-obstacle route with every command passing supervisor. |

## 4. Gate G2 acceptance record

Record T10/T11/T13/T15–T17/T21/T29 and I09/I11/I14 with graph/path IDs, map and topology versions, scenario seed and collision/swept/coverage traces. Compare A* with Dijkstra for the same cost. A change of map version triggers bounded route revalidation; localization-epoch change invalidates map-dependent products. Spectral ablation failure leaves the descriptor disabled. A real route test is restricted to the envelope accepted in G1.

**Required negative cases:** diagonally touching blocked corners never connect; a pure cycle gets a deterministic anchor; parallel corridors retain distinct edge IDs; an edge blocked by the opponent is excluded for both roles without deleting its structural ID; a delayed old plan after cancellation never yields an admitted candidate. A route is PASS only if its full polyline and every commanded swept/stopping tube are valid in the observed coverage at the corresponding version.

## 5. Handoff to Phase 4

Provide sparse embedded graph, structural/collision overlays, validated ego and observation history, accepted simulator profiles and path-independent topology version/migration contract for finite-speed opponent belief.
