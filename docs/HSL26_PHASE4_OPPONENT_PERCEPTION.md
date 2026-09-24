# HSL26 — Phase 4: Opponent Model, Tracking and Belief

**Revision:** 1.0 · **Baseline:** architecture/blueprint revision 2.2; roadmap revision 1.0.  
**Objective:** estimate the opponent robot's base-frame origin and a calibrated, finite-speed belief through occlusion. **Exit:** G3 for declared sensing/visibility conditions. Safety continues to use physical returns independently.

## 1. Entry and profile separation

Requires P2 measured sensor/ego contracts and P3 graph/version semantics. Synthetic detections may test tracking, belief and tactics while GPIS model construction is ongoing; such evidence is labeled `SYNTHETIC_DETECTION` and cannot satisfy real-cloud GPIS accuracy. Obtain verified complete robot geometry or registered scans and calibration `T_B_M`; a partial base mesh or raw cluster centroid is insufficient to claim robot origin.

## 2. Work packages

| ID / owner role | Subtasks and target files | Observable deliverable | Verification |
|---|---|---|---|
| P4.1 Source and Hermite GP | **a** verify units/coverage/licensing, model-to-base transform and disjoint train/validation views; **b** orient normals and sample surface/derivative/offset constraints; **c** fit regularized compact kernel; **d** save arrays, schema and hashes. `train_gpis_prior.py`, `implicit_surface.py`. | `model.npz`, `manifest.json`, training diagnostics. | T25/T26, §8 derivative block/numeric condition checks. |
| P4.2 Registration and detection | **a** segment candidates without removing safety collision evidence; **b** optimize bounded origin/yaw fit with support mask; **c** estimate covariance and yaw observability; **d** reject weak/unsupported/timeout fits. `segmenter.py`, `registration.py`, `validate_gpis_prior.py`. | Per-view held-out origin error, false match and yaw-valid tables. | T09/T25/T26, I10. |
| P4.3 Track association | **a** initialize on repeated admissible evidence; **b** gate subsequent four-state CV measurements; **c** Joseph covariance update; **d** maintain distinct last-measurement/prediction/publication stamps; **e** state transitions SEARCHING/TRACKED/COASTING/OCCLUDED_BELIEF/LOST. `ekf_opponent.py`, `opponent_tracker_node.py`. | Sequence logs with accepted/rejected associations and covariance. | T08/T20, I10. |
| P4.4 Reachability and negative observation | **a** initialize edge intervals plus unknown mass; **b** propagate with bounded speed and topology version; **c** update only regions covered by fresh valid scans; **d** reacquire and normalize; **e** test long edges/junction branches. `topological_belief.py`, `sim/kinematic/sensors.py`. | Belief trace over stationary, occluded and reappearing rival. | T18/T19/T29; I10. |
| P4.5 Runtime and fidelity integration | **a** encode/decode track and belief; **b** bounded worker timeout and stale-result discard; **c** replay real bags and declare MVSim sensor fidelity; **d** compare real-cloud path to synthetic-measurement bypass. | Hash/version-stamped profile report; topic/QoS/latency traces. | T23, I08/I10/I14. |

## 3. Milestones

| Milestone | Dependency | Evidence |
|---|---|---|
| M4.1 Valid model artifact | P4.1/P4.2 | Held-out origin error and support fraction with failure examples; artifact hash and model frame. |
| M4.2 Calibrated track | M4.1 or explicit synthetic detector, P4.3 | Covariance/gating/yaw-valid behavior versus labeled views. |
| M4.3 Occlusion belief | P3 graph, M4.2 and P4.4 | No impossible far-edge arrival; negative scans only where coverage is real. |

## 4. Gate G3 acceptance record

Run T08/T09/T18–T20/T25/T26 and I10 on held-out stationary, moving, blind-zone, partial-view and reappearance sequences. Report position-origin bias and error by range/view, yaw-valid fraction, confidence calibration, false positives/associations, duration/load and LOST degradation. G3 is profile-specific: synthetic measurements can PASS a filter subgate while full real-cloud recognition remains BLOCKED. No emergency obstacle handling depends on model success.

**Required negative cases:** axisymmetric/insufficient views yield `yaw_valid=false`; a query outside compact kernel support cannot be accepted as a zero-level match; a gated outlier does not refresh `last_measurement_stamp`; long-edge belief cannot reach the far endpoint earlier than the bounded travel time; a negative observation through an occlusion or blind sector does not remove belief mass. Report calibration and rejection rates together with average error.

## 5. Handoff to Phase 5

Provide versioned `OpponentTrack` and `OpponentBelief` with validity and fidelity, calibrated origin/yaw behavior, last observation time, conservative reachability and explicit uncertainty/unknown mass. Tactical consumers cannot treat stale predicted pose as a measured rival position.
