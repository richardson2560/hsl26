# HSL26 — Phase 2: Sensing, State and Physical Envelope

**Revision:** 1.0 · **Baseline:** architecture/blueprint revision 2.2; roadmap revision 1.0.  
**Objective:** replace Phase-1 mocks with authorized sensors and establish the measured motion envelope. **Exit:** G0 physical response and G1 evidence for each enabled mode. Paths are targets pending live-checkout audit.

## 1. Entry and authority

Requires Phase-1 G0 software PASS. Real movement remains disabled until stop channel, mux output and driver timeout are verified under the permitted supervised setup. Confirm actual supplied hardware inventory and ROS driver types; MID-360 is the main source, and an additional sensor is not assumed. Keep `real`, `replay` and `kinematic` profiles separate; sensor tests cannot silently use simulator truth.

## 2. Work packages

| ID / owner role | Subtasks and target files | Observable deliverable | Verification |
|---|---|---|---|
| P2.1 Hardware integration | **a** inspect actual LiDAR `PointCloud2` or `CustomMsg` and offset-time origin; **b** map `/livox/lidar`, `/livox/imu`, `/odom` once; **c** validate mux/driver total-silence timeout. `hsl_perception/adapters.py`, `hsl_bringup/config/hardware.yaml`, inherited compatibility patch only if needed. | Hardware inventory, topic/type/QoS/timeout manifest, G0 physical fault trace. | I02/I03/I05; sole `/commands/velocity` writer. |
| P2.2 Ego and frames | **a** select one odom fusion and one map localization owner; **b** calibrate `T_B_L` and frame origins; **c** interpolate pose history/deskew per point; **d** publish coherent `/state/ego_local` and `/state/ego`. `odom_fusion_node.py`, `ego_state_node.py`, `frames.yaml`. | TF graph and clock/epoch record, deskewed scan replay with residuals. | T06/T07/T20/T22, I05. |
| P2.3 Fast obstacle and coverage path | **a** sensor validity and ground/self masks; **b** ray update to first hit; **c** preserve unknown/blind cells and all physical returns; **d** publish bounded `LocalObstacleSnapshot` independently of GPIS. `mapping.py`, `local_obstacles_node.py`, `perception.yaml`. | Recorded obstacle/coverage snapshots and inspection plots. | I06/I07/I08; T24; no free clearing behind hit. |
| P2.4 Calibration and profile | **a** measure footprint including rotating protrusions; **b** run permitted braking/driver response trials across floor, load, battery, speed and turns; **c** measure minimum object, blind and rear coverage; **d** build `LimitsProfile`/`TimingProfile` bounds. `calibrate_braking.py`, `benchmark_latency.py`, `validate_config.py`, `hardware.yaml`/`timing.yaml`. | Raw traces, `braking_report.json`, blind-zone report and approved restricted operating range. | T12/T13, I03/I06–I08; §17 timing inequalities. |
| P2.5 Adapter and ROS fault exercise | **a** exact `EgoState`/obstacle encode/decode; **b** stale sample republishing; **c** dropout, jump and overload; **d** restricted-speed supervised no-target trials. | ROS launch/contract log and explicit mode matrix. | T20/T22/T23/T30, I05/I08. |

No numeric `b_min`, `tau_response`, footprint or blind-sector default from rev1 counts as calibration. A scalar closest-obstacle reading cannot satisfy the coverage contract. Reverse and independent rotation remain disabled until their distinct envelopes are measured.

## 3. Milestones and dependencies

| Milestone | Prerequisite | Gate check |
|---|---|---|
| M2.1 Proven physical stop | P2.1 and authorized controlled test | Stop priority and total command loss yield recorded request delay and measured stop; unresolved real profile stays disarmed. |
| M2.2 Coherent observation | P2.2/P2.3 | Every sample has frame, observation time, epoch and coverage; low obstacles included. |
| M2.3 Restricted operating envelope | M2.1/M2.2 and P2.4/P2.5 | G0 physical and G1 pass only for the enabled speed/direction/turn range. |

These milestones can be prepared in parallel without granting early physical motion.

## 4. Gate G1 acceptance record

Record PASS/FAIL/BLOCKED/NOT_RUN per I03, I05–I08, T07, T12/T13/T20/T22/T23. Attach exact commands, raw sensor bags, physical stop traces, allowed operating conditions, repeat count, conservative uncertainty bounds and hashes. Measure request-to-zero separately from time/distance until physical rest. G1 failure disables or reduces the affected mode; it cannot be repaired by reducing uncertainty numbers on paper.

**Required negative cases:** missing per-point time rejects unsupported deskew; an invalid/no-return ray never clears blind space; a stationary opponent and a wall both remain collision occupied; a low 0.1×0.1×0.15 m obstacle is detected at the enabled speed or that speed is disabled; rear/rotation commands are rejected when coverage or bounds are unavailable. For kill tests, log the last valid command, fault time, first zero request and measured stop event separately.

## 5. Handoff to Phase 3

Provide immutable ego/local obstacle/coverage contracts, validated ROS QoS/TF ownership, calibrated profile IDs and restrictions. Mapping and route following consume these through the blueprint interfaces; they do not bypass supervisor admission.
