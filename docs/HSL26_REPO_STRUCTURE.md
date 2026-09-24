# HSL26 — Repository Structure and Implementation Boundaries

**Revision:** 2.2 · **Date:** 2026-09-24  
**Normative companions:** `HSL26_FINAL_ARCHITECTURE.md` revision 2.2 and `HSL26_TECHNICAL_SPECIFICATION.md` revision 2.2. Work breakdown: `HSL26_IMPLEMENTATION_ROADMAP.md` and phase documents.
**Status:** target repository structure and migration contract; this document does not assert that target modules are already implemented.

## 1. Purpose and evidence boundary

This document translates the revision-2 architecture into concrete repository ownership. It supersedes the earlier repository report wherever that report conflicts with the new authority, temporal-validity, safety, simulation or interface contracts. The supplied session changelog remains the authoritative record of what was reportedly changed in the actual checkout. It states that package scaffolding and static checks exist, while runtime perception, planning, safety, match management, simulation wiring and integration testing were not yet complete.

The repository itself was not attached in this documentation pass. Therefore:

- paths marked **existing/reported** come from the supplied changelog;
- paths marked **required** are revision-2 implementation targets;
- no package, node or test is considered operational merely because it appears in this tree;
- migration must begin by comparing the live checkout with this document and preserving unrelated work.

The official rulebook remains above all software documents. Competition-specific coordinates, maps, score values and metadata require documented provenance and organizer permission regardless of whether they appear in source code, YAML or generated artifacts.

## 2. Dependency and authority rules

The repository uses four strict dependency tiers:

1. `hsl_core`: pure Python domain mathematics and algorithms; no ROS, simulator API or physical I/O.
2. `hsl_interfaces`: ROS 2 IDL only; no algorithms.
3. `ros_ws/src/hsl_*`: thin runtime adapters and state owners that call `hsl_core`.
4. `kobuki`, simulator and hardware adapters: external plant/driver boundaries.

Dependencies point inward. `hsl_core` never imports `rclpy`, generated messages, launch files or `sim`. Decision, navigation and safety never import simulator truth. The learning package is not imported by competition-critical nodes. ROS conversion functions live at adapter boundaries, not as `to_ros_msg()` methods on core dataclasses.

The physical command chain is unique:

```text
hsl_navigation -> MotionCandidate -> hsl_safety supervisor
               -> /hsl/cmd_vel_final -> cmd_vel_mux
               -> /commands/velocity -> Kobuki driver
```

The independent watchdog writes zero commands only to `/hsl/cmd_vel_stop`, a dedicated highest-priority mux input. Test teleoperation uses `/teleop/cmd_vel`. Only `cmd_vel_mux` publishes `/commands/velocity`. No watchdog, planner, simulator or diagnostic node may publish the driver topic directly.

## 3. Target tree

```text
hsl26/
├── README.md
├── LICENSE
├── pyproject.toml                         # optional workspace tooling only
├── .gitignore
├── .gitlab-ci.yml
├── .github/
│   ├── workflows/ci.yml
│   └── CODEOWNERS
├── docker/
│   ├── Dockerfile                         # reported, based on pinned HSL25 image
│   ├── entrypoint.bash
│   ├── build.bash
│   ├── run.bash
│   ├── run_sim.bash
│   ├── into.bash
│   └── stop.bash
├── docs/
│   ├── HSL26_FINAL_ARCHITECTURE.md
│   ├── HSL26_TECHNICAL_SPECIFICATION.md
│   ├── HSL26_REPO_STRUCTURE.md
│   ├── HSL26_IMPLEMENTATION_ROADMAP.md
│   ├── HSL26_PHASE1_IMPLEMENTATION_PLAN.md
│   ├── HSL26_PHASE2_SENSING_AND_CALIBRATION.md
│   ├── HSL26_PHASE3_WORLD_AND_NAVIGATION.md
│   ├── HSL26_PHASE4_OPPONENT_PERCEPTION.md
│   ├── HSL26_PHASE5_MATCH_AND_TACTICS.md
│   ├── HSL26_PHASE6_LEARNING_AND_RELEASE.md
│   ├── HSL26_SESSION_CHANGELOG.md
│   ├── runbook_competition.md
│   ├── rulebook/
│   │   └── Регламент_HSL26_-_v06092026.pdf
│   └── adr/
│       ├── ADR-001-command-authority.md
│       ├── ADR-002-time-and-epochs.md
│       ├── ADR-003-interface-revision-2.md
│       └── ADR-004-goal-provenance.md
├── kobuki/                                 # inherited and commit-pinned hardware layer
│   ├── docker/
│   ├── scripts/
│   └── workspace/src/
│       ├── cmd_vel_mux/
│       ├── kobuki_core/
│       ├── kobuki_ros/
│       ├── kobuki_ros_interfaces/
│       └── livox_ros_driver2/
├── hsl_core/
│   ├── pyproject.toml
│   ├── hsl_core/
│   │   ├── __init__.py
│   │   ├── types.py
│   │   ├── contracts.py
│   │   ├── geometry.py
│   │   ├── kinematics.py
│   │   ├── rules.py
│   │   ├── mapping.py
│   │   ├── topology.py
│   │   ├── perception/
│   │   │   ├── __init__.py
│   │   │   ├── implicit_surface.py
│   │   │   ├── registration.py
│   │   │   ├── segmenter.py
│   │   │   ├── ekf_opponent.py
│   │   │   └── topological_belief.py
│   │   ├── planning/
│   │   │   ├── __init__.py
│   │   │   ├── astar.py
│   │   │   └── intercept.py
│   │   ├── tactics/
│   │   │   ├── __init__.py
│   │   │   ├── options.py
│   │   │   ├── fsm.py
│   │   │   └── utility.py
│   │   ├── control/
│   │   │   ├── __init__.py
│   │   │   ├── braking.py
│   │   │   ├── safety.py
│   │   │   ├── regulated_pursuit.py
│   │   │   └── dwa_local.py
│   │   └── learning/
│   │       ├── __init__.py
│   │       ├── dirichlet.py
│   │       ├── option_value.py
│   │       └── evolution.py
│   └── tests/
│       ├── test_contracts.py
│       ├── test_geometry.py
│       ├── test_kinematics.py
│       ├── test_rules.py
│       ├── test_braking.py
│       ├── test_safety.py
│       ├── test_mapping.py
│       ├── test_topology.py
│       ├── test_spectrum.py
│       ├── test_registration.py
│       ├── test_ekf_opponent.py
│       ├── test_topological_belief.py
│       ├── test_astar.py
│       ├── test_intercept.py
│       ├── test_options.py
│       └── test_learning.py
├── ros_ws/
│   └── src/
│       ├── hsl_interfaces/
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   ├── msg/
│       │   │   ├── ContractHeader.msg
│       │   │   ├── Polygon2.msg
│       │   │   ├── EgoState.msg
│       │   │   ├── OpponentTrack.msg
│       │   │   ├── BeliefCell.msg
│       │   │   ├── OpponentBelief.msg
│       │   │   ├── Obstacle2.msg
│       │   │   ├── CoverageGrid.msg
│       │   │   ├── LocalObstacleSnapshot.msg
│       │   │   ├── TopologyNode.msg
│       │   │   ├── TopologyEdge.msg
│       │   │   ├── GoalZone.msg
│       │   │   ├── WorldSnapshot.msg
│       │   │   ├── SpectralSignature.msg
│       │   │   ├── MatchState.msg
│       │   │   ├── OptionGoal.msg
│       │   │   ├── OptionFeedback.msg
│       │   │   ├── OptionResult.msg
│       │   │   ├── ExecutionState.msg
│       │   │   ├── PathPlan.msg
│       │   │   ├── MotionCandidate.msg
│       │   │   ├── SafetyStatus.msg
│       │   │   ├── SupervisorHeartbeat.msg
│       │   │   ├── WatchdogHealth.msg
│       │   │   └── RuleEvent.msg
│       │   ├── action/
│       │   │   └── ExecuteOption.action
│       │   └── srv/
│       │       ├── ResetStage.srv
│       │       ├── SetGoalZone.srv
│       │       ├── StartStage.srv
│       │       ├── ResolveEvent.srv
│       │       └── RearmSafety.srv
│       ├── hsl_perception/
│       │   ├── hsl_perception/
│       │   │   ├── adapters.py
│       │   │   ├── ego_state_node.py
│       │   │   ├── local_obstacles_node.py
│       │   │   └── opponent_tracker_node.py
│       │   └── launch/perception.launch.py
│       ├── hsl_world/
│       │   ├── hsl_world/
│       │   │   ├── adapters.py
│       │   │   ├── odom_fusion_node.py
│       │   │   ├── map_server_node.py
│       │   │   └── topology_node.py
│       │   ├── config/robot_localization.yaml
│       │   └── launch/world.launch.py
│       ├── hsl_decision/
│       │   ├── hsl_decision/
│       │   │   ├── adapters.py
│       │   │   └── tactics_node.py
│       │   └── launch/decision.launch.py
│       ├── hsl_navigation/
│       │   ├── hsl_navigation/
│       │   │   ├── adapters.py
│       │   │   ├── option_executor_node.py
│       │   │   ├── global_planner_node.py
│       │   │   └── local_control_node.py
│       │   └── launch/navigation.launch.py
│       ├── hsl_safety/
│       │   ├── hsl_safety/
│       │   │   ├── adapters.py
│       │   │   ├── supervisor_node.py
│       │   │   └── watchdog.py
│       │   └── launch/safety.launch.py
│       ├── hsl_match/
│       │   ├── hsl_match/
│       │   │   ├── adapters.py
│       │   │   └── stage_manager_node.py
│       │   └── launch/match.launch.py
│       ├── hsl_bringup/
│       │   ├── hsl_bringup/__init__.py
│       │   ├── launch/
│       │   │   ├── hsl26.launch.py
│       │   │   ├── real_hardware.launch.py
│       │   │   ├── sim_mvsim.launch.py
│       │   │   ├── sim_kinematic.launch.py
│       │   │   └── replay_bag.launch.py
│       │   └── config/
│       │       ├── hardware.yaml
│       │       ├── frames.yaml
│       │       ├── timing.yaml
│       │       ├── perception.yaml
│       │       ├── topology.yaml
│       │       ├── tactics.yaml
│       │       ├── score_profile.yaml
│       │       ├── memory_profiles.yaml
│       │       └── cmd_vel_mux.yaml
│       └── hsl_diagnostics/
│           ├── hsl_diagnostics/terminal_hud_node.py
│           └── launch/diagnostics.launch.py
├── sim/
│   ├── common/
│   │   ├── ports.py
│   │   ├── scenario.py
│   │   └── referee.py
│   ├── kinematic/
│   │   ├── engine.py
│   │   ├── raycaster.py
│   │   ├── sensors.py
│   │   ├── adapters.py
│   │   └── scenarios/
│   └── mvsim/
│       ├── worlds/
│       ├── vehicles/
│       ├── sensors/
│       └── adapter/simulation_adapter.py
├── config/
│   └── schema/
│       ├── hardware.schema.json
│       ├── timing.schema.json
│       ├── perception.schema.json
│       ├── tactics.schema.json
│       ├── policy_manifest.schema.json
│       └── model_manifest.schema.json
├── tools/
│   ├── validate_config.py
│   ├── preflight_check.py
│   ├── benchmark_latency.py
│   ├── calibrate_braking.py
│   ├── train_gpis_prior.py
│   ├── validate_gpis_prior.py
│   ├── bag_replay_eval.py
│   ├── benchmark_policy.py
│   ├── check_no_sim_imports.py
│   ├── check_ros_graph_authority.py
│   └── release_freeze.py
├── tests/
│   ├── contract/
│   ├── integration/
│   ├── launch/
│   ├── simulation/
│   └── hardware/                              # opt-in only; never ordinary CI
└── artifacts/
    ├── models/
    │   └── opponent_gpis/
    │       ├── model.npz
    │       ├── manifest.json
    │       └── validation.json
    ├── policies/
    ├── calibration/
    ├── releases/
    └── reports/
```

## 4. Core package boundaries

### 4.1 Foundational modules

`types.py` contains immutable dataclasses and enums mirroring revision-2 concepts. It contains no ROS conversion logic. `contracts.py` validates schema metadata, temporal validity, epochs, finite values and covariance structure. `geometry.py` owns robust segment/polygon operations and swept-footprint representations. `kinematics.py` owns exact constant-twist integration, wheel conversions and point-cloud deskew. `rules.py` owns capture and arrival predicates and reports intermediate metrics.

These modules form the first implementation slice because all later packages depend on them. They are also the only modules needed for most Phase 1 mathematical tests.

### 4.2 Safety modules

`control/braking.py` implements explicitly named braking models. The simple analytic model and the delay-acceleration model must not share an ambiguous function. `control/safety.py` contains the pure, bounded supervisor evaluator: it accepts immutable records and limits, returns an admitted command/status/heartbeat, and performs no ROS publication.

The ROS supervisor wraps this evaluator in its own operating-system process. The watchdog is another process, not another timer inside the supervisor. It observes completed-decision heartbeats and stage/configuration leases and writes only the highest-priority zero channel. Mutual health is required before motion is released.

### 4.3 Mapping, perception and planning

Mapping and topology keep structural evidence separate from collision occupancy and observed-free coverage. Opponent recognition may suppress structural points only within its candidate-identification copy of the cloud. The safety path always retains static walls, unidentified returns and unknown space.

The GPIS prior is a versioned offline artifact. `implicit_surface.py` loads and evaluates it, while `registration.py` performs bounded online optimization. `ekf_opponent.py` owns a four-state constant-velocity filter. `topological_belief.py` owns multimodal edge-interval belief and unknown mass. No module uses a single point estimate after long occlusion as if it were observed truth.

Planning uses graph edge polylines, not only endpoint chords. A* accepts nonnegative costs with explicit units. Interception helpers return candidate opportunities with assumptions, not capture guarantees. Tactical options are defined once in `options.py`; their initiation, invariant and termination predicates are consumed by both the selector and executor.

## 5. Interface migration rules

The reported bootstrap interfaces are revision 1 and must not be extended field-by-field while old producers continue running. Revision 2 is an atomic compatibility boundary:

- add `ContractHeader.msg` and use it in every domain message;
- migrate `EgoState`, `OpponentTrack`, `MotionCandidate`, `SafetyStatus`, `MatchState`, `WorldSnapshot` and option messages together;
- add local obstacle/coverage, opponent belief, execution authority, supervisor heartbeat, watchdog health, rule event and typed goal-zone records;
- replace untyped target overrides with `SetGoalZone.srv`;
- use `ExecuteOption.action` as the only tactical execution command;
- reject unknown schema versions and mixed producer/consumer revisions at startup.

During migration, a temporary `hsl_interfaces_v1_bridge` may be used only in a development branch if required. It must be removed or disabled in the release profile. Do not encode missing revision-2 provenance using empty strings while still granting motion authority.

## 6. ROS runtime ownership

| Runtime state | Sole owner | Other nodes receive |
|---|---|---|
| `odom→base_link` | fusion or selected driver/fusion configuration | TF and `EgoState`; no duplicate broadcaster |
| `map→odom` | localization owner | TF plus localization epoch |
| structural map and versions | map server | immutable snapshots/version events |
| topological graph | topology node | `WorldSnapshot` |
| opponent filter and belief | opponent tracker process | `OpponentTrack`, `OpponentBelief` |
| active option instance | option executor | `ExecutionState`, action feedback/result |
| final autonomous command | supervisor | `/hsl/cmd_vel_final` and status |
| independent stop latch | watchdog | `/hsl/cmd_vel_stop`, health |
| stage identity/phase/zones | stage manager | leased `MatchState`, `RuleEvent` |
| physical driver command | mux | `/commands/velocity` |

Each node validates inputs before replacing its cache. Timers operate on coherent immutable snapshots. Long-running mapping, registration or planning work carries input versions and is discarded when stale. Safety never blocks on these workers.

The baseline co-locates the logical `GlobalPlannerNode` with the option executor and uses a typed internal request/result API. A deployment in separate processes requires a separately specified, versioned ROS transport contract before use; no implicit planner service is part of revision 2.

## 7. Configuration layout

`hardware.yaml` contains only controller assumptions and authorized raw/logical interfaces tied to calibration identifiers. It must distinguish the reported raw topics `/livox/lidar`, `/livox/imu`, `/odom` from logical topics. It also defines mux inputs and the physical output `/commands/velocity`. IP addresses from the supplied MID-360 configuration remain deployment data, not general mathematical constants.

`timing.yaml` contains periods, leases, expected jitter, compute budgets and driver/mux timeouts. `frames.yaml` contains frame IDs, TF ownership and extrinsic calibration IDs. `perception.yaml`, `topology.yaml` and `tactics.yaml` contain algorithm profiles with schema validation. `score_profile.yaml` distinguishes official values from surrogate training rewards. `memory_profiles.yaml` lists only organizer-approved retention behavior.

Configuration validation is performed before arming. Missing braking, latency, rear-coverage or angular-response evidence disables the associated mode. No Python default silently supplies a competition safety bound.

## 8. Simulation organization

Simulation capabilities are separated so the policy cannot access referee truth. `sim/common/ports.py` defines observation, actuation, simulation-control and referee-truth interfaces. Policy runners receive only observation and admitted-actuation ports. The referee receives truth but never exposes it on policy namespaces.

The NumPy simulator supports `observed_map`, `approved_map` and `oracle` profiles. Only the first two are competition-representative, and each clearly declares what structural prior is available. GPIS is bypassed only at the synthetic-detection boundary; filtering, belief, tactics, rules and safety remain shared. MVSim must use a version-pinned, inspected sensor model; a generic point cloud is not automatically a MID-360 validation.

Replay has no physical actuator devices and cannot be used as a closed-loop counterfactual after commands change. It is for deterministic estimator/contract regression. Runtime graph inspection complements static no-simulator-import checks.

## 9. Artifacts and provenance

Generated files are grouped by meaning:

- `artifacts/models`: non-executable perception model arrays, manifest and held-out validation;
- `artifacts/policies`: baseline/learned parameters, feature/option schema hashes and validation;
- `artifacts/calibration`: measured braking, latency, footprint, extrinsic and blind-zone evidence;
- `artifacts/releases`: immutable release manifest with source, image, configuration, model and policy hashes;
- `artifacts/reports`: benchmark and acceptance reports.

Never store safety calibration under a learning-policy directory. Never load Python pickle or another executable artifact for a competition model. Machine-local exploratory measurements may remain uncommitted; promoted release evidence is versioned deliberately and contains operating-condition scope.

## 10. Testing and CI

CI is ordered from cheapest to most integrated:

| Stage | Content | Required environment |
|---|---|---|
| `lint` | format/static typing, forbidden imports, generated schema drift | Python |
| `unit-core` | pure mathematical and contract tests | Python, no ROS |
| `interface-build` | ROS IDL generation and adapter round trips | ROS 2 Humble |
| `container-build` | pinned Docker build and dependency manifest | Docker runner |
| `launch-smoke` | startup, topic/TF ownership, zero-at-start | Built container |
| `integration-sim` | stage/action/safety/mux behavior, namespace/truth isolation | Container plus simulator |
| `fault-injection` | stale inputs, process deaths, clock/epoch jumps | Container; no physical robot |
| `hardware-opt-in` | braking, blind zone, driver/mux timeout and minimum objects | Supervised final hardware only |
| `release` | hashes, offline cold start and two-stage rehearsal | Final target environment |

`pytest` passing is not a physical certificate. The release gate attaches the actual commands, versions and reports. An observed worst case is not called WCET. Hardware tests are never run automatically on a generic CI runner.

## 11. Docker and execution profiles

The reported corrected Dockerfile uses `nickodema/kobuki:humble-22.04-100625` as a parameterizable inherited hardware base. Before release, verify that the tag remains available, record its immutable digest, rebuild the actual inherited sources and run the complete ROS test suite. A tag name alone is not a reproducible dependency.

The entrypoint sources, in order:

```text
/opt/ros/humble/setup.bash
/workspace_kobuki/install/setup.bash
/workspace_hsl26/install/setup.bash
```

`mode:=real`, `mode:=mvsim`, `mode:=kinematic` and `mode:=replay` choose adapters, not algorithms. The real profile alone receives network/USB/serial access. Sim/replay profiles do not mount or address the physical driver. Release images contain frozen code and artifacts rather than editable bind mounts.

## 12. Branch and change discipline

Every change affecting interfaces, authority, frames, time, geometry or physical limits requires:

1. an architecture/specification reference;
2. an updated unit/contract test;
3. an adapter migration across every producer and consumer;
4. a launch/integration check when runtime ownership changes;
5. an acceptance-report update when a measured envelope changes.

Feature branches should correspond to one vertical capability, such as `feature/phase1-contracts` or `feature/phase1-stop-path`, not one isolated file that cannot run. Do not create independent GitLab/GitHub histories without an explicit project decision. The inherited driver layer is pinned and changed only by a reviewed compatibility patch.

## 13. Definition of repository readiness

The tree is **documentation-ready** when architecture, blueprint and repository contracts agree and every required path has an owner; the roadmap and phase documents must trace to those contracts. It is **build-ready** when the live checkout generates revision-2 interfaces and the container/colcon build passes. It is **Phase-1-ready** when foundational core tests, supervisor, independent watchdog, mux authority and software fault-injection tests pass with zero-only startup. It is **autonomy-ready** only after mapping/perception/navigation/stage gates and physical calibration evidence pass.

The current supplied evidence supports “build-oriented scaffold with reported corrections,” not autonomous operation. `HSL26_IMPLEMENTATION_ROADMAP.md` indexes P1–P6 work and evidence gates; the phase files specify actionable tasks without treating a documented target as an implemented module.
