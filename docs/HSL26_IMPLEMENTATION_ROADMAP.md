# HSL26 Implementation Roadmap and Engineering Register

## 1. Purpose

This document is the operational register for the HSL26 implementation. It
records what will be built, the order of execution, task ownership boundaries,
dependencies, acceptance gates, and current status.

The register is intentionally more detailed than a checklist. Each task
identifies the code surface it changes, the contract it must preserve, and the
evidence required before the next phase may start.

## 2. Status vocabulary

| Status | Meaning |
|---|---|
| `DONE` | Implemented and validated by the stated evidence |
| `IN PROGRESS` | Work has started but acceptance is incomplete |
| `BLOCKED` | Cannot proceed until a dependency or environment issue is resolved |
| `READY` | Dependencies are satisfied and work may start |
| `NOT STARTED` | Planned but not yet assigned or implemented |
| `DEFERRED` | Intentionally postponed without blocking the current gate |

Current repository bootstrap work is complete enough to begin Phase 1
implementation. Full ROS/Docker acceptance remains environment-dependent.

## 3. Baseline completed before Phase 1

### B0. Repository and build foundation — `DONE`

Completed:

- HSL26 repository layout created on top of inherited Kobuki/Livox sources.
- `hsl_core` setuptools metadata added.
- ROS 2 package metadata added for all HSL26 packages.
- `hsl_interfaces` CMake and package metadata added.
- Multi-stage Dockerfile added.
- ROS environment entrypoint added.
- HSL25 inherited hardware image selected as Docker base.
- Nonessential Kobuki packages excluded with `COLCON_IGNORE`.

Evidence:

- Python syntax and package metadata checks passed.
- XML parsing passed.
- `git diff --check` passed.

### B1. Interface and configuration baseline — `DONE`

Completed:

- Message, service, and action contracts populated.
- `EgoState` planar covariance set to 9 elements.
- `OpponentTrack` temporal and spatial validity fields added.
- Frames, hardware, perception, tactics, and score configuration created.
- Raw driver topics and logical HSL26 topics documented.
- MID-360 network addresses documented from `MID360_tb_config.json`.

### B2. Command authority baseline — `DONE`

Completed:

- `cmd_vel_mux` configured with emergency teleoperation priority 100.
- Autonomous supervisor priority set to 50.
- Physical output set to `/commands/velocity`.
- Inherited mux implementation updated to honor the `output` parameter.

## 4. Phase 1: Mathematical Core and Isolated Safety Supervisor

Phase 1 is the mandatory next delivery. No navigation or tactical feature may
be considered production-ready before this phase passes its acceptance gate.

### P1.1 Domain types and ROS mirrors — `IN PROGRESS`

Files:

- `hsl_core/hsl_core/types.py`
- `ros_ws/src/hsl_interfaces/msg/*.msg`
- `ros_ws/src/hsl_safety/hsl_safety/`

Subtasks:

- [x] Define frozen domain dataclasses.
- [x] Define explicit units in field names or documentation.
- [x] Add conversion functions from ROS messages to domain types.
- [x] Add conversion functions from domain decisions to ROS messages.
- [x] Reject missing frame, epoch, or validity metadata.
- [x] Map opponent lifecycle constants without numeric literals.

Acceptance:

- Domain objects can be constructed without ROS.
- ROS conversion tests preserve all timing and epoch fields.
- Invalid objects fail explicitly.

Implementation evidence:

- `hsl_core/hsl_core/types.py` now provides frozen `Pose2D`, `Twist2D`,
  `EgoState`, `OpponentTrack`, `MotionCandidate`, and `SafetyStatus`
  contracts with finite-value, shape, identity, and temporal validation.
- `ros_ws/src/hsl_safety/hsl_safety/conversions.py` provides the ROS boundary
  conversions without importing ROS from `hsl_core`.
- `MotionCandidate.msg` now carries `map_version` and
  `localization_epoch`, so the ROS contract contains the same spatial validity
  metadata as the domain contract.
- Focused tests were added under `hsl_core/tests/test_types.py` and
  `ros_ws/src/hsl_safety/test/test_conversions.py`.
- Python bytecode compilation and `git diff --check` pass. Pytest execution is
  pending because pytest is not installed in the current host environment.

### P1.2 Differential-drive kinematics — `READY`

Files:

- `hsl_core/hsl_core/kinematics.py`
- `hsl_core/tests/test_kinematics.py`

Subtasks:

- [ ] Implement exact constant-twist integration.
- [ ] Implement stable `omega -> 0` branch.
- [ ] Implement angle wrapping.
- [ ] Define numerical tolerance policy.
- [ ] Add footprint polygon generation.
- [ ] Add temporal deskew interface for moving sensor data.

Dependencies:

- P1.1 domain types.

Acceptance:

- Straight and curved trajectory tests pass.
- Near-zero angular velocity produces finite results.
- Results are deterministic across repeated runs.

### P1.3 Braking and dynamic limits — `READY`

Files:

- `hsl_core/hsl_core/control/braking.py`
- `hsl_core/tests/test_braking.py`
- `ros_ws/src/hsl_safety/hsl_safety/braking_envelope.py`

Subtasks:

- [ ] Implement stopping-distance equation.
- [ ] Implement latency-aware admissible-speed equation.
- [ ] Implement residual distance for dynamic opponent motion.
- [ ] Validate positive finite deceleration.
- [ ] Implement linear and angular acceleration clamping.
- [ ] Add explicit safety-margin handling.

Dependencies:

- P1.1 domain types.
- Hardware values from `hsl_bringup/config/hardware.yaml`.

Acceptance:

- Admissible speed is monotonic with free distance.
- Zero or negative clearance yields zero admissible speed.
- Invalid physical parameters raise explicit errors.
- Unsafe candidates are rejected or reduced deterministically.

### P1.4 Official rule predicates — `READY`

Files:

- `hsl_core/hsl_core/rules.py`
- `hsl_core/tests/test_rules.py`

Subtasks:

- [ ] Implement capture distance predicate.
- [ ] Implement 45-degree bearing predicate.
- [ ] Implement line-of-sight requirement.
- [ ] Implement footprint-to-zone arrival predicate.
- [ ] Implement swept-path contact interface.
- [ ] Implement pre-capture collision certification interface.

Acceptance:

- Boundary matrices cover just-inside and just-outside cases.
- LOS failure always vetoes capture.
- Center-point proximity alone cannot certify arrival.

### P1.5 Isolated supervisor process — `READY`

Files:

- `ros_ws/src/hsl_safety/hsl_safety/supervisor_node.py`
- `ros_ws/src/hsl_safety/launch/safety.launch.py`

Subtasks:

- [ ] Implement `SingleThreadedExecutor`.
- [ ] Implement latest-snapshot storage.
- [ ] Implement five-stage safety pipeline.
- [ ] Subscribe to `MotionCandidate`.
- [ ] Subscribe to `EgoState`.
- [ ] Subscribe to `MatchState`.
- [ ] Subscribe to world/safety obstacle state.
- [ ] Subscribe to opponent track validity where required.
- [ ] Publish `/hsl/cmd_vel_final`.
- [ ] Publish `/hsl/safety_status`.
- [ ] Ensure the process imports no heavy perception/planning libraries.

Acceptance:

- Supervisor runs in its own OS process.
- Freeze and finished states publish zero.
- Missing or stale input publishes zero.
- Valid input is admitted only within all configured limits.

### P1.6 Hardware watchdog — `READY`

Files:

- `ros_ws/src/hsl_safety/hsl_safety/watchdog.py`
- `tests/integration/test_watchdog_rate.py`

Subtasks:

- [ ] Implement a 20-30 Hz timer.
- [ ] Publish zero when no valid command exists.
- [ ] Publish zero after candidate expiry.
- [ ] Detect supervisor callback/process staleness.
- [ ] Avoid callback blocking.
- [ ] Measure actual publication frequency in integration tests.

Acceptance:

- Measured output rate is at least 20 Hz.
- Inactivity produces repeated safe output.
- Killing the upstream planner cannot leave a stale positive command active.

### P1.7 Phase 1 integration path — `READY`

Subtasks:

- [ ] Launch `hsl_safety` independently.
- [ ] Publish a synthetic valid `MotionCandidate`.
- [ ] Verify `/hsl/cmd_vel_final`.
- [ ] Verify `cmd_vel_mux` receives the autonomous input.
- [ ] Verify `/commands/velocity`.
- [ ] Override with `/teleop/cmd_vel`.
- [ ] Verify emergency priority.
- [ ] Verify expiration and freeze transitions.

Acceptance:

```text
synthetic candidate
    -> safety supervisor
    -> cmd_vel_final
    -> cmd_vel_mux
    -> commands/velocity
```

## 5. Phase 2: Sensor acquisition and perception

Phase 2 begins only after the Phase 1 gate passes.

### P2.1 Hardware acquisition adapters — `NOT STARTED`

- [ ] Launch Kobuki driver.
- [ ] Launch Livox MID-360 driver.
- [ ] Verify `192.168.88.205` host routing.
- [ ] Verify `192.168.88.105` LiDAR reachability.
- [ ] Remap `/livox/lidar` to `/sensors/points`.
- [ ] Remap `/livox/imu` to `/sensors/imu`.
- [ ] Remap `/odom` to `/wheel_odom`.
- [ ] Verify frame IDs and static transform.

### P2.2 Ego state — `NOT STARTED`

- [ ] Consume logical point cloud, IMU, and wheel odometry.
- [ ] Implement timestamp validation.
- [ ] Implement deskew.
- [ ] Publish `EgoState`.
- [ ] Maintain localization epoch.
- [ ] Reject invalid TF-only state.

### P2.3 Local obstacle path — `NOT STARTED`

- [ ] Build fast obstacle representation.
- [ ] Publish world safety snapshot.
- [ ] Keep obstacle veto independent from opponent identification.
- [ ] Measure worst-case callback latency.

### P2.4 Opponent tracking — `NOT STARTED`

- [ ] Segment candidate points.
- [ ] Apply geometric/volume filters.
- [ ] Implement EKF gating.
- [ ] Implement lifecycle transitions.
- [ ] Set `valid_until` explicitly.
- [ ] Invalidate on epoch/map-version mismatch.
- [ ] Implement bounded coasting and occluded belief.

## 6. Phase 3: World model and navigation

### P3.1 World and topology — `NOT STARTED`

- [ ] Load externally supplied structural map.
- [ ] Build occupancy and clearance representation.
- [ ] Build portal graph.
- [ ] Detect articulation points.
- [ ] Version map changes.
- [ ] Publish `WorldSnapshot`.

### P3.2 Global planning — `NOT STARTED`

- [ ] Implement non-negative A* costs.
- [ ] Implement admissible heuristic.
- [ ] Reject stale map versions.
- [ ] Emit feasible path/goal only.

### P3.3 Local control — `NOT STARTED`

- [ ] Implement regulated pursuit.
- [ ] Implement DWA-style arc evaluation where needed.
- [ ] Emit `MotionCandidate`, never physical commands.
- [ ] Enforce nominal hardware limits before the supervisor.

## 7. Phase 4: Decision, match state, and diagnostics

### P4.1 Decision and options — `NOT STARTED`

- [ ] Implement deterministic initial FSM.
- [ ] Define option preconditions.
- [ ] Define option effects and termination.
- [ ] Add hysteresis.
- [ ] Consume belief state without treating it as ground truth.

### P4.2 Match manager — `NOT STARTED`

- [ ] Implement preparation/freeze/active/deadline states.
- [ ] Publish `MatchState`.
- [ ] Implement reset service.
- [ ] Keep score values configuration-driven.

### P4.3 Diagnostics — `NOT STARTED`

- [ ] Display supervisor health.
- [ ] Display command authority.
- [ ] Display track age and validity.
- [ ] Display network and sensor health.
- [ ] Provide safe terminal indicators required by the competition runbook.

## 8. Phase 5: Simulation, replay, and offline learning

### P5.1 Lightweight simulator — `NOT STARTED`

- [ ] Implement deterministic kinematic engine.
- [ ] Reuse `hsl_core`.
- [ ] Implement private referee state.
- [ ] Ensure ground truth never leaks into decision topics.

### P5.2 MVSim adapter — `NOT STARTED`

- [ ] Define `SimulationAdapter`.
- [ ] Implement `reset`, `step`, `observe`, and
  `ground_truth_for_referee`.
- [ ] Validate against the installed MVSim API rather than pseudocode.

### P5.3 Bag replay — `NOT STARTED`

- [ ] Implement replay launch.
- [ ] Use a single bag clock.
- [ ] Remap bag topics to logical topics.
- [ ] Verify no wall-clock/bag-clock mixing.

### P5.4 Offline learning — `DEFERRED`

- [ ] Dirichlet transition estimation.
- [ ] Option value estimation.
- [ ] Bounded parameter evolution.
- [ ] Held-out evaluation.
- [ ] Frozen artifact export.

Learning must never be imported into the critical supervisor path.

## 9. Cross-cutting engineering tasks

### C1. Build and dependency validation — `IN PROGRESS`

- [ ] Build inside the pinned Docker image.
- [ ] Build inherited Kobuki workspace.
- [ ] Build `hsl_interfaces`.
- [ ] Build all Python ROS packages.
- [ ] Run `colcon test`.
- [ ] Capture build logs as artifacts.

### C2. Static quality — `READY`

- [ ] Python linting.
- [ ] XML and interface validation.
- [ ] No simulator imports in decision/safety.
- [ ] No direct physical-topic publication outside approved nodes.
- [ ] No hidden numeric lifecycle states.
- [ ] Type and unit review.

### C3. Configuration schema — `NOT STARTED`

- [ ] Define schemas for YAML configuration.
- [ ] Validate hardware network values.
- [ ] Validate topic names.
- [ ] Validate positive physical constants.
- [ ] Validate official score profile state.

### C4. Integration test harness — `READY`

- [ ] Synthetic ROS publishers.
- [ ] Topic graph assertions.
- [ ] Timing and expiry tests.
- [ ] Mux priority tests.
- [ ] Watchdog rate tests.
- [ ] Clean shutdown tests.

## 10. Dependency graph

```text
B0/B1/B2
   |
   v
P1.1 -> P1.2 -> P1.3 -> P1.4
   \       \       \       \
    +-------> P1.5 -------> P1.6
                    |
                    v
                P1.7 Gate
                    |
        +-----------+-----------+
        v                       v
      P2 Sensor/Perception    P3 World/Navigation
        |                       |
        +-----------+-----------+
                    v
             P4 Decision/Match
                    |
                    v
             P5 Sim/Replay/Learning
```

## 11. Required evidence register

Every completed task must attach or reference:

- source paths changed;
- tests added or run;
- configuration values used;
- timing measurements where relevant;
- failure behavior;
- known limitations;
- reviewer sign-off for safety-critical changes.

No task is `DONE` solely because code exists.

## 12. Current execution state

At the beginning of this register:

- Repository and package scaffolding: `DONE`.
- Interface and hardware configuration baseline: `DONE`.
- Docker and inherited HSL25 base selection: `DONE`.
- Phase 1 implementation: `READY`.
- ROS 2/Docker runtime validation: `BLOCKED` by the current host environment
  until executed inside the target container.

## 13. Developer operating procedure

The repository is intentionally operated with two environments:

1. **Local Conda/venv on Windows or Linux** for the pure-Python mathematical
   core, lightweight kinematic simulation, and offline training.
2. **Docker Desktop with WSL2 or Linux** for ROS 2 Humble, `colcon`, inherited
   Kobuki/Livox drivers, MVSim, and integration tests.

### 13.1 Local fast path

```powershell
conda create -n hsl26 python=3.10 -y
conda activate hsl26
python -m pip install numpy scipy open3d matplotlib pytest
python -m pip install -e .\hsl_core
python -m pytest .\hsl_core\tests -v
python .\sim\kinematic\engine.py
```

This path must remain independent of ROS 2 and should be used for rapid
iteration on P1.1-P1.4 and later offline training.

### 13.2 ROS/integration path

```bash
docker build -f docker/Dockerfile -t hsl26:latest .
docker run --rm -it hsl26:latest bash
colcon test --packages-select hsl_interfaces hsl_safety hsl_bringup
colcon test-result --verbose
```

The pinned `nickodema/kobuki:humble-22.04-100625` base is a compatibility
requirement for the inherited hardware layer, not an interchangeable image
choice.

### 13.3 Execution modes and evidence

- `sim/kinematic`: fast, headless-capable mathematical and tactical checks.
- `sim/mvsim`: ROS 2 dynamic integration and sensor/latency validation.
- `mode:=real`: physical Kobuki/Livox deployment on the Linux NUC.
- `mode:=replay`: rosbag2 regression and calibration workflow.

The intended simulation commands are:

```bash
ros2 launch hsl_bringup hsl26.launch.py mode:=sim role:=guardian
ros2 launch hsl_bringup hsl26.launch.py mode:=sim role:=explorer
```

These commands are operational targets until the current launch scaffolds are
wired to all required nodes. A task may not be marked `DONE` from a launch
command alone; attach topic-graph, timing, and failure-behavior evidence.

### 13.4 Offline training rule

Layer 7 training must run against the lightweight simulator, not MVSim or the
live robot:

```bash
python -m hsl_core.learning.evolution --episodes 5000 --workers 4
```

Policies require provenance, review, and SHA-256 integrity metadata before
being referenced by the competition configuration. Online adaptation remains
disabled.

The next authorized implementation step is **P1.1: Domain types and ROS
mirrors**, followed by the braking model and isolated supervisor skeleton.

## 13. Change-control rules

1. Do not modify inherited hardware drivers unless a compatibility defect is
   demonstrated and documented.
2. Do not add a direct physical command path around `hsl_safety`.
3. Do not change message fields without updating this register, the technical
   design, and all conversion tests.
4. Do not mark safety tasks complete without timing and failure-path evidence.
5. Do not load online learning into competition-critical processes.
6. Do not use undocumented simulator APIs.
7. Keep configuration-driven values outside algorithm source code.
