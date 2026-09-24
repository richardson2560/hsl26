# HSL26 Session Changelog and Audit Report

## 1. Report scope

This document records the net repository changes completed during the HSL26
bootstrap session. The work was performed directly in the main checkout:

```text
C:\Users\edgar\Documents\MASTER ROBOTIC AND AI\PROJECTS\hsl26
```

The initial bootstrap state was represented by commit `fb1d244`
(`basic structure`). This report is retained as an audit record; subsequent
follow-up corrections are listed in §9 and are intentionally visible as
working-tree changes until committed.

The repository was subsequently audited against the inherited Livox/Kobuki
configuration and the real `MID360_tb_config.json` network settings. The
corrections from that follow-up audit are recorded in §9 below.

The objective of the session was to move the repository from an architectural
scaffold to a build-oriented foundation that can receive the real HSL26
implementation without immediately failing because of missing package metadata,
empty interfaces, invalid launch files, or incompatible inherited hardware
configuration.

## 2. Net result

The repository now contains:

- A unified multi-stage Docker build.
- A development entrypoint that chains the ROS 2 environments.
- A Python package definition for `hsl_core`.
- ROS 2 package metadata for all HSL26 Python nodes.
- A buildable ROS 2 interface package with concrete message, service, and
  action definitions.
- Initial configuration files for frames, hardware, perception, tactics, and
  scoring.
- Valid ROS 2 launch-description scaffolds for real hardware, simulation,
  replay, and top-level bringup.
- A safety-oriented `cmd_vel_mux` configuration with emergency teleoperation
  priority over the autonomous supervisor.
- The required `COLCON_IGNORE` markers for nonessential inherited Kobuki
  packages.
- Python package initializers required by setuptools.
- Documentation updated to record the controlled compatibility change in the
  inherited `kobuki` layer.

## 3. Repository structure after the session

The relevant top-level structure is now:

```text
hsl26/
├── docker/
│   ├── Dockerfile
│   └── entrypoint.bash
├── docs/
│   ├── HSL26_FINAL_ARCHITECTURE.md
│   ├── HSL26_REPO_STRUCTURE.md
│   ├── HSL26_SESSION_CHANGELOG.md
│   └── reference images
├── hsl_core/
│   ├── pyproject.toml
│   ├── hsl_core/
│   │   ├── __init__.py
│   │   ├── control/
│   │   ├── learning/
│   │   ├── perception/
│   │   ├── planning/
│   │   └── tactics/
│   └── tests/
├── kobuki/
│   ├── docker/
│   ├── scripts/
│   └── workspace/src/
│       ├── cmd_vel_mux/
│       ├── kobuki_core/
│       ├── kobuki_ros/
│       ├── kobuki_ros_interfaces/
│       └── livox_ros_driver2/
├── ros_ws/src/
│   ├── hsl_interfaces/
│   ├── hsl_perception/
│   ├── hsl_world/
│   ├── hsl_decision/
│   ├── hsl_navigation/
│   ├── hsl_safety/
│   ├── hsl_match/
│   ├── hsl_bringup/
│   └── hsl_diagnostics/
├── sim/
├── config/
├── tools/
├── tests/
└── artifacts/
```

The inherited `kobuki/` directory remains the hardware and driver layer. HSL26
logic is placed in `hsl_core` and `ros_ws/src/hsl_*`, preserving the documented
separation between pure Python algorithms and ROS 2 adapters.

## 4. Changes by area

### 4.1 Inherited Kobuki package filtering

The following empty markers were added under
`kobuki/workspace/src/kobuki_ros`:

- `kobuki_auto_docking/COLCON_IGNORE`
- `kobuki_random_walker/COLCON_IGNORE`

The following markers were confirmed as present:

- `kobuki_testsuite/COLCON_IGNORE`
- `kobuki_controller_tutorial/COLCON_IGNORE`

This prevents optional tutorial, test, docking, and random-walker packages
from being included in the default `colcon` build.

### 4.2 `cmd_vel_mux` compatibility and safety arbitration

The inherited configuration was changed from example inputs to the HSL26
physical command contract in:

- `kobuki/workspace/src/cmd_vel_mux/config/cmd_vel_mux_params.yaml`
- `kobuki/workspace/src/cmd_vel_mux/src/cmd_vel_mux.cpp`

The configured channels are:

```text
/teleop/cmd_vel       priority 100, timeout 0.20 s
/hsl/cmd_vel_final    priority 50,  timeout 0.15 s
/commands/velocity    physical output
```

The upstream implementation originally published to a hard-coded `cmd_vel`
topic and required `short_desc` for every subscriber. The compatibility change
now makes the output topic configurable while retaining `cmd_vel` as the
default when no `output` parameter is supplied.

This preserves the inherited mux behavior while implementing the intended
authority boundary:

```text
emergency teleoperation > autonomous safety supervisor > Kobuki driver
```

No HSL26 navigation node is authorized to publish directly to
`/commands/velocity` or `/hsl/cmd_vel_final`.

### 4.3 Python core packaging

The file [`hsl_core/pyproject.toml`](../hsl_core/pyproject.toml) was added with
setuptools metadata for the pure Python library:

- Package name: `hsl_core`
- Version: `0.1.0`
- Dependencies: NumPy, SciPy, Open3D, and pytest
- Package discovery rooted at `hsl_core/`

Initializers were added to the `control`, `learning`, `perception`, `planning`,
and `tactics` subpackages so setuptools and Python imports resolve reliably.

The mathematical modules remain intentionally separate from ROS 2 code.

### 4.4 ROS 2 interface package

The following metadata was added or completed:

- `ros_ws/src/hsl_interfaces/package.xml`
- `ros_ws/src/hsl_interfaces/CMakeLists.txt`

The package now declares:

- `ament_cmake`
- `rosidl_default_generators`
- `rosidl_default_runtime`
- `geometry_msgs`
- `std_msgs`
- `builtin_interfaces`
- `action_msgs`

Previously empty interface files were replaced by concrete contracts:

- `msg/EgoState.msg`
- `msg/MatchState.msg`
- `msg/MotionCandidate.msg`
- `msg/OpponentTrack.msg`
- `msg/OptionFeedback.msg`
- `msg/OptionGoal.msg`
- `msg/SafetyStatus.msg`
- `msg/WorldSnapshot.msg`
- `srv/ResetStage.srv`
- `action/ExecuteOption.action`

The definitions are initial engineering contracts, not final competition
semantics. They provide the required typed boundaries so the workspace can
start building and the node implementations can be developed against stable
ROS 2 types.

### 4.5 HSL26 Python ROS 2 packages

Package metadata was added to all eight Python packages:

- `hsl_perception`
- `hsl_world`
- `hsl_decision`
- `hsl_navigation`
- `hsl_safety`
- `hsl_match`
- `hsl_bringup`
- `hsl_diagnostics`

Each package now contains:

- `package.xml`
- `setup.py`
- `setup.cfg`
- `resource/<package_name>`
- Python package initializer where applicable

The package setup files install package metadata and launch files. The
`hsl_bringup` setup additionally installs its YAML configuration files.

The invalid `hsl_safety` entry points that referenced nonexistent
`supervisor_node` and `watchdog` modules were removed. This prevents a
successful package installation from producing executables that fail at
runtime.

### 4.6 Bringup configuration

The following files were changed from comment-only placeholders into explicit
base configuration:

- `ros_ws/src/hsl_bringup/config/frames.yaml`
- `ros_ws/src/hsl_bringup/config/hardware.yaml`
- `ros_ws/src/hsl_bringup/config/perception.yaml`
- `ros_ws/src/hsl_bringup/config/score_profile.yaml`
- `ros_ws/src/hsl_bringup/config/tactics.yaml`

The configuration establishes:

- Standard `map`, `odom`, `base_link`, `lidar_link`, and `imu_link` frame names.
- Kobuki command and sensor topic names.
- Conservative initial hardware limits.
- Sensor QoS intent.
- Measurement-age and obstacle-clearance defaults.
- Stage timing defaults.
- An explicit `official_values_loaded: false` state for score data.
- A deterministic FSM policy with learning disabled by default.

No official maze coordinates, obstacle coordinates, or unannounced competition
values were invented.

### 4.7 Launch scaffolding

The following launch files now contain valid
`generate_launch_description()` functions:

- `hsl_bringup/launch/hsl26.launch.py`
- `hsl_bringup/launch/real_hardware.launch.py`
- `hsl_bringup/launch/sim_mvsim.launch.py`
- `hsl_bringup/launch/replay_bag.launch.py`

The top-level launch accepts:

- `mode`: `real`, `sim`, or `replay`
- `role`: `unset`, `explorer`, or `guardian`
- `bag`: replay bag path

The profile-specific launch files currently emit explicit scaffold messages
instead of starting nonexistent hardware, simulator, or node executables. This
is deliberate: a launch file that starts incorrect or unavailable components
would be more dangerous than a validated placeholder.

### 4.8 Docker build

The unified build files are:

- [`docker/Dockerfile`](../docker/Dockerfile)
- [`docker/entrypoint.bash`](../docker/entrypoint.bash)

The Dockerfile has two stages:

1. `kobuki-base`
   - Starts from `osrf/ros:humble-desktop`.
   - Installs hardware and networking dependencies.
   - Builds `kobuki/workspace/src`.

2. `hsl26-final`
   - Installs Python numerical dependencies.
   - Installs `hsl_core` in editable mode.
   - Builds `ros_ws/src`.
   - Installs the chained ROS 2 entrypoint.

The entrypoint sources:

```text
/opt/ros/humble/setup.bash
/workspace_kobuki/install/setup.bash
/workspace_hsl26/install/setup.bash
```

The Docker build context is the repository root, so all `COPY` paths resolve
against the actual repository structure.

### 4.9 Documentation

[`docs/HSL26_REPO_STRUCTURE.md`](./HSL26_REPO_STRUCTURE.md) was updated to
document that the inherited `kobuki` layer remains frozen in principle, with
one controlled compatibility change in `cmd_vel_mux` to honor its configured
output topic.

This changelog was added as the audit record for the complete session.

## 5. Validation performed

The following checks passed:

- Python AST parsing for all ROS package `setup.py` files.
- Python AST parsing for all Python files under the ROS packages.
- XML parsing for all ROS `package.xml` files.
- Static validation of ROS interface files.
- `setup.py --name` for all eight Python ROS packages.
- Python bytecode compilation with `compileall`.
- `git diff --check`.
- Verification that the required Docker build-context paths exist.
- Verification of `cmd_vel_mux` priorities, timeouts, topics, and output.

The inherited `kobuki` packages were inspected, including:

- `kobuki_core`
- `kobuki_ros`
- `kobuki_ros_interfaces`
- `livox_ros_driver2`
- `cmd_vel_mux`

## 6. Validation not available in this environment

The following full checks were not executable in the current Windows
environment:

- `colcon build`, because ROS 2 is not installed locally.
- `docker build`, because Docker is not available locally.
- `pytest`, because pytest is not installed in the active Python environment.

These limitations do not invalidate the static checks, but the first
containerized validation pass must run:

```bash
docker build -f docker/Dockerfile -t hsl26:bootstrap .
```

Inside the resulting ROS 2 environment, the next required validation is:

```bash
source /opt/ros/humble/setup.bash
source /workspace_kobuki/install/setup.bash
source /workspace_hsl26/install/setup.bash
colcon test --event-handlers console_direct+
```

## 7. Current readiness assessment

### Ready

- Repository layout.
- Python and ROS 2 package metadata.
- ROS interface generation inputs.
- Docker build definition.
- Environment chaining.
- Basic configuration contracts.
- Safety mux topic and priority contract.
- Static syntax and packaging validation.

### Not yet implemented

- Real perception node behavior.
- World map and topology node behavior.
- Decision and option execution behavior.
- Global planner and local controller behavior.
- Safety supervisor and watchdog executables.
- Match-stage runtime logic.
- Diagnostics runtime behavior.
- Real hardware launch wiring.
- MVSim launch wiring.
- ROS bag replay execution.
- Full integration tests.

### Important operational status

The repository is now ready to begin implementation of the real solution, but
it is not yet ready for autonomous robot operation. The launch scaffolds are
intentionally conservative and the safety supervisor remains an implementation
requirement, not an optional component.

## 8. Audit conclusion

The session converted the repository from a documentation-oriented scaffold
with empty package contracts into a coherent build foundation. The inherited
Kobuki/Livox layer remains recognizable and reusable, while HSL26-specific
interfaces, packaging, configuration, Docker integration, and command
authority boundaries are now explicit.

The next engineering phase should implement one vertical slice at a time,
starting with:

1. `hsl_interfaces` build verification inside ROS 2.
2. `hsl_core` unit-test restoration and dependency installation.
3. The isolated safety supervisor and watchdog.
4. A minimal navigation-to-supervisor-to-mux integration test.
5. Hardware and simulator bringup only after those safety contracts pass.

## 9. Follow-up audit corrections

The first bootstrap review identified four compatibility risks. They were
corrected after comparing the HSL26 files with the inherited HSL25 workspace:

1. **Docker hardware base:** `docker/Dockerfile` now uses the pinned
   `nickodema/kobuki:humble-22.04-100625` image as the first-stage base. That
   image is the inherited environment expected to contain Livox-SDK2 and the
   Kobuki/ecl dependencies. The `base_img` build argument remains overridable,
   and the inherited workspace is rebuilt inside the stage.
2. **Livox/Kobuki network and topic contract:** `hardware.yaml` now documents
   the confirmed MikroTik subnet (`192.168.88.0/24`), NUC address
   (`192.168.88.205`), and MID-360 address (`192.168.88.105`). It distinguishes
   raw driver topics (`/livox/lidar`, `/livox/imu`, `/odom`) from the logical
   HSL26 topics (`/sensors/points`, `/sensors/imu`, `/wheel_odom`).
3. **Planar ego covariance:** `EgoState.msg` now uses `float64[9]`, matching
   the planar `(x, y, theta)` `Pose2D` state.
4. **Opponent temporal validity:** `OpponentTrack.msg` now exposes
   `valid_until`, `localization_epoch`, `map_version`, and named lifecycle
   constants for `SEARCHING`, `TRACKED`, `COASTING`, `OCCLUDED_BELIEF`, and
   `LOST`.

An additional packaging conflict was corrected at the same time:
`hsl_bringup` now has its Python package initializer under
`ros_ws/src/hsl_bringup/hsl_bringup/`, matching its `setup.py` declaration.

The corrected files were validated with static parsing, package metadata
commands, topic/configuration checks, and `git diff --check`. A full ROS 2
build and Docker build still require the target container environment.

## 10. Operational workflow documentation

The repository now documents the intended hybrid development workflow in the
root `README.md`, the Phase 1 technical design, and the implementation
roadmap:

- Conda/venv is the fast path for `hsl_core`, the lightweight kinematic
  simulator, and offline Layer 7 training.
- Docker Desktop with WSL2 or Linux is the supported path for ROS 2 Humble,
  `colcon`, Kobuki/Livox, MVSim, and integration tests.
- The four execution modes are explicitly separated: lightweight kinematic
  simulation, MVSim, real hardware, and rosbag replay.
- Docker build and ROS test commands are recorded using the pinned HSL25
  hardware image.
- Simulation launch commands and the offline training command are documented
  as operational targets, with an explicit warning that current launch files
  remain scaffolds until node wiring and acceptance evidence exist.
- Policy artifacts require provenance and SHA-256 integrity metadata, and
  online adaptation remains disabled for competition execution.

This documentation change does not claim that the simulator, launch graph, or
training CLI is already production-ready; it establishes the reproducible
workflow and the evidence required before those surfaces can be marked done.

## 11. P1.1 repository audit and migration manifest

P1.1a–P1.1e were completed as a read-only audit of commit
`1ca74154e8ef8ea3cde24fb0c595e794b24c2c60`. The new evidence is:

- `docs/adr/ADR-003-interface-revision-2.md`, documenting the revision-2
  migration boundary, current bootstrap incompatibilities and command
  authority decisions.
- `artifacts/reports/phase1/P1.1_migration_manifest.json`, containing the
  package/entry-point/launch/test/interface inventory, producer/consumer
  migration table, SHA-256 hashes, mux/Kobuki timeout findings and Livox
  message-time analysis.

The audit found no unexpected direct `/commands/velocity` publisher:
`cmd_vel_mux` remains the sole known physical command authority before the
Kobuki input. It also records four open follow-up items: completing the
revision-2 IDL, wiring runtime producers/consumers, measuring mux/driver
stop behavior, and resolving the active Livox topic/type and point-time
normalization. No Python file was modified.

## 12. P1.2 revision-2 interface contracts

P1.2a–P1.2d replaced the bootstrap `hsl_interfaces` schemas with the
revision-2 contract set defined by the technical specification:

- Added `ContractHeader`, geometry/coverage/obstacle records, topology records,
  execution identity, path/candidate/safety authority records, heartbeat and
  rule-event messages.
- Reworked `EgoState`, `OpponentTrack`, `MatchState`, `OptionGoal`,
  `OptionFeedback`, `MotionCandidate`, `SafetyStatus` and `WorldSnapshot` to
  use `ContractHeader` and revision-2 field names.
- Added `OptionResult`, `StartStage`, `SetGoalZone`, `ResolveEvent` and
  `RearmSafety`; upgraded `ExecuteOption` and `ResetStage` transaction shapes.
- Registered all 24 messages, 5 services and 1 action in the interface
  generator, with generated-test integration through `ament_cmake_pytest`.
- Added static contract tests and sample accepted/rejected encoding fixtures.

Validation passed with 5 static contract tests and complete CMake path
coverage. The ROS 2 generated build is `BLOCKED` on the unavailable pinned
ROS 2/colcon environment; this limitation and the required build command are
recorded in `artifacts/reports/phase1/P1.2_contract_report.json`. Existing ROS
adapters are intentionally not updated in P1.2; they remain incompatible
until P1.5 performs explicit revision-2 adapter migration. The new Python test
file includes its repository path in the required header comment.

## 13. P1.3 pure core foundation

P1.3a–P1.3d are complete. The pure core now provides:

- immutable revision-2 metadata validation, lease/epoch checks, finite
  covariance validation and non-degenerate counterclockwise polygon checks;
- exact SE(2) differential-drive integration with a small-angle `sinc`
  continuation, wheel-rate conversion and planar frame transforms;
- robust segment intersection, strict capture distance (`d < 0.45 m`),
  inclusive bearing (`|beta| <= pi/4`), explicit LOS validity handling,
  coincident-origin rejection and robust capture intervals;
- first-contour-arrival detection with interpolated contact time and point.

The new implementation is ROS-independent and does not read any wall clock.
Boundary and degenerate fixtures are in
`hsl_core/tests/test_p13_foundation.py`; the evidence report is
`artifacts/reports/phase1/P1.3_core_foundation_report.json`. Validation passed
with 9 tests and Python compilation. Full sensor deskew, runtime lease
integration and later world-event fixtures remain assigned to their respective
future phase tasks.

## 15. Removal of obsolete empty test stubs

The repository structure was compared against the revision-2 implementation
boundaries. Six tracked test files contained only a path comment and no test
code: `test_astar.py`, `test_braking.py`, `test_ekf_gating.py`,
`test_kinematics.py`, `test_rules.py` and `test_topological_belief.py`.
They were removed because they falsely inflated the apparent test surface and
were superseded for the current P1.3 slice by
`hsl_core/tests/test_p13_foundation.py`. The corresponding future-phase test
modules must be recreated with real fixtures when their implementation work
starts; they were not silently marked as passed.

No target implementation module was deleted: empty core modules remain because
they are explicitly reserved by `HSL26_REPO_STRUCTURE.md` for later phases.

The cleanup audit also tightened integer metadata validation to reject
`bool` values and replaced dimensional orientation products with direct sign
comparisons in polygon/segment predicates. The audit report and P1.1
migration manifest were updated so evidence contains no references to deleted
stubs.

## 16. Canonical TrackState naming audit

Compared pure-core names and safety conversion references against the
technical specification. Renamed the lifecycle enum definition and type
annotation to canonical `TrackState` with values `0..4`; removed the
pre-revision `OpponentState` compatibility alias so invalid names fail fast.
Updated the tests and adapter imports to use `TrackState`, and added required
repository-path headers to the modified Python files. No future-phase tracker
or belief implementation was invented during this audit.

The same comparison found two non-silent follow-up items: the current
`hsl_safety` conversion module still targets bootstrap message fields, and
some P1.3 helper names predate the final §16 API catalog. Both are recorded as
explicit limitations in the audit report. They require coordinated migrations
of domain records, adapters, result types and fixtures; compatibility
fallbacks were deliberately not added because they would conceal revision-2
contract violations.

## 17. P1.4 braking and pure safety evaluator

Implemented the specification-defined constant-speed delay braking model,
stable admissible-speed inversion, delay-acceleration model and inverse, plus
validated differential-drive wheel-rate limits in
`hsl_core/hsl_core/control/braking.py`. All calculations document SI units
and reject nonfinite, negative, or nonphysical configuration values.

Added `LimitsProfile`, `TimingProfile`, `SafetySnapshot`,
`SafetyEvaluation` and `SafetySupervisor` in
`hsl_core/hsl_core/control/safety.py`. The evaluator supports only the
explicitly permitted conservative forward scalar corridor. It requires finite
clearance, matching frame and clock/localization epochs, fresh candidate/ego
and obstacle timestamps, valid coverage and option identity. It returns
`ADMIT`, `LIMIT` or zero `STOP`, reports `BRAKING_INFEASIBLE` for an already
unsafe stopping state, and forces zero on compute overrun. Reverse motion,
independent rotation and full swept-footprint/dynamic-obstacle checking remain
disabled until their separate evidence exists.

Added adversarial numerical and safety tests in
`hsl_core/tests/test_braking.py` and `hsl_core/tests/test_safety.py`, plus
`artifacts/reports/phase1/P1.4_safety_report.json`. The pure-core suite
initially passed 24 tests, including future-timestamp, stale-lease, identity,
overrun and already-infeasible stopping-state cases.

## 18. Independent P1.4 audit follow-up

Re-audited the attached P1.4 findings and confirmed the admitted-command
stopping distance was previously reported from ego speed rather than the
command speed, and that candidate freshness did not consume
`candidate_lease_s`. Both are corrected. Invalid negative candidate/ego speeds
are now represented as evaluable safety inputs and force `LIMITS_INVALID`
instead of crashing snapshot construction. Steady-clock regression now forces
`COMPUTE_OVERRUN`, and the configured acceleration-during-response bound is
used by the supervisor's stopping and admissible-speed calculations.

Added adversarial tests for these cases and for invalid sequence, boolean and
coverage metadata. The pure-core suite now passes 30 tests.

Overrun and steady-clock-regression STOP results now preserve the computed ego
stopping-distance diagnostic instead of reporting a misleading zero.

## 19. P1.5 ROS adapters and deterministic mocks

Replaced the bootstrap safety conversion module with the explicit revision-2
boundary adapter in `ros_ws/src/hsl_safety/hsl_safety/adapters.py`.
`AdapterContext` validates receiver-local ROS/steady time, schema, validity,
epochs, leases, per-record frames and option identity. `decode_safety_snapshot`
accepts the normative frame split (`MotionCandidate` in `base_link`,
`EgoState`/`LocalObstacleSnapshot` in `odom`) and
`validate_authority_context` permits empty frames only for metadata-only
authority records. `ValidatedCache` rejects stale or replayed sequence/time
pairs before replacement.

Added ROS-free deterministic fixtures for controllable ROS and steady clocks,
source sequence/session and epoch restarts, ego-local/obstacle/candidate/
match/execution/watchdog-health records, and separate autonomy/watchdog/mux
observation sinks. Mocks are test-only and are not added to production launch
files or physical command paths.

Removed the obsolete bootstrap `conversions.py` and its bootstrap conversion
tests so stale field names cannot be reused. Added strict P1.5 adapter/mock
tests and evidence in
`artifacts/reports/phase1/P1.5_adapter_mock_report.json`.

## 14. Independent audit and adversarial verification

An independent audit of P1.2/P1.3 identified and corrected five concrete
issues that the original happy-path tests did not expose:

- angular radians had been added directly to a metric capture distance;
- polygon validation did not reject repeated internal vertices or
  self-intersections;
- header validation accepted `INVALID` records and future publication times;
- `ContractHeader` incorrectly required `stage_id` even for sensor/diagnostic
  metadata;
- covariance validation accepted invalid dimensions and lacked PSD boundary
  coverage.

The corrected API makes angular-to-linear conversion explicit through an
angular reference radius, rejects invalid geometry and separates generic
metadata from authority-stage requirements. Additional adversarial tests cover
singular PSD matrices, indefinite matrices, future leases, invalid records,
duplicate/self-intersecting polygons, wheel-rate round trips and exact
kinematic time composition.

The audit evidence is
`artifacts/reports/phase1/P1_audit_independent_review.json`. The final pure
core run was 10 passed, Python compilation passed, `git diff --check` passed,
and no ROS imports or internal wall-clock reads were found in the pure core.
Remaining limitations are explicitly retained: rosidl/colcon generation,
runtime sequence/session caches, production-scale geometric tolerance policy,
sensor deskew and full world/stage integration.

## 20. P1.6 supervisor and watchdog boundary

Audited the external P1.5 verdict against the revision-2 specification. The
reported scope is valid, but the missing output encoders were a real P1.6
dependency. Added strict `encode_safety_status` and
`encode_supervisor_heartbeat` adapters with copied metadata, validated enum and
identity fields, and exact nanosecond duration serialization.

Implemented ROS-independent, deterministic P1.6 runtime seams in
`ros_ws/src/hsl_safety/hsl_safety/supervisor_node.py` and
`ros_ws/src/hsl_safety/hsl_safety/watchdog.py`. The supervisor advances a
monotonic decision sequence, gates evaluation on watchdog health, and emits a
zero result for missing payloads, exceptions or evaluator failures. The
watchdog starts `DISARMED`, asserts stop output until fresh progressing
identity-bound heartbeats and stage authority are present, rejects stale or
replayed heartbeats, latches faults, and requires an authorized near-zero
rearm with dwell before returning to `READY`. It never produces a nonzero
twist.

The adapter now also rejects candidate/ego/obstacle records with inconsistent
stage identities. Added separate-process launch and console entry point
declarations. P1.6 runtime tests cover startup zeros, health gating, exception
fallback, heartbeat ordering, stale leases, stage/config mismatches, fault
latching, controlled rearm, encoder round trips and invalid output values.
The combined pure-core/P1.5/P1.6 run passes 53 tests. Native ROS 2 execution,
generated-message serialization and launch tests remain `BLOCKED` because the
pinned ROS 2 Humble environment is unavailable on this Windows host.

## 21. P1.7 mux and fault integration

Audited the installed `cmd_vel_mux` implementation rather than assuming its
priority semantics. The normative physical chain is now explicit in
`kobuki/workspace/src/cmd_vel_mux/config/cmd_vel_mux_params.yaml`:
`/hsl/cmd_vel_stop` at priority 200, `/teleop/cmd_vel` at 100, and
`/hsl/cmd_vel_final` at 50, with `/commands/velocity` as the only physical
output. The stop input is explicitly zero-only. The same topics, priorities
and leases are recorded in the HSL26 hardware profile.

Added the ROS-free deterministic mux contract in
`ros_ws/src/hsl_safety/hsl_safety/mux_contract.py`. It rejects invalid or
non-finite commands, guarantees zero-only cold start, selects only fresh
inputs by priority, rejects nonzero watchdog stop messages and never replays
expired commands. Added
`tools/check_ros_graph_authority.py`, which rejects missing or multiple
publishers of the physical output and identifies unauthorized legacy writers.

Added adversarial P1.7 tests for cold start, simultaneous stop/teleop/autonomy,
lease expiry, watchdog zero-only behavior, graph authority and exact YAML
configuration. Evidence is stored in
`artifacts/reports/phase1/P1.7_mux_fault_report.json`.

P1.7 software-contract tests pass. Native ROS launch tests, live graph
inspection, process-death timing, Kobuki driver timeout and physical-stop
response remain explicitly `BLOCKED` because ROS 2 Humble and the hardware are
not available on this host. The real motion profile remains disarmed.

## 22. P1.8 evidence, runbook and release candidate

Completed the P1.8 evidence package without promoting unavailable runtime or
hardware checks to false positives. Added
`artifacts/reports/phase1/P1.8_release_evidence_report.json` with separate
software, ROS-runtime and physical-response statuses, exact commands,
environment versions and the G0 closure decision.

Added `docs/runbook_competition.md` covering the revision-2 command chain,
zero-safe startup, normal/emergency stop, controlled rearm, fault responses,
graph-authority checks and evidence preservation. The runbook explicitly
states that a zero command receipt is not proof of physical stopping.

Added `artifacts/releases/P1_release_candidate_manifest.json` containing the
dirty source revision, SHA-256 hashes of normative and safety-boundary files,
gate-by-gate evidence links, reproducibility commands, release blockers and
the restricted Phase-2 handoff. The candidate is intentionally
`BLOCKED_FOR_RELEASE`: ROS 2 generated builds, native launches, live graph
inspection, process-fault timing, driver timeout and physical stop response
remain unavailable. The real motion profile remains disarmed.
