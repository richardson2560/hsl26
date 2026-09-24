# ADR-003: Revision-2 interface migration and command authority

- **Status:** Accepted for Phase 1 audit; implementation deferred to P1.2
- **Date:** 2026-09-24
- **Scope:** P1.1a-P1.1e repository audit
- **Authority:** `HSL26_PHASE1_IMPLEMENTATION_PLAN.md` revision 2.0 and
  `HSL26_TECHNICAL_SPECIFICATION.md` revision 2.2

## Context

The live checkout is the implementation evidence. The bootstrap interfaces and
launch files exist, but the repository does not yet contain a complete
revision-2 contract or runtime producer/consumer graph. P1.1 must freeze the
migration boundary without inventing adapters or allowing a second physical
velocity-command authority.

The audit was read-only at commit `1ca74154e8ef8ea3cde24fb0c595e794b24c2c60`.
The machine-readable evidence is in
`artifacts/reports/phase1/P1.1_migration_manifest.json`.

## Findings

### Repository and runtime surface

- Eight HSL26 Python packages are present under `ros_ws/src`, plus the
  `hsl_interfaces` CMake package.
- Seven HSL26 launch packages expose scaffold launch descriptions for bringup,
  sensing, world, navigation, decision, match and safety.
- The pure-core test suite contains seven test modules; the ROS safety package
  contains one conversion test module.
- `git status --short` was clean at audit time. The audited revision is recorded
  in the manifest.
- HSL26 Python nodes currently contain no `create_publisher`,
  `create_subscription`, or `publish` calls. Their package dependencies are
  declarations, not evidence of a connected runtime graph.

### Contract migration

The current `hsl_interfaces` package generates eight messages, one service and
one action. They are bootstrap contracts. The Phase 1 plan requires additional
revision-2 records (`ContractHeader`, coverage/obstacle snapshots,
`ExecutionState`, heartbeat/watchdog and rule event) and explicit header,
epoch, map-version and validity semantics. Existing fields must therefore be
replaced or migrated deliberately in P1.2; they must not be silently extended
or treated as revision-2 compatible.

Current ROS conversion code reads/writes `EgoState`, `OpponentTrack`,
`MotionCandidate` and `SafetyStatus`, but no runtime subscriber/publisher is
wired. These conversion users are incompatible until their field mapping is
updated and generated APIs are built from the revision-2 IDL.

### Command authority and timeout behavior

The only configured physical command path is:

`/teleop/cmd_vel` or `/hsl/cmd_vel_final` -> `cmd_vel_mux` ->
`/commands/velocity` -> Kobuki driver.

The vendored mux publishes its configurable `output` and selects the highest
priority non-expired input. The current priorities are emergency teleoperation
100 with a 0.20 s timeout and the HSL26 supervisor 50 with a 0.15 s timeout.
The Kobuki node subscribes to `commands/velocity` and independently zeros the
base after its configured command timeout, whose source default is 0.60 s.

No unexpected direct `/commands/velocity` publisher was found in HSL26 source.
The mux output is therefore the sole known physical command authority. The
relative timeout budget remains an open P1.7 integration item: P1.1 does not
claim that these values constitute a measured stop guarantee.

### Livox message and time representation

`mid360.launch.py` selects transfer type `0`, which is ROS 2
`sensor_msgs/msg/PointCloud2`. The vendored driver also defines transfer type
`1`, `livox_interfaces2/msg/CustomMsg`, whose `CustomPoint.offset_time` is a
`uint32` offset relative to `CustomMsg.timebase` in nanoseconds. The MID360
hardware configuration selects Cartesian 32-bit point data
(`pcl_data_type: 1`), which is a device point encoding and not the ROS
transfer-message selector.

The current HSL26 configuration names `/livox/lidar` but does not prove the
resolved driver topic/type or point-time field at runtime. P1.2/P2 must choose
and validate the normalized sensor adapter; no adapter is invented by P1.1.

## Decisions

1. Treat the live checkout and its hashes as the migration baseline.
2. Treat all current HSL26 interfaces as bootstrap revision 0/1 artifacts until
   P1.2 generates and validates the revision-2 IDL.
3. Preserve the mux as the sole physical velocity-command owner. Any future
   direct `/commands/velocity` producer is a Phase-1 blocker.
4. Keep the mux and driver timeout discrepancy visible as an integration risk;
   resolve it with measured evidence in P1.7 rather than changing inherited
   hardware code during the repository audit.
5. Record Livox transfer type and point-time ambiguity as an open compatibility
   item. Do not infer a normalized point-time contract from `pcl_data_type`.

## Consequences and migration checklist

- P1.2 must freeze the revision-2 IDL, enum values, field order, units and
  header semantics before adapters are connected.
- P1.2/P1.5 must produce a generated API compatibility report and explicitly
  mark scaffold-only producers as missing, rather than implying compatibility.
- P1.7 must measure mux expiry, Kobuki input timeout and physical zero response
  independently.
- P2 must verify the selected Livox ROS message, topic, timestamp source,
  `offset_time` interpretation and deskew input on real driver output.
- The manifest remains the reviewable authority record for P1.1 and must be
  updated whenever the migration boundary changes.
