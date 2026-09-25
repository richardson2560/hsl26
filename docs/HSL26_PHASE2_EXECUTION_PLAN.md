# HSL26 - Phase 2 execution plan

**Revision:** 1.1  
**Baseline:** `HSL26_PHASE2_SENSING_AND_CALIBRATION.md` 1.0,
architecture/blueprint 2.2 and the Phase-1 release evidence.  
**Status:** SIL/kinematic preparation; physical and native ROS gates remain
blocked until the robot and the pinned ROS 2 environment are available.

## 1. Purpose and authority

This document is the working plan for Phase 2. The normative requirements are
the technical specification, `HSL26_FINAL_ARCHITECTURE.md` and
`HSL26_PHASE2_SENSING_AND_CALIBRATION.md`, in that order after the official
competition rules. The Phase-1 reports are evidence of the current software
baseline, not evidence of physical behavior.

The absence of the robot changes the validation method, not the contract:

* SIL tests may validate units, transforms, timing, rejection behavior and
  deterministic algorithms with synthetic data.
* Replay tests may validate the same contracts against recorded bags when they
  become available.
* HIL/physical tests are the only authority for sensor inventory, driver
  timeout, stop response, friction, footprint and blind coverage.
* A SIL PASS must never be promoted to a physical PASS.

## 2. Execution profiles

The profile is selected outside the pure core. Pure modules do not read clocks,
ROS state, simulator truth or hardware parameters implicitly.

| Profile | Allowed inputs | Expected use now | Physical authority |
|---|---|---|---|
| `kinematic` | deterministic NumPy fixtures and the plant model | default algorithm development | none |
| `replay` | recorded, immutable sensor/odom data | regression when bags exist | none; no simulator truth |
| `sim` | MVSim outputs through an adapter | sensor/launch integration | only if sensor timing/coverage is inspected |
| `real` | authorized ROS topics on the robot | competition preparation | required for G0/G1 |

`real` remains disarmed until the stop chain, mux authority and driver timeout
are physically verified. The Phase-2 environment records this policy in
`artifacts/reports/phase2/P2_environment_baseline.json`.

## 3. Work packages and order

### P2.0 Environment and traceability (this change)

Create the profile policy, timing/geometry placeholders, evidence directory,
synthetic-fixture convention and this execution plan. Verify that the Phase-1
software test command remains reproducible.

### P2.1 Hardware integration

On the preparation day, inspect the actual message type and per-point timing
origin before implementing a decoder. Record topic, type, QoS, frame, clock
epoch and timeout in a manifest. Do not infer `CustomMsg` fields from a generic
`PointCloud2`. Confirm the sole physical writer remains the mux output.

Deliverables: hardware inventory, topic manifest, mux timeout trace and
`BLOCKED_FOR_HARDWARE` status until the supervised test is complete.

### P2.2 Ego, frames and deskew

Implement the pure `deskew_cloud` contract in `hsl_core/hsl_core/kinematics.py`
after the timestamp convention is fixed. The function must:

1. reject missing, non-finite or unsupported per-point time;
2. use an injected pose history/interpolator, never an internal clock;
3. transform every point to one declared reference time/frame;
4. preserve point order and return deterministic numeric output.

The first SIL fixture is a static wall at `x=2.0 m`, timestamps in
`[0, 0.1] s`, and a known yaw motion. Acceptance is residual less than
`1 mm` for the synthetic fixture only. That threshold does not certify Livox
calibration.

### P2.3 Fast obstacles and coverage

Implement `LocalObstacleBuilder` in `hsl_core/hsl_core/mapping.py` with an
explicit configuration object. The fast path must retain occupied returns and
unknown/blind cells; it must not clear cells behind a first hit. The initial
fixtures cover:

* ground rejection below `z=-0.10 m`;
* upper crop above `z=0.40 m`;
* self-body rejection within the configured base radius;
* frontal corridor width `0.40 m`;
* a low obstacle at `1.50 m`;
* no-return/invalid rays that leave coverage unknown.

`free_distance_m` is a bounded snapshot field, not a replacement for the
coverage contract or the obstacle geometry.

### P2.4 Calibration tooling

Prepare `tools/calibrate_braking.py` as a real-hardware tool with a dry-run
path that cannot publish motion. On the robot it must record raw odometry,
the first zero request and the measured rest event separately. It must reject
incomplete trials and write raw traces plus `braking_report.json`.

The values in `hardware.yaml` are provisional configuration inputs only. They
must not be labelled measured or used to close G1.

### P2.5 Adapter and fault exercise

The ROS-free safety boundary now validates generated-message-shaped payloads
before they can influence authority. `decode_ego_state_message` checks exact
metadata identity, finite pose/twist values, 3x3 and 2x2 covariance shape and
PSD validity, boolean health flags, localization validity and calibration
provenance. `validate_obstacle_payload` checks bounded grid dimensions,
resolution/origin, one cell and observation timestamp per grid cell, the
three-state alphabet, obstacle count and finite obstacle bounds.

`ValidatedCache` replaces records only after all checks pass and rejects
replayed sequence/timestamp pairs. `validate_observation_progress` rejects
non-monotonic timestamps, configured dropout gaps and discontinuous pose
jumps. A stale observation may be re-emitted for diagnostics, but its original
observation/publication/expiry metadata must be retained; re-publication never
refreshes a lease or grants motion authority. Invalid localization and
`frontal_coverage_valid=False` remain non-authoritative even when the payload
is structurally complete.

The tests cover these cases plus overload limits and exact coverage flags.
Native ROS tests remain separate from pure-core tests and are not run on this
Windows host unless the pinned container is available.

## 4. P2.3 coverage semantics and P2.4 dry-run

`LocalObstacleSnapshot.complete` means that the bounded mapping pipeline
processed a structurally valid input. It does **not** mean that the frontal
corridor is observable. `frontal_coverage_valid` is the separate authority
flag consumed by the safety adapter. `free_distance_m` walks the centreline
coverage from the robot footprint to the first `OCCUPIED` or `UNKNOWN` cell.
Therefore:

* observed free cells produce a positive bounded distance;
* an obstacle stops the distance at its occupied cell;
* unknown space stops the distance conservatively;
* an empty, side-only or out-of-grid observation cannot certify forward
  coverage;
* the grid boundary is never reported as free merely because no return exists.

This separation resolves the apparent empty-corridor paradox without treating
unknown space as free. The current revision adds the field to
`LocalObstacleSnapshot.msg` and requires both flags in the safety adapter.

P2.4 is prepared by `tools/calibrate_braking.py`. `--dry-run` creates a
deterministic synthetic report using 50 Hz odometry samples and labels it
`PASS_SYNTHETIC_ONLY`; `--real` returns `REAL_BLOCKED_FOR_HARDWARE` until an
authorized ROS/Kobuki adapter exists. Synthetic `b_min`, delay and noise are
never written into `hardware.yaml` as measured calibration.

## 5. Evidence and gate policy

Every evidence record must include the profile, source revision, command,
environment versions, input provenance, result, uncertainty and status.
Statuses are:

* `PASS`: the scoped software/replay assertion passed;
* `BLOCKED_FOR_HARDWARE`: a physical prerequisite is unavailable;
* `NOT_RUN`: the test is defined but not attempted;
* `FAIL`: the scoped assertion failed.

Required Phase-2 evidence files:

| Evidence | Planned location | Current status |
|---|---|---|
| Environment/profile baseline | `artifacts/reports/phase2/P2_environment_baseline.json` | PASS (configuration only) |
| Deskew residuals | `artifacts/reports/phase2/P2.2_deskew_report.json` | PASS (SIL) |
| Obstacle/coverage fixtures | `artifacts/reports/phase2/P2.3_mapping_report.json` | PASS (SIL) |
| Hardware inventory and topic manifest | `artifacts/reports/phase2/P2.1_hardware_manifest.json` | BLOCKED_FOR_HARDWARE |
| Braking trials | `artifacts/reports/phase2/braking_report.json` | BLOCKED_FOR_HARDWARE |
| Adapter fault exercise | `artifacts/reports/phase2/P2.5_adapter_fault_report.json` | PASS (ROS-free SIL; native ROS blocked) |
| G1 acceptance record | `artifacts/reports/phase2/P2_G1_acceptance_record.json` | BLOCKED_FOR_HARDWARE |

Raw bags and traces are immutable evidence inputs; derived reports must include
their SHA-256 hashes.

## 6. Development environment and dependencies

For the current SIL/kinematic work, no library beyond the dependencies already
declared in `hsl_core/pyproject.toml` is required:

* Python 3.10+;
* NumPy for point arrays and deterministic fixtures;
* SciPy for numerical interpolation/estimation where selected by a contract;
* pytest for unit and adversarial tests.

Open3D is already declared for future point-cloud tooling, but it is not
required by the pure deskew or mapping tests and must not become an implicit
runtime dependency of the safety path. PyYAML is not needed by the pure core;
ROS 2 Humble, `colcon`, Livox/Kobuki drivers and MVSim belong to the container
or replay/HIL profiles, not the Windows SIL baseline. Install nothing else
until a targeted implementation requires it.

Reproducible Windows SIL commands from a checkout (the first command is run
from the package root so the local `hsl_core` package is importable; the
second adds the ROS-free safety package to `PYTHONPATH`):

```powershell
Set-Location hsl_core
python -m pytest tests -q
Set-Location ..
$env:PYTHONPATH = "$(Get-Location)\hsl_core;$(Get-Location)\ros_ws\src\hsl_safety"
python -m pytest ros_ws\src\hsl_safety\test -q
```

The current baseline is 30 pure-core tests and 29 ROS-free safety-boundary
tests. The Phase-1 report's combined command assumes an installed editable
package and a ROS overlay; it is therefore retained for the container/CI
profile, not used as a Windows checkout smoke test. A new Phase-2 report must
preserve the separation between pure software, ROS runtime and physical
response.

## 7. Immediate next steps

1. Freeze the timestamp convention and pose-history API for P2.2.
2. Implement `deskew_cloud` and its synthetic wall test. **Completed as P2.2
   SIL in the current revision; physical/replay residuals remain pending.**
3. Implement `LocalObstacleBuilder` and its negative coverage tests. **Completed
   as P2.3 SIL in the current revision; sensor/replay and physical coverage
   calibration remain pending.** The independent audit was rechecked: near/far
   ray ordering is now deterministic, while unknown forward space remains
   conservative instead of being promoted to the grid boundary.
4. Add dry-run calibration CLI and report schema; no motion publication in SIL.
   **Completed as P2.4 preparation; physical adapter remains blocked.**
5. Run targeted pure-core tests and write the two scoped Phase-2 reports.
6. Only on the robot: execute preflight, supervised braking trials and the
   hardware manifest; then evaluate G0/G1 with the evidence table above.
