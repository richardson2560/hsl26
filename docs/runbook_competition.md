# HSL26 Phase-1 Safety Runbook

**Revision:** 1.0  
**Date:** 2026-09-25  
**Authority:** `docs/HSL26_TECHNICAL_SPECIFICATION.md`, revision 2.2  
**Release status:** software candidate documented; real-motion release blocked

## 1. Scope and safety rule

This runbook covers the revision-2 command-containment boundary:

```text
/hsl/cmd_vel_stop       priority 200, zero-only watchdog input
/teleop/cmd_vel         priority 100, permitted test teleoperation
/hsl/cmd_vel_final      priority 50, autonomous supervisor output
                              |
                       cmd_vel_mux
                              |
                       /commands/velocity
```

Only `cmd_vel_mux` may publish `/commands/velocity`. A zero message received
by the mux is not evidence that the base physically stopped. Physical stop
timing requires an authorized hardware test and a recorded driver response.

The real profile stays disarmed unless the release manifest marks the driver
timeout, mux behavior, graph authority and physical response as passed.

## 2. Preflight

Run from the repository root:

```powershell
$env:PYTHONPATH = 'C:\path\to\hsl26\hsl_core;C:\path\to\hsl26\ros_ws\src\hsl_safety;C:\path\to\hsl26'
pytest -q hsl_core\tests ros_ws\src\hsl_safety\test
python -m compileall -q hsl_core\hsl_core ros_ws\src\hsl_safety\hsl_safety tools
git diff --check
```

In a sourced ROS 2 environment, additionally run:

```bash
colcon build --symlink-install --event-handlers console_direct+
colcon test --event-handlers console_direct+
ros2 topic info /commands/velocity --verbose
python3 tools/check_ros_graph_authority.py graph_publishers.json
```

The graph check must show exactly one publisher of `/commands/velocity`, owned
by `/cmd_vel_mux`. Any additional writer blocks autonomy.

## 3. Safe startup

1. Keep the physical base disabled or lifted until driver-timeout evidence is
   accepted.
2. Start the mux and verify its output is `/commands/velocity`.
3. Start the independent watchdog. It must report `DISARMED` and assert stop.
4. Start the supervisor. Missing watchdog health must produce a zero command.
5. Verify no nonzero command is emitted before fresh revision-2 records,
   authority identities, leases, epochs and a progressing decision heartbeat
   are present.
6. Verify the watchdog reaches `READY`, then `ACTIVE`, only after all release
   prerequisites hold.
7. During permitted testing, send only bounded commands through
   `/hsl/cmd_vel_final`; never publish directly to `/commands/velocity`.

## 4. Normal stop and emergency stop

For a normal stop, revoke the execution identity and wait for the supervisor
to publish zero. The watchdog remains authoritative until a new controlled
rearm.

For an emergency stop:

1. Publish only a zero `Twist` on `/hsl/cmd_vel_stop`.
2. Confirm the mux active source is `watchdog_stop`.
3. Confirm `/commands/velocity` is zero.
4. Latch the watchdog fault and stop all autonomous producers.
5. Do not release the stop channel because a heartbeat resumes.

If ROS communication is lost entirely, rely only on a separately measured
driver timeout. Do not treat silence as a measured physical stop.

## 5. Controlled rearm

Rearm is rejected unless all conditions below are true:

- the request has an authorized recovery reference;
- stage and configuration identity match;
- the fault cause is cleared;
- measured linear and angular motion are near zero;
- the configured zero-motion dwell is complete;
- supervisor and watchdog health leases are fresh;
- the previous execution identity is revoked;
- a fresh candidate and progressing decision sequence are available.

Acceptance of `RearmSafety` moves the watchdog to a validation state; it does
not grant nonzero motion. A new heartbeat cannot by itself rearm a latched
watchdog.

## 6. Fault response matrix

| Fault | Required response | Release condition |
|---|---|---|
| Missing or stale candidate | Supervisor zero; watchdog remains stop-authoritative | Fresh validated candidate |
| Missing watchdog health | Supervisor zero | Fresh mutual health |
| Supervisor exception or deadline miss | Supervisor zero; watchdog lease expires/latches | Authorized rearm |
| Watchdog process loss | Mux stop input/driver timeout must be verified independently | Process restored and controlled rearm |
| Mux process loss | Driver timeout must stop the base | Mux restored and graph revalidated |
| Future/expired record | Reject record; do not refresh observation age | New valid record |
| Clock/localization epoch change | Reject stale bound records and revoke motion | New coherent epoch data |
| Replay/non-progressing sequence | Reject and latch the watchdog where applicable | Authorized rearm |
| Unauthorized physical publisher | Keep real profile disarmed | Graph contains only mux publisher |

## 7. Evidence capture

For every run, preserve:

- commit SHA and dirty/clean state;
- image and dependency identities;
- configuration and limits hashes;
- exact commands;
- ROS graph output;
- supervisor/watchdog status traces;
- command request timestamps;
- driver physical response timestamps when hardware is authorized;
- fault injection and recovery result;
- accepted motion envelope.

Mock or kinematic evidence must never be labeled as hardware evidence.
