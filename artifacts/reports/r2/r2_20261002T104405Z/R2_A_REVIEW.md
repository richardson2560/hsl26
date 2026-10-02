# R2-A review — start disarmed / fail closed

**Result:** technical PASS for the isolated no-motion subphase; R2 remains
`IN_PROGRESS`.

## Evidence reviewed

- Candidate image: `hsl26@sha256:750ec9f4f99de9b74007285f3cbdc42204830238bb12ddabfc12404c819567df`.
- `r2-unit-tests.txt`: 39 passed.
- `r2-launch.txt` and `r2-container-state.txt`: supervisor and watchdog started
  as separate processes and the isolated container remained running.
- `r2-zero-and-health.txt`: both `/hsl/cmd_vel_stop` and
  `/hsl/cmd_vel_final` were zero; the watchdog reported `healthy: true` and
  `stop_asserted: true` under `UNCONFIGURED_NO_MOTION`.
- `r2-rearm-denied.txt`: `RearmSafety` returned `accepted=False` with an
  explicit calibrated-downstream-evidence requirement.

## Limits of this result

This is not I01–I05 acceptance. No `cmd_vel_mux` or downstream controller was
running; no process-kill, timeout/lease-expiry, latency or physical-stop test
was measured. The profile intentionally contains no accepted calibration and
cannot admit motion. Any later candidate that changes the image or safety code
must repeat this evidence and the pending integration tests.
