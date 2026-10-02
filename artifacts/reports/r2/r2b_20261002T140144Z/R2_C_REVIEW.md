# R2-C review — supervisor process loss

**Result:** technical PASS for the isolated supervisor-loss case. R2 remains
`IN_PROGRESS`.

- `r2c-processes-after-supervisor-kill.txt` shows the supervisor absent while
  `cmd_vel_mux_node` and `safety_watchdog` remained alive.
- `r2c-stop-after-supervisor-kill.txt` recorded 61 zero physical-topic samples
  despite a `0.2 m/s` logical test input.
- `r2c-watchdog-health-after-supervisor-kill.txt` reports `healthy: false`,
  `stop_asserted: true` and `fault_reason: STALE_HEARTBEAT`.

This proves the watchdog detects a missing supervisor heartbeat and preserves
the logical stop. It does not prove behavior after the watchdog itself dies or
the physical controller loses authority.
