# R2-B review — watchdog stop priority through mux

**Result:** technical PASS for isolated mux priority. R2 remains
`IN_PROGRESS`.

## Evidence reviewed

- Candidate image: `hsl26@sha256:6a16bc85fee5950f5f864e3f6534a8b97073c32fb79f4fbbd57bd3540d0f7a68`.
- `r2b-processes.txt`: the mux, supervisor and watchdog ran as distinct
  processes in the test container.
- `r2b-stop-priority.txt`: with `HSL26_TEST_ONLY=1` and an explicit
  `--allow-test-nonzero`, a logical `/hsl/cmd_vel_final` command of `0.2 m/s`
  was injected; all 61 observed physical-topic samples remained zero.
- `r2b-physical-writer.txt`: exactly one publisher existed on
  `/commands/velocity`, `/cmd_vel_mux`, using reliable/volatile QoS.

## Scope limit

No Kobuki driver, device, MVSim or external network was attached. This proves
logical mux priority only; it is not a measured physical stop, downstream
timeout, watchdog-kill or release authorization.
