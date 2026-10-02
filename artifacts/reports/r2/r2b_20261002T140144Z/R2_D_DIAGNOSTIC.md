# R2-D diagnostic — watchdog process loss

**Result:** expected diagnostic failure; blocks physical safety acceptance.

`r2d-processes-after-watchdog-kill.txt` confirms that the watchdog and
supervisor were absent while `cmd_vel_mux_node` remained alive. With the
test-only, explicit `0.2 m/s` logical input, the probe recorded:

```text
FAIL: watchdog stop did not dominate mux output: (0.2, 0.0, 0.0, 0.0, 0.0, 0.0)
```

This is expected from a priority mux once its high-priority stop source times
out, but it means that ROS mux priority is not an independent stop mechanism.
No driver, device, MVSim actuator or physical robot was connected during this
diagnostic. Do not interpret it as a test failure to suppress or as permission
to operate hardware.

## Required closure before physical acceptance

Specify and validate an independent downstream interlock/controller timeout
whose default on watchdog loss is zero output, then measure its latency and
test loss of supervisor, watchdog, mux and controller on the target hardware.
