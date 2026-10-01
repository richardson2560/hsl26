#!/usr/bin/env bash
# Generated ROS setup scripts are not nounset-safe.
set -eo pipefail

test "${HSL26_EXECUTION_PROFILE:-}" = simulation || { echo 'MVSim requires HSL26_EXECUTION_PROFILE=simulation' >&2; exit 64; }
test "${HSL26_SIMULATION_ENABLED:-0}" = 1 || { echo 'Set HSL26_SIMULATION_ENABLED=1 to start MVSim' >&2; exit 64; }

source /opt/ros/humble/setup.bash
source /workspace_kobuki/install/setup.bash
source /workspace_hsl26/install/setup.bash
set -u

if [ "${HSL26_MVSIM_GUI:-0}" = 1 ]; then
    test "${HSL26_SIMULATION_DEVELOPMENT_GUI_ACK:-0}" = 1 || { echo 'GUI requires HSL26_SIMULATION_DEVELOPMENT_GUI_ACK=1' >&2; exit 64; }
    test -n "${DISPLAY:-}" || { echo 'GUI requires DISPLAY; use HSL26_MVSIM_GUI=0 for headless CI' >&2; exit 64; }
    exec ros2 launch mvsim demo_warehouse.launch.py headless:=False use_rviz:=True
fi

exec ros2 launch mvsim demo_warehouse.launch.py headless:=True use_rviz:=False
