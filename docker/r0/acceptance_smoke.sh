#!/usr/bin/env bash
# ROS Humble's generated setup scripts read optional variables before defining
# them, so they cannot be sourced under `set -u`.
set -eo pipefail

source /opt/ros/humble/setup.bash
source /workspace_kobuki/install/setup.bash
source /workspace_hsl26/install/setup.bash
set -u

python3 -c "import hsl_core; import rclpy; print('acceptance environment OK')"

for forbidden in kobuki kobuki_node livox livox_ros2_driver_node mvsim cmd_vel_mux; do
    if pgrep -x "$forbidden" > /dev/null; then
        echo "Proceso prohibido detectado: $forbidden" >&2
        exit 1
    fi
done
