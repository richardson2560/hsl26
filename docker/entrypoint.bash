#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspace_kobuki/install/setup.bash
source /workspace_hsl26/install/setup.bash

exec "$@"
