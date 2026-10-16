#!/usr/bin/env bash
# Source this file; all Python overrides stay inside this shell.
CALIB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$CALIB_ROOT/install/setup.bash"
export PATH="$CALIB_ROOT/.venv/bin:$PATH"
export PYTHONNOUSERSITE=1
export ROS_LOG_DIR="$CALIB_ROOT/logs"
mkdir -p "$ROS_LOG_DIR"
