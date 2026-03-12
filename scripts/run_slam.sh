#!/usr/bin/env bash
set -euo pipefail

# Скрипт запускает режим картографирования PatrolBot.
WORKSPACE_DIR="/home/ubuntu/ROS"
USE_MOCK_HARDWARE="${1:-false}"
LOG_LEVEL="${2:-info}"

cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

ros2 launch patrolbot_bringup slam.launch.py \
  use_mock_hardware:="${USE_MOCK_HARDWARE}" \
  log_level:="${LOG_LEVEL}"
