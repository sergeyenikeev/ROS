#!/usr/bin/env bash
set -euo pipefail

# Скрипт сохраняет текущую карту в каталог /home/ubuntu/ROS/maps.
WORKSPACE_DIR="/home/ubuntu/ROS"
MAP_BASENAME="${1:-patrolbot_map}"
TARGET_PATH="${WORKSPACE_DIR}/maps/${MAP_BASENAME}"

mkdir -p "${WORKSPACE_DIR}/maps"
cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

ros2 run nav2_map_server map_saver_cli -f "${TARGET_PATH}"
