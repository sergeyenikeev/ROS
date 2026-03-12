#!/usr/bin/env bash
set -euo pipefail

# Скрипт запускает локализацию PatrolBot на сохранённой карте.
WORKSPACE_DIR="/home/ubuntu/ROS"
MAP_FILE="${1:-/home/ubuntu/ROS/maps/patrolbot_map.yaml}"
USE_MOCK_HARDWARE="${2:-false}"
LOG_LEVEL="${3:-info}"

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "[ERROR] Файл карты не найден: ${MAP_FILE}"
  exit 1
fi

cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

ros2 launch patrolbot_bringup localization.launch.py \
  map:="${MAP_FILE}" \
  use_mock_hardware:="${USE_MOCK_HARDWARE}" \
  log_level:="${LOG_LEVEL}"
