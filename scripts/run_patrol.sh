#!/usr/bin/env bash
set -euo pipefail

# Скрипт запускает полный профиль патрулирования:
# карта + локализация + Nav2 + mission manager.
# Параметры:
#   $1 - путь к YAML карты;
#   $2 - путь к YAML-миссии;
#   $3 - использовать ли mock-аппаратуру;
#   $4 - уровень логирования.
WORKSPACE_DIR="/home/ubuntu/ROS"
MAP_FILE="${1:-/home/ubuntu/ROS/maps/patrolbot_map.yaml}"
MISSION_FILE="${2:-/home/ubuntu/ROS/src/patrolbot_mission_manager/config/patrol_route.yaml}"
USE_MOCK_HARDWARE="${3:-false}"
LOG_LEVEL="${4:-info}"

# Проверки входных файлов нужны до старта launch, чтобы ошибка была короткой и понятной.
if [[ ! -f "${MAP_FILE}" ]]; then
  echo "[ERROR] Файл карты не найден: ${MAP_FILE}"
  exit 1
fi

if [[ ! -f "${MISSION_FILE}" ]]; then
  echo "[ERROR] Файл маршрута не найден: ${MISSION_FILE}"
  exit 1
fi

cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

ros2 launch patrolbot_bringup patrol.launch.py \
  map:="${MAP_FILE}" \
  mission_file:="${MISSION_FILE}" \
  use_mock_hardware:="${USE_MOCK_HARDWARE}" \
  log_level:="${LOG_LEVEL}"
