#!/usr/bin/env bash
set -euo pipefail

# Скрипт сохраняет текущую карту в каталог /home/ubuntu/ROS/maps.
# Параметр $1 задаёт базовое имя без расширения.
WORKSPACE_DIR="/home/ubuntu/ROS"
MAP_BASENAME="${1:-patrolbot_map}"
TARGET_PATH="${WORKSPACE_DIR}/maps/${MAP_BASENAME}"

# Каталог создаётся заранее, чтобы map_saver_cli не падал на отсутствующем пути.
mkdir -p "${WORKSPACE_DIR}/maps"
cd "${WORKSPACE_DIR}"

source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# map_saver_cli сам создаст .yaml и .pgm рядом с указанным базовым именем.
ros2 run nav2_map_server map_saver_cli -f "${TARGET_PATH}"
