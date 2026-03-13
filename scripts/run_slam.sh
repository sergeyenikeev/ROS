#!/usr/bin/env bash
set -euo pipefail

# Скрипт запускает режим картографирования PatrolBot.
# Параметры:
#   $1 - использовать ли mock-аппаратуру;
#   $2 - уровень логирования ROS 2.
WORKSPACE_DIR="/home/ubuntu/ROS"
USE_MOCK_HARDWARE="${1:-false}"
LOG_LEVEL="${2:-info}"

cd "${WORKSPACE_DIR}"

# Для запуска нужен и системный ROS, и уже собранный локальный workspace.
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# Используем отдельный launch-профиль SLAM, который сам выберет mock или hardware.
ros2 launch patrolbot_bringup slam.launch.py \
  use_mock_hardware:="${USE_MOCK_HARDWARE}" \
  log_level:="${LOG_LEVEL}"
