#!/usr/bin/env bash
set -euo pipefail

# Скрипт запускает только unit-тесты пакетов с чистой логикой.
WORKSPACE_DIR="/home/ubuntu/ROS"

cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

colcon test \
  --packages-select patrolbot_base patrolbot_utils patrolbot_mission_manager \
  --event-handlers console_direct+ \
  --return-code-on-test-failure

colcon test-result --verbose
