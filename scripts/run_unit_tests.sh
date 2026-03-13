#!/usr/bin/env bash
set -euo pipefail

# Скрипт запускает только unit-тесты пакетов с чистой логикой, без launch-testing.
WORKSPACE_DIR="/home/ubuntu/ROS"

cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# Здесь intentionally выбираются только пакеты, где unit-тесты не требуют
# поднятия полной ROS-среды навигации.
colcon test \
  --packages-select patrolbot_base patrolbot_utils patrolbot_mission_manager \
  --event-handlers console_direct+ \
  --return-code-on-test-failure

colcon test-result --verbose
