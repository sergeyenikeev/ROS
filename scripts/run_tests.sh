#!/usr/bin/env bash
set -euo pipefail

# Скрипт запускает полный набор unit и integration тестов.
WORKSPACE_DIR="/home/ubuntu/ROS"

cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
