#!/usr/bin/env bash
set -euo pipefail

# Скрипт запускает полный набор unit и integration тестов проекта.
WORKSPACE_DIR="/home/ubuntu/ROS"

cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# --return-code-on-test-failure позволяет использовать скрипт в CI без
# дополнительной ручной обработки кодов возврата.
colcon test --event-handlers console_direct+ --return-code-on-test-failure

# Итоговый отчёт печатается сразу после тестов, чтобы инженер не делал второй запуск.
colcon test-result --verbose
