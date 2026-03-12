#!/usr/bin/env bash
set -euo pipefail

# Скрипт сохраняет итоговый отчёт по тестам в каталог test_results.
WORKSPACE_DIR="/home/ubuntu/ROS"
REPORT_DIR="${WORKSPACE_DIR}/test_results"
REPORT_FILE="${REPORT_DIR}/latest_test_report.txt"

mkdir -p "${REPORT_DIR}"
cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

colcon test-result --verbose | tee "${REPORT_FILE}"

echo "[INFO] Отчёт сохранён в ${REPORT_FILE}"
