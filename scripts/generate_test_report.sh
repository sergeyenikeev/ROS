#!/usr/bin/env bash
set -euo pipefail

# Скрипт собирает итоговый текстовый отчёт по уже выполненным тестам.
WORKSPACE_DIR="/home/ubuntu/ROS"
REPORT_DIR="${WORKSPACE_DIR}/test_results"
REPORT_FILE="${REPORT_DIR}/latest_test_report.txt"

# Каталог создаётся заранее, чтобы tee мог спокойно записать итоговый файл.
mkdir -p "${REPORT_DIR}"
cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# Отчёт одновременно печатается в консоль и сохраняется на диск.
colcon test-result --verbose | tee "${REPORT_FILE}"

echo "[INFO] Отчёт сохранён в ${REPORT_FILE}"
