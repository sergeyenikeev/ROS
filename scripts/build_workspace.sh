#!/usr/bin/env bash
set -euo pipefail

# Скрипт собирает ROS 2 workspace PatrolBot.
WORKSPACE_DIR="/home/ubuntu/ROS"

if [[ ! -d "${WORKSPACE_DIR}" ]]; then
  echo "[ERROR] Рабочая директория ${WORKSPACE_DIR} не найдена"
  exit 1
fi

cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash

echo "[INFO] Запускается сборка PatrolBot"
colcon build --symlink-install --event-handlers console_direct+

echo "[INFO] Сборка завершена"
