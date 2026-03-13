#!/usr/bin/env bash
set -euo pipefail

# Скрипт собирает весь workspace PatrolBot из целевой директории развёртывания.
WORKSPACE_DIR="/home/ubuntu/ROS"

# Явная проверка каталога полезнее, чем неочевидный провал команды colcon дальше.
if [[ ! -d "${WORKSPACE_DIR}" ]]; then
  echo "[ERROR] Рабочая директория ${WORKSPACE_DIR} не найдена"
  exit 1
fi

cd "${WORKSPACE_DIR}"

# Сначала подключается системный ROS, потому что install/setup.bash проекта ещё
# может не существовать до первой успешной сборки.
source /opt/ros/jazzy/setup.bash

echo "[INFO] Запускается сборка PatrolBot"

# --symlink-install удобен в разработке: изменения в ресурсах и Python/launch
# файлах сразу видны из install-space без копирования.
colcon build --symlink-install --event-handlers console_direct+

echo "[INFO] Сборка завершена"
