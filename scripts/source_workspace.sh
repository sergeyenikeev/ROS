#!/usr/bin/env bash
set -euo pipefail

# Скрипт должен вызываться через source, иначе переменные окружения не сохранятся
# в текущем shell и пользователь не увидит эффекта от подключения workspace.
WORKSPACE_DIR="/home/ubuntu/ROS"

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[ERROR] Этот скрипт нужно запускать так: source /home/ubuntu/ROS/scripts/source_workspace.sh"
  exit 1
fi

# Базовое ROS-окружение подключается всегда.
source /opt/ros/jazzy/setup.bash

# Локальный install/setup.bash подключается только после сборки workspace.
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

# Отдельная переменная удобна для пользовательских скриптов и ручной отладки.
export PATROLBOT_WORKSPACE_DIR="${WORKSPACE_DIR}"
echo "[INFO] Окружение PatrolBot подключено из ${WORKSPACE_DIR}"
