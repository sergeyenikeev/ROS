#!/usr/bin/env bash
set -euo pipefail

# Скрипт должен вызываться через source, иначе переменные окружения не сохранятся в текущем shell.
WORKSPACE_DIR="/home/ubuntu/ROS"

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[ERROR] Этот скрипт нужно запускать так: source /home/ubuntu/ROS/scripts/source_workspace.sh"
  exit 1
fi

source /opt/ros/jazzy/setup.bash

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

export PATROLBOT_WORKSPACE_DIR="${WORKSPACE_DIR}"
echo "[INFO] Окружение PatrolBot подключено из ${WORKSPACE_DIR}"
