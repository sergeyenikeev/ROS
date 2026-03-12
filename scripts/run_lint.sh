#!/usr/bin/env bash
set -euo pipefail

# Скрипт проверяет форматирование C++ файлов через clang-format в режиме без изменений.
WORKSPACE_DIR="/home/ubuntu/ROS"

cd "${WORKSPACE_DIR}"

if ! command -v clang-format >/dev/null 2>&1; then
  echo "[ERROR] clang-format не найден. Установите его через scripts/install_dependencies.sh"
  exit 1
fi

find "${WORKSPACE_DIR}/src" \
  -type f \
  \( -name "*.hpp" -o -name "*.cpp" -o -name "*.h" -o -name "*.cxx" \) \
  -print0 | xargs -0 clang-format --dry-run --Werror

echo "[INFO] Проверка форматирования завершена успешно"
