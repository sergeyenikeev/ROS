#!/usr/bin/env bash
set -euo pipefail

# Скрипт проверяет форматирование C++ файлов через clang-format в режиме dry-run.
# Он не меняет файлы, а только валит запуск при несоответствии стилю.
WORKSPACE_DIR="/home/ubuntu/ROS"

cd "${WORKSPACE_DIR}"

if ! command -v clang-format >/dev/null 2>&1; then
  echo "[ERROR] clang-format не найден. Установите его через scripts/install_dependencies.sh"
  exit 1
fi

# Проверяем только исходники C/C++, потому что для launch, YAML и XML в проекте
# пока не настроен отдельный formatter с жёстким стилем.
find "${WORKSPACE_DIR}/src" \
  -type f \
  \( -name "*.hpp" -o -name "*.cpp" -o -name "*.h" -o -name "*.cxx" \) \
  -print0 | xargs -0 clang-format --dry-run --Werror

echo "[INFO] Проверка форматирования завершена успешно"
