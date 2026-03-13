#!/usr/bin/env bash
set -euo pipefail

# Скрипт подготавливает типовую машину Ubuntu 24.04 под PatrolBot:
# - подключает ROS 2 Jazzy;
# - ставит системные инструменты сборки;
# - устанавливает ROS-зависимости из apt;
# - затем дотягивает недостающее через rosdep по локальному workspace.
WORKSPACE_DIR="/home/ubuntu/ROS"

echo "[INFO] Устанавливаются системные пакеты и ROS 2 Jazzy для PatrolBot"

# Базовые утилиты нужны до подключения ROS-репозитория.
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release git
sudo add-apt-repository universe -y

# Ключ репозитория ROS кладётся в system keyring один раз.
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi

# Репозиторий ROS 2 добавляется только если ещё не был настроен ранее.
if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

sudo apt-get update

# Список пакетов intentionally задан явно, чтобы окружение PatrolBot было
# воспроизводимым, а не зависело от непредсказуемого набора метапакетов.
sudo apt-get install -y \
  build-essential \
  clang-format \
  cmake \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-desktop \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-launch-testing \
  ros-jazzy-launch-testing-ament-cmake \
  ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-waypoint-follower \
  ros-jazzy-rclcpp-action \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-slam-toolbox \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-xacro \
  ros-jazzy-yaml-cpp-vendor

# Скрипт намеренно проверяет наличие workspace до rosdep install, чтобы не
# запускать разрешение зависимостей по несуществующему пути.
if [[ ! -d "${WORKSPACE_DIR}" ]]; then
  echo "[ERROR] Рабочая директория ${WORKSPACE_DIR} не найдена"
  exit 1
fi

# rosdep требует подключённого базового ROS-окружения.
source /opt/ros/jazzy/setup.bash

# rosdep init может уже быть выполнен ранее, поэтому повтор не считаем ошибкой.
sudo rosdep init 2>/dev/null || true
rosdep update

# Эта команда добирает зависимости по реальному содержимому src/, а не по
# предположению скрипта о том, какие пакеты используются в проекте.
rosdep install --from-paths "${WORKSPACE_DIR}/src" --ignore-src -r -y

echo "[INFO] Установка зависимостей завершена"
