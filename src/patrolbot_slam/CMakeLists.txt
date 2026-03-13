cmake_minimum_required(VERSION 3.10)
project(patrolbot_slam)

# Пакет SLAM на текущем этапе состоит только из конфигурации slam_toolbox.
# Отдельный пакет нужен, чтобы режим картографирования не смешивался с Nav2.
find_package(ament_cmake REQUIRED)

# Устанавливаем каталог config целиком, чтобы launch-профиль `slam` мог читать
# параметры из install-space без привязки к исходному дереву проекта.
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
