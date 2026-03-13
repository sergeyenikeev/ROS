cmake_minimum_required(VERSION 3.10)
project(patrolbot_navigation)

# Навигационный пакет содержит только конфиги Nav2, AMCL и тестовую карту.
# Он не компилируется, но должен корректно устанавливаться в share.
find_package(ament_cmake REQUIRED)

# Устанавливаем и параметры навигации, и тестовую карту, потому что launch-цепочка
# использует их напрямую из install-space.
install(DIRECTORY config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
