cmake_minimum_required(VERSION 3.10)
project(patrolbot_description)

# Этот пакет хранит только описание робота и связанные ресурсы визуализации.
# Компиляция здесь не требуется, нужен только корректный install в share.
find_package(ament_cmake REQUIRED)

# Устанавливаем launch, URDF/Xacro и RViz-профиль как данные пакета.
# Именно из install-space эти ресурсы читают остальные launch-файлы.
install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
