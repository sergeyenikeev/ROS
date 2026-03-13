cmake_minimum_required(VERSION 3.10)
project(patrolbot_bringup)

# Пакет bringup не собирает C++ код и не генерирует интерфейсы.
# Его задача — доставить launch-файлы в install-space, чтобы они были доступны
# через `ros2 launch` после сборки workspace.
find_package(ament_cmake REQUIRED)

# Устанавливаем весь каталог launch целиком, потому что профили запуска являются
# основным содержимым этого пакета и должны попадать в share/<package>.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
