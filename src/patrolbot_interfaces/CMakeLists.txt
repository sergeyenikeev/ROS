cmake_minimum_required(VERSION 3.10)
project(patrolbot_interfaces)

# Интерфейсный пакет не содержит runtime-кода. Его задача — сгенерировать
# типы сообщений и сервисов, которыми будут обмениваться узлы PatrolBot.
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Генерация интерфейсов выполняется централизованно в одном месте, чтобы другие
# пакеты зависели уже от готового набора msg/srv, а не дублировали типы локально.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PatrolStatus.msg"
  "srv/StartPatrol.srv"
  "srv/StopPatrol.srv"
)

# Экспорт runtime-зависимости нужен соседним пакетам, которые используют уже
# сгенерированные интерфейсы после сборки.
ament_export_dependencies(rosidl_default_runtime)

ament_package()
