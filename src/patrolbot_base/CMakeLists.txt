cmake_minimum_required(VERSION 3.10)
project(patrolbot_base)

# Базовая платформа — один из центральных runtime-пакетов проекта.
# Здесь собирается библиотека с логикой базы и отдельный исполняемый файл,
# который лишь поднимает узел и передаёт управление в rclcpp::spin.
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(patrolbot_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Логика базы вынесена в библиотеку, чтобы:
# - unit-тесты могли линковаться с тем же кодом;
# - main.cpp оставался минимальным;
# - runtime-узел и тесты не дублировали одни и те же исходники.
add_library(${PROJECT_NAME}_lib
  src/base_node.cpp
  src/mock_motor_driver.cpp
  src/odometry_integrator.cpp
  src/serial_motor_driver.cpp
)

target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_lib PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  patrolbot_utils
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Исполняемый файл intentionally состоит только из main.cpp и линкуется с
# библиотекой. Так сопровождать пакет проще, чем держать весь runtime-код
# непосредственно внутри add_executable.
add_executable(patrolbot_base_node
  src/main.cpp
)

target_compile_features(patrolbot_base_node PUBLIC cxx_std_17)
target_compile_options(patrolbot_base_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(patrolbot_base_node ${PROJECT_NAME}_lib)
ament_target_dependencies(patrolbot_base_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  patrolbot_utils
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Устанавливаем и библиотеку, и runtime-узел. Путь lib/<package> для executable
# соответствует принятой схеме ROS 2 пакетов.
install(TARGETS
  ${PROJECT_NAME}_lib
  patrolbot_base_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Заголовки и конфиги нужны другим пакетам и launch-профилям после установки.
install(DIRECTORY include
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Экспорт делает библиотеку и её зависимости доступными для соседних пакетов,
# если в будущем они захотят переиспользовать код базы напрямую.
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  patrolbot_utils
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(BUILD_TESTING)
  # Внутри блока BUILD_TESTING подключаются только те инструменты, которые нужны
  # для unit-тестов. Так production-сборка не тянет лишние тестовые зависимости.
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_odometry_integrator
    test/test_odometry_integrator.cpp
  )
  target_link_libraries(test_odometry_integrator ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_odometry_integrator
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    patrolbot_utils
    rclcpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
  )
endif()

ament_package()
