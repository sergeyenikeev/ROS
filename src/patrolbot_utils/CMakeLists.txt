cmake_minimum_required(VERSION 3.10)
project(patrolbot_utils)

# Utils-пакет собирает общие переиспользуемые компоненты: логирование,
# диагностику, валидацию параметров, загрузку маршрутов и mock LiDAR.
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Основные утилиты вынесены в библиотеку, чтобы их могли использовать и runtime
# пакеты, и unit-тесты, и отдельные вспомогательные исполняемые файлы.
add_library(${PROJECT_NAME}
  src/diagnostic_helper.cpp
  src/logging_utils.cpp
  src/parameter_validation.cpp
  src/route_loader.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# Mock-сканер собирается как отдельный узел, но переиспользует общие utils.
add_executable(mock_scan_publisher
  src/mock_scan_publisher.cpp
)

target_compile_features(mock_scan_publisher PUBLIC cxx_std_17)
target_compile_options(mock_scan_publisher PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mock_scan_publisher ${PROJECT_NAME})
ament_target_dependencies(mock_scan_publisher
  rclcpp
  sensor_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  mock_scan_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Заголовки и конфиг mock LiDAR должны быть доступны остальным пакетам и launch.
install(DIRECTORY include
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Экспортируем библиотеку и зависимости, чтобы соседние пакеты могли линковаться
# с utils без ручного повторения зависимостей.
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(diagnostic_msgs geometry_msgs rclcpp sensor_msgs yaml_cpp_vendor)

if(BUILD_TESTING)
  # Здесь проверяется чистая логика утилит: валидация параметров и загрузка YAML.
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_parameter_validation
    test/test_parameter_validation.cpp
  )
  target_link_libraries(test_parameter_validation ${PROJECT_NAME})
  ament_target_dependencies(test_parameter_validation
    diagnostic_msgs
    geometry_msgs
    rclcpp
    sensor_msgs
  )

  ament_add_gtest(test_route_loader
    test/test_route_loader.cpp
  )
  target_link_libraries(test_route_loader ${PROJECT_NAME})
  ament_target_dependencies(test_route_loader
    diagnostic_msgs
    geometry_msgs
    rclcpp
    sensor_msgs
  )
endif()

ament_package()
