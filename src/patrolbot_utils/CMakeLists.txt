cmake_minimum_required(VERSION 3.10)
project(patrolbot_utils)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/diagnostic_helper.cpp
  src/logging_utils.cpp
  src/parameter_validation.cpp
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

install(DIRECTORY include
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(diagnostic_msgs geometry_msgs rclcpp sensor_msgs)

ament_package()
