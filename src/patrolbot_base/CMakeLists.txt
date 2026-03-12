cmake_minimum_required(VERSION 3.10)
project(patrolbot_base)

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

install(TARGETS
  ${PROJECT_NAME}_lib
  patrolbot_base_node
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

ament_package()
