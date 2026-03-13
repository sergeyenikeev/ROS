cmake_minimum_required(VERSION 3.10)
project(patrolbot_mission_manager)

# Mission manager — отдельный прикладной слой, стоящий поверх Nav2.
# Как и в базе, логика разбивается на библиотеку и минимальный executable.
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(patrolbot_interfaces REQUIRED)
find_package(patrolbot_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# В библиотеке лежат и ROS-обвязка mission manager, и чистая state machine.
# Это позволяет тестировать автомат независимо от main.cpp.
add_library(${PROJECT_NAME}_lib
  src/mission_manager_node.cpp
  src/mission_state_machine.cpp
)

target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_lib PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  ament_index_cpp
  geometry_msgs
  nav2_msgs
  patrolbot_interfaces
  patrolbot_utils
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(patrolbot_mission_manager_node
  src/main.cpp
)

target_compile_features(patrolbot_mission_manager_node PUBLIC cxx_std_17)
target_compile_options(patrolbot_mission_manager_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(patrolbot_mission_manager_node ${PROJECT_NAME}_lib)
ament_target_dependencies(patrolbot_mission_manager_node
  ament_index_cpp
  geometry_msgs
  nav2_msgs
  patrolbot_interfaces
  patrolbot_utils
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
)

# Устанавливаем и библиотеку, и узел, и YAML-миссии, чтобы запускаться как из
# исходников, так и из install-space на Raspberry Pi.
install(TARGETS
  ${PROJECT_NAME}_lib
  patrolbot_mission_manager_node
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
  ament_index_cpp
  geometry_msgs
  nav2_msgs
  patrolbot_interfaces
  patrolbot_utils
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  # Здесь проверяется чистая прикладная логика конечного автомата миссии.
  # Для этого достаточно unit-тестов на gtest без подъёма полноценного Nav2.
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mission_state_machine
    test/test_mission_state_machine.cpp
  )
  target_link_libraries(test_mission_state_machine ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_mission_state_machine
    ament_index_cpp
    geometry_msgs
    nav2_msgs
    patrolbot_interfaces
    patrolbot_utils
    rclcpp
    rclcpp_action
    std_msgs
    tf2
    tf2_geometry_msgs
  )
endif()

ament_package()
