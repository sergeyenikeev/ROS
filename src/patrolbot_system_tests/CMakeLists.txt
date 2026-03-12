cmake_minimum_required(VERSION 3.10)
project(patrolbot_system_tests)

find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(mock_nav2_action_server
  src/mock_nav2_action_server.cpp
)

target_compile_features(mock_nav2_action_server PUBLIC cxx_std_17)
target_compile_options(mock_nav2_action_server PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(mock_nav2_action_server
  nav2_msgs
  rclcpp
  rclcpp_action
)

install(TARGETS
  mock_nav2_action_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/test_mock_bringup_launch.py)
  add_launch_test(test/test_slam_launch.py)
  add_launch_test(test/test_localization_launch.py)
  add_launch_test(test/test_navigation_launch.py)
  add_launch_test(test/test_patrol_success_launch.py)
  add_launch_test(test/test_patrol_retry_launch.py)
  add_launch_test(test/test_patrol_timeout_launch.py)
endif()

ament_package()
