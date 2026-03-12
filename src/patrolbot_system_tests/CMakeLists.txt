cmake_minimum_required(VERSION 3.10)
project(patrolbot_system_tests)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
