cmake_minimum_required(VERSION 3.10)
project(patrolbot_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
