cmake_minimum_required(VERSION 3.16)
project(hex_arm_bringup VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch rviz test worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()

