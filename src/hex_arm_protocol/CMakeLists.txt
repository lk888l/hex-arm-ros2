cmake_minimum_required(VERSION 3.16)
project(hex_arm_protocol VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
install(DIRECTORY proto DESTINATION share/${PROJECT_NAME})
ament_package(CONFIG_EXTRAS cmake/hex_arm_protocol-extras.cmake)
