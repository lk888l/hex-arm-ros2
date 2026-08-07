cmake_minimum_required(VERSION 3.16)
project(hex_arm_description VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description_contract test/test_description.py TIMEOUT 60)
endif()

ament_package()
