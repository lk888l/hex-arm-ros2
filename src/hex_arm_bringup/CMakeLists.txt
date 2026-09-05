cmake_minimum_required(VERSION 3.16)
project(hex_arm_bringup VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch rviz test worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(controller_constraints test/test_controller_constraints.py TIMEOUT 30)
  ament_add_pytest_test(real_launch_safety test/test_real_launch_safety.py TIMEOUT 30)
endif()

ament_package()
