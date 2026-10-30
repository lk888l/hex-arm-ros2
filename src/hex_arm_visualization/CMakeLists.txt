cmake_minimum_required(VERSION 3.16)
project(hex_arm_visualization VERSION 0.1.0 LANGUAGES NONE)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/motor_monitor DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(motor_monitor test/test_motor_monitor.py TIMEOUT 30)
  ament_add_pytest_test(motor_monitor_integration test/test_motor_monitor_integration.py TIMEOUT 60)
endif()

ament_package()
