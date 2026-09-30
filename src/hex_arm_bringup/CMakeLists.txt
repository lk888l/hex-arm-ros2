cmake_minimum_required(VERSION 3.16)
project(hex_arm_bringup VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS ../../scripts/commission-startup-ros.py
  ../../scripts/commission-meow-ros.py ../../scripts/commission-shutdown-ros.py ../../scripts/graceful-real-launch.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

if(BUILD_TESTING)
  install(DIRECTORY test DESTINATION share/${PROJECT_NAME}
    PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(meow_client test/test_meow_client.py TIMEOUT 30)
  ament_add_pytest_test(startup_client test/test_startup_client.py TIMEOUT 30)
  ament_add_pytest_test(startup_recipe test/test_startup_recipe.py TIMEOUT 30)
  ament_add_pytest_test(controller_constraints test/test_controller_constraints.py TIMEOUT 30)
  ament_add_pytest_test(real_launch_safety test/test_real_launch_safety.py TIMEOUT 30)
  ament_add_pytest_test(gravity_comp_launch test/test_gravity_comp_launch.py TIMEOUT 30)
  ament_add_pytest_test(gravity_comp_launch_runtime test/test_gravity_comp_launch_runtime.py TIMEOUT 90)
  ament_add_pytest_test(view_gui test/test_view_gui.py TIMEOUT 30)
  ament_add_pytest_test(view_isolation test/test_view_isolation.py TIMEOUT 60)
  ament_add_pytest_test(graceful_exit test/test_graceful_exit.py TIMEOUT 60)
  ament_add_pytest_test(shutdown_client test/test_shutdown_client.py TIMEOUT 30)
endif()

ament_package()
