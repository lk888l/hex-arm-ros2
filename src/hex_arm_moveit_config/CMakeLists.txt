cmake_minimum_required(VERSION 3.16)
project(hex_arm_moveit_config VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(moveit_config_contract test/test_moveit_config.py TIMEOUT 60)
  ament_add_pytest_test(self_collision_sampling test/test_self_collision_matrix.py TIMEOUT 60)
  ament_add_pytest_test(moveit_real_launch_contract test/test_moveit_real_launch.py TIMEOUT 60)
  ament_add_pytest_test(moveit_plan_only_collision test/test_moveit_plan_only.py TIMEOUT 90)
  ament_add_pytest_test(moveit_mock_integration test/test_moveit_mock.py TIMEOUT 90)
  set_tests_properties(moveit_plan_only_collision moveit_mock_integration
    PROPERTIES RUN_SERIAL TRUE)
endif()

ament_package()
