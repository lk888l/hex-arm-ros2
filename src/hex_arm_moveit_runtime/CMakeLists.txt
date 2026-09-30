cmake_minimum_required(VERSION 3.16)
project(hex_arm_moveit_runtime VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# The destructor interposer uses pointers to private fields from the installed
# header. Refuse an unreviewed ABI instead of silently risking corrupt cleanup.
if(NOT moveit_ros_planning_VERSION VERSION_EQUAL "2.12.4")
  message(FATAL_ERROR
    "hex_arm_moveit_runtime is validated only with MoveIt 2.12.4; found "
    "${moveit_ros_planning_VERSION}. Re-audit the upstream shutdown bug and "
    "remove or update this compatibility package before building.")
endif()
if(NOT rclcpp_VERSION VERSION_EQUAL "28.1.21")
  message(FATAL_ERROR
    "hex_arm_moveit_runtime is validated only with rclcpp 28.1.21; found "
    "${rclcpp_VERSION}. Re-audit the callback-group shutdown workaround before "
    "building against a different rclcpp release.")
endif()

add_executable(hex_arm_move_group src/hex_arm_move_group.cpp)
target_compile_features(hex_arm_move_group PRIVATE cxx_std_17)
ament_target_dependencies(hex_arm_move_group
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning
  pluginlib
  rclcpp
  std_msgs
  tf2_ros)

add_library(hex_arm_moveit_tem_shutdown SHARED src/tem_shutdown_interpose.cpp)
target_compile_features(hex_arm_moveit_tem_shutdown PRIVATE cxx_std_17)
ament_target_dependencies(hex_arm_moveit_tem_shutdown
  moveit_ros_planning
  rclcpp)
target_link_libraries(hex_arm_moveit_tem_shutdown ${CMAKE_DL_LIBS})

install(TARGETS
  hex_arm_move_group
  hex_arm_moveit_tem_shutdown
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib)

ament_package()
