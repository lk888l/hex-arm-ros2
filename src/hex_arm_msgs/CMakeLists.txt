cmake_minimum_required(VERSION 3.16)
project(hex_arm_msgs VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriverState.msg"
  "msg/StateTiming.msg"
  "msg/InterpolationState.msg"
  "msg/ArmStatus.msg"
  "action/StopArm.action"
  "msg/MotorIdentity.msg"
  "srv/DiscoverMotors.srv"
  "srv/SetOperatingMode.srv"
  "srv/SetGravity.srv"
  DEPENDENCIES builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

