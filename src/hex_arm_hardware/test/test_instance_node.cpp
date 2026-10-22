#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hex_arm_hardware/instance_node.hpp"

TEST(InstanceNode, ExplicitNamespacesAndNamesSurviveControllerManagerGlobalRemaps)
{
  const char * arguments[] = {
    "test_instance_node", "--ros-args", "-r", "__node:=controller_manager", "-r", "__ns:=/whole_robot"};
  rclcpp::init(6, arguments);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  auto left = hex_arm_hardware::make_instance_node("Y6System", "/left", 0);
  auto right = hex_arm_hardware::make_instance_node("Y6System", "/right", 1);
  EXPECT_STREQ(left->get_namespace(), "/left");
  EXPECT_STREQ(right->get_namespace(), "/right");
  EXPECT_STRNE(left->get_name(), "controller_manager");
  EXPECT_STRNE(left->get_name(), right->get_name());
  auto left_state = left->create_publisher<sensor_msgs::msg::JointState>("hex_arm/internal/state", 1);
  auto right_state = right->create_publisher<sensor_msgs::msg::JointState>("hex_arm/internal/state", 1);
  EXPECT_STREQ(left_state->get_topic_name(), "/left/hex_arm/internal/state");
  EXPECT_STREQ(right_state->get_topic_name(), "/right/hex_arm/internal/state");
  auto standalone = hex_arm_hardware::make_instance_node("Y6System", "", 2);
  auto state = standalone->create_publisher<sensor_msgs::msg::JointState>("hex_arm/internal/state", 1);
  EXPECT_STREQ(state->get_topic_name(), "/hex_arm/internal/state");
}
