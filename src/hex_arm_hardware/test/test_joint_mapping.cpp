#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "hex_arm_hardware/joint_mapping.hpp"

TEST(JointMapping, ReordersByName)
{
  sensor_msgs::msg::JointState message;
  message.name = {"joint_3", "joint_1", "joint_2"};
  message.position = {3.0, 1.0, 2.0};
  message.velocity = {30.0, 10.0, 20.0};
  message.effort = {300.0, 100.0, 200.0};
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> tau;
  ASSERT_TRUE(hex_arm_hardware::reorder_joint_state(
    message, {"joint_1", "joint_2", "joint_3"}, q, dq, tau));
  EXPECT_EQ(q, (std::vector<double>{1.0, 2.0, 3.0}));
  EXPECT_EQ(dq, (std::vector<double>{10.0, 20.0, 30.0}));
  EXPECT_EQ(tau, (std::vector<double>{100.0, 200.0, 300.0}));
}

TEST(JointMapping, RejectsMissingDuplicateAndNonFiniteValues)
{
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> tau;
  sensor_msgs::msg::JointState message;
  message.name = {"joint_1", "joint_1"};
  message.position = {0.0, 0.0};
  EXPECT_FALSE(hex_arm_hardware::reorder_joint_state(message, {"joint_1", "joint_2"}, q, dq, tau));
  message.name = {"joint_1", "joint_2"};
  message.position[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(hex_arm_hardware::reorder_joint_state(message, {"joint_1", "joint_2"}, q, dq, tau));
}

