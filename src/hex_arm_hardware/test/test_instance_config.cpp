#include <gtest/gtest.h>
#include "hex_arm_hardware/instance_config.hpp"
#include "hex_arm_hardware/disable_receipt.hpp"
#include "hex_arm_hardware/snapshot_mailbox.hpp"

using namespace hex_arm_hardware;

TEST(InstanceConfig, PrefixedReorderedJointsKeepDeviceCalibrationIdentity)
{
  const std::vector<std::string> ros{
    "left_joint_6", "left_joint_1", "left_joint_4", "left_joint_2", "left_joint_5", "left_joint_3"};
  const std::vector<std::string> devices{
    "joint_6", "joint_1", "joint_4", "joint_2", "joint_5", "joint_3"};
  const std::vector<std::string> wire{
    "joint_3", "joint_2", "joint_1", "joint_6", "joint_5", "joint_4"};
  const auto mapping = wire_joint_mapping(ros, devices, wire);
  const std::array<std::size_t, 6> expected{5, 3, 1, 0, 4, 2};
  EXPECT_EQ(mapping, expected);
  // The same permutation maps measured positions into ROS order and desired
  // positions back to the correct motors, including arbitrary wire ordering.
  std::array<double, 6> positions{};
  for (std::size_t i = 0; i < 6; ++i) {positions[mapping[i]] = 10.0 + i;}
  for (std::size_t i = 0; i < 6; ++i) {EXPECT_EQ(positions[mapping[i]], 10.0 + i);}
}

TEST(InstanceConfig, RejectsIncompleteOrAmbiguousJointMappings)
{
  const std::vector<std::string> names{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  auto duplicate = names; duplicate[5] = "joint_1";
  EXPECT_THROW(validate_device_joints(names, duplicate), std::invalid_argument);
  EXPECT_THROW(validate_device_joints(duplicate, names), std::invalid_argument);
  auto prefixed = names; prefixed[0] = "left_joint_1";
  EXPECT_THROW(validate_device_joints(prefixed, prefixed), std::invalid_argument);
  EXPECT_NO_THROW(validate_device_joints(prefixed, names));
  auto missing = names; missing.pop_back();
  EXPECT_THROW(wire_joint_mapping(names, names, missing), std::invalid_argument);
}

TEST(InstanceConfig, HardwareNodeNamesStayDistinctAndRosCompatible)
{
  EXPECT_NE(io_node_name("left-arm", 0), io_node_name("left_arm", 0));
  EXPECT_NE(io_node_name("left-arm", 0), io_node_name("left_2darm", 0));
  EXPECT_NE(io_node_name("Arm", 0), io_node_name("Arm", 1));
  for (const auto c : io_node_name("1.left/arm", 2)) {
    EXPECT_TRUE((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
      (c >= '0' && c <= '9') || c == '_');
  }
}

TEST(FeedbackPublication, CachedSamplesCannotBecomeNewMeasurements)
{
  FeedbackPublication publication;
  FeedbackSnapshot sample;
  sample.received_ns = 100; sample.source_stamp_ns = 10; sample.received_ros_ns = 5000;
  EXPECT_TRUE(publication.accept(sample, 110, 50));
  EXPECT_FALSE(publication.accept(sample, 120, 50));
  EXPECT_EQ(sample.received_ros_ns, 5000);
  ++sample.source_stamp_ns; sample.received_ns = 120;
  // A ROS clock jump does not alter freshness checks or suppress a new sample.
  sample.received_ros_ns = 1;
  EXPECT_TRUE(publication.accept(sample, 130, 50));
  ++sample.source_stamp_ns;
  EXPECT_FALSE(publication.accept(sample, 180, 50));
  EXPECT_FALSE(publication.accept(sample, 110, 50));
  sample.received_ns = 181;
  EXPECT_TRUE(publication.accept(sample, 190, 50));
}

TEST(DisableReceipt, FaultWithoutSessionIsNotAnAcknowledgedDisable)
{
  DisableReceipt receipt;
  // Rust shutdown may clear its session before backend disable has succeeded.
  EXPECT_FALSE(receipt.confirms(true, true, 0, false));
  receipt.acknowledge(7);
  EXPECT_TRUE(receipt.confirms(true, true, 0, false));
  // Terminal damping keeps its session; successful fault-preserving release
  // is also valid once the actual RPC and fresh inactive state are confirmed.
  EXPECT_TRUE(receipt.confirms(true, true, 7, false));
  EXPECT_FALSE(receipt.confirms(true, true, 8, false));
  EXPECT_FALSE(receipt.confirms(false, true, 0, false));
  EXPECT_FALSE(receipt.confirms(true, false, 7, false));
  EXPECT_FALSE(receipt.confirms(true, true, 0, true));
  receipt.observe_holder(8);
  EXPECT_FALSE(receipt.confirms(true, true, 0, false));
  receipt.acknowledge(9);
  receipt.invalidate();
  EXPECT_FALSE(receipt.confirms(true, true, 0, false));
}
