#pragma once

#include "rclcpp/node.hpp"
#include "hex_arm_hardware/instance_config.hpp"

namespace hex_arm_hardware
{
inline rclcpp::Node::SharedPtr make_instance_node(
  const std::string & hardware_name, const std::string & ros_namespace, std::uint64_t instance)
{
  // A controller_manager launch may globally remap __node and __ns. The
  // hardware node uses its explicit instance configuration in all embeddings.
  return std::make_shared<rclcpp::Node>(io_node_name(hardware_name, instance), ros_namespace,
    rclcpp::NodeOptions().use_global_arguments(false));
}
}  // namespace hex_arm_hardware
