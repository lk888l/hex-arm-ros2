#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace hex_arm_hardware
{
// ROS joint order is arbitrary. Device identities stay tied to calibration,
// independent of ROS prefixes and of the order in an enclosing robot model.
inline void validate_device_joints(
  const std::vector<std::string> & ros_names, const std::vector<std::string> & device_names)
{
  if (ros_names.size() != 6U || device_names.size() != 6U) {
    throw std::invalid_argument("exactly six ROS and device joint names are required");
  }
  auto sorted_ros = ros_names;
  std::sort(sorted_ros.begin(), sorted_ros.end());
  if (sorted_ros.front().empty() ||
    std::adjacent_find(sorted_ros.begin(), sorted_ros.end()) != sorted_ros.end())
  {
    throw std::invalid_argument("ROS joint names must be nonempty and unique");
  }
  auto sorted_device = device_names;
  std::sort(sorted_device.begin(), sorted_device.end());
  const std::vector<std::string> canonical{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  if (sorted_device != canonical) {
    throw std::invalid_argument("device_joint_name must map each of joint_1 through joint_6 exactly once");
  }
}

inline std::array<std::size_t, 6> wire_joint_mapping(
  const std::vector<std::string> & ros_names, const std::vector<std::string> & device_names,
  const std::vector<std::string> & wire_names)
{
  validate_device_joints(ros_names, device_names);
  validate_device_joints(wire_names, wire_names);
  std::array<std::size_t, 6> mapping{};
  for (std::size_t i = 0; i < mapping.size(); ++i) {
    mapping[i] = static_cast<std::size_t>(
      std::find(device_names.begin(), device_names.end(), wire_names[i]) - device_names.begin());
  }
  return mapping;
}

inline std::string io_node_name(const std::string & hardware_name, std::uint64_t instance)
{
  // Escape bytes, including underscore, to avoid punctuation collisions. The
  // ordinal also distinguishes repeated configurations/instances in one process.
  std::string name = "hex_arm_io_";
  constexpr char hex[] = "0123456789abcdef";
  for (const unsigned char c : hardware_name) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
      name += static_cast<char>(c);
    } else {
      name += '_'; name += hex[c >> 4]; name += hex[c & 15];
    }
  }
  return name + "_" + std::to_string(instance);
}
}  // namespace hex_arm_hardware
