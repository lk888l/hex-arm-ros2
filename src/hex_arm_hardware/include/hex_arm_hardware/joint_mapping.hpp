#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

namespace hex_arm_hardware
{

inline bool reorder_joint_state(
  const sensor_msgs::msg::JointState & message,
  const std::vector<std::string> & expected_names,
  std::vector<double> & position,
  std::vector<double> & velocity,
  std::vector<double> & effort)
{
  if (message.name.size() != expected_names.size() ||
    message.position.size() != message.name.size())
  {
    return false;
  }

  std::unordered_map<std::string, std::size_t> source;
  for (std::size_t index = 0; index < message.name.size(); ++index) {
    if (!source.emplace(message.name[index], index).second) {
      return false;
    }
  }

  position.resize(expected_names.size());
  velocity.assign(expected_names.size(), 0.0);
  effort.assign(expected_names.size(), 0.0);
  for (std::size_t target = 0; target < expected_names.size(); ++target) {
    const auto found = source.find(expected_names[target]);
    if (found == source.end()) {
      return false;
    }
    const auto index = found->second;
    const double q = message.position[index];
    const double dq = index < message.velocity.size() ? message.velocity[index] : 0.0;
    const double tau = index < message.effort.size() ? message.effort[index] : 0.0;
    if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(tau)) {
      return false;
    }
    position[target] = q;
    velocity[target] = dq;
    effort[target] = tau;
  }
  return true;
}

}  // namespace hex_arm_hardware

