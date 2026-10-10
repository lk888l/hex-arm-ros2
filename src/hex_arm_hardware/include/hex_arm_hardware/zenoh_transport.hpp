#pragma once

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/node.hpp"
#include "hex_arm_hardware/snapshot_mailbox.hpp"
#include "hex_arm_hardware/transport_trace.hpp"

namespace hex_arm_hardware
{
struct ZenohOptions
{
  std::string endpoint{"tcp/127.0.0.1:7448"};
  std::string prefix{"hexmeow/ubuntu/firefly_y6_meow"};
  std::vector<std::string> joint_names;
  double state_timeout{0.1}, command_max_age{0.05}, command_period{0.01};
  double startup_timeout{30.0}, service_timeout{5.0};
};

// Sole direct-backend owner. Its workers and callbacks perform all encoding,
// ROS diagnostics/tools, Zenoh operations and lifecycle transactions.
class ZenohTransport
{
public:
  ZenohTransport(const ZenohOptions &, const rclcpp::Node::SharedPtr &,
    SnapshotMailbox<FeedbackSnapshot> &, SnapshotMailbox<CommandSnapshot> &, TransportTrace &);
  ~ZenohTransport();
  bool activate(std::uint64_t generation);
  bool deactivate();
  bool allowed() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace hex_arm_hardware
