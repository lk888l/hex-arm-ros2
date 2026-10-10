#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "hex_arm_hardware/transport_trace.hpp"
#include "hex_arm_hardware/snapshot_mailbox.hpp"
#include "hex_arm_hardware/zenoh_transport.hpp"

namespace hex_arm_hardware
{

class HexArmSystem final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HexArmSystem)
  ~HexArmSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool state_is_fresh() const;
  void stop_io_thread();

  std::vector<std::string> joint_names_;
  std::vector<double> hw_position_;
  std::vector<double> hw_velocity_;
  std::vector<double> hw_effort_;
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;

  SnapshotMailbox<FeedbackSnapshot> feedback_mailbox_;
  SnapshotMailbox<CommandSnapshot> command_mailbox_;
  FeedbackSnapshot read_snapshot_;
  ZenohOptions zenoh_options_;
  std::unique_ptr<ZenohTransport> direct_;

  std::chrono::duration<double> state_timeout_{0.1};
  std::chrono::duration<double> activation_timeout_{5.0};
  std::chrono::duration<double> service_timeout_{5.0};

  rclcpp::Node::SharedPtr io_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  std::atomic_bool stop_io_{false};
  std::atomic_bool active_{false};
  std::uint64_t command_sequence_{0};
  std::atomic<std::uint64_t> activation_generation_{0};
  TransportTrace trace_;
};

}  // namespace hex_arm_hardware
