#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "hex_arm_hardware/transport_trace.hpp"

namespace hex_arm_hardware
{

class HexArmSystem final : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HexArmSystem)
  ~HexArmSystem() override;
  std::uint64_t skipped_command_publications() const
  {return skipped_command_publications_.load(std::memory_order_relaxed);}

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
  void receive_state(sensor_msgs::msg::JointState::ConstSharedPtr message);
  bool call_safety_service(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & operation, bool wait_for_discovery = true);
  bool state_is_fresh() const;
  void stop_io_thread();

  std::vector<std::string> joint_names_;
  std::vector<double> hw_position_;
  std::vector<double> hw_velocity_;
  std::vector<double> hw_effort_;
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;

  mutable std::mutex state_mutex_;
  std::condition_variable state_condition_;
  std::vector<double> pending_position_;
  std::vector<double> pending_velocity_;
  std::vector<double> pending_effort_;
  // Trace-only identity, updated with the same lock as accepted feedback.
  std::int64_t pending_state_stamp_ns_{0};
  std::chrono::steady_clock::time_point last_state_time_{};
  bool have_state_{false};

  std::string command_topic_{"/hex_arm/internal/command"};
  std::string state_topic_{"/hex_arm/internal/state"};
  std::string activate_service_{"/hex_arm_bridge/activate_hardware"};
  std::string deactivate_service_{"/hex_arm_bridge/deactivate_hardware"};
  std::chrono::duration<double> state_timeout_{0.1};
  std::chrono::duration<double> activation_timeout_{5.0};
  std::chrono::duration<double> service_timeout_{5.0};

  rclcpp::Node::SharedPtr io_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  std::atomic_bool stop_io_{false};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_endpoint_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> command_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr activate_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr deactivate_client_;
  std::atomic_bool active_{false};
  std::atomic<std::uint64_t> skipped_command_publications_{0};
  std::uint64_t command_sequence_{0};
  std::atomic<std::uint64_t> activation_generation_{0};
  TransportTrace trace_;
};

}  // namespace hex_arm_hardware
