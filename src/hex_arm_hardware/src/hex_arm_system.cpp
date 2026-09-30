#include "hex_arm_hardware/hex_arm_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hex_arm_hardware/joint_mapping.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace hex_arm_hardware
{
namespace
{

double parse_positive_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name, double fallback)
{
  const auto found = info.hardware_parameters.find(name);
  if (found == info.hardware_parameters.end()) {
    return fallback;
  }
  const double value = std::stod(found->second);
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be finite and positive");
  }
  return value;
}

std::string parameter_or(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & fallback)
{
  const auto found = info.hardware_parameters.find(name);
  return found == info.hardware_parameters.end() ? fallback : found->second;
}

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(interfaces.begin(), interfaces.end(), [&name](const auto & item) {
    return item.name == name;
  });
}

}  // namespace

HexArmSystem::~HexArmSystem()
{
  // ResourceManager can destroy a failed component without on_cleanup.
  // Remote disable belongs to lifecycle/supervisor handling; always reclaim
  // the local executor before std::thread's destructor is reached.
  active_.store(false);
  stop_io_thread();
}

hardware_interface::CallbackReturn HexArmSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    if (info_.joints.size() != 6U) {
      throw std::invalid_argument("Firefly Y6 requires exactly six joints");
    }
    for (const auto & joint : info_.joints) {
      if (!has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION) ||
        !has_interface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY) ||
        !has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
        !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY) ||
        !has_interface(joint.state_interfaces, hardware_interface::HW_IF_EFFORT))
      {
        throw std::invalid_argument("joint " + joint.name + " has an incomplete interface set");
      }
      joint_names_.push_back(joint.name);
    }

    command_topic_ = parameter_or(info_, "command_topic", command_topic_);
    state_topic_ = parameter_or(info_, "state_topic", state_topic_);
    activate_service_ = parameter_or(info_, "activate_service", activate_service_);
    deactivate_service_ = parameter_or(info_, "deactivate_service", deactivate_service_);
    state_timeout_ = std::chrono::duration<double>(
      parse_positive_parameter(info_, "state_timeout_sec", state_timeout_.count()));
    activation_timeout_ = std::chrono::duration<double>(
      parse_positive_parameter(info_, "activation_timeout_sec", activation_timeout_.count()));
    service_timeout_ = std::chrono::duration<double>(
      parse_positive_parameter(info_, "service_timeout_sec", service_timeout_.count()));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("HexArmSystem"), "configuration rejected: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto count = joint_names_.size();
  hw_position_.assign(count, 0.0);
  hw_velocity_.assign(count, 0.0);
  hw_effort_.assign(count, 0.0);
  command_position_.assign(count, 0.0);
  command_velocity_.assign(count, 0.0);
  pending_position_.assign(count, 0.0);
  pending_velocity_.assign(count, 0.0);
  pending_effort_.assign(count, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HexArmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 3U);
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_POSITION, &hw_position_[index]);
    interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_VELOCITY, &hw_velocity_[index]);
    interfaces.emplace_back(joint_names_[index], hardware_interface::HW_IF_EFFORT, &hw_effort_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> HexArmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2U);
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &command_position_[index]);
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_VELOCITY, &command_velocity_[index]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn HexArmSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Error recovery may return to UNCONFIGURED without on_cleanup. Join the
  // previous executor before replacing any ROS entity it can still access.
  if (active_.load()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  stop_io_thread();
  static std::atomic_uint instance{0U};
  const auto name = "hex_arm_system_io_" + std::to_string(instance.fetch_add(1U));
  io_node_ = std::make_shared<rclcpp::Node>(name);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(io_node_);

  state_subscription_ = io_node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, rclcpp::SensorDataQoS(),
    std::bind(&HexArmSystem::receive_state, this, std::placeholders::_1));
  command_endpoint_ = io_node_->create_publisher<sensor_msgs::msg::JointState>(
    command_topic_, rclcpp::QoS(1).reliable());
  command_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(command_endpoint_);
  activate_client_ = io_node_->create_client<std_srvs::srv::Trigger>(activate_service_);
  deactivate_client_ = io_node_->create_client<std_srvs::srv::Trigger>(deactivate_service_);
  stop_io_.store(false);
  executor_thread_ = std::thread([this]() {
      while (!stop_io_.load() && rclcpp::ok(io_node_->get_node_base_interface()->get_context())) {
        executor_->spin_once(std::chrono::milliseconds(20));
      }
    });
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexArmSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  stop_io_thread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexArmSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  const bool was_active = active_.exchange(false);
  bool stopped = true;
  if (was_active && io_node_ && rclcpp::ok(io_node_->get_node_base_interface()->get_context())) {
    stopped = call_safety_service(deactivate_client_, "shutdown", false);
  }
  stop_io_thread();
  return stopped ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn HexArmSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    const bool have_fresh_state = state_condition_.wait_for(
      lock, activation_timeout_, [this]() {
        // DDS matching must precede motor enable and the 100 ms watchdog.
        return command_endpoint_ && command_endpoint_->get_subscription_count() > 0 &&
               have_state_ &&
               (std::chrono::steady_clock::now() - last_state_time_) <= state_timeout_;
      });
    if (!have_fresh_state) {
      RCLCPP_ERROR(
        io_node_->get_logger(),
        "activation rejected: fresh six-joint feedback and matched command subscriber required within %.3f s",
        activation_timeout_.count());
      return hardware_interface::CallbackReturn::ERROR;
    }
    command_position_ = pending_position_;
    std::fill(command_velocity_.begin(), command_velocity_.end(), 0.0);
  }
  RCLCPP_INFO(io_node_->get_logger(), "Feedback fresh and command subscriber matched; requesting enable");
  if (!call_safety_service(activate_client_, "activate")) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  active_.store(true);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexArmSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  return call_safety_service(deactivate_client_, "deactivate", false) ?
         hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn HexArmSystem::on_error(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  (void)call_safety_service(deactivate_client_, "error stop", false);
  stop_io_thread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HexArmSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (active_.load() && !state_is_fresh()) {
    RCLCPP_ERROR_THROTTLE(
      io_node_->get_logger(), *io_node_->get_clock(), 1000,
      "feedback older than %.3f s", state_timeout_.count());
    return hardware_interface::return_type::ERROR;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (have_state_) {
    hw_position_ = pending_position_;
    hw_velocity_ = pending_velocity_;
    hw_effort_ = pending_effort_;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HexArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load()) {
    return hardware_interface::return_type::OK;
  }
  if (!std::all_of(command_position_.begin(), command_position_.end(), [](double value) {
      return std::isfinite(value);
    }) || !std::all_of(command_velocity_.begin(), command_velocity_.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    RCLCPP_ERROR(io_node_->get_logger(), "non-finite controller command rejected");
    return hardware_interface::return_type::ERROR;
  }

  if (command_publisher_ && command_publisher_->trylock()) {
    auto & message = command_publisher_->msg_;
    message.header.stamp = io_node_->now();
    message.name = joint_names_;
    message.position = command_position_;
    message.velocity = command_velocity_;
    message.effort.assign(joint_names_.size(), 0.0);
    command_publisher_->unlockAndPublish();
  }
  return hardware_interface::return_type::OK;
}

void HexArmSystem::receive_state(sensor_msgs::msg::JointState::ConstSharedPtr message)
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  if (!reorder_joint_state(*message, joint_names_, position, velocity, effort)) {
    RCLCPP_ERROR_THROTTLE(
      io_node_->get_logger(), *io_node_->get_clock(), 1000,
      "invalid or incomplete joint feedback rejected");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_position_ = std::move(position);
    pending_velocity_ = std::move(velocity);
    pending_effort_ = std::move(effort);
    last_state_time_ = std::chrono::steady_clock::now();
    have_state_ = true;
  }
  state_condition_.notify_all();
}

bool HexArmSystem::call_safety_service(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const std::string & operation, bool wait_for_discovery)
{
  if (!client || !io_node_) {
    return false;
  }
  const auto context = io_node_->get_node_base_interface()->get_context();
  const auto deadline = std::chrono::steady_clock::now() + service_timeout_;
  while (rclcpp::ok(context) && !client->service_is_ready()) {
    // Discovery belongs to activation. A vanished stop service cannot
    // acknowledge disable; fail promptly so the independent Rust owner and
    // supervisor can finish shutdown, including lifecycle error recovery.
    if (!wait_for_discovery || std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(io_node_->get_logger(), "%s service unavailable", operation.c_str());
      return false;
    }
    client->wait_for_service(std::chrono::milliseconds(20));
  }
  if (!rclcpp::ok(context)) {
    return false;
  }
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  while (future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
    // SIGINT stops the ROS executor as well. Waiting the full RPC deadline
    // here can deadlock controller-manager teardown until SIGKILL escalation.
    const bool service_gone = !client->service_is_ready();
    if (!rclcpp::ok(context) || service_gone || std::chrono::steady_clock::now() >= deadline) {
      client->remove_pending_request(future);
      if (rclcpp::ok(context)) {
        RCLCPP_ERROR(io_node_->get_logger(), "%s service %s", operation.c_str(),
          service_gone ? "disappeared while waiting" : "timed out");
      }
      return false;
    }
  }
  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(io_node_->get_logger(), "%s rejected: %s", operation.c_str(), response->message.c_str());
  }
  return response->success;
}

bool HexArmSystem::state_is_fresh() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return have_state_ && (std::chrono::steady_clock::now() - last_state_time_) <= state_timeout_;
}

void HexArmSystem::stop_io_thread()
{
  stop_io_.store(true);
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  if (executor_ && io_node_) {
    executor_->remove_node(io_node_);
  }
  deactivate_client_.reset();
  activate_client_.reset();
  command_publisher_.reset();
  command_endpoint_.reset();
  state_subscription_.reset();
  executor_.reset();
  io_node_.reset();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    have_state_ = false;
  }
  state_condition_.notify_all();
}

}  // namespace hex_arm_hardware

PLUGINLIB_EXPORT_CLASS(hex_arm_hardware::HexArmSystem, hardware_interface::SystemInterface)
