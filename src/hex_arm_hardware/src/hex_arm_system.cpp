#include "hex_arm_hardware/hex_arm_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
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

    zenoh_options_.endpoint = parameter_or(info_, "zenoh_connect", zenoh_options_.endpoint);
    zenoh_options_.prefix = parameter_or(info_, "robot_prefix", zenoh_options_.prefix);
    zenoh_options_.command_period = parse_positive_parameter(info_, "command_period_sec", 0.01);
    zenoh_options_.command_max_age = parse_positive_parameter(info_, "command_max_age_sec", 0.05);
    zenoh_options_.startup_timeout = parse_positive_parameter(info_, "startup_timeout_sec", 30.0);
    if (zenoh_options_.command_max_age > 0.05) {
      throw std::invalid_argument("command_max_age_sec must not exceed 0.05 s");
    }
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
  zenoh_options_.joint_names = joint_names_;
  zenoh_options_.state_timeout = state_timeout_.count();
  zenoh_options_.service_timeout = service_timeout_.count();
  RCLCPP_INFO(rclcpp::get_logger("HexArmSystem"), "communication: direct Zenoh");
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

  try {
    trace_.start();
    direct_ = std::make_unique<ZenohTransport>(zenoh_options_, io_node_,
      feedback_mailbox_, command_mailbox_, trace_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(io_node_->get_logger(), "transport configuration failed: %s", error.what());
    stop_io_thread();
    return hardware_interface::CallbackReturn::ERROR;
  }
  stop_io_.store(false);
  executor_thread_ = std::thread([this]() {
      while (!stop_io_.load() && rclcpp::ok(io_node_->get_node_base_interface()->get_context())) {
        executor_->spin_once(std::chrono::milliseconds(1));
      }
    });
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexArmSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  const bool stopped = !direct_ || direct_->deactivate();
  stop_io_thread();
  return stopped ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn HexArmSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  const bool stopped = !direct_ || direct_->deactivate();
  stop_io_thread();
  return stopped ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn HexArmSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  FeedbackSnapshot snapshot;
  const auto deadline = std::chrono::steady_clock::now() + activation_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    if (feedback_mailbox_.load(snapshot) && snapshot.received_ns &&
      static_cast<double>(monotonic_ns() - snapshot.received_ns) * 1e-9 <= state_timeout_.count()) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (!direct_ || !snapshot.received_ns || !state_is_fresh())
  {
    RCLCPP_ERROR(io_node_->get_logger(), "activation requires fresh feedback and command transport");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::copy(snapshot.position.begin(), snapshot.position.end(), command_position_.begin());
  std::fill(command_velocity_.begin(), command_velocity_.end(), 0.0);
  const auto generation = activation_generation_.fetch_add(1) + 1;
  if (!direct_->activate(generation)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  active_.store(true);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HexArmSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  return (!direct_ || direct_->deactivate()) ?
         hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn HexArmSystem::on_error(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  if (direct_) {(void)direct_->deactivate();}
  stop_io_thread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HexArmSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  (void)feedback_mailbox_.load(read_snapshot_);
  if (active_.load() && ((!read_snapshot_.received_ns ||
    static_cast<double>(monotonic_ns() - read_snapshot_.received_ns) * 1e-9 > state_timeout_.count()) ||
    (!direct_ || !direct_->allowed()))) {return hardware_interface::return_type::ERROR;}
  if (read_snapshot_.received_ns) {
    std::copy(read_snapshot_.position.begin(), read_snapshot_.position.end(), hw_position_.begin());
    std::copy(read_snapshot_.velocity.begin(), read_snapshot_.velocity.end(), hw_velocity_.begin());
    std::copy(read_snapshot_.effort.begin(), read_snapshot_.effort.end(), hw_effort_.begin());
    trace_.emit("direct_read", read_snapshot_.sequence,
      activation_generation_.load(), read_snapshot_.source_stamp_ns);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HexArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load()) {return hardware_interface::return_type::OK;}
  if ((!direct_ || !direct_->allowed()) ||
    !std::all_of(command_position_.begin(), command_position_.end(), [](double value) {
      return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    }) || !std::all_of(command_velocity_.begin(), command_velocity_.end(), [](double value) {
      return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    })) {return hardware_interface::return_type::ERROR;}
  CommandSnapshot command;
  std::copy(command_position_.begin(), command_position_.end(), command.position.begin());
  std::copy(command_velocity_.begin(), command_velocity_.end(), command.velocity.begin());
  command.sequence = ++command_sequence_;
  command.generation = activation_generation_.load();
  command.created_ns = monotonic_ns();
  command_mailbox_.store(command);
  trace_.emit("cxx_write", command.sequence, command.generation, command.created_ns);
  return hardware_interface::return_type::OK;
}

bool HexArmSystem::state_is_fresh() const
{
  FeedbackSnapshot snapshot;
  return feedback_mailbox_.load(snapshot) && snapshot.received_ns &&
         static_cast<double>(monotonic_ns() - snapshot.received_ns) * 1e-9 <= state_timeout_.count();
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
  direct_.reset();
  if (!trace_.stop() && io_node_) {
    RCLCPP_WARN(io_node_->get_logger(), "transport trace drain timed out; incomplete CSV retained");
  }
  executor_.reset();
  io_node_.reset();
  feedback_mailbox_.reset();
  command_mailbox_.reset();
  read_snapshot_ = FeedbackSnapshot{};
}

}  // namespace hex_arm_hardware

PLUGINLIB_EXPORT_CLASS(hex_arm_hardware::HexArmSystem, hardware_interface::SystemInterface)
