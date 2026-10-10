// Integration/benchmark workload that loads the installed, real C++ plugin.
// The launcher owns the isolated Rust --mock process. This executable never
// opens CAN or creates a substitute hardware implementation.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hex_arm_hardware/transport_trace.hpp"

int main(int argc, char ** argv)
{
  if (argc != 6) {
    std::cerr << "transport_probe endpoint prefix duration warmup scenario\n";
    return 2;
  }
  rclcpp::init(0, nullptr);
  try {
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
    auto system = loader.createSharedInstance("hex_arm_hardware/HexArmSystem");
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info.name = "transport_benchmark";
    params.hardware_info.hardware_parameters = {{"zenoh_connect", argv[1]}, {"robot_prefix", argv[2]},
      {"activation_timeout_sec", "5"}, {"startup_timeout_sec", "15"},
      {"service_timeout_sec", "1"}};
    for (int i = 1; i <= 6; ++i) {
      hardware_interface::ComponentInfo joint; joint.name = "joint_" + std::to_string(i);
      for (const auto & name : {"position", "velocity", "effort"}) {
        hardware_interface::InterfaceInfo interface; interface.name = name;
        joint.state_interfaces.push_back(interface);
        if (interface.name != "effort") {joint.command_interfaces.push_back(interface);}
      }
      params.hardware_info.joints.push_back(joint);
    }
    const auto success = hardware_interface::CallbackReturn::SUCCESS;
    const rclcpp_lifecycle::State lifecycle_state;
    if (system->on_init(params) != success || system->on_configure(lifecycle_state) != success) {return 3;}
    if (std::string(argv[5]) == "lifecycle") {
      for (int cycle = 0; cycle < 3; ++cycle) {
        if (system->on_error(lifecycle_state) != success ||
          system->on_configure(lifecycle_state) != success ||
          system->on_configure(lifecycle_state) != success) {return 5;}
      }
      if (system->on_shutdown(lifecycle_state) != success ||
        system->on_cleanup(lifecycle_state) != success ||
        system->on_configure(lifecycle_state) != success) {return 5;}
      rclcpp::shutdown();
      system.reset();
      return 0;
    }
    auto states = system->export_state_interfaces();
    auto commands = system->export_command_interfaces();
    // Allow the three independently published readiness streams to arrive.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    system->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
    if (system->on_activate(lifecycle_state) != success) {return 4;}
    const double duration = std::stod(argv[3]), warmup = std::stod(argv[4]);
    const std::string scenario = argv[5];
    const bool fault_scenario = scenario == "stopped-command" || scenario == "delay-recovery" ||
      scenario == "disconnect" || scenario == "restart" || scenario == "reactivate";
    hex_arm_hardware::TransportTrace trace; trace.start();
    const auto start = std::chrono::steady_clock::now();
    std::cout << "PLUGIN_ACTIVE_NS=" << std::chrono::duration_cast<std::chrono::nanoseconds>(
      start.time_since_epoch()).count() << std::endl;
    auto next = start;
    std::array<double, 6> reference{};
    for (std::size_t i = 0; i < 6; ++i) {reference[i] = *states[i * 3].get_optional<double>();}
    bool gate_closed = false;
    bool reactivated = false;
    std::uint64_t cycle = 0;
    while (rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      if (elapsed >= duration + warmup) {break;}
      trace.emit("control_cycle", ++cycle, 0);
      const auto read_result = system->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
      if (read_result == hardware_interface::return_type::ERROR) {gate_closed = true;}
      if (!fault_scenario && gate_closed) {throw std::runtime_error("unexpected hardware read error");}
      const double inject = warmup + duration / 2;
      if ((scenario == "reactivate" || scenario == "reactivate-stream") &&
        !reactivated && elapsed >= inject)
      {
        if (system->on_deactivate(lifecycle_state) != success ||
          system->on_activate(lifecycle_state) != success) {
          throw std::runtime_error("explicit reactivation failed");
        }
        reactivated = true;
        std::cout << "PLUGIN_REACTIVATED_NS=" <<
          std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count() << std::endl;
      }
      const bool stop_write = elapsed >= inject &&
        (scenario == "stopped-command" || scenario == "reactivate" ||
        (scenario == "delay-recovery" && elapsed < inject + .25));
      if (!stop_write) {
        for (std::size_t i = 0; i < 6; ++i) {
          const bool moving = scenario == "trajectory" && i == 0;
          (void)commands[i * 2].set_value(reference[i] + (moving ? .02 * std::sin(elapsed * .5) : 0));
          (void)commands[i * 2 + 1].set_value(moving ? .01 * std::cos(elapsed * .5) : 0);
        }
        const auto result = system->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
        if (!fault_scenario && result == hardware_interface::return_type::ERROR) {
          throw std::runtime_error("unexpected hardware write error");
        }
      }
      trace.emit("control_cycle_end", cycle, 0);
      next += std::chrono::milliseconds(10);
      // Never burst replay missed control cycles after scheduler suspension.
      if (next < std::chrono::steady_clock::now()) {next = std::chrono::steady_clock::now();}
      std::this_thread::sleep_until(next);
    }
    const auto stopped = system->on_deactivate(lifecycle_state);
    system->on_cleanup(lifecycle_state);
    trace.stop();
    if (fault_scenario && !gate_closed) {throw std::runtime_error("fault did not close hardware gate");}
    if (scenario != "restart" && stopped != success) {throw std::runtime_error("disable not acknowledged");}
    std::cout << "PLUGIN_PROBE_PASSED gate_closed=" << gate_closed << std::endl;
    system.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
