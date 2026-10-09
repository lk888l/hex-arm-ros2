/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Based on moveit2 2.12.4 moveit_ros/move_group/src/move_group.cpp.
// Main owns signal delivery and the teardown order needed by Jazzy rclcpp.

#include <pthread.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <boost/tokenizer.hpp>
#include <moveit/macros/console_colors.hpp>
#include <moveit/move_group/move_group_capability.hpp>
#include <moveit/move_group/move_group_context.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/plan_execution/plan_execution.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <moveit/utils/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>

namespace move_group
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.move_group.executable");
}
}  // namespace

// clang-format off
static const char* const DEFAULT_CAPABILITIES[] = {
  "move_group/LoadGeometryFromFileService",
  "move_group/SaveGeometryToFileService",
  "move_group/GetUrdfService",
  "move_group/MoveGroupCartesianPathService",
  "move_group/MoveGroupKinematicsService",
  "move_group/MoveGroupExecuteTrajectoryAction",
  "move_group/MoveGroupMoveAction",
  "move_group/MoveGroupPlanService",
  "move_group/MoveGroupQueryPlannersService",
  "move_group/MoveGroupStateValidationService",
  "move_group/MoveGroupGetPlanningSceneService",
  "move_group/ApplyPlanningSceneService",
  "move_group/ClearOctomapService",
};
// clang-format on

class MoveGroupExe
{
public:
  MoveGroupExe(const moveit_cpp::MoveItCppPtr& moveit_cpp, const std::string& default_planning_pipeline, bool debug,
               std::function<void()> cancel_executor)
    : cancel_executor_(std::move(cancel_executor))
  {
    bool allow_trajectory_execution;
    moveit_cpp->getNode()->get_parameter_or("allow_trajectory_execution", allow_trajectory_execution, true);
    context_ =
        std::make_shared<MoveGroupContext>(moveit_cpp, default_planning_pipeline, allow_trajectory_execution, debug);
    moveit_cpp->getNode()->get_parameter_or("startup_readiness_token", startup_readiness_token_, std::string{});
#ifdef HEX_ARM_ENABLE_GATE_FAILURE_TEST
    moveit_cpp->getNode()->get_parameter_or("test_fail_execution_capability", test_fail_execution_capability_, false);
#endif
    if (!startup_readiness_token_.empty() && !allow_trajectory_execution)
    {
      throw std::runtime_error("startup readiness gate requires trajectory execution to be configured");
    }
    configureCapabilities();
    if (!deferred_capabilities_.empty())
    {
      RCLCPP_INFO(getLogger(), "MoveIt execution locked: waiting for verified startup hold");
      startup_ready_subscription_ = moveit_cpp->getNode()->create_subscription<std_msgs::msg::String>(
          "/hex_arm/internal/moveit_startup_ready", rclcpp::QoS(1).reliable().transient_local(),
          [this](const std_msgs::msg::String::SharedPtr message) {
            if (message->data != startup_readiness_token_ || deferred_capabilities_.empty() || failed_)
            {
              return;
            }
            try
            {
              for (const auto& name : deferred_capabilities_)
              {
                if (!loadCapability(name))
                {
                  throw std::runtime_error("failed to unlock MoveIt execution capability: " + name);
                }
              }
              deferred_capabilities_.clear();
              RCLCPP_INFO(getLogger(), "MoveIt execution unlocked after verified startup hold");
            }
            catch (const std::exception& error)
            {
              closeExecution(error.what());
            }
            catch (...)
            {
              closeExecution("unknown plugin exception");
            }
          });
    }
  }

  bool failed() const
  {
    return failed_;
  }

  void stopExecution()
  {
    if (context_->plan_execution_)
    {
      context_->plan_execution_->stop();
    }
    if (context_->trajectory_execution_manager_)
    {
      // ExecuteTrajectory owns a callback thread blocked in waitForExecution.
      // Finish its goal while ROS and controller callbacks still exist, before
      // capabilities_.clear() joins that thread during destruction.
      context_->trajectory_execution_manager_->stopExecution(true);
    }
  }

  ~MoveGroupExe()
  {
    startup_ready_subscription_.reset();
    capabilities_.clear();
    context_.reset();
    capability_plugin_loader_.reset();
  }

  void status()
  {
    if (context_)
    {
      if (context_->status())
      {
        if (capabilities_.empty())
        {
          printf("\n" MOVEIT_CONSOLE_COLOR_BLUE
                 "move_group is running but no capabilities are loaded." MOVEIT_CONSOLE_COLOR_RESET "\n\n");
        }
        else
        {
          printf("\n" MOVEIT_CONSOLE_COLOR_GREEN "You can start planning now!" MOVEIT_CONSOLE_COLOR_RESET "\n\n");
        }
        fflush(stdout);
      }
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "No MoveGroup context created. Nothing will work.");
    }
  }

private:
  void closeExecution(const char* reason)
  {
    // A MultiThreadedExecutor worker must never throw out of the unlock
    // callback. Roll back capabilities already loaded by this attempt and
    // let main perform the common teardown with its executor still alive.
    capabilities_.erase(std::remove_if(capabilities_.begin(), capabilities_.end(),
                                      [](const auto& entry) { return isExecutionCapability(entry.first); }),
                        capabilities_.end());
    failed_ = true;
    RCLCPP_ERROR(getLogger(), "MoveIt execution unlock failed; execution closed: %s", reason);
    cancel_executor_();
  }

  static bool isExecutionCapability(const std::string& name)
  {
    return name == "move_group/MoveGroupMoveAction" || name == "move_group/MoveGroupExecuteTrajectoryAction";
  }

  bool loadCapability(const std::string& name)
  {
    try
    {
      printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'..." MOVEIT_CONSOLE_COLOR_RESET "\n", name.c_str());
      std::string plugin_name = name;
#ifdef HEX_ARM_ENABLE_GATE_FAILURE_TEST
      if (test_fail_execution_capability_ && name == "move_group/MoveGroupMoveAction")
      {
        plugin_name = "hex_arm_test/MissingCapability";
      }
#endif
      MoveGroupCapabilityPtr cap = capability_plugin_loader_->createUniqueInstance(plugin_name);
      cap->setContext(context_);
      cap->initialize();
      capabilities_.emplace_back(name, cap);
      return true;
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR_STREAM(getLogger(), "Exception while loading move_group capability '" << name << "': " << ex.what());
      return false;
    }
  }

  void configureCapabilities()
  {
    try
    {
      capability_plugin_loader_ = std::make_shared<pluginlib::ClassLoader<MoveGroupCapability>>(
          "moveit_ros_move_group", "move_group::MoveGroupCapability");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL_STREAM(getLogger(),
                          "Exception while creating plugin loader for move_group capabilities: " << ex.what());
      throw;
    }

    std::set<std::string> capabilities;
    for (const char* capability : DEFAULT_CAPABILITIES)
    {
      capabilities.insert(capability);
    }

    std::string capability_plugins;
    if (context_->moveit_cpp_->getNode()->get_parameter("capabilities", capability_plugins))
    {
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char>> tok(capability_plugins, sep);
      capabilities.insert(tok.begin(), tok.end());
    }
    for (const auto& pipeline_entry : context_->moveit_cpp_->getPlanningPipelines())
    {
      const auto& pipeline_name = pipeline_entry.first;
      std::string pipeline_capabilities;
      if (context_->moveit_cpp_->getNode()->get_parameter(pipeline_name + ".capabilities", pipeline_capabilities))
      {
        boost::char_separator<char> sep(" ");
        boost::tokenizer<boost::char_separator<char>> tok(pipeline_capabilities, sep);
        capabilities.insert(tok.begin(), tok.end());
      }
    }
    if (context_->moveit_cpp_->getNode()->get_parameter("disable_capabilities", capability_plugins))
    {
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char>> tok(capability_plugins, sep);
      for (auto cap_name = tok.begin(); cap_name != tok.end(); ++cap_name)
      {
        capabilities.erase(*cap_name);
      }
    }

    for (const std::string& capability : capabilities)
    {
      // Keep collision/scene/planning services available for startup validation,
      // but expose neither MoveGroup plan-and-execute nor ExecuteTrajectory
      // until the startup client acknowledges this launch's stationary hold.
      if (!startup_readiness_token_.empty() && isExecutionCapability(capability))
      {
        deferred_capabilities_.push_back(capability);
        continue;
      }
      if (!loadCapability(capability))
      {
        throw std::runtime_error("failed to configure MoveIt capability: " + capability);
      }
    }

    std::stringstream ss;
    ss << '\n' << '\n' << "********************************************************" << '\n';
    ss << "* MoveGroup using: " << '\n';
    for (const auto& cap : capabilities_)
    {
      ss << "*     - " << cap.second->getName() << '\n';
    }
    ss << "********************************************************" << '\n';
    RCLCPP_INFO(getLogger(), "%s", ss.str().c_str());
  }

  MoveGroupContextPtr context_;
  std::shared_ptr<pluginlib::ClassLoader<MoveGroupCapability>> capability_plugin_loader_;
  std::vector<std::pair<std::string, MoveGroupCapabilityPtr>> capabilities_;
  std::string startup_readiness_token_;
  std::vector<std::string> deferred_capabilities_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr startup_ready_subscription_;
  std::function<void()> cancel_executor_;
  std::atomic<bool> failed_{ false };
#ifdef HEX_ARM_ENABLE_GATE_FAILURE_TEST
  bool test_fail_execution_capability_{ false };
#endif
};
}  // namespace move_group

namespace
{
void retain_default_callback_group_until_process_exit(const rclcpp::Node::SharedPtr& node)
{
  // rclcpp 28.1.21 can leave a stale weak entity entry in MoveIt's default
  // CallbackGroup. CallbackGroup::~CallbackGroup then dereferences the freed
  // weak control block after all useful ROS objects have already shut down.
  // Keep exactly one shared owner for this process lifetime; the OS reclaims
  // it at exit. CMake rejects other dependency ABIs pending a fresh audit.
  [[maybe_unused]] static const auto* holder = new rclcpp::CallbackGroup::SharedPtr(
      node->get_node_base_interface()->get_default_callback_group());
}

sigset_t block_shutdown_signals()
{
  sigset_t shutdown_signals;
  sigemptyset(&shutdown_signals);
  sigaddset(&shutdown_signals, SIGINT);
  sigaddset(&shutdown_signals, SIGTERM);
  const int result = pthread_sigmask(SIG_BLOCK, &shutdown_signals, nullptr);
  if (result != 0)
  {
    throw std::runtime_error("failed to block SIGINT/SIGTERM: " + std::string(std::strerror(result)));
  }
  return shutdown_signals;
}

void wait_for_shutdown_signal(const sigset_t& shutdown_signals, std::atomic<bool>& stop_waiter,
                              rclcpp::executors::MultiThreadedExecutor& executor)
{
  while (!stop_waiter.load(std::memory_order_acquire))
  {
    timespec timeout{ 0, 100'000'000 };
    const int signal_number = sigtimedwait(&shutdown_signals, nullptr, &timeout);
    if (signal_number == SIGINT || signal_number == SIGTERM)
    {
      if (!stop_waiter.load(std::memory_order_acquire))
      {
        RCLCPP_INFO(rclcpp::get_logger("hex_arm_safe_move_group"), "received signal %d; cancelling executor",
                    signal_number);
        executor.cancel();
      }
      return;
    }
    if (signal_number == -1 && errno != EAGAIN && errno != EINTR)
    {
      RCLCPP_ERROR(rclcpp::get_logger("hex_arm_safe_move_group"), "sigtimedwait failed: %s", std::strerror(errno));
      executor.cancel();
      return;
    }
  }
}

class ShutdownSignalWaiter
{
public:
  ShutdownSignalWaiter(const sigset_t& shutdown_signals, rclcpp::executors::MultiThreadedExecutor& executor)
    : thread_(wait_for_shutdown_signal, std::cref(shutdown_signals), std::ref(stop_), std::ref(executor))
  {
  }

  ~ShutdownSignalWaiter()
  {
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  ShutdownSignalWaiter(const ShutdownSignalWaiter&) = delete;
  ShutdownSignalWaiter& operator=(const ShutdownSignalWaiter&) = delete;

private:
  std::atomic<bool> stop_{ false };
  std::thread thread_;
};
}  // namespace

int main(int argc, char** argv)
{
  int result = 0;
  bool initialized = false;
  bool node_added = false;
  rclcpp::Node::SharedPtr node;
  moveit_cpp::MoveItCppPtr moveit_cpp;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  std::unique_ptr<move_group::MoveGroupExe> move_group_executable;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  try
  {
    const sigset_t shutdown_signals = block_shutdown_signals();

    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);
    initialized = true;
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("move_group", node_options);
    retain_default_callback_group_until_process_exit(node);
    moveit::setNodeLoggerName(node->get_name());
    moveit_cpp::MoveItCpp::Options moveit_cpp_options(node);
    moveit_cpp_options.planning_pipeline_options.parent_namespace =
        node->get_effective_namespace() + ".planning_pipelines";

    std::vector<std::string> planning_pipeline_configs;
    if (node->get_parameter("planning_pipelines", planning_pipeline_configs))
    {
      if (planning_pipeline_configs.empty())
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to read parameter 'move_group.planning_pipelines'");
      }
      else
      {
        for (const auto& config : planning_pipeline_configs)
        {
          moveit_cpp_options.planning_pipeline_options.pipeline_names.push_back(config);
        }
      }
    }

    auto& pipeline_names = moveit_cpp_options.planning_pipeline_options.pipeline_names;
    std::string default_planning_pipeline;
    if (node->get_parameter("default_planning_pipeline", default_planning_pipeline))
    {
      if (std::find(pipeline_names.begin(), pipeline_names.end(), default_planning_pipeline) == pipeline_names.end())
      {
        RCLCPP_WARN(node->get_logger(),
                    "MoveGroup launched with ~default_planning_pipeline '%s' not configured in ~planning_pipelines",
                    default_planning_pipeline.c_str());
        default_planning_pipeline.clear();
      }
    }
    else if (pipeline_names.size() > 1)
    {
      RCLCPP_WARN(node->get_logger(),
                  "MoveGroup launched without ~default_planning_pipeline specifying the namespace for the default "
                  "planning pipeline configuration");
    }
    if (default_planning_pipeline.empty())
    {
      if (!pipeline_names.empty())
      {
        RCLCPP_WARN(node->get_logger(), "Using default pipeline '%s'", pipeline_names.front().c_str());
        default_planning_pipeline = pipeline_names.front();
      }
      else
      {
        RCLCPP_WARN(node->get_logger(),
                    "Falling back to using the the move_group node namespace (deprecated behavior).");
        default_planning_pipeline = "move_group";
        pipeline_names = { default_planning_pipeline };
        moveit_cpp_options.planning_pipeline_options.parent_namespace = node->get_effective_namespace();
      }
      node->set_parameter(rclcpp::Parameter("default_planning_pipeline", default_planning_pipeline));
    }

    moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node, moveit_cpp_options);
    planning_scene_monitor = moveit_cpp->getPlanningSceneMonitorNonConst();
    if (!planning_scene_monitor->getPlanningScene())
    {
      throw std::runtime_error("Planning scene not configured");
    }

    bool debug = false;
    for (int index = 1; index < argc; ++index)
    {
      if (std::strncmp(argv[index], "--debug", 7) == 0)
      {
        debug = true;
        break;
      }
    }
    RCLCPP_INFO(node->get_logger(), "MoveGroup debug mode is %s", debug ? "ON" : "OFF");

    move_group_executable = std::make_unique<move_group::MoveGroupExe>(
        moveit_cpp, default_planning_pipeline, debug, [&executor]() { executor->cancel(); });
    bool monitor_dynamics;
    if (node->get_parameter("monitor_dynamics", monitor_dynamics) && monitor_dynamics)
    {
      RCLCPP_INFO(node->get_logger(), "MoveGroup monitors robot dynamics (higher load)");
      planning_scene_monitor->getStateMonitor()->enableCopyDynamics(true);
    }
    planning_scene_monitor->publishDebugInformation(debug);
    move_group_executable->status();

    executor->add_node(node);
    node_added = true;
    {
      ShutdownSignalWaiter signal_waiter(shutdown_signals, *executor);
      executor->spin();
    }
    result = move_group_executable->failed() ? 1 : 0;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("hex_arm_safe_move_group"), "MoveGroup failed: %s", error.what());
    result = 1;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger("hex_arm_safe_move_group"), "MoveGroup failed with an unknown exception");
    result = 1;
  }

  // Normal signals, startup failures and unlock failures share this order.
  // Stop and detach before releasing main-node callback groups, while the
  // former executor and ROS context are still alive.
  if (executor)
  {
    executor->cancel();
    if (node_added)
    {
      try
      {
        executor->remove_node(node);
      }
      catch (const std::exception& error)
      {
        RCLCPP_ERROR(rclcpp::get_logger("hex_arm_safe_move_group"), "MoveGroup detach failed: %s", error.what());
        result = 1;
      }
    }
  }
  if (move_group_executable)
  {
    try
    {
      move_group_executable->stopExecution();
    }
    catch (const std::exception& error)
    {
      RCLCPP_ERROR(rclcpp::get_logger("hex_arm_safe_move_group"), "MoveGroup execution stop failed: %s", error.what());
      result = 1;
    }
  }
  move_group_executable.reset();
  planning_scene_monitor.reset();
  moveit_cpp.reset();
  node.reset();
  executor.reset();
  if (initialized)
  {
    rclcpp::shutdown();
  }
  return result;
}
