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
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <moveit/utils/logger.hpp>
#include <rclcpp/rclcpp.hpp>
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
  MoveGroupExe(const moveit_cpp::MoveItCppPtr& moveit_cpp, const std::string& default_planning_pipeline, bool debug)
  {
    bool allow_trajectory_execution;
    moveit_cpp->getNode()->get_parameter_or("allow_trajectory_execution", allow_trajectory_execution, true);
    context_ =
        std::make_shared<MoveGroupContext>(moveit_cpp, default_planning_pipeline, allow_trajectory_execution, debug);
    configureCapabilities();
  }

  ~MoveGroupExe()
  {
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
      return;
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
      try
      {
        printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'..." MOVEIT_CONSOLE_COLOR_RESET "\n", capability.c_str());
        MoveGroupCapabilityPtr cap = capability_plugin_loader_->createUniqueInstance(capability);
        cap->setContext(context_);
        cap->initialize();
        capabilities_.push_back(cap);
      }
      catch (pluginlib::PluginlibException& ex)
      {
        RCLCPP_ERROR_STREAM(getLogger(),
                            "Exception while loading move_group capability '" << capability << "': " << ex.what());
      }
    }

    std::stringstream ss;
    ss << '\n' << '\n' << "********************************************************" << '\n';
    ss << "* MoveGroup using: " << '\n';
    for (const MoveGroupCapabilityPtr& cap : capabilities_)
    {
      ss << "*     - " << cap->getName() << '\n';
    }
    ss << "********************************************************" << '\n';
    RCLCPP_INFO(getLogger(), "%s", ss.str().c_str());
  }

  MoveGroupContextPtr context_;
  std::shared_ptr<pluginlib::ClassLoader<MoveGroupCapability>> capability_plugin_loader_;
  std::vector<MoveGroupCapabilityPtr> capabilities_;
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
}  // namespace

int main(int argc, char** argv)
{
  const sigset_t shutdown_signals = block_shutdown_signals();

  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_group", node_options);
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

  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node, moveit_cpp_options);
  auto planning_scene_monitor = moveit_cpp->getPlanningSceneMonitorNonConst();
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR(node->get_logger(), "Planning scene not configured");
    planning_scene_monitor.reset();
    moveit_cpp.reset();
    node.reset();
    rclcpp::shutdown();
    return 1;
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

  auto move_group_executable =
      std::make_unique<move_group::MoveGroupExe>(moveit_cpp, default_planning_pipeline, debug);
  bool monitor_dynamics;
  if (node->get_parameter("monitor_dynamics", monitor_dynamics) && monitor_dynamics)
  {
    RCLCPP_INFO(node->get_logger(), "MoveGroup monitors robot dynamics (higher load)");
    planning_scene_monitor->getStateMonitor()->enableCopyDynamics(true);
  }
  planning_scene_monitor->publishDebugInformation(debug);
  move_group_executable->status();

  {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::atomic<bool> stop_waiter{ false };
    std::thread signal_waiter(wait_for_shutdown_signal, std::cref(shutdown_signals), std::ref(stop_waiter),
                              std::ref(executor));
    executor.spin();
    stop_waiter.store(true, std::memory_order_release);
    signal_waiter.join();
    executor.remove_node(node);

    // Tear down every owner of main-node callback groups while their former
    // executor and the ROS context are still alive.
    move_group_executable.reset();
    planning_scene_monitor.reset();
    moveit_cpp.reset();
    node.reset();
  }
  rclcpp::shutdown();
  return 0;
}
