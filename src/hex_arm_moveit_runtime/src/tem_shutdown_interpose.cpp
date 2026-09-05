// SPDX-License-Identifier: BSD-3-Clause
//
// MoveIt 2.12.4 starts a private executor inside TrajectoryExecutionManager,
// but its destructor does not stop and detach that executor before the owned
// controller-manager node begins destroying callback groups.  Interpose the
// installed destructor with the missing ordering prelude, then delegate all
// remaining cleanup to the original implementation.

#include <dlfcn.h>

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <thread>

#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>

namespace
{
using TrajectoryExecutionManager = trajectory_execution_manager::TrajectoryExecutionManager;
using OriginalDestructor = void (*)(TrajectoryExecutionManager*);

// This package is intentionally pinned to MoveIt 2.12.4 in CMake. Explicit
// template instantiation retrieves pointers to the installed class members
// without copying or guessing their ABI offsets.
template <typename Tag, typename Tag::type Member>
struct PrivateMember
{
  friend typename Tag::type get(Tag)
  {
    return Member;
  }
};

struct ControllerNodeTag
{
  using type = rclcpp::Node::SharedPtr TrajectoryExecutionManager::*;
  friend type get(ControllerNodeTag);
};
template struct PrivateMember<ControllerNodeTag, &TrajectoryExecutionManager::controller_mgr_node_>;

struct PrivateExecutorTag
{
  using type = std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> TrajectoryExecutionManager::*;
  friend type get(PrivateExecutorTag);
};
template struct PrivateMember<PrivateExecutorTag, &TrajectoryExecutionManager::private_executor_>;

struct PrivateThreadTag
{
  using type = std::thread TrajectoryExecutionManager::*;
  friend type get(PrivateThreadTag);
};
template struct PrivateMember<PrivateThreadTag, &TrajectoryExecutionManager::private_executor_thread_>;

OriginalDestructor original_destructor()
{
  static auto original = reinterpret_cast<OriginalDestructor>(
      dlsym(RTLD_NEXT, "_ZN28trajectory_execution_manager26TrajectoryExecutionManagerD1Ev"));
  if (original == nullptr)
  {
    std::fprintf(stderr, "hex-arm: cannot resolve the MoveIt 2.12.4 TEM destructor: %s\n", dlerror());
    std::abort();
  }
  return original;
}

void ordered_trajectory_execution_manager_destructor(TrajectoryExecutionManager* self)
{
  try
  {
    auto& executor = self->*get(PrivateExecutorTag{});
    auto& executor_thread = self->*get(PrivateThreadTag{});
    auto& controller_node = self->*get(ControllerNodeTag{});
    if (executor)
    {
      executor->cancel();
    }
    if (executor_thread.joinable())
    {
      executor_thread.join();
    }
    if (executor && controller_node)
    {
      executor->remove_node(controller_node);
    }
    // Destroy callback groups while the ROS context and their former executor
    // are both still alive.  Leaving this to the generated member-destructor
    // sequence is the Jazzy rclcpp lifetime race this shim avoids.
    controller_node.reset();
  }
  catch (const std::exception& error)
  {
    std::fprintf(stderr, "hex-arm: TEM ordered shutdown failed: %s\n", error.what());
    std::abort();
  }
  catch (...)
  {
    std::fprintf(stderr, "hex-arm: TEM ordered shutdown failed with an unknown exception\n");
    std::abort();
  }
  original_destructor()(self);
}
}  // namespace

extern "C" void tem_destructor_interpose(TrajectoryExecutionManager* self)
    __asm__("_ZN28trajectory_execution_manager26TrajectoryExecutionManagerD1Ev");

extern "C" void tem_destructor_interpose(TrajectoryExecutionManager* self)
{
  ordered_trajectory_execution_manager_destructor(self);
}

extern "C" void tem_destructor_interpose_base(TrajectoryExecutionManager* self)
    __asm__("_ZN28trajectory_execution_manager26TrajectoryExecutionManagerD2Ev");

extern "C" void tem_destructor_interpose_base(TrajectoryExecutionManager* self)
{
  tem_destructor_interpose(self);
}
