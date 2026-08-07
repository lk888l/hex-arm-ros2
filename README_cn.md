# Firefly Y6 ROS 2 驱动

面向六轴 Firefly Y6 机械臂的 ROS 2 Jazzy 驱动、仿真与调试（commissioning）工作区。对外公开的运动接口是由 `joint_trajectory_controller` 暴露的标准 `control_msgs/action/FollowJointTrajectory` action：

```text
/firefly_arm_controller/follow_joint_trajectory
```

本项目刻意将 ROS 控制回路与 USB/CAN-FD 回路分离：

```text
MoveIt / FollowJointTrajectory
  -> ros2_control + firefly_arm_controller
  -> hex_arm_hardware/SystemInterface
  -> hex_arm_bridge（ROS lifecycle <-> Zenoh robot_api）
  -> hex_arm_controller（Rust 安全状态机，1 kHz 软实时回路）
  -> 用户态 gs_usb -> CAN-FD 电机
```

Rust 进程拥有电机总线及全部安全决策权。ROS 桥接层不实现轨迹 action，也无法绕过独占会话或激活状态机。

## 支持的驱动后端（backend）

| 后端 | 用途 | 硬件访问 |
|---|---|---|
| `view` | URDF、关节方向、限位与 RViz 检查 | 无 |
| `mock` | ros2_control/JTC 生命周期与 action 集成测试 | 无 |
| `gz` | 通过 `gz_ros2_control` 使用 Gazebo Harmonic 物理仿真 | 无 |
| `real` | Rust 控制器、Zenoh 桥接、ros2_control | 仅 `/dev/bus/usb` |

在不接入 USB 的情况下启动开发容器。请在 **Ubuntu 24.04 WSL shell** 中运行以下命令，而不是在 PowerShell 中：WSL 必须能够解析 `/mnt/wslg` 和 `/tmp/.X11-unix`，RViz 和 Gazebo 才能访问 WSLg。

```bash
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2
docker compose build
docker compose up -d
docker exec -it ros2-jazzy-arm bash
```

容器内的提示符主机名为 `hex-arm-dev`。较旧的 `ros2-jazzy` 容器使用主机名 `ros2-dev`；请勿在其中 source 本工作区生成的 `install/` 目录树。`--symlink-install` 构建绑定在 `ros2-jazzy-arm` 使用的 `/workspaces/hex_arm_ros2` 挂载点上。

在 `ros2-jazzy-arm` 内构建一次：

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh
source install/setup.bash
```

然后每个终端选择一个图形化模式：

```bash
# URDF + 关节滑块 + RViz
ros2 launch hex_arm_bringup view.launch.py

# ros2_control GenericSystem + 轨迹控制器 + RViz
ros2 launch hex_arm_bringup mock.launch.py use_rviz:=true

# Gazebo Harmonic 物理仿真 + RViz
ros2 launch hex_arm_bringup gz.launch.py headless:=false use_rviz:=true
```

在启动另一种模式之前，请先按 `Ctrl-C` 停止当前 launch。如果 `source install/setup.bash` 报错称 `/workspaces/hex_arm_ros2` 下的路径缺失，说明当前所在的容器不对；请在 WSL 宿主机上执行 `docker exec -it ros2-jazzy-arm bash` 进入正确的容器。

real 配置故意做成独立的 Compose override。它只映射 USB 总线，并且从不启用 Docker 特权模式：

```bash
docker compose -f compose.yaml -f compose.real.yaml up -d
```

将 `config/hardware/firefly_y6.example.yaml` 复制为被 git 忽略的 `*.local.yaml`，填写每一项 identity、方向、偏移和限位，然后在真实启动前进行校验。该示例明确标记为不完整，bringup 层和 Rust 控制器都会拒绝使用它。

```bash
ros2 launch hex_arm_bringup real.launch.py \
  hardware_profile:=/workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

## 安全边界

真实启动始终从 disabled（禁用）状态开始。激活需要六个全新识别（fresh identified）的电机、一份完整校准的配置文件、一个独占的 robot_api 会话，以及一次显式的 ros2_control 生命周期转换。电机故障、反馈过期、非有限（non-finite）指令、USB 失败或指令看门狗超时，都会锁存整臂故障并触发停止/禁用。

WSL2、Docker、Zenoh 和用户态 USB 均不属于硬实时或经过安全认证的组件。调试（commissioning）时必须配备可用的物理急停开关。本仓库不声称关节/节点映射、零位偏移、运动方向、扭矩标定或执行精度已在硬件上得到验证。

历史遗留的 MoveIt SRDF 未安装，因为它会禁用全部 21 对自碰撞。在重新生成并审查碰撞模型之前，MoveIt 集成仅限于稳定的关节名称、控制器名称和标准轨迹 action。

## 可复现性

`hex_arm.repos` 固定了上游源码的修订版本。仓库中检入的 `xpkg_urdf_firefly_y6` 包是指定描述修订版本的可溯源快照（provenance-preserving snapshot），包含原始网格文件。仅在更新或审计上游源码时运行：

```bash
vcs import . < hex_arm.repos
```

常规构建不会静默拉取可变分支。

## 测试级别

```bash
./scripts/test.sh unit
./scripts/test.sh protocol
./scripts/test.sh mock
./scripts/test.sh gz
```

`unit` 不依赖硬件。`protocol` 使用 mock 电机后端启动 Rust 控制器，并验证 Zenoh 发现/事件以及 ROS 生命周期桥接。`mock` 测试 FollowJointTrajectory 的发送/取消和控制器生命周期。`gz` 以无头模式启动 Gazebo 并验证两条轨迹。真实调试是 `docs/commissioning.md` 中一份单独的、有人监督的检查清单。
