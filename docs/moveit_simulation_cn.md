# MoveIt 图形化 mock 仿真

[English](moveit_simulation.md) | **中文**

此流程会验证 MoveIt、OMPL、碰撞检测、逆运动学、`FollowJointTrajectory`
以及现有的 ros2_control `GenericSystem`。它不会访问 USB/CAN 硬件，也不是物理仿真。

## 本地 Ubuntu 24.04 启动（当前机器）

先在 Ubuntu 桌面的宿主机终端中启动容器并检查图形转发：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

进入提示符主机名为 `hex-arm-dev` 的 `ros2-jazzy-arm` 容器后执行：

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh  # 首次使用或源码改动后执行
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

最后一条命令会启动 `move_group` 和 RViz，正常情况下 Ubuntu 桌面会弹出 MoveIt
界面。

## WSL2 启动（仅 Windows 10/11）

WSL2 是 **Windows Subsystem for Linux 2**：Ubuntu 运行在 Windows 内部，图形
窗口由 WSLg 显示。在 Windows 的 Ubuntu/WSL 终端（不是 PowerShell）中执行：

```bash
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2  # 仓库不在此处时请替换路径
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

进入同一个 `ros2-jazzy-arm` 容器后，MoveIt 命令与本地 Ubuntu 相同：

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh  # 首次使用或源码改动后执行
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

如果现有 `hex_arm_bringup mock.launch.py` 使用相同的 `ROS_DOMAIN_ID`，请先停止
该会话；两个 launch 文件有意使用相同的控制器名称。如果 RViz 报
`could not connect to display :0`，请在宿主机运行
`./scripts/docker-dev.sh doctor`。脚本会在本地 Ubuntu 检查 Xauthority，在 WSL2
使用 WSLg 配置；不要用 `xhost +` 绕过访问控制。自检通过后重新进入容器再启动
MoveIt。

在 RViz 中选择 `arm` 规划组。拖动交互标记，或选择名为 `ready` 的状态；先点击
**Plan**，确认预览正确后再点击 **Plan & Execute**。执行链路如下：

```text
MoveIt RViz -> move_group -> firefly_arm_controller
             -> ros2_control GenericSystem -> /joint_states -> RViz
```

默认 `sim` 配置保留 URDF 中的 `6.0 rad/s` 上限，并使用 0.1 的 MoveIt 默认
缩放系数。无需连接硬件，即可预览首次实机调试所采用的保守低速策略：

```bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py \
  limits_profile:=commissioning
```

可用配置如下：

| 配置 | 每轴最大速度 | 用途 |
|---|---:|---|
| `sim` | 6.0 rad/s | 纯软件图形化测试 |
| `commissioning` | 0.2 rad/s | 首次有人监督的实机调试 |
| `verified` | 6.0 rad/s | 仅在分阶段真机验证完成后使用 |

MoveIt 限位是规划限位，不是真机安全边界。真机启动必须使用已经验证的本地硬件
profile，并保证其中的 `velocity_rad_s` 不大于所选 MoveIt profile。Rust 控制器
还会根据该硬件 profile 独立限制位置目标的变化速率。MoveIt 需要加速度限制来完成
轨迹时间参数化：`sim` 只为软件可视化使用历史值 10.0 rad/s²；两个面向真机的
profile 在实测前都暂时保持 0.2 rad/s²。这里不对真机加速度安全性作出保证，Rust
目前也没有独立实施加速度硬限制。

新的 SRDF 会继续检查非相邻连杆之间的自碰撞，只排除六对直接相邻的连杆；历史
SRDF 曾禁用全部 21 对连杆碰撞，因此没有复用。第一版暂时使用 `link_6` 作为规划
末端，直到获得经过标定的固定 TCP/工具坐标系。

运行以下命令可执行无界面的规划与轨迹执行冒烟测试：

```bash
python3 src/hex_arm_moveit_config/test/test_moveit_mock.py
```
