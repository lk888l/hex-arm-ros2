# Firefly Y6 ROS 2 驱动

[English](README.md) | **中文**

六轴 Firefly Y6 的 ROS 2 Jazzy 驱动、MoveIt 2 规划和 Gazebo 仿真工作区。
默认构建包含 Meow 与 CiA402 驱动；当前真机使用 **Meow 固件、can2，无夹爪及附加载荷**。

| 要做什么 | 入口 | 命令执行位置 |
|---|---|---|
| 打开 MoveIt，规划并模拟执行 | [MoveIt 模拟](#moveit-quick-start) | 容器 |
| 用 MoveIt 控制当前机械臂 | [MoveIt 真机](#real-hardware) | 宿主机 |
| 手扶拖动，使用重力补偿和阻尼 | [手扶拖动](#hand-guiding) | 容器 |
| 查看模型、普通 mock 或 Gazebo | [其他模拟模式](#simulation-modes) | 容器 |

导航：[环境与构建](#environment) · [MoveIt 真机](#real-hardware) ·
[开发与测试](#development) · [常见问题](#troubleshooting) · [文档索引](#documentation)

<a id="environment"></a>

## 环境与构建

本地 Ubuntu 24.04 使用图形桌面的终端；WSL2 使用 Ubuntu/WSL 终端。
`docker-dev.sh` 在**宿主机**执行，`build.sh` 和 `ros2` 在**容器**执行：

| 环境 | 工作区 | 识别方式 |
|---|---|---|
| 宿主机 | `/home/kk/kk_data/ros2_project/hex-arm-ros2` | 本机用户的终端 |
| 容器 | `/workspaces/hex_arm_ros2` | `root@hex-arm-dev`，容器名 `ros2-jazzy-arm` |

宿主机仓库路径不同时替换下方 `cd` 路径；容器挂载路径仍为 `/workspaces/hex_arm_ros2`。
首次使用或 Dockerfile 改动后，先在宿主机仓库目录执行 `./scripts/docker-dev.sh build`。
随后启动容器并进入：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

首次使用或源码更新后，在容器内构建工作区：

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh
source install/setup.bash
```

已有构建时跳过 `build.sh`；每个新容器终端仍需 `source install/setup.bash`。
真机控制运行期间，先完成正常停机，再修改配置、构建或切换控制程序。

<a id="docker-setup"></a>
<a id="nvidia-独立显卡加速"></a>

### Docker 与图形环境

脚本自动选择本地 Ubuntu 的 X11 配置或 WSL2 的 WSLg 配置。
NVIDIA 主机需要 Container Toolkit，`HEX_ARM_GPU=auto` 自动选择 GPU；`none` 禁用 NVIDIA。
无图形桌面时使用 `HEX_ARM_HEADLESS=1`。环境诊断在宿主机运行 `./scripts/docker-dev.sh doctor`。

原生 Compose、NVIDIA 安装和 WSL2 配置见 [Docker 开发环境](docs/docker_development_cn.md)；
窗口、授权与 OpenGL 问题见 [图形界面诊断](docs/gui_and_cli_simulation_cn.md)。

<a id="moveit-quick-start"></a>

## MoveIt 模拟

按上节准备容器与构建后，在容器执行：

```bash
cd /workspaces/hex_arm_ros2
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

该入口包含模拟硬件、轨迹控制器、`move_group` 和带 MotionPlanning 面板的 RViz，
使用 `mock_components/GenericSystem`，不连接机械臂。默认姿态为
`startup_ready = [0,-1.350,1.430,-0.300,0,0]` rad。

在 RViz 选择规划组 `arm` 和当前起始状态，使用 **Interact** 拖动末端。
先点 **Plan** 检查预览，再点 **Plan & Execute** 模拟执行。
停止时在 launch 终端按 Ctrl+C，等待子进程退出。

| 可选参数 | 用途 |
|---|---|
| `use_rviz:=false` | 无界面规划与模拟执行 |
| `limits_profile:=commissioning` | 体验保守低速限制；默认 `sim`，另可选 `verified` |

`limits_profile` 只改变模拟规划限位。物理效果需要 Gazebo；
详细操作、碰撞模型和限速配置见 [MoveIt 仿真指南](docs/moveit_simulation_cn.md)。

<a id="real-hardware"></a>

## MoveIt 真机

本节命令对应当前 can2 替换臂。使用本机配置
`config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml`，
其中保存电机身份、方向、零偏与运动范围。
`*.local.yaml` 不提交 Git；新检出仓库或换臂后，先按实物建立配置并核对标定，
见 [软件零点校准](docs/zero_calibration_cn.md)。

### 1. 检查入口姿态

先停止 GUI MIT、重力补偿及其他控制入口，同一时间仅保留一个真机控制程序和一个运动客户端。
每次断电重启后，在自动启动前按下表摆放；单位均为 rad：

| 姿态 | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| 折叠入口 `folded_position_rad` | 0 | **−1.570** | **1.570** | 0 | 0 | 0 |
| 启动完成 `startup_ready` | 0 | −1.350 | 1.430 | −0.300 | 0 | 0 |

绝对编码器会核对入口姿态。启动按 **J2 → J4 → J3** 执行：
J2 到 −1.350（8 s）、J4 到 −0.300（10 s）、J3 到 1.430（6 s）。
权威配置见 [startup.yaml](src/hex_arm_controller/config/startup.yaml)。

折叠摆放允许 J2/J3 相对参考各偏差 **0.01 rad（约 0.57°）**；本部署 profile
也配置了 `measured_position_margin_rad: 0.01`。因此 J2 可在 −1.580～−1.560 rad、
J3 可在 1.560～1.580 rad 进入启动。使能保持和轨迹起点会将边界外的微小偏差收回到
合法目标范围，实际反馈仍用于重力计算和故障检查；软零偏与正常运动限位保持原值。

### 2. 启动 MoveIt（宿主机完整命令）

**如果当前提示符是 `root@hex-arm-dev`，先执行 `exit` 返回宿主机。**
然后从宿主机图形终端执行：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh up
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml \
  enable_execution:=true \
  position_limits:=hardware \
  dynamics_limits:=custom \
  planning_limits_file:=/workspaces/hex_arm_ros2/src/hex_arm_moveit_config/config/joint_limits_deployment.yaml \
  align_folded:=true \
  allow_enable_transient:=true
```

此命令会使能并自动移动机械臂；保持终端运行。
profile 和规划 YAML 参数使用**容器内绝对路径**。
监督入口核对 CAN 适配器序列号与物理通道，并负责信号转发、审计日志和最终失能校验。

| 参数 | 作用 |
|---|---|
| `enable_execution:=true` | 使能真机，启动验收通过后允许轨迹执行 |
| `position_limits:=hardware` | 取 URDF 与硬件 profile 的位置范围交集 |
| `dynamics_limits:=custom` + `planning_limits_file:=...` | 取正式规划 YAML 与硬件的动态限值交集；速度还受 URDF 限制 |
| `align_folded:=true` | 允许基座及腕部进行有界折叠姿态对齐 |
| `allow_enable_transient:=true` | 允许使能和重力渐入时的短暂速度余量，随后仍需静止验收 |

无界面规划服务可追加 `use_rviz:=false`。无图形桌面的宿主机还需在 `up` 与
`real-launch` 两条命令前同时设置 `HEX_ARM_HEADLESS=1`，入口会自动关闭 RViz。
只观察和规划而不使能时，将 `enable_execution`、`align_folded`、
`allow_enable_transient` **同时改为 `false`**。

### 3. 等待验收，再规划执行

启动动作结束后还有 **3 秒 ready 静止验收**。
期间 MoveIt 执行动作接口尚未开放，RViz 等待验收通过后自动打开。
看到 `MoveIt startup verified: execution available` 后，在 RViz 选择 `arm`，
以当前状态为起点执行 **Plan** / **Plan & Execute**。无界面客户端也必须等待这条日志。
`startup_ready:=false` 不能绕过真机自动启动。

灰影规划预览默认按每个轨迹点 `0.05 s` 循环播放。在当前 RViz 窗口设置
**Displays → MotionPlanning → Planned Path → State Display Time = 0.05 s**，
保持 **Loop Animation** 勾选；若播放暂停，可取消后再次勾选以解除暂停。
修改 **Velocity Scaling** 或 **Accel Scaling** 后需重新 **Plan**。
灰影显示规划预览，固定播放时序不与真机速度同步；
查看实际反馈姿态请使用由 `/hex_arm/internal/state` 硬件反馈更新的 **Scene Robot**。

当前正式配置：

| 项目 | 数值或行为 |
|---|---|
| 位置范围 | J1 ±2.86；J2 [−1.57,2.09]；J3/J4 ±1.57；J5 ±1.54；J6 ±2.79 rad |
| 硬件速度 / 加速度 | 2.234021 rad/s / 1.256637 rad/s² |
| MoveIt 规划速度 / 加速度 | 2.234021 rad/s（128°/s）/ **0.9375 rad/s²** |
| Kp / Kd | Kp `[100,100,150,110,80,80]` N·m/rad；Kd 六轴 15 N·m·s/rad |
| 重力补偿 | 六轴比例 1.0；J2 前馈限幅 ±5 N·m |
| 扭矩预算 | 总上限与 PD 配置上限均为 1000‰；额外预留 0，`pd_allocation: remaining` |

前馈和 PD **共用总 100% 额度**；每帧 PD 使用前馈占用后剩余的额度，
独立的 N·m 限幅仍生效。旧 profile 未指定预算策略时仍使用 15% 预留和 `fixed`。
重力前馈连续按实测六轴姿态计算，涵盖 J2 的 0～1.57 rad 范围。

注意以下限速与单位关系：

- GUI 的 Rev/s、Rev/s² 分别乘 `2π`，才是 ROS / MoveIt / Rust 的 rad/s、rad/s²。
- 位置与动态默认均为 `commissioning`；仅切换位置或提高硬件速度，仍保留 0.1 rad/s、0.1 rad/s² 动态上限。
- RViz 速度和加速度缩放是额外比例，均设为 1.0 才能使用完整规划限值。
- Meow 开机展开、关机收拢和准备对齐沿用本次 MoveIt 的速度、加速度上限，按行程自动计算轨迹时长；关机返回 ready 固定使用 0.5/0.5 速度、加速度缩放。开机后的静止验收为 3 秒。
- 旧 `replacement.local.yaml` 保留窄窗口；日常部署使用上方命令的 `moveit_deployment.local.yaml`。

2026-09-30 已通过三个大范围 MoveIt 组合目标、返回 ready、受控回折及确认失能，
当时总扭矩上限为 650‰。当前 100% 与动态 PD 分配已通过离线校验，尚未重新做真机动作复测；
完整 URDF 边界及附加载荷未穷举验收。
当前规划速度在 96°/s 的基础上再次提高 1/3，加速度在 0.75 rad/s² 的基础上再次提高 1/4；这组更高限值尚未进行真机动作复测。
配置细节与实测见 [替换臂部署说明](docs/meow_replacement_deployment_cn.md)和
[验收记录](docs/commissioning_evidence/2026-09-30-can2-moveit-deployment.md)。

### 4. 正常停止

在拥有 `real-launch` 的终端第一次按 **Ctrl+C**，机械臂先经 MoveIt 返回 ready，
再按 **J3 → J4 → J2** 受控回折并确认失能。
关机返回 ready 固定使用 **0.5/0.5** 速度、加速度缩放。
再次 Ctrl+C 或故障走立即停机路径。
返回 ready 和回折的关机轨迹逐条使用 **0.1 rad 路径跟踪容差**；最终到位仍需满足 0.015 rad
位置误差及 0.02 rad/s 静止检查。返回轨迹经 MoveIt 严格规划和逐点检查后交给 FJT 控制器执行。

等待回折结果与 `VERIFIED structured disabled_confirmed` 后再断电。
保留本次 `startup-ready.json`、`launch.log`、`driver-shutdown.json` 路径；
后续停止工具需要本次启动报告。额外 ROS 终端须使用同一容器和相同 `ROS_DOMAIN_ID`。
更换 CAN 接口见 [接口绑定说明](docs/meow_mit_deployment_cn.md#更换-can-接口)。

<a id="hand-guiding"></a>

## 手扶拖动：重力补偿与阻尼

本模式使用 `Kp=0`、重力前馈与速度阻尼，不执行展开或轨迹跟踪。
先停止其他控制程序，在已构建的容器工作区执行：

```bash
cd /workspaces/hex_arm_ros2
source install/setup.bash
ros2 launch hex_arm_bringup gravity_comp.launch.py \
  hardware_profile:=/workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.hand_guiding_full_range.local.yaml \
  activate_hardware:=true \
  damping:='[2.0, 2.0, 2.0, 2.0, 2.0, 2.0]'
```

使能及重力渐入期间扶稳机械臂，等待 `hand_guiding_ready` 后拖动。
阻尼使运动减速，但不会锁定位置；本模式没有 MoveIt 碰撞检查。
停止前先扶稳，在另一个已加载环境的容器终端执行：

```bash
ros2 service call /hex_arm_gravity_comp/stop std_srvs/srv/Trigger '{}'
```

Ctrl+C 也会停止，但不会自动回位或折叠。
阻尼调节、反馈余量、速度保护和失能回执见 [手扶拖动操作说明](docs/hand_guiding_cn.md)。

<a id="simulation-modes"></a>

## 其他模拟模式

以下命令均在已构建并加载环境的**容器终端**执行，一次选择一种模式：

| 模式 | 功能 |
|---|---|
| `view` | RViz + 关节滑块，仅预览模型 |
| `mock` | ros2_control 模拟硬件，支持轨迹 action |
| `gz` | Gazebo Harmonic 物理仿真 |

```bash
# URDF 与关节滑块
ros2 launch hex_arm_bringup view.launch.py

# 普通 mock
ros2 launch hex_arm_bringup mock.launch.py use_rviz:=true

# Gazebo
ros2 launch hex_arm_bringup gz.launch.py headless:=false use_rviz:=true
```

这些入口没有 MotionPlanning 面板。`view` 使用独立命名空间；滑块单位为 rad，
Centre 将 J3 设为模型 v2 的 0。使用完毕后在启动终端按 Ctrl+C。

<a id="simulation-cli"></a>

### 命令行发送模拟轨迹

先启动 `mock` 或 `gz`，在另一个已加载环境的容器终端发送：

```bash
ros2 action send_goal /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, -0.32, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}"
```

成功时返回 `SUCCEEDED`、`error_code: 0`。目标必须包含六轴，角度以 rad 填写，
多点轨迹的 `time_from_start` 必须递增。该接口不经过 MoveIt 碰撞检查，
mock/gz 也不会自动拦截超限命令。Ctrl+C 仅退出发送目标的 CLI 客户端。
完整字段说明和 Python 示例见 [命令行模拟指南](docs/gui_and_cli_simulation_cn.md)。

<a id="development"></a>

## 开发与测试

### 控制接口与架构

MoveIt 通过 `/firefly_arm_controller/follow_joint_trajectory` 的标准
`control_msgs/action/FollowJointTrajectory` 接口执行轨迹：

```text
MoveIt
  -> ros2_control / firefly_arm_controller (100 Hz)
  -> hex_arm_hardware (C++)
  -> hex_arm_bridge (Python, ROS <-> Zenoh / Protobuf)
  -> hex_arm_controller (Rust, 500 Hz)
  -> SocketCAN / CAN-FD / Meow MIT 电机
```

MoveIt 检查碰撞，轨迹控制器检查跟踪误差，Rust 负责总线、插值、坐标换算、
重力补偿及底层保护。模块职责、CiA402 与历史工具构建见
[运行架构与迁移](docs/architecture_refactor_cn.md)。

### 测试级别

根据改动选择测试，在已构建并加载环境的容器中执行：

| 命令 | 范围 |
|---|---|
| `./scripts/test.sh unit` | 无硬件单元测试 |
| `./scripts/test.sh protocol` | mock 电机、Zenoh 与 ROS 桥接 |
| `./scripts/test.sh mock` | 轨迹 action 与控制器生命周期 |
| `./scripts/test.sh gz` | 无界面 Gazebo 轨迹 |
| `python3 src/hex_arm_moveit_config/test/test_moveit_mock.py` | 无界面 MoveIt 规划、碰撞检查和模拟执行 |

### 可复现性

`hex_arm.repos` 固定上游修订，检入的描述包保留原始网格和来源记录。
仅更新或审计上游时运行 `vcs import . < hex_arm.repos`；日常构建使用检入源码。
保留 `install/` 及其在 `build/` 中的链接目标。

<a id="troubleshooting"></a>

## 常见问题

| 现象 | 处理方法 |
|---|---|
| `docker is not installed or is not on PATH`，提示符为 `root@hex-arm-dev` | 执行 `exit` 返回宿主机，再运行 `docker-dev.sh`；若已在宿主机则检查 Docker 和 PATH |
| `ros2: command not found` 或找不到包 | 进入 `ros2-jazzy-arm`，加载 `install/setup.bash`；缺少构建时先运行 `build.sh` |
| source 报路径缺失 | 核对容器及挂载；不要在宿主机或旧容器复用本容器生成的 symlink-install |
| 窗口不出现、显示授权错误 | 在宿主机图形终端运行 `docker-dev.sh doctor`；按图形诊断文档处理 |
| 只有模型/滑块，没有 MotionPlanning | `view` 仅预览；规划使用 MoveIt mock 或上方真机入口 |
| 真机启动时 RViz 尚未打开 | 等待启动和 3 秒静止验收；失败时查看本次 `startup-ready.json` 的 `passed/error` |
| `another supervised real launch already owns can2` | 在原启动终端完成正常停止 |
| `Meow torque ceiling lacks PD/gravity headroom` | 核对是否用了旧 profile 或旧驱动；当前部署需 `remaining` 预算和已重新构建的 controller |
| 停机尾部出现 `context is invalid` 或 `process has died` | 查看完整日志、启动报告及 `driver-shutdown.json` 的 `disabled_confirmed`，不能仅凭尾部 ROS 错误判断 |
| J2 滑块与电机 Rev 正负相反 | 本臂方向为 −1，`q = −2π × motor_rev + zero_offset_rad`；滑块显示 ROS 角度 |
| J3 与旧记录相差约 1.57 rad | 模型 v2 使用 `q_v2 = q_v1 − 1.57`；profile 须为 schema v3、`joint_coordinate_version: 2` |
| Plan 失败或目标碰撞、超限 | 从当前状态选择可达且无碰撞的目标；`commissioning_start` 只是规划参考 |

同一个 `ROS_DOMAIN_ID` 下避免同时启动多套机械臂 launch。
启动门控与退出日志细节见 [启动门控记录](docs/commissioning_evidence/2026-09-30-moveit-startup-gate.md)。

<a id="documentation"></a>

## 文档索引

| 文档 | 用途 |
|---|---|
| [替换臂 Meow 部署](docs/meow_replacement_deployment_cn.md) | 当前真机配置、动态限位、预算与受控停止 |
| [Meow MIT 与 CAN 绑定](docs/meow_mit_deployment_cn.md) | 协议单位、接口迁移与 profile |
| [手扶拖动操作说明](docs/hand_guiding_cn.md) | 重力补偿、阻尼、反馈保护与停止 |
| [软件零点校准](docs/zero_calibration_cn.md) | 换臂或重新标定 |
| [MoveIt 仿真](docs/moveit_simulation_cn.md) | 规划窗口、碰撞模型与限位配置 |
| [图形界面与命令行模拟](docs/gui_and_cli_simulation_cn.md) | GUI 排查、action 与 Python 示例 |
| [Docker 开发环境](docs/docker_development_cn.md) | Compose、NVIDIA 安装与 WSL2 |
| [运行架构与迁移](docs/architecture_refactor_cn.md) | 模块职责、构建与独立部署 |
| [当前实机状态与后续优化](docs/commissioning_cn.md) | 历史调试过程与后续计划 |
| [旧版 CiA402 部署](docs/cia402_deployment_cn.md) | 升级前固件的部署与验收 |
| [验收记录索引](docs/commissioning_evidence/README.md) | 各轮试验结果与原始证据 |

历史 CiA402 记录属于 Meow 升级前的结果；当前机械臂以本臂 Meow 配置与验收记录为准。
