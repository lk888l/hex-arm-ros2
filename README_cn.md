# Firefly Y6 ROS 2 驱动
[English (英文版)](README.md)


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

## Docker 开发环境启动

### 本地 Ubuntu 24.04（当前机器）

“本地 Ubuntu”是指电脑直接安装并启动 Ubuntu，而不是在 Windows 中运行 Ubuntu。
当前仓库路径 `/home/kk/kk_data/ros2_project/hex-arm-ros2` 属于这种情况。在 Ubuntu
桌面的终端中执行：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh build
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

`build` 只需在首次使用或 Dockerfile 改动后执行。以后日常启动通常只需要：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

脚本在本地 Ubuntu 中自动使用 `compose.ubuntu.yaml`，只读传入当前 X11 授权
cookie，并映射 `/dev/dri`，以便 RViz、MoveIt 和 Gazebo 弹出图形窗口。

#### 不使用辅助脚本：原生 Docker Compose 命令

`docker compose` 是 Docker 自带的 Compose CLI。以下命令与本地 Ubuntu 下的
`docker-dev.sh` 等价。必须从 Ubuntu 图形桌面的终端执行，并在当前终端设置
`HEX_ARM_XAUTHORITY`：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2

# 优先使用桌面会话提供的 XAUTHORITY；未提供时回退到 ~/.Xauthority
if [[ -n "${XAUTHORITY:-}" ]]; then
  export HEX_ARM_XAUTHORITY="$XAUTHORITY"
else
  export HEX_ARM_XAUTHORITY="$(getent passwd "$(id -u)" | cut -d: -f6)/.Xauthority"
fi

# 必须打印 GUI prerequisites: OK；否则先检查 DISPLAY、Xauthority 或 /dev/dri
test -n "${DISPLAY:-}" \
  && test -r "$HEX_ARM_XAUTHORITY" \
  && test -e /dev/dri \
  && echo "GUI prerequisites: OK"

# 首次使用或 Dockerfile 改动后构建
docker compose -f compose.ubuntu.yaml build

# 后台启动容器
docker compose -f compose.ubuntu.yaml up -d

# 可选：检查容器能否连接 X11，并显示 OpenGL 渲染器
docker compose -f compose.ubuntu.yaml exec -T ros2-jazzy-arm \
  bash -lc 'xdpyinfo >/dev/null && glxinfo -B'

# 进入 ROS 2 容器
docker compose -f compose.ubuntu.yaml exec ros2-jazzy-arm bash
```

退出容器后，可在同一个已经设置 `HEX_ARM_XAUTHORITY` 的宿主机终端查看日志或
停止容器：

```bash
docker compose -f compose.ubuntu.yaml logs -f
docker compose -f compose.ubuntu.yaml down
```

每次新开宿主机终端，都要重新设置 `HEX_ARM_XAUTHORITY`，再执行上述 Compose
命令。请不要把本地 Ubuntu 的 `compose.ubuntu.yaml` 换成 WSL2 使用的
`compose.yaml`，也不需要执行 `xhost +`。

#### NVIDIA 独立显卡加速

本地 Ubuntu 上，脚本默认使用 `HEX_ARM_GPU=auto`：检测到可用 NVIDIA GPU 时
自动叠加 `compose.nvidia.yaml`；没有 NVIDIA GPU 时保持 `/dev/dri` 通用路径。
首次使用 NVIDIA 容器前，需在宿主机安装 NVIDIA Container Toolkit（不会在
Dockerfile 中安装宿主机显卡驱动）：

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor --yes \
      -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Docker 重启会停止当时正在运行的全部容器，但不会停止宿主机直接运行的 CUDA
进程。安装完成后重新创建并检查本项目容器：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
```

也可以显式选择模式：

```bash
# 强制 NVIDIA；缺少 GPU 或 Toolkit 时立即报错
HEX_ARM_GPU=nvidia ./scripts/docker-dev.sh up

# 禁用 NVIDIA override，使用 AMD/Intel DRI 或软件渲染
HEX_ARM_GPU=none ./scripts/docker-dev.sh up
```

不使用辅助脚本时，先按上一节设置 `HEX_ARM_XAUTHORITY`，再显式叠加 NVIDIA
override：

```bash
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml up -d
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml \
  exec -T ros2-jazzy-arm nvidia-smi
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml \
  exec -T ros2-jazzy-arm glxinfo -B
```

`doctor` 成功时应显示 RTX 型号、`OpenGL renderer string: NVIDIA ...` 和
`NVIDIA GPU acceleration: OK`。若仍显示 `llvmpipe`，则是 CPU 软件渲染。
镜像已经包含 GLVND/OpenGL 用户态依赖；不要把 NVIDIA 内核驱动或宿主机驱动包
写入 Dockerfile。若同时进行 LeRobot 等 CUDA 训练，RViz/Gazebo 会与训练任务
共享显存和算力，建议错峰运行。

### WSL2（仅 Windows 10/11）

WSL2 是 **Windows Subsystem for Linux 2**，即 Windows 内置的 Linux 虚拟化
环境。Ubuntu 若是从 Windows 中启动、`uname -r` 的输出包含
`microsoft-standard-WSL2`，才属于 WSL2；它通过 WSLg 显示 Linux 图形窗口。

必须打开 Windows 中的 **Ubuntu/WSL 终端**执行以下命令，不要在 PowerShell 或
CMD 中执行。仓库路径不同则替换第一行：

```bash
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2
./scripts/docker-dev.sh build
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

同一个辅助脚本会在 WSL2 中自动使用 `compose.yaml` 和 WSLg socket。两种环境都
不需要执行权限过宽的 `xhost +`。现有镜像本身已经是 Ubuntu 24.04（ROS 2
Jazzy），因此无需再复制维护一份内容相同的 Ubuntu Dockerfile。

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

在启动另一种模式之前，请先按 `Ctrl-C` 停止当前 launch。如果 `source
install/setup.bash` 报错称 `/workspaces/hex_arm_ros2` 下的路径缺失，说明当前
所在的容器不对；请在宿主机执行 `./scripts/docker-dev.sh shell` 进入正确容器。

## 用命令行驱动模拟机械臂（`ros2 action send_goal` 详解）

mock / gz 模式启动后，在另一个终端（已 `source install/setup.bash`）执行：

```bash
ros2 action send_goal /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, 1.25, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}"
```

### 命令格式

```text
ros2 action send_goal <action_server> <action_type> "<goal_yaml>"
```

| 参数 | 本例取值 | 说明 |
|---|---|---|
| 子命令 | `ros2 action send_goal` | 发送 action goal 的 CLI 子命令；`ros2 action` 还支持 `list`、`info`、`type` |
| action server | `/firefly_arm_controller/follow_joint_trajectory` | 轨迹控制器暴露的 action 服务端。必须先启动 mock/gz launch，否则 CLI 会一直显示 waiting |
| action 类型 | `control_msgs/action/FollowJointTrajectory` | 决定 YAML 的解析方式，必须与工程一致 |
| goal 内容 | 双引号包裹的 YAML | 字段规则见下 |

### YAML 字段规则

- `trajectory.joint_names`：关节名列表。**顺序固定**为 `joint_1` ~ `joint_6`，且必须 6 个全部给出（控制器配置了 `allow_partial_joints_goal: false`，缺关节会被拒绝）。
- `trajectory.points`：轨迹点数组，本例只给 1 个点。每个点包含：
  - `positions`：目标角度，单位**弧度（rad）**，数量必须等于 6 且按 `joint_names` 顺序。建议保持在 URDF 限位内：joint_1 ±2.86、joint_2 −1.57~2.09、joint_3 0~3.14、joint_4 ±1.57、joint_5 ±1.54、joint_6 ±2.79。注意 mock/gz 配置**未启用命令限位拦截**，超限位置不会被自动钳制，请自行确保数值安全。
  - `time_from_start`：相对目标被接受时刻的时间偏移，`{sec: 2, nanosec: 0}` 表示 2 秒内到达；控制器使用 `interpolation_method: splines`（样条插值）平滑运动。
  - 可选字段：`velocities`、`accelerations`、`effort`；不填时由控制器自行插值。
  - 可以放多个点组成多段轨迹，每段的 `time_from_start` 递增即可。

### Shell 与使用细节

- 整段 YAML 用**双引号**包住，防止空格被拆成多个 shell 参数；行尾的 `\` 是续行符，全部写成一行也可以。
- 先 source 环境（交互式 bash 会自动加载；否则手动执行 `source /opt/ros/jazzy/setup.bash` 与 `source install/setup.bash`）。
- 发送后应看到 `Goal accepted with ID: ...`；执行完成输出 `Goal finished with status: SUCCEEDED`（`error_code: 0`）。
- 验证实际到达位置：`ros2 topic echo --once /joint_states`，`position` 应与目标一致。
- 再发一个新 goal 会取消/替换正在执行的旧 goal（控制器默认行为）；`Ctrl-C` 只结束 CLI 客户端本身。
- gz 模式使用同样的命令；gz 走仿真时间，轨迹按 Gazebo 时钟推进。

更多图形界面与命令行排查见
[docs/gui_and_cli_simulation_cn.md](docs/gui_and_cli_simulation_cn.md)。


real 配置故意做成独立的 Compose override。它只映射 USB 总线，并且从不启用 Docker 特权模式：

```bash
HEX_ARM_REAL=1 ./scripts/docker-dev.sh up
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
