# Firefly Y6 ROS 2 驱动

[English](README.md) | **中文**

框架重构与独立 Docker 部署见 [运行架构与迁移说明](docs/architecture_refactor_cn.md)。
默认构建已排除历史 CiA402/USB 调试工具；真机执行每次自动完成 J2 → J4 → J3。

面向六轴 Firefly Y6 机械臂的 ROS 2 Jazzy 驱动、MoveIt 2 运动规划与 Gazebo 仿真工作区。
**只想打开 MoveIt 2 窗口，直接按下面的快速启动操作即可，无需连接机械臂。**

导航：[MoveIt 2 快速启动](#moveit-quick-start) · [其他模拟模式](#simulation-modes) ·
[命令行驱动](#simulation-cli) · [常见问题](#troubleshooting) ·
[Docker 进阶配置](#docker-setup) · [真机入口](#real-hardware) · [文档索引](#documentation)

<a id="moveit-quick-start"></a>

## MoveIt 2 模拟窗口：快速启动

此入口打开的是 **RViz + MotionPlanning 面板**，支持逆运动学、碰撞检测、轨迹规划和
模拟执行。底层使用 ros2_control 的 `mock_components/GenericSystem`；不访问 USB/CAN
硬件，也不模拟重力、接触等物理效果。需要物理仿真时使用后面的 Gazebo 模式。

### 1. 在宿主机终端进入容器

本地 Ubuntu 24.04：从图形桌面的终端执行。首次使用或 Dockerfile 改动后，先在仓库
根目录运行 `./scripts/docker-dev.sh build` 构建镜像；日常启动只需：

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

仓库位置不同则替换第一行。WSL2 用户在 **Ubuntu/WSL 终端**中执行同一组脚本，
不要从 PowerShell/CMD 启动；脚本会自动选择 WSLg 配置。
容器已经运行时可直接执行 `./scripts/docker-dev.sh shell`。

### 2. 在容器内启动 MoveIt 2

正确容器名为 `ros2-jazzy-arm`，提示符主机名为 `hex-arm-dev`。
首次使用或源码改动后，在 `/workspaces/hex_arm_ros2` 下先运行 `./scripts/build.sh`
构建工作区。已构建时，每次新开容器终端执行：

```bash
cd /workspaces/hex_arm_ros2
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

这里 `hex_arm_moveit_config` 是 ROS 包名，`moveit_mock.launch.py` 是该包的启动文件。
它会依次加载模拟硬件、关节状态广播器和 `firefly_arm_controller`，随后启动
`move_group` 与带 MoveIt 配置的 RViz。**这一条 launch 已包含所需控制器，不必再运行
`hex_arm_bringup mock.launch.py`。** 保持这个终端运行，桌面会出现 RViz 窗口。

此模拟入口的默认关节姿态为 `[0,-1.350,1.430,-0.300,0,0]` rad，命名状态为
`startup_ready`。它只设置 mock 关节的初始显示，不是断电重启前的实机摆放姿态；
真机按绝对编码器反馈启动。
Firefly Y6 描述 v2 已把 J3 的机械/CAD 零位设为 `0`。旧坐标按
`q_v2 = q_v1 - 1.57 rad` 换算；硬件 profile 必须使用 schema v3，并包含
`joint_coordinate_version: 2`。

### 3. 在窗口里规划并模拟执行

1. 在 **MotionPlanning** 中选择规划组 `arm`，以当前机器人状态作为起点。
2. 使用 RViz 的 **Interact** 工具拖动末端交互标记，设置目标位置和姿态。
3. 点击 **Plan** 检查轨迹预览；规划失败时先检查目标是否可达、有无碰撞或超限。
4. 点击 **Plan & Execute** 规划并执行，观察模拟机械臂与关节状态更新。

`Plan` 只生成预览；`Plan & Execute` 在此 mock 入口下驱动模拟关节。
配置中的 `commissioning_start` 仅是规划参考，不是已标定的真机 home，也不保证
在严格碰撞模型下可以规划到达。

### 常用启动参数

参数写在 launch 文件名之后，格式为 `参数名:=值`。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_rviz` | `true` | 自动打开 RViz；设为 `false` 可运行无界面的规划与模拟执行 |
| `limits_profile` | `sim` | MoveIt 速度/加速度限位配置，可选 `sim`、`commissioning`、`verified`；日常模拟使用 `sim` |

```bash
# 无界面运行 MoveIt mock
ros2 launch hex_arm_moveit_config moveit_mock.launch.py use_rviz:=false

# 在模拟窗口中体验保守低速限位
ros2 launch hex_arm_moveit_config moveit_mock.launch.py limits_profile:=commissioning

# 查看启动参数，不启动节点
ros2 launch hex_arm_moveit_config moveit_mock.launch.py --show-args
```

选择 `limits_profile` 只改变规划限位，后端始终是 mock。各配置的具体数值与碰撞模型
说明见 [MoveIt 仿真指南](docs/moveit_simulation_cn.md)。

停止时在 launch 终端按 `Ctrl+C`，等待所有子进程退出。切换模式前先停止当前 launch；
同一个 `ROS_DOMAIN_ID` 下不要同时启动多套机械臂 launch，以免控制器、TF 和关节状态冲突。

<a id="simulation-modes"></a>

## 其他模拟模式：按用途选择

以下命令均在**已经构建并 source 的容器终端**中执行，一次选择一种模式。
MoveIt mock 使用 `mock` 后端；它与普通 mock 的区别是额外提供 MoveIt 规划服务和面板。

| 模式 | 窗口与功能 | 适合做什么 |
|---|---|---|
| **MoveIt mock**（上文） | RViz + MotionPlanning + 模拟控制器 | 拖动末端、规划、模拟执行 |
| `view` | RViz + 关节滑块，无轨迹控制器 | 检查 URDF、关节方向和限位 |
| 普通 `mock` | RViz + ros2_control 模拟硬件，无 MoveIt 面板 | 测试轨迹 action 和控制器生命周期 |
| `gz` | Gazebo Harmonic + 可选 RViz，无 MoveIt 面板 | 物理仿真与轨迹控制 |

```bash
# URDF + 关节滑块 + RViz
ros2 launch hex_arm_bringup view.launch.py

# 普通 mock：通过命令行或脚本发送关节轨迹
ros2 launch hex_arm_bringup mock.launch.py use_rviz:=true

# Gazebo 物理仿真 + RViz
ros2 launch hex_arm_bringup gz.launch.py headless:=false use_rviz:=true
```

这三条命令都不会加载 MoveIt MotionPlanning 面板。需要规划窗口时，使用上文的
`ros2 launch hex_arm_moveit_config moveit_mock.launch.py`。

每次 `view` 启动都会使用独立的 `/hex_arm_view_<id>` 命名空间，隔离关节状态、
模型描述和 TF，避免多个预览窗口或控制器互相干扰。滑块单位为弧度；Centre
会将 J3 设为 0（对应修改前的 1.57 rad 姿态）。使用完毕后在启动终端按 Ctrl+C 退出。

<a id="simulation-cli"></a>

## 用命令行驱动模拟机械臂（`ros2 action send_goal` 详解）

先启动上文的普通 `mock` 或 `gz` 模式。另开一个宿主机终端，通过
`./scripts/docker-dev.sh shell` 进入同一个容器，然后执行：

```bash
cd /workspaces/hex_arm_ros2
source install/setup.bash
ros2 action send_goal /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, -0.32, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}"
```

执行成功时应看到 `Goal finished with status: SUCCEEDED`（`error_code: 0`）。
用 `ros2 topic echo --once /joint_states` 检查反馈。此命令直接给轨迹控制器发送目标，
不会经过 MoveIt 的规划和碰撞检查；六个关节角以弧度填写，并遵守 URDF 限位。

<details>
<summary>展开：action 命令格式、轨迹字段与使用细节</summary>

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

- `trajectory.joint_names`：关节名列表。本例按 `joint_1` ~ `joint_6` 排列，必须 6 个全部给出（控制器配置了 `allow_partial_joints_goal: false`，缺关节会被拒绝）。
- `trajectory.points`：轨迹点数组，本例只给 1 个点。每个点包含：
  - `positions`：目标角度，单位**弧度（rad）**，数量必须等于 6 且按 `joint_names` 顺序。建议保持在 URDF 限位内：joint_1 ±2.86、joint_2 −1.57~2.09、joint_3 ±1.57、joint_4 ±1.57、joint_5 ±1.54、joint_6 ±2.79。注意 mock/gz 配置**未启用命令限位拦截**，超限位置不会被自动钳制，请自行确保数值安全。
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

</details>

<a id="troubleshooting"></a>

## 常见问题

| 现象 | 处理方法 |
|---|---|
| `ros2: command not found` | 从宿主机用 `./scripts/docker-dev.sh shell` 进入容器；容器内执行 `source /opt/ros/jazzy/setup.bash` 和 `source /workspaces/hex_arm_ros2/install/setup.bash` |
| 找不到 `hex_arm_moveit_config` 或 `hex_arm_moveit_runtime` | 在正确容器的 `/workspaces/hex_arm_ros2` 下执行 `./scripts/build.sh`，成功后重新 `source install/setup.bash` |
| source 报 `/workspaces/hex_arm_ros2/...` 路径缺失 | 确认进入的是 `ros2-jazzy-arm`（主机名 `hex-arm-dev`）；不要在旧 `ros2-jazzy` 容器或宿主机复用容器生成的 symlink-install |
| 窗口不出现，或报 `could not connect to display` / `Authorization required` | 在**宿主机图形终端、仓库根目录**运行 `./scripts/docker-dev.sh doctor`；修复后重新进入容器并启动。不要使用 `xhost +` |
| 只有模型/滑块，没有 MotionPlanning | 停止当前 launch，改用 `hex_arm_moveit_config moveit_mock.launch.py`，并保留 `use_rviz:=true` |
| 控制器重名、关节状态跳动或启动卡在控制器阶段 | 检查是否同时运行了其他机械臂 launch；在各自的 launch 终端正常退出后，只保留一个模式 |
| Plan 失败、目标显示碰撞或超限 | 从当前状态设置一个可达、无碰撞的目标；不要把 `commissioning_start` 当作必定有效的初始演示目标 |

更多 X11、WSLg、NVIDIA 与 CLI 排查见
[图形界面和命令行模拟指南](docs/gui_and_cli_simulation_cn.md)。

<a id="docker-setup"></a>

## Docker 开发环境（进阶）

日常启动使用前面的 `docker-dev.sh` 即可。本地 Ubuntu 自动选择
`compose.ubuntu.yaml`，并只读传入当前 X11 授权 cookie；WSL2 自动选择
`compose.yaml` 与 WSLg socket。`HEX_ARM_GPU=auto` 会在检测到可用 NVIDIA GPU 时
叠加 `compose.nvidia.yaml`，需要宿主机已安装 NVIDIA Container Toolkit。

<details>
<summary>原生 Docker Compose 命令</summary>

### 不使用辅助脚本：原生 Docker Compose 命令

`docker compose` 是 Docker 自带的 Compose CLI。以下命令对应本地 Ubuntu 的
AMD/Intel DRI 路径；NVIDIA 主机还需叠加下一项中的 override。
必须从 Ubuntu 图形桌面的终端执行，并在当前终端设置 `HEX_ARM_XAUTHORITY`：

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

</details>

<details>
<summary>NVIDIA 独立显卡：安装与验证</summary>

### NVIDIA 独立显卡加速

本地 Ubuntu 上，脚本默认使用 `HEX_ARM_GPU=auto`：检测到可用 NVIDIA GPU 时
自动叠加 `compose.nvidia.yaml`；没有 NVIDIA GPU 时保持 `/dev/dri` 通用路径。
NVIDIA override 会清除基础 device 映射，因此仅有 NVIDIA 设备节点的宿主机不
需要 `/dev/dri`，包括 `use_rviz:=false` 的 real-launch；`HEX_ARM_GPU=none` 仍
要求 `/dev/dri`。
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

override 使用标准 Compose `!reset` 标签；若 `docker compose config` 无法识别
该标签，请先升级 Docker Compose 插件。

`doctor` 成功时应显示 RTX 型号、`OpenGL renderer string: NVIDIA ...` 和
`NVIDIA GPU acceleration: OK`。若仍显示 `llvmpipe`，则是 CPU 软件渲染。
镜像已经包含 GLVND/OpenGL 用户态依赖；不要把 NVIDIA 内核驱动或宿主机驱动包
写入 Dockerfile。若同时进行 LeRobot 等 CUDA 训练，RViz/Gazebo 会与训练任务
共享显存和算力，建议错峰运行。

</details>

<details>
<summary>WSL2 启动说明</summary>

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

</details>

<a id="real-hardware"></a>

## 当前真机部署

当前使用 **Firefly Y6、Meow 固件、can2，无夹爪及附加载荷**。六轴通信、重力补偿、
J2→J4→J3 顺序启动和 MoveIt 小范围执行已验证；完整行程、较高速度和负载工况仍待验收。
完整参数、已知待回归问题与下一阶段任务统一维护在
[当前实机状态与后续优化](docs/commissioning_cn.md)。

### 启动与停止

本机配置为 `config/hardware/firefly_y6.meow.can2.local.yaml`，包含已采用的电机身份、
方向、零偏和运动窗口。该文件不提交 Git；换机时从 `firefly_y6.meow_mit.example.yaml`
建立新配置并重新核对标定，不能直接套用本机配置。

每次断电后重新上电，并在执行自动启动前，先将机械臂放在折叠入口姿态：

| 姿态 | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| 断电重启摆放位置 `folded_position_rad` | 0 | **−1.570** | **1.570** | 0 | 0 | 0 |
| 自动启动完成位置 `startup_ready` | 0 | −1.350 | **1.430** | −0.300 | 0 | 0 |

以上单位均为 rad。绝对编码器会在使能前核对折叠入口位置；自动启动仍严格按照
**J2 → J4 → J3** 的顺序执行。随后在宿主机仓库目录的图形终端执行：

```bash
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh up
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.local.yaml \
  enable_execution:=true
```

入口核对反馈和位置后使能，默认依次移动 **J2→−1.350（8 s）、J4→−0.300（10 s）、
J3→1.430（6 s）**，其余关节最终为 0。权威数值位于
[`src/hex_arm_controller/config/startup.yaml`](src/hex_arm_controller/config/startup.yaml)，
日常操作说明位于[当前实机状态与后续优化](docs/commissioning_cn.md#日常启动与停止)。
该顺序退出路径由操作者确认实物无碰撞。
等待 `startup_ready reached and verified; controller continues holding` 后，
在 RViz 选择 `arm`，以当前状态为起点进行 `Plan` / `Plan & Execute`。
后续 MoveIt 执行使用严格碰撞检查，并受当前本机运动窗口约束。

当前 Kp 为 `[80,80,120,110,80,80]`，Kd 均为 15；J2/J3/J4 重力比例为 `1.0/1.05/0.7`。
速度和加速度上限分别为 0.1 rad/s、0.1 rad/s²；具体限位见部署进度文档。
真机执行每次必须完成自动启动顺序，不能用 `startup_ready:=false` 绕过；
省略 `enable_execution:=true` 为保持失能的观察和规划。

当前本机 profile 已配置退出阻尼：在拥有 real-launch 的终端第一次按 Ctrl-C，
会先经 MoveIt **回安全启动位**，再阻尼下落、确认失能；不要提前断电。
再次 Ctrl-C 或故障走立即停机路径。等待柔和阶段结果及 `VERIFIED structured disabled_confirmed`。
该阻尼参数尚待实机验收，入口限制、失败行为和参数见[退出阻尼说明](docs/shutdown_damping_cn.md)。
切换上位机、修改配置或重新编译前，先停止当前控制端。

### CAN 接口与继续开发

CAN 名称可配置；当前使用 can2，接口速率为 1 Mbps 仲裁 / 4 Mbps 数据。
选择接口时同时校验 USB 适配器序列号和物理通道。更换接口后，使用
`scripts/bind-can-profile.py` 生成保留身份与标定的新 profile，再通过
`HEX_ARM_CAN_IFACE` 选择对应接口；完整命令见 [Meow MIT 部署说明](docs/meow_mit_deployment_cn.md#更换-can-接口)。

清理后保留 `install/` 及其在 `build/` 中的必要链接目标。修改源码后，先停止控制，
在容器工作区执行 `./scripts/build.sh`，再 `source install/setup.bash`；编译缓存会重新生成。
后续按重复性回归、模型与限位核对、逐步扩大运动范围、连续运行与故障恢复、部署固化的顺序推进，
详见 [后续优化步骤](docs/commissioning_cn.md#后续优化顺序)。

## 控制接口与架构

MoveIt 规划轨迹后，通过标准 `control_msgs/action/FollowJointTrajectory` action
交给 `joint_trajectory_controller`，接口为 `/firefly_arm_controller/follow_joint_trajectory`。
当前真机链路为：

```text
MoveIt / FollowJointTrajectory
  -> ros2_control + firefly_arm_controller（100 Hz）
  -> hex_arm_hardware/SystemInterface（C++ 硬件插件）
  -> hex_arm_bridge（Python，ROS <-> Zenoh / Protobuf）
  -> hex_arm_controller（Rust，500 Hz 电机指令循环）
  -> SocketCAN（当前 can2，可配置）-> CAN-FD / Meow MIT 电机
```

- **ros2_control** 管理控制器与硬件生命周期，并循环执行读取状态、更新轨迹控制器、写入目标。
  `joint_trajectory_controller` 根据轨迹时间生成关节目标，检查跟踪和到位误差。
- **硬件插件** 把六轴位置/速度命令以及位置/速度/力矩反馈接入 ros2_control，
  通过 ROS 接口与桥接层交换数据。
- **桥接层** 在 ROS 与 Rust 的 Zenoh / Protobuf 接口之间转换指令和反馈，
  协调生命周期及独占控制会话；它不另行实现轨迹 action。
- **Rust 驱动** 独占电机总线，完成指令插值、方向与零偏换算、重力补偿、MIT 数据发送和反馈接收，
  并执行限位、输出限制及指令/反馈超时保护。

碰撞检查由 MoveIt 承担，轨迹误差检查由轨迹控制器承担，电机通信与底层保护由 Rust 承担。

## 可复现性

`hex_arm.repos` 固定了上游源码的修订版本。仓库中检入的 `xpkg_urdf_firefly_y6` 包是指定描述修订版本的可溯源快照（provenance-preserving snapshot），包含原始网格文件。仅在更新或审计上游源码时运行：

```bash
vcs import . < hex_arm.repos
```

常规构建不会静默拉取可变分支。

## 测试级别

以下命令在已经构建并 source 的容器终端中执行：

```bash
./scripts/test.sh unit
./scripts/test.sh protocol
./scripts/test.sh mock
./scripts/test.sh gz
```

`unit` 不依赖硬件。`protocol` 使用 mock 电机后端启动 Rust 控制器，并验证 Zenoh
发现/事件以及 ROS 生命周期桥接。`mock` 测试 FollowJointTrajectory 的发送/取消和
控制器生命周期。`gz` 以无头模式启动 Gazebo 并验证两条轨迹。真机调试使用独立的
[当前实机状态与后续优化](docs/commissioning_cn.md)。

MoveIt 的规划、严格碰撞检查与模拟执行有独立的无界面冒烟测试，在构建并 source 后运行：

```bash
python3 src/hex_arm_moveit_config/test/test_moveit_mock.py
```

<a id="documentation"></a>

## 文档索引

| 文档 | 内容 |
|---|---|
| [MoveIt 仿真指南](docs/moveit_simulation_cn.md) | 模拟窗口、限速配置、碰撞模型与 MoveIt 真机规划入口 |
| [图形界面与命令行模拟指南](docs/gui_and_cli_simulation_cn.md) | X11/WSLg/NVIDIA 诊断、action 与 Python 驱动示例 |
| [Meow MIT 实机部署](docs/meow_mit_deployment_cn.md) | 新固件协议、参数单位、profile 与当前部署流程 |
| [当前实机状态与后续优化](docs/commissioning_cn.md) | 当前参数、启动停止、已知待回归问题与后续步骤 |
