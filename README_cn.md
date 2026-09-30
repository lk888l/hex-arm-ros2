# Firefly Y6 ROS 2 驱动

[English](README.md) | **中文**

框架重构与独立 Docker 部署见 [运行架构与迁移说明](docs/architecture_refactor_cn.md)。
默认构建包含 Meow 与 CiA402 驱动；历史 CiA402/USB 调试工具不默认安装。
当前 Meow 真机执行每次自动完成 J2 → J4 → J3。

旧版 CiA402 固件、无界面 Docker 和分阶段扩大规划窗口见
[CiA402 真机部署与验收](docs/cia402_deployment_cn.md)。2026-09-29 的 can2 测试在软件零偏
校准后使用重力补偿，通过 J2 +0.24 rad、J6 ±0.24 rad、J4 −0.12 rad 独立往返；
详见[扩大行程记录](docs/commissioning_evidence/2026-09-29-can2-expanded.md)。完整六轴 MoveIt 执行尚未验收。

当前替换臂已升级为 Meow 固件，保留本臂软件零偏；操作见[替换臂 Meow 部署入口](docs/meow_replacement_deployment_cn.md)。新版适配、参数与实测结果见
[Meow 升级验收记录](docs/commissioning_evidence/2026-09-29-can2-meow-upgrade.md)。
上述 CiA402 结果属于升级前记录，不能直接作为新版固件验收依据。

更换机械臂或重新校准后，使用[折叠姿态软件零点校准工具](docs/zero_calibration_cn.md)
读取六轴编码器、计算 `zero_offset_rad`、生成本机配置并独立复查。

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
J2→J4→J3 顺序启动和 MoveIt 小范围执行已验证。操作者已确认 GUI 大范围 MIT 和全范围
重力补偿正常；正式 MoveIt 范围、动态配置及本次验收见[替换臂部署入口](docs/meow_replacement_deployment_cn.md)。
此前窄窗口调试参数、记录与历史待回归问题保留在
[当前实机状态与后续优化](docs/commissioning_cn.md)。

### 启动与停止

本机部署配置为 `config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml`，包含已采用的电机身份、
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
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml \
  enable_execution:=true position_limits:=hardware dynamics_limits:=custom \
  planning_limits_file:=/workspaces/hex_arm_ros2/src/hex_arm_moveit_config/config/joint_limits_deployment.yaml \
  align_folded:=true allow_enable_transient:=true
```

入口核对反馈和位置后使能，默认依次移动 **J2→−1.350（8 s）、J4→−0.300（10 s）、
J3→1.430（6 s）**，其余关节最终为 0。权威数值位于
[`src/hex_arm_controller/config/startup.yaml`](src/hex_arm_controller/config/startup.yaml)，
日常操作说明位于[当前实机状态与后续优化](docs/commissioning_cn.md#日常启动与停止)。
该顺序退出路径由操作者确认实物无碰撞。
等待 `startup_ready reached and verified; controller continues holding` 后，
在 RViz 选择 `arm`，以当前状态为起点进行 `Plan` / `Plan & Execute`。
后续 MoveIt 执行使用严格碰撞检查，并受当前本机运动窗口约束。

当前 Kp 为 `[100,100,150,110,80,80]`，Kd 均为 15，重力比例均为 1.0。
部署 profile 的速度、加速度均为约 1.257 SI 单位，等于 GUI 的 0.2 Rev/s、0.2 Rev/s² 乘 `2π`。
正式规划 YAML 的加速度为 0.6 rad/s²，给 Rust 连续插值保留余量。
`dynamics_limits:=hardware` 直接采用硬件上限；`custom` 再与所选规划 YAML 取交集。
两者均解除 commissioning 的隐藏 0.1 限速；RViz 的规划比例另行设置。
常规真机轨迹执行每次必须完成自动启动顺序，不能用 `startup_ready:=false` 绕过；
省略 `enable_execution:=true` 为保持失能的观察和规划。

在拥有 real-launch 的终端第一次按 Ctrl-C，会先经 MoveIt **回安全启动位**，再按
J3→J4→J2 受控回折并确认失能。再次 Ctrl-C 或故障走立即停机路径。
等待受控回折结果及 `VERIFIED structured disabled_confirmed` 后再断电；本部署 profile 使用回折停机。
切换上位机、修改配置或重新编译前，先停止当前控制端。

### 旧版 CiA402 电机

默认构建的 `hex_arm_controller` 同时包含 Meow 和 CiA402 后端，运行时由硬件 profile
中的 `bus.protocol` 选择。旧电机从 `config/hardware/firefly_y6.example.yaml` 新建
`*.local.yaml`，明确使用 `protocol: cia402`、`loop_hz: 1000`，并按旧电机实物重新核对
身份、方向、零位、力矩比例和运动窗口。配置仍须为 schema v3 / 关节坐标 v2，
且通过标定后才能设置 `calibrated: true`。仓库内旧版
`firefly_y6.discovered.local.yaml` 是 schema v2、未标定记录，不能直接使能。

使用 `./scripts/build.sh` 构建后，CiA402 真机 MoveIt 入口沿用 `real-launch moveit`，
并传入 CiA402 本机 profile。`enable_execution:=true` 时，启动客户端先检查六轴静止、
当前姿态在该 profile 的限位内；使能后验证当前位置保持误差和速度，再允许轨迹执行。
CiA402 不执行 Meow 的折叠退出序列。Docker `real-launch` 仅接受已绑定的 SocketCAN
接口；旧版单轴诊断工具仍需用 `HEX_ARM_BUILD_COMMISSIONING=ON` 单独构建。

### CAN 接口与继续开发

CAN 名称可配置；当前使用 can2，接口速率为 1 Mbps 仲裁 / 4 Mbps 数据。
选择接口时同时校验 USB 适配器序列号和物理通道。更换接口后，使用
`scripts/bind-can-profile.py` 生成保留身份与标定的新 profile，再通过
`HEX_ARM_CAN_IFACE` 选择对应接口；完整命令见 [Meow MIT 部署说明](docs/meow_mit_deployment_cn.md#更换-can-接口)。

清理后保留 `install/` 及其在 `build/` 中的必要链接目标。修改源码后，先停止控制，
在容器工作区执行 `./scripts/build.sh`，再 `source install/setup.bash`；编译缓存会重新生成。
后续按重复性回归、模型与限位核对、逐步扩大运动范围、连续运行与故障恢复、部署固化的顺序推进，
详见 [后续优化步骤](docs/commissioning_cn.md#后续优化顺序)。

## 手扶拖动：重力补偿与阻尼

专用 `gravity_comp.launch.py` 从当前实测姿态进入手扶拖动：`Kp=0`、目标速度为零，
输出为经过 profile 比例及限幅处理的重力矩，加上 `−Kd × 实测速度` 阻尼。
它不执行折叠展开、位置保持或轨迹跟踪，也不启动 MoveIt / ros2_control。
松手后阻尼会使运动减速，但不会锁定位置；重力模型、载荷和补偿比例有误差时仍会漂移。

先停止其他机械臂控制程序。在容器工作区构建并加载新入口：

```bash
./scripts/build.sh --packages-select hex_arm_controller hex_arm_bridge hex_arm_bringup
source install/setup.bash
ros2 launch hex_arm_bringup gravity_comp.launch.py \
  hardware_profile:=/workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.hand_guiding_full_range.local.yaml \
  activate_hardware:=true \
  damping:='[2.0, 2.0, 2.0, 2.0, 2.0, 2.0]'
```

当前臂使用 `replacement.local.yaml`，其适配器及六轴身份与当前 can2 实读值匹配；
`firefly_y6.meow.can2.local.yaml` 是旧臂配置。只重绑 CAN 适配器不会迁移电机身份或零位标定。
上面的 `hand_guiding_full_range.local.yaml` 基于本臂配置，将六轴位置限位扩大到 URDF 范围，
并使用独立的六轴手扶速度保护门槛 2.0 rad/s：
J1 `[-2.86, 2.86]`、J2 `[-1.57, 2.09]`、J3/J4 `[-1.57, 1.57]`、
J5 `[-1.54, 1.54]`、J6 `[-2.79, 2.79]` rad。原 `replacement.local.yaml` 保留窄窗口。
新配置属于用户授权的全范围拖动调试候选；继承的标定标志不代表新增范围已经实机验收。
单轴模型限位不能保证关节组合无碰撞，也不等于已核实的机械硬限位。
六轴重力补偿比例均为 1.0，力矩上限沿用原试调值，大幅改变姿态后可能支撑不足，
需要持续扶稳，不能把松手悬停作为已实现的保证。配置文件直接加载，无需重新编译。

`damping` 顺序为 J1～J6，单位为 N·m·s/rad，必须为六个有限正数。
默认各轴为 2.0；增大时松手减速更强，拖动也更费力，减小时阻力和松手减速能力一起降低。
改变参数需要先停止，再重新启动。持续漂移应核对重力模型、安装方向、末端载荷及每轴
`gravity_compensation_scale`；原先配合位置 PD 使用的补偿比例不一定能实现零刚度悬停。
不要通过把现有轨迹控制的 `default_kp` 改为零来启用这个功能。

进入模式需要已验证、已标定的 schema v3 profile、六轴新鲜反馈、关节在限位内，
且各轴实测速度不超过 0.02 rad/s。使能前及重力渐入期间应手扶支撑，等待日志
`hand_guiding_ready` 后再拖动。渐入速率沿用 `gravity_startup_slew_rate_nm_s`，未配置时为
5 N·m/s；渐入后重力补偿直接跟随实测姿态。限位、速度、力矩和温度保护保持生效，
拖动专用 profile 设置 `controller.hand_guiding_velocity_limits_rad_s` 为六轴各 2.0 rad/s
（约 114.6°/s），仅在 `GRAVITY_COMP` 模式中替代普通实测速度门槛；该数组是绝对门槛，
不额外叠加 J2/J4 的速度余量。运动命令和其他模式仍使用 `joints[].limits.velocity_rad_s`。
省略此参数时，手扶模式沿用每轴命令速度加实测余量；数组顺序为 J1～J6，必须包含六个
有限正数且不超过 6 rad/s。阻尼不能保证速度受限，因此仍保留超速保护和非有限反馈拒绝。
超速会故障失能；这不是位置锁定或行程不足。
`controller.hand_guiding_position_margin_rad` 在本拖动配置中为 0.012 rad（约 0.69°），
只在进入/运行 `GRAVITY_COMP` 时替代普通位置反馈余量，其他模式及外部位置命令保持严格限位。
未配置时沿用各轴 `measured_position_margin_rad`；该参数接受有限的 0～0.012 rad，
并要求反馈范围小于一整圈，以满足现有使能位置一致性检查及单圈坐标约束。
反馈处于余量内时，仅把零刚度 MIT 位置目标字段限制到合法命令范围；`Kp` 仍为零。
重力计算始终使用实际编码器角度，不对角度截断；超过反馈余量或非有限读数仍触发保护。
J2 在折叠位置接近模型下限 −1.57 rad，只能向范围内拖动。
本模式没有 MoveIt 碰撞检查，需要在有间隙的工作范围内操作。

停止前先扶稳机械臂，再从另一个已 source 的终端执行：

```bash
ros2 service call /hex_arm_gravity_comp/stop std_srvs/srv/Trigger '{}'
```

服务成功表示所有轴已确认失能且会话已释放，整组 launch 随后退出。
Ctrl-C 同样触发停止，但不会自动回位或折叠；失能后机械臂需要外部支撑。
管理节点每 50 ms 续期，驱动在 500 ms 未收到有效续期后锁存故障并失能，
失败时继续重试失能。管理进程崩溃、冻结以及通信中断都不会留下无限期的补偿输出。
故障后不自动恢复；支撑机械臂并停止该 launch，查明原因后重新启动。
驱动退出确认写在当前 ROS 日志目录的 `hand-guiding-shutdown.json` 中，
只有 `disabled_confirmed` 表示确认失能。

省略 `activate_hardware:=true` 时只观察状态；`use_rviz:=true` 可打开姿态显示。
不接机械臂时可先验证软件链路（mock 不模拟实际重力或拖动手感）：

```bash
ros2 launch hex_arm_bringup gravity_comp.launch.py \
  hardware_profile:=/workspaces/hex_arm_ros2/src/hex_arm_controller/test/firefly_y6.mock.yaml \
  mock:=true activate_hardware:=true
```

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
