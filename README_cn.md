# Firefly Y6 ROS 2 驱动

[English](README.md) | **中文**

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

默认关节姿态为 `[0, -1.350, 3.000, -0.300, 0, 0]` rad，命名状态为 `startup_ready`。这只设置模拟关节的初始显示；真机按实测反馈启动。

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

<a id="simulation-cli"></a>

## 用命令行驱动模拟机械臂（`ros2 action send_goal` 详解）

先启动上文的普通 `mock` 或 `gz` 模式。另开一个宿主机终端，通过
`./scripts/docker-dev.sh shell` 进入同一个容器，然后执行：

```bash
cd /workspaces/hex_arm_ros2
source install/setup.bash
ros2 action send_goal /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, 1.25, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}"
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

## 真机入口与调试参考

### CAN 接口可以选择

驱动没有写死 `can0`。接口名由 profile 的 `bus.interface` 指定；物理通道及
USB 序列号由 sysfs 识别，不从 `canN` 的数字猜测。切换端口时，在停止控制器后
为所选接口生成一份新的配置，电机身份、零点和运动参数会保留：

```bash
# 宿主机仓库目录；先确保所选接口已按 1M/4M 配置并启用。
export HEX_ARM_CAN_IFACE=can0
python3 scripts/bind-can-profile.py \
  --interface "$HEX_ARM_CAN_IFACE" \
  --profile config/hardware/firefly_y6.meow.local.yaml \
  --output config/hardware/firefly_y6.selected.local.yaml

./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh real-launch bringup \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.selected.local.yaml \
  activate_hardware:=false use_rviz:=false
```

把 `can0` 换成实际接口即可。输入 profile 必须属于这台机械臂；上例
`firefly_y6.meow.local.yaml` 是本机采集的本地文件，不随 Git 分发。
工具不覆盖已有输出，重新选择时请使用新文件名。运行过程中不热切换总线。
`HEX_ARM_CAN_SERIAL`、`HEX_ARM_CAN_CHANNEL` 仍可显式指定；省略时脚本读取所选
接口的实际值。控制器仍会核对完整六轴身份、CAN 时序及适配器。

### 折叠参考与顺序启动试验

断电折叠参考为 `[0, -1.570, 3.140, 0, 0, 0]` rad；理想启动姿态为
`[0, -1.350, 3.000, -0.300, 0, 0]` rad。
`moveit_mock.launch.py` 默认直接显示理想姿态，RViz 的命名状态为
`startup_ready`。真实启动顺序是 **J2 → −1.350、J4 → −0.300、J3 → 3.000**，
各阶段分别用 8、10、6 秒的平滑轨迹，并等待反馈到位后再进入下一阶段。

已确认参考姿态、无负载及实物路径无碰撞后，可使用专门的有界 Meow 试验入口：

```bash
# 会使能真实电机并移动；在已确认的折叠参考姿态运行。
HEX_ARM_CAN_IFACE=can0 ./scripts/docker-dev.sh real-launch startup \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.local.yaml \
  allow_startup_motion:=true
```

它先做完整身份/反馈/力矩预算检查，原位渐入重力补偿，然后执行固定顺序；
结束后保持 3 秒并确认失能。任一步超时、反馈失效、速度或跟踪超限都会停止。
此入口不会设置 `calibrated: true`，也不会自动启动 MoveIt 执行。到位容差沿用
GUI 的 0.003 Rev（约 0.01885 rad），不等于 ROS 轨迹控制器的 0.005 rad 验收。

MIT-pp-test 的 Kp/Kd 基线为 **80 Nm/rad、15 Nm·s/rad**。GUI 基线重力比例为
`[0,0.3,0.7,0.7,0,0]`；本机无夹爪实测后调整为 `[0,1.0,1.05,0.7,0,0]`，
J3 的 PD 预算为 450‰，其余轴为 500‰，总输出预算仍为 650‰。
固定顺序已在真机完成，最终 J2/J3/J4 误差约 0.0020/0.0043/0.0025 rad。
这些是本机参数，具体范围和证据见 [现场记录](docs/commissioning_evidence/2026-09-07-meow-startup.md)。
出厂校准由驱动读取。中间折叠路径的网格接触已由本次
操作者确认不构成实物碰撞；该固定试验不使用 MoveIt 规划，严格碰撞矩阵仍保留。
J4 commissioning 下限为 −0.35 rad，以包含 −0.300 rad 目标。

**J4 Kp=110 下，can2 已通过顺序启动、MoveIt 小步执行和 60 秒保持，该轮未复现此前的 J4 跟踪超限。**
后续 15 mrad 的 J2 回程触发原有到位保护，双向运动仍待调优。当前 Kp 为
`[80,80,120,110,80,80]`、全部 Kd=15。以已核对的折叠姿态执行：

```bash
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.local.yaml \
  enable_execution:=true
```

`startup_ready` 默认 true：控制器就绪后处理 J6 偏差，再严格按 J2→J4→J3
到 `[0,-1.350,3.000,-0.300,0,0]` 并持续保持。J2/J3/J4 在各自动作前保持实测折叠位置，
小摆放偏差在使能前检查，不重新设置零偏。等待日志
`startup_ready reached and verified; controller continues holding` 后再操作 RViz 执行。需要原位保持时加
`startup_ready:=false`；仅观察时省略 `enable_execution:=true`。
J6 保留零偏，自动回零仍在 profile 范围内限速进行。已有成功试验仅覆盖受限启动与目标邻域，
完整行程和带负载运动尚未验收。运行、停止及参数范围见
[当前 can2 真机部署记录](docs/commissioning_evidence/2026-09-08-can2-kp110-deployment.md)。


真机已有独立的观察/规划入口；执行需要完成 profile 标定和实机验收。
最新 Meow 固件与上位机 MIT-pp-test 对齐的部署流程见
[Meow MIT 实机部署](docs/meow_mit_deployment_cn.md)，使用 `bus.protocol: meow`。
模拟规划或执行成功，不代表真机已完成验证。

真机调试需具备物理急停，未知总线先做只读发现。在 Docker 中运行真机时使用
`./scripts/docker-dev.sh real-launch` 受监督入口，停止后等待控制器确认失能并退出。
完整步骤见部署文档与 [commissioning 清单](docs/commissioning_cn.md)。

<details>
<summary>历史 CiA402 参考：总线发现、受监督启动与安全边界</summary>

以下保留旧 CiA402 调试背景，其中 `can2`、轴参数和单圈窗口均为历史记录。
新 Meow 固件的配置、单位与标定要求以部署文档为准，不直接沿用旧参数。

## 真机：先做只读发现

现场 CAN-FD 链路的完整契约是仲裁段 `1 Mbit/s, SP=0.8, SJW=5`、数据段
`4 Mbit/s, SP=0.8, SJW=3`，并关闭自动 bus-off restart（`restart-ms 0`）。启动
任何 ROS 或 GUI 进程前，先在本地 Ubuntu 宿主机配置：

```bash
sudo ip link set dev can0 down
sudo ip link set dev can0 type can \
  bitrate 1000000 sample-point 0.8 sjw 5 \
  dbitrate 4000000 dsample-point 0.8 dsjw 3 \
  fd on restart-ms 0
sudo ip link set dev can0 up
ip -details -statistics link show dev can0
```

宿主机也可用 `can-config set can0 4M` 设置 1M/4M 时序，但它不会修改电机固件，
当前版本也不写 `restart-ms`。ROS 驱动、MoveIt 或电机 GUI 正在使用 `can0` 时禁止
运行该命令，执行后仍须核对完整链路。详见
[有监督的硬件调试清单](docs/commissioning_cn.md)，然后启动普通容器：

```bash
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

Ubuntu Compose 使用 host network，因此容器直接看到宿主机 `can0`；SocketCAN
路径不需要 `HEX_ARM_REAL=1`。在已经构建的容器内，先用以下命令识别六个机械臂
节点和已知辅助节点；该过程不加载硬件 profile，也不初始化驱动器：

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/hex_arm_ros2/install/setup.bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --discover-only --transport socket-can --interface can0 \
  --expected-node 1 --expected-node 2 --expected-node 3 \
  --expected-node 4 --expected-node 5 --expected-node 6 \
  --auxiliary-node 15 --timeout 2 --sdo-timeout-sec 0.25
```

该模式监听心跳，并且只允许 CANopen 身份 SDO upload；不能发送 NMT、PDO、SDO
download、控制字或电机指令。接通电源前请先阅读
[docs/commissioning_cn.md](docs/commissioning_cn.md)。旧的直连 `gs_usb` 路径仍需
`HEX_ARM_REAL=1`；它固定为 1M/5M，不得用于现场 1M/4M 链路。

发现成功只能证明节点存在且身份可读。本机已确认直接映射 node 1→joint_1 到
node 6→joint_6；node 15（`0x0f`）是夹爪，仍完全排除在机械臂的初始化、使能、
失能和指令路径之外。但它作为物理载荷不能被忽略：配置 `tip_payload` 后，启动必须
精确匹配 node 15 的 `0x1018` 指纹，并把固定质量/质心合并到 `link_6` 的重力模型。
方向、零点、限位、力矩缩放和真实 TCP 仍需 commissioning。GUI 的 direct userspace `gs_usb`
路径与 ROS SocketCAN 路径不得同时独占同一 USB-CANFD 适配器。

将 `config/hardware/firefly_y6.example.yaml` 复制为被 git 忽略的 `*.local.yaml`，
填写节点 identity、严格的 `expected_link` 时序/USB 指纹和轴参数候选，并保持
`bus.direct_joint_mapping: true`。结构与指纹
复核后，`validated: true`、`calibrated: false` 的 profile 可以用于禁用状态观察；
激活门会明确拒绝尚未标定的 profile。

任何真机 launch 之前，先离线验证 profile、URDF 动力学模型和单圈命令窗口；该命令
不会打开 CAN：

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml \
  --validate-profile-only
```

真机 launch 建议从**宿主机**使用受监督 Docker 入口。接口、通道必须与 YAML
profile 完全一致（下面使用当前现场的 `can2`、channel 2）：

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch bringup \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

bringup 仍默认 `activate_hardware:=false`，该辅助命令不会自动使能。它为本次启动
建立独立进程组；收到 `Ctrl-C`、SIGTERM 或终端挂断后，只向该组转发信号并继续
等待。只有 `hex_arm_controller` 通过“六轴确认失能→心跳 consumer 解除”路径干净
退出，才会输出 `VERIFIED clean controller exit`。另一个使用 `can1` 的 launch 不会
被发信号或结束。如果没有看到该确认行，应把退出视为未确认，先检查保留的
`/tmp/hex-arm-real-launch.../launch.log`，不要立即启动同接口的新 owner。

新的 MoveIt 真机入口同样默认只观察和规划，并固定使用低速 commissioning limits：

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/my_arm.local.yaml
```

不要用裸
`docker exec ... bash -lc 'ros2 launch ...'` 包裹真机 launch：中断外层 exec 时可能
留下 launch/controller 子进程。正常停止也不要使用 `pkill`、`killall`、重启容器或
`docker compose down`，因为这些方式无法给出控制器的失能/解除心跳确认。应保持
受监督命令连接，直到它输出最终验证结果。

在 profile 完成标定且修正后的 joint_2 零偏/范围通过真机验收之前，不得设置
`enable_execution:=true`。

注意：`0x603F=0x8130` 是可保留的 last-error；若当前 `0x6041=0x0231`，驱动器
已是 non-Fault/non-OE，不要为擦除历史码执行 fault reset。正常退出会在主站心跳仍
发送时，先确认六轴失能，再逐轴回读确认 `0x1016:01=0`，避免退出本身重新制造
heartbeat-lost。只有 `0x6041` 当前 Fault bit 为 1 且 last-error 恰为 `0x8130`
时，才可使用受门控的显式恢复 CLI；完整命令和判据见
[真机调试文档](docs/commissioning_cn.md)。

## 安全边界

`activate_hardware` 默认是 `false`，所以上述命令只启动 Rust 控制器、桥接、
robot_state_publisher 和可选 RViz，不启动 ros2_control 或轨迹控制器。观察启动仍会在
disabled 状态初始化驱动器，因此面对未知总线时必须先执行 `--discover-only`。

真实启动始终从 disabled（禁用）状态开始。激活需要六个全新识别（fresh identified）的电机、一份完整校准的配置文件、一个独占的 robot_api 会话，以及一次显式的 ros2_control 生命周期转换。电机故障、反馈过期、非有限（non-finite）指令、CAN 传输失败或指令看门狗超时，都会锁存整臂故障并触发停止/禁用。

WSL2、Docker、Zenoh、SocketCAN 和用户态 USB 均不属于硬实时或经过安全认证的组件。调试（commissioning）时必须配备可用的物理急停开关。当前姿态证据约为
`q=[0,-1.570,3.140,0,0,0]`；旧方向候选 `[-1,-1,+1,+1,+1,+1]` 和由单次快照
拟合的零偏只能作为 commissioning 证据，不是动作验证。修正此前遗漏的 joint_2
负号后，其完整 URDF 范围约映射到 `[-0.3310,+0.2515]` 电机圈数，因此旧的 seam
阻断是错误零偏造成的。本地硬件、URDF 和 MoveIt 下限仍统一为 `-1.570`；
断电重启后若反馈低于该值，必须 fail-closed 并重新确认参考，不会为了隐藏摆放误差而放宽命令范围。
joint_2 仍从该下限起步，必须先向范围内重新调试。

旧 `hex-ros2-arm` 桥和生成的 MoveIt 包已经过审计而不是直接复制：方向候选、J1--J3
的 0.85 力矩换算、规划链和 FJT 接口已经进入当前项目；自动进入 ACTIVE、无误差校验
即报告 action 成功、6/10 rad 运动限位、未经复核的固定重力和关闭全部自碰撞的 SRDF 都被明确
拒绝。详见[旧仓库复用审计](docs/commissioning_cn.md#旧-hex-ros2-arm-复用审计)。

桥发送空 `kp`、`kd` 和 `tau_ff`，由 Rust 选择经过复核的逐轴低增益，并依据机械臂
URDF 与已配置的固定 `tip_payload` 自动计算重力前馈。非空的外部 `tau_ff` 会绕过
这条载荷模型和 `gravity_compensation_scale` 自动路径。API 的 `kp`/`kd` 是关节侧
SI 增益，单位分别为 `Nm/rad` 和 `Nm*s/rad`，`tau_ff` 是关节侧 `Nm`。因此逐轴
`torque_scale` 在下发到电机时作用于所有产矩 MIT 项--前馈以及 P、D 增益系数，
电机实测力矩返回 ROS 时则使用其倒数。ROS/MoveIt 桥刻意发送空数组，因此走的是
载荷感知路径。本地现场 profile
才是逐轴增益、重力比例、限制和力矩权限的权威记录。当前断点 J1..J6 的 Kp/Kd 为
`60/2.5`、`80/2.5`、`2/0.3`、`30/1`、`30/1`、`20/1`，J2/J3/J4 分阶段
重力比例为 `0.25/0/0`；这些仍只是 commissioning 候选，不是安全认证值。
本仓库不声称零位、运动方向、力矩标定或执行精度已经通过真机动作验证。

真机硬件 profile 使用 schema v2，并强制显式填写 URDF `base_link` 坐标系下（m/s²）的
`gravity_vector_base_m_s2`；由于安装方向属于安全关键参数，v1 或缺少该字段都会被
拒绝。当前本地 commissioning 候选为 `[0.0, 0.0, -9.81]`，并继续保持未标定。
commissioning 与 runtime 都先用它计算重力，再应用逐轴
`gravity_compensation_scale`；逐轴标量不能用来改变重力方向。`SetGravity` 只覆盖
当前会话，release、shutdown 或新会话都会恢复 profile 值。迁移和验证要求见
[真机调试文档](docs/commissioning_cn.md)。

默认严格 SRDF `firefly_y6.srdf` 只排除六对直接相邻连杆。mock MoveIt 与
`enable_execution:=true` 的真机 MoveIt 始终使用这份严格矩阵，因此 15 对非相邻
连杆在所有可执行路径上都会继续做碰撞检查。单独的
`firefly_y6.plan_only.srdf` 只会在 `enable_execution:=false` 的真机 MoveIt 中加载，
它仅为修正后的实测折叠姿态中来源 collision mesh 报告的两对接触增加
`PlanOnlySurveyedFold` 例外：`link_1`--`link_5` 和 `link_2`--`link_4`。其余十三对
非相邻连杆仍由 FCL 检查；离线 guard 在此前错误的
`q=[0,+1.570,3.140,0,0,0]` 姿态仍会暴露五对未放宽接触。MoveIt 全范围 100,000
姿态采样没有发现任何永久碰撞的非相邻对，因此没有
照搬额外 `Never` 对或旧 TEMP 全禁用矩阵。这个 plan-only 覆盖只是等待碰撞网格
修正前的已知模型补丁，不是物理安全结论。基于该放宽矩阵得到的规划结果只可用于
可视化/调试预览，不能直接复用为执行轨迹；真机执行前必须在严格 SRDF 下重新规划并
重新通过碰撞验证，同时修正碰撞几何和实体起始姿态。本地 GR80 条目中的 0.41 kg
质量/质心来自 trial URDF，只按 identity mount 合入 `link_6` 作为重力模型占位。
`inertial_calibrated: false` 会让 `calibrated: true` profile 直接无效，从而禁止真机
MoveIt 执行。`link_6` 暂时作为规划末端，仍需审查已标定的工具惯量、TCP/工具
坐标系和最终碰撞几何；真机 MoveIt launch 因此只把 `link_6` 作为临时法兰末端。
其 launch 契约和 mock 回归已做离线测试，但本文不声称已经执行过真机电机动作或
MoveIt 真机轨迹。

</details>

## 控制接口与架构

对外公开的运动接口是由 `joint_trajectory_controller` 暴露的标准 `control_msgs/action/FollowJointTrajectory` action：

```text
/firefly_arm_controller/follow_joint_trajectory
```

真机执行链路将 ROS 控制回路与 USB/CAN-FD 回路分离：

```text
MoveIt / FollowJointTrajectory
  -> ros2_control + firefly_arm_controller
  -> hex_arm_hardware/SystemInterface
  -> hex_arm_bridge（ROS lifecycle <-> Zenoh robot_api）
  -> hex_arm_controller（Rust 安全状态机，1 kHz 软实时回路）
  -> SocketCAN can0（现场）或用户态 gs_usb（旧路径）-> CAN-FD 电机
```

Rust 进程拥有电机总线及全部安全决策权。ROS 桥接层不实现轨迹 action，也无法绕过独占会话或激活状态机。

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
[有人监督的检查清单](docs/commissioning_cn.md)。

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
| [Commissioning 清单](docs/commissioning_cn.md) | 硬件发现、标定、受监督操作与历史验收记录 |
