# Firefly Y6 ROS 2 驱动
[English (英文版)](README.md)

最新固件与上位机 MIT-pp-test 对齐的部署入口见 [Meow MIT 实机部署](docs/meow_mit_deployment_cn.md)。新 profile 使用 `bus.protocol: meow`；下文旧 CiA402 commissioning 记录保留作历史证据。


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
  -> SocketCAN can0（现场）或用户态 gs_usb（旧路径）-> CAN-FD 电机
```

Rust 进程拥有电机总线及全部安全决策权。ROS 桥接层不实现轨迹 action，也无法绕过独占会话或激活状态机。

## 支持的驱动后端（backend）

| 后端 | 用途 | 硬件访问 |
|---|---|---|
| `view` | URDF、关节方向、限位与 RViz 检查 | 无 |
| `mock` | ros2_control/JTC 生命周期与 action 集成测试 | 无 |
| `gz` | 通过 `gz_ros2_control` 使用 Gazebo Harmonic 物理仿真 | 无 |
| `real` | Rust 控制器、Zenoh 桥接、ros2_control | 宿主机 `can0`，或旧 `gs_usb` 的 `/dev/bus/usb` |

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

脚本在本地 Ubuntu 中自动使用 `compose.ubuntu.yaml`，并只读传入当前 X11 授权
cookie。AMD/Intel 通用路径映射 `/dev/dri`；NVIDIA override 改由 Container
Toolkit 注入设备。

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
