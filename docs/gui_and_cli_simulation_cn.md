# Firefly Y6：图形界面启动与命令行模拟驱动指南

[English](gui_and_cli_simulation.md) | **中文**

> 适用范围：本地 Ubuntu 24.04 或 WSL2 + Docker 下的 `hex_arm_ros2`
>（ROS 2 Jazzy）。

## 本地 Ubuntu 24.04

本地 Ubuntu 使用旧的 WSL2 Compose 时，容器会得到不存在的
`/mnt/wslg/runtime-dir`，同时缺少宿主机 Xauthority cookie。典型报错是先出现
`Authorization required`，随后 RViz 报 `could not connect to display :0`。

从桌面会话中的终端进入仓库根目录：

```bash
./scripts/docker-dev.sh build
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

`doctor` 至少应输出 `X11 authorization: OK` 和 OpenGL 版本。它只读挂载当前
会话的 Xauthority，不会关闭 X server 的访问控制，因此不要执行 `xhost +`。
GNOME Wayland 桌面也通过 Xwayland 使用这套方式。

进入容器后运行 MoveIt：

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

若 OpenGL renderer 是 `llvmpipe`，GUI 仍可正常使用，但 Gazebo 为软件渲染。
NVIDIA 主机若需要容器硬件加速，还需在宿主机单独安装 NVIDIA Container Toolkit；
它不是“窗口无法弹出”的必要修复条件。

## WSL2 专项诊断结论

1. **图形窗口打不开的根因（本机已实测定位）**：当前 `ros2-jazzy-arm` 容器是从 Windows 侧（PowerShell / Docker Desktop）创建的，工程以 `\\wsl.localhost\...` 这种 UNC 路径挂载进容器，`compose.yaml` 里声明的 `/tmp/.X11-unix` 挂载没有生效。容器里虽然能看到 X socket 文件，但**无法建立连接**，于是 RViz、joint_state_publisher_gui 直接报 `qt.qpa.xcb: could not connect to display :0` 崩溃退出。launch 本身、机器人模型、ros2_control 都是正常的。
2. **修复方式**：按 README 的要求，在 **Ubuntu WSL shell**（不是 PowerShell）里重建容器，步骤见下文 WSL2 诊断记录。
3. **命令行模拟驱动：完全可以**，而且已在当前机器上实测通过。`mock`（RViz 显示）和 `gz`（Gazebo 物理仿真）两种模式都暴露标准的 `FollowJointTrajectory` action，用 `ros2 action send_goal` 或一段 Python 脚本即可驱动，详见第 4 节。

---

## WSL2：先确认 WSLg（图形系统）本身正常

在 **Ubuntu-24.04 的 WSL 终端**里执行：

```bash
echo "DISPLAY=$DISPLAY WAYLAND=$WAYLAND_DISPLAY"
ls -la /tmp/.X11-unix        # 应能看到 X0 socket
ls /mnt/wslg                 # 应能看到 .X11-unix、PulseServer 等
```

本机实测结果（正常）：

```text
DISPLAY=:0 WAYLAND=wayland-0
/tmp/.X11-unix/X0            # srwxrwxrwx socket
```

还可以用下面这段 Python 直接跟 X 服务器做一次握手，连接成功即说明 WSLg 正常：

```bash
python3 - <<'EOF'
import socket
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect('/tmp/.X11-unix/X0')
s.sendall(b'l\x00\x00\x0b\x00\x00\x00\x00\x00\x00\x00\x00')
print('X OK, reply:', s.recv(8).hex())
EOF
```

正常输出类似 `X OK, reply: 00190b0000000700`；如果报错或卡住，说明 WSLg 本身有问题（先 `wsl --update` / 重启 WSL），与 Docker 无关。

---

## WSL2 图形窗口诊断记录

### 现象

按 README 执行 `ros2 launch hex_arm_bringup view.launch.py` 后，终端里出现：

```text
[rviz2-3] qt.qpa.xcb: could not connect to display :0
[rviz2-3] qt.qpa.plugin: Could not load the Qt platform plugin "xcb" ...
[ERROR] [rviz2-3]: process has died ...
[joint_state_publisher_gui-2] qt.qpa.xcb: could not connect to display :0
[ERROR] [joint_state_publisher_gui-2]: process has died ...
```

`robot_state_publisher` 正常启动，只有需要弹窗的进程死掉——这是典型的「容器连不上 X 服务器」。

### 根因（WSL2 实测数据）

对当前容器 `docker inspect ros2-jazzy-arm` 检查后发现：

| 检查项 | 结果 | 说明 |
|---|---|---|
| 工作区挂载源 | `\\wsl.localhost\Ubuntu-24.04\home\kk_wsl\...` | 从 Windows 侧创建的 UNC 挂载 |
| 容器 Hostname | `docker-desktop` | 引擎是 Docker Desktop，不是 WSL 内原生 dockerd |
| 挂载列表 | 只有工作区 + `/mnt/wslg` | `compose.yaml` 声明的 `/tmp/.X11-unix` **缺失** |
| 容器内 X socket | `stat` 能看到文件，但 `connect()` 返回 `FileNotFoundError` | socket 不可连接 |
| WSL 宿主机 X socket | 握手成功（`X OK, reply: 00190b0000000700`） | WSLg 本身正常 |
| 新建一个正确挂载 `/tmp/.X11-unix` 的容器 | 握手成功 | 证明修好挂载即可解决 |

结论：**容器是被 Windows 侧的 Docker Desktop 创建出来的，X11 socket 的挂载损坏/缺失，导致所有 Qt/X11 图形程序无法连上 `:0`**。README 里那句“请在 Ubuntu 24.04 WSL shell 中运行，而不是在 PowerShell 中运行”正是为了防止这种情况。

### 修复：在 WSL shell 里重建容器（推荐）

> 注意：如果 VS Code 的 Dev Containers 正连着这个容器（当前容器里确实有 vscode-server 和打开的终端），先断开/关掉 VS Code 连接，重建会中断这些会话。

```bash
# 在 Ubuntu-24.04 的 WSL 终端中执行（不是 PowerShell）
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2

docker compose down          # 只删容器，不影响镜像和工程文件
docker compose up -d         # 从 WSL 原生路径重建，三个挂载都会生效
docker exec -it ros2-jazzy-arm bash
```

进容器后确认挂载和 X 是否恢复：

```bash
docker inspect ros2-jazzy-arm | grep -A 10 '"Mounts"'   # 应能看到 /tmp/.X11-unix
docker exec ros2-jazzy-arm ls /tmp/.X11-unix           # 应能看到 X0
```

然后在容器里（交互式 bash 会自动 source，或手动）：

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/hex_arm_ros2/install/setup.bash
ros2 launch hex_arm_bringup view.launch.py             # 滑块窗口 + RViz
```

### 不想重建容器时的临时方案（备用）

不动现有容器，另开一个挂载正确的新容器跑 GUI（ROS 通信走 host 网络，与旧容器互通；两者不要同时 launch 同一种模式）：

```bash
# 在 WSL 终端执行
docker run -d --name ros2-jazzy-gui \
  --network host --ipc host \
  -e DISPLAY=:0 -e WAYLAND_DISPLAY=wayland-0 \
  -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
  -e PULSE_SERVER=unix:/mnt/wslg/PulseServer \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v /home/kk_wsl/ros2_ws/code/hex_arm_ros2:/workspaces/hex_arm_ros2 \
  hex-arm-jazzy:local sleep infinity

docker exec -it ros2-jazzy-gui bash
source /opt/ros/jazzy/setup.bash
source /workspaces/hex_arm_ros2/install/setup.bash
ros2 launch hex_arm_bringup view.launch.py
```

这个方案的 X socket 连通性已实测通过；完整 GUI 流程建议自己跑一次确认。

---

## 4. 用命令行/脚本驱动模拟机械臂

工程提供三种模式，只有 `view` 是纯展示，`mock` 和 `gz` 都有真正的控制回路：

| 模式 | 内容 | 启动命令 | 驱动方式 |
|---|---|---|---|
| `view` | URDF + 关节滑块 + RViz，无控制回路 | `ros2 launch hex_arm_bringup view.launch.py` | 拖动滑块（GUI）；命令行请用 mock/gz |
| `mock` | ros2_control + 模拟硬件 + 轨迹控制器 + RViz | `ros2 launch hex_arm_bringup mock.launch.py use_rviz:=true` | `FollowJointTrajectory` action（命令行/脚本） |
| `gz` | Gazebo Harmonic 物理仿真（+RViz） | `ros2 launch hex_arm_bringup gz.launch.py headless:=false use_rviz:=true` | `FollowJointTrajectory` action（命令行/脚本） |

统一的外部接口：

```text
/firefly_arm_controller/follow_joint_trajectory   (control_msgs/action/FollowJointTrajectory)
```

关节顺序固定为 `joint_1 ... joint_6`，单位是弧度（rad），限位如下（来自工程 URDF）：

| 关节 | 限位 (rad) |
|---|---|
| joint_1 | −2.86 ~ 2.86 |
| joint_2 | −1.57 ~ 2.09 |
| joint_3 | 0 ~ 3.14 |
| joint_4 | −1.57 ~ 1.57 |
| joint_5 | −1.54 ~ 1.54 |
| joint_6 | −2.79 ~ 2.79 |

### 4.1 命令行：`ros2 action send_goal`（已在本机实测）

先启动 mock 模式（开 RViz 就能看到机械臂动）：

```bash
# 终端 1
source /opt/ros/jazzy/setup.bash
source /workspaces/hex_arm_ros2/install/setup.bash
ros2 launch hex_arm_bringup mock.launch.py use_rviz:=true
```

另开一个终端发送轨迹目标：

```bash
# 终端 2（同样先 source 两行）
ros2 action send_goal /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, 1.25, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}"
```

预期输出（本机实测）：

```text
Goal accepted with ID: ...
...
Goal finished with status: SUCCEEDED
```

查看关节实际位置（应等于目标值）：

```bash
ros2 topic echo --once /joint_states
# position: [0.15, 0.25, 1.25, -0.2, 0.15, -0.1]
```

其他常用命令：

```bash
ros2 action list -t                      # 查看可用的 action
ros2 topic list                          # 查看话题
ros2 topic echo /joint_states            # 持续查看关节状态
```

### 4.2 gz 物理仿真模式

```bash
# 终端 1：打开 Gazebo 窗口和 RViz
ros2 launch hex_arm_bringup gz.launch.py headless:=false use_rviz:=true

# 终端 2：同一套 send_goal 命令即可驱动
ros2 action send_goal /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, 1.25, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}"
```

说明：
- `headless:=false` 才会弹出 Gazebo 窗口；默认 `headless:=true` 是后台物理仿真。
- gz 模式使用仿真时间（`use_sim_time`），轨迹执行由 Gazebo 时钟驱动。

### 4.3 用 Python 脚本驱动（可编程）

把下面内容存成 `drive_arm.py`（写法与仓库自带测试 `src/hex_arm_bringup/test/trajectory_test_common.py` 一致）：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f"joint_{i}" for i in range(1, 7)]

def main() -> None:
    rclpy.init()
    node = Node("drive_arm")
    client = ActionClient(
        node, FollowJointTrajectory,
        "/firefly_arm_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=10):
        raise SystemExit("action server 不可用：先启动 mock/gz launch")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = [0.15, 0.25, 1.25, -0.2, 0.15, -0.1]
    point.time_from_start.sec = 2
    goal.trajectory.points = [point]

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        raise SystemExit("goal 被拒绝")
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    print("result status:", result.result().status)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

```bash
python3 drive_arm.py
```

### 4.4 无界面自动化验证

仓库自带测试可以直接验证 mock/gz 的完整轨迹链路（发送两条轨迹 + 取消一条），不需要任何窗口：

```bash
./scripts/test.sh mock
./scripts/test.sh gz
```

本机 `./scripts/test.sh mock` 已实测通过。

---

## 5. 常见问题速查

| 报错/现象 | 原因 | 处理 |
|---|---|---|
| `qt.qpa.xcb: could not connect to display :0` | 本地 Ubuntu 缺 Xauthority，或 WSL2 的 X socket 挂载失效 | 在宿主机运行 `./scripts/docker-dev.sh doctor`，再按对应章节重建容器 |
| `command not found: ros2 / rviz2` | 没 source ROS 环境 | `source /opt/ros/jazzy/setup.bash`（或进入交互式 bash） |
| `Package 'hex_arm_bringup' not found` | 没 source 工作区 install | `source /workspaces/hex_arm_ros2/install/setup.bash`，没有就先 `./scripts/build.sh` |
| 从 PowerShell 跑 `docker compose` | 会以 UNC 路径挂载，X socket 失效 | 一律在 Ubuntu WSL shell 中执行 |
| Gazebo 很卡/窗口空白 | WSLg 软件渲染 | 正常现象，属软件渲染；可接受即可 |
| `ros2 action send_goal` 提示 server 不可用 | launch 还没起完 / 控制器未激活 | 等几秒再发，或用 `ros2 action list -t` 确认 |

---

## 6. 附：本次诊断执行过的关键命令（留档）

```bash
# WSL 宿主机
python3 x11probe.py                      # /tmp/.X11-unix/X0 握手 → OK

# 容器内
docker exec ros2-jazzy-arm env            # DISPLAY=:0 等环境正确
docker inspect ros2-jazzy-arm             # 挂载缺 /tmp/.X11-unix；Hostname=docker-desktop
docker exec ros2-jazzy-arm python3 x11probe.py   # 容器内握手 → FileNotFoundError

# 对照实验
docker run --rm -v /tmp/.X11-unix:/tmp/.X11-unix \
  --entrypoint python3 hex-arm-jazzy:local x11probe.py   # 新容器握手 → OK

# 命令行驱动验证
./scripts/test.sh mock                   # 通过
ros2 action send_goal ...                # SUCCEEDED，/joint_states 到达目标位姿
```
