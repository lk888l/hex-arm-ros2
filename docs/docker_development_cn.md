# Docker 开发环境

[English](docker_development.md) | **中文** · [返回 README](../README_cn.md)

日常启动见 [README 的环境与构建](../README_cn.md#environment)。本地 Ubuntu 自动选择
`compose.ubuntu.yaml`，并只读传入当前 X11 授权 cookie；WSL2 自动选择
`compose.yaml` 与 WSLg socket。`HEX_ARM_GPU=auto` 会在检测到可用 NVIDIA GPU 时
叠加 `compose.nvidia.yaml`，需要宿主机已安装 NVIDIA Container Toolkit。


## 不使用辅助脚本：原生 Docker Compose 命令

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



## NVIDIA 独立显卡加速

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



## WSL2（仅 Windows 10/11）

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

