# Docker development environment

**English** | [中文](docker_development_cn.md) · [Back to README](../README.md)

For daily use, follow [the README environment and build steps](../README.md#environment). On native Ubuntu,
`docker-dev.sh` selects `compose.ubuntu.yaml` and mounts the current X11
credential read-only. On WSL2 it selects `compose.yaml` and WSLg sockets.
`HEX_ARM_GPU=auto` adds `compose.nvidia.yaml` when a working NVIDIA GPU is detected;
this requires NVIDIA Container Toolkit on the host.


## Direct Docker Compose commands (without the helper)

`docker compose` is Docker's Compose CLI. The following commands use
the native Ubuntu AMD/Intel DRI path; NVIDIA hosts also need the override below.
Run them from an Ubuntu graphical desktop terminal and set `HEX_ARM_XAUTHORITY`
in that terminal first:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2

# Prefer XAUTHORITY from the desktop session; otherwise use ~/.Xauthority
if [[ -n "${XAUTHORITY:-}" ]]; then
  export HEX_ARM_XAUTHORITY="$XAUTHORITY"
else
  export HEX_ARM_XAUTHORITY="$(getent passwd "$(id -u)" | cut -d: -f6)/.Xauthority"
fi

# This must print GUI prerequisites: OK
test -n "${DISPLAY:-}" \
  && test -r "$HEX_ARM_XAUTHORITY" \
  && test -e /dev/dri \
  && echo "GUI prerequisites: OK"

# Build on first use or after changing the Dockerfile
docker compose -f compose.ubuntu.yaml build

# Start the container in the background
docker compose -f compose.ubuntu.yaml up -d

# Optional: verify X11 access and show the OpenGL renderer
docker compose -f compose.ubuntu.yaml exec -T ros2-jazzy-arm \
  bash -lc 'xdpyinfo >/dev/null && glxinfo -B'

# Enter the ROS 2 container
docker compose -f compose.ubuntu.yaml exec ros2-jazzy-arm bash
```

After leaving the container, inspect logs or stop it from the same host terminal,
where `HEX_ARM_XAUTHORITY` is still set:

```bash
docker compose -f compose.ubuntu.yaml logs -f
docker compose -f compose.ubuntu.yaml down
```

Set `HEX_ARM_XAUTHORITY` again in every new host terminal before running these
Compose commands. Do not replace native Ubuntu's `compose.ubuntu.yaml` with the
WSL2-only `compose.yaml`, and do not run `xhost +`.



## NVIDIA discrete GPU acceleration

On native Ubuntu, the helper defaults to `HEX_ARM_GPU=auto`: it automatically
adds `compose.nvidia.yaml` when a working NVIDIA GPU is detected, and otherwise
keeps the generic `/dev/dri` path. The NVIDIA override clears that base device
mapping, so an NVIDIA-only host does not need `/dev/dri`, including a real
launch with `use_rviz:=false`. `HEX_ARM_GPU=none` still requires `/dev/dri`.
Before first use, install NVIDIA Container
Toolkit on the host. Do not install the host graphics driver in the Dockerfile:

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

Restarting Docker stops all containers that are running at that moment, but
does not stop CUDA processes running directly on the host. Recreate and check
this project's container afterward:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
```

The mode can also be selected explicitly:

```bash
# Require NVIDIA; fail immediately when the GPU or Toolkit is unavailable
HEX_ARM_GPU=nvidia ./scripts/docker-dev.sh up

# Disable the NVIDIA override and use AMD/Intel DRI or software rendering
HEX_ARM_GPU=none ./scripts/docker-dev.sh up
```

Without the helper, set `HEX_ARM_XAUTHORITY` as shown above and explicitly add
the NVIDIA override:

```bash
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml up -d
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml \
  exec -T ros2-jazzy-arm nvidia-smi
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml \
  exec -T ros2-jazzy-arm glxinfo -B
```

The override uses Compose's standard `!reset` tag. If `docker compose config`
does not recognize it, update the Docker Compose plugin first.

A successful `doctor` reports the RTX model,
`OpenGL renderer string: NVIDIA ...`, and `NVIDIA GPU acceleration: OK`.
`llvmpipe` means CPU software rendering. The image already contains its
GLVND/OpenGL userspace dependencies; do not add the NVIDIA kernel or host driver
packages to the Dockerfile. RViz/Gazebo shares GPU memory and compute with
host CUDA workloads such as LeRobot training, so avoid running them together.



## WSL2 (Windows 10/11 only)

WSL2 means **Windows Subsystem for Linux 2**, the Linux virtualization
environment built into Windows. Ubuntu is running under WSL2 only when it was
started from Windows and `uname -r` contains `microsoft-standard-WSL2`.
Linux GUI windows are displayed through WSLg.

Run the following commands in the **Ubuntu/WSL terminal** in Windows, not in
PowerShell or Command Prompt. Replace the first path if the repository is
elsewhere:

```bash
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2
./scripts/docker-dev.sh build
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

The same helper selects `compose.yaml` and the WSLg sockets on WSL2. Neither
environment requires the overly broad `xhost +` command. The existing image is
already Ubuntu 24.04 (ROS 2 Jazzy), so a duplicate native-Ubuntu Dockerfile is
unnecessary.

