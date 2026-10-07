# Firefly Y6: GUI startup and command-line simulation control guide

**English** | [中文](gui_and_cli_simulation_cn.md)

> Scope: `hex_arm_ros2` on native Ubuntu 24.04 or WSL2 with Docker
> (ROS 2 Jazzy).

## Native Ubuntu 24.04

Using the old WSL2 Compose file on native Ubuntu supplies a nonexistent
`/mnt/wslg/runtime-dir` and does not pass the desktop session's Xauthority
cookie. The typical failure starts with `Authorization required`, followed by
RViz reporting `could not connect to display :0`.

Run the following from the repository root in a terminal opened by the desktop
session:

```bash
./scripts/docker-dev.sh build
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
./scripts/docker-dev.sh shell
```

`doctor` should report at least `X11 authorization: OK` and an OpenGL
version. The native Compose file mounts the current Xauthority read-only and
does not disable X server access control; do not run `xhost +`. This also
works on a GNOME Wayland session through Xwayland.

Inside the container, start MoveIt with:

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

An OpenGL renderer of `llvmpipe` is functional software rendering, though
Gazebo will be slower.

### NVIDIA GPU checks and fallback

On an NVIDIA host, first install Container Toolkit as described in the
[Docker development guide](docker_development.md#nvidia-discrete-gpu-acceleration). The native Ubuntu
helper then adds `compose.nvidia.yaml` automatically:

```bash
./scripts/docker-dev.sh up
./scripts/docker-dev.sh doctor
```

`doctor` must report the RTX model inside the container, an NVIDIA OpenGL
renderer, and `NVIDIA GPU acceleration: OK`. Force NVIDIA mode when a silent
CPU-rendering fallback would be unacceptable:

```bash
HEX_ARM_GPU=nvidia ./scripts/docker-dev.sh doctor
```

Temporarily disable the NVIDIA override and retain the existing `/dev/dri`
path with:

```bash
HEX_ARM_GPU=none ./scripts/docker-dev.sh up
```

The equivalent direct Compose command is:

```bash
docker compose -f compose.ubuntu.yaml -f compose.nvidia.yaml up -d
```

Container Toolkit injects devices and driver libraries that match the host.
The project Dockerfile does not install the NVIDIA kernel driver or CUDA SDK.
Toolkit is not required merely to open the GUI windows.

## WSL2-specific conclusions

1. **Root cause of the missing GUI windows, verified on this machine**: the current
   `ros2-jazzy-arm` container was created from Windows (PowerShell / Docker Desktop).
   The workspace was mounted through a `\\wsl.localhost\...` UNC path, and the
   `/tmp/.X11-unix` mount declared in `compose.yaml` did not take effect. Although
   an X socket file appears inside the container, a connection cannot be established.
   RViz and `joint_state_publisher_gui` therefore exit with
   `qt.qpa.xcb: could not connect to display :0`. The launch files, robot model,
   and ros2_control are otherwise working.
2. **Fix**: recreate the container from an **Ubuntu WSL shell**, not PowerShell,
   as described in the WSL2 diagnosis below.
3. **Command-line simulation control works** and has been verified on this machine.
   Both `mock` (RViz visualization) and `gz` (Gazebo physics simulation) expose
   the standard `FollowJointTrajectory` action. Drive either mode with
   `ros2 action send_goal` or a short Python program; see Section 4.

---

## WSL2: verify WSLg itself first

Run the following in an **Ubuntu-24.04 WSL terminal**:

```bash
env | grep -E 'DISPLAY|WAYLAND_DISPLAY'
ls -la /tmp/.X11-unix        # X0 socket should be present
ls /mnt/wslg                 # .X11-unix, PulseServer, and similar entries should be present
```

Observed healthy result on this machine:

```text
DISPLAY=:0 WAYLAND_DISPLAY=wayland-0
/tmp/.X11-unix/X0            # srwxrwxrwx socket
```

The following Python snippet performs a direct handshake with the X server.
A successful connection confirms that WSLg is healthy:

```bash
python3 - <<'EOF'
import socket
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect('/tmp/.X11-unix/X0')
s.sendall(b'l\x00\x00\x0b\x00\x00\x00\x00\x00\x00\x00\x00')
print('X OK, reply:', s.recv(8).hex())
EOF
```

Normal output looks like `X OK, reply: 00190b0000000700`. An error or hang means
WSLg itself needs attention first, such as `wsl --update` or a WSL restart; that
problem is independent of Docker.

---

## WSL2 GUI diagnosis

### Symptom

After running `ros2 launch hex_arm_bringup view.launch.py` as described in the
README, the terminal reports:

```text
[rviz2-3] qt.qpa.xcb: could not connect to display :0
[rviz2-3] qt.qpa.plugin: Could not load the Qt platform plugin xcb ...
[ERROR] [rviz2-3]: process has died ...
[joint_state_publisher_gui-2] qt.qpa.xcb: could not connect to display :0
[ERROR] [joint_state_publisher_gui-2]: process has died ...
```

`robot_state_publisher` remains healthy while only the windowed processes exit.
This is the typical signature of a container that cannot reach the X server.

### Root cause: WSL2 measurements

`docker inspect ros2-jazzy-arm` showed:

| Check | Result | Meaning |
|---|---|---|
| Workspace mount source | `\\wsl.localhost\Ubuntu-24.04\home\kk_wsl\...` | UNC mount created from Windows |
| Container hostname | `docker-desktop` | The engine is Docker Desktop, not a native dockerd inside WSL |
| Mount list | Workspace and `/mnt/wslg` only | The `/tmp/.X11-unix` mount declared by `compose.yaml` is **missing** |
| X socket inside the container | `stat` sees a file, but `connect()` returns `FileNotFoundError` | The socket is unusable |
| X socket on the WSL host | Handshake succeeds with `X OK, reply: 00190b0000000700` | WSLg itself is healthy |
| New container with a correct `/tmp/.X11-unix` mount | Handshake succeeds | Correcting the mount resolves the problem |

Conclusion: **the container was created through Docker Desktop from Windows, and
its X11 socket mount is missing or damaged, so Qt/X11 programs cannot connect to
`:0`**. The README instruction to run from an Ubuntu 24.04 WSL shell rather than
PowerShell exists to prevent this situation.

### Recommended fix: recreate the container from WSL

> If VS Code is currently attached to this container through Dev Containers,
> disconnect from the container first. Otherwise, VS Code may automatically stop
> or recreate it while you are working.

Run the following commands in an **Ubuntu 24.04 WSL terminal**, not in
PowerShell:

```bash
cd /home/kk_wsl/ros2_ws/code/hex_arm_ros2
docker compose down
docker compose up -d
docker exec -it ros2-jazzy-arm bash
```

After entering the container, confirm that the graphical socket is present:

```bash
env | grep -E 'DISPLAY|WAYLAND_DISPLAY'
ls -la /tmp/.X11-unix
docker inspect ros2-jazzy-arm | grep -A 10 Mounts
```

The expected environment includes `DISPLAY=:0`, and the mount list should
contain both `/mnt/wslg` and `/tmp/.X11-unix`.

Then launch the visualization:

```bash
cd /workspaces/hex_arm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hex_arm_bringup view.launch.py
```

If the RViz window appears and the six sliders in
`joint_state_publisher_gui` move the model, the graphical path is working.

### Temporary alternative: start a separate GUI-capable container

If the current development container must remain untouched, create a second
container from an Ubuntu 24.04 WSL terminal:

```bash
docker run -d --name ros2-jazzy-gui \
  --network host \
  --ipc host \
  -e DISPLAY=:0 \
  -e WAYLAND_DISPLAY=wayland-0 \
  -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
  -e PULSE_SERVER=/mnt/wslg/PulseServer \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v /home/kk_wsl/ros2_ws/code/hex_arm_ros2:/workspaces/hex_arm_ros2 \
  osrf/ros:jazzy-desktop
```

Enter that container, build or source the workspace, and run the same launch
command. This is useful for diagnosis, but recreating the main container from
WSL is the cleaner long-term solution.

## 4. Command-line simulation control

The project provides three launch modes:

| Mode | Launch command | Purpose |
|---|---|---|
| Model viewing | `ros2 launch hex_arm_bringup view.launch.py` | Inspect the URDF and move joints manually; no trajectory controller |
| Mock hardware | `ros2 launch hex_arm_bringup mock.launch.py` | Fastest way to test the ros2_control trajectory interface |
| Gazebo simulation | `ros2 launch hex_arm_bringup gz_sim.launch.py` | Test the same controller interface with simulated dynamics |

Both mock hardware and Gazebo expose the same action:

```text
/firefly_arm_controller/follow_joint_trajectory
```

The canonical joint order is:

```text
joint_1, joint_2, joint_3, joint_4, joint_5, joint_6
```

The URDF position limits are:

| Joint | Minimum (rad) | Maximum (rad) |
|---|---:|---:|
| `joint_1` | -2.86 | 2.86 |
| `joint_2` | -1.57 | 2.09 |
| `joint_3` | -1.57 | 1.57 |
| `joint_4` | -1.57 | 1.57 |
| `joint_5` | -1.54 | 1.54 |
| `joint_6` | -2.79 | 2.79 |

Keep every test target inside these limits. The simulated MoveIt configuration
may use the nominal URDF velocity limit of `6.0 rad/s`; the real commissioning
profile remains independently capped at `0.1 rad/s` until the hardware is
validated.

### 4.1 Send a trajectory to mock hardware

Terminal 1:

```bash
cd /workspaces/hex_arm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hex_arm_bringup mock.launch.py
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/hex_arm_ros2/install/setup.bash

ros2 action send_goal \
  /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  'trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, -0.32, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}'
```

A successful run reports that the goal was accepted and finishes with
`STATUS_SUCCEEDED`. You can watch the joint states in another terminal:

```bash
ros2 topic echo /joint_states
```

Useful inspection commands:

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 action list -t
```

### 4.2 Send the same trajectory to Gazebo

Terminal 1:

```bash
cd /workspaces/hex_arm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hex_arm_bringup gz_sim.launch.py
```

Wait until Gazebo, `robot_state_publisher`, and
`firefly_arm_controller` are ready. Then run the same
`ros2 action send_goal` command from section 4.1 in a second terminal.

Using one controller action for mock hardware, Gazebo, MoveIt, and the real arm
is intentional: clients do not need to change when the execution backend
changes.

To display the Gazebo window, launch it explicitly with:

```bash
ros2 launch hex_arm_bringup gz_sim.launch.py headless:=false
```

The default `headless:=true` runs physics without a window. Gazebo mode uses
simulation time, so trajectory execution follows the Gazebo clock.

### 4.3 Drive the arm from Python

Save the following as `drive_arm.py`. It uses the same action API as
`src/hex_arm_bringup/test/trajectory_test_common.py`:

```python
#!/usr/bin/env python3
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f'joint_{i}' for i in range(1, 7)]


def main() -> None:
    rclpy.init()
    node = Node('drive_arm')
    client = ActionClient(
        node,
        FollowJointTrajectory,
        '/firefly_arm_controller/follow_joint_trajectory',
    )
    if not client.wait_for_server(timeout_sec=10):
        raise SystemExit('action server unavailable; start the mock or Gazebo launch first')

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
        raise SystemExit('goal was rejected')

    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    print('result status:', result.result().status)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

```bash
python3 drive_arm.py
```

### 4.4 Headless automated verification

The repository test scripts exercise the complete mock or Gazebo trajectory
path by sending two trajectories and cancelling one. No window is required:

```bash
./scripts/test.sh mock
./scripts/test.sh gz
```

## 5. Quick troubleshooting

| Error or symptom | Likely cause | Resolution |
|---|---|---|
| `qt.qpa.xcb: could not connect to display :0` | Xauthority is missing on native Ubuntu, or the WSL2 X socket mount is broken | Run `./scripts/docker-dev.sh doctor` on the host, then recreate the matching container |
| `command not found: ros2` or `rviz2` | The ROS environment was not sourced | Run `source /opt/ros/jazzy/setup.bash` or enter an interactive container shell |
| `Package 'hex_arm_bringup' not found` | The workspace installation was not sourced or built | Run `source /workspaces/hex_arm_ros2/install/setup.bash`; if it does not exist, run `./scripts/build.sh` |
| `docker compose` was run from a PowerShell path | Docker received a UNC workspace path and the X socket became unusable | Run all Compose commands from an Ubuntu WSL shell |
| Gazebo is slow or its window is blank | WSLg is using software rendering | This can be normal; use headless mode if the graphical window is unnecessary |
| `ros2 action send_goal` cannot find the server | The launch is still starting or the controller is inactive | Wait briefly, then confirm with `ros2 action list -t` and `ros2 control list_controllers` |

## 6. Diagnostic command record

These commands summarize the checks used to isolate the GUI issue and verify
trajectory control:

```bash
# WSL host: verify that /tmp/.X11-unix/X0 accepts a handshake
python3 x11probe.py

# Existing container: inspect environment, mounts, and the X11 handshake
docker exec ros2-jazzy-arm env
docker inspect ros2-jazzy-arm
docker exec ros2-jazzy-arm python3 x11probe.py

# Control experiment: mount the X socket in a fresh container
docker run --rm -v /tmp/.X11-unix:/tmp/.X11-unix \
  --entrypoint python3 hex-arm-jazzy:local x11probe.py

# Headless trajectory verification
./scripts/test.sh mock
ros2 action send_goal ...
```
