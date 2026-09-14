# Firefly Y6 ROS 2 driver

**English** | [中文](README_cn.md)

See [architecture and production Docker migration](docs/architecture_refactor_cn.md)
for the modular Meow driver, opt-in legacy tools, and mandatory ordered startup.

ROS 2 Jazzy driver, MoveIt 2 motion planning, and Gazebo simulation workspace for
the six-axis Firefly Y6 arm. **To open the MoveIt 2 window without connecting an
arm, follow the quick start below.**

Navigation: [MoveIt 2 quick start](#moveit-quick-start) ·
[Other simulation modes](#simulation-modes) · [CLI control](#simulation-cli) ·
[Troubleshooting](#troubleshooting) · [Docker setup](#docker-setup) ·
[Real hardware](#real-hardware) · [Documentation](#documentation)

<a id="moveit-quick-start"></a>

## MoveIt 2 simulation window: quick start

This entry opens **RViz with the MotionPlanning panel** for inverse kinematics,
collision checking, trajectory planning, and simulated execution. It uses
ros2_control's `mock_components/GenericSystem`, with no USB/CAN hardware access.
It does not simulate gravity or contact physics; use Gazebo for physics simulation.

### 1. Enter the container from a host terminal

On native Ubuntu 24.04, use a terminal opened from the graphical desktop. On first
use or after changing the Dockerfile, run `./scripts/docker-dev.sh build` from
the repository root to build the image. For daily use:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

Replace the repository path if needed. On WSL2, run the same scripts from the
**Ubuntu/WSL terminal**, not PowerShell or Command Prompt; the helper selects
WSLg automatically. If the container is already running, go straight to
`./scripts/docker-dev.sh shell`.

### 2. Start MoveIt 2 inside the container

The container is named `ros2-jazzy-arm`, with prompt hostname `hex-arm-dev`.
On first use or after source changes, run `./scripts/build.sh` from
`/workspaces/hex_arm_ros2` to build the workspace. Once built, run the following
in each new container terminal:

```bash
cd /workspaces/hex_arm_ros2
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

`hex_arm_moveit_config` is the ROS package; `moveit_mock.launch.py` is its launch
file. It loads mock hardware, the joint state broadcaster, and
`firefly_arm_controller`, then starts `move_group` and RViz with the MoveIt
configuration. **This launch includes the required controllers; there is no need
to start `hex_arm_bringup mock.launch.py` separately.** Leave the terminal running
while using the RViz window.

This mock entry defaults to `[0,-1.350,1.430,-0.300,0,0]` rad, named
`startup_ready`. It initializes mock joints and is not the physical placement
posture used after a power cycle; real hardware starts from absolute-encoder feedback.
Firefly Y6 description v2 centers J3 at its mechanical/CAD zero. Convert legacy
J3 coordinates with `q_v2 = q_v1 - 1.57 rad`; hardware profiles must use schema
v3 with `joint_coordinate_version: 2`.

### 3. Plan and execute in the window

1. Select planning group `arm` in **MotionPlanning**, using the current robot state as the start.
2. Use RViz's **Interact** tool to drag the end-effector marker to a target position and orientation.
3. Click **Plan** to inspect the preview. If planning fails, check reachability, collisions, and joint limits.
4. Click **Plan & Execute** to plan and execute, then watch the simulated arm and joint states update.

`Plan` produces a preview. `Plan & Execute` drives simulated joints when using
this mock entry point. The named `commissioning_start` state is a planning
reference, not a calibrated hardware home or a guaranteed valid target under the
strict collision model.

### Common launch arguments

Place arguments after the launch filename, using `name:=value`.

| Argument | Default | Meaning |
|---|---|---|
| `use_rviz` | `true` | Opens RViz automatically; `false` runs planning and simulated execution without a window |
| `limits_profile` | `sim` | MoveIt velocity/acceleration limits; accepts `sim`, `commissioning`, or `verified`; use `sim` for routine simulation |

```bash
# Run MoveIt mock without a window
ros2 launch hex_arm_moveit_config moveit_mock.launch.py use_rviz:=false

# Preview conservative low-speed limits in the simulation window
ros2 launch hex_arm_moveit_config moveit_mock.launch.py limits_profile:=commissioning

# List launch arguments without starting nodes
ros2 launch hex_arm_moveit_config moveit_mock.launch.py --show-args
```

Changing `limits_profile` only changes planning limits; the backend stays mock.
See the [MoveIt simulation guide](docs/moveit_simulation.md) for limit values and
collision-model details.

Stop with `Ctrl+C` in the launch terminal and wait for all child processes to
exit before switching modes. Run one arm launch at a time in the same
`ROS_DOMAIN_ID` to avoid conflicting controllers, TF, and joint states.

<a id="simulation-modes"></a>

## Other simulation modes

Run these commands in a **built and sourced container terminal**, choosing one
mode at a time. MoveIt mock uses the `mock` backend and adds MoveIt planning
services and the MotionPlanning panel to the ordinary mock setup.

| Mode | Windows and capabilities | Use it for |
|---|---|---|
| **MoveIt mock** (above) | RViz + MotionPlanning + mock controllers | End-effector interaction, planning, simulated execution |
| `view` | RViz + joint sliders, no trajectory controller | Inspecting the URDF, joint directions, and limits |
| Ordinary `mock` | RViz + ros2_control mock hardware, no MoveIt panel | Testing trajectory actions and controller lifecycles |
| `gz` | Gazebo Harmonic + optional RViz, no MoveIt panel | Physics simulation and trajectory control |

```bash
# URDF + joint sliders + RViz
ros2 launch hex_arm_bringup view.launch.py

# Ordinary mock: send joint trajectories from the CLI or a script
ros2 launch hex_arm_bringup mock.launch.py use_rviz:=true

# Gazebo physics + RViz
ros2 launch hex_arm_bringup gz.launch.py headless:=false use_rviz:=true
```

These three commands do not load the MoveIt MotionPlanning panel. To open the
planning window, use `ros2 launch hex_arm_moveit_config moveit_mock.launch.py`.

Each `view` launch uses a unique `/hex_arm_view_<id>` namespace for its joint
states, robot description, and TF topics, so preview windows do not interfere
with each other or a running controller. Slider angles are radians; Center sets
J3 to 0 (the former 1.57 rad pose). Stop the launch with Ctrl+C when finished.

<a id="simulation-cli"></a>

## Driving the simulated arm from the CLI (`ros2 action send_goal` reference)

Start the ordinary `mock` or `gz` mode above. Open another host terminal,
enter the same container with `./scripts/docker-dev.sh shell`, then run:

```bash
cd /workspaces/hex_arm_ros2
source install/setup.bash
ros2 action send_goal /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, -0.32, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}"
```

A successful goal ends with `Goal finished with status: SUCCEEDED`
(`error_code: 0`). Check feedback with `ros2 topic echo --once /joint_states`.
This command sends a target directly to the trajectory controller, bypassing
MoveIt planning and collision checking. Supply six joint angles in radians,
within the URDF limits.

<details>
<summary>Expand: action syntax, trajectory fields, and usage details</summary>

### Command format

```text
ros2 action send_goal <action_server> <action_type> "<goal_yaml>"
```

| Argument | Example | Meaning |
|---|---|---|
| Subcommand | `ros2 action send_goal` | Sends an action goal; `ros2 action` also supports `list`, `info`, and `type` |
| Action server | `/firefly_arm_controller/follow_joint_trajectory` | The trajectory controller's action server. Start a `mock`/`gz` launch first, otherwise the CLI keeps printing `waiting` |
| Action type | `control_msgs/action/FollowJointTrajectory` | Tells the CLI how to parse the YAML; must match this workspace |
| Goal payload | YAML in double quotes | See field rules below |

### YAML field rules

- `trajectory.joint_names`: joint-name list. This example lists
  `joint_1` ... `joint_6`; all six must be present (the controller sets
  `allow_partial_joints_goal: false`, so partial goals are rejected).
- `trajectory.points`: array of trajectory points (one in this example). Each
  point has:
  - `positions`: target angles in **radians**, exactly six values in
    `joint_names` order. Keep them inside the URDF limits: joint_1 ±2.86,
    joint_2 −1.57~2.09, joint_3 ±1.57, joint_4 ±1.57, joint_5 ±1.54,
    joint_6 ±2.79. Note that the mock/gz configs do **not** enable command
    limit clamping, so out-of-limit values are not automatically rejected —
    make sure the numbers are safe yourself.
  - `time_from_start`: offset from the moment the goal is accepted;
    `{sec: 2, nanosec: 0}` means "reach within 2 seconds". The controller
    smooths the motion with `interpolation_method: splines`.
  - Optional: `velocities`, `accelerations`, `effort`; omitted fields are
    interpolated by the controller.
  - Add more points to build multi-segment trajectories, increasing
    `time_from_start` for each.

### Shell and usage details

- Wrap the whole YAML in **double quotes** so spaces are not split into
  separate shell arguments; the trailing `\` is just a line continuation and
  can be removed to put everything on one line.
- Source the environment first (interactive bash does this automatically;
  otherwise run `source /opt/ros/jazzy/setup.bash` and
  `source install/setup.bash`).
- After sending, expect `Goal accepted with ID: ...` and then
  `Goal finished with status: SUCCEEDED` (`error_code: 0`).
- Verify the final pose with `ros2 topic echo --once /joint_states`; the
  `position` values should match the goal.
- A new goal cancels/replaces the goal currently being executed (default
  controller behavior); `Ctrl-C` only exits the CLI client.
- The same command works for `gz`; there the trajectory advances with the
  Gazebo (simulation) clock.

More GUI and CLI troubleshooting: [docs/gui_and_cli_simulation.md](docs/gui_and_cli_simulation.md).

</details>

<a id="troubleshooting"></a>

## Troubleshooting

| Symptom | What to do |
|---|---|
| `ros2: command not found` | Enter the container with `./scripts/docker-dev.sh shell` on the host; inside, source `/opt/ros/jazzy/setup.bash` and `/workspaces/hex_arm_ros2/install/setup.bash` |
| Package `hex_arm_moveit_config` or `hex_arm_moveit_runtime` not found | Run `./scripts/build.sh` in `/workspaces/hex_arm_ros2` inside the correct container, then source `install/setup.bash` again |
| Sourcing reports missing `/workspaces/hex_arm_ros2/...` paths | Use `ros2-jazzy-arm` (hostname `hex-arm-dev`); do not reuse its symlink-install from the old `ros2-jazzy` container or the host |
| No window, `could not connect to display`, or `Authorization required` | Run `./scripts/docker-dev.sh doctor` from the repository root in a **host graphical terminal**, fix the issue, then re-enter and relaunch; do not use `xhost +` |
| Model/sliders appear, but no MotionPlanning panel | Stop that launch and use `hex_arm_moveit_config moveit_mock.launch.py` with `use_rviz:=true` |
| Duplicate controllers, jumping joint states, or controller startup stalls | Check for other arm launches; stop them normally in their own terminals and leave only one mode running |
| Plan fails, or a target collides or exceeds limits | Choose a reachable, collision-free target from the current state; `commissioning_start` is not a guaranteed valid demo target |

See the [GUI and CLI simulation guide](docs/gui_and_cli_simulation.md) for more
X11, WSLg, NVIDIA, and CLI troubleshooting.

<a id="docker-setup"></a>

## Docker development environment: advanced setup

For daily use, follow the helper commands in the quick start. On native Ubuntu,
`docker-dev.sh` selects `compose.ubuntu.yaml` and mounts the current X11
credential read-only. On WSL2 it selects `compose.yaml` and WSLg sockets.
`HEX_ARM_GPU=auto` adds `compose.nvidia.yaml` when a working NVIDIA GPU is detected;
this requires NVIDIA Container Toolkit on the host.

<details>
<summary>Direct Docker Compose commands</summary>

### Direct Docker Compose commands (without the helper)

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

</details>

<details>
<summary>NVIDIA GPU: installation and verification</summary>

### NVIDIA discrete GPU acceleration

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

</details>

<details>
<summary>WSL2 startup details</summary>

### WSL2 (Windows 10/11 only)

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

</details>

<a id="real-hardware"></a>

## Current hardware deployment

The current setup is a **Firefly Y6 with Meow firmware on can2, without a gripper or extra payload**.
Six-axis communication, gravity compensation, ordered J2 → J4 → J3 startup and small MoveIt motions
have been verified. Full travel, higher speeds and payload operation remain to be validated.
The configuration, open regression item and remaining work are maintained in
[Current hardware state and next steps](docs/commissioning.md).

### Start and stop

Use `config/hardware/firefly_y6.meow.can2.local.yaml` for this arm. It contains the current motor
identities, directions, encoder offsets and motion envelope and is excluded from Git. For another
arm, create a profile from `firefly_y6.meow_mit.example.yaml` and verify its calibration separately.

After each power cycle and before automatic startup, place the arm at the folded entry posture:

| Posture | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Power-cycle placement `folded_position_rad` | 0 | **−1.570** | **1.570** | 0 | 0 | 0 |
| Automatic startup result `startup_ready` | 0 | −1.350 | **1.430** | −0.300 | 0 | 0 |

All values are radians. Absolute encoders verify the folded entry before enable;
the required motion order remains **J2 → J4 → J3**. From a graphical host terminal in the repository:

```bash
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh up
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.local.yaml \
  enable_execution:=true
```

The entry checks feedback and position before enabling. Startup then moves **J2 to −1.350 (8 s),
J4 to −0.300 (10 s), and J3 to 1.430 (6 s)**, with the other joints ending at zero.
The authoritative values are in
[`src/hex_arm_controller/config/startup.yaml`](src/hex_arm_controller/config/startup.yaml),
with operating instructions in [Current hardware state](docs/commissioning.md#start-and-stop).
The operator has verified physical clearance for this ordered folded exit.
Wait for `startup_ready reached and verified; controller continues holding`, then select `arm`
and the current start state in RViz for `Plan` / `Plan & Execute`. Subsequent MoveIt execution
uses strict collision checks and the current local motion envelope.

Current Kp is `[80,80,120,110,80,80]`, Kd is 15 on every axis, and J2/J3/J4 gravity scales are
`1.0/1.05/0.7`. Velocity and acceleration caps are 0.1 rad/s and 0.1 rad/s²; see the deployment
status document for detailed bounds. Real execution always requires ordered startup;
`startup_ready:=false` cannot bypass it. Omitting `enable_execution:=true` selects
disabled observation and planning.

The local profile now opts into shutdown damping. The first Ctrl-C in the owning real-launch terminal
can move the arm: MoveIt returns to startup_ready, then Rust damps descent and confirms disable.
A second interrupt or a fault requests immediate teardown. Wait for both the soft-stop result and
`VERIFIED structured disabled_confirmed`; keep power on until completion. Gains await physical validation.
See [shutdown behavior and configuration](docs/shutdown_damping_cn.md); bare ros2 launch bypasses this wrapper.
Stop the current controller before switching applications, editing profiles or rebuilding.

### CAN selection and further development

The CAN interface is configurable. The current setup uses can2 at 1 Mbps arbitration / 4 Mbps data.
Interface selection also checks the USB adapter serial and physical channel. To change interfaces,
use `scripts/bind-can-profile.py` to create a profile that retains motor identities and calibration,
then select the matching interface through `HEX_ARM_CAN_IFACE`. See the
[Meow MIT interface binding instructions (Chinese)](docs/meow_mit_deployment_cn.md#更换-can-接口).

Cleanup retains `install/` and its required link targets in `build/`. After source changes, stop
control and run `./scripts/build.sh` in the container workspace, then source `install/setup.bash`.
Build caches will be recreated. Continue with repeatability tests, model and limit checks,
gradual envelope expansion, continuous operation and recovery tests, then freeze the deployment;
see [Next optimization steps](docs/commissioning.md#next-optimization-steps).

## Control interface and architecture

MoveIt sends planned trajectories to `joint_trajectory_controller` through the standard
`control_msgs/action/FollowJointTrajectory` action at `/firefly_arm_controller/follow_joint_trajectory`.
The current hardware path is:

```text
MoveIt / FollowJointTrajectory
  -> ros2_control + firefly_arm_controller (100 Hz)
  -> hex_arm_hardware/SystemInterface (C++ hardware plugin)
  -> hex_arm_bridge (Python, ROS <-> Zenoh / Protobuf)
  -> hex_arm_controller (Rust, 500 Hz motor command loop)
  -> SocketCAN (currently can2, configurable) -> CAN-FD / Meow MIT motors
```

- **ros2_control** manages controller and hardware lifecycles and cycles through reading state,
  updating controllers and writing targets. The trajectory controller generates timed joint
  targets and checks tracking and goal errors.
- **The hardware plugin** exposes six-axis position/velocity commands and position/velocity/effort
  feedback to ros2_control and exchanges data with the bridge through ROS interfaces.
- **The bridge** translates commands and feedback between ROS and the Rust Zenoh / Protobuf API
  and coordinates lifecycle transitions and exclusive control sessions. It does not implement
  a separate trajectory action.
- **The Rust driver** owns the motor bus and handles interpolation, direction and offset conversion,
  gravity compensation, MIT commands and feedback, plus joint limits, output caps and
  command/feedback timeouts.

MoveIt checks collisions, the trajectory controller checks tracking errors, and Rust handles
motor communication and low-level protections.

## Reproducibility

`hex_arm.repos` pins upstream source revisions. The checked-in
`xpkg_urdf_firefly_y6` package is a provenance-preserving snapshot of the
specified description revision, including original meshes. Run:

```bash
vcs import . < hex_arm.repos
```

only when updating or auditing upstream sources; the normal build does not
silently fetch mutable branches.

## Test levels

Run these commands in a built and sourced container terminal:

```bash
./scripts/test.sh unit
./scripts/test.sh protocol
./scripts/test.sh mock
./scripts/test.sh gz
```

`unit` is hardware-free. `protocol` starts the Rust controller with its mock
motor backend and validates Zenoh discovery/events plus the ROS lifecycle bridge.
`mock` exercises FollowJointTrajectory send/cancel and controller lifecycle. `gz`
launches Gazebo headlessly and verifies two trajectories. Real commissioning uses
a separate [hardware validation plan](docs/commissioning.md).

For a separate headless smoke test of MoveIt planning, strict collision checking,
and mock execution, run this after building and sourcing the workspace:

```bash
python3 src/hex_arm_moveit_config/test/test_moveit_mock.py
```

<a id="documentation"></a>

## Documentation

| Document | Contents |
|---|---|
| [MoveIt simulation guide](docs/moveit_simulation.md) | Simulation window, limit profiles, collision models, and real MoveIt planning entry |
| [GUI and CLI simulation guide](docs/gui_and_cli_simulation.md) | X11/WSLg/NVIDIA diagnosis, action usage, and Python examples |
| [Meow MIT deployment (Chinese)](docs/meow_mit_deployment_cn.md) | New firmware protocol, parameter units, profiles, and current deployment flow |
| [Current hardware state and next steps](docs/commissioning.md) | Current settings, start/stop commands, open regression item and optimization plan |
