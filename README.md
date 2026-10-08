# Firefly Y6 ROS 2 driver

**English** | [中文](README_cn.md)

ROS 2 Jazzy drivers, MoveIt 2 planning, and Gazebo simulation for the six-axis Firefly Y6.
The default build includes Meow and CiA402 drivers. Current hardware uses
**Meow firmware on can2, without a gripper or extra payload**.

| Task | Entry point | Run commands in |
|---|---|---|
| Plan and execute in simulation | [MoveIt mock](#moveit-quick-start) | Container |
| Control the current arm with MoveIt | [Real MoveIt](#real-hardware) | Host |
| Hand guide with gravity compensation and damping | [Hand guiding](#hand-guiding) | Container |
| Preview the model, ordinary mock, or Gazebo | [Other simulation modes](#simulation-modes) | Container |

Navigation: [Environment and build](#environment) · [Real MoveIt](#real-hardware) ·
[Development and tests](#development) · [Troubleshooting](#troubleshooting) · [Documentation](#documentation)

<a id="environment"></a>

## Environment and build

On native Ubuntu 24.04, use a desktop terminal. On WSL2, use the Ubuntu/WSL terminal.
Run `docker-dev.sh` on the **host**, and `build.sh` and `ros2` inside the **container**:

| Environment | Workspace | Identification |
|---|---|---|
| Host | `/home/kk/kk_data/ros2_project/hex-arm-ros2` | Your local user terminal |
| Container | `/workspaces/hex_arm_ros2` | `root@hex-arm-dev`, container name `ros2-jazzy-arm` |

Replace the host `cd` path if needed; the container mount remains `/workspaces/hex_arm_ros2`.
On first use or after Dockerfile changes, run `./scripts/docker-dev.sh build` in the host repository.
Start and enter the container:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
./scripts/docker-dev.sh up
./scripts/docker-dev.sh shell
```

On first use or after source changes, build inside the container:

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh
source install/setup.bash
```

Skip `build.sh` when already built; source `install/setup.bash` in each new container terminal.
Complete normal hardware shutdown before editing profiles, rebuilding, or switching controllers.

<a id="docker-setup"></a>
<a id="nvidia-discrete-gpu-acceleration"></a>

### Docker and graphics

The helper selects native Ubuntu X11 or WSL2 WSLg configuration automatically.
NVIDIA hosts need Container Toolkit; `HEX_ARM_GPU=auto` selects the GPU automatically, and `none`
disables NVIDIA. Use `HEX_ARM_HEADLESS=1` without a desktop. Diagnose the environment with
`./scripts/docker-dev.sh doctor` on the host.

See [Docker development](docs/docker_development.md) for direct Compose, NVIDIA installation,
and WSL2 setup, or [GUI diagnosis](docs/gui_and_cli_simulation.md) for display and OpenGL problems.

<a id="moveit-quick-start"></a>

## MoveIt mock

After preparing the container and build above, run inside the container:

```bash
cd /workspaces/hex_arm_ros2
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

This includes mock hardware, the trajectory controller, `move_group`, and RViz with MotionPlanning.
It uses `mock_components/GenericSystem` and does not connect to the arm.
The initial pose is `startup_ready = [0,-1.350,1.430,-0.300,0,0]` rad.

Select planning group `arm` and the current start state in RViz, then drag the end-effector with
**Interact**. Click **Plan** to inspect the preview, then **Plan & Execute** for simulated execution.
Stop with Ctrl+C in the launch terminal and wait for its children to exit.

| Optional argument | Purpose |
|---|---|
| `use_rviz:=false` | Planning and mock execution without a window |
| `limits_profile:=commissioning` | Conservative low-speed limits; default `sim`, also accepts `verified` |

`limits_profile` only selects mock planning limits. Use Gazebo for physics.
See the [MoveIt simulation guide](docs/moveit_simulation.md) for operation, collision models, and limits.

<a id="real-hardware"></a>

## Real MoveIt

These commands target the current replacement arm on can2. Use the local
`config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml`
with its motor identities, directions, encoder offsets, and motion envelope.
`*.local.yaml` is excluded from Git. For a new checkout or another arm, create a profile for the
actual hardware and verify calibration; see [zero calibration (Chinese)](docs/zero_calibration_cn.md).

### 1. Check the entry posture

Stop GUI MIT, gravity compensation, and other control entries. Run one hardware controller and
one motion client at a time. After each power cycle, place the arm at the folded entry below
before automatic startup. All values are radians:

| Posture | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Folded entry `folded_position_rad` | 0 | **−1.570** | **1.570** | 0 | 0 | 0 |
| Startup result `startup_ready` | 0 | −1.350 | 1.430 | −0.300 | 0 | 0 |

Absolute encoders check the entry posture. Startup follows **J2 → J4 → J3**:
J2 to −1.350 (8 s), J4 to −0.300 (10 s), and J3 to 1.430 (6 s).
The authoritative recipe is [startup.yaml](src/hex_arm_controller/config/startup.yaml).

Folded placement allows **0.01 rad (about 0.57°)** of J2/J3 variation around the reference.
The deployment profile also sets `measured_position_margin_rad: 0.01`, accepting J2 from
−1.580 to −1.560 rad and J3 from 1.560 to 1.580 rad at startup. The activation hold and
trajectory start bound small endpoint deviations to legal command positions. Actual feedback
continues to drive gravity and fault checks; zero offsets and normal command limits retain their values.

### 2. Launch MoveIt (complete host command)

**If your prompt is `root@hex-arm-dev`, run `exit` first to return to the host.**
Then run from a host graphical terminal:

```bash
cd /home/kk/kk_data/ros2_project/hex-arm-ros2
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh up
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.moveit_deployment.local.yaml \
  enable_execution:=true \
  position_limits:=hardware \
  dynamics_limits:=custom \
  planning_limits_file:=/workspaces/hex_arm_ros2/src/hex_arm_moveit_config/config/joint_limits_deployment.yaml \
  align_folded:=true \
  allow_enable_transient:=true
```

This enables and automatically moves the arm; leave the terminal running.
Profile and planning YAML arguments use **absolute paths inside the container**.
The supervisor verifies adapter serial and physical channel, forwards signals,
retains audit logs, and verifies final disable.

| Argument | Effect |
|---|---|
| `enable_execution:=true` | Enables hardware and permits trajectories after startup verification |
| `position_limits:=hardware` | Intersects URDF and hardware profile position limits |
| `dynamics_limits:=custom` + `planning_limits_file:=...` | Intersects production planning dynamics with hardware; velocity also respects the URDF cap |
| `align_folded:=true` | Allows bounded base and wrist alignment at the folded entry |
| `allow_enable_transient:=true` | Allows a brief velocity allowance during enable and gravity ramp, followed by stationary verification |

Append `use_rviz:=false` for services without RViz. Without a graphical desktop, also set
`HEX_ARM_HEADLESS=1` before both `up` and `real-launch`; the helper disables RViz automatically.
For disabled observation and planning, set `enable_execution`, `align_folded`, and
`allow_enable_transient` **all to `false`**.

### 3. Wait for verification, then execute

Startup movements are followed by a **10-second stationary ready hold**.
MoveIt execution actions remain unavailable during startup; RViz opens after verification.
Wait for `MoveIt startup verified: execution available`, then select `arm` and the current start
state in RViz for **Plan** / **Plan & Execute**. Headless clients also wait for this message.
`startup_ready:=false` cannot bypass automatic hardware startup.

Current production settings:

| Setting | Value or behavior |
|---|---|
| Position ranges | J1 ±2.86; J2 [−1.57,2.09]; J3/J4 ±1.57; J5 ±1.54; J6 ±2.79 rad |
| Hardware velocity / acceleration | 1.256637 rad/s / 1.256637 rad/s² |
| MoveIt planning velocity / acceleration | 1.256637 rad/s / **0.6 rad/s²** |
| Kp / Kd | Kp `[100,100,150,110,80,80]` N·m/rad; Kd 15 N·m·s/rad on each axis |
| Gravity compensation | All scales 1.0; J2 feedforward cap ±5 N·m |
| Torque budget | Total and configured PD caps 1000‰; extra reserve 0, `pd_allocation: remaining` |

Feedforward and PD **share one total 100% ceiling**; each frame allocates the remaining budget to PD.
Independent N·m limits still apply. Old profiles without an explicit budget keep the 15% reserve
and `fixed` allocation.
Gravity feedforward continuously uses the measured six-axis pose, including J2 from 0 to 1.57 rad.

Limits and units:

- Multiply GUI Rev/s and Rev/s² by `2π` to obtain ROS / MoveIt / Rust rad/s and rad/s².
- Position and dynamics default to `commissioning`. Changing only positions or hardware speed still leaves the 0.1 rad/s and 0.1 rad/s² planning caps.
- Meow unfolding, folded return, and preparatory alignment use this launch's MoveIt velocity and acceleration caps, with durations calculated from travel. Return-to-ready uses 1.0/1.0 scaling. The 10-second startup stationary check remains.
- RViz velocity and acceleration scaling are additional multipliers; set both to 1.0 for full planning limits.
- The old `replacement.local.yaml` retains narrow windows; use `moveit_deployment.local.yaml` from the command above.

The 2026-09-30 trials passed three large-range MoveIt targets, return to ready, controlled folding,
and confirmed disable with a 650‰ total torque ceiling. The current 100% ceiling and remaining PD
allocation passed offline validation, without a new hardware motion trial. The complete URDF
boundary and added payloads have not been exhaustively tested.
See [replacement-arm deployment (Chinese)](docs/meow_replacement_deployment_cn.md) and
[measured results (Chinese)](docs/commissioning_evidence/2026-09-30-can2-moveit-deployment.md).

### 4. Normal shutdown

The first **Ctrl+C** in the owning `real-launch` terminal returns through ready using MoveIt,
then performs the controlled **J3 → J4 → J2** folded return and confirms disable.
A second Ctrl+C or a fault requests immediate shutdown.

Wait for the return result and `VERIFIED structured disabled_confirmed` before removing power.
Retain this run's `startup-ready.json`, `launch.log`, and `driver-shutdown.json` paths;
the separate stop tool requires the current startup report. Extra ROS terminals must use
the same container and `ROS_DOMAIN_ID`.
For another CAN interface, see [binding instructions (Chinese)](docs/meow_mit_deployment_cn.md#更换-can-接口).

<a id="hand-guiding"></a>

## Hand guiding with gravity compensation

This mode uses `Kp=0`, gravity feedforward, and velocity damping without unfolding or trajectory
tracking. Stop other controllers first, then run in the built container workspace:

```bash
cd /workspaces/hex_arm_ros2
source install/setup.bash
ros2 launch hex_arm_bringup gravity_comp.launch.py \
  hardware_profile:=/workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.hand_guiding_full_range.local.yaml \
  activate_hardware:=true \
  damping:='[2.0, 2.0, 2.0, 2.0, 2.0, 2.0]'
```

Support the arm during enable and gravity ramp; wait for `hand_guiding_ready` before guiding.
Damping slows motion without locking position. This mode has no MoveIt collision checking.
Support the arm before stopping, then call from another sourced container terminal:

```bash
ros2 service call /hex_arm_gravity_comp/stop std_srvs/srv/Trigger '{}'
```

Ctrl+C also stops without returning or folding.
See [hand-guiding instructions](docs/hand_guiding.md) for damping, feedback margins,
speed protection, and disable acknowledgements.

<a id="simulation-modes"></a>

## Other simulation modes

Run these commands in a built and sourced **container terminal**, selecting one mode at a time:

| Mode | Function |
|---|---|
| `view` | RViz and joint sliders for model preview |
| `mock` | ros2_control mock hardware with trajectory actions |
| `gz` | Gazebo Harmonic physics simulation |

```bash
# URDF and joint sliders
ros2 launch hex_arm_bringup view.launch.py

# Ordinary mock
ros2 launch hex_arm_bringup mock.launch.py use_rviz:=true

# Gazebo
ros2 launch hex_arm_bringup gz.launch.py headless:=false use_rviz:=true
```

These entries do not provide MotionPlanning. `view` uses an isolated namespace and radian sliders;
Centre sets J3 to model v2 zero. Stop with Ctrl+C in the launch terminal.

<a id="simulation-cli"></a>

### Send simulated trajectories from the CLI

Start `mock` or `gz` first, then send from another sourced container terminal:

```bash
ros2 action send_goal /firefly_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.15, 0.25, -0.32, -0.2, 0.15, -0.1], time_from_start: {sec: 2, nanosec: 0}}]}"
```

Success returns `SUCCEEDED` with `error_code: 0`. Include all six axes with positions in radians;
multi-point trajectories need increasing `time_from_start`. This interface bypasses MoveIt
collision checking, and mock/gz do not automatically reject out-of-limit commands.
Ctrl+C only exits the CLI sender. Field details and Python examples are in the
[CLI simulation guide](docs/gui_and_cli_simulation.md).

<a id="development"></a>

## Development and tests

### Control interface and architecture

MoveIt executes through the standard `control_msgs/action/FollowJointTrajectory` interface at
`/firefly_arm_controller/follow_joint_trajectory`:

```text
MoveIt
  -> ros2_control / firefly_arm_controller (100 Hz)
  -> hex_arm_hardware (C++)
  -> hex_arm_bridge (Python, ROS <-> Zenoh / Protobuf)
  -> hex_arm_controller (Rust, 500 Hz)
  -> SocketCAN / CAN-FD / Meow MIT motors
```

MoveIt checks collisions; the trajectory controller checks tracking errors. Rust owns the bus,
interpolation, coordinate conversion, gravity compensation, and low-level protection.
See [architecture and migration (Chinese)](docs/architecture_refactor_cn.md) for module ownership,
CiA402 support, and legacy tool builds.

### Test levels

Choose checks appropriate to the change, in a built and sourced container:

| Command | Scope |
|---|---|
| `./scripts/test.sh unit` | Hardware-free unit tests |
| `./scripts/test.sh protocol` | Mock motors, Zenoh, and the ROS bridge |
| `./scripts/test.sh mock` | Trajectory actions and controller lifecycle |
| `./scripts/test.sh gz` | Headless Gazebo trajectories |
| `python3 src/hex_arm_moveit_config/test/test_moveit_mock.py` | Headless MoveIt planning, collisions, and mock execution |

### Reproducibility

`hex_arm.repos` pins upstream revisions; the checked-in description preserves original meshes
and provenance. Run `vcs import . < hex_arm.repos` only to update or audit upstream sources;
daily builds use checked-in sources. Retain `install/` and its symlink targets under `build/`.

<a id="troubleshooting"></a>

## Troubleshooting

| Symptom | What to do |
|---|---|
| `docker is not installed or is not on PATH` with `root@hex-arm-dev` | Run `exit` and use `docker-dev.sh` on the host. If already on the host, check Docker and PATH |
| `ros2: command not found` or missing packages | Enter `ros2-jazzy-arm` and source `install/setup.bash`; run `build.sh` first if the build is missing |
| Sourcing reports missing paths | Check the container and mount; do not reuse this container's symlink-install on the host or in an old container |
| No window or display authorization errors | Run `docker-dev.sh doctor` in a host graphical terminal and follow the GUI guide |
| Model/sliders without MotionPlanning | `view` only previews the model; use MoveIt mock or the real entry above |
| RViz has not opened during hardware startup | Wait for startup and the 10-second hold; inspect `passed/error` in this run's `startup-ready.json` on failure |
| `another supervised real launch already owns can2` | Complete normal shutdown in the original launch terminal |
| `Meow torque ceiling lacks PD/gravity headroom` | Check for an old profile or driver; current deployment requires the `remaining` budget and rebuilt controller |
| `context is invalid` or `process has died` at shutdown | Inspect the complete log, startup report, and `disabled_confirmed` in `driver-shutdown.json`, rather than only trailing ROS errors |
| J2 slider and motor Rev have opposite signs | This arm's direction is −1: `q = −2π × motor_rev + zero_offset_rad`. Sliders show ROS angles |
| J3 differs from old records by about 1.57 rad | Model v2 uses `q_v2 = q_v1 − 1.57`; profiles require schema v3 and `joint_coordinate_version: 2` |
| Plan fails or the target collides/exceeds limits | Choose a reachable, collision-free target from the current state; `commissioning_start` is only a planning reference |

Avoid multiple arm launches in one `ROS_DOMAIN_ID`.
See the [startup gate record (Chinese)](docs/commissioning_evidence/2026-09-30-moveit-startup-gate.md)
for action gating and shutdown log details.

<a id="documentation"></a>

## Documentation

| Document | Purpose |
|---|---|
| [Replacement-arm Meow deployment (Chinese)](docs/meow_replacement_deployment_cn.md) | Current hardware settings, dynamics, torque budget, and controlled stopping |
| [Meow MIT and CAN binding (Chinese)](docs/meow_mit_deployment_cn.md) | Protocol units, interface migration, and profiles |
| [Hand-guiding instructions](docs/hand_guiding.md) | Gravity compensation, damping, feedback protection, and stopping |
| [Zero calibration (Chinese)](docs/zero_calibration_cn.md) | Replacement arms and recalibration |
| [MoveIt simulation](docs/moveit_simulation.md) | Planning window, collision models, and limit profiles |
| [GUI and CLI simulation](docs/gui_and_cli_simulation.md) | GUI troubleshooting, actions, and Python examples |
| [Docker development](docs/docker_development.md) | Compose, NVIDIA installation, and WSL2 |
| [Architecture and migration (Chinese)](docs/architecture_refactor_cn.md) | Module ownership, builds, and standalone deployment |
| [Hardware state and next steps](docs/commissioning.md) | Historical tuning and development plan |
| [Legacy CiA402 deployment (Chinese)](docs/cia402_deployment_cn.md) | Deployment and qualification before the firmware upgrade |
| [Commissioning records](docs/commissioning_evidence/README.md) | Trial results and supporting evidence |

Historical CiA402 results predate the Meow upgrade. Use the current arm's Meow profile and
commissioning records for its deployment.
