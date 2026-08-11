# MoveIt graphical mock simulation

**English** | [中文](moveit_simulation_cn.md)

This workflow exercises MoveIt, OMPL, collision checking, inverse kinematics,
`FollowJointTrajectory`, and the existing ros2_control `GenericSystem`. It does
not access USB/CAN hardware and is not a physics simulation.

Run these commands inside the `ros2-jazzy-arm` container:

```bash
cd /workspaces/hex_arm_ros2
./scripts/build.sh
source install/setup.bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py
```

Stop any existing `hex_arm_bringup mock.launch.py` session first when it uses
the same `ROS_DOMAIN_ID`; both launch files intentionally use the same
controller names. If RViz reports `could not connect to display :0` from a
long-running Docker container, stop the launch and restart/recreate the
container so its `/tmp/.X11-unix` and `/mnt/wslg` mounts pick up the current
WSLg sockets, then run the command again.

In RViz, select the `arm` planning group. Drag the interactive marker or choose
the named `ready` state, then use **Plan** first and **Plan & Execute** after the
preview looks correct. The execution path is:

```text
MoveIt RViz -> move_group -> firefly_arm_controller
             -> ros2_control GenericSystem -> /joint_states -> RViz
```

The default `sim` profile keeps the URDF limit of `6.0 rad/s` and uses a default
MoveIt scaling factor of 0.1. To preview the deliberately slow first-hardware
policy without connecting hardware:

```bash
ros2 launch hex_arm_moveit_config moveit_mock.launch.py \
  limits_profile:=commissioning
```

Available profiles are:

| Profile | Per-joint maximum | Intended use |
|---|---:|---|
| `sim` | 6.0 rad/s | software-only graphical testing |
| `commissioning` | 0.2 rad/s | first supervised hardware commissioning |
| `verified` | 6.0 rad/s | only after staged real-hardware verification |

MoveIt limits are planning limits, not the real safety boundary. A real launch
must use a verified local hardware profile whose `velocity_rad_s` is no greater
than the selected MoveIt profile. The Rust controller independently rate-limits
position-target changes using that hardware profile. MoveIt needs acceleration
limits to time-parameterize trajectories: `sim` uses the historical 10.0
rad/s^2 value only for software visualization, while both real-oriented profiles
stay at a provisional 0.2 rad/s^2 until measured. No real acceleration safety
claim is made, and acceleration is not yet enforced independently in Rust.

The new SRDF keeps non-adjacent self-collision checking enabled. Only the six
directly adjacent link pairs are excluded; the historical SRDF that disabled all
21 link pairs is intentionally not reused. The first version uses `link_6` as
the planning tip until a calibrated fixed TCP/tool frame is available.

Run the non-GUI planning and execution smoke test with:

```bash
python3 src/hex_arm_moveit_config/test/test_moveit_mock.py
```
