# Current hardware state and next steps

**English** | [中文](commissioning_cn.md)

Applies to this Firefly Y6 arm with Meow firmware and no gripper or extra payload.
Consolidated on 2026-09-09; hardware results are from testing through 2026-09-08.
The active profile is `config/hardware/firefly_y6.meow.can2.local.yaml`. Keep its motor identities,
directions and encoder offsets; this local profile is excluded from Git.

## Current state

- Six-axis can2 communication, measured feedback, MIT control and gravity compensation have been verified in operation.
- Repeated J2 → J4 → J3 startup succeeded. The latest 10-second hold had a maximum error of about 0.00196 rad.
- MoveIt small-motion execution and a 60-second hold passed. The operator reports normal behavior with no obvious buzzing or vibration in the current small window.
- A previous 0.015 rad J2 return exceeded the 0.005 rad goal tolerance. Its cause remains open and it belongs in repeatability testing.
- Full travel, higher speeds and payload changes remain unverified. Profile validation applies to this arm and the existing envelope.

## Current configuration

| Setting | Value |
|---|---|
| ROS / motor command rate | 100 Hz / 500 Hz |
| Motor mode | Meow MIT, `0x4401 = 4` |
| Kp, J1–J6 | `[80,80,120,110,80,80]` Nm/rad |
| Kd | 15 Nm·s/rad on every axis |
| Gravity scales | `[0,1.0,1.05,0.7,0,0]` |
| Gravity torque clamps | `[0.2,5,5,1,0.2,0.1]` Nm; zero-scale axes have no gravity feed-forward |
| PD / total output | J3 450‰, other axes 500‰; total 650‰ of motor peak |
| Velocity / acceleration caps | 0.1 rad/s / 0.1 rad/s² |
| Path / goal tolerance | 0.02 / 0.005 rad, 1 s goal wait |
| Command / feedback timeout | 100 ms each |
| Gravity vector / payload | `[0,0,-9.81]` m/s² / no added payload |

Joint command windows in radians: J1 `[-0.03,0.01]`, J2 `[-1.572,-1.33]`, J3 `[2.98,3.14]`,
J4 `[-0.32,0.01]`, J5 `[-0.03,0.01]`, J6 `[-0.25,0.02]`.
MoveIt intersects the model, commissioning and hardware limits; its J2 lower bound is −1.570.

## Start and stop

Use the verified folded entry pose `[0,-1.570,3.140,0,0,0]`. Entry checks happen before enable:
stationary feedback, profile bounds, placement differences up to 0.03 rad on J1/J5 and 0.01 rad on J2/J3/J4.
J6 is aligned within its selected bounds when needed; existing encoder offsets are retained.

From a graphical host terminal in the repository:

```bash
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh up
HEX_ARM_CAN_IFACE=can2 ./scripts/docker-dev.sh real-launch moveit \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.meow.can2.local.yaml \
  enable_execution:=true
```

Startup moves J2 to −1.350 in 8 s, J4 to −0.300 in 10 s, then J3 to 3.000 in 6 s.
Other joints finish at zero; J2/J3/J4 keep measured positions until their turn.
This fixed folded exit follows the operator-verified physical path. MoveIt execution uses strict collision checking.

Wait for `startup_ready reached and verified; controller continues holding`, then select `arm` and the current
start state in RViz. Use `startup_ready:=false` for measured-pose holding. Omitting `enable_execution:=true`
selects observation/planning.

Stop with Ctrl-C in the owning launch terminal and wait for `VERIFIED clean controller exit`.
Stop the current owner before switching applications, changing hardware profiles or rebuilding.
The CAN interface is configurable; rebinding also checks the physical adapter serial/channel.

## Next optimization steps

1. Repeat bidirectional small moves and holding at different poses, including the J2 return case. Tune one gain or compensation parameter at a time based on measured errors.
2. Reconcile physical travel, zero offsets, collision geometry and environmental obstacles across URDF, MoveIt and the hardware profile.
3. Expand position bounds by joint, then test coordinated motion. Increase speed and acceleration after each envelope passes.
4. Validate repeated power-up/startup, continuous trajectories, longer operation, temperature and communication-loss stop/recovery.
5. Freeze the software version, final profile, CAN binding and operating commands. Update the model and payload parameters when adding a gripper or load.

## Build artifacts

Cleanup retains `install/` and the files it links to in `build/` for the existing deployment.
Build caches, test outputs and historical logs have been removed. Stop control before rebuilding;
run `./scripts/build.sh` inside `/workspaces/hex_arm_ros2`, then source `install/setup.bash`.
The next rebuild recreates caches and takes longer. Future runs generate new logs as usual.
The current local hardware profile, earlier trial records and backup profiles are retained for reference.
Use this document for current operation and optimization steps.
