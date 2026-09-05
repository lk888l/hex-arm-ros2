# DISABLED observe and J1 baseline — 2026-08-20

Status: DISABLED-only observation completed; no drive was enabled and no
motion target was sent.

## Launch and artifact

- Launch start: `2026-08-20 13:42:55 +08:00`.
- Launch stop: `2026-08-20 13:44:18 +08:00`.
- Installed controller SHA-256:
  `3de9b78d632c47ba867a946907edf9bb4bedfd7771e9030779e3b05061e04c7e`.
- Supervisor audit log:
  `/tmp/hex-arm-real-launch.can2.81bQee/launch.log` inside container
  `ros2-jazzy-arm`.
- Supervisor audit-log SHA-256:
  `6a0a600f5f0eb785c4bb5aba7537e53f5ae2c8e4ffbaee7bb8051abeef2cfefd`.
- Adapter: HexMeow Quad CAN-FD, serial
  `C9E29601798421B29AC2D419C12D9502`, can2/channel 2, ifindex 35, USB
  device 11.

Actual launch command:

```bash
HEX_ARM_CAN_IFACE=can2 HEX_ARM_CAN_CHANNEL=2 \
  ./scripts/docker-dev.sh real-launch bringup \
  /workspaces/hex_arm_ros2/config/hardware/firefly_y6.discovered.local.yaml \
  activate_hardware:=false use_rviz:=false
```

The launch emitted all three required observations:

- `Firefly Y6 controller ready and DISABLED`;
- bridge configured with a fresh DISABLED-state observation;
- `OBSERVE MODE: ros2_control, hardware activation, and trajectory controllers remain stopped`.

No compressed-MIT enable confirmation, explicit hardware activation, or
ros2_control hardware-spawner start occurred.

## State observations

`/hex_arm/driver_state` reported:

- mode `1` (`DISABLED`), `session_owned=false`;
- `profile_valid=true`, `calibrated=false`;
- all motors online, feedback fresh, no latched fault and fault code zero;
- exact profile identities for nodes 1 through 6 and auxiliary node 15.

Two `/hex_arm/internal/state` samples approximately 23 seconds apart were
bit-for-bit identical in the printed values:

| Joint | Position (rad) | Velocity (rad/s) | Reported effort (Nm) |
| --- | ---: | ---: | ---: |
| joint_1 | +0.186926305 | -0.0 | -0.0 |
| joint_2 | -1.568824410 | -0.0 | +1.189270735 |
| joint_3 | +3.138803959 | +0.0 | +0.0 |
| joint_4 | +0.007919515 | +0.0 | +0.0 |
| joint_5 | +0.019357033 | +0.0 | +0.0 |
| joint_6 | +0.132728174 | +0.0 | +0.0 |

For the current J1 direction `-1` and candidate zero offset `-0.025651 rad`,
the observed J1 position corresponds to approximately
`-0.033832728927 rev`. A future positive 5 mrad ROS target would correspond to
a negative raw change of `-0.000795774715 rev`, ending near
`-0.034628503642 rev`; this is a prediction for direction checking, not a
motion result.

The ROS diagnostics temperature compatibility fields remained unchanged
between the two samples:

| Joint | Compatibility temperature (degC) |
| --- | ---: |
| joint_1 | 29.2 |
| joint_2 | 29.7 |
| joint_3 | 29.3 |
| joint_4 | 31.3 |
| joint_5 | 31.0 |
| joint_6 | 30.9 |

These fields select motor temperature when available; they do **not** expose
driver and motor temperatures independently. This observation therefore does
not satisfy the dual-temperature gate required before a first J1 motion.

## Shutdown and decision

One SIGINT was relayed only to the supervised launch process group. The
controller logged orderly drive disable and heartbeat-consumer disarm, nodes
1 through 6 were each confirmed disabled/disarmed, all child processes exited
cleanly, and the supervisor reported:

`VERIFIED clean controller exit after the orderly disable/heartbeat-disarm path`

The post-run audit retained the same can2 ifindex 35 / USB device 11 and
reported ERROR-ACTIVE with TEC/REC `0/0`. All CAN xstats and netdev
error/drop counters remained zero. Expected initialization/shutdown traffic
raised TX from `7271 packets / 285231 bytes` to
`9214 packets / 289111 bytes`; two subsequent samples showed no further TX.
There was no new `gs_usb`, `-EPROTO`, disconnect, reset, or re-enumeration
entry, no can2 socket owner, no listener on TCP 7448, and no launch/controller
process left behind. The unrelated can1 socket was not touched.

J1 `q=+0.186926305 rad` is inside the temporary `[-0.25,+0.25] rad` profile
window, and a positive 5 mrad survey would remain inside it. This is only a
fresh, static direction-test baseline. It does not validate J1 zero offset,
negative direction motion, physical limits, torque scale, tracking, or
temperature safety. No J1 motion is admitted until a fixed-policy diagnostic
adds independent driver/motor temperature gates, tighter enable stability,
low torque caps, exact target readback, and selected-axis-first cleanup.
