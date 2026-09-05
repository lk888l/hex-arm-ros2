# J1 negative fixed-position censored-torque identification — 2026-08-20

Status: PASS for the bounded negative-torque identification, exact rollback,
and confirmed cleanup. NO-GO for promoting a profile parameter, replaying the
downsweep, or enabling general position/MoveIt execution.

## Identity and fixed policy

- Run start: `2026-08-20 16:28:07 +08:00`.
- Git HEAD: `2800acc9a36d6495b3bdbf8e8eecd5abb3f36993`, with an intentionally
  dirty commissioning worktree.
- The raw transcript was not copied to a durable log file. The numerical
  record below is reconstructed directly from the complete captured terminal
  output; no raw-log SHA is claimed.
- Installed controller SHA-256:
  `a377a914844ac53d526594911d2d2d3ae2f730cc1516112e6ec8c02abde28fba`.
- Run-profile SHA-256:
  `1d082b04566d2e3398a622277ac62c71118c89a5d885c0b5c0c5ea2d9504dac3`.
- Source SHA-256 values:
  - `commissioning.rs`: `0dac2cda36d62cb1aa9306cb8aed7ebe5f6c36f04c13936281ee20444579304f`;
  - `main.rs`: `1c20cc939d759e8ce5c087f048d361a5bd860b734b21ce15d3eb6f022a28a77e`;
  - `backend.rs`: `80aff66e0a115331fd245c4ab452df19b0247329692bba269f3534816b36341d`.
- Adapter: HexMeow Quad CAN-FD, serial
  `C9E29601798421B29AC2D419C12D9502`, `can2`, channel `2`, ifindex `47`,
  USB device `14`, path `3-2.2:1.0`.
- Fixed policy: J1/node 1 only; position code fixed at the fresh start;
  joint-side Kp/Kd `60.0/2.5`; temporary drive permissions `30/20` permille;
  negative feed-forward levels of `0.025 Nm` held for `0.100 s`, capped at
  `0.600 Nm`; reject the current level at `-0.300 mrad`; hard-stop at
  `-0.500 mrad`, `+0.300 mrad`, or `0.005 rad/s`; measured and estimated total
  torque capped at `1.0 Nm`; six-axis dual-temperature gates active.

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/firefly_y6.discovered.local.yaml \
  --diagnose-axis joint_1 \
  --diagnostic-mode joint1-negative-torque-censored \
  --allow-diagnostic-motion \
  --acknowledge-joint1-negative-torque-censored
```

The strict-zero SocketCAN preflight passed with can2 ERROR-ACTIVE, TEC/REC
`0/0`, all CAN xstats and netdev error/drop counters at zero, and no can2
socket owner. The J2 `1 mrad` measurement-only hard-stop margin allowed its
passive feedback to be observed while its command lower limit remained exactly
`-1.570 rad`; J2 was never enabled.

## Observed sequence

- All six nodes initialized disabled; node 15 remained auxiliary-only. J1's
  `30/20` drive caps were written/read back before J1 alone reached strict MIT
  Operation Enabled (`0x0237`, mode display `5`, control-word readback
  `0x000F`).
- The zero-additive stability dwell passed in `0.254382 s` at
  `q0=-0.048916504 rad`, zero displacement and velocity, measured torque
  `0.037165 Nm`, and driver/motor temperatures `33.4/30.7 degC`.
- The fixed-position milestones were:
  - `-0.102613 mrad` at level 4 / `-0.100 Nm`;
  - `-0.205226 mrad` at level 6 / `-0.150 Nm`;
  - `-0.303350 mrad` at level 8 / `-0.200 Nm`, velocity
    `-0.00097125 rad/s`, measured torque `-0.111494 Nm`, estimated PD
    `+0.020629 Nm`.
- Level 8 was rejected before promotion. The controller restored the preceding
  feedback-proven, wire-distinct level 7 target (`-0.175 Nm`). Exact compressed
  target readback matched expected and actual words:
  `0xFF7FF83B / 0x990A7FF7`.
- During the bounded one-second frozen hold at `-0.175 Nm`:
  - samples: `194`;
  - mean/minimum/maximum displacement:
    `-0.312688 / -0.314958 / -0.308216 mrad`;
  - position standard deviation: `0.001962 mrad`;
  - peak velocity: `0.000211609 rad/s`;
  - mean measured/estimated-total torque: `-0.131801 / -0.156221 Nm`;
  - final 250 ms: `51` samples, zero quantized position span and effectively
    zero peak velocity; terminal stability passed.
- Whole-run peaks remained inside the hard envelope: absolute displacement
  `0.314958 mrad`, velocity `0.00177503 rad/s`, measured torque `0.222988 Nm`,
  estimated total torque `0.182608 Nm`, and driver/motor temperature
  `33.4/30.7 degC`. No position trajectory was constructed.

## Cleanup and decision

The feedback-proven frozen target was restored through the shared RPDO sender,
J1 was confirmed non-torque first, nodes 2..6 were confirmed non-torque, and
all heartbeat consumers were disarmed. The process exited `0`.

Post-run can2 remained ifindex `47`, ERROR-ACTIVE with TEC/REC `0/0`; all CAN
xstats and netdev error/drop counters remained zero. TX rose from `4724` to
`7783` packets and stopped. can2 had no residual socket or controller process,
and the kernel journal contained no USB/gs_usb error, disconnect, reset, or
re-enumeration event during the run.

Together with the positive run, this establishes path-dependent response
brackets of approximately `+0.425..+0.450 Nm` and `-0.175..-0.200 Nm` for the
two particular starts. Their asymmetry is consistent with preload, friction,
and/or backlash; neither frozen value is a calibrated Coulomb-friction or
gravity term. Do not store either as a profile bias or replay either sweep.
They may only size a new, independently authorized position experiment whose
compensation is zero at rest, smoothly follows requested motion direction, and
retains the existing torque/temperature/CAN/selected-first shutdown gates.

