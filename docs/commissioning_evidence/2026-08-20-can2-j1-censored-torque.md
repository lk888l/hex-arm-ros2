# J1 fixed-position censored-torque identification — 2026-08-20

Status: PASS for the bounded positive-torque identification and confirmed
cleanup. NO-GO for promoting a profile parameter, repeating the same upsweep,
or opening J1/MoveIt position execution.

## Artifacts and exact policy

- Run start: `2026-08-20 15:59:28 +08:00`.
- Git HEAD: `2800acc9a36d6495b3bdbf8e8eecd5abb3f36993`, with an intentionally
  dirty commissioning worktree. The installed binary hash below is the
  authoritative executable identity.
- Raw transcript:
  `log/commissioning/2026-08-20T1600+0800_can2_j1_censored_torque.log`
  (gitignored local artifact, `35634` bytes).
- Raw transcript SHA-256:
  `baa68d98e0932c0cdc8d0daa395166d0b5cc1a555ed196e31827a972fd2a4d86`.
- Installed controller SHA-256:
  `57582ae6bcbb0114f989264d30f823e3139db308fb6923ada69f1ea317ea8d71`.
- Run-profile SHA-256:
  `5a8a7cf972a42106608c6de7b6afa01c0b82cb8a521e23b7939c09433b48bcc8`.
- Runtime source SHA-256 values:
  - `backend.rs`: `2343894568d9a50fd187b80c5948d51ad1ce9cec5b438f3725fa29cb77ea6e0e`;
  - `commissioning.rs`: `c088b28cb91cceb7b299b75f5b3bb501d6a0dbb8ffd5a0567de579ca3a73adc9`;
  - `main.rs`: `290f75a37b23556e6575a9e2f53d83f7c737c0b0f52bcbcfe57b82e1e107c2bd`.
- Adapter: HexMeow Quad CAN-FD, serial
  `C9E29601798421B29AC2D419C12D9502`, `can2`, channel `2`, ifindex `47`,
  USB device `14`, path `3-2.2:1.0`.
- Fixed policy: J1/node 1 only; position code fixed at the fresh starting
  position; joint-side Kp/Kd `60.0/2.5`; temporary drive permissions `30/20`
  permille; positive feed-forward levels of `0.025 Nm` held for `0.100 s`,
  capped at `0.600 Nm`; reject the current level at `+0.300 mrad`; hard-stop
  at `+0.500 mrad`, `-0.300 mrad`, or `0.005 rad/s`; measured and estimated
  total torque capped at `1.0 Nm`; six-axis dual-temperature gates active.

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/firefly_y6.discovered.local.yaml \
  --diagnose-axis joint_1 \
  --diagnostic-mode joint1-torque-censored \
  --allow-diagnostic-motion \
  --acknowledge-joint1-torque-censored
```

This J1 mode used the ordinary strict-zero SocketCAN preflight and rejected
all J2 high-tier/historical-counter permissions. Before the run, the shared
adapter had remained in the same new USB epoch for more than 30 minutes under
the existing can1 load. can2 was ERROR-ACTIVE with TEC/REC `0/0`, every CAN
xstat and netdev error/drop counter was zero, and can2 had no socket owner.

## Observed sequence

- All six arm nodes initialized disabled; node 15 remained auxiliary-only.
  J1's `30/20` drive permissions were written and read back before J1 alone
  reached strict MIT Operation Enabled (`0x0237`, mode display `5`, control
  word `0x000F`).
- The zero-additive enable gate passed after `0.254394 s` at fixed command
  `q0=0.075290442 rad`: measured displacement `-0.004679 mrad`, essentially
  zero velocity and torque, and driver/motor temperatures `30.9/29.3 degC`.
- The position command remained exactly `0.075290442 rad` at every level.
  The first positive displacement milestones were:
  - `+0.100367 mrad` at level 14 / `0.350 Nm`;
  - `+0.200741 mrad` at level 17 / `0.425 Nm`;
  - `+0.303537 mrad` at level 18 / `0.450 Nm`, with velocity
    `0.00125570 rad/s`, measured torque `0.408812 Nm`, and estimated PD
    `-0.021352 Nm`.
- Level 18 was rejected before it could become the persistent target. The
  controller restored the preceding feedback-proven, wire-distinct level 17
  target (`0.425 Nm`). Its exact compressed target readback matched:
  `0xFF7FF76E / 0x59737FF7` expected and actual.
- During the bounded one-second frozen hold at level 17:
  - samples: `194`;
  - mean/minimum/maximum displacement:
    `0.308718 / 0.303537 / 0.312716 mrad`;
  - position standard deviation: `0.002358 mrad`;
  - peak velocity: `0.00022553 rad/s`;
  - mean measured/estimated-total torque: `0.390038 / 0.406454 Nm`;
  - final 250 ms: `51` samples, position span `0.002436 mrad`, peak velocity
    `0.00022553 rad/s`; terminal stability passed.
- Whole-run safety peaks remained inside the hard envelope: displacement
  `0.312716 mrad`, velocity `0.00264468 rad/s`, measured torque `0.557471 Nm`,
  estimated total torque `0.437540 Nm`, and driver/motor temperature
  `31.0/29.3 degC`. No position trajectory existed or was entered.

## Cleanup and post-run state

The registered cleanup target at completion was the bounded, feedback-proven
level-17 target, not a newly constructed position or torque target. The
installed binary's generic warning called this a “zero-additive diagnostic
baseline”; that wording was stale, while the stored target semantics and exact
readback above were correct. The source wording was corrected after this run
to say “registered feedback-verified diagnostic baseline”; no runtime behavior
was changed.

Cleanup restored that registered target through the shared sender, confirmed
J1 non-torque first, then confirmed nodes 2..6 non-torque, stopped the sender,
and disarmed all six heartbeat consumers. The process exited `0`.

Post-run can2 remained the same ifindex/USB epoch, ERROR-ACTIVE with TEC/REC
`0/0`; all CAN xstats and netdev error/drop counters remained zero. TX rose
from `0` to `4060` packets and then stopped. can2 had no residual socket or
controller process, and the kernel journal had no new `gs_usb`, `EPROTO`,
disconnect, reset, or re-enumeration event.

## Decision

This run proves that a fixed J1 position target and positive feed-forward were
consumed, that the first small positive response is path-dependent in the
`0.425..0.450 Nm` level interval for this particular start, and that the
censor/readback/cleanup chain works. It does **not** identify Coulomb friction,
zero position, torque scale, a reusable feed-forward bias, negative-direction
behavior, or position tracking.

Do not write `0.425 Nm` into the profile, replay this upsweep, enlarge the cap,
or use this result to pass the failed +5 mrad position survey. The separately
authorized negative-direction identification was subsequently completed and
is recorded in `2026-08-20-can2-j1-negative-censored-torque.md`. J1 remains
uncalibrated; the two directional brackets may only bound a later guarded,
phase-shaped position experiment. Real MoveIt execution remains locked.
