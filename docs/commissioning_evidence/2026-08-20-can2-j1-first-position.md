# J1 fixed first-position survey — 2026-08-20

Status: completed with an intentional acceptance failure. The drive consumed
both changed and returned targets exactly, but J1 did not move far enough to
pass the tracking gate. Cleanup completed normally.

## Artifacts and command

- Run start: `2026-08-20 15:13:56 +08:00`.
- Git HEAD: `2800acc9a36d6495b3bdbf8e8eecd5abb3f36993`, with an intentionally
  dirty commissioning worktree. The installed binary hash below is the
  authoritative executable identity.
- Raw transcript:
  `log/commissioning/2026-08-20T1514+0800_can2_j1_first_position.log`
  (gitignored local artifact, `14173` bytes).
- Raw transcript SHA-256:
  `c1edf7b13820bca1d38c80e924345d2097ff597b16c0aad94238cebe6215d2b1`.
- Installed controller SHA-256:
  `ca7e1d6e8f79ec72e51692fd872e805a34591ec8d032b99d774e0729507e0576`.
- Run-profile SHA-256:
  `1e11a28f7f5706d6c6b5d229674b8d98565dc7d56ce03cd0d09f2b6465e9d49c`.
- Controller source SHA-256 values:
  - `backend.rs`: `2343894568d9a50fd187b80c5948d51ad1ce9cec5b438f3725fa29cb77ea6e0e`;
  - `commissioning.rs`: `0ba940f0b5bb5327689ac769b84059043876fe620d60c4879e0b87b9730a562f`;
  - `main.rs`: `71189db7442553bca70207553e79828a5e48bc14d4bbf5455a88b956dfe147e4`.
- Adapter: HexMeow Quad CAN-FD, serial
  `C9E29601798421B29AC2D419C12D9502`, `can2`, channel `2`, ifindex `35`,
  USB path `3-2.2:1.0`.
- Fixed J1 policy: node `1`, direction `-1`, zero-offset candidate
  `-0.025651 rad`, torque scale `0.85`, joint-side Kp/Kd `60.0/2.5`,
  `+0.005 rad / 4.0 s` complete round trip, and temporary drive-side
  torque/PD permissions `30/20` permille.

```bash
ros2 run hex_arm_controller hex_arm_controller -- \
  --profile /workspaces/hex_arm_ros2/config/hardware/firefly_y6.discovered.local.yaml \
  --diagnose-axis joint_1 \
  --diagnostic-mode joint1-first-position \
  --allow-diagnostic-motion \
  --acknowledge-joint1-first-position
```

The J1 mode used the ordinary strict-zero SocketCAN preflight. It rejected all
J2 high-tier and historical-xstats acknowledgements. Immediately before the
run, can2 was ERROR-ACTIVE with TEC/REC `0/0`, every CAN xstat and netdev
error/drop counter zero, the exact adapter identity above, and no can2 socket
owner. An unrelated controller remained on can1 and was not modified.

## Observed sequence

- All six arm nodes initialized disabled; node 15 remained auxiliary-only.
- J1's temporary `30/20` drive permissions were written and read back while
  disabled. J1 then reached strict compressed-MIT Operation Enabled with
  status `0x0237`, mode display `5`, and control-word readback `0x000F`.
- The fixed-position enable gate passed after `0.254401 s`:
  - initial commanded position `-0.022825157 rad`;
  - measured position `-0.022943689 rad`;
  - displacement `-0.000118531 rad`;
  - velocity approximately zero;
  - measured torque `-0.037165 Nm`;
  - driver/motor temperatures `30.7/29.2 degC`.
- The first quantized changed position target matched exact
  `0x2004:02/03` readback: lower/upper
  `0xFF7FF7FF / 0x8BAE7FF7`.
- At the commanded trajectory peak:
  - commanded position `-0.017825235 rad` (`+0.005 rad` from the start);
  - measured position `-0.022770293 rad`;
  - sampled displacement only `+0.000054864 rad`;
  - measured velocity approximately zero;
  - measured torque `0.260153 Nm`;
  - estimated joint-side PD/total torque `0.296703 Nm`;
  - driver/motor temperatures `30.9/29.2 degC`.
- The largest positive displacement seen by the acceptance accumulator was
  `0.000064 rad`, far below the required `0.002500 rad`. No position, speed,
  temperature, force, state, feedback, or transport hard gate fired.
- The persistent start target was republished and matched exact returned
  `0x2004:02/03` readback: lower/upper
  `0xFF7FF7FF / 0x8BAF7FF7`.

## Cleanup and post-run checks

- The controller restored the persistent fixed-position baseline, then the
  selected-first shutdown path disabled J1 before handling the other axes.
  The transcript confirms all six drives disabled and all six `0x1016`
  heartbeat consumers disarmed.
- The process exited `1` only because the measured positive peak was below the
  50% motion-acceptance threshold. There was no cleanup error.
- Post-run can2 remained the same ifindex and USB path, ERROR-ACTIVE with
  TEC/REC `0/0`; all CAN xstats and netdev error/drop counters remained zero.
  TX rose from `12307` to `17995` packets and then stopped.
- There was no new `gs_usb`, `-EPROTO`, disconnect, reset, or re-enumeration
  entry, no remaining can2 socket, and no controller process.

## Decision

This run validates J1/node-1 identity, direction-command plumbing, the
compressed target consumer, fixed low drive permissions, bounded enable,
temperature/force/state guards, returned-target readback, and confirmed
cleanup. It does **not** validate J1 position tracking, zero offset, limits, or
both motion signs.

The result is consistent with static friction exceeding the roughly
`0.30 Nm` joint-side PD effort produced by a 5 mrad error at Kp `60`. Do not
replay the same position survey, enlarge its distance, or promote gains from
this one result. The next experiment must keep position fixed and use a
separately authorized, feedback-censored positive torque ramp to locate first
motion before another position survey is considered. The hardware profile
remains `calibrated: false`, and real MoveIt execution remains locked.
