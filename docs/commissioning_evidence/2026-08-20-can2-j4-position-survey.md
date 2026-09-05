# J4 bounded position and assistance survey — 2026-08-20

Status: **FAIL CLOSED for position qualification; command, direction, return,
and cleanup paths verified.** J4 is frozen at the existing gravity-off `40/2`
mapping candidate. This evidence does not authorize normal assistance,
MoveIt execution, or a larger-force retry.

## Bound configuration

- Interface: `can2`, channel 2, gs_usb serial
  `C9E29601798421B29AC2D419C12D9502`, ifindex 47, USB device 14.
- Final run-profile SHA-256:
  `07512ffffed18fb2795a2f749edad2016ebf3960a9b211ddaf6026e0da3dc0fc`.
- Final installed binary SHA-256:
  `9084665a91185d2717af595956f03dfde824b0b6aa20ced55e5da92f92986017`.
- Git HEAD: `2800acc9a36d6495b3bdbf8e8eecd5abb3f36993` with a dirty working tree.
- Fixed command in each position survey: joint-side `-0.005 rad`, complete
  out-and-back duration `4.0 s`; unchanged acceptance peak `-0.002500 rad`.
- Persistent profile remained `gravity_compensation_scale=0.0`, `Kp/Kd=40/2`,
  `calibrated=false`; temporary gains and assistance were diagnostic-only.

## Bounded runs

1. Gravity-off enable at `30/1`, temporary trajectory `60/2`: exact target
   consumption and clean return/disable, but only about `-1.016 mrad` at
   `-0.234 Nm`. Raw transcript SHA-256:
   `d8f86445a9ff3ba9c0712d2dd967fda78e7e009c2f0fb9e3fee0546050050233`.
2. Gravity-off enable at profile `40/2`, temporary trajectory `80/4`: peak
   about `-1.122 mrad`, measured/estimated torque about
   `-0.320/-0.307 Nm`; clean fail-closed return. Raw transcript SHA-256:
   `61b5c3c9439d7fdde82e388754b8f6d11c042f278233853991078fe4b6885ecf`.
3. A fixed-position negative-torque staircase sent `-0.025 Nm`, then
   `-0.050 Nm`; the second level produced about `-0.125 mrad` and the next
   fresh sample crossed the unchanged `0.005 rad/s` identification guard.
   No position trajectory was constructed. Raw transcript SHA-256:
   `c1691b3bfba29abfc08f153a2e00aa54a564f1699d5349c7b8aadcc82ed73464`.
4. Temporary `80/4` with phase-shaped assistance peaking at `-0.15 Nm`
   completed the entire trajectory, return and both exact readbacks, but
   reached only about `-1.544 mrad`. Raw transcript SHA-256:
   `62b808f346a7ab427a11c676049af6608eb16aeb41893cd85ce5d6d45959f409`.
5. The final reviewed envelope increased only the phase-shaped peak to
   `-0.30 Nm`, with drive caps configured/read back as `90/50 permille`.
   It completed the entire trajectory and return but reached only
   `-2.081 mrad`. Raw transcript SHA-256:
   `9cee0941624ce75e1a716f0dc2c456930522a5770a58e3c3baf8298ac30fd39f`.

## Final-run telemetry

- Start position: `-0.006643753 rad`.
- Quarter: displacement `-0.699954 mrad`, velocity `-0.002328 rad/s`,
  measured torque `-0.268398 Nm`.
- Midpoint sample: displacement `-2.032637 mrad`, essentially zero velocity,
  assistance `-0.299997 Nm`, PD estimate `-0.237385 Nm`, measured/estimated
  total torque `-0.545454/-0.537382 Nm`.
- Three-quarter: displacement `-1.581918 mrad`, return velocity
  `+0.001515 rad/s`.
- First changed target and returned persistent baseline both matched exact
  `0x2004:02/03` readback.
- Peak driver/motor temperatures stayed about `35.3/32.6 C`.
- The persistent baseline was restored; all six drives were confirmed
  disabled and all six heartbeat consumers were disarmed. Exit status was 1
  solely because the measured peak did not reach the acceptance threshold.
- Post-run can2 remained ERROR-ACTIVE with TEC/REC=0, every CAN xstat and
  netdev error/drop counter at zero, no CAN socket owner, and no new USB/gs_usb
  kernel event.

## Decision

The repeated evidence verifies J4 command direction, compressed-target
consumption, bounded negative motion, positive return, thermal envelope and
fail-closed cleanup. It does **not** verify the required tracking accuracy.
Do not copy either diagnostic assistance value into normal gravity or payload
feed-forward, and do not raise force again in this commissioning round. Keep
`40/2` only as the next mapping candidate, keep gravity compensation at zero,
and require a separate mechanical friction/load investigation before another
J4 position-qualification design.
