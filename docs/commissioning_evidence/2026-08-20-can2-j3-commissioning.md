# J3 bounded gravity and position survey — 2026-08-20

Status: **FAIL CLOSED for position qualification; direction, bounded command,
return, and cleanup paths verified.** J3 is frozen at the conservative
gravity-off `2/0.3` profile. This evidence does not authorize normal payload
feed-forward, a larger assistance retry, or MoveIt execution.

## Bound configuration

- Interface: `can2`, channel 2, gs_usb serial
  `C9E29601798421B29AC2D419C12D9502`, final-run ifindex 51.
- Final diagnostic run-profile SHA-256:
  `813265013ffc37e3b2f5afc564b40daaec542a504fc25bc5f6c2420b14a4dad2`.
- Restored conservative local-profile SHA-256 after recording this run:
  `b8d7c04ad91da8da522740883bf6012ccbf954a4d8dbd9a7c4d356859a234505`.
- Final installed binary SHA-256:
  `a5477d8d643d2c92f2281e27c8db7d2e88b18763e58e0bb5fb0496fa2a92e4`.
- Git HEAD: `2800acc9a36d6495b3bdbf8e8eecd5abb3f36993` with a dirty working tree.
- Final diagnostic source SHA-256 values: `commissioning.rs`
  `7b2bf73f898a7ebd70156f0c2f41292f2a158df5974c8cb0ea35a36fba143ca5`,
  `main.rs`
  `5cdad21cf5b346e3da786bf479a3793c9e118516e4c2d424e81f0a3241f82513`,
  and `backend.rs`
  `4c8ee765401cf983ed53f244e6199c42e047ce0eee8dff5e8bd36f16ddf6be72`.
- Final fixed command: joint-side `-0.005 rad`, complete out-and-back duration
  `4.0 s`, temporary true joint-side `Kp/Kd=80/4`, phase-shaped assistance
  `0 -> -0.25 -> 0 Nm`, and unchanged acceptance peak `-0.002500 rad`.
- The persistent profile remains `gravity_compensation_scale=0.0`,
  `Kp/Kd=2/0.3`, `calibrated=false`; the diagnostic gains, caps, narrow window,
  and assistance were one-run-only values.

## Bounded runs

1. The low-tier fixed-position gravity-unload staircase covered
   `-0.025..-0.250 Nm`. Every level remained stable; the fixed `-0.25 Nm`
   window had about `-0.094 mrad` mean displacement and driver/motor
   temperatures near `33.5/29.7 C`. Raw transcript: 24,653 bytes, SHA-256
   `e01cbacb3ded2fe74e19a627d36ed2a671411f30eeaaca10c37e0ca718b8c4fe`.
2. The bounded mid-tier identification rejected the `-0.30 Nm` candidate when
   velocity reached about `-0.005012 rad/s` at `-0.121 mrad`; it restored the
   previously verified `-0.25 Nm` target and cleaned up. Raw transcript:
   20,812 bytes, SHA-256
   `2e5035f1f57b73cba556c3463ce535844dc521192275baef66510241cef55a82`.
3. A gravity-off `-0.005 rad / 4 s` position survey with temporary `80/4`
   consumed the target and returned cleanly, but reached only about
   `-0.552 mrad` versus the fixed `-2.500 mrad` acceptance threshold. Raw
   transcript: 13,630 bytes, SHA-256
   `a8b3926d7fe8711d912ffeaba2707da90916439d83581685107a2ae45d4473a6`.
4. The final trajectory-synchronous assistance run used the same temporary
   `80/4` gains and only a `-0.25 Nm` phase-shaped peak. It completed the
   round trip but reached only `-0.684 mrad`. Raw transcript: 14,179 bytes,
   SHA-256
   `f89a3375e33f184ca10c13ab785bc46f2c536599f9e4f77432dab4cfa9c27166`.

## Final-run telemetry

- J3 strict MIT operation was confirmed with status word `0x0237`, mode
  display 5, and control-word readback `0x000F`. The temporary `30/20`
  permille drive caps were configured and read back while disabled.
- Post-enable stability passed for `0.252411 s` with zero recorded position and
  velocity excursion before the trajectory.
- Start position: `3.138548136 rad`; initial measured torque `+0.074329 Nm`.
- Quarter: displacement `-0.127554 mrad`, velocity `-0.000281 rad/s`,
  assistance `-0.125973 Nm`, measured/estimated total torque
  `-0.222988/-0.316177 Nm`.
- Midpoint: displacement `-0.673532 mrad`, velocity `-0.000201 rad/s`,
  assistance `-0.250000 Nm`, PD estimate `-0.345332 Nm`, and
  measured/estimated total torque `-0.631800/-0.595332 Nm`.
- Three-quarter: displacement `-0.621080 mrad`, return velocity
  `+0.000562 rad/s`, and assistance `-0.124035 Nm`.
- Return settle: residual displacement `-0.353575 mrad`, effectively zero
  velocity, assistance zero, and measured torque `-0.037165 Nm`.
- The persistent baseline was restored. All six drives were confirmed disabled
  and all six heartbeat consumers were disarmed. Exit status was nonzero only
  because the signed peak `-0.684 mrad` did not reach `-2.500 mrad`.
- Post-run can2 remained ERROR-ACTIVE with TEC/REC=0, all CAN xstats and
  netdev error/drop counters at zero, no can2 socket owner, and no new
  USB/gs_usb kernel event.

## Decision

The runs verify J3's negative command direction, bounded RPDO command path,
bounded assistance response, return behavior, and fail-closed cleanup. They do
**not** qualify position tracking, gravity compensation, payload dynamics, or
mechanical travel. Do not promote `80/4`, `-0.25 Nm`, or either diagnostic
torque-cap setting into normal control. Keep J3 at gravity scale zero and the
conservative `2/0.3` profile, and require a separate mechanical/load/friction
investigation before another J3 force increase. MoveIt real execution remains
locked.
