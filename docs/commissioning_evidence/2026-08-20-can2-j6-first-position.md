# J6 bounded first-position survey — 2026-08-20

Status: **PASS for one bounded J6 negative 5 mrad round trip; candidate gains only.**
This does not set the hardware profile `calibrated` flag and does not authorize
MoveIt execution or a general jog interface.

## Bound configuration

- Interface: `can2`, channel 2, gs_usb serial
  `C9E29601798421B29AC2D419C12D9502`, ifindex 47, USB device 14.
- Successful run-profile SHA-256:
  `da2b615dfc4bcec7a0f72f90aab912b7e85477ec92dfc0a9b2e378fc0fb78275`.
- Successful installed binary SHA-256:
  `119fb6eb1b6448e10130dea3f0b9e0134de54729216c7022e1b7dfcf22500451`.
- Fixed command: joint-side `-0.005 rad`, complete out-and-back duration `4.0 s`.
- Drive caps were configured and read back while disabled as `50/30 permille`
  (`0x6072:00` / `0x2004:0E`).
- Successful enable hold used `Kp/Kd=25/1`; its trajectory used the mapping
  boundary `Kp/Kd=50/2`. Both runs used strict-zero SocketCAN preflight.

## Fail-closed discovery runs

The first binary tried temporary `50/2` against a `20/1` profile mapping. It
passed enable stability but rejected the trajectory before publishing a
changed position because Kp 50 exceeded the configured Kp 40 mapping limit.
Raw transcript SHA-256:
`959861ab783c43237e330a6cacc7c2a4bbfc04d26ffa6470f38f06107f97a884`.

After moving that proof into request validation, the fixed mapping-boundary
`40/2` run completed both exact readbacks and returned cleanly, but its peak
`-0.002349 rad` did not reach the unchanged `-0.002500 rad` acceptance gate.
Raw transcript SHA-256:
`7a190a5959a536a881113cc7f5e72b445aa4d0e2ba2af38abc6d3144c1a8e7ac`.

## Accepted 50/2 run

Raw transcript SHA-256:
`f233660519fddddd84905abc2fb65d0f8d6ad89fb08d6051a897ca4ba3a94f02`.

- Initial logical position: `+0.130398363 rad`.
- First changed target and returned persistent baseline both matched exact
  `0x2004:02/03` readback.
- Most-negative logical displacement: `-0.002760857 rad`.
- Most-negative raw motor displacement: `-0.000439405 rev`.
- Continuous return error: `0.000364020 rad`; return velocity was zero.
- Peak speed: `0.006927010 rad/s`.
- Peak measured / estimated total torque: `0.129870 / 0.118923 Nm`.
- Peak driver / motor temperature: `34.6 / 31.9 C`.
- Six-axis isolation held; process exit was 0 and all six heartbeat consumers
  were disarmed after confirmed non-torque states.
- Post-run can2 remained ERROR-ACTIVE with TEC/REC=0, all CAN xstats and netdev
  error/drop counters at zero, and no new USB/gs_usb fault log.

## Decision

`50/2` is promoted only as the next J6 profile candidate. The run proves one
small negative excursion and positive return around this start pose; it does
not prove full travel, absolute zero, backlash/repeatability, thermal
endurance, payload calibration, or whole-arm trajectory execution. The
one-shot J6 diagnostic exact-locks the pre-promotion profile and therefore
fails closed if replayed with this promoted profile.
