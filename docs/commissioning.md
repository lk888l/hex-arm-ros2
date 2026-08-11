# Supervised hardware commissioning

**English** | [中文](commissioning_cn.md)

Do not run real bringup until the arm is mechanically secured, its workspace
is clear, and the independent physical emergency stop has been tested. WSL2
and Docker are not safety systems.

1. Attach the `1209:2323` adapter to WSL with `usbipd`, start the real Compose
   override, and verify the container can list the USB device. Do not map the
   full host or use `privileged`.
2. Keep motor power disabled. Start the Rust controller in a read-only
   discovery workflow and record all six CANopen node IDs plus vendor, product,
   revision, and serial identity fingerprints. Unexpected or duplicate nodes
   stop the procedure.
3. Create a `*.local.yaml` profile. Enter one joint/node mapping at a time.
   The old sign vector and torque factors are hints only, never evidence.
4. With the physical e-stop in reach, use the lowest practical torque limit to
   verify each axis independently. Confirm positive ROS motion, then record
   `direction` and restore the joint to a safe pose before moving on.
5. Record motor zero and calculate `zero_offset_rad` from
   `q_ros = direction * 2*pi*q_motor_rev + zero_offset_rad`. Treat joint 3 as a
   known special case; do not edit the URDF origin to hide an actuator zero.
6. Verify both software limits at low speed without approaching a mechanical
   hard stop. Record torque scaling from a controlled calibration, not from the
   historical GUI default.
7. Mark `calibrated: true` only after all six axes pass direction, zero, limit,
   and identity checks. Mark `validated: true` only after a second-person
   review of the complete file.
8. Run: disabled discovery, single-axis low-speed tracking, six-axis hold, then
   one short trajectory. Abort on temperature rise, stale diagnostics, noise,
   unexpected direction, or any latched fault.

`clear_fault` never auto-enables the arm. After every fault, inspect and remove
the physical cause before clearing; activation remains an explicit
ros2_control lifecycle operation.
