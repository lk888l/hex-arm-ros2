//! Deliberately narrow, non-ROS single-axis commissioning motion.
//!
//! This module is intentionally not a general jog interface. It permits one
//! small, smooth out-and-back motion after the real backend has verified the
//! bus, all identities, the auxiliary-node allowlist, and the disabled state.

use std::array;
use std::f32::consts::{PI, TAU};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};

use crate::backend::{
    FeedbackSnapshot, MotorBackend, RealBackend, DOF, J1_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
    J1_FIRST_POSITION_TORQUE_PERMILLE, J3_GRAVITY_UNLOAD_KP_KD_TORQUE_PERMILLE,
    J3_GRAVITY_UNLOAD_TORQUE_PERMILLE, J4_ASSISTED_POSITION_KP_KD_TORQUE_PERMILLE,
    J4_ASSISTED_POSITION_TORQUE_PERMILLE, J4_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
    J4_FIRST_POSITION_TORQUE_PERMILLE, J5_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
    J5_FIRST_POSITION_TORQUE_PERMILLE, J6_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
    J6_FIRST_POSITION_TORQUE_PERMILLE,
};
use crate::conversion::{
    motor_kd_to_ros, motor_kp_to_ros, motor_position_to_ros, motor_torque_to_ros,
    motor_velocity_to_ros, ros_target_to_motor, MotorTarget, RosTarget,
};
use crate::profile::HardwareProfile;

pub const MAX_COMMISSION_DELTA_RAD: f32 = 0.03;
pub const MAX_COMMISSION_DURATION_SEC: f32 = 30.0;
const MIN_COMMISSION_DELTA_RAD: f32 = 1.0e-4;
// Commissioning uses the gains explicitly reviewed in the hardware profile.
// Keep a final fail-closed ceiling instead of silently clamping them: with the
// CLI's small position/velocity bounds, measured-state guards, and the drive's
// independent kp/kd torque permille remain active at these ceilings.
// The fail-closed ceilings include the Kp=72, Kd=2.75 operating point already
// exercised on the isomorphic CAN1 arm. They remain independent of both the
// reviewed per-joint profile values and the drive-side kp/kd torque cap.
const COMMISSION_KP_HARD_MAX_NM_RAD: f32 = 80.0;
const COMMISSION_KD_HARD_MAX_NM_S_RAD: f32 = 4.0;
const FEEDBACK_WAIT_SEC: f32 = 3.0;
const LOOP_PERIOD: Duration = Duration::from_millis(4);
const STABILITY_DWELL: Duration = Duration::from_millis(250);
const ENABLE_STABILITY_TIMEOUT: Duration = Duration::from_secs(3);
const RETURN_SETTLE_TIMEOUT: Duration = Duration::from_secs(3);
const STABLE_VELOCITY_RAD_S: f32 = 0.02;
const ENABLE_STABILITY_POSITION_RAD: f32 = 0.003;
const TRACKING_ERROR_MARGIN_RAD: f32 = 0.015;
const EXCURSION_MARGIN_RAD: f32 = 0.005;
const OPPOSITE_DIRECTION_MARGIN_RAD: f32 = 0.003;
const MINIMUM_REQUESTED_EXCURSION_FRACTION: f32 = 0.5;
const LIMIT_EPSILON: f32 = 1.0e-3;
const MOTION_TELEMETRY_MILESTONES: [(f32, &str); 3] =
    [(0.25, "quarter"), (0.50, "peak"), (0.75, "three_quarter")];
const CENSORED_GRAVITY_HOLD_DISPLACEMENT_MILESTONES: [(f32, &str); 3] = [
    (0.00010, "positive_0p1_mrad"),
    (0.00020, "positive_0p2_mrad"),
    (0.00030, "positive_0p3_mrad_censor"),
];
const PHASE_ENABLE_STABILITY: &str = "enable_stability";
const PHASE_TRAJECTORY: &str = "trajectory";
const PHASE_RETURN_SETTLE: &str = "return_settle";
const PHASE_DIAGNOSTIC_HOLD: &str = "diagnostic_hold";
const PHASE_DIAGNOSTIC_STAIRCASE: &str = "diagnostic_tau_ff_staircase";
const PHASE_DIAGNOSTIC_POSITION_GRAVITY_RAMP: &str = "diagnostic_position_gravity_feedforward_ramp";
const PHASE_DIAGNOSTIC_POSITION_ROUND_TRIP: &str = "diagnostic_position_round_trip";
const PHASE_DIAGNOSTIC_POSITION_RETURN: &str = "diagnostic_position_return_settle";
const PHASE_DIAGNOSTIC_CENSORED_GRAVITY_HOLD: &str = "diagnostic_censored_gravity_hold";
const PHASE_J1_FIRST_POSITION_ENABLE: &str = "joint_1_first_position_enable_stability";
const PHASE_J1_FIRST_POSITION_TRAJECTORY: &str = "joint_1_first_position_round_trip";
const PHASE_J1_FIRST_POSITION_RETURN: &str = "joint_1_first_position_return";
const PHASE_J1_CENSORED_TORQUE: &str = "joint_1_fixed_position_censored_torque";
const PHASE_J3_GRAVITY_UNLOAD: &str = "joint_3_fixed_position_gravity_unload";
const PHASE_J3_ASSISTED_POSITION: &str = "joint_3_assisted_position_round_trip";
const PHASE_J4_FIRST_POSITION_ENABLE: &str = "joint_4_first_position_enable_stability";
const PHASE_J4_FIRST_POSITION_TRAJECTORY: &str = "joint_4_first_position_round_trip";
const PHASE_J4_FIRST_POSITION_RETURN: &str = "joint_4_first_position_return";
const PHASE_J4_CENSORED_TORQUE: &str = "joint_4_fixed_position_censored_torque";
const PHASE_J4_ASSISTED_POSITION_TRAJECTORY: &str = "joint_4_assisted_position_round_trip";
const PHASE_J5_FIRST_POSITION_ENABLE: &str = "joint_5_first_position_enable_stability";
const PHASE_J5_FIRST_POSITION_TRAJECTORY: &str = "joint_5_first_position_round_trip";
const PHASE_J5_FIRST_POSITION_RETURN: &str = "joint_5_first_position_return";
const PHASE_J6_FIRST_POSITION_ENABLE: &str = "joint_6_first_position_enable_stability";
const PHASE_J6_FIRST_POSITION_TRAJECTORY: &str = "joint_6_first_position_round_trip";
const PHASE_J6_FIRST_POSITION_RETURN: &str = "joint_6_first_position_return";
const DIAGNOSTIC_JOINT_INDEX: usize = 1;
const MIN_DIAGNOSTIC_HOLD_SEC: f32 = 0.25;
const MAX_DIAGNOSTIC_HOLD_SEC: f32 = 2.0;
const MIN_DIAGNOSTIC_DWELL_SEC: f32 = 0.15;
const MAX_DIAGNOSTIC_DWELL_SEC: f32 = 0.50;
const HIGH_TIER_MAX_DIAGNOSTIC_DWELL_SEC: f32 = 0.25;
const MIN_DIAGNOSTIC_TORQUE_STEP_NM: f32 = 0.025;
const LOW_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM: f32 = 0.25;
const HIGH_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM: f32 = 1.75;
const MAX_DIAGNOSTIC_TORQUE_STEP_NM: f32 = 0.25;
const MAX_DIAGNOSTIC_STEPS: usize = 50;
const MAX_DIAGNOSTIC_ACTIVE_SEC: f32 = 8.0;
const HIGH_TIER_MAX_DIAGNOSTIC_ACTIVE_SEC: f32 = 4.0;
const LOW_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD: f32 = 0.001;
const HIGH_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD: f32 = 0.002;
const MAX_DIAGNOSTIC_EXCURSION_RAD: f32 = 0.002;
const MAX_DIAGNOSTIC_VELOCITY_RAD_S: f32 = 0.02;
const DIAGNOSTIC_BREAKAWAY_POSITION_RAD: f32 = 0.001;
const DIAGNOSTIC_BREAKAWAY_VELOCITY_RAD_S: f32 = 0.01;
const MAX_DIAGNOSTIC_OPPOSITE_POSITION_RAD: f32 = 0.0005;
const LOW_TIER_MAX_DIAGNOSTIC_TOTAL_TORQUE_NM: f32 = 1.0;
const HIGH_TIER_MAX_DIAGNOSTIC_TOTAL_TORQUE_NM: f32 = 2.5;
const POSITION_DIAGNOSTIC_DELTA_RAD: f32 = 0.005;
const POSITION_DIAGNOSTIC_DURATION_SEC: f32 = 4.0;
const POSITION_DIAGNOSTIC_MAX_ACTIVE_SEC: f32 = 8.0;
const POSITION_DIAGNOSTIC_MAX_TOTAL_TORQUE_NM: f32 = 2.0;
const POSITION_DIAGNOSTIC_MAX_FEEDFORWARD_NM: f32 = 1.35;
const POSITION_DIAGNOSTIC_STABILITY_POSITION_RAD: f32 = 0.0005;
const POSITION_DIAGNOSTIC_STABILITY_VELOCITY_RAD_S: f32 = 0.01;
const POSITION_DIAGNOSTIC_MAX_OPPOSITE_POSITION_RAD: f32 = 0.0005;
const POSITION_DIAGNOSTIC_MAX_POSITIVE_MARGIN_RAD: f32 = 0.001;
const POSITION_DIAGNOSTIC_ABSOLUTE_UPPER_OFFSET_RAD: f32 = 0.008;
const POSITION_DIAGNOSTIC_RETURN_POSITION_RAD: f32 = 0.00125;
const POSITION_DIAGNOSTIC_REQUIRED_PEAK_FRACTION: f32 = 0.5;
const POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE: f32 = 0.25;
const POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE: f32 = 0.65;
// The position survey may reach the reviewed 0.65 gravity scale only after an
// already exercised 0.25-scale enable and stability dwell.  These 375 fixed
// 4 ms transitions form a 1.5 s half-cosine ramp. Scheduler delay can only
// slow the ramp: advancement additionally requires strictly newer feedback.
const POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS: usize = 375;
const POSITION_DIAGNOSTIC_GRAVITY_RAMP_DURATION_SEC: f32 = 1.5;
// The shared sender runs at 1 kHz. A TPDO used to promote a target into the
// persistent baseline must be timestamped after two complete sender periods,
// in addition to the 4 ms minimum between successive target updates.
const POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY: Duration = Duration::from_millis(2);
const POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_SCALE_STEP: f32 = 0.0017;
const POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_POSITIVE_RAD: f32 = 0.002;
const POSITION_DIAGNOSTIC_GRAVITY_RAMP_ABSOLUTE_UPPER_OFFSET_RAD: f32 = 0.004;
const POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_POSITION_RAD: f32 = 0.0015;
const POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_VELOCITY_RAD_S: f32 = 0.015;
const POSITION_DIAGNOSTIC_EXPECTED_KP: f32 = 80.0;
const POSITION_DIAGNOSTIC_EXPECTED_KD: f32 = 2.5;
const POSITION_DIAGNOSTIC_EXPECTED_TORQUE_PERMILLE: u16 = 200;
const POSITION_DIAGNOSTIC_EXPECTED_KP_KD_TORQUE_PERMILLE: u16 = 100;
// A fixed-q, hold-only identification mode. It deliberately stops increasing
// feed-forward before the already observed displacement becomes a position
// command: the currently published target is censored at +0.30 mrad and is
// never promoted into the persistent cleanup baseline. The immediately prior
// feedback-verified microstep is held for at most one second of identification
// and exact readback. There is no position-trajectory branch in this mode.
const CENSORED_GRAVITY_HOLD_PROFILE_SCALE: f32 = 0.25;
const CENSORED_GRAVITY_HOLD_CAP_SCALE: f32 = 0.55;
const CENSORED_GRAVITY_HOLD_STEPS: usize = 375;
const CENSORED_GRAVITY_HOLD_NOMINAL_DURATION_SEC: f32 = 1.5;
const CENSORED_GRAVITY_HOLD_MAX_SCALE_STEP: f32 = 0.0013;
const CENSORED_GRAVITY_HOLD_TRIGGER_RAD: f32 = 0.00030;
const CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD: f32 = 0.00050;
const CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S: f32 = 0.005;
const CENSORED_GRAVITY_HOLD_FREEZE_DURATION: Duration = Duration::from_secs(1);
const CENSORED_GRAVITY_HOLD_TERMINAL_DWELL: Duration = Duration::from_millis(250);
const CENSORED_GRAVITY_HOLD_TERMINAL_VELOCITY_RAD_S: f32 = 0.002;
const CENSORED_GRAVITY_HOLD_TERMINAL_POSITION_SPAN_RAD: f32 = 0.00010;
const CENSORED_GRAVITY_HOLD_MIN_STATISTICS_SAMPLES: usize = 100;
const CENSORED_GRAVITY_HOLD_MIN_TERMINAL_SAMPLES: usize = 40;
const CENSORED_GRAVITY_HOLD_MAX_ACTIVE_SEC: f32 = 4.0;
const J1_FIRST_POSITION_INDEX: usize = 0;
const J1_FIRST_POSITION_DELTA_RAD: f32 = 0.005;
const J1_FIRST_POSITION_DURATION_SEC: f32 = 4.0;
const J1_FIRST_POSITION_EXPECTED_KP: f32 = 60.0;
const J1_FIRST_POSITION_EXPECTED_KD: f32 = 2.5;
// The original 60 Nm/rad position survey was consumed by the drive but could
// produce only 0.30 Nm at the 5 mrad endpoint and did not move measurably. The
// later fixed-position surveys bounded the first small response near
// +0.425..+0.450 Nm and -0.175..-0.200 Nm. A follow-up trajectory therefore
// uses the already reviewed commissioning ceiling of 80 Nm/rad plus a smooth,
// direction-dependent feed-forward envelope. The envelope is exactly zero at
// start, turn-around, and return; neither frozen survey torque becomes a bias.
const J1_FIRST_POSITION_TRAJECTORY_KP: f32 = 80.0;
const J1_FIRST_POSITION_TRAJECTORY_KD: f32 = 4.0;
const J1_FIRST_POSITION_POSITIVE_COMPENSATION_NM: f32 = 0.30;
const J1_FIRST_POSITION_NEGATIVE_COMPENSATION_NM: f32 = 0.175;
const J1_FIRST_POSITION_MAX_COMPENSATED_FEEDFORWARD_NM: f32 = 0.41;
const J1_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD: f32 = -0.025_651;
const J1_FIRST_POSITION_EXPECTED_TORQUE_SCALE: f32 = 0.85;
const J1_FIRST_POSITION_EXPECTED_GRAVITY_SCALE: f32 = 1.0;
const J1_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE: u16 = 200;
const J1_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE: u16 = 100;
const J1_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD: f32 = -0.25;
const J1_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD: f32 = 0.25;
const J1_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S: f32 = 0.1;
const J1_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2: f32 = 0.1;
const J1_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM: f32 = 6.0;
const J1_FIRST_POSITION_INITIAL_Q_LOWER_RAD: f32 = -0.24;
const J1_FIRST_POSITION_INITIAL_Q_UPPER_RAD: f32 = 0.24;
const J1_FIRST_POSITION_MAX_GRAVITY_NM: f32 = 0.10;
const J1_FIRST_POSITION_MAX_TOTAL_TORQUE_NM: f32 = 1.0;
const J1_FIRST_POSITION_MAX_ACTIVE_SEC: f32 = 8.0;
const J1_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD: f32 = -0.0005;
const J1_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD: f32 = 0.006;
const J1_FIRST_POSITION_ENABLE_HARD_POSITION_RAD: f32 = 0.001;
const J1_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S: f32 = 0.02;
const J1_FIRST_POSITION_GO_POSITION_RAD: f32 = 0.0005;
const J1_FIRST_POSITION_GO_VELOCITY_RAD_S: f32 = 0.01;
const J1_FIRST_POSITION_REQUIRED_PEAK_RAD: f32 = 0.0025;
const J1_FIRST_POSITION_RETURN_POSITION_RAD: f32 = 0.00125;
const J1_FIRST_POSITION_RETURN_VELOCITY_RAD_S: f32 = 0.01;
const J1_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV: f32 = 0.000_397_887;
// Follow-up identification after the fixed +5 mrad survey proved that the
// position target reached the drive but 0.30 Nm of P torque did not overcome
// static friction. This mode never changes the position code: it raises only
// positive joint-side feed-forward in fixed 25 mNm / 100 ms levels, rejects
// the first target observed at +0.30 mrad, and freezes the preceding target
// only if that target was proven by newer all-axis TPDO1 feedback.
const J1_CENSORED_TORQUE_STEP_NM: f32 = 0.025;
const J1_CENSORED_TORQUE_CAP_NM: f32 = 0.60;
const J1_CENSORED_TORQUE_DWELL: Duration = Duration::from_millis(100);
const J1_CENSORED_TORQUE_LEVELS: usize = 24;
const J1_CENSORED_TORQUE_TRIGGER_RAD: f32 = 0.00030;
const J1_CENSORED_TORQUE_HARD_POSITIVE_RAD: f32 = 0.00050;
const J1_CENSORED_TORQUE_HARD_NEGATIVE_RAD: f32 = -0.00030;
const J1_CENSORED_TORQUE_HARD_VELOCITY_RAD_S: f32 = 0.005;
const J1_CENSORED_TORQUE_MAX_FEEDFORWARD_NM: f32 = 0.70;
const J1_CENSORED_TORQUE_MAX_ACTIVE_SEC: f32 = 4.5;
const J1_CENSORED_TORQUE_FREEZE_DURATION: Duration = Duration::from_secs(1);
const J5_FIRST_POSITION_INDEX: usize = 4;
const J5_FIRST_POSITION_DELTA_RAD: f32 = -0.005;
const J5_FIRST_POSITION_DURATION_SEC: f32 = 4.0;
const J5_FIRST_POSITION_EXPECTED_KP: f32 = 30.0;
const J5_FIRST_POSITION_EXPECTED_KD: f32 = 1.0;
const J5_FIRST_POSITION_TRAJECTORY_KP: f32 = 50.0;
const J5_FIRST_POSITION_TRAJECTORY_KD: f32 = 2.0;
const J5_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD: f32 = 0.026_979;
const J5_FIRST_POSITION_EXPECTED_TORQUE_SCALE: f32 = 1.0;
const J5_FIRST_POSITION_EXPECTED_GRAVITY_SCALE: f32 = 1.0;
const J5_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE: u16 = 200;
const J5_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE: u16 = 100;
const J5_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD: f32 = -0.25;
const J5_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD: f32 = 0.25;
const J5_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S: f32 = 0.1;
const J5_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2: f32 = 0.1;
const J5_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM: f32 = 1.5;
const J5_FIRST_POSITION_INITIAL_Q_LOWER_RAD: f32 = -0.24;
const J5_FIRST_POSITION_INITIAL_Q_UPPER_RAD: f32 = 0.24;
const J5_FIRST_POSITION_MAX_FEEDFORWARD_NM: f32 = 0.10;
const J5_FIRST_POSITION_MAX_TOTAL_TORQUE_NM: f32 = 0.50;
const J5_FIRST_POSITION_MAX_ACTIVE_SEC: f32 = 8.0;
const J5_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD: f32 = -0.006;
const J5_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD: f32 = 0.0005;
const J5_FIRST_POSITION_ENABLE_HARD_POSITION_RAD: f32 = 0.001;
const J5_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S: f32 = 0.02;
const J5_FIRST_POSITION_GO_POSITION_RAD: f32 = 0.0005;
const J5_FIRST_POSITION_GO_VELOCITY_RAD_S: f32 = 0.01;
const J5_FIRST_POSITION_REQUIRED_PEAK_RAD: f32 = 0.0025;
const J5_FIRST_POSITION_RETURN_POSITION_RAD: f32 = 0.00125;
const J5_FIRST_POSITION_RETURN_VELOCITY_RAD_S: f32 = 0.01;
const J5_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV: f32 = 0.000_397_887;
const J4_FIRST_POSITION_INDEX: usize = 3;
const J4_FIRST_POSITION_DELTA_RAD: f32 = -0.005;
const J4_FIRST_POSITION_DURATION_SEC: f32 = 4.0;
const J4_FIRST_POSITION_EXPECTED_KP: f32 = 40.0;
const J4_FIRST_POSITION_EXPECTED_KD: f32 = 2.0;
const J4_FIRST_POSITION_TRAJECTORY_KP: f32 = 80.0;
const J4_FIRST_POSITION_TRAJECTORY_KD: f32 = 4.0;
const J4_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD: f32 = 0.000_144;
const J4_FIRST_POSITION_EXPECTED_TORQUE_SCALE: f32 = 1.0;
const J4_FIRST_POSITION_EXPECTED_GRAVITY_SCALE: f32 = 0.0;
const J4_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE: u16 = 200;
const J4_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE: u16 = 100;
const J4_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD: f32 = -0.25;
const J4_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD: f32 = 0.25;
const J4_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S: f32 = 0.1;
const J4_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2: f32 = 0.1;
const J4_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM: f32 = 1.5;
const J4_FIRST_POSITION_INITIAL_Q_LOWER_RAD: f32 = -0.24;
const J4_FIRST_POSITION_INITIAL_Q_UPPER_RAD: f32 = 0.24;
const J4_FIRST_POSITION_MAX_FEEDFORWARD_NM: f32 = 0.01;
const J4_FIRST_POSITION_MAX_TOTAL_TORQUE_NM: f32 = 0.50;
const J4_FIRST_POSITION_MAX_ACTIVE_SEC: f32 = 8.0;
const J4_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD: f32 = -0.006;
const J4_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD: f32 = 0.0005;
const J4_FIRST_POSITION_ENABLE_HARD_POSITION_RAD: f32 = 0.001;
const J4_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S: f32 = 0.02;
const J4_FIRST_POSITION_GO_POSITION_RAD: f32 = 0.0005;
const J4_FIRST_POSITION_GO_VELOCITY_RAD_S: f32 = 0.01;
const J4_FIRST_POSITION_REQUIRED_PEAK_RAD: f32 = 0.0025;
const J4_FIRST_POSITION_RETURN_POSITION_RAD: f32 = 0.00125;
const J4_FIRST_POSITION_RETURN_VELOCITY_RAD_S: f32 = 0.01;
const J4_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV: f32 = 0.000_397_887;
const J4_ASSISTED_POSITION_PEAK_TORQUE_NM: f32 = -0.30;
const J4_ASSISTED_POSITION_MAX_FEEDFORWARD_NM: f32 = 0.31;
const J4_ASSISTED_POSITION_MAX_TOTAL_TORQUE_NM: f32 = 0.75;
// Fixed-q negative-torque identification after both position-channel surveys
// proved exact target consumption but stalled below the acceptance threshold.
// Each level must complete a feedback-proven dwell before it can become the
// cleanup baseline. The first -0.30 mrad response rejects the current level
// and freezes the preceding wire-distinct target; no position trajectory is
// constructed by this mode.
const J4_CENSORED_TORQUE_STEP_NM: f32 = 0.025;
const J4_CENSORED_TORQUE_CAP_NM: f32 = 0.45;
const J4_CENSORED_TORQUE_DWELL: Duration = Duration::from_millis(100);
const J4_CENSORED_TORQUE_LEVELS: usize = 18;
const J4_CENSORED_TORQUE_TRIGGER_RAD: f32 = -0.00030;
const J4_CENSORED_TORQUE_HARD_NEGATIVE_RAD: f32 = -0.00050;
const J4_CENSORED_TORQUE_HARD_POSITIVE_RAD: f32 = 0.00030;
const J4_CENSORED_TORQUE_HARD_VELOCITY_RAD_S: f32 = 0.005;
const J4_CENSORED_TORQUE_MAX_FEEDFORWARD_NM: f32 = 0.45;
const J4_CENSORED_TORQUE_MAX_TOTAL_TORQUE_NM: f32 = 0.50;
const J4_CENSORED_TORQUE_MAX_ACTIVE_SEC: f32 = 4.5;
const J4_CENSORED_TORQUE_FREEZE_DURATION: Duration = Duration::from_secs(1);
const J3_GRAVITY_UNLOAD_INDEX: usize = 2;
const J3_GRAVITY_UNLOAD_EXPECTED_ZERO_OFFSET_RAD: f32 = 1.545_484;
const J3_GRAVITY_UNLOAD_EXPECTED_TORQUE_SCALE: f32 = 0.85;
const J3_GRAVITY_UNLOAD_EXPECTED_GRAVITY_SCALE: f32 = 0.0;
const J3_GRAVITY_UNLOAD_EXPECTED_KP: f32 = 2.0;
const J3_GRAVITY_UNLOAD_EXPECTED_KD: f32 = 0.3;
const J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_TORQUE_PERMILLE: u16 = 250;
const J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE: u16 = 100;
const J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_LOWER_RAD: f32 = 2.85;
#[allow(clippy::approx_constant)] // surveyed/profile limit is deliberately 3.14, not mathematical PI
const J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_UPPER_RAD: f32 = 3.14;
const J3_GRAVITY_UNLOAD_STEP_NM: f32 = 0.05;
const J3_GRAVITY_UNLOAD_CAP_NM: f32 = 0.75;
const J3_GRAVITY_UNLOAD_LEVELS: usize = 15;
const J3_GRAVITY_UNLOAD_DWELL: Duration = Duration::from_millis(100);
const J3_GRAVITY_UNLOAD_TRIGGER_RAD: f32 = -0.00030;
const J3_GRAVITY_UNLOAD_TRIGGER_VELOCITY_RAD_S: f32 = -0.003;
const J3_GRAVITY_UNLOAD_HARD_NEGATIVE_RAD: f32 = -0.001;
const J3_GRAVITY_UNLOAD_HARD_POSITIVE_RAD: f32 = 0.00030;
const J3_GRAVITY_UNLOAD_HARD_VELOCITY_RAD_S: f32 = 0.010;
const J3_GRAVITY_UNLOAD_MAX_TOTAL_TORQUE_NM: f32 = 0.90;
const J3_GRAVITY_UNLOAD_MAX_ACTIVE_SEC: f32 = 4.0;
const J3_GRAVITY_UNLOAD_FREEZE_DURATION: Duration = Duration::from_secs(1);
const J3_GRAVITY_UNLOAD_INITIAL_Q_LOWER_RAD: f32 = 3.138;
const J3_GRAVITY_UNLOAD_INITIAL_Q_UPPER_RAD: f32 = 3.1397;
const J3_ASSISTED_POSITION_DELTA_RAD: f32 = -0.005;
const J3_ASSISTED_POSITION_DURATION_SEC: f32 = 4.0;
const J3_ASSISTED_POSITION_PEAK_TORQUE_NM: f32 = -0.25;
const J3_ASSISTED_POSITION_EXPECTED_KP: f32 = 80.0;
const J3_ASSISTED_POSITION_EXPECTED_KD: f32 = 4.0;
const J3_ASSISTED_POSITION_EXPECTED_TORQUE_PERMILLE: u16 = 30;
const J3_ASSISTED_POSITION_EXPECTED_KP_KD_TORQUE_PERMILLE: u16 = 20;
const J3_ASSISTED_POSITION_EXPECTED_LOWER_RAD: f32 = 3.13;
#[allow(clippy::approx_constant)]
const J3_ASSISTED_POSITION_EXPECTED_UPPER_RAD: f32 = 3.14;
const J3_ASSISTED_POSITION_EXPECTED_VELOCITY_RAD_S: f32 = 0.02;
const J3_ASSISTED_POSITION_EXPECTED_ACCELERATION_RAD_S2: f32 = 0.02;
const J3_ASSISTED_POSITION_EXPECTED_TORQUE_NM: f32 = 0.9;
const J3_ASSISTED_POSITION_MAX_TOTAL_TORQUE_NM: f32 = 0.9;
const J3_ASSISTED_POSITION_MAX_ACTIVE_SEC: f32 = 8.0;
const J6_FIRST_POSITION_INDEX: usize = 5;
const J6_FIRST_POSITION_DELTA_RAD: f32 = -0.005;
const J6_FIRST_POSITION_DURATION_SEC: f32 = 4.0;
const J6_FIRST_POSITION_EXPECTED_KP: f32 = 25.0;
const J6_FIRST_POSITION_EXPECTED_KD: f32 = 1.0;
// The compressed-gain mapping is configured around the trial profile's 25/1
// defaults and deliberately admits at most 2x those values. Use that proved
// physical boundary rather than widening the mapping to mirror J5.
const J6_FIRST_POSITION_TRAJECTORY_KP: f32 = 50.0;
const J6_FIRST_POSITION_TRAJECTORY_KD: f32 = 2.0;
const J6_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD: f32 = 0.121_463;
const J6_FIRST_POSITION_EXPECTED_TORQUE_SCALE: f32 = 1.0;
const J6_FIRST_POSITION_EXPECTED_GRAVITY_SCALE: f32 = 1.0;
const J6_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE: u16 = 200;
const J6_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE: u16 = 100;
const J6_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD: f32 = -0.25;
const J6_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD: f32 = 0.35;
const J6_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S: f32 = 0.1;
const J6_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2: f32 = 0.1;
const J6_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM: f32 = 1.5;
const J6_FIRST_POSITION_INITIAL_Q_LOWER_RAD: f32 = -0.24;
const J6_FIRST_POSITION_INITIAL_Q_UPPER_RAD: f32 = 0.34;
const J6_FIRST_POSITION_MAX_FEEDFORWARD_NM: f32 = 0.10;
const J6_FIRST_POSITION_MAX_TOTAL_TORQUE_NM: f32 = 0.50;
const J6_FIRST_POSITION_MAX_ACTIVE_SEC: f32 = 8.0;
const J6_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD: f32 = -0.006;
const J6_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD: f32 = 0.0005;
const J6_FIRST_POSITION_ENABLE_HARD_POSITION_RAD: f32 = 0.001;
const J6_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S: f32 = 0.02;
const J6_FIRST_POSITION_GO_POSITION_RAD: f32 = 0.0005;
const J6_FIRST_POSITION_GO_VELOCITY_RAD_S: f32 = 0.01;
const J6_FIRST_POSITION_REQUIRED_PEAK_RAD: f32 = 0.0025;
const J6_FIRST_POSITION_RETURN_POSITION_RAD: f32 = 0.00125;
const J6_FIRST_POSITION_RETURN_VELOCITY_RAD_S: f32 = 0.01;
const J6_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV: f32 = 0.000_397_887;
// The drive's published protection limits are much higher (120 C driver,
// 110 C motor).  This deliberately conservative host gate is for a short
// commissioning experiment, not a replacement for the drive protections.
const MAX_DIAGNOSTIC_TEMPERATURE_C: f32 = 70.0;
const MIN_PLAUSIBLE_DIAGNOSTIC_TEMPERATURE_C: f32 = -40.0;
const MAX_DIAGNOSTIC_TEMPERATURE_RISE_C: f32 = 2.0;
const RPDO_READBACK_SETTLE: Duration = Duration::from_millis(25);

#[derive(Debug, Default)]
struct MotionTelemetryMilestones {
    next: usize,
}

impl MotionTelemetryMilestones {
    /// Return each crossed milestone exactly once. The caller may invoke this
    /// repeatedly for the same sample so a delayed loop iteration cannot lose
    /// a milestone, while ordinary 4 ms iterations emit nothing.
    fn take_due(&mut self, normalized_time: f32) -> Option<&'static str> {
        let (threshold, label) = *MOTION_TELEMETRY_MILESTONES.get(self.next)?;
        if normalized_time < threshold {
            return None;
        }
        self.next += 1;
        Some(label)
    }
}

#[derive(Debug, Default)]
struct CensoredGravityHoldDisplacementMilestones {
    next: usize,
}

impl CensoredGravityHoldDisplacementMilestones {
    /// Return every newly crossed positive-displacement boundary exactly once.
    /// Repeated calls on one feedback sample preserve all crossed boundaries
    /// if the sample jumps over more than one milestone.
    fn take_due(&mut self, measured_delta_rad: f32) -> Option<(f32, &'static str)> {
        let (threshold_rad, label) =
            *CENSORED_GRAVITY_HOLD_DISPLACEMENT_MILESTONES.get(self.next)?;
        if measured_delta_rad < threshold_rad {
            return None;
        }
        self.next += 1;
        Some((threshold_rad, label))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StabilityGateStatus {
    Waiting,
    Stable,
    TimedOut,
}

/// Require a condition to remain true for a continuous dwell window, with a
/// phase-wide timeout. A single failing observation discards all accumulated
/// dwell time instead of allowing intermittent samples to add up.
#[derive(Debug, Clone, Copy)]
struct ContinuousStabilityGate {
    required_dwell: Duration,
    timeout: Duration,
    stable_since: Option<Duration>,
}

impl ContinuousStabilityGate {
    fn new(required_dwell: Duration, timeout: Duration) -> Self {
        Self {
            required_dwell,
            timeout,
            stable_since: None,
        }
    }

    fn observe(&mut self, elapsed: Duration, condition_satisfied: bool) -> StabilityGateStatus {
        // A sample beyond the deadline cannot turn a timed-out phase into a
        // success, even if it happens to complete the dwell window.
        if elapsed > self.timeout {
            return StabilityGateStatus::TimedOut;
        }

        if condition_satisfied {
            let stable_since = *self.stable_since.get_or_insert(elapsed);
            if elapsed.saturating_sub(stable_since) >= self.required_dwell {
                return StabilityGateStatus::Stable;
            }
        } else {
            self.stable_since = None;
        }

        if elapsed >= self.timeout {
            StabilityGateStatus::TimedOut
        } else {
            StabilityGateStatus::Waiting
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CommissioningTelemetrySample {
    commanded_q: f32,
    measured_q: f32,
    measured_delta: f32,
    measured_velocity: f32,
    measured_torque: f32,
    gravity_ff: f32,
    kp: f32,
    kd: f32,
    estimated_pd_torque: f32,
}

#[derive(Debug, Clone, PartialEq)]
struct SafetyAbortTelemetrySample {
    telemetry: CommissioningTelemetrySample,
    raw_motor_position_rev: f32,
    raw_motor_velocity_rev_s: f32,
    raw_motor_torque_nm: f32,
    safe_target_built: bool,
    target_build_error: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommissioningRequest {
    pub selected_index: usize,
    pub delta_rad: f32,
    /// Total time for the complete start -> delta -> start round trip.
    pub duration_sec: f32,
}

/// Compile-time-fixed authorization for J1's first physical position survey.
/// It carries no user-controlled distance, duration, gains, torque, or drive
/// limits and therefore cannot become a general jog request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint1FirstPositionDiagnosticRequest {
    pub authorized: bool,
}

/// Compile-time-fixed authorization for J5's first physical motion. It is a
/// single negative 5 mrad round trip toward the observed zero direction and
/// carries no caller-selected distance, gains, torque, or timing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint5FirstPositionDiagnosticRequest {
    pub authorized: bool,
}

/// Compile-time-fixed authorization for J4's first bounded position-channel
/// survey with gravity feed-forward deliberately held at zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint4FirstPositionDiagnosticRequest {
    pub authorized: bool,
}

/// Independent authorization for J4's fixed-position, negative-torque
/// identification. It cannot authorize the J4 position survey or any
/// caller-selected torque/duration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint4CensoredTorqueDiagnosticRequest {
    pub authorized: bool,
}

/// Independent authorization for J4's fixed -5 mrad survey with a
/// compile-time trajectory-synchronous negative feed-forward envelope. It
/// cannot authorize the gravity-off survey, torque identification, or a
/// caller-selected trajectory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint4AssistedPositionDiagnosticRequest {
    pub authorized: bool,
}

/// Independent authorization for J3's fixed inward 5 mrad survey. Its
/// negative assistance follows the trajectory phase and is exactly zero at
/// enable and return; callers cannot select distance, timing, gains, or force.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint3AssistedPositionDiagnosticRequest {
    pub authorized: bool,
}

/// Compile-time-fixed authorization for J6's first physical motion. It is a
/// single negative 5 mrad round trip toward the observed zero direction and
/// carries no caller-controlled distance, gain, torque, or timing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint6FirstPositionDiagnosticRequest {
    pub authorized: bool,
}

/// Independent, compile-time-fixed authorization for the J1 fixed-position
/// positive-torque identification. It deliberately cannot authorize the J1
/// position survey, a J2 diagnostic, or a user-selected torque staircase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint1CensoredTorqueDiagnosticRequest {
    pub authorized: bool,
}

/// Independent authorization for the mirror-image J1 fixed-position
/// negative-torque identification. Keeping it as a separate public request
/// prevents a caller authorized for the exercised positive path from silently
/// selecting the opposite physical torque direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint1NegativeCensoredTorqueDiagnosticRequest {
    pub authorized: bool,
}

/// Independent authorization for J3's fixed-position negative gravity-load
/// identification. It never constructs a position trajectory and accepts no
/// caller-selected force, timing, gain, or limit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint3GravityUnloadDiagnosticRequest {
    pub authorized: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Joint1CensoredTorqueDirection {
    Positive,
    Negative,
}

impl Joint1CensoredTorqueDirection {
    const fn sign(self) -> f32 {
        match self {
            Self::Positive => 1.0,
            Self::Negative => -1.0,
        }
    }

    const fn label(self) -> &'static str {
        match self {
            Self::Positive => "positive",
            Self::Negative => "negative",
        }
    }

    fn directional_delta(self, measured_delta_rad: f32) -> f32 {
        self.sign() * measured_delta_rad
    }
}

impl Joint1CensoredTorqueDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J1_FIRST_POSITION_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_1 censored-torque diagnostic requires its independent fixed-policy authorization"
        );
        // Reuse every reviewed hardware/profile lock from the first-position
        // survey. The extra trajectory feasibility checks are conservative
        // even though this mode never constructs a position trajectory.
        Joint1FirstPositionDiagnosticRequest { authorized: true }.validate(profile)
    }
}

impl Joint1NegativeCensoredTorqueDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J1_FIRST_POSITION_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_1 negative censored-torque diagnostic requires its independent fixed-policy authorization"
        );
        Joint1FirstPositionDiagnosticRequest { authorized: true }.validate(profile)
    }
}

impl Joint3GravityUnloadDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J3_GRAVITY_UNLOAD_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_3 gravity-unload diagnostic requires its independent fixed-policy authorization"
        );
        profile.validate_single_turn_command_windows()?;
        validate_commissioning_gains(profile)?;
        anyhow::ensure!(
            profile.bus.direct_joint_mapping,
            "joint_3 gravity-unload diagnostic requires direct_joint_mapping=true"
        );
        let joint = profile
            .joints
            .get(J3_GRAVITY_UNLOAD_INDEX)
            .context("joint_3 gravity-unload profile has no joint_3")?;
        anyhow::ensure!(
            joint.name == "joint_3" && joint.node_id == 3 && joint.direction == 1,
            "joint_3 gravity-unload diagnostic requires canonical joint_3 at node_id=3 with direction=1"
        );
        anyhow::ensure!(
            joint.zero_offset_rad.to_bits()
                == J3_GRAVITY_UNLOAD_EXPECTED_ZERO_OFFSET_RAD.to_bits(),
            "joint_3 gravity-unload diagnostic requires zero_offset_rad={J3_GRAVITY_UNLOAD_EXPECTED_ZERO_OFFSET_RAD:.6} (profile has {:.6})",
            joint.zero_offset_rad
        );
        anyhow::ensure!(
            joint.torque_scale.to_bits() == J3_GRAVITY_UNLOAD_EXPECTED_TORQUE_SCALE.to_bits()
                && joint.gravity_compensation_scale.to_bits()
                    == J3_GRAVITY_UNLOAD_EXPECTED_GRAVITY_SCALE.to_bits(),
            "joint_3 gravity-unload diagnostic requires torque/gravity scales={J3_GRAVITY_UNLOAD_EXPECTED_TORQUE_SCALE:.2}/{J3_GRAVITY_UNLOAD_EXPECTED_GRAVITY_SCALE:.1} (profile has {:.6}/{:.6})",
            joint.torque_scale,
            joint.gravity_compensation_scale
        );
        anyhow::ensure!(
            joint.default_kp.to_bits() == J3_GRAVITY_UNLOAD_EXPECTED_KP.to_bits()
                && joint.default_kd.to_bits() == J3_GRAVITY_UNLOAD_EXPECTED_KD.to_bits(),
            "joint_3 gravity-unload diagnostic requires profile Kp/Kd={J3_GRAVITY_UNLOAD_EXPECTED_KP:.1}/{J3_GRAVITY_UNLOAD_EXPECTED_KD:.1} (profile has {:.6}/{:.6})",
            joint.default_kp,
            joint.default_kd
        );
        anyhow::ensure!(
            joint.torque_permille == J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_TORQUE_PERMILLE
                && joint.kp_kd_torque_permille
                    == J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE,
            "joint_3 gravity-unload diagnostic requires reviewed profile torque caps {J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_TORQUE_PERMILLE}/{J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE} before its fixed {J3_GRAVITY_UNLOAD_TORQUE_PERMILLE}/{J3_GRAVITY_UNLOAD_KP_KD_TORQUE_PERMILLE} override"
        );
        anyhow::ensure!(
            joint.limits.position_lower_rad.to_bits()
                == J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_LOWER_RAD.to_bits()
                && joint.limits.position_upper_rad.to_bits()
                    == J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_UPPER_RAD.to_bits()
                && joint.limits.measured_position_margin_rad.to_bits() == 0.0_f32.to_bits()
                && joint.limits.velocity_rad_s.to_bits() == 0.1_f32.to_bits()
                && joint.limits.acceleration_rad_s2.to_bits() == 0.1_f32.to_bits()
                && joint.limits.torque_nm.to_bits() == 7.5_f32.to_bits(),
            "joint_3 gravity-unload diagnostic requires limits [2.85,3.14], zero measured margin, velocity/acceleration=0.1/0.1, torque=7.5"
        );
        Ok(())
    }
}

impl Joint3AssistedPositionDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J3_GRAVITY_UNLOAD_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_3 assisted-position diagnostic requires its independent fixed-policy authorization"
        );
        profile.validate_single_turn_command_windows()?;
        validate_commissioning_gains(profile)?;
        anyhow::ensure!(
            profile.bus.direct_joint_mapping,
            "joint_3 assisted-position diagnostic requires direct_joint_mapping=true"
        );
        let joint = profile
            .joints
            .get(J3_GRAVITY_UNLOAD_INDEX)
            .context("joint_3 assisted-position profile has no joint_3")?;
        anyhow::ensure!(
            joint.name == "joint_3" && joint.node_id == 3 && joint.direction == 1,
            "joint_3 assisted-position diagnostic requires canonical joint_3 at node_id=3 with direction=1"
        );
        anyhow::ensure!(
            joint.zero_offset_rad.to_bits()
                == J3_GRAVITY_UNLOAD_EXPECTED_ZERO_OFFSET_RAD.to_bits()
                && joint.torque_scale.to_bits()
                    == J3_GRAVITY_UNLOAD_EXPECTED_TORQUE_SCALE.to_bits()
                && joint.gravity_compensation_scale.to_bits() == 0.0_f32.to_bits(),
            "joint_3 assisted-position diagnostic requires the reviewed zero offset, torque scale, and zero gravity scale"
        );
        anyhow::ensure!(
            joint.default_kp.to_bits() == J3_ASSISTED_POSITION_EXPECTED_KP.to_bits()
                && joint.default_kd.to_bits() == J3_ASSISTED_POSITION_EXPECTED_KD.to_bits(),
            "joint_3 assisted-position diagnostic requires Kp/Kd={J3_ASSISTED_POSITION_EXPECTED_KP:.1}/{J3_ASSISTED_POSITION_EXPECTED_KD:.1}"
        );
        anyhow::ensure!(
            joint.torque_permille == J3_ASSISTED_POSITION_EXPECTED_TORQUE_PERMILLE
                && joint.kp_kd_torque_permille
                    == J3_ASSISTED_POSITION_EXPECTED_KP_KD_TORQUE_PERMILLE,
            "joint_3 assisted-position diagnostic requires fixed 30/20 profile drive caps"
        );
        anyhow::ensure!(
            joint.limits.position_lower_rad.to_bits()
                == J3_ASSISTED_POSITION_EXPECTED_LOWER_RAD.to_bits()
                && joint.limits.position_upper_rad.to_bits()
                    == J3_ASSISTED_POSITION_EXPECTED_UPPER_RAD.to_bits()
                && joint.limits.measured_position_margin_rad.to_bits() == 0.0_f32.to_bits()
                && joint.limits.velocity_rad_s.to_bits()
                    == J3_ASSISTED_POSITION_EXPECTED_VELOCITY_RAD_S.to_bits()
                && joint.limits.acceleration_rad_s2.to_bits()
                    == J3_ASSISTED_POSITION_EXPECTED_ACCELERATION_RAD_S2.to_bits()
                && joint.limits.torque_nm.to_bits()
                    == J3_ASSISTED_POSITION_EXPECTED_TORQUE_NM.to_bits(),
            "joint_3 assisted-position diagnostic requires limits [3.13,3.14], zero measured margin, velocity/acceleration=0.02/0.02, torque=0.9"
        );
        let peak_velocity =
            PI * J3_ASSISTED_POSITION_DELTA_RAD.abs() / J3_ASSISTED_POSITION_DURATION_SEC;
        let peak_acceleration = 2.0 * PI * PI * J3_ASSISTED_POSITION_DELTA_RAD.abs()
            / (J3_ASSISTED_POSITION_DURATION_SEC * J3_ASSISTED_POSITION_DURATION_SEC);
        anyhow::ensure!(
            peak_velocity <= joint.limits.velocity_rad_s
                && peak_acceleration <= joint.limits.acceleration_rad_s2,
            "joint_3 assisted-position trajectory exceeds its reviewed velocity/acceleration limits"
        );
        Ok(())
    }
}

impl Joint1FirstPositionDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J1_FIRST_POSITION_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_1 first-position diagnostic requires its independent fixed-policy authorization"
        );
        profile.validate_single_turn_command_windows()?;
        validate_commissioning_gains(profile)?;
        anyhow::ensure!(
            profile.bus.direct_joint_mapping,
            "joint_1 first-position diagnostic requires direct_joint_mapping=true"
        );
        let joint = profile
            .joints
            .get(J1_FIRST_POSITION_INDEX)
            .context("joint_1 first-position profile has no joint_1")?;
        anyhow::ensure!(
            joint.name == "joint_1" && joint.node_id == 1 && joint.direction == -1,
            "joint_1 first-position diagnostic requires canonical joint_1 at node_id=1 with direction=-1"
        );
        anyhow::ensure!(
            joint.zero_offset_rad.to_bits()
                == J1_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD.to_bits(),
            "joint_1 first-position diagnostic requires zero_offset_rad={J1_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD:.6} (profile has {:.6})",
            joint.zero_offset_rad
        );
        anyhow::ensure!(
            joint.default_kp.to_bits() == J1_FIRST_POSITION_EXPECTED_KP.to_bits()
                && joint.default_kd.to_bits() == J1_FIRST_POSITION_EXPECTED_KD.to_bits(),
            "joint_1 first-position diagnostic requires true Kp/Kd={J1_FIRST_POSITION_EXPECTED_KP:.1}/{J1_FIRST_POSITION_EXPECTED_KD:.1} (profile has {:.6}/{:.6})",
            joint.default_kp,
            joint.default_kd
        );
        anyhow::ensure!(
            joint.torque_scale.to_bits() == J1_FIRST_POSITION_EXPECTED_TORQUE_SCALE.to_bits(),
            "joint_1 first-position diagnostic requires torque_scale={J1_FIRST_POSITION_EXPECTED_TORQUE_SCALE:.2} (profile has {:.6})",
            joint.torque_scale
        );
        anyhow::ensure!(
            joint.gravity_compensation_scale.to_bits()
                == J1_FIRST_POSITION_EXPECTED_GRAVITY_SCALE.to_bits(),
            "joint_1 first-position diagnostic requires gravity_compensation_scale={J1_FIRST_POSITION_EXPECTED_GRAVITY_SCALE:.1} (profile has {:.6})",
            joint.gravity_compensation_scale
        );
        anyhow::ensure!(
            joint.torque_permille == J1_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE
                && joint.kp_kd_torque_permille
                    == J1_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE,
            "joint_1 first-position diagnostic requires reviewed profile torque_permille/kp_kd_torque_permille={J1_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE}/{J1_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE} before applying its lower fixed 30/20 override (profile has {}/{})",
            joint.torque_permille,
            joint.kp_kd_torque_permille
        );
        anyhow::ensure!(
            joint.limits.position_lower_rad.to_bits()
                == J1_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD.to_bits()
                && joint.limits.position_upper_rad.to_bits()
                    == J1_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD.to_bits()
                && joint.limits.measured_position_margin_rad.to_bits() == 0.0_f32.to_bits()
                && joint.limits.velocity_rad_s.to_bits()
                    == J1_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S.to_bits()
                && joint.limits.acceleration_rad_s2.to_bits()
                    == J1_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2.to_bits()
                && joint.limits.torque_nm.to_bits()
                    == J1_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM.to_bits(),
            "joint_1 first-position diagnostic requires command limits [-0.25,0.25] rad, zero measured margin, velocity/acceleration=0.1/0.1, torque=6.0; profile has [{:.6},{:.6}] margin {:.6}, {:.6}/{:.6}, {:.6}",
            joint.limits.position_lower_rad,
            joint.limits.position_upper_rad,
            joint.limits.measured_position_margin_rad,
            joint.limits.velocity_rad_s,
            joint.limits.acceleration_rad_s2,
            joint.limits.torque_nm
        );
        let peak_velocity = PI * J1_FIRST_POSITION_DELTA_RAD / J1_FIRST_POSITION_DURATION_SEC;
        let peak_acceleration = 2.0 * PI * PI * J1_FIRST_POSITION_DELTA_RAD
            / (J1_FIRST_POSITION_DURATION_SEC * J1_FIRST_POSITION_DURATION_SEC);
        anyhow::ensure!(
            peak_velocity <= joint.limits.velocity_rad_s,
            "joint_1 fixed survey peak target velocity {peak_velocity:.6} rad/s exceeds the profile limit {:.6} rad/s",
            joint.limits.velocity_rad_s
        );
        anyhow::ensure!(
            peak_acceleration <= joint.limits.acceleration_rad_s2,
            "joint_1 fixed survey peak target acceleration {peak_acceleration:.6} rad/s^2 exceeds the profile limit {:.6} rad/s^2",
            joint.limits.acceleration_rad_s2
        );
        Ok(())
    }
}

impl Joint5FirstPositionDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J5_FIRST_POSITION_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_5 first-position diagnostic requires its independent fixed-policy authorization"
        );
        profile.validate_single_turn_command_windows()?;
        validate_commissioning_gains(profile)?;
        anyhow::ensure!(
            profile.bus.direct_joint_mapping,
            "joint_5 first-position diagnostic requires direct_joint_mapping=true"
        );
        let joint = profile
            .joints
            .get(J5_FIRST_POSITION_INDEX)
            .context("joint_5 first-position profile has no joint_5")?;
        anyhow::ensure!(
            joint.name == "joint_5" && joint.node_id == 5 && joint.direction == 1,
            "joint_5 first-position diagnostic requires canonical joint_5 at node_id=5 with direction=1"
        );
        anyhow::ensure!(
            joint.zero_offset_rad.to_bits()
                == J5_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD.to_bits(),
            "joint_5 first-position diagnostic requires zero_offset_rad={J5_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD:.6} (profile has {:.6})",
            joint.zero_offset_rad
        );
        anyhow::ensure!(
            joint.default_kp.to_bits() == J5_FIRST_POSITION_EXPECTED_KP.to_bits()
                && joint.default_kd.to_bits() == J5_FIRST_POSITION_EXPECTED_KD.to_bits(),
            "joint_5 first-position diagnostic requires true Kp/Kd={J5_FIRST_POSITION_EXPECTED_KP:.1}/{J5_FIRST_POSITION_EXPECTED_KD:.1} (profile has {:.6}/{:.6})",
            joint.default_kp,
            joint.default_kd
        );
        anyhow::ensure!(
            joint.torque_scale.to_bits() == J5_FIRST_POSITION_EXPECTED_TORQUE_SCALE.to_bits()
                && joint.gravity_compensation_scale.to_bits()
                    == J5_FIRST_POSITION_EXPECTED_GRAVITY_SCALE.to_bits(),
            "joint_5 first-position diagnostic requires torque/gravity scales={J5_FIRST_POSITION_EXPECTED_TORQUE_SCALE:.1}/{J5_FIRST_POSITION_EXPECTED_GRAVITY_SCALE:.1} (profile has {:.6}/{:.6})",
            joint.torque_scale,
            joint.gravity_compensation_scale
        );
        anyhow::ensure!(
            joint.torque_permille == J5_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE
                && joint.kp_kd_torque_permille
                    == J5_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE,
            "joint_5 first-position diagnostic requires reviewed profile torque caps {J5_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE}/{J5_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE} before its fixed 50/30 override (profile has {}/{})",
            joint.torque_permille,
            joint.kp_kd_torque_permille
        );
        anyhow::ensure!(
            joint.limits.position_lower_rad.to_bits()
                == J5_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD.to_bits()
                && joint.limits.position_upper_rad.to_bits()
                    == J5_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD.to_bits()
                && joint.limits.measured_position_margin_rad.to_bits() == 0.0_f32.to_bits()
                && joint.limits.velocity_rad_s.to_bits()
                    == J5_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S.to_bits()
                && joint.limits.acceleration_rad_s2.to_bits()
                    == J5_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2.to_bits()
                && joint.limits.torque_nm.to_bits()
                    == J5_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM.to_bits(),
            "joint_5 first-position diagnostic requires limits [-0.25,0.25], zero measured margin, velocity/acceleration=0.1/0.1, torque=1.5"
        );
        let peak_velocity = PI * J5_FIRST_POSITION_DELTA_RAD.abs() / J5_FIRST_POSITION_DURATION_SEC;
        let peak_acceleration = 2.0 * PI * PI * J5_FIRST_POSITION_DELTA_RAD.abs()
            / (J5_FIRST_POSITION_DURATION_SEC * J5_FIRST_POSITION_DURATION_SEC);
        anyhow::ensure!(
            peak_velocity <= joint.limits.velocity_rad_s
                && peak_acceleration <= joint.limits.acceleration_rad_s2,
            "joint_5 fixed survey trajectory exceeds reviewed velocity/acceleration limits"
        );
        Ok(())
    }
}

impl Joint4FirstPositionDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J4_FIRST_POSITION_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_4 first-position diagnostic requires its independent fixed-policy authorization"
        );
        profile.validate_single_turn_command_windows()?;
        validate_commissioning_gains(profile)?;
        anyhow::ensure!(
            profile.bus.direct_joint_mapping,
            "joint_4 first-position diagnostic requires direct_joint_mapping=true"
        );
        let joint = profile
            .joints
            .get(J4_FIRST_POSITION_INDEX)
            .context("joint_4 first-position profile has no joint_4")?;
        anyhow::ensure!(
            joint.name == "joint_4" && joint.node_id == 4 && joint.direction == 1,
            "joint_4 first-position diagnostic requires canonical joint_4 at node_id=4 with direction=1"
        );
        anyhow::ensure!(
            joint.zero_offset_rad.to_bits()
                == J4_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD.to_bits(),
            "joint_4 first-position diagnostic requires zero_offset_rad={J4_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD:.6} (profile has {:.6})",
            joint.zero_offset_rad
        );
        anyhow::ensure!(
            joint.default_kp.to_bits() == J4_FIRST_POSITION_EXPECTED_KP.to_bits()
                && joint.default_kd.to_bits() == J4_FIRST_POSITION_EXPECTED_KD.to_bits(),
            "joint_4 first-position diagnostic requires profile Kp/Kd={J4_FIRST_POSITION_EXPECTED_KP:.1}/{J4_FIRST_POSITION_EXPECTED_KD:.1} (profile has {:.6}/{:.6})",
            joint.default_kp,
            joint.default_kd
        );
        anyhow::ensure!(
            J4_FIRST_POSITION_TRAJECTORY_KP <= 2.0 * joint.default_kp.max(1.0)
                && J4_FIRST_POSITION_TRAJECTORY_KD <= 2.0 * joint.default_kd.max(0.1),
            "joint_4 temporary trajectory gains must fit the configured compressed-MIT mapping"
        );
        anyhow::ensure!(
            joint.torque_scale.to_bits() == J4_FIRST_POSITION_EXPECTED_TORQUE_SCALE.to_bits()
                && joint.gravity_compensation_scale.to_bits()
                    == J4_FIRST_POSITION_EXPECTED_GRAVITY_SCALE.to_bits(),
            "joint_4 first-position diagnostic requires torque/gravity scales={J4_FIRST_POSITION_EXPECTED_TORQUE_SCALE:.1}/{J4_FIRST_POSITION_EXPECTED_GRAVITY_SCALE:.1} (profile has {:.6}/{:.6})",
            joint.torque_scale,
            joint.gravity_compensation_scale
        );
        anyhow::ensure!(
            joint.torque_permille == J4_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE
                && joint.kp_kd_torque_permille
                    == J4_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE,
            "joint_4 first-position diagnostic requires reviewed profile torque caps {J4_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE}/{J4_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE} before its fixed 60/50 override (profile has {}/{})",
            joint.torque_permille,
            joint.kp_kd_torque_permille
        );
        anyhow::ensure!(
            joint.limits.position_lower_rad.to_bits()
                == J4_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD.to_bits()
                && joint.limits.position_upper_rad.to_bits()
                    == J4_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD.to_bits()
                && joint.limits.measured_position_margin_rad.to_bits() == 0.0_f32.to_bits()
                && joint.limits.velocity_rad_s.to_bits()
                    == J4_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S.to_bits()
                && joint.limits.acceleration_rad_s2.to_bits()
                    == J4_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2.to_bits()
                && joint.limits.torque_nm.to_bits()
                    == J4_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM.to_bits(),
            "joint_4 first-position diagnostic requires limits [-0.25,0.25], zero measured margin, velocity/acceleration=0.1/0.1, torque=1.5"
        );
        let peak_velocity = PI * J4_FIRST_POSITION_DELTA_RAD.abs() / J4_FIRST_POSITION_DURATION_SEC;
        let peak_acceleration = 2.0 * PI * PI * J4_FIRST_POSITION_DELTA_RAD.abs()
            / (J4_FIRST_POSITION_DURATION_SEC * J4_FIRST_POSITION_DURATION_SEC);
        anyhow::ensure!(
            peak_velocity <= joint.limits.velocity_rad_s
                && peak_acceleration <= joint.limits.acceleration_rad_s2,
            "joint_4 fixed survey trajectory exceeds reviewed velocity/acceleration limits"
        );
        Ok(())
    }
}

impl Joint4CensoredTorqueDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J4_FIRST_POSITION_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_4 censored-torque diagnostic requires its independent fixed-policy authorization"
        );
        Joint4FirstPositionDiagnosticRequest { authorized: true }.validate(profile)
    }
}

impl Joint4AssistedPositionDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J4_FIRST_POSITION_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_4 assisted-position diagnostic requires its independent fixed-policy authorization"
        );
        Joint4FirstPositionDiagnosticRequest { authorized: true }.validate(profile)
    }
}

impl Joint6FirstPositionDiagnosticRequest {
    pub const fn selected_index(self) -> usize {
        J6_FIRST_POSITION_INDEX
    }

    pub fn validate(self, profile: &HardwareProfile) -> Result<()> {
        anyhow::ensure!(
            self.authorized,
            "joint_6 first-position diagnostic requires its independent fixed-policy authorization"
        );
        profile.validate_single_turn_command_windows()?;
        validate_commissioning_gains(profile)?;
        anyhow::ensure!(
            profile.bus.direct_joint_mapping,
            "joint_6 first-position diagnostic requires direct_joint_mapping=true"
        );
        let joint = profile
            .joints
            .get(J6_FIRST_POSITION_INDEX)
            .context("joint_6 first-position profile has no joint_6")?;
        anyhow::ensure!(
            joint.name == "joint_6" && joint.node_id == 6 && joint.direction == 1,
            "joint_6 first-position diagnostic requires canonical joint_6 at node_id=6 with direction=1"
        );
        anyhow::ensure!(
            joint.zero_offset_rad.to_bits()
                == J6_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD.to_bits(),
            "joint_6 first-position diagnostic requires zero_offset_rad={J6_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD:.6} (profile has {:.6})",
            joint.zero_offset_rad
        );
        anyhow::ensure!(
            joint.default_kp.to_bits() == J6_FIRST_POSITION_EXPECTED_KP.to_bits()
                && joint.default_kd.to_bits() == J6_FIRST_POSITION_EXPECTED_KD.to_bits(),
            "joint_6 first-position diagnostic requires profile Kp/Kd={J6_FIRST_POSITION_EXPECTED_KP:.1}/{J6_FIRST_POSITION_EXPECTED_KD:.1} (profile has {:.6}/{:.6})",
            joint.default_kp,
            joint.default_kd
        );
        anyhow::ensure!(
            J6_FIRST_POSITION_TRAJECTORY_KP <= 2.0 * joint.default_kp.max(1.0)
                && J6_FIRST_POSITION_TRAJECTORY_KD <= 2.0 * joint.default_kd.max(0.1),
            "joint_6 temporary trajectory gains must fit the configured compressed-MIT mapping"
        );
        anyhow::ensure!(
            joint.torque_scale.to_bits() == J6_FIRST_POSITION_EXPECTED_TORQUE_SCALE.to_bits()
                && joint.gravity_compensation_scale.to_bits()
                    == J6_FIRST_POSITION_EXPECTED_GRAVITY_SCALE.to_bits(),
            "joint_6 first-position diagnostic requires torque/gravity scales={J6_FIRST_POSITION_EXPECTED_TORQUE_SCALE:.1}/{J6_FIRST_POSITION_EXPECTED_GRAVITY_SCALE:.1} (profile has {:.6}/{:.6})",
            joint.torque_scale,
            joint.gravity_compensation_scale
        );
        anyhow::ensure!(
            joint.torque_permille == J6_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE
                && joint.kp_kd_torque_permille
                    == J6_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE,
            "joint_6 first-position diagnostic requires reviewed profile torque caps {J6_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE}/{J6_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE} before its fixed 50/30 override (profile has {}/{})",
            joint.torque_permille,
            joint.kp_kd_torque_permille
        );
        anyhow::ensure!(
            joint.limits.position_lower_rad.to_bits()
                == J6_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD.to_bits()
                && joint.limits.position_upper_rad.to_bits()
                    == J6_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD.to_bits()
                && joint.limits.measured_position_margin_rad.to_bits() == 0.0_f32.to_bits()
                && joint.limits.velocity_rad_s.to_bits()
                    == J6_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S.to_bits()
                && joint.limits.acceleration_rad_s2.to_bits()
                    == J6_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2.to_bits()
                && joint.limits.torque_nm.to_bits()
                    == J6_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM.to_bits(),
            "joint_6 first-position diagnostic requires limits [-0.25,0.35], zero measured margin, velocity/acceleration=0.1/0.1, torque=1.5"
        );
        let peak_velocity = PI * J6_FIRST_POSITION_DELTA_RAD.abs() / J6_FIRST_POSITION_DURATION_SEC;
        let peak_acceleration = 2.0 * PI * PI * J6_FIRST_POSITION_DELTA_RAD.abs()
            / (J6_FIRST_POSITION_DURATION_SEC * J6_FIRST_POSITION_DURATION_SEC);
        anyhow::ensure!(
            peak_velocity <= joint.limits.velocity_rad_s
                && peak_acceleration <= joint.limits.acceleration_rad_s2,
            "joint_6 fixed survey trajectory exceeds reviewed velocity/acceleration limits"
        );
        Ok(())
    }
}

/// A deliberately non-general J2 diagnostic.  `Hold` never changes the
/// feedback-derived position or feed-forward target.  `TauFfStaircase` keeps
/// the same position/Kp/Kd hold and adds only small positive ROS-joint torque
/// steps. `GravityHoldCensored` only changes gravity feed-forward while its
/// position target remains bitwise fixed, and freezes the previous verified
/// microstep at its displacement censor. `PositionRoundTrip` is one fixed,
/// positive, smooth 5 mrad survey at the lower software/mechanical boundary;
/// it is not a general jog primitive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SingleAxisDiagnosticMode {
    Hold {
        duration_sec: f32,
    },
    TauFfStaircase {
        peak_additive_torque_nm: f32,
        step_torque_nm: f32,
        dwell_sec: f32,
    },
    GravityHoldCensored,
    PositionRoundTrip {
        delta_rad: f32,
        duration_sec: f32,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SingleAxisDiagnosticRequest {
    pub selected_index: usize,
    pub mode: SingleAxisDiagnosticMode,
    /// Second, independent acknowledgement for requests above the low-energy
    /// 0.25 Nm additive tier, the censored gravity hold, and the fixed position
    /// round trip. Merely setting this flag never loosens a request whose peak
    /// remains in the low tier.
    pub high_torque_authorized: bool,
    /// Third, mode-specific acknowledgement for the feedback-censored gravity
    /// hold. Keeping this in the library request prevents non-CLI callers from
    /// bypassing the independent acknowledgement enforced by the CLI.
    pub censored_gravity_hold_authorized: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct DiagnosticTorqueLimits {
    max_additive_torque_nm: f32,
    max_total_torque_nm: f32,
    max_dwell_sec: f32,
    max_active_sec: f32,
    high_tier: bool,
}

impl SingleAxisDiagnosticRequest {
    /// Whether this request actually crosses the separately authorized
    /// high-torque boundary.  CLI-only safety acknowledgements use this exact
    /// predicate so they cannot drift from the diagnostic tier selection.
    pub fn uses_high_torque_tier(&self) -> bool {
        self.torque_limits().high_tier
    }

    pub fn validate(&self, profile: &HardwareProfile) -> Result<()> {
        validate_commissioning_gains(profile)?;
        anyhow::ensure!(
            self.selected_index == DIAGNOSTIC_JOINT_INDEX,
            "the bounded brake/RPDO diagnostic is currently restricted to joint_2"
        );
        anyhow::ensure!(
            self.selected_index < profile.joints.len(),
            "diagnostic joint index {} is outside the hardware profile",
            self.selected_index
        );
        anyhow::ensure!(
            matches!(self.mode, SingleAxisDiagnosticMode::GravityHoldCensored)
                || !self.censored_gravity_hold_authorized,
            "censored gravity-hold authorization is valid only for censored gravity-hold mode"
        );
        match self.mode {
            SingleAxisDiagnosticMode::Hold { duration_sec } => {
                anyhow::ensure!(
                    !self.high_torque_authorized,
                    "high-torque diagnostic authorization is valid only for tau-ff-staircase mode"
                );
                anyhow::ensure!(
                    duration_sec.is_finite()
                        && (MIN_DIAGNOSTIC_HOLD_SEC..=MAX_DIAGNOSTIC_HOLD_SEC)
                            .contains(&duration_sec),
                    "diagnostic hold duration must be finite and within [{MIN_DIAGNOSTIC_HOLD_SEC}, {MAX_DIAGNOSTIC_HOLD_SEC}] seconds"
                );
            }
            SingleAxisDiagnosticMode::TauFfStaircase {
                peak_additive_torque_nm,
                step_torque_nm,
                dwell_sec,
            } => {
                let limits = self.torque_limits();
                anyhow::ensure!(
                    peak_additive_torque_nm
                        <= LOW_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM
                        || self.high_torque_authorized,
                    "joint_2 diagnostic peak above {LOW_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM:.3} Nm requires explicit high-torque authorization"
                );
                anyhow::ensure!(
                    peak_additive_torque_nm.is_finite()
                        && (MIN_DIAGNOSTIC_TORQUE_STEP_NM
                            ..=limits.max_additive_torque_nm)
                            .contains(&peak_additive_torque_nm),
                    "joint_2 diagnostic peak additive torque must be positive and within [{MIN_DIAGNOSTIC_TORQUE_STEP_NM}, {:.3}] Nm for the selected tier",
                    limits.max_additive_torque_nm
                );
                anyhow::ensure!(
                    step_torque_nm.is_finite()
                        && (MIN_DIAGNOSTIC_TORQUE_STEP_NM..=peak_additive_torque_nm)
                            .contains(&step_torque_nm)
                        && step_torque_nm <= MAX_DIAGNOSTIC_TORQUE_STEP_NM,
                    "joint_2 diagnostic torque step must be within [{MIN_DIAGNOSTIC_TORQUE_STEP_NM}, min(peak, {MAX_DIAGNOSTIC_TORQUE_STEP_NM})] Nm"
                );
                anyhow::ensure!(
                    dwell_sec.is_finite()
                        && (MIN_DIAGNOSTIC_DWELL_SEC..=limits.max_dwell_sec)
                            .contains(&dwell_sec),
                    "diagnostic staircase dwell must be finite and within [{MIN_DIAGNOSTIC_DWELL_SEC}, {:.3}] seconds for the selected tier",
                    limits.max_dwell_sec
                );
                let levels = diagnostic_torque_levels(peak_additive_torque_nm, step_torque_nm)?;
                anyhow::ensure!(
                    levels.len() <= MAX_DIAGNOSTIC_STEPS,
                    "diagnostic staircase has {} levels; maximum is {MAX_DIAGNOSTIC_STEPS}",
                    levels.len()
                );
                let active_sec = (levels.len() as f32 + 1.0) * dwell_sec;
                anyhow::ensure!(
                    active_sec <= limits.max_active_sec,
                    "diagnostic staircase plus baseline restore would remain active for {active_sec:.3} s; selected-tier maximum is {:.3} s",
                    limits.max_active_sec
                );
            }
            SingleAxisDiagnosticMode::PositionRoundTrip {
                delta_rad,
                duration_sec,
            } => {
                anyhow::ensure!(
                    self.high_torque_authorized,
                    "joint_2 position-round-trip requires explicit high-torque authorization"
                );
                anyhow::ensure!(
                    delta_rad.to_bits() == POSITION_DIAGNOSTIC_DELTA_RAD.to_bits(),
                    "joint_2 position-round-trip delta must be exactly +{POSITION_DIAGNOSTIC_DELTA_RAD:.3} rad"
                );
                anyhow::ensure!(
                    duration_sec.to_bits() == POSITION_DIAGNOSTIC_DURATION_SEC.to_bits(),
                    "joint_2 position-round-trip duration must be exactly {POSITION_DIAGNOSTIC_DURATION_SEC:.1} seconds"
                );
                let joint = &profile.joints[self.selected_index];
                anyhow::ensure!(
                    (joint.gravity_compensation_scale
                        - POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE)
                        .abs()
                        <= f32::EPSILON,
                    "joint_2 position-round-trip requires gravity_compensation_scale={POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE:.2} (profile has {:.6})",
                    joint.gravity_compensation_scale
                );
                anyhow::ensure!(
                    (joint.default_kp - POSITION_DIAGNOSTIC_EXPECTED_KP).abs() <= f32::EPSILON
                        && (joint.default_kd - POSITION_DIAGNOSTIC_EXPECTED_KD).abs()
                            <= f32::EPSILON,
                    "joint_2 position-round-trip requires Kp/Kd={POSITION_DIAGNOSTIC_EXPECTED_KP:.1}/{POSITION_DIAGNOSTIC_EXPECTED_KD:.1} (profile has {:.6}/{:.6})",
                    joint.default_kp,
                    joint.default_kd
                );
                anyhow::ensure!(
                    joint.torque_permille == POSITION_DIAGNOSTIC_EXPECTED_TORQUE_PERMILLE
                        && joint.kp_kd_torque_permille
                            == POSITION_DIAGNOSTIC_EXPECTED_KP_KD_TORQUE_PERMILLE,
                    "joint_2 position-round-trip requires torque_permille/kp_kd_torque_permille={POSITION_DIAGNOSTIC_EXPECTED_TORQUE_PERMILLE}/{POSITION_DIAGNOSTIC_EXPECTED_KP_KD_TORQUE_PERMILLE} (profile has {}/{})",
                    joint.torque_permille,
                    joint.kp_kd_torque_permille
                );
                let peak_velocity = PI * delta_rad / duration_sec;
                anyhow::ensure!(
                    peak_velocity <= joint.limits.velocity_rad_s,
                    "joint_2 position-round-trip peak target velocity {peak_velocity:.6} rad/s exceeds the profile limit {:.6} rad/s",
                    joint.limits.velocity_rad_s
                );
                let peak_acceleration = 2.0 * PI * PI * delta_rad / (duration_sec * duration_sec);
                anyhow::ensure!(
                    peak_acceleration <= joint.limits.acceleration_rad_s2,
                    "joint_2 position-round-trip peak target acceleration {peak_acceleration:.6} rad/s^2 exceeds the profile limit {:.6} rad/s^2",
                    joint.limits.acceleration_rad_s2
                );
            }
            SingleAxisDiagnosticMode::GravityHoldCensored => {
                anyhow::ensure!(
                    self.high_torque_authorized,
                    "joint_2 censored gravity hold requires explicit high-torque authorization"
                );
                anyhow::ensure!(
                    self.censored_gravity_hold_authorized,
                    "joint_2 censored gravity hold requires its independent mode-specific authorization"
                );
                let joint = &profile.joints[self.selected_index];
                anyhow::ensure!(
                    (joint.gravity_compensation_scale
                        - CENSORED_GRAVITY_HOLD_PROFILE_SCALE)
                        .abs()
                        <= f32::EPSILON,
                    "joint_2 censored gravity hold requires the previously exercised gravity_compensation_scale={CENSORED_GRAVITY_HOLD_PROFILE_SCALE:.2} profile baseline (profile has {:.6})",
                    joint.gravity_compensation_scale
                );
                anyhow::ensure!(
                    (joint.default_kp - POSITION_DIAGNOSTIC_EXPECTED_KP).abs() <= f32::EPSILON
                        && (joint.default_kd - POSITION_DIAGNOSTIC_EXPECTED_KD).abs()
                            <= f32::EPSILON,
                    "joint_2 censored gravity hold requires Kp/Kd={POSITION_DIAGNOSTIC_EXPECTED_KP:.1}/{POSITION_DIAGNOSTIC_EXPECTED_KD:.1} (profile has {:.6}/{:.6})",
                    joint.default_kp,
                    joint.default_kd
                );
                anyhow::ensure!(
                    joint.torque_permille == POSITION_DIAGNOSTIC_EXPECTED_TORQUE_PERMILLE
                        && joint.kp_kd_torque_permille
                            == POSITION_DIAGNOSTIC_EXPECTED_KP_KD_TORQUE_PERMILLE,
                    "joint_2 censored gravity hold requires torque_permille/kp_kd_torque_permille={POSITION_DIAGNOSTIC_EXPECTED_TORQUE_PERMILLE}/{POSITION_DIAGNOSTIC_EXPECTED_KP_KD_TORQUE_PERMILLE} (profile has {}/{})",
                    joint.torque_permille,
                    joint.kp_kd_torque_permille
                );
            }
        }
        Ok(())
    }

    fn torque_limits(&self) -> DiagnosticTorqueLimits {
        let high_tier = matches!(
            self.mode,
            SingleAxisDiagnosticMode::TauFfStaircase {
                peak_additive_torque_nm,
                ..
            } if peak_additive_torque_nm > LOW_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM
        );
        if matches!(
            self.mode,
            SingleAxisDiagnosticMode::PositionRoundTrip { .. }
                | SingleAxisDiagnosticMode::GravityHoldCensored
        ) {
            return DiagnosticTorqueLimits {
                max_additive_torque_nm: 0.0,
                max_total_torque_nm: POSITION_DIAGNOSTIC_MAX_TOTAL_TORQUE_NM,
                max_dwell_sec: 0.0,
                max_active_sec: if matches!(
                    self.mode,
                    SingleAxisDiagnosticMode::GravityHoldCensored
                ) {
                    CENSORED_GRAVITY_HOLD_MAX_ACTIVE_SEC
                } else {
                    POSITION_DIAGNOSTIC_MAX_ACTIVE_SEC
                },
                high_tier: true,
            };
        }
        if high_tier {
            DiagnosticTorqueLimits {
                max_additive_torque_nm: HIGH_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM,
                max_total_torque_nm: HIGH_TIER_MAX_DIAGNOSTIC_TOTAL_TORQUE_NM,
                max_dwell_sec: HIGH_TIER_MAX_DIAGNOSTIC_DWELL_SEC,
                max_active_sec: HIGH_TIER_MAX_DIAGNOSTIC_ACTIVE_SEC,
                high_tier,
            }
        } else {
            DiagnosticTorqueLimits {
                max_additive_torque_nm: LOW_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM,
                max_total_torque_nm: LOW_TIER_MAX_DIAGNOSTIC_TOTAL_TORQUE_NM,
                max_dwell_sec: MAX_DIAGNOSTIC_DWELL_SEC,
                max_active_sec: MAX_DIAGNOSTIC_ACTIVE_SEC,
                high_tier,
            }
        }
    }
}

impl CommissioningRequest {
    pub fn validate(&self, profile: &HardwareProfile) -> Result<()> {
        validate_commissioning_gains(profile)?;
        anyhow::ensure!(
            self.selected_index < DOF,
            "commissioning joint index {} is outside 0..{}",
            self.selected_index,
            DOF
        );
        anyhow::ensure!(
            self.delta_rad.is_finite()
                && (MIN_COMMISSION_DELTA_RAD..=MAX_COMMISSION_DELTA_RAD)
                    .contains(&self.delta_rad.abs()),
            "--delta-rad magnitude must be within [{MIN_COMMISSION_DELTA_RAD}, {MAX_COMMISSION_DELTA_RAD}] rad"
        );
        anyhow::ensure!(
            self.duration_sec.is_finite()
                && self.duration_sec > 0.0
                && self.duration_sec <= MAX_COMMISSION_DURATION_SEC,
            "--duration-sec must be finite and within (0, {MAX_COMMISSION_DURATION_SEC}] seconds"
        );

        let joint = &profile.joints[self.selected_index];
        // q(t)=q0 + 0.5*dq*(1-cos(2*pi*t/T)); the peak setpoint
        // speed is pi*|dq|/T. This is stricter than the average-speed bound
        // |dq|/T and guarantees the entire smooth round trip respects profile.
        let peak_velocity = PI * self.delta_rad.abs() / self.duration_sec;
        anyhow::ensure!(
            peak_velocity <= joint.limits.velocity_rad_s,
            "{} smooth commissioning peak velocity {:.6} rad/s exceeds profile limit {:.6} rad/s; increase --duration-sec to at least {:.3}",
            joint.name,
            peak_velocity,
            joint.limits.velocity_rad_s,
            PI * self.delta_rad.abs() / joint.limits.velocity_rad_s
        );
        // The same sinusoidal round trip has peak acceleration
        // 2*pi^2*|delta|/T^2. Enforce the independent hardware-profile limit,
        // rather than relying on MoveIt time parameterization.
        let peak_acceleration =
            2.0 * PI * PI * self.delta_rad.abs() / (self.duration_sec * self.duration_sec);
        anyhow::ensure!(
            peak_acceleration <= joint.limits.acceleration_rad_s2,
            "{} smooth commissioning peak acceleration {:.6} rad/s^2 exceeds profile limit {:.6} rad/s^2; increase --duration-sec to at least {:.3}",
            joint.name,
            peak_acceleration,
            joint.limits.acceleration_rad_s2,
            (2.0 * PI * PI * self.delta_rad.abs() / joint.limits.acceleration_rad_s2).sqrt()
        );
        Ok(())
    }
}

/// Run one smooth out-and-back motion. The caller owns process-signal handling
/// and final disable handling across initialization, enable, motion, and all
/// error paths, so this function neither competes for signals nor hides a
/// disable failure.
pub async fn run_single_axis_commissioning(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: CommissioningRequest,
) -> Result<()> {
    run_single_axis_commissioning_inner(backend, profile, dynamics, request, None).await
}

/// Run the one reviewed J3 inward survey with trajectory-synchronous force
/// assistance. The assistance is zero before motion and after return; all
/// caller-selectable commissioning inputs are replaced by compile-time values.
pub async fn run_joint3_assisted_position_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint3AssistedPositionDiagnosticRequest,
) -> Result<()> {
    backend
        .ensure_joint3_gravity_unload_strict_session(profile)
        .context("joint_3 assisted-position backend/profile authority gate failed")?;
    request.validate(profile)?;
    tracing::info!(
        phase = PHASE_J3_ASSISTED_POSITION,
        joint = "joint_3",
        delta_rad = J3_ASSISTED_POSITION_DELTA_RAD,
        duration_sec = J3_ASSISTED_POSITION_DURATION_SEC,
        peak_assistance_nm = J3_ASSISTED_POSITION_PEAK_TORQUE_NM,
        "starting fixed-policy trajectory-synchronous assisted position survey"
    );
    run_single_axis_commissioning_inner(
        backend,
        profile,
        dynamics,
        CommissioningRequest {
            selected_index: request.selected_index(),
            delta_rad: J3_ASSISTED_POSITION_DELTA_RAD,
            duration_sec: J3_ASSISTED_POSITION_DURATION_SEC,
        },
        Some(J3_ASSISTED_POSITION_PEAK_TORQUE_NM),
    )
    .await
}

async fn run_single_axis_commissioning_inner(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: CommissioningRequest,
    peak_phase_assistance_nm: Option<f32>,
) -> Result<()> {
    request.validate(profile)?;
    profile.validate_single_turn_command_windows()?;

    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let joint = &profile.joints[request.selected_index];
    let assisted_temperature_baseline = if peak_phase_assistance_nm.is_some() {
        Some(joint1_first_position_temperature_baseline(
            profile,
            &initial_feedback,
        )?)
    } else {
        None
    };
    if peak_phase_assistance_nm.is_some() {
        anyhow::ensure!(
            request.selected_index == J3_GRAVITY_UNLOAD_INDEX
                && request.delta_rad.to_bits() == J3_ASSISTED_POSITION_DELTA_RAD.to_bits()
                && request.duration_sec.to_bits() == J3_ASSISTED_POSITION_DURATION_SEC.to_bits(),
            "phase assistance is restricted to the fixed joint_3 inward 5 mrad / 4 s survey"
        );
        anyhow::ensure!(
            (J3_GRAVITY_UNLOAD_INITIAL_Q_LOWER_RAD..=J3_GRAVITY_UNLOAD_INITIAL_Q_UPPER_RAD)
                .contains(&initial_q[request.selected_index]),
            "joint_3 assisted-position start {:.6} rad is outside [{:.6}, {:.6}] rad",
            initial_q[request.selected_index],
            J3_GRAVITY_UNLOAD_INITIAL_Q_LOWER_RAD,
            J3_GRAVITY_UNLOAD_INITIAL_Q_UPPER_RAD
        );
    }
    let end_position = initial_q[request.selected_index] + request.delta_rad;
    anyhow::ensure!(
        (joint.limits.position_lower_rad..=joint.limits.position_upper_rad).contains(&end_position),
        "{} requested endpoint {:.6} rad is outside [{:.6}, {:.6}] rad",
        joint.name,
        end_position,
        joint.limits.position_lower_rad,
        joint.limits.position_upper_rad
    );

    let initial_targets = build_safe_hold_targets(
        profile,
        dynamics,
        &initial_q,
        request.selected_index,
        initial_q[request.selected_index],
    )?;
    let assisted_active_started = if peak_phase_assistance_nm.is_some() {
        backend
            .register_single_axis_diagnostic_baseline(request.selected_index, initial_targets)
            .context("register joint_3 assisted-position zero-force baseline")?;
        backend
            .enable_joint3_gravity_unload_diagnostic_axis(initial_targets)
            .await
            .context("configure exact 30/20 caps and enable only joint_3")?;
        Some(Instant::now())
    } else {
        backend
            .enable_commissioning_axis(request.selected_index, initial_targets)
            .await
            .with_context(|| format!("enable only {} for commissioning", joint.name))?;
        None
    };

    tracing::warn!(
        joint = %joint.name,
        node_id = joint.node_id,
        delta_rad = request.delta_rad,
        duration_sec = request.duration_sec,
        "commissioning axis enabled; holding the starting pose until the post-enable stability gate passes; all other drives remain disabled"
    );

    // Enabling can expose a gravity-model or gain mismatch before the planned
    // trajectory begins. Keep commanding the pre-enable pose and require both
    // position and velocity to be continuously quiet before proceeding. This
    // phase uses a symmetric drift guard independent of the requested motion
    // direction; directional excursion semantics start with the trajectory.
    let (start_feedback, start_q, initial_targets) =
        wait_for_enabled_axis_stability(backend, profile, dynamics, request, initial_q, None)
            .await?;
    if let (Some(active_started), Some(temperature_baseline)) =
        (assisted_active_started, assisted_temperature_baseline)
    {
        validate_joint3_assisted_active_time(active_started, "enable stability")?;
        validate_six_axis_diagnostic_temperatures(
            profile,
            &start_feedback,
            temperature_baseline,
            "joint_3 assisted-position",
        )?;
    }
    let mut excursion =
        MeasuredExcursion::new(initial_q[request.selected_index], request.delta_rad);
    tracing::warn!(
        joint = %joint.name,
        node_id = joint.node_id,
        delta_rad = request.delta_rad,
        duration_sec = request.duration_sec,
        "single-axis commissioning motion started after the post-enable stability gate passed; all other drives remain disabled"
    );
    log_commissioning_telemetry(
        "start",
        PHASE_TRAJECTORY,
        joint,
        0.0,
        commissioning_telemetry_sample(
            joint,
            &start_feedback,
            request.selected_index,
            initial_q[request.selected_index],
            start_q[request.selected_index],
            initial_q[request.selected_index],
            initial_targets[request.selected_index],
        ),
    );

    let started_at = Instant::now();
    let duration = Duration::from_secs_f32(request.duration_sec);
    let mut telemetry_milestones = MotionTelemetryMilestones::default();
    loop {
        let elapsed = started_at.elapsed();
        if elapsed >= duration {
            break;
        }
        let normalized_time = elapsed.as_secs_f32() / request.duration_sec;
        let phase = round_trip_phase(normalized_time);
        let commanded_position = initial_q[request.selected_index] + phase * request.delta_rad;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during commissioning motion"
        );
        if let Some(active_started) = assisted_active_started {
            validate_joint3_assisted_active_time(active_started, "trajectory")?;
            backend
                .ensure_single_axis_commissioning_state(request.selected_index)
                .context("joint_3 assisted-position single-axis state failed during trajectory")?;
        }
        let feedback = backend.feedback();
        if let Some(temperature_baseline) = assisted_temperature_baseline {
            validate_six_axis_diagnostic_temperatures(
                profile,
                &feedback,
                temperature_baseline,
                "joint_3 assisted-position",
            )?;
        }
        let log_abort = |abort_stage, error: &anyhow::Error| {
            log_safety_abort_telemetry(
                abort_stage,
                PHASE_TRAJECTORY,
                normalized_time,
                error,
                profile,
                dynamics,
                &feedback,
                request.selected_index,
                initial_q[request.selected_index],
                commanded_position,
            );
        };
        let measured_q = result_with_safety_abort_hook(
            validate_feedback(
                profile,
                &feedback,
                Some((
                    request.selected_index,
                    commanded_position,
                    request.delta_rad,
                )),
            ),
            |error| log_abort("validate_feedback", error),
        )?;
        let mut targets = result_with_safety_abort_hook(
            build_safe_hold_targets(
                profile,
                dynamics,
                &measured_q,
                request.selected_index,
                commanded_position,
            ),
            |error| log_abort("build_safe_hold_targets", error),
        )?;
        if let Some(peak_assistance_nm) = peak_phase_assistance_nm {
            targets = apply_joint3_phase_assistance(
                profile,
                targets,
                commanded_position,
                phase * peak_assistance_nm,
            )?;
            let telemetry = commissioning_telemetry_sample(
                joint,
                &feedback,
                request.selected_index,
                initial_q[request.selected_index],
                measured_q[request.selected_index],
                commanded_position,
                targets[request.selected_index],
            );
            anyhow::ensure!(
                (telemetry.gravity_ff + telemetry.estimated_pd_torque).abs()
                    <= J3_ASSISTED_POSITION_MAX_TOTAL_TORQUE_NM,
                "joint_3 assisted-position estimated total torque {:.6} Nm exceeds {:.6} Nm",
                telemetry.gravity_ff + telemetry.estimated_pd_torque,
                J3_ASSISTED_POSITION_MAX_TOTAL_TORQUE_NM
            );
        }
        result_with_safety_abort_hook(
            excursion.observe(measured_q[request.selected_index]),
            |error| log_abort("excursion_guard", error),
        )?;
        backend.set_targets(targets).await?;
        while let Some(milestone) = telemetry_milestones.take_due(normalized_time) {
            log_commissioning_telemetry(
                milestone,
                PHASE_TRAJECTORY,
                joint,
                normalized_time,
                commissioning_telemetry_sample(
                    joint,
                    &feedback,
                    request.selected_index,
                    initial_q[request.selected_index],
                    measured_q[request.selected_index],
                    commanded_position,
                    targets[request.selected_index],
                ),
            );
        }

        tokio::time::sleep(LOOP_PERIOD).await;
    }

    // Explicitly command the exact starting pose until position and velocity
    // have both remained settled for a continuous dwell window. This may end
    // early, but can wait up to the fail-closed return timeout.
    let settle_started_at = Instant::now();
    let mut return_stability = ContinuousStabilityGate::new(STABILITY_DWELL, RETURN_SETTLE_TIMEOUT);
    let allowed_return_error = return_tolerance_rad(request.delta_rad);
    let (return_stable, final_q, final_feedback, final_targets, return_error, return_velocity) = loop {
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during commissioning return settle"
        );
        if let Some(active_started) = assisted_active_started {
            validate_joint3_assisted_active_time(active_started, "return settle")?;
            backend
                .ensure_single_axis_commissioning_state(request.selected_index)
                .context("joint_3 assisted-position single-axis state failed during return")?;
        }
        let feedback = backend.feedback();
        if let Some(temperature_baseline) = assisted_temperature_baseline {
            validate_six_axis_diagnostic_temperatures(
                profile,
                &feedback,
                temperature_baseline,
                "joint_3 assisted-position",
            )?;
        }
        let normalized_time =
            settle_started_at.elapsed().as_secs_f32() / RETURN_SETTLE_TIMEOUT.as_secs_f32();
        let log_abort = |abort_stage, error: &anyhow::Error| {
            log_safety_abort_telemetry(
                abort_stage,
                PHASE_RETURN_SETTLE,
                normalized_time,
                error,
                profile,
                dynamics,
                &feedback,
                request.selected_index,
                initial_q[request.selected_index],
                initial_q[request.selected_index],
            );
        };
        let measured_q = result_with_safety_abort_hook(
            validate_feedback(
                profile,
                &feedback,
                Some((
                    request.selected_index,
                    initial_q[request.selected_index],
                    request.delta_rad,
                )),
            ),
            |error| log_abort("validate_feedback", error),
        )?;
        let targets = result_with_safety_abort_hook(
            build_safe_hold_targets(
                profile,
                dynamics,
                &measured_q,
                request.selected_index,
                initial_q[request.selected_index],
            ),
            |error| log_abort("build_safe_hold_targets", error),
        )?;
        result_with_safety_abort_hook(
            excursion.observe(measured_q[request.selected_index]),
            |error| log_abort("excursion_guard", error),
        )?;
        backend.set_targets(targets).await?;

        let return_error =
            (measured_q[request.selected_index] - initial_q[request.selected_index]).abs();
        let return_velocity = motor_velocity_to_ros(
            feedback.joints[request.selected_index].velocity_rev_s,
            joint,
        );
        let elapsed = settle_started_at.elapsed();
        match return_stability.observe(
            elapsed,
            return_error <= allowed_return_error && return_velocity.abs() <= STABLE_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => {
                break (
                    true,
                    measured_q,
                    feedback,
                    targets,
                    return_error,
                    return_velocity,
                );
            }
            StabilityGateStatus::TimedOut => {
                break (
                    false,
                    measured_q,
                    feedback,
                    targets,
                    return_error,
                    return_velocity,
                );
            }
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    };
    log_commissioning_telemetry(
        "return_settle",
        PHASE_RETURN_SETTLE,
        joint,
        1.0,
        commissioning_telemetry_sample(
            joint,
            &final_feedback,
            request.selected_index,
            initial_q[request.selected_index],
            final_q[request.selected_index],
            initial_q[request.selected_index],
            final_targets[request.selected_index],
        ),
    );
    if !return_stable {
        anyhow::bail!(
            "{} did not settle at its starting pose within {:.3} s: position {:.6} rad, start {:.6} rad, error {:.6} rad (limit {:.6} rad), velocity {:.6} rad/s (limit {:.6} rad/s); both limits must hold continuously for {:.3} s",
            joint.name,
            RETURN_SETTLE_TIMEOUT.as_secs_f32(),
            final_q[request.selected_index],
            initial_q[request.selected_index],
            return_error,
            allowed_return_error,
            return_velocity,
            STABLE_VELOCITY_RAD_S,
            STABILITY_DWELL.as_secs_f32()
        );
    }
    let measured_peak_delta = excursion.validate_completed()?;
    tracing::info!(
        joint = %joint.name,
        measured_peak_delta_rad = measured_peak_delta,
        return_error_rad = return_error,
        "single-axis commissioning round trip completed"
    );
    Ok(())
}

fn apply_joint3_phase_assistance(
    profile: &HardwareProfile,
    mut targets: [MotorTarget; DOF],
    commanded_position_rad: f32,
    assistance_nm: f32,
) -> Result<[MotorTarget; DOF]> {
    anyhow::ensure!(
        assistance_nm.is_finite()
            && (J3_ASSISTED_POSITION_PEAK_TORQUE_NM..=0.0).contains(&assistance_nm),
        "joint_3 trajectory assistance {assistance_nm:.6} Nm is outside [{:.6}, 0] Nm",
        J3_ASSISTED_POSITION_PEAK_TORQUE_NM
    );
    let joint = &profile.joints[J3_GRAVITY_UNLOAD_INDEX];
    let model_feedforward_nm =
        motor_torque_to_ros(targets[J3_GRAVITY_UNLOAD_INDEX].torque_nm, joint);
    anyhow::ensure!(
        model_feedforward_nm.abs() <= 1.0e-4,
        "joint_3 assisted-position requires zero model feed-forward, got {model_feedforward_nm:.6} Nm"
    );
    let total_feedforward_nm = model_feedforward_nm + assistance_nm;
    anyhow::ensure!(
        total_feedforward_nm.abs() <= J3_ASSISTED_POSITION_PEAK_TORQUE_NM.abs(),
        "joint_3 assisted-position feed-forward {total_feedforward_nm:.6} Nm exceeds {:.6} Nm",
        J3_ASSISTED_POSITION_PEAK_TORQUE_NM.abs()
    );
    targets[J3_GRAVITY_UNLOAD_INDEX] = ros_target_to_motor(
        RosTarget {
            position_rad: commanded_position_rad,
            velocity_rad_s: 0.0,
            torque_nm: total_feedforward_nm,
            kp_nm_rad: J3_ASSISTED_POSITION_EXPECTED_KP,
            kd_nm_s_rad: J3_ASSISTED_POSITION_EXPECTED_KD,
        },
        joint,
    );
    Ok(targets)
}

fn validate_joint3_assisted_active_time(active_started: Instant, phase: &str) -> Result<()> {
    anyhow::ensure!(
        active_started.elapsed().as_secs_f32() <= J3_ASSISTED_POSITION_MAX_ACTIVE_SEC,
        "joint_3 assisted-position exceeded its {J3_ASSISTED_POSITION_MAX_ACTIVE_SEC:.3} s confirmed-active limit during {phase}"
    );
    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct DiagnosticTemperatureBaseline {
    driver_c: f32,
    motor_c: f32,
}

#[derive(Debug, Clone, Copy)]
struct SixAxisDiagnosticTemperatureBaseline {
    joints: [DiagnosticTemperatureBaseline; DOF],
}

#[derive(Debug, Clone, Copy)]
struct DiagnosticStabilityGuards {
    torque_limits: DiagnosticTorqueLimits,
    temperature_baseline: DiagnosticTemperatureBaseline,
    active_started: Instant,
    position_round_trip: bool,
    censored_gravity_hold: bool,
    selected_gravity_scale: Option<f32>,
}

#[derive(Debug, Clone, Copy)]
struct DiagnosticObservation {
    telemetry: CommissioningTelemetrySample,
    driver_temperature_c: f32,
    motor_temperature_c: f32,
    breakaway_detected: bool,
}

#[derive(Debug, Default, Clone, Copy)]
struct DiagnosticPeaks {
    position_delta_rad: f32,
    velocity_rad_s: f32,
    measured_torque_nm: f32,
    estimated_total_torque_nm: f32,
    driver_temperature_c: f32,
    motor_temperature_c: f32,
}

impl DiagnosticPeaks {
    fn observe(&mut self, observation: DiagnosticObservation) {
        let telemetry = observation.telemetry;
        self.position_delta_rad = self.position_delta_rad.max(telemetry.measured_delta.abs());
        self.velocity_rad_s = self.velocity_rad_s.max(telemetry.measured_velocity.abs());
        self.measured_torque_nm = self.measured_torque_nm.max(telemetry.measured_torque.abs());
        self.estimated_total_torque_nm = self
            .estimated_total_torque_nm
            .max((telemetry.gravity_ff + telemetry.estimated_pd_torque).abs());
        self.driver_temperature_c = self
            .driver_temperature_c
            .max(observation.driver_temperature_c);
        self.motor_temperature_c = self
            .motor_temperature_c
            .max(observation.motor_temperature_c);
    }
}

/// Keep signal/error cleanup one feedback confirmation behind the target
/// currently on the wire.  A prospective ramp target becomes the persistent
/// cleanup baseline only after a strictly newer six-axis feedback snapshot has
/// passed every guard while that target is published.
#[derive(Debug, Clone, Copy, PartialEq)]
struct PositionRampTargetProgress {
    published: [MotorTarget; DOF],
    verified: [MotorTarget; DOF],
}

impl PositionRampTargetProgress {
    fn new(published: [MotorTarget; DOF], verified: [MotorTarget; DOF]) -> Self {
        Self {
            published,
            verified,
        }
    }

    fn confirm_published(&mut self) -> [MotorTarget; DOF] {
        self.verified = self.published;
        self.verified
    }

    fn publish(&mut self, next: [MotorTarget; DOF]) {
        self.published = next;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DiagnosticLevelOutcome {
    Completed,
    BreakawayDetected,
}

fn requires_fast_diagnostic_restore(
    torque_limits: DiagnosticTorqueLimits,
    operation: &Result<DiagnosticLevelOutcome>,
) -> bool {
    torque_limits.high_tier && !matches!(operation, Ok(DiagnosticLevelOutcome::Completed))
}

/// Run the one compile-time-fixed J1 +5 mrad / 4 s position survey.
/// This is intentionally separate from the configurable J2 diagnostic enum:
/// it has its own authorization, profile locks, drive caps, dynamic window,
/// torque gates, and acceptance evidence, and accepts no motion parameters.
/// Its phase-shaped friction compensation is zero at every stationary point.
pub async fn run_joint1_first_position_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint1FirstPositionDiagnosticRequest,
) -> Result<()> {
    // Targets and feedback conversion below must use the identical profile
    // allocation that opened the backend; value-equal clones are rejected.
    // The separately supplied dynamics object is contained by the every-poll
    // |G1| <= 0.10 Nm and estimated-total <= 1.0 Nm gates, while all five
    // unselected axes remain in confirmed non-torque states.
    backend
        .ensure_joint1_first_position_strict_session(profile)
        .context("joint_1 fixed diagnostic backend/profile authority gate failed")?;
    request.validate(profile)?;
    let selected_index = request.selected_index();
    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let joint = &profile.joints[selected_index];
    let initial_position_rad = initial_q[selected_index];
    validate_joint1_first_position_dynamic_window(joint, initial_position_rad)?;
    let initial_motor_position_rev = initial_feedback.joints[selected_index].position_rev;
    let temperature_baseline =
        joint1_first_position_temperature_baseline(profile, &initial_feedback)?;
    let initial_targets = build_safe_hold_targets(
        profile,
        dynamics,
        &initial_q,
        selected_index,
        initial_position_rad,
    )?;
    joint1_first_position_observation(
        profile,
        &initial_feedback,
        selected_index,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        initial_targets[selected_index],
        initial_position_rad,
    )
    .context("pre-enable joint_1 first-position safety gate failed")?;

    // The baseline is outside the cancellable enable future. A signal during
    // configure, cap readback, enable, or motion can therefore re-publish it
    // before the selected-first confirmed shutdown begins.
    backend
        .register_single_axis_diagnostic_baseline(selected_index, initial_targets)
        .context("register joint_1 pre-enable persistent baseline")?;
    backend
        .enable_joint1_first_position_diagnostic_axis(initial_targets)
        .await
        .context("configure fixed 30/20 drive caps and enable only joint_1")?;
    // This timer starts only after MIT Operation Enabled and mode-display
    // confirmation returns. It bounds the host's confirmed-active diagnostic
    // window; it does not claim to time the lower-level set_mode handshake.
    let diagnostic_started = Instant::now();
    let mut peaks = DiagnosticPeaks::default();
    let stable_targets = wait_for_joint1_first_position_enable_stability(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, stable_targets)
        .context("advance joint_1 persistent baseline after the 250 ms GO dwell")?;

    run_joint1_first_position_round_trip(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        stable_targets,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    tracing::info!(
        phase = PHASE_J1_FIRST_POSITION_RETURN,
        joint = %joint.name,
        node_id = joint.node_id,
        fixed_delta_rad = J1_FIRST_POSITION_DELTA_RAD,
        fixed_duration_sec = J1_FIRST_POSITION_DURATION_SEC,
        drive_torque_permille = J1_FIRST_POSITION_TORQUE_PERMILLE,
        drive_kp_kd_torque_permille = J1_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        peak_position_delta_rad = peaks.position_delta_rad,
        peak_velocity_rad_s = peaks.velocity_rad_s,
        peak_measured_torque_nm = peaks.measured_torque_nm,
        peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
        peak_driver_temperature_c = peaks.driver_temperature_c,
        peak_motor_temperature_c = peaks.motor_temperature_c,
        elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
        "joint_1 fixed first-position survey completed; this result does not authorize a general jog or normal control"
    );
    Ok(())
}

/// Run the one compile-time-fixed J5 -5 mrad / 4 s survey toward zero.
/// It uses no friction/feed-forward experiment and cannot be widened into a
/// caller-selected jog.
pub async fn run_joint5_first_position_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint5FirstPositionDiagnosticRequest,
) -> Result<()> {
    backend
        .ensure_joint5_first_position_strict_session(profile)
        .context("joint_5 fixed diagnostic backend/profile authority gate failed")?;
    request.validate(profile)?;
    let selected_index = request.selected_index();
    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let joint = &profile.joints[selected_index];
    let initial_position_rad = initial_q[selected_index];
    validate_joint5_first_position_dynamic_window(joint, initial_position_rad)?;
    let initial_motor_position_rev = initial_feedback.joints[selected_index].position_rev;
    let temperature_baseline =
        joint1_first_position_temperature_baseline(profile, &initial_feedback)?;
    let initial_targets = build_safe_hold_targets(
        profile,
        dynamics,
        &initial_q,
        selected_index,
        initial_position_rad,
    )?;
    joint5_first_position_observation(
        profile,
        &initial_feedback,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        initial_targets[selected_index],
        initial_position_rad,
        J5_FIRST_POSITION_EXPECTED_KP,
        J5_FIRST_POSITION_EXPECTED_KD,
    )
    .context("pre-enable joint_5 first-position safety gate failed")?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, initial_targets)
        .context("register joint_5 pre-enable persistent baseline")?;
    backend
        .enable_joint5_first_position_diagnostic_axis(initial_targets)
        .await
        .context("configure fixed 50/30 drive caps and enable only joint_5")?;
    let diagnostic_started = Instant::now();
    let mut peaks = DiagnosticPeaks::default();
    let stable_targets = wait_for_joint5_first_position_enable_stability(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, stable_targets)
        .context("advance joint_5 persistent baseline after the 250 ms GO dwell")?;
    run_joint5_first_position_round_trip(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        stable_targets,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    tracing::info!(
        phase = PHASE_J5_FIRST_POSITION_RETURN,
        joint = %joint.name,
        node_id = joint.node_id,
        fixed_delta_rad = J5_FIRST_POSITION_DELTA_RAD,
        fixed_duration_sec = J5_FIRST_POSITION_DURATION_SEC,
        drive_torque_permille = J5_FIRST_POSITION_TORQUE_PERMILLE,
        drive_kp_kd_torque_permille = J5_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        peak_position_delta_rad = peaks.position_delta_rad,
        peak_velocity_rad_s = peaks.velocity_rad_s,
        peak_measured_torque_nm = peaks.measured_torque_nm,
        peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
        peak_driver_temperature_c = peaks.driver_temperature_c,
        peak_motor_temperature_c = peaks.motor_temperature_c,
        elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
        "joint_5 fixed first-position survey completed; this does not authorize general motion"
    );
    Ok(())
}

/// Run the compile-time-fixed J4 -5 mrad / 4 s position-channel survey with
/// gravity feed-forward held at its reviewed zero-scale value.
pub async fn run_joint4_first_position_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint4FirstPositionDiagnosticRequest,
) -> Result<()> {
    backend
        .ensure_joint4_first_position_strict_session(profile)
        .context("joint_4 fixed diagnostic backend/profile authority gate failed")?;
    request.validate(profile)?;
    let selected_index = request.selected_index();
    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let joint = &profile.joints[selected_index];
    let initial_position_rad = initial_q[selected_index];
    validate_joint4_first_position_dynamic_window(joint, initial_position_rad)?;
    let initial_motor_position_rev = initial_feedback.joints[selected_index].position_rev;
    let temperature_baseline =
        joint1_first_position_temperature_baseline(profile, &initial_feedback)?;
    let initial_targets = build_safe_hold_targets(
        profile,
        dynamics,
        &initial_q,
        selected_index,
        initial_position_rad,
    )?;
    joint4_first_position_observation(
        profile,
        &initial_feedback,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        initial_targets[selected_index],
        initial_position_rad,
        J4_FIRST_POSITION_EXPECTED_KP,
        J4_FIRST_POSITION_EXPECTED_KD,
    )
    .context("pre-enable joint_4 first-position safety gate failed")?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, initial_targets)
        .context("register joint_4 pre-enable persistent baseline")?;
    backend
        .enable_joint4_first_position_diagnostic_axis(initial_targets)
        .await
        .context("configure fixed 60/50 drive caps and enable only joint_4")?;
    let diagnostic_started = Instant::now();
    let mut peaks = DiagnosticPeaks::default();
    let stable_targets = wait_for_joint4_first_position_enable_stability(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, stable_targets)
        .context("advance joint_4 persistent baseline after the 250 ms GO dwell")?;
    run_joint4_first_position_round_trip(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        stable_targets,
        diagnostic_started,
        &mut peaks,
        0.0,
    )
    .await?;
    tracing::info!(
        phase = PHASE_J4_FIRST_POSITION_RETURN,
        joint = %joint.name,
        node_id = joint.node_id,
        fixed_delta_rad = J4_FIRST_POSITION_DELTA_RAD,
        fixed_duration_sec = J4_FIRST_POSITION_DURATION_SEC,
        drive_torque_permille = J4_FIRST_POSITION_TORQUE_PERMILLE,
        drive_kp_kd_torque_permille = J4_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        peak_position_delta_rad = peaks.position_delta_rad,
        peak_velocity_rad_s = peaks.velocity_rad_s,
        peak_measured_torque_nm = peaks.measured_torque_nm,
        peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
        peak_driver_temperature_c = peaks.driver_temperature_c,
        peak_motor_temperature_c = peaks.motor_temperature_c,
        elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
        "joint_4 fixed first-position survey completed; gravity scale remains uncommissioned"
    );
    Ok(())
}

/// Run the independently authorized J4 survey whose negative assistance is
/// zero at the start pose, follows the same half-cosine phase as the fixed
/// -5 mrad target, and returns to zero before the baseline settle. This avoids
/// a fixed-position torque step and cannot construct any other trajectory.
pub async fn run_joint4_assisted_position_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint4AssistedPositionDiagnosticRequest,
) -> Result<()> {
    backend
        .ensure_joint4_first_position_strict_session(profile)
        .context("joint_4 assisted-position backend/profile authority gate failed")?;
    request.validate(profile)?;
    let selected_index = request.selected_index();
    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let joint = &profile.joints[selected_index];
    let initial_position_rad = initial_q[selected_index];
    validate_joint4_first_position_dynamic_window(joint, initial_position_rad)?;
    let initial_motor_position_rev = initial_feedback.joints[selected_index].position_rev;
    let temperature_baseline =
        joint1_first_position_temperature_baseline(profile, &initial_feedback)?;
    let initial_targets = build_safe_hold_targets(
        profile,
        dynamics,
        &initial_q,
        selected_index,
        initial_position_rad,
    )?;
    joint4_first_position_observation(
        profile,
        &initial_feedback,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        initial_targets[selected_index],
        initial_position_rad,
        J4_FIRST_POSITION_EXPECTED_KP,
        J4_FIRST_POSITION_EXPECTED_KD,
    )
    .context("pre-enable joint_4 assisted-position safety gate failed")?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, initial_targets)
        .context("register joint_4 assisted-position pre-enable persistent baseline")?;
    backend
        .enable_joint4_assisted_position_diagnostic_axis(initial_targets)
        .await
        .context("configure fixed 90/50 drive caps and enable only joint_4")?;
    let diagnostic_started = Instant::now();
    let mut peaks = DiagnosticPeaks::default();
    let stable_targets = wait_for_joint4_first_position_enable_stability(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, stable_targets)
        .context("advance joint_4 assisted-position baseline after the 250 ms GO dwell")?;
    run_joint4_first_position_round_trip(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        stable_targets,
        diagnostic_started,
        &mut peaks,
        J4_ASSISTED_POSITION_PEAK_TORQUE_NM,
    )
    .await?;
    tracing::info!(
        phase = PHASE_J4_ASSISTED_POSITION_TRAJECTORY,
        joint = %joint.name,
        node_id = joint.node_id,
        fixed_delta_rad = J4_FIRST_POSITION_DELTA_RAD,
        fixed_duration_sec = J4_FIRST_POSITION_DURATION_SEC,
        peak_assistance_nm = J4_ASSISTED_POSITION_PEAK_TORQUE_NM,
        drive_torque_permille = J4_ASSISTED_POSITION_TORQUE_PERMILLE,
        drive_kp_kd_torque_permille = J4_ASSISTED_POSITION_KP_KD_TORQUE_PERMILLE,
        peak_position_delta_rad = peaks.position_delta_rad,
        peak_velocity_rad_s = peaks.velocity_rad_s,
        peak_measured_torque_nm = peaks.measured_torque_nm,
        peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
        peak_driver_temperature_c = peaks.driver_temperature_c,
        peak_motor_temperature_c = peaks.motor_temperature_c,
        elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
        "joint_4 trajectory-synchronous assisted survey completed; no gravity scale was promoted"
    );
    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct Joint3VerifiedGravityTarget {
    level: usize,
    feedforward_nm: f32,
    targets: [MotorTarget; DOF],
}

#[derive(Debug, Clone, Copy)]
enum Joint3GravityUnloadStop {
    MotionCensor {
        rejected_level: usize,
        rejected_feedforward_nm: f32,
        trigger_position_delta_rad: f32,
        trigger_velocity_rad_s: f32,
    },
    TorqueCap,
}

impl Joint3GravityUnloadStop {
    const fn label(self) -> &'static str {
        match self {
            Self::MotionCensor { .. } => "inward_motion_censor",
            Self::TorqueCap => "right_censored_torque_cap",
        }
    }
}

/// Unload J3 from its surveyed upper-limit pose without constructing a
/// position trajectory. The selected position code remains fixed while
/// negative joint-side feed-forward advances in feedback-proven 0.05 Nm
/// levels. The first inward response rejects its current level and freezes the
/// preceding wire-distinct target for exact readback and bounded statistics.
/// If the first level itself crosses the censor, it is logged and the
/// registered zero-feed-forward baseline is restored immediately.
pub async fn run_joint3_gravity_unload_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint3GravityUnloadDiagnosticRequest,
) -> Result<()> {
    backend
        .ensure_joint3_gravity_unload_strict_session(profile)
        .context("joint_3 gravity-unload backend/profile authority gate failed")?;
    request.validate(profile)?;
    let selected_index = request.selected_index();
    let joint = &profile.joints[selected_index];
    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let initial_position_rad = initial_q[selected_index];
    anyhow::ensure!(
        (J3_GRAVITY_UNLOAD_INITIAL_Q_LOWER_RAD..=J3_GRAVITY_UNLOAD_INITIAL_Q_UPPER_RAD)
            .contains(&initial_position_rad),
        "joint_3 gravity-unload q0={initial_position_rad:.6} rad is outside the reviewed near-upper interval [{J3_GRAVITY_UNLOAD_INITIAL_Q_LOWER_RAD:.6}, {J3_GRAVITY_UNLOAD_INITIAL_Q_UPPER_RAD:.6}]"
    );
    let initial_motor_position_rev = initial_feedback.joints[selected_index].position_rev;
    let temperature_baseline =
        joint1_first_position_temperature_baseline(profile, &initial_feedback)?;
    let initial_targets = build_safe_hold_targets(
        profile,
        dynamics,
        &initial_q,
        selected_index,
        initial_position_rad,
    )?;
    joint3_gravity_unload_observation(
        profile,
        &initial_feedback,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        initial_targets[selected_index],
    )
    .context("pre-enable joint_3 gravity-unload safety gate failed")?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, initial_targets)
        .context("register joint_3 zero-feed-forward pre-enable baseline")?;
    backend
        .enable_joint3_gravity_unload_diagnostic_axis(initial_targets)
        .await
        .context("configure fixed 30/20 caps and enable only joint_3")?;
    let diagnostic_started = Instant::now();
    let mut peaks = DiagnosticPeaks::default();
    let stable_targets = wait_for_joint3_gravity_unload_enable_stability(
        backend,
        profile,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        initial_targets,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, stable_targets)
        .context("register joint_3 post-enable zero-feed-forward baseline")?;
    run_joint3_gravity_unload_ramp(
        backend,
        profile,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        stable_targets,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    tracing::info!(
        phase = PHASE_J3_GRAVITY_UNLOAD,
        joint = %joint.name,
        node_id = joint.node_id,
        torque_step_nm = J3_GRAVITY_UNLOAD_STEP_NM,
        torque_cap_nm = -J3_GRAVITY_UNLOAD_CAP_NM,
        peak_position_delta_rad = peaks.position_delta_rad,
        peak_velocity_rad_s = peaks.velocity_rad_s,
        peak_measured_torque_nm = peaks.measured_torque_nm,
        peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
        peak_driver_temperature_c = peaks.driver_temperature_c,
        peak_motor_temperature_c = peaks.motor_temperature_c,
        elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
        "joint_3 fixed-position gravity-unload identification completed; no position trajectory was constructed"
    );
    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct Joint4VerifiedTorqueTarget {
    level: usize,
    additive_torque_nm: f32,
    targets: [MotorTarget; DOF],
}

#[derive(Debug, Clone, Copy)]
enum Joint4CensoredTorqueStop {
    DisplacementCensor {
        rejected_level: usize,
        rejected_additive_torque_nm: f32,
        trigger_position_delta_rad: f32,
        trigger_velocity_rad_s: f32,
    },
    TorqueCap,
}

impl Joint4CensoredTorqueStop {
    const fn label(self) -> &'static str {
        match self {
            Self::DisplacementCensor { .. } => "negative_displacement_censor",
            Self::TorqueCap => "right_censored_torque_cap",
        }
    }
}

/// Raise only negative joint-side feed-forward while holding J4's initial
/// position code fixed. The first target observed at -0.30 mrad is rejected;
/// the preceding all-axis-feedback-proven target is frozen for exact readback
/// and one second of statistics. This path never constructs a position
/// trajectory.
pub async fn run_joint4_censored_torque_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint4CensoredTorqueDiagnosticRequest,
) -> Result<()> {
    backend
        .ensure_joint4_first_position_strict_session(profile)
        .context("joint_4 censored-torque backend/profile authority gate failed")?;
    request.validate(profile)?;
    let selected_index = J4_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let initial_position_rad = initial_q[selected_index];
    validate_joint4_first_position_dynamic_window(joint, initial_position_rad)?;
    let initial_motor_position_rev = initial_feedback.joints[selected_index].position_rev;
    let temperature_baseline =
        joint1_first_position_temperature_baseline(profile, &initial_feedback)?;
    let initial_targets = build_safe_hold_targets(
        profile,
        dynamics,
        &initial_q,
        selected_index,
        initial_position_rad,
    )?;
    joint4_censored_torque_observation(
        profile,
        &initial_feedback,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        initial_targets[selected_index],
    )
    .context("pre-enable joint_4 censored-torque safety gate failed")?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, initial_targets)
        .context("register joint_4 censored-torque pre-enable baseline")?;
    backend
        .enable_joint4_first_position_diagnostic_axis(initial_targets)
        .await
        .context("configure fixed 60/50 drive caps and enable only joint_4")?;
    let diagnostic_started = Instant::now();
    let mut peaks = DiagnosticPeaks::default();
    let stable_targets = wait_for_joint4_first_position_enable_stability(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, stable_targets)
        .context("register joint_4 censored-torque post-stability baseline")?;
    let model_feedforward_nm = motor_torque_to_ros(stable_targets[selected_index].torque_nm, joint);
    anyhow::ensure!(
        model_feedforward_nm.abs() <= J4_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        "joint_4 censored-torque baseline {model_feedforward_nm:.6} Nm is not the required zero-gravity hold"
    );
    run_joint4_censored_torque_ramp(
        backend,
        profile,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        stable_targets,
        model_feedforward_nm,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    tracing::info!(
        phase = PHASE_J4_CENSORED_TORQUE,
        joint = %joint.name,
        node_id = joint.node_id,
        torque_step_nm = J4_CENSORED_TORQUE_STEP_NM,
        torque_cap_nm = J4_CENSORED_TORQUE_CAP_NM,
        peak_position_delta_rad = peaks.position_delta_rad,
        peak_velocity_rad_s = peaks.velocity_rad_s,
        peak_measured_torque_nm = peaks.measured_torque_nm,
        peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
        peak_driver_temperature_c = peaks.driver_temperature_c,
        peak_motor_temperature_c = peaks.motor_temperature_c,
        elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
        "joint_4 fixed-position censored-torque identification completed; no position trajectory was constructed"
    );
    Ok(())
}

/// Run the compile-time-fixed J6 -5 mrad / 4 s first-motion survey toward
/// zero. It uses the already bounded small-axis policy and cannot be widened
/// into a caller-selected jog.
pub async fn run_joint6_first_position_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint6FirstPositionDiagnosticRequest,
) -> Result<()> {
    backend
        .ensure_joint6_first_position_strict_session(profile)
        .context("joint_6 fixed diagnostic backend/profile authority gate failed")?;
    request.validate(profile)?;
    let selected_index = request.selected_index();
    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let joint = &profile.joints[selected_index];
    let initial_position_rad = initial_q[selected_index];
    validate_joint6_first_position_dynamic_window(joint, initial_position_rad)?;
    let initial_motor_position_rev = initial_feedback.joints[selected_index].position_rev;
    let temperature_baseline =
        joint1_first_position_temperature_baseline(profile, &initial_feedback)?;
    let initial_targets = build_safe_hold_targets(
        profile,
        dynamics,
        &initial_q,
        selected_index,
        initial_position_rad,
    )?;
    joint6_first_position_observation(
        profile,
        &initial_feedback,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        initial_targets[selected_index],
        initial_position_rad,
        J6_FIRST_POSITION_EXPECTED_KP,
        J6_FIRST_POSITION_EXPECTED_KD,
    )
    .context("pre-enable joint_6 first-position safety gate failed")?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, initial_targets)
        .context("register joint_6 pre-enable persistent baseline")?;
    backend
        .enable_joint6_first_position_diagnostic_axis(initial_targets)
        .await
        .context("configure fixed 50/30 drive caps and enable only joint_6")?;
    let diagnostic_started = Instant::now();
    let mut peaks = DiagnosticPeaks::default();
    let stable_targets = wait_for_joint6_first_position_enable_stability(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, stable_targets)
        .context("advance joint_6 persistent baseline after the 250 ms GO dwell")?;
    run_joint6_first_position_round_trip(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        stable_targets,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    tracing::info!(
        phase = PHASE_J6_FIRST_POSITION_RETURN,
        joint = %joint.name,
        node_id = joint.node_id,
        fixed_delta_rad = J6_FIRST_POSITION_DELTA_RAD,
        fixed_duration_sec = J6_FIRST_POSITION_DURATION_SEC,
        drive_torque_permille = J6_FIRST_POSITION_TORQUE_PERMILLE,
        drive_kp_kd_torque_permille = J6_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        peak_position_delta_rad = peaks.position_delta_rad,
        peak_velocity_rad_s = peaks.velocity_rad_s,
        peak_measured_torque_nm = peaks.measured_torque_nm,
        peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
        peak_driver_temperature_c = peaks.driver_temperature_c,
        peak_motor_temperature_c = peaks.motor_temperature_c,
        elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
        "joint_6 fixed first-position survey completed; this does not authorize general motion"
    );
    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct Joint1VerifiedTorqueTarget {
    level: usize,
    additive_torque_nm: f32,
    targets: [MotorTarget; DOF],
}

#[derive(Debug, Clone, Copy)]
enum Joint1CensoredTorqueStop {
    DisplacementCensor {
        rejected_level: usize,
        rejected_additive_torque_nm: f32,
        trigger_position_delta_rad: f32,
        trigger_velocity_rad_s: f32,
    },
    TorqueCap,
}

impl Joint1CensoredTorqueStop {
    const fn label(self) -> &'static str {
        match self {
            Self::DisplacementCensor { .. } => "positive_displacement_censor",
            Self::TorqueCap => "right_censored_torque_cap",
        }
    }
}

/// Raise only positive joint-side feed-forward while keeping J1's compressed
/// position code fixed. The first target observed at +0.30 mrad is rejected;
/// the preceding nonzero target is eligible for a one-second frozen hold only
/// after an all-six-axis TPDO1 snapshot proved it for the complete 100 ms
/// level. This function contains no position-trajectory branch.
pub async fn run_joint1_censored_torque_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint1CensoredTorqueDiagnosticRequest,
) -> Result<()> {
    backend
        .ensure_joint1_first_position_strict_session(profile)
        .context("joint_1 censored-torque backend/profile authority gate failed")?;
    request.validate(profile)?;
    run_joint1_directional_censored_torque_diagnostic(
        backend,
        profile,
        dynamics,
        Joint1CensoredTorqueDirection::Positive,
    )
    .await
}

/// Mirror-image fixed-position identification for negative joint-side torque.
/// It has a distinct public authorization and cannot be selected by the
/// positive request above.
pub async fn run_joint1_negative_censored_torque_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: Joint1NegativeCensoredTorqueDiagnosticRequest,
) -> Result<()> {
    backend
        .ensure_joint1_first_position_strict_session(profile)
        .context("joint_1 negative censored-torque backend/profile authority gate failed")?;
    request.validate(profile)?;
    run_joint1_directional_censored_torque_diagnostic(
        backend,
        profile,
        dynamics,
        Joint1CensoredTorqueDirection::Negative,
    )
    .await
}

async fn run_joint1_directional_censored_torque_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    direction: Joint1CensoredTorqueDirection,
) -> Result<()> {
    let selected_index = J1_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let initial_position_rad = initial_q[selected_index];
    validate_joint1_first_position_dynamic_window(joint, initial_position_rad)?;
    let initial_motor_position_rev = initial_feedback.joints[selected_index].position_rev;
    let temperature_baseline =
        joint1_first_position_temperature_baseline(profile, &initial_feedback)?;
    let initial_targets = build_safe_hold_targets(
        profile,
        dynamics,
        &initial_q,
        selected_index,
        initial_position_rad,
    )?;
    let initial_observation = joint1_first_position_observation(
        profile,
        &initial_feedback,
        selected_index,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        initial_targets[selected_index],
        initial_position_rad,
    )
    .context("pre-enable joint_1 censored-torque safety gate failed")?;
    anyhow::ensure!(
        initial_observation.telemetry.gravity_ff.abs() <= J1_FIRST_POSITION_MAX_GRAVITY_NM,
        "joint_1 censored-torque model baseline {:.6} Nm exceeds {:.6} Nm",
        initial_observation.telemetry.gravity_ff,
        J1_FIRST_POSITION_MAX_GRAVITY_NM
    );

    backend
        .register_single_axis_diagnostic_baseline(selected_index, initial_targets)
        .context("register joint_1 censored-torque pre-enable baseline")?;
    backend
        .enable_joint1_first_position_diagnostic_axis(initial_targets)
        .await
        .context("configure fixed 30/20 drive caps and enable only joint_1")?;
    let diagnostic_started = Instant::now();
    let mut peaks = DiagnosticPeaks::default();
    let stable_targets = wait_for_joint1_first_position_enable_stability(
        backend,
        profile,
        dynamics,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    backend
        .register_single_axis_diagnostic_baseline(selected_index, stable_targets)
        .context("register joint_1 censored-torque post-stability baseline")?;

    let model_feedforward_nm = motor_torque_to_ros(stable_targets[selected_index].torque_nm, joint);
    anyhow::ensure!(
        model_feedforward_nm.abs() <= J1_FIRST_POSITION_MAX_GRAVITY_NM,
        "joint_1 censored-torque stable model baseline {model_feedforward_nm:.6} Nm exceeds {J1_FIRST_POSITION_MAX_GRAVITY_NM:.6} Nm"
    );
    run_joint1_censored_torque_ramp(
        backend,
        profile,
        &initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        stable_targets,
        model_feedforward_nm,
        direction,
        diagnostic_started,
        &mut peaks,
    )
    .await?;
    tracing::info!(
        phase = PHASE_J1_CENSORED_TORQUE,
        joint = %joint.name,
        node_id = joint.node_id,
        torque_direction = direction.label(),
        torque_step_nm = J1_CENSORED_TORQUE_STEP_NM,
        torque_cap_nm = J1_CENSORED_TORQUE_CAP_NM,
        peak_position_delta_rad = peaks.position_delta_rad,
        peak_velocity_rad_s = peaks.velocity_rad_s,
        peak_measured_torque_nm = peaks.measured_torque_nm,
        peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
        peak_driver_temperature_c = peaks.driver_temperature_c,
        peak_motor_temperature_c = peaks.motor_temperature_c,
        elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
        "joint_1 fixed-position censored-torque identification completed; no position trajectory was constructed"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn run_joint1_censored_torque_ramp(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    model_feedforward_nm: f32,
    direction: Joint1CensoredTorqueDirection,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let selected_index = J1_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let fixed_position_code =
        compressed_target_position_code(&baseline_targets[selected_index], joint);
    let baseline_words = compressed_target_words(&baseline_targets[selected_index], joint);
    let mut previous_feedback_at = backend
        .feedback()
        .oldest_tpdo1_at
        .context("joint_1 censored-torque ramp requires an initial all-axis TPDO1 timestamp")?;
    let mut verified_nonzero: Option<Joint1VerifiedTorqueTarget> = None;
    let mut displacement_milestones = CensoredGravityHoldDisplacementMilestones::default();

    for level in 1..=J1_CENSORED_TORQUE_LEVELS {
        validate_joint1_censored_torque_active_time(diagnostic_started, "torque staircase")?;
        let additive_torque_nm = joint1_censored_torque_additive(level, direction)?;
        let targets = build_joint1_censored_torque_targets(
            profile,
            baseline_targets,
            initial_q[selected_index],
            model_feedforward_nm,
            additive_torque_nm,
            direction,
        )?;
        let target_words = compressed_target_words(&targets[selected_index], joint);
        anyhow::ensure!(
            compressed_target_position_code(&targets[selected_index], joint) == fixed_position_code,
            "joint_1 censored-torque level changed the fixed position code"
        );
        let prior_words = verified_nonzero
            .map(|verified| compressed_target_words(&verified.targets[selected_index], joint))
            .unwrap_or(baseline_words);
        anyhow::ensure!(
            target_words != prior_words,
            "joint_1 censored-torque level {level} did not produce a wire-distinct target"
        );

        let prospective_feedback = backend.feedback();
        let prospective = joint1_censored_torque_observation(
            profile,
            &prospective_feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
            direction,
        )?;
        peaks.observe(prospective);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_1 torque publish")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed before joint_1 censored-torque publish"
        );
        backend.set_targets(targets).await?;
        let published_at = Instant::now();
        let proof_after = published_at + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
        let level_deadline = published_at + J1_CENSORED_TORQUE_DWELL;
        let mut feedback_proved = false;
        let mut latest_feedback_at = previous_feedback_at;

        let latest_observation = loop {
            validate_joint1_censored_torque_active_time(diagnostic_started, "torque-level dwell")?;
            anyhow::ensure!(
                !backend.transport_failed(),
                "CAN transport failed during joint_1 censored-torque dwell"
            );
            backend
                .ensure_single_axis_commissioning_state(selected_index)
                .context("six-axis state contract failed during joint_1 censored-torque dwell")?;
            let feedback = backend.feedback();
            let observation = joint1_censored_torque_observation(
                profile,
                &feedback,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                targets[selected_index],
                direction,
            )?;
            peaks.observe(observation);
            while let Some((threshold_rad, milestone)) = displacement_milestones
                .take_due(direction.directional_delta(observation.telemetry.measured_delta))
            {
                tracing::info!(
                    phase = PHASE_J1_CENSORED_TORQUE,
                    milestone,
                    joint = %joint.name,
                    node_id = joint.node_id,
                    torque_direction = direction.label(),
                    level,
                    additive_torque_nm,
                    threshold_rad,
                    measured_delta_rad = observation.telemetry.measured_delta,
                    measured_velocity_rad_s = observation.telemetry.measured_velocity,
                    feedforward_nm = observation.telemetry.gravity_ff,
                    estimated_pd_torque_nm = observation.telemetry.estimated_pd_torque,
                    measured_torque_nm = observation.telemetry.measured_torque,
                    "joint_1 censored-torque displacement milestone crossed"
                );
            }
            if joint1_censored_torque_should_censor(observation, direction) {
                let frozen = verified_nonzero.context(
                    "joint_1 reached its directional 0.30 mrad censor before any nonzero wire-distinct torque level completed its feedback-proven dwell",
                )?;
                let frozen_words = compressed_target_words(&frozen.targets[selected_index], joint);
                anyhow::ensure!(
                    frozen_words != target_words && frozen_words != baseline_words,
                    "joint_1 censored-torque rollback is not wire-distinct from both rejected target and zero-additive baseline"
                );
                backend
                    .register_single_axis_diagnostic_baseline(selected_index, frozen.targets)
                    .context("register preceding verified joint_1 torque target before rollback")?;
                backend
                    .set_targets(frozen.targets)
                    .await
                    .context("restore preceding verified joint_1 torque target")?;
                let freeze_started = Instant::now();
                tracing::warn!(
                    phase = PHASE_J1_CENSORED_TORQUE,
                    joint = %joint.name,
                    node_id = joint.node_id,
                    torque_direction = direction.label(),
                    rejected_level = level,
                    rejected_additive_torque_nm = additive_torque_nm,
                    trigger_position_delta_rad = observation.telemetry.measured_delta,
                    trigger_velocity_rad_s = observation.telemetry.measured_velocity,
                    frozen_verified_level = frozen.level,
                    frozen_verified_additive_torque_nm = frozen.additive_torque_nm,
                    "joint_1 displacement censor rejected the current torque and restored the preceding feedback-proven target"
                );
                return run_joint1_censored_torque_frozen_hold(
                    backend,
                    profile,
                    initial_q,
                    initial_motor_position_rev,
                    temperature_baseline,
                    baseline_targets,
                    frozen,
                    direction,
                    Joint1CensoredTorqueStop::DisplacementCensor {
                        rejected_level: level,
                        rejected_additive_torque_nm: additive_torque_nm,
                        trigger_position_delta_rad: observation.telemetry.measured_delta,
                        trigger_velocity_rad_s: observation.telemetry.measured_velocity,
                    },
                    freeze_started,
                    diagnostic_started,
                    peaks,
                )
                .await;
            }

            if let Some(feedback_at) = feedback.oldest_tpdo1_at {
                if feedback_at > previous_feedback_at && feedback_at > proof_after {
                    feedback_proved = true;
                    latest_feedback_at = feedback_at;
                }
            }
            if Instant::now() >= level_deadline && feedback_proved {
                break observation;
            }
            tokio::time::sleep(LOOP_PERIOD).await;
        };

        previous_feedback_at = latest_feedback_at;
        let verified = Joint1VerifiedTorqueTarget {
            level,
            additive_torque_nm,
            targets,
        };
        backend
            .register_single_axis_diagnostic_baseline(selected_index, targets)
            .context("promote completed feedback-proven joint_1 torque level")?;
        verified_nonzero = Some(verified);
        log_diagnostic_telemetry(
            "level_feedback_proven",
            PHASE_J1_CENSORED_TORQUE,
            joint,
            level,
            J1_CENSORED_TORQUE_LEVELS,
            model_feedforward_nm,
            additive_torque_nm,
            diagnostic_started.elapsed(),
            latest_observation,
        );

        if level == J1_CENSORED_TORQUE_LEVELS {
            backend.set_targets(targets).await?;
            let freeze_started = Instant::now();
            return run_joint1_censored_torque_frozen_hold(
                backend,
                profile,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                baseline_targets,
                verified,
                direction,
                Joint1CensoredTorqueStop::TorqueCap,
                freeze_started,
                diagnostic_started,
                peaks,
            )
            .await;
        }
    }
    unreachable!("fixed joint_1 censored-torque levels always return at their cap")
}

#[allow(clippy::too_many_arguments)]
async fn run_joint1_censored_torque_frozen_hold(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    frozen: Joint1VerifiedTorqueTarget,
    direction: Joint1CensoredTorqueDirection,
    stop: Joint1CensoredTorqueStop,
    freeze_started: Instant,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let selected_index = J1_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let freeze_deadline = freeze_started + J1_CENSORED_TORQUE_FREEZE_DURATION;
    anyhow::ensure!(
        compressed_target_position_code(&frozen.targets[selected_index], joint)
            == compressed_target_position_code(&baseline_targets[selected_index], joint),
        "joint_1 frozen torque target changed the fixed position code"
    );
    anyhow::ensure!(
        compressed_target_words(&frozen.targets[selected_index], joint)
            != compressed_target_words(&baseline_targets[selected_index], joint),
        "joint_1 frozen torque target is not wire-distinct from baseline"
    );

    confirm_joint1_censored_torque_target_readback_while_guarded(
        backend,
        profile,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        frozen.targets[selected_index],
        direction,
        freeze_deadline,
        diagnostic_started,
        peaks,
    )
    .await?;

    let mut statistics = CensoredHoldStatistics::default();
    let mut last_feedback_at: Option<Instant> = None;
    while Instant::now() < freeze_deadline {
        validate_joint1_censored_torque_active_time(diagnostic_started, "frozen statistics")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_1 frozen torque statistics"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_1 frozen torque statistics")?;
        let feedback = backend.feedback();
        let observation = joint1_censored_torque_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            frozen.targets[selected_index],
            direction,
        )?;
        peaks.observe(observation);
        let feedback_at = feedback
            .oldest_tpdo1_at
            .context("joint_1 frozen torque statistics require all-axis TPDO1 timestamps")?;
        if last_feedback_at.is_none_or(|previous| feedback_at > previous) {
            statistics.observe(feedback_at, observation)?;
            last_feedback_at = Some(feedback_at);
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let summary = statistics.summarize()?;
    let (
        rejected_level,
        rejected_additive_torque_nm,
        trigger_position_delta_rad,
        trigger_velocity_rad_s,
    ) = match stop {
        Joint1CensoredTorqueStop::DisplacementCensor {
            rejected_level,
            rejected_additive_torque_nm,
            trigger_position_delta_rad,
            trigger_velocity_rad_s,
        } => (
            Some(rejected_level),
            Some(rejected_additive_torque_nm),
            Some(trigger_position_delta_rad),
            Some(trigger_velocity_rad_s),
        ),
        Joint1CensoredTorqueStop::TorqueCap => (None, None, None, None),
    };
    tracing::info!(
        phase = PHASE_J1_CENSORED_TORQUE,
        joint = %joint.name,
        node_id = joint.node_id,
        torque_direction = direction.label(),
        stop_reason = stop.label(),
        rejected_level = ?rejected_level,
        rejected_additive_torque_nm = ?rejected_additive_torque_nm,
        trigger_position_delta_rad = ?trigger_position_delta_rad,
        trigger_velocity_rad_s = ?trigger_velocity_rad_s,
        frozen_verified_level = frozen.level,
        frozen_verified_additive_torque_nm = frozen.additive_torque_nm,
        sample_count = summary.sample_count,
        mean_position_delta_rad = summary.mean_position_delta_rad,
        position_stddev_rad = summary.position_stddev_rad,
        minimum_position_delta_rad = summary.minimum_position_delta_rad,
        maximum_position_delta_rad = summary.maximum_position_delta_rad,
        peak_velocity_rad_s = summary.peak_velocity_rad_s,
        mean_measured_torque_nm = summary.mean_measured_torque_nm,
        mean_estimated_total_torque_nm = summary.mean_estimated_total_torque_nm,
        peak_driver_temperature_c = summary.peak_driver_temperature_c,
        peak_motor_temperature_c = summary.peak_motor_temperature_c,
        terminal_sample_count = summary.terminal_sample_count,
        terminal_position_span_rad = summary.terminal_position_span_rad,
        terminal_peak_velocity_rad_s = summary.terminal_peak_velocity_rad_s,
        terminal_stable = summary.terminal_stable,
        "joint_1 fixed-position censored-torque statistics completed"
    );
    anyhow::ensure!(
        summary.terminal_stable,
        "joint_1 frozen torque target was not terminally stable: final span {:.6} rad, peak velocity {:.6} rad/s, samples {}",
        summary.terminal_position_span_rad,
        summary.terminal_peak_velocity_rad_s,
        summary.terminal_sample_count
    );
    Ok(())
}

fn build_joint1_censored_torque_targets(
    profile: &HardwareProfile,
    mut baseline_targets: [MotorTarget; DOF],
    hold_position_rad: f32,
    model_feedforward_nm: f32,
    additive_torque_nm: f32,
    direction: Joint1CensoredTorqueDirection,
) -> Result<[MotorTarget; DOF]> {
    anyhow::ensure!(
        additive_torque_nm.is_finite()
            && additive_torque_nm.signum() == direction.sign()
            && (J1_CENSORED_TORQUE_STEP_NM..=J1_CENSORED_TORQUE_CAP_NM)
                .contains(&additive_torque_nm.abs()),
        "joint_1 {} additive torque magnitude must remain in [{J1_CENSORED_TORQUE_STEP_NM:.3}, {J1_CENSORED_TORQUE_CAP_NM:.3}] Nm",
        direction.label()
    );
    anyhow::ensure!(
        model_feedforward_nm.abs() <= J1_FIRST_POSITION_MAX_GRAVITY_NM,
        "joint_1 model feed-forward {model_feedforward_nm:.6} Nm exceeds {J1_FIRST_POSITION_MAX_GRAVITY_NM:.6} Nm"
    );
    let total_feedforward_nm = model_feedforward_nm + additive_torque_nm;
    anyhow::ensure!(
        total_feedforward_nm.abs() <= J1_CENSORED_TORQUE_MAX_FEEDFORWARD_NM,
        "joint_1 total feed-forward {total_feedforward_nm:.6} Nm exceeds {J1_CENSORED_TORQUE_MAX_FEEDFORWARD_NM:.6} Nm"
    );
    let joint = &profile.joints[J1_FIRST_POSITION_INDEX];
    baseline_targets[J1_FIRST_POSITION_INDEX] = ros_target_to_motor(
        RosTarget {
            position_rad: hold_position_rad,
            velocity_rad_s: 0.0,
            torque_nm: total_feedforward_nm,
            kp_nm_rad: joint.default_kp,
            kd_nm_s_rad: joint.default_kd,
        },
        joint,
    );
    Ok(baseline_targets)
}

fn joint1_censored_torque_additive(
    level: usize,
    direction: Joint1CensoredTorqueDirection,
) -> Result<f32> {
    anyhow::ensure!(
        (1..=J1_CENSORED_TORQUE_LEVELS).contains(&level),
        "joint_1 censored-torque level {level} is outside 1..={J1_CENSORED_TORQUE_LEVELS}"
    );
    let additive_torque_nm = direction.sign() * level as f32 * J1_CENSORED_TORQUE_STEP_NM;
    anyhow::ensure!(
        additive_torque_nm.abs() <= J1_CENSORED_TORQUE_CAP_NM,
        "joint_1 censored-torque level {level} exceeds the fixed {J1_CENSORED_TORQUE_CAP_NM:.3} Nm cap"
    );
    Ok(additive_torque_nm)
}

fn joint1_censored_torque_should_censor(
    observation: DiagnosticObservation,
    direction: Joint1CensoredTorqueDirection,
) -> bool {
    direction.directional_delta(observation.telemetry.measured_delta)
        >= J1_CENSORED_TORQUE_TRIGGER_RAD
}

fn validate_joint1_censored_torque_active_time(
    diagnostic_started: Instant,
    phase: &str,
) -> Result<()> {
    anyhow::ensure!(
        diagnostic_started.elapsed().as_secs_f32() <= J1_CENSORED_TORQUE_MAX_ACTIVE_SEC,
        "joint_1 censored-torque diagnostic exceeded its {J1_CENSORED_TORQUE_MAX_ACTIVE_SEC:.3} s confirmed-active limit during {phase}"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn joint1_censored_torque_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    direction: Joint1CensoredTorqueDirection,
) -> Result<DiagnosticObservation> {
    let observation = joint1_observation_with_feedforward_limit(
        profile,
        feedback,
        J1_FIRST_POSITION_INDEX,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        initial_q[J1_FIRST_POSITION_INDEX],
        J1_CENSORED_TORQUE_MAX_FEEDFORWARD_NM,
        J1_FIRST_POSITION_EXPECTED_KP,
        J1_FIRST_POSITION_EXPECTED_KD,
    )?;
    let directional_delta = direction.directional_delta(observation.telemetry.measured_delta);
    anyhow::ensure!(
        (J1_CENSORED_TORQUE_HARD_NEGATIVE_RAD..J1_CENSORED_TORQUE_HARD_POSITIVE_RAD)
            .contains(&directional_delta),
        "joint_1 {} censored-torque directional displacement {:.6} rad left [{:.6}, {:.6}) rad",
        direction.label(),
        directional_delta,
        J1_CENSORED_TORQUE_HARD_NEGATIVE_RAD,
        J1_CENSORED_TORQUE_HARD_POSITIVE_RAD
    );
    anyhow::ensure!(
        observation.telemetry.measured_velocity.abs() < J1_CENSORED_TORQUE_HARD_VELOCITY_RAD_S,
        "joint_1 censored-torque velocity {:.6} rad/s reached the {:.6} rad/s hard layer",
        observation.telemetry.measured_velocity,
        J1_CENSORED_TORQUE_HARD_VELOCITY_RAD_S
    );
    Ok(observation)
}

#[allow(clippy::too_many_arguments)]
async fn confirm_joint1_censored_torque_target_readback_while_guarded(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    direction: Joint1CensoredTorqueDirection,
    freeze_deadline: Instant,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let settle_started = Instant::now();
    while settle_started.elapsed() < RPDO_READBACK_SETTLE {
        validate_joint1_censored_torque_readback_poll(
            backend,
            profile,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            target,
            direction,
            freeze_deadline,
            diagnostic_started,
            peaks,
        )?;
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let mut readback_future =
        Box::pin(backend.confirm_commissioning_target_readback(J1_FIRST_POSITION_INDEX, target));
    let readback = loop {
        tokio::select! {
            result = &mut readback_future => {
                break result.context("joint_1 censored-torque exact target readback failed")?;
            }
            _ = tokio::time::sleep(LOOP_PERIOD) => {
                validate_joint1_censored_torque_readback_poll(
                    backend,
                    profile,
                    initial_q,
                    initial_motor_position_rev,
                    temperature_baseline,
                    target,
                    direction,
                    freeze_deadline,
                    diagnostic_started,
                    peaks,
                )?;
            }
        }
    };
    validate_joint1_censored_torque_readback_poll(
        backend,
        profile,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        direction,
        freeze_deadline,
        diagnostic_started,
        peaks,
    )?;
    let joint = &profile.joints[J1_FIRST_POSITION_INDEX];
    tracing::info!(
        phase = PHASE_J1_CENSORED_TORQUE,
        milestone = "frozen_feedback_proven_target",
        joint = %joint.name,
        node_id = joint.node_id,
        torque_direction = direction.label(),
        expected_lower = %format_args!("0x{:08X}", readback.expected_lower),
        expected_upper = %format_args!("0x{:08X}", readback.expected_upper),
        actual_lower = %format_args!("0x{:08X}", readback.actual_lower),
        actual_upper = %format_args!("0x{:08X}", readback.actual_upper),
        "joint_1 frozen torque target matched exact 0x2004:02/03 readback"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn validate_joint1_censored_torque_readback_poll(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    direction: Joint1CensoredTorqueDirection,
    freeze_deadline: Instant,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    anyhow::ensure!(
        Instant::now() < freeze_deadline,
        "joint_1 exact frozen-target readback exceeded its one-second hold"
    );
    validate_joint1_censored_torque_active_time(diagnostic_started, "target readback")?;
    anyhow::ensure!(
        !backend.transport_failed(),
        "CAN transport failed during joint_1 censored-torque target readback"
    );
    backend
        .ensure_single_axis_commissioning_state(J1_FIRST_POSITION_INDEX)
        .context("six-axis state contract failed during joint_1 torque readback")?;
    let feedback = backend.feedback();
    let observation = joint1_censored_torque_observation(
        profile,
        &feedback,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        direction,
    )?;
    peaks.observe(observation);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn wait_for_joint1_first_position_enable_stability(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<[MotorTarget; DOF]> {
    let selected_index = J1_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let started = Instant::now();
    let mut stability = ContinuousStabilityGate::new(STABILITY_DWELL, ENABLE_STABILITY_TIMEOUT);
    loop {
        validate_joint1_first_position_active_time(diagnostic_started, "enable stability")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_1 first-position enable stability"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_1 enable stability")?;
        let feedback = backend.feedback();
        let measured_q = validate_feedback(profile, &feedback, None)?;
        let targets = build_safe_hold_targets(
            profile,
            dynamics,
            &measured_q,
            selected_index,
            initial_q[selected_index],
        )?;
        let observation = joint1_first_position_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
            initial_q[selected_index],
        )?;
        anyhow::ensure!(
            observation.telemetry.measured_delta.abs()
                <= J1_FIRST_POSITION_ENABLE_HARD_POSITION_RAD,
            "joint_1 enable displacement {:.6} rad exceeds the {:.6} rad hard gate",
            observation.telemetry.measured_delta,
            J1_FIRST_POSITION_ENABLE_HARD_POSITION_RAD
        );
        anyhow::ensure!(
            observation.telemetry.measured_velocity.abs()
                <= J1_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S,
            "joint_1 enable velocity {:.6} rad/s exceeds the {:.6} rad/s hard gate",
            observation.telemetry.measured_velocity,
            J1_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S
        );
        peaks.observe(observation);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_1 stability hold publish")?;
        backend.set_targets(targets).await?;
        let elapsed = started.elapsed();
        match stability.observe(
            elapsed,
            observation.telemetry.measured_delta.abs() <= J1_FIRST_POSITION_GO_POSITION_RAD
                && observation.telemetry.measured_velocity.abs()
                    <= J1_FIRST_POSITION_GO_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => {
                log_diagnostic_telemetry(
                    "go_dwell_complete",
                    PHASE_J1_FIRST_POSITION_ENABLE,
                    joint,
                    0,
                    1,
                    observation.telemetry.gravity_ff,
                    0.0,
                    diagnostic_started.elapsed(),
                    observation,
                );
                return Ok(targets);
            }
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_1 did not satisfy the {:.6} rad / {:.6} rad/s GO gate continuously for {:.3} s within {:.3} s",
                J1_FIRST_POSITION_GO_POSITION_RAD,
                J1_FIRST_POSITION_GO_VELOCITY_RAD_S,
                STABILITY_DWELL.as_secs_f32(),
                ENABLE_STABILITY_TIMEOUT.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_joint1_first_position_round_trip(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let selected_index = J1_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let baseline_position_code =
        compressed_target_position_code(&baseline_targets[selected_index], joint);
    let trajectory_started = Instant::now();
    let trajectory_duration = Duration::from_secs_f32(J1_FIRST_POSITION_DURATION_SEC);
    let mut readback_pause = Duration::ZERO;
    let mut changed_target_readback = false;
    let mut maximum_positive_delta_rad = 0.0_f32;
    let mut most_negative_raw_delta_rev = 0.0_f32;
    let mut milestones = MotionTelemetryMilestones::default();

    loop {
        let trajectory_elapsed = trajectory_started.elapsed().saturating_sub(readback_pause);
        if trajectory_elapsed >= trajectory_duration {
            break;
        }
        validate_joint1_first_position_active_time(diagnostic_started, "trajectory")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_1 fixed position trajectory"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_1 fixed trajectory")?;
        let feedback = backend.feedback();
        let measured_q = validate_feedback(profile, &feedback, None)?;
        let normalized_time = trajectory_elapsed.as_secs_f32() / J1_FIRST_POSITION_DURATION_SEC;
        let commanded_q = initial_q[selected_index]
            + round_trip_phase(normalized_time) * J1_FIRST_POSITION_DELTA_RAD;
        let (targets, model_gravity_ff_nm, friction_compensation_nm) =
            build_joint1_first_position_trajectory_targets(
                profile,
                dynamics,
                &measured_q,
                commanded_q,
                normalized_time,
            )?;
        let observation = joint1_compensated_position_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
            commanded_q,
        )?;
        peaks.observe(observation);
        maximum_positive_delta_rad =
            maximum_positive_delta_rad.max(observation.telemetry.measured_delta);
        most_negative_raw_delta_rev = most_negative_raw_delta_rev
            .min(feedback.joints[selected_index].position_rev - initial_motor_position_rev);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_1 target publish")?;
        backend.set_targets(targets).await?;

        while let Some(milestone) = milestones.take_due(normalized_time) {
            log_diagnostic_telemetry(
                milestone,
                PHASE_J1_FIRST_POSITION_TRAJECTORY,
                joint,
                1,
                1,
                model_gravity_ff_nm,
                friction_compensation_nm,
                diagnostic_started.elapsed(),
                observation,
            );
        }

        let position_code = compressed_target_position_code(&targets[selected_index], joint);
        if !changed_target_readback && position_code != baseline_position_code {
            let readback_started = Instant::now();
            confirm_joint1_first_position_target_readback_while_guarded(
                backend,
                profile,
                selected_index,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                targets[selected_index],
                commanded_q,
                diagnostic_started,
                "first_quantized_changed_position",
                J1_FIRST_POSITION_MAX_COMPENSATED_FEEDFORWARD_NM,
                J1_FIRST_POSITION_TRAJECTORY_KP,
                J1_FIRST_POSITION_TRAJECTORY_KD,
                peaks,
            )
            .await?;
            readback_pause += readback_started.elapsed();
            changed_target_readback = true;
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    anyhow::ensure!(
        changed_target_readback,
        "joint_1 fixed survey completed without exact readback of a changed position code"
    );

    // Re-publish the same persistent baseline before settling. Any later
    // cancellation reaches the identical in-memory cleanup command.
    backend
        .ensure_single_axis_commissioning_state(selected_index)
        .context("six-axis state contract failed before joint_1 baseline return")?;
    backend.set_targets(baseline_targets).await?;
    let return_started = Instant::now();
    let mut return_stability = ContinuousStabilityGate::new(STABILITY_DWELL, RETURN_SETTLE_TIMEOUT);
    let (return_error_rad, return_velocity_rad_s) = loop {
        validate_joint1_first_position_active_time(diagnostic_started, "return settle")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_1 return settle"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_1 return settle")?;
        let feedback = backend.feedback();
        let observation = joint1_first_position_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            baseline_targets[selected_index],
            initial_q[selected_index],
        )?;
        peaks.observe(observation);
        maximum_positive_delta_rad =
            maximum_positive_delta_rad.max(observation.telemetry.measured_delta);
        most_negative_raw_delta_rev = most_negative_raw_delta_rev
            .min(feedback.joints[selected_index].position_rev - initial_motor_position_rev);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_1 return publish")?;
        backend.set_targets(baseline_targets).await?;
        let return_error = observation.telemetry.measured_delta.abs();
        let return_velocity = observation.telemetry.measured_velocity;
        match return_stability.observe(
            return_started.elapsed(),
            return_error <= J1_FIRST_POSITION_RETURN_POSITION_RAD
                && return_velocity.abs() <= J1_FIRST_POSITION_RETURN_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => break (return_error, return_velocity),
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_1 did not return within {:.3} s: error {return_error:.6} rad (limit {:.6}), velocity {return_velocity:.6} rad/s (limit {:.6}); both must hold continuously for {:.3} s",
                RETURN_SETTLE_TIMEOUT.as_secs_f32(),
                J1_FIRST_POSITION_RETURN_POSITION_RAD,
                J1_FIRST_POSITION_RETURN_VELOCITY_RAD_S,
                STABILITY_DWELL.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    };

    confirm_joint1_first_position_target_readback_while_guarded(
        backend,
        profile,
        selected_index,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        baseline_targets[selected_index],
        initial_q[selected_index],
        diagnostic_started,
        "returned_persistent_baseline",
        J1_FIRST_POSITION_MAX_GRAVITY_NM,
        J1_FIRST_POSITION_EXPECTED_KP,
        J1_FIRST_POSITION_EXPECTED_KD,
        peaks,
    )
    .await?;
    validate_joint1_first_position_acceptance(
        maximum_positive_delta_rad,
        most_negative_raw_delta_rev,
    )?;
    tracing::info!(
        phase = PHASE_J1_FIRST_POSITION_RETURN,
        joint = %joint.name,
        node_id = joint.node_id,
        maximum_positive_delta_rad,
        most_negative_raw_delta_rev,
        return_error_rad,
        return_velocity_rad_s,
        "joint_1 fixed survey met logical/raw direction and continuous return acceptance gates"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn confirm_joint1_first_position_target_readback_while_guarded(
    backend: &RealBackend,
    profile: &HardwareProfile,
    selected_index: usize,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    diagnostic_started: Instant,
    milestone: &'static str,
    maximum_feedforward_nm: f32,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let settle_started = Instant::now();
    while settle_started.elapsed() < RPDO_READBACK_SETTLE {
        validate_joint1_first_position_readback_poll(
            backend,
            profile,
            selected_index,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            target,
            commanded_q,
            diagnostic_started,
            maximum_feedforward_nm,
            expected_kp_nm_rad,
            expected_kd_nm_s_rad,
            peaks,
        )?;
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let mut readback_future =
        Box::pin(backend.confirm_commissioning_target_readback(selected_index, target));
    let readback = loop {
        tokio::select! {
            result = &mut readback_future => {
                break result.context("joint_1 exact compressed-MIT target readback failed")?;
            }
            _ = tokio::time::sleep(LOOP_PERIOD) => {
                validate_joint1_first_position_readback_poll(
                    backend,
                    profile,
                    selected_index,
                    initial_q,
                    initial_motor_position_rev,
                    temperature_baseline,
                    target,
                    commanded_q,
                    diagnostic_started,
                    maximum_feedforward_nm,
                    expected_kp_nm_rad,
                    expected_kd_nm_s_rad,
                    peaks,
                )?;
            }
        }
    };
    validate_joint1_first_position_readback_poll(
        backend,
        profile,
        selected_index,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        commanded_q,
        diagnostic_started,
        maximum_feedforward_nm,
        expected_kp_nm_rad,
        expected_kd_nm_s_rad,
        peaks,
    )?;
    let joint = &profile.joints[selected_index];
    tracing::info!(
        phase = PHASE_J1_FIRST_POSITION_TRAJECTORY,
        milestone,
        joint = %joint.name,
        node_id = joint.node_id,
        commanded_q,
        expected_lower = %format_args!("0x{:08X}", readback.expected_lower),
        expected_upper = %format_args!("0x{:08X}", readback.expected_upper),
        actual_lower = %format_args!("0x{:08X}", readback.actual_lower),
        actual_upper = %format_args!("0x{:08X}", readback.actual_upper),
        "joint_1 held target matched exact 0x2004:02/03 readback"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn validate_joint1_first_position_readback_poll(
    backend: &RealBackend,
    profile: &HardwareProfile,
    selected_index: usize,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    diagnostic_started: Instant,
    maximum_feedforward_nm: f32,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    validate_joint1_first_position_active_time(diagnostic_started, "SDO target readback")?;
    anyhow::ensure!(
        !backend.transport_failed(),
        "CAN transport failed during joint_1 target readback"
    );
    backend
        .ensure_single_axis_commissioning_state(selected_index)
        .context("six-axis state contract failed during joint_1 target readback")?;
    let feedback = backend.feedback();
    let observation = joint1_observation_with_feedforward_limit(
        profile,
        &feedback,
        selected_index,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        commanded_q,
        maximum_feedforward_nm,
        expected_kp_nm_rad,
        expected_kd_nm_s_rad,
    )?;
    peaks.observe(observation);
    Ok(())
}

fn validate_joint5_first_position_dynamic_window(
    joint: &crate::profile::JointProfile,
    initial_position_rad: f32,
) -> Result<()> {
    anyhow::ensure!(
        initial_position_rad.is_finite()
            && (J5_FIRST_POSITION_INITIAL_Q_LOWER_RAD
                ..=J5_FIRST_POSITION_INITIAL_Q_UPPER_RAD)
                .contains(&initial_position_rad),
        "joint_5 q0={initial_position_rad:.6} rad is outside the fixed [{J5_FIRST_POSITION_INITIAL_Q_LOWER_RAD:.6}, {J5_FIRST_POSITION_INITIAL_Q_UPPER_RAD:.6}] range"
    );
    let lower = initial_position_rad + J5_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD;
    let upper = initial_position_rad + J5_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD;
    anyhow::ensure!(
        lower >= joint.limits.position_lower_rad && upper <= joint.limits.position_upper_rad,
        "joint_5 dynamic window [{lower:.6}, {upper:.6}] is not wholly inside profile limits [{:.6}, {:.6}]",
        joint.limits.position_lower_rad,
        joint.limits.position_upper_rad
    );
    Ok(())
}

fn validate_joint5_first_position_active_time(
    diagnostic_started: Instant,
    phase: &str,
) -> Result<()> {
    anyhow::ensure!(
        diagnostic_started.elapsed().as_secs_f32() <= J5_FIRST_POSITION_MAX_ACTIVE_SEC,
        "joint_5 first-position diagnostic exceeded {J5_FIRST_POSITION_MAX_ACTIVE_SEC:.3} s during {phase}"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn joint5_first_position_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
) -> Result<DiagnosticObservation> {
    fixed_position_observation(
        profile,
        feedback,
        J5_FIRST_POSITION_INDEX,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        commanded_q,
        "joint_5",
        J5_FIRST_POSITION_INDEX,
        J5_FIRST_POSITION_DELTA_RAD,
        J5_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD,
        J5_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD,
        J5_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S,
        J5_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        J5_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        J5_FIRST_POSITION_MAX_TOTAL_TORQUE_NM,
        expected_kp_nm_rad,
        expected_kd_nm_s_rad,
    )
}

#[allow(clippy::too_many_arguments)]
async fn wait_for_joint5_first_position_enable_stability(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<[MotorTarget; DOF]> {
    let selected_index = J5_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let started = Instant::now();
    let mut stability = ContinuousStabilityGate::new(STABILITY_DWELL, ENABLE_STABILITY_TIMEOUT);
    loop {
        validate_joint5_first_position_active_time(diagnostic_started, "enable stability")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_5 enable stability"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_5 enable stability")?;
        let feedback = backend.feedback();
        let measured_q = validate_feedback(profile, &feedback, None)?;
        let targets = build_safe_hold_targets(
            profile,
            dynamics,
            &measured_q,
            selected_index,
            initial_q[selected_index],
        )?;
        let observation = joint5_first_position_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
            initial_q[selected_index],
            J5_FIRST_POSITION_EXPECTED_KP,
            J5_FIRST_POSITION_EXPECTED_KD,
        )?;
        anyhow::ensure!(
            observation.telemetry.measured_delta.abs()
                <= J5_FIRST_POSITION_ENABLE_HARD_POSITION_RAD,
            "joint_5 enable displacement {:.6} rad exceeds {:.6} rad",
            observation.telemetry.measured_delta,
            J5_FIRST_POSITION_ENABLE_HARD_POSITION_RAD
        );
        peaks.observe(observation);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_5 stability publish")?;
        backend.set_targets(targets).await?;
        match stability.observe(
            started.elapsed(),
            observation.telemetry.measured_delta.abs() <= J5_FIRST_POSITION_GO_POSITION_RAD
                && observation.telemetry.measured_velocity.abs()
                    <= J5_FIRST_POSITION_GO_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => {
                log_diagnostic_telemetry(
                    "go_dwell_complete",
                    PHASE_J5_FIRST_POSITION_ENABLE,
                    joint,
                    0,
                    1,
                    observation.telemetry.gravity_ff,
                    0.0,
                    diagnostic_started.elapsed(),
                    observation,
                );
                return Ok(targets);
            }
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_5 did not satisfy the {:.6} rad / {:.6} rad/s GO gate continuously for {:.3} s",
                J5_FIRST_POSITION_GO_POSITION_RAD,
                J5_FIRST_POSITION_GO_VELOCITY_RAD_S,
                STABILITY_DWELL.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

#[allow(clippy::too_many_arguments)]
async fn confirm_joint5_first_position_target_readback_while_guarded(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    diagnostic_started: Instant,
    milestone: &'static str,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let poll = || -> Result<DiagnosticObservation> {
        validate_joint5_first_position_active_time(diagnostic_started, "SDO target readback")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_5 target readback"
        );
        backend
            .ensure_single_axis_commissioning_state(J5_FIRST_POSITION_INDEX)
            .context("six-axis state contract failed during joint_5 target readback")?;
        joint5_first_position_observation(
            profile,
            &backend.feedback(),
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            target,
            commanded_q,
            expected_kp_nm_rad,
            expected_kd_nm_s_rad,
        )
    };
    let settle_started = Instant::now();
    while settle_started.elapsed() < RPDO_READBACK_SETTLE {
        peaks.observe(poll()?);
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let mut readback_future =
        Box::pin(backend.confirm_commissioning_target_readback(J5_FIRST_POSITION_INDEX, target));
    let readback = loop {
        tokio::select! {
            result = &mut readback_future => {
                break result.context("joint_5 exact compressed-MIT target readback failed")?;
            }
            _ = tokio::time::sleep(LOOP_PERIOD) => {
                peaks.observe(poll()?);
            }
        }
    };
    peaks.observe(poll()?);
    let joint = &profile.joints[J5_FIRST_POSITION_INDEX];
    tracing::info!(
        phase = PHASE_J5_FIRST_POSITION_TRAJECTORY,
        milestone,
        joint = %joint.name,
        node_id = joint.node_id,
        commanded_q,
        expected_lower = %format_args!("0x{:08X}", readback.expected_lower),
        expected_upper = %format_args!("0x{:08X}", readback.expected_upper),
        actual_lower = %format_args!("0x{:08X}", readback.actual_lower),
        actual_upper = %format_args!("0x{:08X}", readback.actual_upper),
        "joint_5 held target matched exact 0x2004:02/03 readback"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn run_joint5_first_position_round_trip(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let selected_index = J5_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let baseline_position_code =
        compressed_target_position_code(&baseline_targets[selected_index], joint);
    let trajectory_started = Instant::now();
    let trajectory_duration = Duration::from_secs_f32(J5_FIRST_POSITION_DURATION_SEC);
    let mut readback_pause = Duration::ZERO;
    let mut changed_target_readback = false;
    let mut most_negative_delta_rad = 0.0_f32;
    let mut most_negative_raw_delta_rev = 0.0_f32;
    let mut milestones = MotionTelemetryMilestones::default();
    loop {
        let trajectory_elapsed = trajectory_started.elapsed().saturating_sub(readback_pause);
        if trajectory_elapsed >= trajectory_duration {
            break;
        }
        validate_joint5_first_position_active_time(diagnostic_started, "trajectory")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_5 trajectory"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_5 trajectory")?;
        let feedback = backend.feedback();
        let measured_q = validate_feedback(profile, &feedback, None)?;
        let normalized_time = trajectory_elapsed.as_secs_f32() / J5_FIRST_POSITION_DURATION_SEC;
        let commanded_q = initial_q[selected_index]
            + round_trip_phase(normalized_time) * J5_FIRST_POSITION_DELTA_RAD;
        let targets = build_joint5_first_position_trajectory_targets(
            profile,
            dynamics,
            &measured_q,
            commanded_q,
        )?;
        let observation = joint5_first_position_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
            commanded_q,
            J5_FIRST_POSITION_TRAJECTORY_KP,
            J5_FIRST_POSITION_TRAJECTORY_KD,
        )?;
        peaks.observe(observation);
        most_negative_delta_rad = most_negative_delta_rad.min(observation.telemetry.measured_delta);
        most_negative_raw_delta_rev = most_negative_raw_delta_rev
            .min(feedback.joints[selected_index].position_rev - initial_motor_position_rev);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_5 target publish")?;
        backend.set_targets(targets).await?;
        while let Some(milestone) = milestones.take_due(normalized_time) {
            log_diagnostic_telemetry(
                milestone,
                PHASE_J5_FIRST_POSITION_TRAJECTORY,
                joint,
                1,
                1,
                observation.telemetry.gravity_ff,
                0.0,
                diagnostic_started.elapsed(),
                observation,
            );
        }
        let position_code = compressed_target_position_code(&targets[selected_index], joint);
        if !changed_target_readback && position_code != baseline_position_code {
            let readback_started = Instant::now();
            confirm_joint5_first_position_target_readback_while_guarded(
                backend,
                profile,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                targets[selected_index],
                commanded_q,
                diagnostic_started,
                "first_quantized_changed_position",
                J5_FIRST_POSITION_TRAJECTORY_KP,
                J5_FIRST_POSITION_TRAJECTORY_KD,
                peaks,
            )
            .await?;
            readback_pause += readback_started.elapsed();
            changed_target_readback = true;
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    anyhow::ensure!(
        changed_target_readback,
        "joint_5 survey completed without exact readback of a changed position code"
    );
    backend.set_targets(baseline_targets).await?;
    let return_started = Instant::now();
    let mut return_stability = ContinuousStabilityGate::new(STABILITY_DWELL, RETURN_SETTLE_TIMEOUT);
    let (return_error_rad, return_velocity_rad_s) = loop {
        validate_joint5_first_position_active_time(diagnostic_started, "return settle")?;
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_5 return settle")?;
        let feedback = backend.feedback();
        let observation = joint5_first_position_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            baseline_targets[selected_index],
            initial_q[selected_index],
            J5_FIRST_POSITION_EXPECTED_KP,
            J5_FIRST_POSITION_EXPECTED_KD,
        )?;
        peaks.observe(observation);
        most_negative_delta_rad = most_negative_delta_rad.min(observation.telemetry.measured_delta);
        most_negative_raw_delta_rev = most_negative_raw_delta_rev
            .min(feedback.joints[selected_index].position_rev - initial_motor_position_rev);
        backend.set_targets(baseline_targets).await?;
        let return_error = observation.telemetry.measured_delta.abs();
        let return_velocity = observation.telemetry.measured_velocity;
        match return_stability.observe(
            return_started.elapsed(),
            return_error <= J5_FIRST_POSITION_RETURN_POSITION_RAD
                && return_velocity.abs() <= J5_FIRST_POSITION_RETURN_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => break (return_error, return_velocity),
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_5 did not return within {:.3} s: error {return_error:.6} rad, velocity {return_velocity:.6} rad/s",
                RETURN_SETTLE_TIMEOUT.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    };
    confirm_joint5_first_position_target_readback_while_guarded(
        backend,
        profile,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        baseline_targets[selected_index],
        initial_q[selected_index],
        diagnostic_started,
        "returned_persistent_baseline",
        J5_FIRST_POSITION_EXPECTED_KP,
        J5_FIRST_POSITION_EXPECTED_KD,
        peaks,
    )
    .await?;
    anyhow::ensure!(
        most_negative_delta_rad <= -J5_FIRST_POSITION_REQUIRED_PEAK_RAD,
        "joint_5 negative peak {most_negative_delta_rad:.6} rad did not reach -{J5_FIRST_POSITION_REQUIRED_PEAK_RAD:.6} rad"
    );
    anyhow::ensure!(
        most_negative_raw_delta_rev <= -J5_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV,
        "joint_5 raw motor peak {most_negative_raw_delta_rev:.9} rev did not reach -{J5_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV:.9} rev"
    );
    tracing::info!(
        phase = PHASE_J5_FIRST_POSITION_RETURN,
        joint = %joint.name,
        node_id = joint.node_id,
        most_negative_delta_rad,
        most_negative_raw_delta_rev,
        return_error_rad,
        return_velocity_rad_s,
        "joint_5 fixed survey met logical/raw direction and continuous return gates"
    );
    Ok(())
}

fn validate_joint6_first_position_dynamic_window(
    joint: &crate::profile::JointProfile,
    initial_position_rad: f32,
) -> Result<()> {
    anyhow::ensure!(
        initial_position_rad.is_finite()
            && (J6_FIRST_POSITION_INITIAL_Q_LOWER_RAD
                ..=J6_FIRST_POSITION_INITIAL_Q_UPPER_RAD)
                .contains(&initial_position_rad),
        "joint_6 q0={initial_position_rad:.6} rad is outside the fixed [{J6_FIRST_POSITION_INITIAL_Q_LOWER_RAD:.6}, {J6_FIRST_POSITION_INITIAL_Q_UPPER_RAD:.6}] range"
    );
    let lower = initial_position_rad + J6_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD;
    let upper = initial_position_rad + J6_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD;
    anyhow::ensure!(
        lower >= joint.limits.position_lower_rad && upper <= joint.limits.position_upper_rad,
        "joint_6 dynamic window [{lower:.6}, {upper:.6}] is not wholly inside profile limits [{:.6}, {:.6}]",
        joint.limits.position_lower_rad,
        joint.limits.position_upper_rad
    );
    Ok(())
}

fn validate_joint4_first_position_dynamic_window(
    joint: &crate::profile::JointProfile,
    initial_position_rad: f32,
) -> Result<()> {
    anyhow::ensure!(
        initial_position_rad.is_finite()
            && (J4_FIRST_POSITION_INITIAL_Q_LOWER_RAD
                ..=J4_FIRST_POSITION_INITIAL_Q_UPPER_RAD)
                .contains(&initial_position_rad),
        "joint_4 q0={initial_position_rad:.6} rad is outside the fixed [{J4_FIRST_POSITION_INITIAL_Q_LOWER_RAD:.6}, {J4_FIRST_POSITION_INITIAL_Q_UPPER_RAD:.6}] range"
    );
    let lower = initial_position_rad + J4_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD;
    let upper = initial_position_rad + J4_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD;
    anyhow::ensure!(
        lower >= joint.limits.position_lower_rad && upper <= joint.limits.position_upper_rad,
        "joint_4 dynamic window [{lower:.6}, {upper:.6}] is not wholly inside profile limits [{:.6}, {:.6}]",
        joint.limits.position_lower_rad,
        joint.limits.position_upper_rad
    );
    Ok(())
}

fn validate_joint4_first_position_active_time(
    diagnostic_started: Instant,
    phase: &str,
) -> Result<()> {
    anyhow::ensure!(
        diagnostic_started.elapsed().as_secs_f32() <= J4_FIRST_POSITION_MAX_ACTIVE_SEC,
        "joint_4 first-position diagnostic exceeded {J4_FIRST_POSITION_MAX_ACTIVE_SEC:.3} s during {phase}"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn joint4_first_position_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
) -> Result<DiagnosticObservation> {
    joint4_position_observation_with_limits(
        profile,
        feedback,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        commanded_q,
        expected_kp_nm_rad,
        expected_kd_nm_s_rad,
        J4_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        J4_FIRST_POSITION_MAX_TOTAL_TORQUE_NM,
    )
}

#[allow(clippy::too_many_arguments)]
fn joint4_position_observation_with_limits(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
    maximum_feedforward_nm: f32,
    maximum_total_torque_nm: f32,
) -> Result<DiagnosticObservation> {
    fixed_position_observation(
        profile,
        feedback,
        J4_FIRST_POSITION_INDEX,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        commanded_q,
        "joint_4",
        J4_FIRST_POSITION_INDEX,
        J4_FIRST_POSITION_DELTA_RAD,
        J4_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD,
        J4_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD,
        J4_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S,
        maximum_feedforward_nm,
        J4_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        maximum_total_torque_nm,
        expected_kp_nm_rad,
        expected_kd_nm_s_rad,
    )
}

#[allow(clippy::too_many_arguments)]
async fn wait_for_joint4_first_position_enable_stability(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<[MotorTarget; DOF]> {
    let selected_index = J4_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let started = Instant::now();
    let mut stability = ContinuousStabilityGate::new(STABILITY_DWELL, ENABLE_STABILITY_TIMEOUT);
    loop {
        validate_joint4_first_position_active_time(diagnostic_started, "enable stability")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_4 enable stability"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_4 enable stability")?;
        let feedback = backend.feedback();
        let measured_q = validate_feedback(profile, &feedback, None)?;
        let targets = build_safe_hold_targets(
            profile,
            dynamics,
            &measured_q,
            selected_index,
            initial_q[selected_index],
        )?;
        let observation = joint4_first_position_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
            initial_q[selected_index],
            J4_FIRST_POSITION_EXPECTED_KP,
            J4_FIRST_POSITION_EXPECTED_KD,
        )?;
        anyhow::ensure!(
            observation.telemetry.measured_delta.abs()
                <= J4_FIRST_POSITION_ENABLE_HARD_POSITION_RAD,
            "joint_4 enable displacement {:.6} rad exceeds {:.6} rad",
            observation.telemetry.measured_delta,
            J4_FIRST_POSITION_ENABLE_HARD_POSITION_RAD
        );
        peaks.observe(observation);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_4 stability publish")?;
        backend.set_targets(targets).await?;
        match stability.observe(
            started.elapsed(),
            observation.telemetry.measured_delta.abs() <= J4_FIRST_POSITION_GO_POSITION_RAD
                && observation.telemetry.measured_velocity.abs()
                    <= J4_FIRST_POSITION_GO_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => {
                log_diagnostic_telemetry(
                    "go_dwell_complete",
                    PHASE_J4_FIRST_POSITION_ENABLE,
                    joint,
                    0,
                    1,
                    observation.telemetry.gravity_ff,
                    0.0,
                    diagnostic_started.elapsed(),
                    observation,
                );
                return Ok(targets);
            }
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_4 did not satisfy the {:.6} rad / {:.6} rad/s GO gate continuously for {:.3} s",
                J4_FIRST_POSITION_GO_POSITION_RAD,
                J4_FIRST_POSITION_GO_VELOCITY_RAD_S,
                STABILITY_DWELL.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

#[allow(clippy::too_many_arguments)]
async fn confirm_joint4_first_position_target_readback_while_guarded(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    diagnostic_started: Instant,
    phase: &'static str,
    milestone: &'static str,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
    maximum_feedforward_nm: f32,
    maximum_total_torque_nm: f32,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let poll = || -> Result<DiagnosticObservation> {
        validate_joint4_first_position_active_time(diagnostic_started, "SDO target readback")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_4 target readback"
        );
        backend
            .ensure_single_axis_commissioning_state(J4_FIRST_POSITION_INDEX)
            .context("six-axis state contract failed during joint_4 target readback")?;
        joint4_position_observation_with_limits(
            profile,
            &backend.feedback(),
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            target,
            commanded_q,
            expected_kp_nm_rad,
            expected_kd_nm_s_rad,
            maximum_feedforward_nm,
            maximum_total_torque_nm,
        )
    };
    let settle_started = Instant::now();
    while settle_started.elapsed() < RPDO_READBACK_SETTLE {
        peaks.observe(poll()?);
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let mut readback_future =
        Box::pin(backend.confirm_commissioning_target_readback(J4_FIRST_POSITION_INDEX, target));
    let readback = loop {
        tokio::select! {
            result = &mut readback_future => {
                break result.context("joint_4 exact compressed-MIT target readback failed")?;
            }
            _ = tokio::time::sleep(LOOP_PERIOD) => {
                peaks.observe(poll()?);
            }
        }
    };
    peaks.observe(poll()?);
    let joint = &profile.joints[J4_FIRST_POSITION_INDEX];
    tracing::info!(
        phase,
        milestone,
        joint = %joint.name,
        node_id = joint.node_id,
        commanded_q,
        expected_lower = %format_args!("0x{:08X}", readback.expected_lower),
        expected_upper = %format_args!("0x{:08X}", readback.expected_upper),
        actual_lower = %format_args!("0x{:08X}", readback.actual_lower),
        actual_upper = %format_args!("0x{:08X}", readback.actual_upper),
        "joint_4 held target matched exact 0x2004:02/03 readback"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn run_joint4_first_position_round_trip(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
    peak_assistance_nm: f32,
) -> Result<()> {
    anyhow::ensure!(
        peak_assistance_nm == 0.0
            || peak_assistance_nm.to_bits() == J4_ASSISTED_POSITION_PEAK_TORQUE_NM.to_bits(),
        "joint_4 position survey accepts only zero assistance or the fixed {:.3} Nm envelope",
        J4_ASSISTED_POSITION_PEAK_TORQUE_NM
    );
    let assisted = peak_assistance_nm != 0.0;
    let maximum_feedforward_nm = if assisted {
        J4_ASSISTED_POSITION_MAX_FEEDFORWARD_NM
    } else {
        J4_FIRST_POSITION_MAX_FEEDFORWARD_NM
    };
    let maximum_total_torque_nm = if assisted {
        J4_ASSISTED_POSITION_MAX_TOTAL_TORQUE_NM
    } else {
        J4_FIRST_POSITION_MAX_TOTAL_TORQUE_NM
    };
    let trajectory_phase = if assisted {
        PHASE_J4_ASSISTED_POSITION_TRAJECTORY
    } else {
        PHASE_J4_FIRST_POSITION_TRAJECTORY
    };
    let selected_index = J4_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let baseline_position_code =
        compressed_target_position_code(&baseline_targets[selected_index], joint);
    let trajectory_started = Instant::now();
    let trajectory_duration = Duration::from_secs_f32(J4_FIRST_POSITION_DURATION_SEC);
    let mut readback_pause = Duration::ZERO;
    let mut changed_target_readback = false;
    let mut most_negative_delta_rad = 0.0_f32;
    let mut most_negative_raw_delta_rev = 0.0_f32;
    let mut milestones = MotionTelemetryMilestones::default();
    loop {
        let trajectory_elapsed = trajectory_started.elapsed().saturating_sub(readback_pause);
        if trajectory_elapsed >= trajectory_duration {
            break;
        }
        validate_joint4_first_position_active_time(diagnostic_started, "trajectory")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_4 trajectory"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_4 trajectory")?;
        let feedback = backend.feedback();
        let measured_q = validate_feedback(profile, &feedback, None)?;
        let normalized_time = trajectory_elapsed.as_secs_f32() / J4_FIRST_POSITION_DURATION_SEC;
        let path_phase = round_trip_phase(normalized_time);
        let commanded_q = initial_q[selected_index] + path_phase * J4_FIRST_POSITION_DELTA_RAD;
        let assistance_nm = path_phase * peak_assistance_nm;
        let targets = if assisted {
            build_joint4_assisted_position_trajectory_targets(
                profile,
                dynamics,
                &measured_q,
                commanded_q,
                assistance_nm,
            )?
        } else {
            build_joint4_first_position_trajectory_targets(
                profile,
                dynamics,
                &measured_q,
                commanded_q,
            )?
        };
        let observation = joint4_position_observation_with_limits(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
            commanded_q,
            J4_FIRST_POSITION_TRAJECTORY_KP,
            J4_FIRST_POSITION_TRAJECTORY_KD,
            maximum_feedforward_nm,
            maximum_total_torque_nm,
        )?;
        peaks.observe(observation);
        most_negative_delta_rad = most_negative_delta_rad.min(observation.telemetry.measured_delta);
        most_negative_raw_delta_rev = most_negative_raw_delta_rev
            .min(feedback.joints[selected_index].position_rev - initial_motor_position_rev);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_4 target publish")?;
        backend.set_targets(targets).await?;
        while let Some(milestone) = milestones.take_due(normalized_time) {
            log_diagnostic_telemetry(
                milestone,
                trajectory_phase,
                joint,
                1,
                1,
                observation.telemetry.gravity_ff - assistance_nm,
                assistance_nm,
                diagnostic_started.elapsed(),
                observation,
            );
        }
        let position_code = compressed_target_position_code(&targets[selected_index], joint);
        if !changed_target_readback && position_code != baseline_position_code {
            let readback_started = Instant::now();
            confirm_joint4_first_position_target_readback_while_guarded(
                backend,
                profile,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                targets[selected_index],
                commanded_q,
                diagnostic_started,
                trajectory_phase,
                "first_quantized_changed_position",
                J4_FIRST_POSITION_TRAJECTORY_KP,
                J4_FIRST_POSITION_TRAJECTORY_KD,
                maximum_feedforward_nm,
                maximum_total_torque_nm,
                peaks,
            )
            .await?;
            readback_pause += readback_started.elapsed();
            changed_target_readback = true;
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    anyhow::ensure!(
        changed_target_readback,
        "joint_4 survey completed without exact readback of a changed position code"
    );
    backend.set_targets(baseline_targets).await?;
    let return_started = Instant::now();
    let mut return_stability = ContinuousStabilityGate::new(STABILITY_DWELL, RETURN_SETTLE_TIMEOUT);
    let (return_error_rad, return_velocity_rad_s) = loop {
        validate_joint4_first_position_active_time(diagnostic_started, "return settle")?;
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_4 return settle")?;
        let feedback = backend.feedback();
        let observation = joint4_first_position_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            baseline_targets[selected_index],
            initial_q[selected_index],
            J4_FIRST_POSITION_EXPECTED_KP,
            J4_FIRST_POSITION_EXPECTED_KD,
        )?;
        peaks.observe(observation);
        most_negative_delta_rad = most_negative_delta_rad.min(observation.telemetry.measured_delta);
        most_negative_raw_delta_rev = most_negative_raw_delta_rev
            .min(feedback.joints[selected_index].position_rev - initial_motor_position_rev);
        backend.set_targets(baseline_targets).await?;
        let return_error = observation.telemetry.measured_delta.abs();
        let return_velocity = observation.telemetry.measured_velocity;
        match return_stability.observe(
            return_started.elapsed(),
            return_error <= J4_FIRST_POSITION_RETURN_POSITION_RAD
                && return_velocity.abs() <= J4_FIRST_POSITION_RETURN_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => break (return_error, return_velocity),
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_4 did not return within {:.3} s: error {return_error:.6} rad, velocity {return_velocity:.6} rad/s",
                RETURN_SETTLE_TIMEOUT.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    };
    confirm_joint4_first_position_target_readback_while_guarded(
        backend,
        profile,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        baseline_targets[selected_index],
        initial_q[selected_index],
        diagnostic_started,
        trajectory_phase,
        "returned_persistent_baseline",
        J4_FIRST_POSITION_EXPECTED_KP,
        J4_FIRST_POSITION_EXPECTED_KD,
        J4_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        J4_FIRST_POSITION_MAX_TOTAL_TORQUE_NM,
        peaks,
    )
    .await?;
    anyhow::ensure!(
        most_negative_delta_rad <= -J4_FIRST_POSITION_REQUIRED_PEAK_RAD,
        "joint_4 negative peak {most_negative_delta_rad:.6} rad did not reach -{J4_FIRST_POSITION_REQUIRED_PEAK_RAD:.6} rad"
    );
    anyhow::ensure!(
        most_negative_raw_delta_rev <= -J4_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV,
        "joint_4 raw motor peak {most_negative_raw_delta_rev:.9} rev did not reach -{J4_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV:.9} rev"
    );
    tracing::info!(
        phase = PHASE_J4_FIRST_POSITION_RETURN,
        joint = %joint.name,
        node_id = joint.node_id,
        most_negative_delta_rad,
        most_negative_raw_delta_rev,
        return_error_rad,
        return_velocity_rad_s,
        "joint_4 fixed survey met logical/raw direction and continuous return gates"
    );
    Ok(())
}

fn validate_joint3_gravity_unload_active_time(
    diagnostic_started: Instant,
    phase: &str,
) -> Result<()> {
    anyhow::ensure!(
        diagnostic_started.elapsed().as_secs_f32() <= J3_GRAVITY_UNLOAD_MAX_ACTIVE_SEC,
        "joint_3 gravity-unload diagnostic exceeded its {J3_GRAVITY_UNLOAD_MAX_ACTIVE_SEC:.3} s confirmed-active limit during {phase}"
    );
    Ok(())
}

fn joint3_gravity_unload_feedforward(level: usize) -> Result<f32> {
    anyhow::ensure!(
        (1..=J3_GRAVITY_UNLOAD_LEVELS).contains(&level),
        "joint_3 gravity-unload level {level} is outside 1..={J3_GRAVITY_UNLOAD_LEVELS}"
    );
    let magnitude = level as f32 * J3_GRAVITY_UNLOAD_STEP_NM;
    anyhow::ensure!(
        magnitude <= J3_GRAVITY_UNLOAD_CAP_NM,
        "joint_3 gravity-unload level {level} exceeds the fixed {J3_GRAVITY_UNLOAD_CAP_NM:.3} Nm cap"
    );
    Ok(-magnitude)
}

fn joint3_gravity_unload_should_censor(observation: DiagnosticObservation) -> bool {
    observation.telemetry.measured_delta <= J3_GRAVITY_UNLOAD_TRIGGER_RAD
        || observation.telemetry.measured_velocity <= J3_GRAVITY_UNLOAD_TRIGGER_VELOCITY_RAD_S
}

fn build_joint3_gravity_unload_targets(
    profile: &HardwareProfile,
    mut baseline_targets: [MotorTarget; DOF],
    hold_position_rad: f32,
    feedforward_nm: f32,
) -> Result<[MotorTarget; DOF]> {
    anyhow::ensure!(
        feedforward_nm.is_finite()
            && feedforward_nm.is_sign_negative()
            && (J3_GRAVITY_UNLOAD_STEP_NM..=J3_GRAVITY_UNLOAD_CAP_NM)
                .contains(&feedforward_nm.abs()),
        "joint_3 gravity-unload feed-forward must be negative with magnitude in [{J3_GRAVITY_UNLOAD_STEP_NM:.3}, {J3_GRAVITY_UNLOAD_CAP_NM:.3}] Nm"
    );
    let joint = &profile.joints[J3_GRAVITY_UNLOAD_INDEX];
    baseline_targets[J3_GRAVITY_UNLOAD_INDEX] = ros_target_to_motor(
        RosTarget {
            position_rad: hold_position_rad,
            velocity_rad_s: 0.0,
            torque_nm: feedforward_nm,
            kp_nm_rad: J3_GRAVITY_UNLOAD_EXPECTED_KP,
            kd_nm_s_rad: J3_GRAVITY_UNLOAD_EXPECTED_KD,
        },
        joint,
    );
    Ok(baseline_targets)
}

#[allow(clippy::too_many_arguments)]
fn joint3_gravity_unload_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
) -> Result<DiagnosticObservation> {
    let observation = fixed_position_observation(
        profile,
        feedback,
        J3_GRAVITY_UNLOAD_INDEX,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        initial_q[J3_GRAVITY_UNLOAD_INDEX],
        "joint_3 gravity-unload",
        J3_GRAVITY_UNLOAD_INDEX,
        0.0,
        J3_GRAVITY_UNLOAD_HARD_NEGATIVE_RAD,
        J3_GRAVITY_UNLOAD_HARD_POSITIVE_RAD,
        J3_GRAVITY_UNLOAD_HARD_VELOCITY_RAD_S,
        J3_GRAVITY_UNLOAD_CAP_NM,
        0.0,
        J3_GRAVITY_UNLOAD_MAX_TOTAL_TORQUE_NM,
        J3_GRAVITY_UNLOAD_EXPECTED_KP,
        J3_GRAVITY_UNLOAD_EXPECTED_KD,
    )?;
    anyhow::ensure!(
        observation.telemetry.measured_delta > J3_GRAVITY_UNLOAD_HARD_NEGATIVE_RAD
            && observation.telemetry.measured_delta < J3_GRAVITY_UNLOAD_HARD_POSITIVE_RAD,
        "joint_3 gravity-unload displacement {:.6} rad reached its open hard layer ({:.6}, {:.6}) rad",
        observation.telemetry.measured_delta,
        J3_GRAVITY_UNLOAD_HARD_NEGATIVE_RAD,
        J3_GRAVITY_UNLOAD_HARD_POSITIVE_RAD
    );
    anyhow::ensure!(
        observation.telemetry.measured_velocity.abs() < J3_GRAVITY_UNLOAD_HARD_VELOCITY_RAD_S,
        "joint_3 gravity-unload velocity {:.6} rad/s reached the {:.6} rad/s hard layer",
        observation.telemetry.measured_velocity,
        J3_GRAVITY_UNLOAD_HARD_VELOCITY_RAD_S
    );
    Ok(observation)
}

#[allow(clippy::too_many_arguments)]
async fn wait_for_joint3_gravity_unload_enable_stability(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    targets: [MotorTarget; DOF],
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<[MotorTarget; DOF]> {
    let joint = &profile.joints[J3_GRAVITY_UNLOAD_INDEX];
    let started = Instant::now();
    let mut stability = ContinuousStabilityGate::new(STABILITY_DWELL, ENABLE_STABILITY_TIMEOUT);
    loop {
        validate_joint3_gravity_unload_active_time(diagnostic_started, "enable stability")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_3 gravity-unload enable stability"
        );
        backend
            .ensure_single_axis_commissioning_state(J3_GRAVITY_UNLOAD_INDEX)
            .context("six-axis state contract failed during joint_3 enable stability")?;
        let observation = joint3_gravity_unload_observation(
            profile,
            &backend.feedback(),
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[J3_GRAVITY_UNLOAD_INDEX],
        )?;
        peaks.observe(observation);
        backend.set_targets(targets).await?;
        match stability.observe(
            started.elapsed(),
            observation.telemetry.measured_delta.abs() <= 0.00020
                && observation.telemetry.measured_velocity.abs() <= 0.002,
        ) {
            StabilityGateStatus::Stable => {
                log_diagnostic_telemetry(
                    "go_dwell_complete",
                    PHASE_J3_GRAVITY_UNLOAD,
                    joint,
                    0,
                    J3_GRAVITY_UNLOAD_LEVELS,
                    0.0,
                    0.0,
                    diagnostic_started.elapsed(),
                    observation,
                );
                return Ok(targets);
            }
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_3 did not remain within 0.000200 rad and 0.002000 rad/s for {:.3} s during zero-feed-forward enable stability",
                STABILITY_DWELL.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_joint3_gravity_unload_ramp(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let joint = &profile.joints[J3_GRAVITY_UNLOAD_INDEX];
    let fixed_position_code =
        compressed_target_position_code(&baseline_targets[J3_GRAVITY_UNLOAD_INDEX], joint);
    let baseline_words = compressed_target_words(&baseline_targets[J3_GRAVITY_UNLOAD_INDEX], joint);
    let mut previous_feedback_at = backend
        .feedback()
        .oldest_tpdo1_at
        .context("joint_3 gravity-unload ramp requires an initial all-axis TPDO1 timestamp")?;
    let mut verified_nonzero: Option<Joint3VerifiedGravityTarget> = None;

    for level in 1..=J3_GRAVITY_UNLOAD_LEVELS {
        validate_joint3_gravity_unload_active_time(diagnostic_started, "torque staircase")?;
        let feedforward_nm = joint3_gravity_unload_feedforward(level)?;
        let targets = build_joint3_gravity_unload_targets(
            profile,
            baseline_targets,
            initial_q[J3_GRAVITY_UNLOAD_INDEX],
            feedforward_nm,
        )?;
        let target_words = compressed_target_words(&targets[J3_GRAVITY_UNLOAD_INDEX], joint);
        anyhow::ensure!(
            compressed_target_position_code(&targets[J3_GRAVITY_UNLOAD_INDEX], joint)
                == fixed_position_code,
            "joint_3 gravity-unload level changed the fixed position code"
        );
        let prior_words = verified_nonzero
            .map(|verified| {
                compressed_target_words(&verified.targets[J3_GRAVITY_UNLOAD_INDEX], joint)
            })
            .unwrap_or(baseline_words);
        anyhow::ensure!(
            target_words != prior_words,
            "joint_3 gravity-unload level {level} did not produce a wire-distinct target"
        );
        peaks.observe(joint3_gravity_unload_observation(
            profile,
            &backend.feedback(),
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[J3_GRAVITY_UNLOAD_INDEX],
        )?);
        backend
            .ensure_single_axis_commissioning_state(J3_GRAVITY_UNLOAD_INDEX)
            .context("six-axis state contract failed before joint_3 gravity publish")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed before joint_3 gravity-unload publish"
        );
        backend.set_targets(targets).await?;
        let published_at = Instant::now();
        let proof_after = published_at + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
        let level_deadline = published_at + J3_GRAVITY_UNLOAD_DWELL;
        let mut feedback_proved = false;
        let mut latest_feedback_at = previous_feedback_at;

        let latest_observation = loop {
            validate_joint3_gravity_unload_active_time(diagnostic_started, "torque-level dwell")?;
            anyhow::ensure!(
                !backend.transport_failed(),
                "CAN transport failed during joint_3 gravity-unload dwell"
            );
            backend
                .ensure_single_axis_commissioning_state(J3_GRAVITY_UNLOAD_INDEX)
                .context("six-axis state contract failed during joint_3 gravity-unload dwell")?;
            let feedback = backend.feedback();
            let observation = joint3_gravity_unload_observation(
                profile,
                &feedback,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                targets[J3_GRAVITY_UNLOAD_INDEX],
            )?;
            peaks.observe(observation);
            if joint3_gravity_unload_should_censor(observation) {
                log_diagnostic_telemetry(
                    "inward_motion_censor",
                    PHASE_J3_GRAVITY_UNLOAD,
                    joint,
                    level,
                    J3_GRAVITY_UNLOAD_LEVELS,
                    feedforward_nm,
                    0.0,
                    diagnostic_started.elapsed(),
                    observation,
                );
                let Some(frozen) = verified_nonzero else {
                    backend.set_targets(baseline_targets).await.context(
                        "restore zero-feed-forward joint_3 baseline after first-level censor",
                    )?;
                    tracing::warn!(
                        phase = PHASE_J3_GRAVITY_UNLOAD,
                        joint = %joint.name,
                        node_id = joint.node_id,
                        rejected_level = level,
                        rejected_feedforward_nm = feedforward_nm,
                        trigger_position_delta_rad = observation.telemetry.measured_delta,
                        trigger_velocity_rad_s = observation.telemetry.measured_velocity,
                        "joint_3 response was left-censored below the first feedback-proven nonzero level; restored zero-feed-forward baseline and stopped"
                    );
                    return Ok(());
                };
                let frozen_words =
                    compressed_target_words(&frozen.targets[J3_GRAVITY_UNLOAD_INDEX], joint);
                anyhow::ensure!(
                    frozen_words != target_words && frozen_words != baseline_words,
                    "joint_3 gravity-unload rollback is not wire-distinct from both rejected target and zero baseline"
                );
                backend
                    .register_single_axis_diagnostic_baseline(
                        J3_GRAVITY_UNLOAD_INDEX,
                        frozen.targets,
                    )
                    .context("register preceding verified joint_3 gravity target")?;
                backend
                    .set_targets(frozen.targets)
                    .await
                    .context("restore preceding verified joint_3 gravity target")?;
                tracing::warn!(
                    phase = PHASE_J3_GRAVITY_UNLOAD,
                    joint = %joint.name,
                    node_id = joint.node_id,
                    rejected_level = level,
                    rejected_feedforward_nm = feedforward_nm,
                    trigger_position_delta_rad = observation.telemetry.measured_delta,
                    trigger_velocity_rad_s = observation.telemetry.measured_velocity,
                    frozen_verified_level = frozen.level,
                    frozen_verified_feedforward_nm = frozen.feedforward_nm,
                    "joint_3 inward-motion censor rejected the current load and restored the preceding feedback-proven target"
                );
                return run_joint3_gravity_unload_frozen_hold(
                    backend,
                    profile,
                    initial_q,
                    initial_motor_position_rev,
                    temperature_baseline,
                    baseline_targets,
                    frozen,
                    Joint3GravityUnloadStop::MotionCensor {
                        rejected_level: level,
                        rejected_feedforward_nm: feedforward_nm,
                        trigger_position_delta_rad: observation.telemetry.measured_delta,
                        trigger_velocity_rad_s: observation.telemetry.measured_velocity,
                    },
                    Instant::now(),
                    diagnostic_started,
                    peaks,
                )
                .await;
            }
            if let Some(feedback_at) = feedback.oldest_tpdo1_at {
                if feedback_at > previous_feedback_at && feedback_at > proof_after {
                    feedback_proved = true;
                    latest_feedback_at = feedback_at;
                }
            }
            if Instant::now() >= level_deadline && feedback_proved {
                break observation;
            }
            tokio::time::sleep(LOOP_PERIOD).await;
        };

        previous_feedback_at = latest_feedback_at;
        let verified = Joint3VerifiedGravityTarget {
            level,
            feedforward_nm,
            targets,
        };
        backend
            .register_single_axis_diagnostic_baseline(J3_GRAVITY_UNLOAD_INDEX, targets)
            .context("promote completed feedback-proven joint_3 gravity level")?;
        verified_nonzero = Some(verified);
        log_diagnostic_telemetry(
            "level_feedback_proven",
            PHASE_J3_GRAVITY_UNLOAD,
            joint,
            level,
            J3_GRAVITY_UNLOAD_LEVELS,
            feedforward_nm,
            0.0,
            diagnostic_started.elapsed(),
            latest_observation,
        );
        if level == J3_GRAVITY_UNLOAD_LEVELS {
            return run_joint3_gravity_unload_frozen_hold(
                backend,
                profile,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                baseline_targets,
                verified,
                Joint3GravityUnloadStop::TorqueCap,
                Instant::now(),
                diagnostic_started,
                peaks,
            )
            .await;
        }
    }
    unreachable!("fixed joint_3 gravity-unload levels always return at their cap")
}

#[allow(clippy::too_many_arguments)]
async fn run_joint3_gravity_unload_frozen_hold(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    frozen: Joint3VerifiedGravityTarget,
    stop: Joint3GravityUnloadStop,
    freeze_started: Instant,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let joint = &profile.joints[J3_GRAVITY_UNLOAD_INDEX];
    let freeze_deadline = freeze_started + J3_GRAVITY_UNLOAD_FREEZE_DURATION;
    anyhow::ensure!(
        compressed_target_position_code(&frozen.targets[J3_GRAVITY_UNLOAD_INDEX], joint)
            == compressed_target_position_code(&baseline_targets[J3_GRAVITY_UNLOAD_INDEX], joint,),
        "joint_3 frozen gravity target changed the fixed position code"
    );
    anyhow::ensure!(
        compressed_target_words(&frozen.targets[J3_GRAVITY_UNLOAD_INDEX], joint)
            != compressed_target_words(&baseline_targets[J3_GRAVITY_UNLOAD_INDEX], joint,),
        "joint_3 frozen gravity target is not wire-distinct from baseline"
    );
    confirm_joint3_gravity_target_readback_while_guarded(
        backend,
        profile,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        frozen.targets[J3_GRAVITY_UNLOAD_INDEX],
        freeze_deadline,
        diagnostic_started,
        peaks,
    )
    .await?;

    let mut statistics = CensoredHoldStatistics::default();
    let mut last_feedback_at: Option<Instant> = None;
    while Instant::now() < freeze_deadline {
        validate_joint3_gravity_unload_active_time(diagnostic_started, "frozen statistics")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_3 frozen gravity statistics"
        );
        backend
            .ensure_single_axis_commissioning_state(J3_GRAVITY_UNLOAD_INDEX)
            .context("six-axis state contract failed during joint_3 frozen gravity statistics")?;
        let feedback = backend.feedback();
        let observation = joint3_gravity_unload_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            frozen.targets[J3_GRAVITY_UNLOAD_INDEX],
        )?;
        peaks.observe(observation);
        let feedback_at = feedback
            .oldest_tpdo1_at
            .context("joint_3 frozen gravity statistics require all-axis TPDO1 timestamps")?;
        if last_feedback_at.is_none_or(|previous| feedback_at > previous) {
            statistics.observe(feedback_at, observation)?;
            last_feedback_at = Some(feedback_at);
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let summary = statistics.summarize()?;
    let (
        rejected_level,
        rejected_feedforward_nm,
        trigger_position_delta_rad,
        trigger_velocity_rad_s,
    ) = match stop {
        Joint3GravityUnloadStop::MotionCensor {
            rejected_level,
            rejected_feedforward_nm,
            trigger_position_delta_rad,
            trigger_velocity_rad_s,
        } => (
            Some(rejected_level),
            Some(rejected_feedforward_nm),
            Some(trigger_position_delta_rad),
            Some(trigger_velocity_rad_s),
        ),
        Joint3GravityUnloadStop::TorqueCap => (None, None, None, None),
    };
    tracing::info!(
        phase = PHASE_J3_GRAVITY_UNLOAD,
        joint = %joint.name,
        node_id = joint.node_id,
        stop_reason = stop.label(),
        rejected_level = ?rejected_level,
        rejected_feedforward_nm = ?rejected_feedforward_nm,
        trigger_position_delta_rad = ?trigger_position_delta_rad,
        trigger_velocity_rad_s = ?trigger_velocity_rad_s,
        frozen_verified_level = frozen.level,
        frozen_verified_feedforward_nm = frozen.feedforward_nm,
        sample_count = summary.sample_count,
        mean_position_delta_rad = summary.mean_position_delta_rad,
        position_stddev_rad = summary.position_stddev_rad,
        minimum_position_delta_rad = summary.minimum_position_delta_rad,
        maximum_position_delta_rad = summary.maximum_position_delta_rad,
        peak_velocity_rad_s = summary.peak_velocity_rad_s,
        mean_measured_torque_nm = summary.mean_measured_torque_nm,
        mean_estimated_total_torque_nm = summary.mean_estimated_total_torque_nm,
        peak_driver_temperature_c = summary.peak_driver_temperature_c,
        peak_motor_temperature_c = summary.peak_motor_temperature_c,
        terminal_sample_count = summary.terminal_sample_count,
        terminal_position_span_rad = summary.terminal_position_span_rad,
        terminal_peak_velocity_rad_s = summary.terminal_peak_velocity_rad_s,
        terminal_stable = summary.terminal_stable,
        "joint_3 fixed-position gravity-unload statistics completed"
    );
    anyhow::ensure!(
        summary.terminal_stable,
        "joint_3 frozen gravity target was not terminally stable: final span {:.6} rad, peak velocity {:.6} rad/s, samples {}",
        summary.terminal_position_span_rad,
        summary.terminal_peak_velocity_rad_s,
        summary.terminal_sample_count
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn confirm_joint3_gravity_target_readback_while_guarded(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    freeze_deadline: Instant,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let poll = || -> Result<DiagnosticObservation> {
        anyhow::ensure!(
            Instant::now() < freeze_deadline,
            "joint_3 exact frozen-target readback exceeded its one-second hold"
        );
        validate_joint3_gravity_unload_active_time(diagnostic_started, "target readback")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_3 gravity-unload target readback"
        );
        backend
            .ensure_single_axis_commissioning_state(J3_GRAVITY_UNLOAD_INDEX)
            .context("six-axis state contract failed during joint_3 gravity readback")?;
        joint3_gravity_unload_observation(
            profile,
            &backend.feedback(),
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            target,
        )
    };
    let settle_started = Instant::now();
    while settle_started.elapsed() < RPDO_READBACK_SETTLE {
        peaks.observe(poll()?);
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let mut readback_future =
        Box::pin(backend.confirm_commissioning_target_readback(J3_GRAVITY_UNLOAD_INDEX, target));
    let readback = loop {
        tokio::select! {
            result = &mut readback_future => {
                break result.context("joint_3 gravity-unload exact target readback failed")?;
            }
            _ = tokio::time::sleep(LOOP_PERIOD) => {
                peaks.observe(poll()?);
            }
        }
    };
    peaks.observe(poll()?);
    let joint = &profile.joints[J3_GRAVITY_UNLOAD_INDEX];
    tracing::info!(
        phase = PHASE_J3_GRAVITY_UNLOAD,
        milestone = "frozen_feedback_proven_target",
        joint = %joint.name,
        node_id = joint.node_id,
        expected_lower = %format_args!("0x{:08X}", readback.expected_lower),
        expected_upper = %format_args!("0x{:08X}", readback.expected_upper),
        actual_lower = %format_args!("0x{:08X}", readback.actual_lower),
        actual_upper = %format_args!("0x{:08X}", readback.actual_upper),
        "joint_3 frozen gravity target matched exact 0x2004:02/03 readback"
    );
    Ok(())
}

fn validate_joint4_censored_torque_active_time(
    diagnostic_started: Instant,
    phase: &str,
) -> Result<()> {
    anyhow::ensure!(
        diagnostic_started.elapsed().as_secs_f32() <= J4_CENSORED_TORQUE_MAX_ACTIVE_SEC,
        "joint_4 censored-torque diagnostic exceeded its {J4_CENSORED_TORQUE_MAX_ACTIVE_SEC:.3} s confirmed-active limit during {phase}"
    );
    Ok(())
}

fn joint4_censored_torque_additive(level: usize) -> Result<f32> {
    anyhow::ensure!(
        (1..=J4_CENSORED_TORQUE_LEVELS).contains(&level),
        "joint_4 censored-torque level {level} is outside 1..={J4_CENSORED_TORQUE_LEVELS}"
    );
    let requested_magnitude = level as f32 * J4_CENSORED_TORQUE_STEP_NM;
    anyhow::ensure!(
        requested_magnitude <= J4_CENSORED_TORQUE_CAP_NM + f32::EPSILON,
        "joint_4 censored-torque level {level} exceeds the fixed {J4_CENSORED_TORQUE_CAP_NM:.3} Nm cap"
    );
    let additive_torque_nm = -requested_magnitude.min(J4_CENSORED_TORQUE_CAP_NM);
    Ok(additive_torque_nm)
}

fn joint4_censored_torque_should_censor(measured_delta_rad: f32) -> bool {
    measured_delta_rad <= J4_CENSORED_TORQUE_TRIGGER_RAD
}

fn build_joint4_censored_torque_targets(
    profile: &HardwareProfile,
    mut baseline_targets: [MotorTarget; DOF],
    hold_position_rad: f32,
    model_feedforward_nm: f32,
    additive_torque_nm: f32,
) -> Result<[MotorTarget; DOF]> {
    anyhow::ensure!(
        additive_torque_nm.is_finite()
            && additive_torque_nm.is_sign_negative()
            && (J4_CENSORED_TORQUE_STEP_NM..=J4_CENSORED_TORQUE_CAP_NM)
                .contains(&additive_torque_nm.abs()),
        "joint_4 additive torque must be negative with magnitude in [{J4_CENSORED_TORQUE_STEP_NM:.3}, {J4_CENSORED_TORQUE_CAP_NM:.3}] Nm"
    );
    anyhow::ensure!(
        model_feedforward_nm.abs() <= J4_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        "joint_4 censored-torque model feed-forward {model_feedforward_nm:.6} Nm is not the required zero baseline"
    );
    let total_feedforward_nm = model_feedforward_nm + additive_torque_nm;
    anyhow::ensure!(
        total_feedforward_nm.abs() <= J4_CENSORED_TORQUE_MAX_FEEDFORWARD_NM,
        "joint_4 censored-torque feed-forward {total_feedforward_nm:.6} Nm exceeds {J4_CENSORED_TORQUE_MAX_FEEDFORWARD_NM:.6} Nm"
    );
    let joint = &profile.joints[J4_FIRST_POSITION_INDEX];
    baseline_targets[J4_FIRST_POSITION_INDEX] = ros_target_to_motor(
        RosTarget {
            position_rad: hold_position_rad,
            velocity_rad_s: 0.0,
            torque_nm: total_feedforward_nm,
            kp_nm_rad: J4_FIRST_POSITION_EXPECTED_KP,
            kd_nm_s_rad: J4_FIRST_POSITION_EXPECTED_KD,
        },
        joint,
    );
    Ok(baseline_targets)
}

#[allow(clippy::too_many_arguments)]
fn joint4_censored_torque_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
) -> Result<DiagnosticObservation> {
    let observation = fixed_position_observation(
        profile,
        feedback,
        J4_FIRST_POSITION_INDEX,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        initial_q[J4_FIRST_POSITION_INDEX],
        "joint_4 censored-torque",
        J4_FIRST_POSITION_INDEX,
        0.0,
        J4_CENSORED_TORQUE_HARD_NEGATIVE_RAD,
        J4_CENSORED_TORQUE_HARD_POSITIVE_RAD,
        J4_CENSORED_TORQUE_HARD_VELOCITY_RAD_S,
        J4_CENSORED_TORQUE_MAX_FEEDFORWARD_NM,
        0.0,
        J4_CENSORED_TORQUE_MAX_TOTAL_TORQUE_NM,
        J4_FIRST_POSITION_EXPECTED_KP,
        J4_FIRST_POSITION_EXPECTED_KD,
    )?;
    anyhow::ensure!(
        observation.telemetry.measured_delta > J4_CENSORED_TORQUE_HARD_NEGATIVE_RAD
            && observation.telemetry.measured_delta < J4_CENSORED_TORQUE_HARD_POSITIVE_RAD,
        "joint_4 censored-torque displacement {:.6} rad reached its open hard layer ({:.6}, {:.6}) rad",
        observation.telemetry.measured_delta,
        J4_CENSORED_TORQUE_HARD_NEGATIVE_RAD,
        J4_CENSORED_TORQUE_HARD_POSITIVE_RAD
    );
    anyhow::ensure!(
        observation.telemetry.measured_velocity.abs() < J4_CENSORED_TORQUE_HARD_VELOCITY_RAD_S,
        "joint_4 censored-torque velocity {:.6} rad/s reached the {:.6} rad/s hard layer",
        observation.telemetry.measured_velocity,
        J4_CENSORED_TORQUE_HARD_VELOCITY_RAD_S
    );
    Ok(observation)
}

#[allow(clippy::too_many_arguments)]
async fn run_joint4_censored_torque_ramp(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    model_feedforward_nm: f32,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let selected_index = J4_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let fixed_position_code =
        compressed_target_position_code(&baseline_targets[selected_index], joint);
    let baseline_words = compressed_target_words(&baseline_targets[selected_index], joint);
    let mut previous_feedback_at = backend
        .feedback()
        .oldest_tpdo1_at
        .context("joint_4 censored-torque ramp requires an initial all-axis TPDO1 timestamp")?;
    let mut verified_nonzero: Option<Joint4VerifiedTorqueTarget> = None;

    for level in 1..=J4_CENSORED_TORQUE_LEVELS {
        validate_joint4_censored_torque_active_time(diagnostic_started, "torque staircase")?;
        let additive_torque_nm = joint4_censored_torque_additive(level)?;
        let targets = build_joint4_censored_torque_targets(
            profile,
            baseline_targets,
            initial_q[selected_index],
            model_feedforward_nm,
            additive_torque_nm,
        )?;
        let target_words = compressed_target_words(&targets[selected_index], joint);
        anyhow::ensure!(
            compressed_target_position_code(&targets[selected_index], joint) == fixed_position_code,
            "joint_4 censored-torque level changed the fixed position code"
        );
        let prior_words = verified_nonzero
            .map(|verified| compressed_target_words(&verified.targets[selected_index], joint))
            .unwrap_or(baseline_words);
        anyhow::ensure!(
            target_words != prior_words,
            "joint_4 censored-torque level {level} did not produce a wire-distinct target"
        );
        let prospective_feedback = backend.feedback();
        let prospective = joint4_censored_torque_observation(
            profile,
            &prospective_feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
        )?;
        peaks.observe(prospective);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_4 torque publish")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed before joint_4 censored-torque publish"
        );
        backend.set_targets(targets).await?;
        let published_at = Instant::now();
        let proof_after = published_at + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
        let level_deadline = published_at + J4_CENSORED_TORQUE_DWELL;
        let mut feedback_proved = false;
        let mut latest_feedback_at = previous_feedback_at;

        let latest_observation = loop {
            validate_joint4_censored_torque_active_time(diagnostic_started, "torque-level dwell")?;
            anyhow::ensure!(
                !backend.transport_failed(),
                "CAN transport failed during joint_4 censored-torque dwell"
            );
            backend
                .ensure_single_axis_commissioning_state(selected_index)
                .context("six-axis state contract failed during joint_4 censored-torque dwell")?;
            let feedback = backend.feedback();
            let observation = joint4_censored_torque_observation(
                profile,
                &feedback,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                targets[selected_index],
            )?;
            peaks.observe(observation);
            if joint4_censored_torque_should_censor(observation.telemetry.measured_delta) {
                let frozen = verified_nonzero.context(
                    "joint_4 reached -0.30 mrad before any nonzero wire-distinct torque level completed its feedback-proven dwell",
                )?;
                let frozen_words = compressed_target_words(&frozen.targets[selected_index], joint);
                anyhow::ensure!(
                    frozen_words != target_words && frozen_words != baseline_words,
                    "joint_4 censored-torque rollback is not wire-distinct from both rejected target and zero-additive baseline"
                );
                backend
                    .register_single_axis_diagnostic_baseline(selected_index, frozen.targets)
                    .context("register preceding verified joint_4 torque target before rollback")?;
                backend
                    .set_targets(frozen.targets)
                    .await
                    .context("restore preceding verified joint_4 torque target")?;
                let freeze_started = Instant::now();
                tracing::warn!(
                    phase = PHASE_J4_CENSORED_TORQUE,
                    joint = %joint.name,
                    node_id = joint.node_id,
                    rejected_level = level,
                    rejected_additive_torque_nm = additive_torque_nm,
                    trigger_position_delta_rad = observation.telemetry.measured_delta,
                    trigger_velocity_rad_s = observation.telemetry.measured_velocity,
                    frozen_verified_level = frozen.level,
                    frozen_verified_additive_torque_nm = frozen.additive_torque_nm,
                    "joint_4 negative displacement censor rejected the current torque and restored the preceding feedback-proven target"
                );
                return run_joint4_censored_torque_frozen_hold(
                    backend,
                    profile,
                    initial_q,
                    initial_motor_position_rev,
                    temperature_baseline,
                    baseline_targets,
                    frozen,
                    Joint4CensoredTorqueStop::DisplacementCensor {
                        rejected_level: level,
                        rejected_additive_torque_nm: additive_torque_nm,
                        trigger_position_delta_rad: observation.telemetry.measured_delta,
                        trigger_velocity_rad_s: observation.telemetry.measured_velocity,
                    },
                    freeze_started,
                    diagnostic_started,
                    peaks,
                )
                .await;
            }
            if let Some(feedback_at) = feedback.oldest_tpdo1_at {
                if feedback_at > previous_feedback_at && feedback_at > proof_after {
                    feedback_proved = true;
                    latest_feedback_at = feedback_at;
                }
            }
            if Instant::now() >= level_deadline && feedback_proved {
                break observation;
            }
            tokio::time::sleep(LOOP_PERIOD).await;
        };

        previous_feedback_at = latest_feedback_at;
        let verified = Joint4VerifiedTorqueTarget {
            level,
            additive_torque_nm,
            targets,
        };
        backend
            .register_single_axis_diagnostic_baseline(selected_index, targets)
            .context("promote completed feedback-proven joint_4 torque level")?;
        verified_nonzero = Some(verified);
        log_diagnostic_telemetry(
            "level_feedback_proven",
            PHASE_J4_CENSORED_TORQUE,
            joint,
            level,
            J4_CENSORED_TORQUE_LEVELS,
            model_feedforward_nm,
            additive_torque_nm,
            diagnostic_started.elapsed(),
            latest_observation,
        );
        if level == J4_CENSORED_TORQUE_LEVELS {
            return run_joint4_censored_torque_frozen_hold(
                backend,
                profile,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                baseline_targets,
                verified,
                Joint4CensoredTorqueStop::TorqueCap,
                Instant::now(),
                diagnostic_started,
                peaks,
            )
            .await;
        }
    }
    unreachable!("fixed joint_4 censored-torque levels always return at their cap")
}

#[allow(clippy::too_many_arguments)]
async fn run_joint4_censored_torque_frozen_hold(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    frozen: Joint4VerifiedTorqueTarget,
    stop: Joint4CensoredTorqueStop,
    freeze_started: Instant,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let selected_index = J4_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let freeze_deadline = freeze_started + J4_CENSORED_TORQUE_FREEZE_DURATION;
    anyhow::ensure!(
        compressed_target_position_code(&frozen.targets[selected_index], joint)
            == compressed_target_position_code(&baseline_targets[selected_index], joint),
        "joint_4 frozen torque target changed the fixed position code"
    );
    anyhow::ensure!(
        compressed_target_words(&frozen.targets[selected_index], joint)
            != compressed_target_words(&baseline_targets[selected_index], joint),
        "joint_4 frozen torque target is not wire-distinct from baseline"
    );
    confirm_joint4_censored_torque_target_readback_while_guarded(
        backend,
        profile,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        frozen.targets[selected_index],
        freeze_deadline,
        diagnostic_started,
        peaks,
    )
    .await?;

    let mut statistics = CensoredHoldStatistics::default();
    let mut last_feedback_at: Option<Instant> = None;
    while Instant::now() < freeze_deadline {
        validate_joint4_censored_torque_active_time(diagnostic_started, "frozen statistics")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_4 frozen torque statistics"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_4 frozen torque statistics")?;
        let feedback = backend.feedback();
        let observation = joint4_censored_torque_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            frozen.targets[selected_index],
        )?;
        peaks.observe(observation);
        let feedback_at = feedback
            .oldest_tpdo1_at
            .context("joint_4 frozen torque statistics require all-axis TPDO1 timestamps")?;
        if last_feedback_at.is_none_or(|previous| feedback_at > previous) {
            statistics.observe(feedback_at, observation)?;
            last_feedback_at = Some(feedback_at);
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let summary = statistics.summarize()?;
    let (
        rejected_level,
        rejected_additive_torque_nm,
        trigger_position_delta_rad,
        trigger_velocity_rad_s,
    ) = match stop {
        Joint4CensoredTorqueStop::DisplacementCensor {
            rejected_level,
            rejected_additive_torque_nm,
            trigger_position_delta_rad,
            trigger_velocity_rad_s,
        } => (
            Some(rejected_level),
            Some(rejected_additive_torque_nm),
            Some(trigger_position_delta_rad),
            Some(trigger_velocity_rad_s),
        ),
        Joint4CensoredTorqueStop::TorqueCap => (None, None, None, None),
    };
    tracing::info!(
        phase = PHASE_J4_CENSORED_TORQUE,
        joint = %joint.name,
        node_id = joint.node_id,
        stop_reason = stop.label(),
        rejected_level = ?rejected_level,
        rejected_additive_torque_nm = ?rejected_additive_torque_nm,
        trigger_position_delta_rad = ?trigger_position_delta_rad,
        trigger_velocity_rad_s = ?trigger_velocity_rad_s,
        frozen_verified_level = frozen.level,
        frozen_verified_additive_torque_nm = frozen.additive_torque_nm,
        sample_count = summary.sample_count,
        mean_position_delta_rad = summary.mean_position_delta_rad,
        position_stddev_rad = summary.position_stddev_rad,
        minimum_position_delta_rad = summary.minimum_position_delta_rad,
        maximum_position_delta_rad = summary.maximum_position_delta_rad,
        peak_velocity_rad_s = summary.peak_velocity_rad_s,
        mean_measured_torque_nm = summary.mean_measured_torque_nm,
        mean_estimated_total_torque_nm = summary.mean_estimated_total_torque_nm,
        peak_driver_temperature_c = summary.peak_driver_temperature_c,
        peak_motor_temperature_c = summary.peak_motor_temperature_c,
        terminal_sample_count = summary.terminal_sample_count,
        terminal_position_span_rad = summary.terminal_position_span_rad,
        terminal_peak_velocity_rad_s = summary.terminal_peak_velocity_rad_s,
        terminal_stable = summary.terminal_stable,
        "joint_4 fixed-position censored-torque statistics completed"
    );
    anyhow::ensure!(
        summary.terminal_stable,
        "joint_4 frozen torque target was not terminally stable: final span {:.6} rad, peak velocity {:.6} rad/s, samples {}",
        summary.terminal_position_span_rad,
        summary.terminal_peak_velocity_rad_s,
        summary.terminal_sample_count
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn confirm_joint4_censored_torque_target_readback_while_guarded(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    freeze_deadline: Instant,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let poll = || -> Result<DiagnosticObservation> {
        anyhow::ensure!(
            Instant::now() < freeze_deadline,
            "joint_4 exact frozen-target readback exceeded its one-second hold"
        );
        validate_joint4_censored_torque_active_time(diagnostic_started, "target readback")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_4 censored-torque target readback"
        );
        backend
            .ensure_single_axis_commissioning_state(J4_FIRST_POSITION_INDEX)
            .context("six-axis state contract failed during joint_4 torque readback")?;
        joint4_censored_torque_observation(
            profile,
            &backend.feedback(),
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            target,
        )
    };
    let settle_started = Instant::now();
    while settle_started.elapsed() < RPDO_READBACK_SETTLE {
        peaks.observe(poll()?);
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let mut readback_future =
        Box::pin(backend.confirm_commissioning_target_readback(J4_FIRST_POSITION_INDEX, target));
    let readback = loop {
        tokio::select! {
            result = &mut readback_future => {
                break result.context("joint_4 censored-torque exact target readback failed")?;
            }
            _ = tokio::time::sleep(LOOP_PERIOD) => {
                peaks.observe(poll()?);
            }
        }
    };
    peaks.observe(poll()?);
    let joint = &profile.joints[J4_FIRST_POSITION_INDEX];
    tracing::info!(
        phase = PHASE_J4_CENSORED_TORQUE,
        milestone = "frozen_feedback_proven_target",
        joint = %joint.name,
        node_id = joint.node_id,
        expected_lower = %format_args!("0x{:08X}", readback.expected_lower),
        expected_upper = %format_args!("0x{:08X}", readback.expected_upper),
        actual_lower = %format_args!("0x{:08X}", readback.actual_lower),
        actual_upper = %format_args!("0x{:08X}", readback.actual_upper),
        "joint_4 frozen torque target matched exact 0x2004:02/03 readback"
    );
    Ok(())
}

fn validate_joint6_first_position_active_time(
    diagnostic_started: Instant,
    phase: &str,
) -> Result<()> {
    anyhow::ensure!(
        diagnostic_started.elapsed().as_secs_f32() <= J6_FIRST_POSITION_MAX_ACTIVE_SEC,
        "joint_6 first-position diagnostic exceeded {J6_FIRST_POSITION_MAX_ACTIVE_SEC:.3} s during {phase}"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn joint6_first_position_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
) -> Result<DiagnosticObservation> {
    fixed_position_observation(
        profile,
        feedback,
        J6_FIRST_POSITION_INDEX,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        commanded_q,
        "joint_6",
        J6_FIRST_POSITION_INDEX,
        J6_FIRST_POSITION_DELTA_RAD,
        J6_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD,
        J6_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD,
        J6_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S,
        J6_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        J6_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        J6_FIRST_POSITION_MAX_TOTAL_TORQUE_NM,
        expected_kp_nm_rad,
        expected_kd_nm_s_rad,
    )
}

#[allow(clippy::too_many_arguments)]
async fn wait_for_joint6_first_position_enable_stability(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<[MotorTarget; DOF]> {
    let selected_index = J6_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let started = Instant::now();
    let mut stability = ContinuousStabilityGate::new(STABILITY_DWELL, ENABLE_STABILITY_TIMEOUT);
    loop {
        validate_joint6_first_position_active_time(diagnostic_started, "enable stability")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_6 enable stability"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_6 enable stability")?;
        let feedback = backend.feedback();
        let measured_q = validate_feedback(profile, &feedback, None)?;
        let targets = build_safe_hold_targets(
            profile,
            dynamics,
            &measured_q,
            selected_index,
            initial_q[selected_index],
        )?;
        let observation = joint6_first_position_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
            initial_q[selected_index],
            J6_FIRST_POSITION_EXPECTED_KP,
            J6_FIRST_POSITION_EXPECTED_KD,
        )?;
        anyhow::ensure!(
            observation.telemetry.measured_delta.abs()
                <= J6_FIRST_POSITION_ENABLE_HARD_POSITION_RAD,
            "joint_6 enable displacement {:.6} rad exceeds {:.6} rad",
            observation.telemetry.measured_delta,
            J6_FIRST_POSITION_ENABLE_HARD_POSITION_RAD
        );
        peaks.observe(observation);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_6 stability publish")?;
        backend.set_targets(targets).await?;
        match stability.observe(
            started.elapsed(),
            observation.telemetry.measured_delta.abs() <= J6_FIRST_POSITION_GO_POSITION_RAD
                && observation.telemetry.measured_velocity.abs()
                    <= J6_FIRST_POSITION_GO_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => {
                log_diagnostic_telemetry(
                    "go_dwell_complete",
                    PHASE_J6_FIRST_POSITION_ENABLE,
                    joint,
                    0,
                    1,
                    observation.telemetry.gravity_ff,
                    0.0,
                    diagnostic_started.elapsed(),
                    observation,
                );
                return Ok(targets);
            }
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_6 did not satisfy the {:.6} rad / {:.6} rad/s GO gate continuously for {:.3} s",
                J6_FIRST_POSITION_GO_POSITION_RAD,
                J6_FIRST_POSITION_GO_VELOCITY_RAD_S,
                STABILITY_DWELL.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

#[allow(clippy::too_many_arguments)]
async fn confirm_joint6_first_position_target_readback_while_guarded(
    backend: &RealBackend,
    profile: &HardwareProfile,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    diagnostic_started: Instant,
    milestone: &'static str,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let poll = || -> Result<DiagnosticObservation> {
        validate_joint6_first_position_active_time(diagnostic_started, "SDO target readback")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_6 target readback"
        );
        backend
            .ensure_single_axis_commissioning_state(J6_FIRST_POSITION_INDEX)
            .context("six-axis state contract failed during joint_6 target readback")?;
        joint6_first_position_observation(
            profile,
            &backend.feedback(),
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            target,
            commanded_q,
            expected_kp_nm_rad,
            expected_kd_nm_s_rad,
        )
    };
    let settle_started = Instant::now();
    while settle_started.elapsed() < RPDO_READBACK_SETTLE {
        peaks.observe(poll()?);
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let mut readback_future =
        Box::pin(backend.confirm_commissioning_target_readback(J6_FIRST_POSITION_INDEX, target));
    let readback = loop {
        tokio::select! {
            result = &mut readback_future => {
                break result.context("joint_6 exact compressed-MIT target readback failed")?;
            }
            _ = tokio::time::sleep(LOOP_PERIOD) => {
                peaks.observe(poll()?);
            }
        }
    };
    peaks.observe(poll()?);
    let joint = &profile.joints[J6_FIRST_POSITION_INDEX];
    tracing::info!(
        phase = PHASE_J6_FIRST_POSITION_TRAJECTORY,
        milestone,
        joint = %joint.name,
        node_id = joint.node_id,
        commanded_q,
        expected_lower = %format_args!("0x{:08X}", readback.expected_lower),
        expected_upper = %format_args!("0x{:08X}", readback.expected_upper),
        actual_lower = %format_args!("0x{:08X}", readback.actual_lower),
        actual_upper = %format_args!("0x{:08X}", readback.actual_upper),
        "joint_6 held target matched exact 0x2004:02/03 readback"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn run_joint6_first_position_round_trip(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    diagnostic_started: Instant,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let selected_index = J6_FIRST_POSITION_INDEX;
    let joint = &profile.joints[selected_index];
    let baseline_position_code =
        compressed_target_position_code(&baseline_targets[selected_index], joint);
    let trajectory_started = Instant::now();
    let trajectory_duration = Duration::from_secs_f32(J6_FIRST_POSITION_DURATION_SEC);
    let mut readback_pause = Duration::ZERO;
    let mut changed_target_readback = false;
    let mut most_negative_delta_rad = 0.0_f32;
    let mut most_negative_raw_delta_rev = 0.0_f32;
    let mut milestones = MotionTelemetryMilestones::default();
    loop {
        let trajectory_elapsed = trajectory_started.elapsed().saturating_sub(readback_pause);
        if trajectory_elapsed >= trajectory_duration {
            break;
        }
        validate_joint6_first_position_active_time(diagnostic_started, "trajectory")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_6 trajectory"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_6 trajectory")?;
        let feedback = backend.feedback();
        let measured_q = validate_feedback(profile, &feedback, None)?;
        let normalized_time = trajectory_elapsed.as_secs_f32() / J6_FIRST_POSITION_DURATION_SEC;
        let commanded_q = initial_q[selected_index]
            + round_trip_phase(normalized_time) * J6_FIRST_POSITION_DELTA_RAD;
        let targets = build_joint6_first_position_trajectory_targets(
            profile,
            dynamics,
            &measured_q,
            commanded_q,
        )?;
        let observation = joint6_first_position_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            targets[selected_index],
            commanded_q,
            J6_FIRST_POSITION_TRAJECTORY_KP,
            J6_FIRST_POSITION_TRAJECTORY_KD,
        )?;
        peaks.observe(observation);
        most_negative_delta_rad = most_negative_delta_rad.min(observation.telemetry.measured_delta);
        most_negative_raw_delta_rev = most_negative_raw_delta_rev
            .min(feedback.joints[selected_index].position_rev - initial_motor_position_rev);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed before joint_6 target publish")?;
        backend.set_targets(targets).await?;
        while let Some(milestone) = milestones.take_due(normalized_time) {
            log_diagnostic_telemetry(
                milestone,
                PHASE_J6_FIRST_POSITION_TRAJECTORY,
                joint,
                1,
                1,
                observation.telemetry.gravity_ff,
                0.0,
                diagnostic_started.elapsed(),
                observation,
            );
        }
        let position_code = compressed_target_position_code(&targets[selected_index], joint);
        if !changed_target_readback && position_code != baseline_position_code {
            let readback_started = Instant::now();
            confirm_joint6_first_position_target_readback_while_guarded(
                backend,
                profile,
                initial_q,
                initial_motor_position_rev,
                temperature_baseline,
                targets[selected_index],
                commanded_q,
                diagnostic_started,
                "first_quantized_changed_position",
                J6_FIRST_POSITION_TRAJECTORY_KP,
                J6_FIRST_POSITION_TRAJECTORY_KD,
                peaks,
            )
            .await?;
            readback_pause += readback_started.elapsed();
            changed_target_readback = true;
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    anyhow::ensure!(
        changed_target_readback,
        "joint_6 survey completed without exact readback of a changed position code"
    );
    backend.set_targets(baseline_targets).await?;
    let return_started = Instant::now();
    let mut return_stability = ContinuousStabilityGate::new(STABILITY_DWELL, RETURN_SETTLE_TIMEOUT);
    let (return_error_rad, return_velocity_rad_s) = loop {
        validate_joint6_first_position_active_time(diagnostic_started, "return settle")?;
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("six-axis state contract failed during joint_6 return settle")?;
        let feedback = backend.feedback();
        let observation = joint6_first_position_observation(
            profile,
            &feedback,
            initial_q,
            initial_motor_position_rev,
            temperature_baseline,
            baseline_targets[selected_index],
            initial_q[selected_index],
            J6_FIRST_POSITION_EXPECTED_KP,
            J6_FIRST_POSITION_EXPECTED_KD,
        )?;
        peaks.observe(observation);
        most_negative_delta_rad = most_negative_delta_rad.min(observation.telemetry.measured_delta);
        most_negative_raw_delta_rev = most_negative_raw_delta_rev
            .min(feedback.joints[selected_index].position_rev - initial_motor_position_rev);
        backend.set_targets(baseline_targets).await?;
        let return_error = observation.telemetry.measured_delta.abs();
        let return_velocity = observation.telemetry.measured_velocity;
        match return_stability.observe(
            return_started.elapsed(),
            return_error <= J6_FIRST_POSITION_RETURN_POSITION_RAD
                && return_velocity.abs() <= J6_FIRST_POSITION_RETURN_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => break (return_error, return_velocity),
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_6 did not return within {:.3} s: error {return_error:.6} rad, velocity {return_velocity:.6} rad/s",
                RETURN_SETTLE_TIMEOUT.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    };
    confirm_joint6_first_position_target_readback_while_guarded(
        backend,
        profile,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        baseline_targets[selected_index],
        initial_q[selected_index],
        diagnostic_started,
        "returned_persistent_baseline",
        J6_FIRST_POSITION_EXPECTED_KP,
        J6_FIRST_POSITION_EXPECTED_KD,
        peaks,
    )
    .await?;
    anyhow::ensure!(
        most_negative_delta_rad <= -J6_FIRST_POSITION_REQUIRED_PEAK_RAD,
        "joint_6 negative peak {most_negative_delta_rad:.6} rad did not reach -{J6_FIRST_POSITION_REQUIRED_PEAK_RAD:.6} rad"
    );
    anyhow::ensure!(
        most_negative_raw_delta_rev <= -J6_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV,
        "joint_6 raw motor peak {most_negative_raw_delta_rev:.9} rev did not reach -{J6_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV:.9} rev"
    );
    tracing::info!(
        phase = PHASE_J6_FIRST_POSITION_RETURN,
        joint = %joint.name,
        node_id = joint.node_id,
        most_negative_delta_rad,
        most_negative_raw_delta_rev,
        return_error_rad,
        return_velocity_rad_s,
        "joint_6 fixed survey met logical/raw direction and continuous return gates"
    );
    Ok(())
}

/// Run one short, bounded J2 diagnostic. This is never a raw-torque or general
/// jog mode: hold/staircase retain one position target, while the position
/// variant admits only its fixed positive 5 mrad smooth round trip. The caller
/// owns signals and the final confirmed all-axis disable/heartbeat-disarm.
pub async fn run_single_axis_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: SingleAxisDiagnosticRequest,
) -> Result<()> {
    request.validate(profile)?;
    profile.validate_single_turn_command_windows()?;
    let torque_limits = request.torque_limits();

    let initial_feedback = wait_for_safe_feedback(backend).await?;
    let initial_q = validate_feedback(profile, &initial_feedback, None)?;
    let selected_index = request.selected_index;
    let joint = &profile.joints[selected_index];
    let start_proximity_rad = if torque_limits.high_tier {
        HIGH_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD
    } else {
        LOW_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD
    };
    validate_diagnostic_start_position(joint, initial_q[selected_index], start_proximity_rad)?;
    let temperature_baseline = diagnostic_temperature_baseline(&initial_feedback, selected_index)?;
    let position_round_trip = matches!(
        request.mode,
        SingleAxisDiagnosticMode::PositionRoundTrip { .. }
    );
    let gravity_hold_censored =
        matches!(request.mode, SingleAxisDiagnosticMode::GravityHoldCensored);
    let fixed_position_gravity_hold = position_round_trip || gravity_hold_censored;
    // The position survey's reviewed operating profile remains locked to
    // scale 0.65, but entering MIT at that value produced a measured velocity
    // beyond the hard gate. Its configure-time object and first shared frame
    // must therefore use the separately exercised 0.25-scale hold.
    let initial_targets = if fixed_position_gravity_hold {
        build_position_diagnostic_hold_targets(
            profile,
            dynamics,
            &initial_q,
            selected_index,
            initial_q[selected_index],
            POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE,
        )?
    } else {
        build_safe_hold_targets(
            profile,
            dynamics,
            &initial_q,
            selected_index,
            initial_q[selected_index],
        )?
    };
    if fixed_position_gravity_hold {
        let observation = position_diagnostic_observation(
            profile,
            &initial_feedback,
            selected_index,
            &initial_q,
            temperature_baseline,
            initial_targets[selected_index],
            initial_q[selected_index],
            torque_limits,
        )
        .context("pre-enable position diagnostic torque/position/temperature gate failed")?;
        if gravity_hold_censored {
            validate_censored_gravity_hold_identification_layer(observation)
                .context("pre-enable censored gravity-hold identification layer failed")?;
        }
    } else {
        diagnostic_observation(
            profile,
            &initial_feedback,
            selected_index,
            &initial_q,
            temperature_baseline,
            initial_targets[selected_index],
            0.0,
            torque_limits,
        )
        .context("pre-enable diagnostic torque/position/temperature gate failed")?;
    }
    // Persist a zero-additive hold outside this cancellable future before the
    // first enable attempt. Signal cleanup can therefore remove feed-forward
    // without waiting for an SDO even if cancellation lands during enable.
    backend
        .register_single_axis_diagnostic_baseline(selected_index, initial_targets)
        .context("register pre-enable diagnostic baseline")?;
    backend
        .enable_diagnostic_axis(selected_index, initial_targets)
        .await
        .with_context(|| format!("enable only {} for bounded diagnostics", joint.name))?;
    // The high-tier wall clock begins as soon as J2 is confirmed enabled, not
    // after the stability dwell. This bounds the complete torque-capable
    // interval, including stability, the first SDO proof, every level, and the
    // normal baseline restore.
    let diagnostic_started = Instant::now();

    // Reuse the already exercised symmetric post-enable stability gate.  The
    // synthetic delta only sizes its tracking guard; this path never commands
    // a position delta.
    let stability_request = CommissioningRequest {
        selected_index,
        delta_rad: MIN_COMMISSION_DELTA_RAD,
        duration_sec: 1.0,
    };
    let (mut stable_feedback, mut stable_q, mut stable_targets) = wait_for_enabled_axis_stability(
        backend,
        profile,
        dynamics,
        stability_request,
        initial_q,
        Some(DiagnosticStabilityGuards {
            torque_limits,
            temperature_baseline,
            active_started: diagnostic_started,
            position_round_trip: fixed_position_gravity_hold,
            censored_gravity_hold: gravity_hold_censored,
            selected_gravity_scale: fixed_position_gravity_hold
                .then_some(POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE),
        }),
    )
    .await?;
    let mut peaks = DiagnosticPeaks::default();
    if gravity_hold_censored {
        run_censored_gravity_hold(
            backend,
            profile,
            dynamics,
            selected_index,
            &initial_q,
            &stable_q,
            temperature_baseline,
            stable_feedback.oldest_tpdo1_at,
            stable_targets,
            initial_targets,
            diagnostic_started,
            torque_limits,
            &mut peaks,
        )
        .await?;
        tracing::info!(
            phase = PHASE_DIAGNOSTIC_CENSORED_GRAVITY_HOLD,
            joint = %joint.name,
            node_id = joint.node_id,
            peak_position_delta_rad = peaks.position_delta_rad,
            peak_velocity_rad_s = peaks.velocity_rad_s,
            peak_measured_torque_nm = peaks.measured_torque_nm,
            peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
            peak_driver_temperature_c = peaks.driver_temperature_c,
            peak_motor_temperature_c = peaks.motor_temperature_c,
            elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
            "censored fixed-position gravity-hold identification completed without entering a position trajectory"
        );
        return Ok(());
    }
    if position_round_trip {
        run_position_diagnostic_gravity_ramp(
            backend,
            profile,
            dynamics,
            selected_index,
            &initial_q,
            &stable_q,
            temperature_baseline,
            stable_feedback.oldest_tpdo1_at,
            stable_targets,
            initial_targets,
            diagnostic_started,
            torque_limits,
            &mut peaks,
        )
        .await?;

        // The endpoint itself has now been proven by fresh feedback and is the
        // persistent cleanup baseline. Require the tighter continuous
        // stability dwell again at full scale before even constructing a
        // position trajectory.
        (stable_feedback, stable_q, stable_targets) = wait_for_enabled_axis_stability(
            backend,
            profile,
            dynamics,
            stability_request,
            initial_q,
            Some(DiagnosticStabilityGuards {
                torque_limits,
                temperature_baseline,
                active_started: diagnostic_started,
                position_round_trip: true,
                censored_gravity_hold: false,
                selected_gravity_scale: Some(POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE),
            }),
        )
        .await?;
    }
    let (baseline_targets, model_gravity_ff_nm) = build_diagnostic_hold_targets(
        profile,
        dynamics,
        &stable_q,
        selected_index,
        initial_q[selected_index],
        0.0,
        torque_limits,
    )?;
    if position_round_trip {
        position_diagnostic_observation(
            profile,
            &stable_feedback,
            selected_index,
            &initial_q,
            temperature_baseline,
            baseline_targets[selected_index],
            initial_q[selected_index],
            torque_limits,
        )
        .context("post-stability position diagnostic baseline gate failed")?;
        anyhow::ensure!(
            compressed_target_words(&baseline_targets[selected_index], joint)
                == compressed_target_words(&stable_targets[selected_index], joint),
            "post-ramp stability target does not match the final full-scale baseline"
        );
        anyhow::ensure!(
            compressed_target_words(&baseline_targets[selected_index], joint)
                != compressed_target_words(&initial_targets[selected_index], joint),
            "full-scale position diagnostic baseline did not change from its 0.25-scale configure target"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed before post-ramp baseline proof")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed before post-ramp baseline proof"
        );
        backend.set_targets(baseline_targets).await?;
        confirm_position_target_readback_while_guarded(
            backend,
            profile,
            selected_index,
            &initial_q,
            temperature_baseline,
            baseline_targets[selected_index],
            initial_q[selected_index],
            diagnostic_started,
            torque_limits,
            PHASE_DIAGNOSTIC_POSITION_ROUND_TRIP,
            "post_ramp_full_scale_baseline",
            true,
            false,
            &mut peaks,
        )
        .await?;
    }
    backend
        .register_single_axis_diagnostic_baseline(selected_index, baseline_targets)
        .context("register post-stability zero-additive diagnostic baseline")?;

    if let SingleAxisDiagnosticMode::TauFfStaircase {
        peak_additive_torque_nm,
        step_torque_nm,
        ..
    } = request.mode
    {
        let configured_initial_words =
            compressed_target_words(&initial_targets[selected_index], joint);
        let (probe_targets, _) = build_diagnostic_hold_targets(
            profile,
            dynamics,
            &stable_q,
            selected_index,
            initial_q[selected_index],
            step_torque_nm.min(peak_additive_torque_nm),
            torque_limits,
        )?;
        let (peak_targets, _) = build_diagnostic_hold_targets(
            profile,
            dynamics,
            &stable_q,
            selected_index,
            initial_q[selected_index],
            peak_additive_torque_nm,
            torque_limits,
        )?;
        anyhow::ensure!(
            compressed_target_words(&probe_targets[selected_index], joint)
                != configured_initial_words,
            "lowest diagnostic probe does not change the quantized 0x2004:02/03 words from the SDO-configured initial target; no RPDO-consumption proof is possible"
        );
        anyhow::ensure!(
            compressed_target_words(&baseline_targets[selected_index], joint)
                != compressed_target_words(&peak_targets[selected_index], joint),
            "requested diagnostic peak does not change the quantized 0x2004:02/03 target; increase it without exceeding the diagnostic cap"
        );
        anyhow::ensure!(
            compressed_target_words(&peak_targets[selected_index], joint)
                != configured_initial_words,
            "diagnostic peak does not change the quantized 0x2004:02/03 words from the SDO-configured initial target; no peak RPDO-consumption proof is possible"
        );
    }
    if let SingleAxisDiagnosticMode::PositionRoundTrip { delta_rad, .. } = request.mode {
        let peak_commanded_q = initial_q[selected_index] + delta_rad;
        let peak_targets = build_safe_hold_targets(
            profile,
            dynamics,
            &stable_q,
            selected_index,
            peak_commanded_q,
        )?;
        anyhow::ensure!(
            compressed_target_position_code(&baseline_targets[selected_index], joint)
                != compressed_target_position_code(&peak_targets[selected_index], joint),
            "joint_2 position-round-trip peak does not change the quantized 0x2004 position field from baseline"
        );
    }

    let (maximum_positive_excursion_rad, maximum_opposite_excursion_rad, absolute_upper_rad) = if matches!(
        request.mode,
        SingleAxisDiagnosticMode::PositionRoundTrip { .. }
    ) {
        (
            POSITION_DIAGNOSTIC_DELTA_RAD + POSITION_DIAGNOSTIC_MAX_POSITIVE_MARGIN_RAD,
            POSITION_DIAGNOSTIC_MAX_OPPOSITE_POSITION_RAD,
            joint.limits.position_lower_rad + POSITION_DIAGNOSTIC_ABSOLUTE_UPPER_OFFSET_RAD,
        )
    } else if torque_limits.high_tier {
        (
            MAX_DIAGNOSTIC_EXCURSION_RAD,
            MAX_DIAGNOSTIC_OPPOSITE_POSITION_RAD,
            joint.limits.position_lower_rad + 2.0 * HIGH_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD,
        )
    } else {
        (
            MAX_DIAGNOSTIC_EXCURSION_RAD,
            MAX_DIAGNOSTIC_OPPOSITE_POSITION_RAD,
            joint.limits.position_upper_rad,
        )
    };
    tracing::warn!(
        joint = %joint.name,
        node_id = joint.node_id,
        mode = ?request.mode,
        position_hold_rad = initial_q[selected_index],
        model_gravity_ff_nm,
        high_torque_tier = torque_limits.high_tier,
        max_additive_torque_nm = torque_limits.max_additive_torque_nm,
        max_total_torque_nm = torque_limits.max_total_torque_nm,
        maximum_positive_excursion_rad,
        maximum_opposite_excursion_rad,
        absolute_upper_rad,
        max_velocity_rad_s = MAX_DIAGNOSTIC_VELOCITY_RAD_S,
        max_temperature_c = MAX_DIAGNOSTIC_TEMPERATURE_C,
        max_temperature_rise_c = MAX_DIAGNOSTIC_TEMPERATURE_RISE_C,
        max_active_sec = torque_limits.max_active_sec,
        "bounded joint_2 diagnostic enabled; no unknown brake object is written"
    );

    let operation: Result<DiagnosticLevelOutcome> = async {
        match request.mode {
            SingleAxisDiagnosticMode::Hold { duration_sec } => {
                run_diagnostic_level(
                    backend,
                    profile,
                    dynamics,
                    selected_index,
                    &initial_q,
                    temperature_baseline,
                    baseline_targets,
                    model_gravity_ff_nm,
                    0.0,
                    0,
                    1,
                    Duration::from_secs_f32(duration_sec),
                    false,
                    false,
                    PHASE_DIAGNOSTIC_HOLD,
                    diagnostic_started,
                    torque_limits,
                    &mut peaks,
                )
                .await
            }
            SingleAxisDiagnosticMode::TauFfStaircase {
                peak_additive_torque_nm,
                step_torque_nm,
                dwell_sec,
            } => {
                let levels = diagnostic_torque_levels(peak_additive_torque_nm, step_torque_nm)?;
                let total_levels = levels.len();
                let mut outcome = DiagnosticLevelOutcome::Completed;
                for (level_index, additive_torque_nm) in levels.into_iter().enumerate() {
                    let (targets, gravity_ff_nm) = build_diagnostic_hold_targets(
                        profile,
                        dynamics,
                        &stable_q,
                        selected_index,
                        initial_q[selected_index],
                        additive_torque_nm,
                        torque_limits,
                    )?;
                    let is_first_probe = level_index == 0;
                    let is_peak = level_index + 1 == total_levels;
                    outcome = run_diagnostic_level(
                        backend,
                        profile,
                        dynamics,
                        selected_index,
                        &initial_q,
                        temperature_baseline,
                        targets,
                        gravity_ff_nm,
                        additive_torque_nm,
                        level_index + 1,
                        total_levels,
                        Duration::from_secs_f32(dwell_sec),
                        is_first_probe || (!torque_limits.high_tier && is_peak),
                        true,
                        PHASE_DIAGNOSTIC_STAIRCASE,
                        diagnostic_started,
                        torque_limits,
                        &mut peaks,
                    )
                    .await?;
                    if outcome == DiagnosticLevelOutcome::BreakawayDetected {
                        break;
                    }
                }
                Ok(outcome)
            }
            SingleAxisDiagnosticMode::PositionRoundTrip {
                delta_rad,
                duration_sec,
            } => {
                run_position_round_trip_diagnostic(
                    backend,
                    profile,
                    dynamics,
                    selected_index,
                    &initial_q,
                    temperature_baseline,
                    baseline_targets,
                    delta_rad,
                    duration_sec,
                    diagnostic_started,
                    torque_limits,
                    &mut peaks,
                )
                .await?;
                Ok(DiagnosticLevelOutcome::Completed)
            }
            SingleAxisDiagnosticMode::GravityHoldCensored => {
                unreachable!("censored gravity hold returns before the general diagnostic branch")
            }
        }
    }
    .await;

    // On every ordinary success/error path, remove the additive term before
    // returning to the outer selected-first confirmed disable. High-tier
    // breakaway/error exits take the fast path: one guarded shared-target
    // update, with no 150 ms dwell and no SDO readback. A normal completed run
    // retains the exact baseline readback as its final RPDO evidence.
    let fast_high_tier_restore = requires_fast_diagnostic_restore(torque_limits, &operation);
    let restore: Result<()> = if fast_high_tier_restore {
        async {
            backend
                .ensure_single_axis_commissioning_state(selected_index)
                .context("single-axis drive-state contract failed before fast baseline restore")?;
            anyhow::ensure!(
                !backend.transport_failed(),
                "CAN transport failed before fast diagnostic baseline restore"
            );
            if !matches!(
                request.mode,
                SingleAxisDiagnosticMode::PositionRoundTrip { .. }
            ) {
                let feedback = backend.feedback();
                let observation = diagnostic_observation(
                    profile,
                    &feedback,
                    selected_index,
                    &initial_q,
                    temperature_baseline,
                    baseline_targets[selected_index],
                    0.0,
                    torque_limits,
                )?;
                peaks.observe(observation);
            }
            backend
                .set_targets(baseline_targets)
                .await
                .context("immediately publish zero-additive diagnostic baseline")?;
            tracing::warn!(
                joint = %joint.name,
                "high-tier diagnostic exited early; published baseline without dwell/readback before selected-first disable"
            );
            Ok(())
        }
        .await
    } else {
        let restore_requires_readback = matches!(
            request.mode,
            SingleAxisDiagnosticMode::TauFfStaircase { .. }
        );
        run_diagnostic_level(
            backend,
            profile,
            dynamics,
            selected_index,
            &initial_q,
            temperature_baseline,
            baseline_targets,
            model_gravity_ff_nm,
            0.0,
            0,
            1,
            Duration::from_secs_f32(MIN_DIAGNOSTIC_DWELL_SEC),
            restore_requires_readback,
            false,
            "diagnostic_baseline_restore",
            diagnostic_started,
            torque_limits,
            &mut peaks,
        )
        .await
        .map(|_| ())
    };

    let outcome = match (operation, restore) {
        (Ok(outcome), Ok(())) => outcome,
        (Err(operation_error), Ok(())) => return Err(operation_error),
        (Ok(_), Err(restore_error)) => {
            return Err(restore_error).context("diagnostic baseline restore failed")
        }
        (Err(operation_error), Err(restore_error)) => {
            return Err(anyhow::anyhow!(
                "diagnostic failed: {operation_error:#}; baseline restore also failed: {restore_error:#}"
            ))
        }
    };

    tracing::info!(
        joint = %joint.name,
        node_id = joint.node_id,
        outcome = ?outcome,
        peak_position_delta_rad = peaks.position_delta_rad,
        peak_velocity_rad_s = peaks.velocity_rad_s,
        peak_measured_torque_nm = peaks.measured_torque_nm,
        peak_estimated_total_torque_nm = peaks.estimated_total_torque_nm,
        peak_driver_temperature_c = peaks.driver_temperature_c,
        peak_motor_temperature_c = peaks.motor_temperature_c,
        elapsed_sec = diagnostic_started.elapsed().as_secs_f32(),
        "bounded joint_2 diagnostic completed; this result alone does not identify or release a mechanical holding brake"
    );
    Ok(())
}

fn diagnostic_torque_levels(peak_nm: f32, step_nm: f32) -> Result<Vec<f32>> {
    anyhow::ensure!(
        peak_nm.is_finite() && step_nm.is_finite() && peak_nm > 0.0 && step_nm > 0.0,
        "diagnostic torque peak and step must be finite and positive"
    );
    anyhow::ensure!(
        step_nm <= peak_nm,
        "diagnostic torque step cannot exceed its peak"
    );
    let count = (peak_nm / step_nm).ceil() as usize;
    anyhow::ensure!(count > 0, "diagnostic torque staircase is empty");
    Ok((1..=count)
        .map(|index| (index as f32 * step_nm).min(peak_nm))
        .collect())
}

fn position_diagnostic_gravity_ramp_scale(step: usize) -> Result<f32> {
    anyhow::ensure!(
        step <= POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS,
        "position diagnostic gravity-ramp step {step} exceeds {POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS}"
    );
    let normalized = step as f32 / POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS as f32;
    let blend = 0.5 * (1.0 - (PI * normalized).cos());
    Ok(POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE
        + (POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE - POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE)
            * blend)
}

fn censored_gravity_hold_scale(step: usize) -> Result<f32> {
    anyhow::ensure!(
        step <= CENSORED_GRAVITY_HOLD_STEPS,
        "censored gravity-hold step {step} exceeds {CENSORED_GRAVITY_HOLD_STEPS}"
    );
    if step == 0 {
        return Ok(CENSORED_GRAVITY_HOLD_PROFILE_SCALE);
    }
    if step == CENSORED_GRAVITY_HOLD_STEPS {
        return Ok(CENSORED_GRAVITY_HOLD_CAP_SCALE);
    }
    let normalized = step as f32 / CENSORED_GRAVITY_HOLD_STEPS as f32;
    let blend = 0.5 * (1.0 - (PI * normalized).cos());
    Ok(CENSORED_GRAVITY_HOLD_PROFILE_SCALE
        + (CENSORED_GRAVITY_HOLD_CAP_SCALE - CENSORED_GRAVITY_HOLD_PROFILE_SCALE) * blend)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CensoredGravityHoldDecision {
    Advance,
    FreezePrevious,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CensoredGravityHoldPollDecision {
    AwaitTargetProof,
    ConfirmPublished,
    FreezePrevious,
}

fn censored_gravity_hold_decision(
    observation: DiagnosticObservation,
) -> Result<CensoredGravityHoldDecision> {
    validate_censored_gravity_hold_identification_layer(observation)?;
    if observation.telemetry.measured_delta >= CENSORED_GRAVITY_HOLD_TRIGGER_RAD {
        Ok(CensoredGravityHoldDecision::FreezePrevious)
    } else {
        Ok(CensoredGravityHoldDecision::Advance)
    }
}

fn censored_gravity_hold_poll_decision(
    observation: DiagnosticObservation,
    target_is_feedback_proven: bool,
) -> Result<CensoredGravityHoldPollDecision> {
    match censored_gravity_hold_decision(observation)? {
        CensoredGravityHoldDecision::FreezePrevious => {
            Ok(CensoredGravityHoldPollDecision::FreezePrevious)
        }
        CensoredGravityHoldDecision::Advance if target_is_feedback_proven => {
            Ok(CensoredGravityHoldPollDecision::ConfirmPublished)
        }
        CensoredGravityHoldDecision::Advance => {
            Ok(CensoredGravityHoldPollDecision::AwaitTargetProof)
        }
    }
}

fn validate_censored_gravity_hold_identification_layer(
    observation: DiagnosticObservation,
) -> Result<()> {
    let telemetry = observation.telemetry;
    anyhow::ensure!(
        telemetry.measured_delta < CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD,
        "joint_2 censored gravity-hold displacement {:.6} rad reached the {:.6} rad identification exit layer",
        telemetry.measured_delta,
        CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD
    );
    anyhow::ensure!(
        telemetry.measured_velocity.abs()
            < CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S,
        "joint_2 censored gravity-hold velocity {:.6} rad/s reached the {:.6} rad/s identification exit layer",
        telemetry.measured_velocity,
        CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S
    );
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CensoredVerifiedTarget {
    step: usize,
    scale: f32,
    targets: [MotorTarget; DOF],
}

#[derive(Debug, Default)]
struct CensoredVerifiedTargetHistory {
    distinct: Vec<CensoredVerifiedTarget>,
}

impl CensoredVerifiedTargetHistory {
    fn record(
        &mut self,
        step: usize,
        scale: f32,
        targets: [MotorTarget; DOF],
        selected_index: usize,
        joint: &crate::profile::JointProfile,
    ) {
        let words = compressed_target_words(&targets[selected_index], joint);
        if self.distinct.last().is_none_or(|entry| {
            compressed_target_words(&entry.targets[selected_index], joint) != words
        }) {
            self.distinct.push(CensoredVerifiedTarget {
                step,
                scale,
                targets,
            });
        }
    }

    fn rollback_candidate(
        &self,
        rejected_targets: &[MotorTarget; DOF],
        configured_initial_targets: &[MotorTarget; DOF],
        selected_index: usize,
        joint: &crate::profile::JointProfile,
    ) -> Option<CensoredVerifiedTarget> {
        let rejected_words = compressed_target_words(&rejected_targets[selected_index], joint);
        let configured_words =
            compressed_target_words(&configured_initial_targets[selected_index], joint);
        let fixed_position_code =
            compressed_target_position_code(&configured_initial_targets[selected_index], joint);
        self.distinct.iter().rev().copied().find(|entry| {
            let target = &entry.targets[selected_index];
            let words = compressed_target_words(target, joint);
            entry.scale > CENSORED_GRAVITY_HOLD_PROFILE_SCALE
                && words != rejected_words
                && words != configured_words
                && compressed_target_position_code(target, joint) == fixed_position_code
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct CensoredHoldStatisticsSample {
    captured_at: Instant,
    position_delta_rad: f32,
    velocity_rad_s: f32,
    measured_torque_nm: f32,
    estimated_total_torque_nm: f32,
    driver_temperature_c: f32,
    motor_temperature_c: f32,
}

#[derive(Debug, Default)]
struct CensoredHoldStatistics {
    samples: Vec<CensoredHoldStatisticsSample>,
}

#[derive(Debug, Clone, Copy)]
struct CensoredHoldStatisticsSummary {
    sample_count: usize,
    mean_position_delta_rad: f32,
    position_stddev_rad: f32,
    minimum_position_delta_rad: f32,
    maximum_position_delta_rad: f32,
    peak_velocity_rad_s: f32,
    mean_measured_torque_nm: f32,
    mean_estimated_total_torque_nm: f32,
    peak_driver_temperature_c: f32,
    peak_motor_temperature_c: f32,
    terminal_sample_count: usize,
    terminal_position_span_rad: f32,
    terminal_peak_velocity_rad_s: f32,
    terminal_stable: bool,
}

impl CensoredHoldStatistics {
    fn observe(&mut self, captured_at: Instant, observation: DiagnosticObservation) -> Result<()> {
        anyhow::ensure!(
            self.samples
                .last()
                .is_none_or(|sample| captured_at > sample.captured_at),
            "censored gravity-hold statistics require strictly newer TPDO1 timestamps"
        );
        let telemetry = observation.telemetry;
        self.samples.push(CensoredHoldStatisticsSample {
            captured_at,
            position_delta_rad: telemetry.measured_delta,
            velocity_rad_s: telemetry.measured_velocity,
            measured_torque_nm: telemetry.measured_torque,
            estimated_total_torque_nm: telemetry.gravity_ff + telemetry.estimated_pd_torque,
            driver_temperature_c: observation.driver_temperature_c,
            motor_temperature_c: observation.motor_temperature_c,
        });
        Ok(())
    }

    fn summarize(&self) -> Result<CensoredHoldStatisticsSummary> {
        anyhow::ensure!(
            self.samples.len() >= CENSORED_GRAVITY_HOLD_MIN_STATISTICS_SAMPLES,
            "censored gravity hold collected {} fresh samples; at least {} are required",
            self.samples.len(),
            CENSORED_GRAVITY_HOLD_MIN_STATISTICS_SAMPLES
        );
        let count = self.samples.len() as f64;
        let mean_position = self
            .samples
            .iter()
            .map(|sample| f64::from(sample.position_delta_rad))
            .sum::<f64>()
            / count;
        let position_variance = self
            .samples
            .iter()
            .map(|sample| {
                let error = f64::from(sample.position_delta_rad) - mean_position;
                error * error
            })
            .sum::<f64>()
            / count;
        let mean = |value: fn(&CensoredHoldStatisticsSample) -> f32| -> f32 {
            (self
                .samples
                .iter()
                .map(|sample| f64::from(value(sample)))
                .sum::<f64>()
                / count) as f32
        };
        let minimum_position_delta_rad = self
            .samples
            .iter()
            .map(|sample| sample.position_delta_rad)
            .fold(f32::INFINITY, f32::min);
        let maximum_position_delta_rad = self
            .samples
            .iter()
            .map(|sample| sample.position_delta_rad)
            .fold(f32::NEG_INFINITY, f32::max);
        let peak_velocity_rad_s = self
            .samples
            .iter()
            .map(|sample| sample.velocity_rad_s.abs())
            .fold(0.0_f32, f32::max);
        let peak_driver_temperature_c = self
            .samples
            .iter()
            .map(|sample| sample.driver_temperature_c)
            .fold(f32::NEG_INFINITY, f32::max);
        let peak_motor_temperature_c = self
            .samples
            .iter()
            .map(|sample| sample.motor_temperature_c)
            .fold(f32::NEG_INFINITY, f32::max);

        let last_captured_at = self.samples.last().unwrap().captured_at;
        let terminal_start = last_captured_at
            .checked_sub(CENSORED_GRAVITY_HOLD_TERMINAL_DWELL)
            .unwrap_or(self.samples[0].captured_at);
        let terminal: Vec<_> = self
            .samples
            .iter()
            .filter(|sample| sample.captured_at >= terminal_start)
            .collect();
        let terminal_minimum_position = terminal
            .iter()
            .map(|sample| sample.position_delta_rad)
            .fold(f32::INFINITY, f32::min);
        let terminal_maximum_position = terminal
            .iter()
            .map(|sample| sample.position_delta_rad)
            .fold(f32::NEG_INFINITY, f32::max);
        let terminal_position_span_rad = terminal_maximum_position - terminal_minimum_position;
        let terminal_peak_velocity_rad_s = terminal
            .iter()
            .map(|sample| sample.velocity_rad_s.abs())
            .fold(0.0_f32, f32::max);
        let terminal_stable = terminal.len() >= CENSORED_GRAVITY_HOLD_MIN_TERMINAL_SAMPLES
            && terminal_position_span_rad <= CENSORED_GRAVITY_HOLD_TERMINAL_POSITION_SPAN_RAD
            && terminal_peak_velocity_rad_s <= CENSORED_GRAVITY_HOLD_TERMINAL_VELOCITY_RAD_S;

        Ok(CensoredHoldStatisticsSummary {
            sample_count: self.samples.len(),
            mean_position_delta_rad: mean_position as f32,
            position_stddev_rad: position_variance.sqrt() as f32,
            minimum_position_delta_rad,
            maximum_position_delta_rad,
            peak_velocity_rad_s,
            mean_measured_torque_nm: mean(|sample| sample.measured_torque_nm),
            mean_estimated_total_torque_nm: mean(|sample| sample.estimated_total_torque_nm),
            peak_driver_temperature_c,
            peak_motor_temperature_c,
            terminal_sample_count: terminal.len(),
            terminal_position_span_rad,
            terminal_peak_velocity_rad_s,
            terminal_stable,
        })
    }
}

fn position_gravity_ramp_feedback_proves_target(
    feedback_at: Instant,
    previous_feedback_at: Instant,
    target_proof_after: Instant,
) -> bool {
    feedback_at > previous_feedback_at && feedback_at > target_proof_after
}

fn validate_joint1_first_position_dynamic_window(
    joint: &crate::profile::JointProfile,
    initial_position_rad: f32,
) -> Result<()> {
    anyhow::ensure!(
        initial_position_rad.is_finite()
            && (J1_FIRST_POSITION_INITIAL_Q_LOWER_RAD
                ..=J1_FIRST_POSITION_INITIAL_Q_UPPER_RAD)
                .contains(&initial_position_rad),
        "joint_1 initial position q0={initial_position_rad:.6} rad is outside the fixed [{J1_FIRST_POSITION_INITIAL_Q_LOWER_RAD:.6}, {J1_FIRST_POSITION_INITIAL_Q_UPPER_RAD:.6}] rad first-position range"
    );
    let lower = initial_position_rad + J1_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD;
    let upper = initial_position_rad + J1_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD;
    anyhow::ensure!(
        lower >= joint.limits.position_lower_rad && upper <= joint.limits.position_upper_rad,
        "joint_1 dynamic first-position window [{lower:.6}, {upper:.6}] rad around q0={initial_position_rad:.6} is not wholly inside profile limits [{:.6}, {:.6}]",
        joint.limits.position_lower_rad,
        joint.limits.position_upper_rad
    );
    Ok(())
}

fn validate_joint1_first_position_active_time(
    diagnostic_started: Instant,
    phase: &str,
) -> Result<()> {
    anyhow::ensure!(
        diagnostic_started.elapsed().as_secs_f32() <= J1_FIRST_POSITION_MAX_ACTIVE_SEC,
        "joint_1 first-position diagnostic exceeded its {J1_FIRST_POSITION_MAX_ACTIVE_SEC:.3} s active-time limit during {phase}"
    );
    Ok(())
}

fn validate_joint1_first_position_temperature(
    joint_name: &str,
    label: &str,
    temperature_c: f32,
    baseline_c: f32,
) -> Result<()> {
    anyhow::ensure!(
        temperature_c.is_finite()
            && (MIN_PLAUSIBLE_DIAGNOSTIC_TEMPERATURE_C..=MAX_DIAGNOSTIC_TEMPERATURE_C)
                .contains(&temperature_c),
        "{joint_name} {label} temperature {temperature_c:.3} C is outside [{MIN_PLAUSIBLE_DIAGNOSTIC_TEMPERATURE_C:.1}, {MAX_DIAGNOSTIC_TEMPERATURE_C:.1}] C"
    );
    anyhow::ensure!(
        temperature_c - baseline_c <= MAX_DIAGNOSTIC_TEMPERATURE_RISE_C,
        "{joint_name} {label} temperature rose by {:.3} C from baseline {baseline_c:.3} C; limit is {MAX_DIAGNOSTIC_TEMPERATURE_RISE_C:.3} C",
        temperature_c - baseline_c
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn joint1_first_position_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
) -> Result<DiagnosticObservation> {
    joint1_observation_with_feedforward_limit(
        profile,
        feedback,
        selected_index,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        commanded_q,
        J1_FIRST_POSITION_MAX_GRAVITY_NM,
        J1_FIRST_POSITION_EXPECTED_KP,
        J1_FIRST_POSITION_EXPECTED_KD,
    )
}

#[allow(clippy::too_many_arguments)]
fn joint1_compensated_position_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
) -> Result<DiagnosticObservation> {
    joint1_observation_with_feedforward_limit(
        profile,
        feedback,
        selected_index,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        commanded_q,
        J1_FIRST_POSITION_MAX_COMPENSATED_FEEDFORWARD_NM,
        J1_FIRST_POSITION_TRAJECTORY_KP,
        J1_FIRST_POSITION_TRAJECTORY_KD,
    )
}

#[allow(clippy::too_many_arguments)]
fn joint1_observation_with_feedforward_limit(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    maximum_feedforward_nm: f32,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
) -> Result<DiagnosticObservation> {
    fixed_position_observation(
        profile,
        feedback,
        selected_index,
        initial_q,
        initial_motor_position_rev,
        temperature_baseline,
        target,
        commanded_q,
        "joint_1",
        J1_FIRST_POSITION_INDEX,
        J1_FIRST_POSITION_DELTA_RAD,
        J1_FIRST_POSITION_WINDOW_LOWER_OFFSET_RAD,
        J1_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD,
        J1_FIRST_POSITION_ENABLE_HARD_VELOCITY_RAD_S,
        maximum_feedforward_nm,
        J1_FIRST_POSITION_MAX_GRAVITY_NM,
        J1_FIRST_POSITION_MAX_TOTAL_TORQUE_NM,
        expected_kp_nm_rad,
        expected_kd_nm_s_rad,
    )
}

#[allow(clippy::too_many_arguments)]
fn fixed_position_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_q: &[f32; DOF],
    initial_motor_position_rev: f32,
    temperature_baseline: SixAxisDiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    diagnostic_name: &str,
    expected_selected_index: usize,
    delta_rad: f32,
    window_lower_offset_rad: f32,
    window_upper_offset_rad: f32,
    maximum_velocity_rad_s: f32,
    maximum_feedforward_nm: f32,
    minimum_feedforward_limit_nm: f32,
    maximum_total_torque_nm: f32,
    expected_kp_nm_rad: f32,
    expected_kd_nm_s_rad: f32,
) -> Result<DiagnosticObservation> {
    anyhow::ensure!(
        selected_index == expected_selected_index,
        "{diagnostic_name} first-position observation is restricted to joint index {expected_selected_index}"
    );
    let measured_q = validate_feedback(profile, feedback, None)?;
    let joint = &profile.joints[selected_index];
    let state = feedback.joints[selected_index];
    for (index, (profile_joint, baseline)) in profile
        .joints
        .iter()
        .zip(temperature_baseline.joints)
        .enumerate()
    {
        let feedback_joint = feedback.joints[index];
        validate_joint1_first_position_temperature(
            &profile_joint.name,
            "driver",
            feedback_joint.driver_temperature_c,
            baseline.driver_c,
        )?;
        validate_joint1_first_position_temperature(
            &profile_joint.name,
            "motor",
            feedback_joint.motor_temperature_c,
            baseline.motor_c,
        )?;
    }
    let telemetry = commissioning_telemetry_sample(
        joint,
        feedback,
        selected_index,
        initial_q[selected_index],
        measured_q[selected_index],
        commanded_q,
        target,
    );
    anyhow::ensure!(
        telemetry.measured_delta >= window_lower_offset_rad
            && telemetry.measured_delta <= window_upper_offset_rad,
        "{diagnostic_name} measured displacement {:.6} rad left its dynamic [{:.6}, {:.6}] rad window",
        telemetry.measured_delta,
        window_lower_offset_rad,
        window_upper_offset_rad
    );
    anyhow::ensure!(
        telemetry.measured_velocity.abs() <= maximum_velocity_rad_s,
        "{diagnostic_name} measured velocity {:.6} rad/s exceeds {:.6} rad/s",
        telemetry.measured_velocity,
        maximum_velocity_rad_s
    );
    let command_lower = initial_q[selected_index].min(initial_q[selected_index] + delta_rad);
    let command_upper = initial_q[selected_index].max(initial_q[selected_index] + delta_rad);
    anyhow::ensure!(
        commanded_q >= command_lower
            && commanded_q <= command_upper
            && (joint.limits.position_lower_rad..=joint.limits.position_upper_rad)
                .contains(&commanded_q),
        "{diagnostic_name} command {commanded_q:.6} rad is outside the fixed [{command_lower:.6}, {command_upper:.6}] path or profile limits"
    );
    anyhow::ensure!(
        maximum_feedforward_nm.is_finite()
            && maximum_feedforward_nm >= minimum_feedforward_limit_nm
            && telemetry.gravity_ff.abs() <= maximum_feedforward_nm,
        "{diagnostic_name} feed-forward {:.6} Nm exceeds {:.6} Nm",
        telemetry.gravity_ff,
        maximum_feedforward_nm
    );
    anyhow::ensure!(
        telemetry.measured_torque.abs() <= maximum_total_torque_nm,
        "{diagnostic_name} measured torque {:.6} Nm exceeds {:.6} Nm",
        telemetry.measured_torque,
        maximum_total_torque_nm
    );
    let estimated_total_torque = telemetry.gravity_ff + telemetry.estimated_pd_torque;
    anyhow::ensure!(
        estimated_total_torque.abs() <= maximum_total_torque_nm,
        "{diagnostic_name} estimated commanded total torque {estimated_total_torque:.6} Nm exceeds {maximum_total_torque_nm:.6} Nm"
    );
    anyhow::ensure!(
        (telemetry.kp - expected_kp_nm_rad).abs() <= 1.0e-4
            && (telemetry.kd - expected_kd_nm_s_rad).abs() <= 1.0e-4,
        "{diagnostic_name} target did not retain required true Kp/Kd={expected_kp_nm_rad:.1}/{expected_kd_nm_s_rad:.1}"
    );
    let raw_motor_delta_rev = state.position_rev - initial_motor_position_rev;
    anyhow::ensure!(
        raw_motor_delta_rev.is_finite(),
        "{diagnostic_name} raw motor displacement is non-finite"
    );
    Ok(DiagnosticObservation {
        telemetry,
        driver_temperature_c: state.driver_temperature_c,
        motor_temperature_c: state.motor_temperature_c,
        breakaway_detected: false,
    })
}

fn validate_joint1_first_position_acceptance(
    maximum_positive_delta_rad: f32,
    most_negative_raw_delta_rev: f32,
) -> Result<()> {
    anyhow::ensure!(
        (J1_FIRST_POSITION_REQUIRED_PEAK_RAD..=J1_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD)
            .contains(&maximum_positive_delta_rad),
        "joint_1 positive peak {maximum_positive_delta_rad:.6} rad is outside [{J1_FIRST_POSITION_REQUIRED_PEAK_RAD:.6}, {J1_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD:.6}] rad"
    );
    anyhow::ensure!(
        most_negative_raw_delta_rev <= -J1_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV,
        "joint_1 raw motor peak {most_negative_raw_delta_rev:.9} rev did not reach the required negative change -{J1_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV:.9} rev"
    );
    Ok(())
}

fn validate_diagnostic_start_position(
    joint: &crate::profile::JointProfile,
    measured_q: f32,
    maximum_inward_offset_rad: f32,
) -> Result<()> {
    let lower_bound_error = measured_q - joint.limits.position_lower_rad;
    anyhow::ensure!(
        measured_q.is_finite()
            && lower_bound_error >= -LOW_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD
            && lower_bound_error <= maximum_inward_offset_rad,
        "joint_2 diagnostic start offset from lower limit must be in [{:.6}, {:.6}] rad; lower limit {:.6} rad, measured {:.6} rad (offset {lower_bound_error:.6} rad)",
        -LOW_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD,
        maximum_inward_offset_rad,
        joint.limits.position_lower_rad,
        measured_q
    );
    Ok(())
}

fn compressed_target_words(
    target: &MotorTarget,
    joint: &crate::profile::JointProfile,
) -> (u32, u32) {
    let compressed = hex_motor::cia402::CompressedMitTarget {
        position: target.position_rev,
        velocity: target.velocity_rev_s,
        torque: target.torque_nm,
        kp: target.kp_nm_rev,
        kd: target.kd_nm_s_rev,
    };
    hex_motor::cia402::compressed_mit::packed_target_words(&compressed, &joint.compressed_mapping())
}

fn compressed_target_position_code(
    target: &MotorTarget,
    joint: &crate::profile::JointProfile,
) -> u16 {
    (compressed_target_words(target, joint).1 >> 16) as u16
}

fn build_diagnostic_hold_targets(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    gravity_reference_q: &[f32; DOF],
    selected_index: usize,
    hold_position_rad: f32,
    additive_torque_nm: f32,
    torque_limits: DiagnosticTorqueLimits,
) -> Result<([MotorTarget; DOF], f32)> {
    anyhow::ensure!(
        additive_torque_nm.is_finite()
            && (0.0..=torque_limits.max_additive_torque_nm).contains(&additive_torque_nm),
        "diagnostic additive torque must be finite and in [0, {:.3}] Nm for the selected tier",
        torque_limits.max_additive_torque_nm
    );
    let mut targets = build_safe_hold_targets(
        profile,
        dynamics,
        gravity_reference_q,
        selected_index,
        hold_position_rad,
    )?;
    let joint = &profile.joints[selected_index];
    let model_gravity_ff_nm = motor_torque_to_ros(targets[selected_index].torque_nm, joint);
    let total_feedforward_nm = model_gravity_ff_nm + additive_torque_nm;
    anyhow::ensure!(
        total_feedforward_nm.abs() <= joint.limits.torque_nm,
        "{} diagnostic feed-forward {:.6} Nm exceeds profile limit {:.6} Nm",
        joint.name,
        total_feedforward_nm,
        joint.limits.torque_nm
    );
    anyhow::ensure!(
        total_feedforward_nm.abs() <= torque_limits.max_total_torque_nm,
        "{} diagnostic feed-forward {:.6} Nm exceeds diagnostic hard limit {:.6} Nm",
        joint.name,
        total_feedforward_nm,
        torque_limits.max_total_torque_nm
    );
    targets[selected_index] = ros_target_to_motor(
        RosTarget {
            position_rad: hold_position_rad,
            velocity_rad_s: 0.0,
            torque_nm: total_feedforward_nm,
            kp_nm_rad: joint.default_kp,
            kd_nm_s_rad: joint.default_kd,
        },
        joint,
    );
    Ok((targets, model_gravity_ff_nm))
}

fn diagnostic_temperature_baseline(
    feedback: &FeedbackSnapshot,
    selected_index: usize,
) -> Result<DiagnosticTemperatureBaseline> {
    let state = feedback
        .joints
        .get(selected_index)
        .context("diagnostic temperature joint index is outside feedback")?;
    validate_diagnostic_temperature("driver", state.driver_temperature_c, None)?;
    validate_diagnostic_temperature("motor", state.motor_temperature_c, None)?;
    Ok(DiagnosticTemperatureBaseline {
        driver_c: state.driver_temperature_c,
        motor_c: state.motor_temperature_c,
    })
}

fn joint1_first_position_temperature_baseline(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
) -> Result<SixAxisDiagnosticTemperatureBaseline> {
    anyhow::ensure!(
        profile.joints.len() == DOF,
        "joint_1 first-position temperature baseline requires six profile joints"
    );
    let mut joints = [DiagnosticTemperatureBaseline {
        driver_c: 0.0,
        motor_c: 0.0,
    }; DOF];
    for (index, (profile_joint, feedback_joint)) in
        profile.joints.iter().zip(&feedback.joints).enumerate()
    {
        validate_joint1_first_position_temperature(
            &profile_joint.name,
            "driver",
            feedback_joint.driver_temperature_c,
            feedback_joint.driver_temperature_c,
        )?;
        validate_joint1_first_position_temperature(
            &profile_joint.name,
            "motor",
            feedback_joint.motor_temperature_c,
            feedback_joint.motor_temperature_c,
        )?;
        joints[index] = DiagnosticTemperatureBaseline {
            driver_c: feedback_joint.driver_temperature_c,
            motor_c: feedback_joint.motor_temperature_c,
        };
    }
    Ok(SixAxisDiagnosticTemperatureBaseline { joints })
}

fn validate_six_axis_diagnostic_temperatures(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    baseline: SixAxisDiagnosticTemperatureBaseline,
    diagnostic_name: &str,
) -> Result<()> {
    anyhow::ensure!(
        profile.joints.len() == DOF,
        "{diagnostic_name} temperature gate requires six profile joints"
    );
    for (index, (profile_joint, baseline_joint)) in
        profile.joints.iter().zip(baseline.joints).enumerate()
    {
        let feedback_joint = feedback.joints[index];
        validate_joint1_first_position_temperature(
            &profile_joint.name,
            "driver",
            feedback_joint.driver_temperature_c,
            baseline_joint.driver_c,
        )
        .with_context(|| format!("{diagnostic_name} six-axis temperature gate"))?;
        validate_joint1_first_position_temperature(
            &profile_joint.name,
            "motor",
            feedback_joint.motor_temperature_c,
            baseline_joint.motor_c,
        )
        .with_context(|| format!("{diagnostic_name} six-axis temperature gate"))?;
    }
    Ok(())
}

fn validate_diagnostic_temperature(
    label: &str,
    temperature_c: f32,
    baseline_c: Option<f32>,
) -> Result<()> {
    anyhow::ensure!(
        temperature_c.is_finite()
            && (MIN_PLAUSIBLE_DIAGNOSTIC_TEMPERATURE_C..=MAX_DIAGNOSTIC_TEMPERATURE_C)
                .contains(&temperature_c),
        "joint_2 {label} temperature {temperature_c:.3} C is outside the diagnostic range [{MIN_PLAUSIBLE_DIAGNOSTIC_TEMPERATURE_C:.1}, {MAX_DIAGNOSTIC_TEMPERATURE_C:.1}] C"
    );
    if let Some(baseline_c) = baseline_c {
        anyhow::ensure!(
            temperature_c - baseline_c <= MAX_DIAGNOSTIC_TEMPERATURE_RISE_C,
            "joint_2 {label} temperature rose by {:.3} C from baseline {:.3} C; diagnostic limit is {:.3} C",
            temperature_c - baseline_c,
            baseline_c,
            MAX_DIAGNOSTIC_TEMPERATURE_RISE_C
        );
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn diagnostic_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_q: &[f32; DOF],
    temperature_baseline: DiagnosticTemperatureBaseline,
    target: MotorTarget,
    additive_torque_nm: f32,
    torque_limits: DiagnosticTorqueLimits,
) -> Result<DiagnosticObservation> {
    let measured_q = validate_feedback(profile, feedback, None)?;
    let joint = &profile.joints[selected_index];
    let state = feedback.joints[selected_index];
    validate_diagnostic_temperature(
        "driver",
        state.driver_temperature_c,
        Some(temperature_baseline.driver_c),
    )?;
    validate_diagnostic_temperature(
        "motor",
        state.motor_temperature_c,
        Some(temperature_baseline.motor_c),
    )?;
    let telemetry = commissioning_telemetry_sample(
        joint,
        feedback,
        selected_index,
        initial_q[selected_index],
        measured_q[selected_index],
        initial_q[selected_index],
        target,
    );
    if torque_limits.high_tier {
        let high_tier_absolute_upper =
            joint.limits.position_lower_rad + 2.0 * HIGH_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD;
        anyhow::ensure!(
            measured_q[selected_index] <= high_tier_absolute_upper,
            "joint_2 high-tier absolute position {:.6} rad exceeds lower limit plus 0.004 rad ({high_tier_absolute_upper:.6} rad)",
            measured_q[selected_index]
        );
    }
    anyhow::ensure!(
        telemetry.measured_delta.abs() <= MAX_DIAGNOSTIC_EXCURSION_RAD,
        "joint_2 diagnostic excursion {:.6} rad exceeds hard limit {:.6} rad",
        telemetry.measured_delta,
        MAX_DIAGNOSTIC_EXCURSION_RAD
    );
    anyhow::ensure!(
        telemetry.measured_velocity.abs() <= MAX_DIAGNOSTIC_VELOCITY_RAD_S,
        "joint_2 diagnostic velocity {:.6} rad/s exceeds hard limit {:.6} rad/s",
        telemetry.measured_velocity,
        MAX_DIAGNOSTIC_VELOCITY_RAD_S
    );
    anyhow::ensure!(
        telemetry.gravity_ff.abs() <= torque_limits.max_total_torque_nm,
        "joint_2 diagnostic feed-forward {:.6} Nm exceeds hard limit {:.6} Nm",
        telemetry.gravity_ff,
        torque_limits.max_total_torque_nm
    );
    anyhow::ensure!(
        telemetry.measured_torque.abs() <= torque_limits.max_total_torque_nm,
        "joint_2 measured torque {:.6} Nm exceeds diagnostic hard limit {:.6} Nm",
        telemetry.measured_torque,
        torque_limits.max_total_torque_nm
    );
    let estimated_total_torque = telemetry.gravity_ff + telemetry.estimated_pd_torque;
    anyhow::ensure!(
        estimated_total_torque.abs() <= torque_limits.max_total_torque_nm,
        "joint_2 estimated total torque {estimated_total_torque:.6} Nm exceeds diagnostic hard limit {:.6} Nm",
        torque_limits.max_total_torque_nm
    );
    if additive_torque_nm > 0.0 || torque_limits.high_tier {
        anyhow::ensure!(
            telemetry.measured_delta >= -MAX_DIAGNOSTIC_OPPOSITE_POSITION_RAD,
            "joint_2 moved {:.6} rad opposite the sole permitted positive diagnostic direction",
            telemetry.measured_delta
        );
        anyhow::ensure!(
            telemetry.measured_velocity >= -DIAGNOSTIC_BREAKAWAY_VELOCITY_RAD_S,
            "joint_2 velocity {:.6} rad/s is opposite the sole permitted positive diagnostic direction",
            telemetry.measured_velocity
        );
    }
    let breakaway_detected = additive_torque_nm > 0.0
        && (telemetry.measured_delta >= DIAGNOSTIC_BREAKAWAY_POSITION_RAD
            || telemetry.measured_velocity >= DIAGNOSTIC_BREAKAWAY_VELOCITY_RAD_S);
    Ok(DiagnosticObservation {
        telemetry,
        driver_temperature_c: state.driver_temperature_c,
        motor_temperature_c: state.motor_temperature_c,
        breakaway_detected,
    })
}

#[allow(clippy::too_many_arguments)]
fn position_diagnostic_observation(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_q: &[f32; DOF],
    temperature_baseline: DiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    torque_limits: DiagnosticTorqueLimits,
) -> Result<DiagnosticObservation> {
    let measured_q = validate_feedback(profile, feedback, None)?;
    let joint = &profile.joints[selected_index];
    let state = feedback.joints[selected_index];
    validate_diagnostic_temperature(
        "driver",
        state.driver_temperature_c,
        Some(temperature_baseline.driver_c),
    )?;
    validate_diagnostic_temperature(
        "motor",
        state.motor_temperature_c,
        Some(temperature_baseline.motor_c),
    )?;
    anyhow::ensure!(
        torque_limits.high_tier
            && (torque_limits.max_total_torque_nm - POSITION_DIAGNOSTIC_MAX_TOTAL_TORQUE_NM).abs()
                <= f32::EPSILON,
        "position-round-trip did not receive its dedicated high-tier torque envelope"
    );
    let telemetry = commissioning_telemetry_sample(
        joint,
        feedback,
        selected_index,
        initial_q[selected_index],
        measured_q[selected_index],
        commanded_q,
        target,
    );
    let measured_delta = telemetry.measured_delta;
    anyhow::ensure!(
        measured_delta >= -POSITION_DIAGNOSTIC_MAX_OPPOSITE_POSITION_RAD,
        "joint_2 position-round-trip moved {measured_delta:.6} rad opposite the permitted positive excursion"
    );
    let maximum_positive_delta =
        POSITION_DIAGNOSTIC_DELTA_RAD + POSITION_DIAGNOSTIC_MAX_POSITIVE_MARGIN_RAD;
    anyhow::ensure!(
        measured_delta <= maximum_positive_delta,
        "joint_2 position-round-trip excursion {measured_delta:.6} rad exceeds {maximum_positive_delta:.6} rad"
    );
    let absolute_upper =
        joint.limits.position_lower_rad + POSITION_DIAGNOSTIC_ABSOLUTE_UPPER_OFFSET_RAD;
    anyhow::ensure!(
        measured_q[selected_index] <= absolute_upper,
        "joint_2 position-round-trip absolute position {:.6} rad exceeds lower limit plus {:.3} rad ({absolute_upper:.6} rad)",
        measured_q[selected_index],
        POSITION_DIAGNOSTIC_ABSOLUTE_UPPER_OFFSET_RAD
    );
    anyhow::ensure!(
        telemetry.measured_velocity.abs() <= MAX_DIAGNOSTIC_VELOCITY_RAD_S,
        "joint_2 position-round-trip velocity {:.6} rad/s exceeds {:.6} rad/s",
        telemetry.measured_velocity,
        MAX_DIAGNOSTIC_VELOCITY_RAD_S
    );
    anyhow::ensure!(
        commanded_q >= initial_q[selected_index]
            && commanded_q <= initial_q[selected_index] + POSITION_DIAGNOSTIC_DELTA_RAD,
        "joint_2 position-round-trip command {commanded_q:.6} rad is outside its fixed positive 5 mrad path"
    );
    anyhow::ensure!(
        commanded_q <= absolute_upper,
        "joint_2 position-round-trip target {commanded_q:.6} rad exceeds its absolute upper guard {absolute_upper:.6} rad"
    );
    anyhow::ensure!(
        telemetry.gravity_ff.abs() <= POSITION_DIAGNOSTIC_MAX_FEEDFORWARD_NM,
        "joint_2 position-round-trip feed-forward command {:.6} Nm exceeds its independent {:.6} Nm cap",
        telemetry.gravity_ff,
        POSITION_DIAGNOSTIC_MAX_FEEDFORWARD_NM
    );
    anyhow::ensure!(
        telemetry.measured_torque.abs() <= torque_limits.max_total_torque_nm,
        "joint_2 position-round-trip measured torque {:.6} Nm exceeds {:.6} Nm",
        telemetry.measured_torque,
        torque_limits.max_total_torque_nm
    );
    let estimated_total_torque = telemetry.gravity_ff + telemetry.estimated_pd_torque;
    anyhow::ensure!(
        estimated_total_torque.abs() <= torque_limits.max_total_torque_nm,
        "joint_2 position-round-trip estimated commanded total torque {estimated_total_torque:.6} Nm exceeds {:.6} Nm",
        torque_limits.max_total_torque_nm
    );
    Ok(DiagnosticObservation {
        telemetry,
        driver_temperature_c: state.driver_temperature_c,
        motor_temperature_c: state.motor_temperature_c,
        breakaway_detected: false,
    })
}

fn validate_position_gravity_ramp_observation(
    profile: &HardwareProfile,
    selected_index: usize,
    observation: DiagnosticObservation,
) -> Result<()> {
    let joint = &profile.joints[selected_index];
    let telemetry = observation.telemetry;
    anyhow::ensure!(
        telemetry.measured_delta >= -POSITION_DIAGNOSTIC_MAX_OPPOSITE_POSITION_RAD,
        "joint_2 gravity ramp moved {:.6} rad opposite its permitted direction",
        telemetry.measured_delta
    );
    anyhow::ensure!(
        telemetry.measured_delta <= POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_POSITIVE_RAD,
        "joint_2 gravity-ramp excursion {:.6} rad exceeds {:.6} rad",
        telemetry.measured_delta,
        POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_POSITIVE_RAD
    );
    let absolute_upper = joint.limits.position_lower_rad
        + POSITION_DIAGNOSTIC_GRAVITY_RAMP_ABSOLUTE_UPPER_OFFSET_RAD;
    anyhow::ensure!(
        telemetry.measured_q <= absolute_upper,
        "joint_2 gravity-ramp absolute position {:.6} rad exceeds lower limit plus {:.3} rad ({absolute_upper:.6} rad)",
        telemetry.measured_q,
        POSITION_DIAGNOSTIC_GRAVITY_RAMP_ABSOLUTE_UPPER_OFFSET_RAD
    );
    anyhow::ensure!(
        telemetry.measured_velocity.abs() <= MAX_DIAGNOSTIC_VELOCITY_RAD_S,
        "joint_2 gravity-ramp velocity {:.6} rad/s exceeds the unchanged {:.6} rad/s hard limit",
        telemetry.measured_velocity,
        MAX_DIAGNOSTIC_VELOCITY_RAD_S
    );
    Ok(())
}

fn validate_position_gravity_ramp_go_peaks(
    peak_position_delta_rad: f32,
    peak_velocity_rad_s: f32,
) -> Result<()> {
    anyhow::ensure!(
        peak_position_delta_rad <= POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_POSITION_RAD,
        "joint_2 gravity-ramp peak excursion {peak_position_delta_rad:.6} rad exceeds the {POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_POSITION_RAD:.6} rad trajectory GO limit"
    );
    anyhow::ensure!(
        peak_velocity_rad_s <= POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_VELOCITY_RAD_S,
        "joint_2 gravity-ramp peak velocity {peak_velocity_rad_s:.6} rad/s exceeds the {POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_VELOCITY_RAD_S:.6} rad/s trajectory GO limit"
    );
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum CensoredGravityHoldStop {
    DisplacementCensor {
        trigger_step: usize,
        trigger_scale: f32,
        trigger_position_delta_rad: f32,
        trigger_velocity_rad_s: f32,
    },
    ScaleCap,
}

impl CensoredGravityHoldStop {
    fn label(self) -> &'static str {
        match self {
            Self::DisplacementCensor { .. } => "displacement_censor",
            Self::ScaleCap => "scale_cap_right_censored",
        }
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_censored_gravity_hold(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    selected_index: usize,
    initial_q: &[f32; DOF],
    gravity_reference_q: &[f32; DOF],
    temperature_baseline: DiagnosticTemperatureBaseline,
    initial_oldest_tpdo1_at: Option<Instant>,
    published_targets: [MotorTarget; DOF],
    registered_targets: [MotorTarget; DOF],
    diagnostic_started: Instant,
    torque_limits: DiagnosticTorqueLimits,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let joint = &profile.joints[selected_index];
    let expected_start = build_position_diagnostic_hold_targets(
        profile,
        dynamics,
        gravity_reference_q,
        selected_index,
        initial_q[selected_index],
        CENSORED_GRAVITY_HOLD_PROFILE_SCALE,
    )?;
    anyhow::ensure!(
        compressed_target_words(&published_targets[selected_index], joint)
            == compressed_target_words(&expected_start[selected_index], joint),
        "censored gravity hold did not start from the exact 0.25-scale target"
    );
    let fixed_position_code =
        compressed_target_position_code(&expected_start[selected_index], joint);
    tracing::warn!(
        phase = PHASE_DIAGNOSTIC_CENSORED_GRAVITY_HOLD,
        joint = %joint.name,
        node_id = joint.node_id,
        initial_gravity_scale = CENSORED_GRAVITY_HOLD_PROFILE_SCALE,
        maximum_gravity_scale = CENSORED_GRAVITY_HOLD_CAP_SCALE,
        transitions = CENSORED_GRAVITY_HOLD_STEPS,
        nominal_ramp_duration_sec = CENSORED_GRAVITY_HOLD_NOMINAL_DURATION_SEC,
        displacement_censor_rad = CENSORED_GRAVITY_HOLD_TRIGGER_RAD,
        identification_position_exit_rad = CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD,
        identification_velocity_exit_rad_s = CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S,
        hard_negative_excursion_rad = -POSITION_DIAGNOSTIC_MAX_OPPOSITE_POSITION_RAD,
        hard_positive_excursion_rad = POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_POSITIVE_RAD,
        hard_velocity_rad_s = MAX_DIAGNOSTIC_VELOCITY_RAD_S,
        freeze_duration_sec = CENSORED_GRAVITY_HOLD_FREEZE_DURATION.as_secs_f32(),
        "starting feedback-censored fixed-position gravity hold; this mode has no position trajectory"
    );
    let mut progress = PositionRampTargetProgress::new(published_targets, registered_targets);
    let mut last_feedback_at = initial_oldest_tpdo1_at
        .context("censored gravity hold requires an initial six-axis oldest-TPDO1 timestamp")?;
    let mut target_proof_after = Instant::now() + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
    let mut step = 0_usize;
    let mut verified_history = CensoredVerifiedTargetHistory::default();
    verified_history.record(
        0,
        CENSORED_GRAVITY_HOLD_PROFILE_SCALE,
        registered_targets,
        selected_index,
        joint,
    );
    let mut milestones = MotionTelemetryMilestones::default();
    let mut displacement_milestones = CensoredGravityHoldDisplacementMilestones::default();

    loop {
        anyhow::ensure!(
            diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
            "joint_2 censored gravity hold exceeded its {:.3} s active-time limit during ramp",
            torque_limits.max_active_sec
        );
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during the censored gravity-hold ramp"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed during censored gravity ramp")?;
        let feedback = backend.feedback();
        let current_scale = censored_gravity_hold_scale(step)?;
        let observation = position_diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            progress.published[selected_index],
            initial_q[selected_index],
            torque_limits,
        )
        .context("published censored gravity-hold target failed its fresh-feedback gate")?;
        validate_position_gravity_ramp_observation(profile, selected_index, observation).context(
            "published censored gravity-hold target left its fixed-position hard envelope",
        )?;
        peaks.observe(observation);

        while let Some((threshold_rad, milestone)) =
            displacement_milestones.take_due(observation.telemetry.measured_delta)
        {
            tracing::info!(
                phase = PHASE_DIAGNOSTIC_CENSORED_GRAVITY_HOLD,
                joint = %joint.name,
                node_id = joint.node_id,
                milestone,
                displacement_threshold_rad = threshold_rad,
                step,
                gravity_scale = current_scale,
                gravity_ff_nm = observation.telemetry.gravity_ff,
                estimated_pd_torque_nm = observation.telemetry.estimated_pd_torque,
                estimated_total_torque_nm = observation.telemetry.gravity_ff
                    + observation.telemetry.estimated_pd_torque,
                measured_torque_nm = observation.telemetry.measured_torque,
                measured_delta_rad = observation.telemetry.measured_delta,
                measured_velocity_rad_s = observation.telemetry.measured_velocity,
                "censored gravity-hold positive-displacement milestone crossed before target promotion"
            );
        }

        let feedback_at = feedback.oldest_tpdo1_at;
        let target_is_feedback_proven = feedback_at.is_some_and(|captured_at| {
            position_gravity_ramp_feedback_proves_target(
                captured_at,
                last_feedback_at,
                target_proof_after,
            )
        });
        let poll_decision =
            censored_gravity_hold_poll_decision(observation, target_is_feedback_proven)
                .with_context(|| {
                    format!(
                        "censored gravity-hold identification layer at scale {current_scale:.6}"
                    )
                })?;

        if poll_decision == CensoredGravityHoldPollDecision::FreezePrevious {
            // Crucially, do not call confirm_published(): the target that
            // elicited the censor never becomes signal/error cleanup state.
            let frozen = verified_history
                .rollback_candidate(
                    &progress.published,
                    &registered_targets,
                    selected_index,
                    joint,
                )
                .context(
                    "censored displacement reached before a non-0.25, quantized-distinct feedback-verified rollback target existed",
                )?;
            let frozen_targets = frozen.targets;
            let frozen_scale = frozen.scale;
            let frozen_words = compressed_target_words(&frozen_targets[selected_index], joint);
            anyhow::ensure!(
                compressed_target_position_code(&frozen_targets[selected_index], joint)
                    == fixed_position_code,
                "censored gravity-hold rollback target changed the selected position code"
            );
            anyhow::ensure!(
                frozen_scale > CENSORED_GRAVITY_HOLD_PROFILE_SCALE
                    && frozen_words
                        != compressed_target_words(&progress.published[selected_index], joint)
                    && frozen_words
                        != compressed_target_words(&registered_targets[selected_index], joint),
                "censored gravity-hold rollback target is not distinct from both the rejected target and configured 0.25 target"
            );
            backend
                .ensure_single_axis_commissioning_state(selected_index)
                .context("single-axis drive-state contract failed before censor rollback")?;
            anyhow::ensure!(
                !backend.transport_failed(),
                "CAN transport failed before censor rollback"
            );
            // This target was already proven by fresh feedback earlier in the
            // ramp. Register it before publishing so asynchronous signal/error
            // cleanup cannot restore the now-rejected quantized wire target.
            backend
                .register_single_axis_diagnostic_baseline(selected_index, frozen_targets)
                .context("register quantized-distinct verified censor rollback target")?;
            backend
                .set_targets(frozen_targets)
                .await
                .context("restore the previous feedback-verified gravity microstep")?;
            let freeze_started = Instant::now();
            let freeze_proof_after = freeze_started + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
            tracing::warn!(
                phase = PHASE_DIAGNOSTIC_CENSORED_GRAVITY_HOLD,
                joint = %joint.name,
                node_id = joint.node_id,
                trigger_step = step,
                trigger_scale = current_scale,
                trigger_position_delta_rad = observation.telemetry.measured_delta,
                trigger_velocity_rad_s = observation.telemetry.measured_velocity,
                frozen_verified_step = frozen.step,
                frozen_verified_scale = frozen_scale,
                "censored displacement reached; rejected the current target and froze a prior feedback-verified, quantized-distinct microstep"
            );
            return run_censored_gravity_hold_frozen_identification(
                backend,
                profile,
                selected_index,
                initial_q,
                temperature_baseline,
                registered_targets,
                frozen_targets,
                frozen_scale,
                CensoredGravityHoldStop::DisplacementCensor {
                    trigger_step: step,
                    trigger_scale: current_scale,
                    trigger_position_delta_rad: observation.telemetry.measured_delta,
                    trigger_velocity_rad_s: observation.telemetry.measured_velocity,
                },
                feedback_at.unwrap_or(last_feedback_at),
                freeze_proof_after,
                freeze_started,
                diagnostic_started,
                torque_limits,
                peaks,
            )
            .await;
        }

        if poll_decision == CensoredGravityHoldPollDecision::AwaitTargetProof {
            anyhow::ensure!(
                feedback_at.is_some(),
                "censored gravity-hold feedback has no six-axis oldest-TPDO1 timestamp"
            );
            tokio::time::sleep(LOOP_PERIOD).await;
            continue;
        }
        anyhow::ensure!(
            poll_decision == CensoredGravityHoldPollDecision::ConfirmPublished,
            "unexpected censored gravity-hold polling decision"
        );
        last_feedback_at = feedback_at
            .context("feedback-proven censored target has no six-axis oldest-TPDO1 timestamp")?;

        let verified_targets = progress.confirm_published();
        backend
            .register_single_axis_diagnostic_baseline(selected_index, verified_targets)
            .context("advance persistent baseline to verified censored gravity target")?;
        verified_history.record(step, current_scale, verified_targets, selected_index, joint);

        let normalized = step as f32 / CENSORED_GRAVITY_HOLD_STEPS as f32;
        while let Some(milestone) = milestones.take_due(normalized) {
            log_diagnostic_telemetry(
                milestone,
                PHASE_DIAGNOSTIC_CENSORED_GRAVITY_HOLD,
                joint,
                step,
                CENSORED_GRAVITY_HOLD_STEPS,
                observation.telemetry.gravity_ff,
                0.0,
                diagnostic_started.elapsed(),
                observation,
            );
        }

        if step == CENSORED_GRAVITY_HOLD_STEPS {
            // Re-publish the already verified cap target so the bounded freeze
            // and exact readback have their own two-sender-period proof.
            backend.set_targets(verified_targets).await?;
            let freeze_started = Instant::now();
            let freeze_proof_after = freeze_started + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
            tracing::info!(
                phase = PHASE_DIAGNOSTIC_CENSORED_GRAVITY_HOLD,
                joint = %joint.name,
                node_id = joint.node_id,
                transitions = CENSORED_GRAVITY_HOLD_STEPS,
                nominal_duration_sec = CENSORED_GRAVITY_HOLD_NOMINAL_DURATION_SEC,
                frozen_verified_scale = current_scale,
                "censored gravity hold reached its fixed scale cap without reaching the displacement censor"
            );
            return run_censored_gravity_hold_frozen_identification(
                backend,
                profile,
                selected_index,
                initial_q,
                temperature_baseline,
                registered_targets,
                verified_targets,
                current_scale,
                CensoredGravityHoldStop::ScaleCap,
                last_feedback_at,
                freeze_proof_after,
                freeze_started,
                diagnostic_started,
                torque_limits,
                peaks,
            )
            .await;
        }

        let next_step = step + 1;
        let next_scale = censored_gravity_hold_scale(next_step)?;
        anyhow::ensure!(
            next_scale >= current_scale
                && next_scale <= CENSORED_GRAVITY_HOLD_CAP_SCALE
                && next_scale - current_scale <= CENSORED_GRAVITY_HOLD_MAX_SCALE_STEP,
            "censored gravity-hold scale transition {current_scale:.6}->{next_scale:.6} violates its monotonic, endpoint, or step cap"
        );
        let prospective_targets = build_position_diagnostic_hold_targets(
            profile,
            dynamics,
            gravity_reference_q,
            selected_index,
            initial_q[selected_index],
            next_scale,
        )?;
        anyhow::ensure!(
            compressed_target_position_code(&prospective_targets[selected_index], joint)
                == fixed_position_code,
            "censored gravity-hold prospective target changed the selected position code"
        );
        let prospective = position_diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            prospective_targets[selected_index],
            initial_q[selected_index],
            torque_limits,
        )
        .context("prospective censored gravity-hold target failed its torque/temperature gate")?;
        validate_position_gravity_ramp_observation(profile, selected_index, prospective).context(
            "prospective censored gravity-hold target left its fixed-position hard envelope",
        )?;
        validate_censored_gravity_hold_identification_layer(prospective)
            .context("prospective censored gravity-hold target failed the identification layer")?;
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed before censored gravity publish")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed before the next censored gravity target"
        );
        backend.set_targets(prospective_targets).await?;
        progress.publish(prospective_targets);
        target_proof_after = Instant::now() + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
        step = next_step;
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_censored_gravity_hold_frozen_identification(
    backend: &RealBackend,
    profile: &HardwareProfile,
    selected_index: usize,
    initial_q: &[f32; DOF],
    temperature_baseline: DiagnosticTemperatureBaseline,
    configured_initial_targets: [MotorTarget; DOF],
    frozen_targets: [MotorTarget; DOF],
    frozen_scale: f32,
    stop: CensoredGravityHoldStop,
    previous_feedback_at: Instant,
    target_proof_after: Instant,
    freeze_started: Instant,
    diagnostic_started: Instant,
    torque_limits: DiagnosticTorqueLimits,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let joint = &profile.joints[selected_index];
    let freeze_deadline = freeze_started + CENSORED_GRAVITY_HOLD_FREEZE_DURATION;
    anyhow::ensure!(
        compressed_target_position_code(&frozen_targets[selected_index], joint)
            == compressed_target_position_code(&configured_initial_targets[selected_index], joint),
        "frozen censored gravity-hold target changed the configured position code"
    );
    anyhow::ensure!(
        compressed_target_words(&frozen_targets[selected_index], joint)
            != compressed_target_words(&configured_initial_targets[selected_index], joint),
        "frozen censored gravity-hold target did not change the configured 0.25-scale words; exact RPDO proof is impossible"
    );

    let (mut last_feedback_at, proof_observation) = loop {
        anyhow::ensure!(
            Instant::now() < freeze_deadline,
            "censored gravity-hold frozen target was not feedback-proven within its one-second identification window"
        );
        anyhow::ensure!(
            diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
            "joint_2 censored gravity hold exceeded its {:.3} s active-time limit while proving the frozen target",
            torque_limits.max_active_sec
        );
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed while proving the frozen censored gravity target"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context(
                "single-axis drive-state contract failed while proving frozen gravity target",
            )?;
        let feedback = backend.feedback();
        let observation = position_diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            frozen_targets[selected_index],
            initial_q[selected_index],
            torque_limits,
        )?;
        validate_position_gravity_ramp_observation(profile, selected_index, observation)?;
        validate_censored_gravity_hold_identification_layer(observation)?;
        peaks.observe(observation);
        let feedback_at = feedback
            .oldest_tpdo1_at
            .context("frozen censored gravity feedback has no six-axis oldest-TPDO1 timestamp")?;
        if !position_gravity_ramp_feedback_proves_target(
            feedback_at,
            previous_feedback_at,
            target_proof_after,
        ) {
            tokio::time::sleep(LOOP_PERIOD).await;
            continue;
        }
        break (feedback_at, observation);
    };

    let readback_milestone = match stop {
        CensoredGravityHoldStop::DisplacementCensor { .. } => "censor_previous_verified_target",
        CensoredGravityHoldStop::ScaleCap => "right_censored_scale_cap_target",
    };
    let readback = confirm_position_target_readback_while_guarded(
        backend,
        profile,
        selected_index,
        initial_q,
        temperature_baseline,
        frozen_targets[selected_index],
        initial_q[selected_index],
        diagnostic_started,
        torque_limits,
        PHASE_DIAGNOSTIC_CENSORED_GRAVITY_HOLD,
        readback_milestone,
        false,
        true,
        peaks,
    );
    tokio::time::timeout_at(tokio::time::Instant::from_std(freeze_deadline), readback)
        .await
        .context("exact frozen-target readback exceeded the one-second censored hold")??;

    let mut statistics = CensoredHoldStatistics::default();
    statistics.observe(last_feedback_at, proof_observation)?;
    while Instant::now() < freeze_deadline {
        anyhow::ensure!(
            diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
            "joint_2 censored gravity hold exceeded its {:.3} s active-time limit during frozen statistics",
            torque_limits.max_active_sec
        );
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during censored gravity-hold statistics"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed during frozen statistics")?;
        let feedback = backend.feedback();
        let observation = position_diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            frozen_targets[selected_index],
            initial_q[selected_index],
            torque_limits,
        )?;
        validate_position_gravity_ramp_observation(profile, selected_index, observation)?;
        validate_censored_gravity_hold_identification_layer(observation)?;
        peaks.observe(observation);
        let feedback_at = feedback
            .oldest_tpdo1_at
            .context("censored gravity-hold statistics have no six-axis oldest-TPDO1 timestamp")?;
        if feedback_at <= last_feedback_at {
            tokio::time::sleep(LOOP_PERIOD).await;
            continue;
        }
        last_feedback_at = feedback_at;
        statistics.observe(feedback_at, observation)?;
        tokio::time::sleep(LOOP_PERIOD).await;
    }

    let summary = statistics.summarize()?;
    let (trigger_step, trigger_scale, trigger_position_delta_rad, trigger_velocity_rad_s) =
        match stop {
            CensoredGravityHoldStop::DisplacementCensor {
                trigger_step,
                trigger_scale,
                trigger_position_delta_rad,
                trigger_velocity_rad_s,
            } => (
                Some(trigger_step),
                Some(trigger_scale),
                Some(trigger_position_delta_rad),
                Some(trigger_velocity_rad_s),
            ),
            CensoredGravityHoldStop::ScaleCap => (None, None, None, None),
        };
    tracing::info!(
        phase = PHASE_DIAGNOSTIC_CENSORED_GRAVITY_HOLD,
        joint = %joint.name,
        node_id = joint.node_id,
        stop_reason = stop.label(),
        trigger_step = ?trigger_step,
        trigger_scale = ?trigger_scale,
        trigger_position_delta_rad = ?trigger_position_delta_rad,
        trigger_velocity_rad_s = ?trigger_velocity_rad_s,
        frozen_verified_scale = frozen_scale,
        sample_count = summary.sample_count,
        mean_position_delta_rad = summary.mean_position_delta_rad,
        position_stddev_rad = summary.position_stddev_rad,
        minimum_position_delta_rad = summary.minimum_position_delta_rad,
        maximum_position_delta_rad = summary.maximum_position_delta_rad,
        peak_velocity_rad_s = summary.peak_velocity_rad_s,
        mean_measured_torque_nm = summary.mean_measured_torque_nm,
        mean_estimated_total_torque_nm = summary.mean_estimated_total_torque_nm,
        peak_driver_temperature_c = summary.peak_driver_temperature_c,
        peak_motor_temperature_c = summary.peak_motor_temperature_c,
        terminal_sample_count = summary.terminal_sample_count,
        terminal_position_span_rad = summary.terminal_position_span_rad,
        terminal_peak_velocity_rad_s = summary.terminal_peak_velocity_rad_s,
        terminal_stable = summary.terminal_stable,
        freeze_duration_sec = CENSORED_GRAVITY_HOLD_FREEZE_DURATION.as_secs_f32(),
        "censored gravity-hold frozen-target statistics completed; no position trajectory exists in this mode"
    );
    anyhow::ensure!(
        summary.terminal_stable,
        "censored gravity-hold frozen target was not terminally stable: last {:.3} s span {:.6} rad (limit {:.6}), peak velocity {:.6} rad/s (limit {:.6}), samples {} (minimum {})",
        CENSORED_GRAVITY_HOLD_TERMINAL_DWELL.as_secs_f32(),
        summary.terminal_position_span_rad,
        CENSORED_GRAVITY_HOLD_TERMINAL_POSITION_SPAN_RAD,
        summary.terminal_peak_velocity_rad_s,
        CENSORED_GRAVITY_HOLD_TERMINAL_VELOCITY_RAD_S,
        summary.terminal_sample_count,
        CENSORED_GRAVITY_HOLD_MIN_TERMINAL_SAMPLES
    );
    Ok(())
}

/// Raise only the selected-axis gravity feed-forward from the previously
/// exercised 0.25 scale to the profile-locked 0.65 scale. Every transition is
/// separated by at least 4 ms and by strictly newer TPDO1 feedback from all six
/// axes. The ordinary feedback gates still require fresh TPDO1 and TPDO2 on
/// every axis. Cleanup retains the last target proven by such a snapshot,
/// never the next prospective target.
#[allow(clippy::too_many_arguments)]
async fn run_position_diagnostic_gravity_ramp(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    selected_index: usize,
    initial_q: &[f32; DOF],
    gravity_reference_q: &[f32; DOF],
    temperature_baseline: DiagnosticTemperatureBaseline,
    initial_oldest_tpdo1_at: Option<Instant>,
    published_targets: [MotorTarget; DOF],
    registered_targets: [MotorTarget; DOF],
    diagnostic_started: Instant,
    torque_limits: DiagnosticTorqueLimits,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let joint = &profile.joints[selected_index];
    let expected_start = build_position_diagnostic_hold_targets(
        profile,
        dynamics,
        gravity_reference_q,
        selected_index,
        initial_q[selected_index],
        POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE,
    )?;
    anyhow::ensure!(
        compressed_target_words(&published_targets[selected_index], joint)
            == compressed_target_words(&expected_start[selected_index], joint),
        "joint_2 gravity ramp did not start from the exact 0.25-scale hold target"
    );

    let mut progress = PositionRampTargetProgress::new(published_targets, registered_targets);
    let mut last_feedback_at = initial_oldest_tpdo1_at
        .context("joint_2 gravity ramp requires an initial six-axis oldest-TPDO1 timestamp")?;
    // `published_targets` was installed by the final low-scale stability-loop
    // update immediately before it returned. Do not promote it until feedback
    // is both new and necessarily later than two shared-sender periods.
    let mut target_proof_after = Instant::now() + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
    let mut step = 0_usize;
    let mut peak_position_delta_rad = 0.0_f32;
    let mut peak_velocity_rad_s = 0.0_f32;
    let mut milestones = MotionTelemetryMilestones::default();

    loop {
        anyhow::ensure!(
            diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
            "joint_2 position diagnostic exceeded its {:.3} s active-time limit during the gravity ramp",
            torque_limits.max_active_sec
        );
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during the joint_2 gravity ramp"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed during the gravity ramp")?;

        let feedback = backend.feedback();
        let feedback_at = feedback
            .oldest_tpdo1_at
            .context("joint_2 gravity ramp feedback has no six-axis oldest-TPDO1 timestamp")?;
        if !position_gravity_ramp_feedback_proves_target(
            feedback_at,
            last_feedback_at,
            target_proof_after,
        ) {
            tokio::time::sleep(LOOP_PERIOD).await;
            continue;
        }
        last_feedback_at = feedback_at;

        let observation = position_diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            progress.published[selected_index],
            initial_q[selected_index],
            torque_limits,
        )
        .context("published joint_2 gravity-ramp target failed its fresh-feedback gate")?;
        validate_position_gravity_ramp_observation(profile, selected_index, observation)
            .context("published joint_2 gravity-ramp target left its fixed-position envelope")?;
        peaks.observe(observation);
        peak_position_delta_rad =
            peak_position_delta_rad.max(observation.telemetry.measured_delta.abs());
        peak_velocity_rad_s =
            peak_velocity_rad_s.max(observation.telemetry.measured_velocity.abs());

        // Only this fresh, fully guarded observation may advance the
        // cancellation baseline. The next target remains merely prospective
        // until the following iteration proves it in the same way.
        let verified_targets = progress.confirm_published();
        backend
            .register_single_axis_diagnostic_baseline(selected_index, verified_targets)
            .context("advance persistent baseline to feedback-verified gravity-ramp target")?;

        let normalized = step as f32 / POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS as f32;
        while let Some(milestone) = milestones.take_due(normalized) {
            log_diagnostic_telemetry(
                milestone,
                PHASE_DIAGNOSTIC_POSITION_GRAVITY_RAMP,
                joint,
                step,
                POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS,
                observation.telemetry.gravity_ff,
                0.0,
                diagnostic_started.elapsed(),
                observation,
            );
        }

        if step == POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS {
            validate_position_gravity_ramp_go_peaks(peak_position_delta_rad, peak_velocity_rad_s)?;
            tracing::info!(
                phase = PHASE_DIAGNOSTIC_POSITION_GRAVITY_RAMP,
                joint = %joint.name,
                node_id = joint.node_id,
                transitions = POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS,
                nominal_duration_sec = POSITION_DIAGNOSTIC_GRAVITY_RAMP_DURATION_SEC,
                final_gravity_scale = POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE,
                final_gravity_ff_nm = observation.telemetry.gravity_ff,
                peak_position_delta_rad,
                peak_velocity_rad_s,
                "feedback-driven joint_2 gravity ramp passed its trajectory GO gates"
            );
            return Ok(());
        }

        let next_step = step + 1;
        let current_scale = position_diagnostic_gravity_ramp_scale(step)?;
        let next_scale = position_diagnostic_gravity_ramp_scale(next_step)?;
        anyhow::ensure!(
            next_scale >= current_scale
                && next_scale - current_scale
                    <= POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_SCALE_STEP,
            "joint_2 gravity-ramp scale transition {current_scale:.6}->{next_scale:.6} violates its monotonic step cap"
        );
        let prospective_targets = build_position_diagnostic_hold_targets(
            profile,
            dynamics,
            gravity_reference_q,
            selected_index,
            initial_q[selected_index],
            next_scale,
        )?;
        let prospective = position_diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            prospective_targets[selected_index],
            initial_q[selected_index],
            torque_limits,
        )
        .context("prospective joint_2 gravity-ramp target failed its torque/temperature gate")?;
        validate_position_gravity_ramp_observation(profile, selected_index, prospective)
            .context("prospective joint_2 gravity-ramp target left its fixed-position envelope")?;
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed before the next joint_2 gravity-ramp target"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed before gravity-ramp publish")?;
        backend.set_targets(prospective_targets).await?;
        progress.publish(prospective_targets);
        target_proof_after = Instant::now() + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
        step = next_step;
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

fn validate_position_return_observation(observation: DiagnosticObservation) -> Result<()> {
    anyhow::ensure!(
        observation.telemetry.measured_delta.abs() <= POSITION_DIAGNOSTIC_RETURN_POSITION_RAD,
        "joint_2 returned-baseline position error {:.6} rad exceeds {:.6} rad",
        observation.telemetry.measured_delta.abs(),
        POSITION_DIAGNOSTIC_RETURN_POSITION_RAD
    );
    anyhow::ensure!(
        observation.telemetry.measured_velocity.abs()
            <= POSITION_DIAGNOSTIC_STABILITY_VELOCITY_RAD_S,
        "joint_2 returned-baseline velocity {:.6} rad/s exceeds {:.6} rad/s",
        observation.telemetry.measured_velocity,
        POSITION_DIAGNOSTIC_STABILITY_VELOCITY_RAD_S
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn confirm_position_target_readback_while_guarded(
    backend: &RealBackend,
    profile: &HardwareProfile,
    selected_index: usize,
    initial_q: &[f32; DOF],
    temperature_baseline: DiagnosticTemperatureBaseline,
    target: MotorTarget,
    commanded_q: f32,
    diagnostic_started: Instant,
    torque_limits: DiagnosticTorqueLimits,
    phase: &'static str,
    readback_milestone: &'static str,
    enforce_return_envelope: bool,
    enforce_censored_identification_layer: bool,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let settle_started = Instant::now();
    while settle_started.elapsed() < RPDO_READBACK_SETTLE {
        anyhow::ensure!(
            diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
            "joint_2 position-round-trip exceeded its {:.3} s active-time limit before SDO readback",
            torque_limits.max_active_sec
        );
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed before joint_2 position-round-trip SDO readback"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed before position SDO readback")?;
        let feedback = backend.feedback();
        let observation = position_diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            target,
            commanded_q,
            torque_limits,
        )?;
        if enforce_return_envelope {
            validate_position_return_observation(observation)?;
        }
        if enforce_censored_identification_layer {
            validate_censored_gravity_hold_identification_layer(observation)?;
        }
        peaks.observe(observation);
        tokio::time::sleep(LOOP_PERIOD).await;
    }
    let mut readback_future =
        Box::pin(backend.confirm_commissioning_target_readback(selected_index, target));
    let readback = loop {
        tokio::select! {
            result = &mut readback_future => {
                break result.context("position-round-trip exact compressed-MIT target readback failed")?;
            }
            _ = tokio::time::sleep(LOOP_PERIOD) => {
                anyhow::ensure!(
                    diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
                    "joint_2 position-round-trip exceeded its {:.3} s active-time limit during SDO readback",
                    torque_limits.max_active_sec
                );
                anyhow::ensure!(
                    !backend.transport_failed(),
                    "CAN transport failed during joint_2 position-round-trip SDO readback"
                );
                backend
                    .ensure_single_axis_commissioning_state(selected_index)
                    .context("single-axis drive-state contract failed during position SDO readback")?;
                let feedback = backend.feedback();
                let observation = position_diagnostic_observation(
                    profile,
                    &feedback,
                    selected_index,
                    initial_q,
                    temperature_baseline,
                    target,
                    commanded_q,
                    torque_limits,
                )?;
                if enforce_return_envelope {
                    validate_position_return_observation(observation)?;
                }
                if enforce_censored_identification_layer {
                    validate_censored_gravity_hold_identification_layer(observation)?;
                }
                peaks.observe(observation);
            }
        }
    };
    anyhow::ensure!(
        diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
        "joint_2 position-round-trip exceeded its {:.3} s active-time limit during SDO readback",
        torque_limits.max_active_sec
    );
    anyhow::ensure!(
        !backend.transport_failed(),
        "CAN transport failed after joint_2 position-round-trip SDO readback"
    );
    backend
        .ensure_single_axis_commissioning_state(selected_index)
        .context("single-axis drive-state contract failed after position SDO readback")?;
    let feedback = backend.feedback();
    let observation = position_diagnostic_observation(
        profile,
        &feedback,
        selected_index,
        initial_q,
        temperature_baseline,
        target,
        commanded_q,
        torque_limits,
    )?;
    if enforce_return_envelope {
        validate_position_return_observation(observation)?;
    }
    if enforce_censored_identification_layer {
        validate_censored_gravity_hold_identification_layer(observation)?;
    }
    peaks.observe(observation);
    let joint = &profile.joints[selected_index];
    tracing::info!(
        phase,
        milestone = readback_milestone,
        joint = %joint.name,
        node_id = joint.node_id,
        commanded_q,
        expected_lower = %format_args!("0x{:08X}", readback.expected_lower),
        expected_upper = %format_args!("0x{:08X}", readback.expected_upper),
        actual_lower = %format_args!("0x{:08X}", readback.actual_lower),
        actual_upper = %format_args!("0x{:08X}", readback.actual_upper),
        "held position target matched exact 0x2004:02/03 readback"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn run_position_round_trip_diagnostic(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    selected_index: usize,
    initial_q: &[f32; DOF],
    temperature_baseline: DiagnosticTemperatureBaseline,
    baseline_targets: [MotorTarget; DOF],
    delta_rad: f32,
    duration_sec: f32,
    diagnostic_started: Instant,
    torque_limits: DiagnosticTorqueLimits,
    peaks: &mut DiagnosticPeaks,
) -> Result<()> {
    let joint = &profile.joints[selected_index];
    let baseline_position_code =
        compressed_target_position_code(&baseline_targets[selected_index], joint);
    let trajectory_started = Instant::now();
    let trajectory_duration = Duration::from_secs_f32(duration_sec);
    let mut readback_pause = Duration::ZERO;
    let mut readback_confirmed = false;
    let mut max_positive_delta = 0.0_f32;
    let mut milestones = MotionTelemetryMilestones::default();

    loop {
        let trajectory_elapsed = trajectory_started.elapsed().saturating_sub(readback_pause);
        if trajectory_elapsed >= trajectory_duration {
            break;
        }
        anyhow::ensure!(
            diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
            "joint_2 position-round-trip exceeded its {:.3} s active-time limit",
            torque_limits.max_active_sec
        );
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_2 position-round-trip"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed during position-round-trip")?;
        let feedback = backend.feedback();
        let measured_q = validate_feedback(profile, &feedback, None)?;
        let normalized_time = trajectory_elapsed.as_secs_f32() / duration_sec;
        let commanded_q = initial_q[selected_index] + round_trip_phase(normalized_time) * delta_rad;
        let targets =
            build_safe_hold_targets(profile, dynamics, &measured_q, selected_index, commanded_q)?;
        let observation = position_diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            targets[selected_index],
            commanded_q,
            torque_limits,
        )?;
        peaks.observe(observation);
        max_positive_delta = max_positive_delta.max(observation.telemetry.measured_delta);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed before position target publish")?;
        backend.set_targets(targets).await?;

        while let Some(milestone) = milestones.take_due(normalized_time) {
            log_diagnostic_telemetry(
                milestone,
                PHASE_DIAGNOSTIC_POSITION_ROUND_TRIP,
                joint,
                1,
                1,
                observation.telemetry.gravity_ff,
                0.0,
                diagnostic_started.elapsed(),
                observation,
            );
        }

        let position_code = compressed_target_position_code(&targets[selected_index], joint);
        if !readback_confirmed && position_code != baseline_position_code {
            let readback_started = Instant::now();
            confirm_position_target_readback_while_guarded(
                backend,
                profile,
                selected_index,
                initial_q,
                temperature_baseline,
                targets[selected_index],
                commanded_q,
                diagnostic_started,
                torque_limits,
                PHASE_DIAGNOSTIC_POSITION_ROUND_TRIP,
                "first_quantized_nonbaseline_position",
                false,
                false,
                peaks,
            )
            .await?;
            readback_pause += readback_started.elapsed();
            readback_confirmed = true;
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }

    anyhow::ensure!(
        readback_confirmed,
        "joint_2 position-round-trip completed without exact readback of a quantized non-baseline position target"
    );
    // Publish the persistent start-pose baseline before waiting for return.
    // Signal/error cleanup owns the same target, so cancellation at any later
    // instruction still converges on this command before selected-first disable.
    backend
        .ensure_single_axis_commissioning_state(selected_index)
        .context("single-axis drive-state contract failed before position baseline return")?;
    backend.set_targets(baseline_targets).await?;

    let return_started = Instant::now();
    let mut return_stability = ContinuousStabilityGate::new(STABILITY_DWELL, RETURN_SETTLE_TIMEOUT);
    let (return_error, return_velocity) = loop {
        anyhow::ensure!(
            diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
            "joint_2 position-round-trip exceeded its {:.3} s active-time limit during return settle",
            torque_limits.max_active_sec
        );
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_2 position return settle"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed during position return settle")?;
        let feedback = backend.feedback();
        let observation = position_diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            baseline_targets[selected_index],
            initial_q[selected_index],
            torque_limits,
        )?;
        peaks.observe(observation);
        max_positive_delta = max_positive_delta.max(observation.telemetry.measured_delta);
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed before return baseline publish")?;
        backend.set_targets(baseline_targets).await?;
        let return_error = observation.telemetry.measured_delta.abs();
        let return_velocity = observation.telemetry.measured_velocity;
        let elapsed = return_started.elapsed();
        match return_stability.observe(
            elapsed,
            return_error <= POSITION_DIAGNOSTIC_RETURN_POSITION_RAD
                && return_velocity.abs() <= POSITION_DIAGNOSTIC_STABILITY_VELOCITY_RAD_S,
        ) {
            StabilityGateStatus::Stable => break (return_error, return_velocity),
            StabilityGateStatus::TimedOut => anyhow::bail!(
                "joint_2 position-round-trip did not return within {:.3} s: error {return_error:.6} rad (limit {:.6}), velocity {return_velocity:.6} rad/s (limit {:.6}); both must hold continuously for {:.3} s",
                RETURN_SETTLE_TIMEOUT.as_secs_f32(),
                POSITION_DIAGNOSTIC_RETURN_POSITION_RAD,
                POSITION_DIAGNOSTIC_STABILITY_VELOCITY_RAD_S,
                STABILITY_DWELL.as_secs_f32()
            ),
            StabilityGateStatus::Waiting => {}
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    };

    // Freeze the returned baseline and prove the drive consumed it. This SDO
    // is success-only; every signal/error path uses the persistent baseline
    // and selected-first shutdown without waiting for an SDO.
    confirm_position_target_readback_while_guarded(
        backend,
        profile,
        selected_index,
        initial_q,
        temperature_baseline,
        baseline_targets[selected_index],
        initial_q[selected_index],
        diagnostic_started,
        torque_limits,
        PHASE_DIAGNOSTIC_POSITION_ROUND_TRIP,
        "returned_baseline",
        true,
        false,
        peaks,
    )
    .await?;

    let required_peak = POSITION_DIAGNOSTIC_REQUIRED_PEAK_FRACTION * delta_rad;
    anyhow::ensure!(
        max_positive_delta >= required_peak,
        "joint_2 position-round-trip peak {max_positive_delta:.6} rad did not reach the required 50% excursion {required_peak:.6} rad"
    );
    tracing::info!(
        phase = PHASE_DIAGNOSTIC_POSITION_RETURN,
        joint = %joint.name,
        node_id = joint.node_id,
        measured_peak_delta_rad = max_positive_delta,
        return_error_rad = return_error,
        return_velocity_rad_s = return_velocity,
        "bounded joint_2 position round trip completed and settled at its persistent baseline"
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn run_diagnostic_level(
    backend: &RealBackend,
    profile: &HardwareProfile,
    _dynamics: &hex_arm_dynamics::ArmDynamics,
    selected_index: usize,
    initial_q: &[f32; DOF],
    temperature_baseline: DiagnosticTemperatureBaseline,
    targets: [MotorTarget; DOF],
    model_gravity_ff_nm: f32,
    additive_torque_nm: f32,
    level_index: usize,
    total_levels: usize,
    duration: Duration,
    require_exact_readback: bool,
    stop_on_breakaway: bool,
    phase: &'static str,
    diagnostic_started: Instant,
    torque_limits: DiagnosticTorqueLimits,
    peaks: &mut DiagnosticPeaks,
) -> Result<DiagnosticLevelOutcome> {
    let joint = &profile.joints[selected_index];
    let target = targets[selected_index];
    backend
        .ensure_single_axis_commissioning_state(selected_index)
        .context("prospective diagnostic target state gate failed")?;
    anyhow::ensure!(
        !backend.transport_failed(),
        "CAN transport failed before prospective diagnostic target check"
    );
    let prospective_feedback = backend.feedback();
    let prospective_observation = diagnostic_observation(
        profile,
        &prospective_feedback,
        selected_index,
        initial_q,
        temperature_baseline,
        target,
        additive_torque_nm,
        torque_limits,
    )?;
    peaks.observe(prospective_observation);
    if stop_on_breakaway && prospective_observation.breakaway_detected {
        log_diagnostic_telemetry(
            "breakaway_before_next_target",
            phase,
            joint,
            level_index,
            total_levels,
            model_gravity_ff_nm,
            additive_torque_nm,
            diagnostic_started.elapsed(),
            prospective_observation,
        );
        tracing::warn!(
            joint = %joint.name,
            additive_torque_nm,
            measured_delta_rad = prospective_observation.telemetry.measured_delta,
            measured_velocity_rad_s = prospective_observation.telemetry.measured_velocity,
            "encoder breakaway threshold was already reached; refusing to publish the next torque step"
        );
        return Ok(DiagnosticLevelOutcome::BreakawayDetected);
    }
    backend
        .set_targets(targets)
        .await
        .with_context(|| format!("publish {phase} target for {}", joint.name))?;
    let level_started = Instant::now();
    let mut start_logged = false;
    let mut readback_confirmed = false;
    loop {
        anyhow::ensure!(
            diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
            "joint_2 diagnostic exceeded the selected-tier active-time limit of {:.3} s",
            torque_limits.max_active_sec
        );
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed during joint_2 diagnostic"
        );
        backend
            .ensure_single_axis_commissioning_state(selected_index)
            .context("single-axis drive-state contract failed during diagnostics")?;
        let feedback = backend.feedback();
        let observation = diagnostic_observation(
            profile,
            &feedback,
            selected_index,
            initial_q,
            temperature_baseline,
            target,
            additive_torque_nm,
            torque_limits,
        )?;
        peaks.observe(observation);
        if !start_logged {
            log_diagnostic_telemetry(
                "level_start",
                phase,
                joint,
                level_index,
                total_levels,
                model_gravity_ff_nm,
                additive_torque_nm,
                diagnostic_started.elapsed(),
                observation,
            );
            start_logged = true;
        }
        if stop_on_breakaway && observation.breakaway_detected {
            log_diagnostic_telemetry(
                "breakaway_stop",
                phase,
                joint,
                level_index,
                total_levels,
                model_gravity_ff_nm,
                additive_torque_nm,
                diagnostic_started.elapsed(),
                observation,
            );
            tracing::warn!(
                joint = %joint.name,
                additive_torque_nm,
                measured_delta_rad = observation.telemetry.measured_delta,
                measured_velocity_rad_s = observation.telemetry.measured_velocity,
                "encoder breakaway threshold reached; refusing every higher torque step"
            );
            return Ok(DiagnosticLevelOutcome::BreakawayDetected);
        }

        if require_exact_readback
            && !readback_confirmed
            && level_started.elapsed() >= RPDO_READBACK_SETTLE
        {
            let mut readback_future =
                Box::pin(backend.confirm_commissioning_target_readback(selected_index, target));
            let readback = loop {
                tokio::select! {
                    result = &mut readback_future => {
                        break result.context("exact compressed-MIT target readback failed")?;
                    }
                    _ = tokio::time::sleep(LOOP_PERIOD) => {
                        anyhow::ensure!(
                            diagnostic_started.elapsed().as_secs_f32()
                                <= torque_limits.max_active_sec,
                            "joint_2 diagnostic exceeded its {:.3} s active-time limit during SDO readback",
                            torque_limits.max_active_sec
                        );
                        anyhow::ensure!(
                            !backend.transport_failed(),
                            "CAN transport failed during joint_2 diagnostic SDO readback"
                        );
                        backend
                            .ensure_single_axis_commissioning_state(selected_index)
                            .context("single-axis drive-state contract failed during SDO readback")?;
                        let monitor_feedback = backend.feedback();
                        let monitor = diagnostic_observation(
                            profile,
                            &monitor_feedback,
                            selected_index,
                            initial_q,
                            temperature_baseline,
                            target,
                            additive_torque_nm,
                            torque_limits,
                        )?;
                        peaks.observe(monitor);
                        if stop_on_breakaway && monitor.breakaway_detected {
                            log_diagnostic_telemetry(
                                "breakaway_during_sdo_readback",
                                phase,
                                joint,
                                level_index,
                                total_levels,
                                model_gravity_ff_nm,
                                additive_torque_nm,
                                diagnostic_started.elapsed(),
                                monitor,
                            );
                            tracing::warn!(
                                joint = %joint.name,
                                additive_torque_nm,
                                measured_delta_rad = monitor.telemetry.measured_delta,
                                measured_velocity_rad_s = monitor.telemetry.measured_velocity,
                                "encoder breakaway threshold reached during SDO readback; cancelling readback and refusing every higher torque step"
                            );
                            return Ok(DiagnosticLevelOutcome::BreakawayDetected);
                        }
                    }
                }
            };
            anyhow::ensure!(
                diagnostic_started.elapsed().as_secs_f32() <= torque_limits.max_active_sec,
                "joint_2 diagnostic exceeded its {:.3} s active-time limit during SDO readback",
                torque_limits.max_active_sec
            );
            let post_readback_feedback = backend.feedback();
            backend
                .ensure_single_axis_commissioning_state(selected_index)
                .context("single-axis drive-state contract failed after SDO readback")?;
            let post_readback = diagnostic_observation(
                profile,
                &post_readback_feedback,
                selected_index,
                initial_q,
                temperature_baseline,
                target,
                additive_torque_nm,
                torque_limits,
            )?;
            peaks.observe(post_readback);
            tracing::info!(
                phase,
                joint = %joint.name,
                node_id = joint.node_id,
                level_index,
                total_levels,
                additive_torque_nm,
                expected_lower = %format_args!("0x{:08X}", readback.expected_lower),
                expected_upper = %format_args!("0x{:08X}", readback.expected_upper),
                actual_lower = %format_args!("0x{:08X}", readback.actual_lower),
                actual_upper = %format_args!("0x{:08X}", readback.actual_upper),
                "exact SDO readback matched the held compressed-MIT target"
            );
            log_diagnostic_telemetry(
                "exact_sdo_readback",
                phase,
                joint,
                level_index,
                total_levels,
                model_gravity_ff_nm,
                additive_torque_nm,
                diagnostic_started.elapsed(),
                post_readback,
            );
            readback_confirmed = true;
        }

        if level_started.elapsed() >= duration {
            anyhow::ensure!(
                !require_exact_readback || readback_confirmed,
                "diagnostic level ended before exact 0x2004:02/03 readback was confirmed"
            );
            log_diagnostic_telemetry(
                "level_end",
                phase,
                joint,
                level_index,
                total_levels,
                model_gravity_ff_nm,
                additive_torque_nm,
                diagnostic_started.elapsed(),
                observation,
            );
            return Ok(DiagnosticLevelOutcome::Completed);
        }
        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

#[allow(clippy::too_many_arguments)]
fn log_diagnostic_telemetry(
    milestone: &'static str,
    phase: &'static str,
    joint: &crate::profile::JointProfile,
    level_index: usize,
    total_levels: usize,
    model_gravity_ff_nm: f32,
    additive_torque_nm: f32,
    elapsed: Duration,
    observation: DiagnosticObservation,
) {
    let sample = observation.telemetry;
    tracing::info!(
        milestone,
        phase,
        joint = %joint.name,
        node_id = joint.node_id,
        level_index,
        total_levels,
        elapsed_sec = elapsed.as_secs_f32(),
        commanded_q = sample.commanded_q,
        measured_q = sample.measured_q,
        measured_delta = sample.measured_delta,
        measured_velocity = sample.measured_velocity,
        measured_torque = sample.measured_torque,
        model_gravity_ff_nm,
        additive_torque_nm,
        total_feedforward_nm = sample.gravity_ff,
        kp = sample.kp,
        kd = sample.kd,
        estimated_pd_torque_nm = sample.estimated_pd_torque,
        estimated_total_torque_nm = sample.gravity_ff + sample.estimated_pd_torque,
        driver_temperature_c = observation.driver_temperature_c,
        motor_temperature_c = observation.motor_temperature_c,
        breakaway_detected = observation.breakaway_detected,
        "bounded single-axis diagnostic telemetry (q rad, velocity rad/s, torque Nm, gains per rad)"
    );
}

fn validate_diagnostic_enable_stability_guards(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_position_rad: f32,
    measured_position_rad: f32,
    guards: DiagnosticStabilityGuards,
) -> Result<()> {
    let joint = &profile.joints[selected_index];
    let state = feedback.joints[selected_index];
    validate_diagnostic_temperature(
        "driver",
        state.driver_temperature_c,
        Some(guards.temperature_baseline.driver_c),
    )?;
    validate_diagnostic_temperature(
        "motor",
        state.motor_temperature_c,
        Some(guards.temperature_baseline.motor_c),
    )?;
    let delta = measured_position_rad - initial_position_rad;
    if guards.position_round_trip {
        // No position motion is permitted during either the low-FF enable
        // dwell or the full-FF post-ramp dwell. Keep both pre-trajectory
        // phases inside the ramp envelope rather than admitting the wider
        // +5 mrad trajectory envelope here.
        let absolute_upper = joint.limits.position_lower_rad
            + POSITION_DIAGNOSTIC_GRAVITY_RAMP_ABSOLUTE_UPPER_OFFSET_RAD;
        anyhow::ensure!(
            measured_position_rad <= absolute_upper,
            "joint_2 position-round-trip absolute position {measured_position_rad:.6} rad exceeds its {absolute_upper:.6} rad guard during enable stability"
        );
        anyhow::ensure!(
            delta >= -POSITION_DIAGNOSTIC_MAX_OPPOSITE_POSITION_RAD,
            "joint_2 moved {delta:.6} rad opposite the permitted positive position survey during enable stability"
        );
        anyhow::ensure!(
            delta <= POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_POSITIVE_RAD,
            "joint_2 position-round-trip enable-stability excursion {delta:.6} rad exceeds its positive guard"
        );
    } else if guards.torque_limits.high_tier {
        let absolute_upper =
            joint.limits.position_lower_rad + 2.0 * HIGH_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD;
        anyhow::ensure!(
            measured_position_rad <= absolute_upper,
            "joint_2 high-tier absolute position {measured_position_rad:.6} rad exceeds lower limit plus 0.004 rad ({absolute_upper:.6} rad) during enable stability"
        );
        anyhow::ensure!(
            delta >= -MAX_DIAGNOSTIC_OPPOSITE_POSITION_RAD,
            "joint_2 moved {delta:.6} rad opposite the sole permitted positive high-tier direction during enable stability"
        );
    }
    if !guards.position_round_trip {
        anyhow::ensure!(
            delta.abs() <= MAX_DIAGNOSTIC_EXCURSION_RAD,
            "joint_2 diagnostic enable-stability excursion {delta:.6} rad exceeds hard limit {MAX_DIAGNOSTIC_EXCURSION_RAD:.6} rad"
        );
    }
    let velocity = motor_velocity_to_ros(state.velocity_rev_s, joint);
    anyhow::ensure!(
        velocity.abs() <= MAX_DIAGNOSTIC_VELOCITY_RAD_S,
        "joint_2 diagnostic enable-stability velocity {velocity:.6} rad/s exceeds hard limit {MAX_DIAGNOSTIC_VELOCITY_RAD_S:.6} rad/s"
    );
    if guards.torque_limits.high_tier && !guards.position_round_trip {
        anyhow::ensure!(
            velocity >= -DIAGNOSTIC_BREAKAWAY_VELOCITY_RAD_S,
            "joint_2 enable-stability velocity {velocity:.6} rad/s is opposite the sole permitted positive high-tier direction"
        );
    }
    if guards.censored_gravity_hold {
        anyhow::ensure!(
            delta < CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD,
            "joint_2 censored gravity-hold displacement {delta:.6} rad reached the {CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD:.6} rad identification exit layer during enable stability"
        );
        anyhow::ensure!(
            velocity.abs() < CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S,
            "joint_2 censored gravity-hold velocity {velocity:.6} rad/s reached the {CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S:.6} rad/s identification exit layer during enable stability"
        );
    }
    Ok(())
}

fn enable_stability_limits(diagnostic_guards: Option<DiagnosticStabilityGuards>) -> (f32, f32) {
    if diagnostic_guards.is_some_and(|guards| guards.censored_gravity_hold) {
        (
            CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD,
            CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S,
        )
    } else if diagnostic_guards.is_some_and(|guards| guards.position_round_trip) {
        (
            POSITION_DIAGNOSTIC_STABILITY_POSITION_RAD,
            POSITION_DIAGNOSTIC_STABILITY_VELOCITY_RAD_S,
        )
    } else {
        (ENABLE_STABILITY_POSITION_RAD, STABLE_VELOCITY_RAD_S)
    }
}

async fn wait_for_enabled_axis_stability(
    backend: &RealBackend,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    request: CommissioningRequest,
    initial_q: [f32; DOF],
    diagnostic_guards: Option<DiagnosticStabilityGuards>,
) -> Result<(FeedbackSnapshot, [f32; DOF], [MotorTarget; DOF])> {
    let joint = &profile.joints[request.selected_index];
    let started_at = Instant::now();
    let mut stability = ContinuousStabilityGate::new(STABILITY_DWELL, ENABLE_STABILITY_TIMEOUT);
    let mut peak_position_deviation = 0.0_f32;
    let mut peak_velocity = 0.0_f32;

    loop {
        if let Some(guards) = diagnostic_guards {
            anyhow::ensure!(
                guards.active_started.elapsed().as_secs_f32()
                    <= guards.torque_limits.max_active_sec,
                "joint_2 diagnostic exceeded the selected-tier active-time limit of {:.3} s during enable stability",
                guards.torque_limits.max_active_sec
            );
        }
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed while waiting for post-enable commissioning stability"
        );
        backend
            .ensure_single_axis_commissioning_state(request.selected_index)
            .context("single-axis drive-state contract failed during enable stability")?;
        let feedback = backend.feedback();
        let normalized_time =
            started_at.elapsed().as_secs_f32() / ENABLE_STABILITY_TIMEOUT.as_secs_f32();
        let log_abort = |abort_stage, error: &anyhow::Error| {
            log_safety_abort_telemetry(
                abort_stage,
                PHASE_ENABLE_STABILITY,
                normalized_time,
                error,
                profile,
                dynamics,
                &feedback,
                request.selected_index,
                initial_q[request.selected_index],
                initial_q[request.selected_index],
            );
        };
        let measured_q = result_with_safety_abort_hook(
            validate_feedback(
                profile,
                &feedback,
                Some((
                    request.selected_index,
                    initial_q[request.selected_index],
                    request.delta_rad,
                )),
            ),
            |error| log_abort("validate_feedback", error),
        )?;
        if let Some(guards) = diagnostic_guards {
            result_with_safety_abort_hook(
                validate_diagnostic_enable_stability_guards(
                    profile,
                    &feedback,
                    request.selected_index,
                    initial_q[request.selected_index],
                    measured_q[request.selected_index],
                    guards,
                ),
                |error| log_abort("diagnostic_stability_guard", error),
            )?;
        }
        if !diagnostic_guards.is_some_and(|guards| guards.position_round_trip) {
            result_with_safety_abort_hook(
                validate_symmetric_enable_drift(
                    initial_q[request.selected_index],
                    measured_q[request.selected_index],
                ),
                |error| log_abort("symmetric_drift_guard", error),
            )?;
        }
        let targets_result =
            match diagnostic_guards.and_then(|guards| guards.selected_gravity_scale) {
                Some(gravity_scale) => build_position_diagnostic_hold_targets(
                    profile,
                    dynamics,
                    &measured_q,
                    request.selected_index,
                    initial_q[request.selected_index],
                    gravity_scale,
                ),
                None => build_safe_hold_targets(
                    profile,
                    dynamics,
                    &measured_q,
                    request.selected_index,
                    initial_q[request.selected_index],
                ),
            };
        let targets = result_with_safety_abort_hook(targets_result, |error| {
            log_abort("build_safe_hold_targets", error)
        })?;
        if let Some(guards) = diagnostic_guards {
            let diagnostic_gate = if guards.position_round_trip {
                position_diagnostic_observation(
                    profile,
                    &feedback,
                    request.selected_index,
                    &initial_q,
                    guards.temperature_baseline,
                    targets[request.selected_index],
                    initial_q[request.selected_index],
                    guards.torque_limits,
                )
                .and_then(|observation| {
                    if guards.censored_gravity_hold {
                        validate_censored_gravity_hold_identification_layer(observation)
                    } else {
                        Ok(())
                    }
                })
            } else {
                diagnostic_observation(
                    profile,
                    &feedback,
                    request.selected_index,
                    &initial_q,
                    guards.temperature_baseline,
                    targets[request.selected_index],
                    0.0,
                    guards.torque_limits,
                )
                .map(|_| ())
            };
            result_with_safety_abort_hook(diagnostic_gate, |error| {
                log_abort("prospective_diagnostic_hold_guard", error)
            })?;
        }
        backend
            .ensure_single_axis_commissioning_state(request.selected_index)
            .context("single-axis drive-state contract failed before stability hold publish")?;
        backend.set_targets(targets).await?;

        let position_deviation =
            (measured_q[request.selected_index] - initial_q[request.selected_index]).abs();
        let velocity = motor_velocity_to_ros(
            feedback.joints[request.selected_index].velocity_rev_s,
            joint,
        );
        peak_position_deviation = peak_position_deviation.max(position_deviation);
        peak_velocity = peak_velocity.max(velocity.abs());

        let elapsed = started_at.elapsed();
        let (stability_position_limit, stability_velocity_limit) =
            enable_stability_limits(diagnostic_guards);
        match stability.observe(
            elapsed,
            position_deviation <= stability_position_limit
                && velocity.abs() <= stability_velocity_limit,
        ) {
            StabilityGateStatus::Stable => {
                tracing::info!(
                    joint = %joint.name,
                    elapsed_sec = elapsed.as_secs_f32(),
                    position_deviation_rad = position_deviation,
                    velocity_rad_s = velocity,
                    peak_position_deviation_rad = peak_position_deviation,
                    peak_velocity_rad_s = peak_velocity,
                    required_dwell_sec = STABILITY_DWELL.as_secs_f32(),
                    "commissioning axis remained stable after enable"
                );
                return Ok((feedback, measured_q, targets));
            }
            StabilityGateStatus::TimedOut => {
                log_commissioning_telemetry(
                    "enable_stability_timeout",
                    PHASE_ENABLE_STABILITY,
                    joint,
                    0.0,
                    commissioning_telemetry_sample(
                        joint,
                        &feedback,
                        request.selected_index,
                        initial_q[request.selected_index],
                        measured_q[request.selected_index],
                        initial_q[request.selected_index],
                        targets[request.selected_index],
                    ),
                );
                tracing::error!(
                    joint = %joint.name,
                    position_deviation_rad = position_deviation,
                    velocity_rad_s = velocity,
                    peak_position_deviation_rad = peak_position_deviation,
                    peak_velocity_rad_s = peak_velocity,
                    position_limit_rad = stability_position_limit,
                    velocity_limit_rad_s = stability_velocity_limit,
                    required_dwell_sec = STABILITY_DWELL.as_secs_f32(),
                    "commissioning axis failed to stabilize after enable"
                );
                anyhow::bail!(
                    "{} did not stabilize after enable within {:.3} s: current position deviation {:.6} rad (limit {:.6} rad), current velocity {:.6} rad/s (limit {:.6} rad/s), peak position deviation {:.6} rad, peak velocity {:.6} rad/s; both limits must hold continuously for {:.3} s",
                    joint.name,
                    ENABLE_STABILITY_TIMEOUT.as_secs_f32(),
                    position_deviation,
                    stability_position_limit,
                    velocity,
                    stability_velocity_limit,
                    peak_position_deviation,
                    peak_velocity,
                    STABILITY_DWELL.as_secs_f32()
                );
            }
            StabilityGateStatus::Waiting => {}
        }

        tokio::time::sleep(LOOP_PERIOD).await;
    }
}

fn commissioning_telemetry_sample(
    joint: &crate::profile::JointProfile,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_q: f32,
    measured_q: f32,
    commanded_q: f32,
    target: MotorTarget,
) -> CommissioningTelemetrySample {
    let state = feedback.joints[selected_index];
    let measured_velocity = motor_velocity_to_ros(state.velocity_rev_s, joint);
    let measured_torque = motor_torque_to_ros(state.torque_nm, joint);
    let gravity_ff = motor_torque_to_ros(target.torque_nm, joint);
    let kp = motor_kp_to_ros(target.kp_nm_rev, joint);
    let kd = motor_kd_to_ros(target.kd_nm_s_rev, joint);

    // Estimate the PD term in the same motor-side units consumed by MIT, then
    // convert it back to the ROS joint convention. This remains correct if a
    // future profile uses a non-unit direction or torque scale.
    let estimated_motor_pd_torque = target.kp_nm_rev * (target.position_rev - state.position_rev)
        + target.kd_nm_s_rev * (target.velocity_rev_s - state.velocity_rev_s);
    let estimated_pd_torque = motor_torque_to_ros(estimated_motor_pd_torque, joint);

    CommissioningTelemetrySample {
        commanded_q,
        measured_q,
        measured_delta: measured_q - initial_q,
        measured_velocity,
        measured_torque,
        gravity_ff,
        kp,
        kd,
        estimated_pd_torque,
    }
}

fn log_commissioning_telemetry(
    milestone: &'static str,
    phase: &'static str,
    joint: &crate::profile::JointProfile,
    normalized_time: f32,
    sample: CommissioningTelemetrySample,
) {
    tracing::info!(
        milestone,
        phase,
        joint = %joint.name,
        node_id = joint.node_id,
        normalized_time,
        commanded_q = sample.commanded_q,
        measured_q = sample.measured_q,
        measured_delta = sample.measured_delta,
        measured_velocity = sample.measured_velocity,
        measured_torque = sample.measured_torque,
        gravity_ff = sample.gravity_ff,
        kp = sample.kp,
        kd = sample.kd,
        estimated_pd_torque = sample.estimated_pd_torque,
        estimated_total_torque = sample.gravity_ff + sample.estimated_pd_torque,
        "single-axis commissioning telemetry milestone (q rad, velocity rad/s, torque Nm, gains per rad)"
    );
}

/// Preserve the exact safety error while giving the caller one opportunity to
/// emit phase-specific abort telemetry before it leaves the control phase.
fn result_with_safety_abort_hook<T, F>(result: Result<T>, on_abort: F) -> Result<T>
where
    F: FnOnce(&anyhow::Error),
{
    match result {
        Ok(value) => Ok(value),
        Err(error) => {
            on_abort(&error);
            Err(error)
        }
    }
}

fn feedback_positions_unchecked(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
) -> [f32; DOF] {
    array::from_fn(|index| {
        motor_position_to_ros(feedback.joints[index].position_rev, &profile.joints[index])
    })
}

/// Build a telemetry sample without trusting feedback validity. The ordinary
/// safe-target builder is attempted first so an abort record matches the exact
/// command that would have been sent. If that builder rejects the state, a
/// selected-axis-only target is reconstructed for diagnostics and is never
/// published to hardware.
fn safety_abort_telemetry_sample(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_q: f32,
    commanded_q: f32,
) -> SafetyAbortTelemetrySample {
    let measured_q = feedback_positions_unchecked(profile, feedback);
    let joint = &profile.joints[selected_index];
    let gravity_torque =
        dynamics.gravity_torque_with(&measured_q, profile.gravity_vector_base_m_s2);
    let gravity_ff = gravity_torque
        .get(selected_index)
        .copied()
        .unwrap_or(f32::NAN)
        * joint.gravity_compensation_scale;
    let diagnostic_target = ros_target_to_motor(
        RosTarget {
            position_rad: commanded_q,
            velocity_rad_s: 0.0,
            torque_nm: gravity_ff,
            kp_nm_rad: joint.default_kp,
            kd_nm_s_rad: joint.default_kd,
        },
        joint,
    );
    let (target, safe_target_built, target_build_error) = match build_safe_hold_targets(
        profile,
        dynamics,
        &measured_q,
        selected_index,
        commanded_q,
    ) {
        Ok(targets) => (targets[selected_index], true, None),
        Err(error) => (diagnostic_target, false, Some(format!("{error:#}"))),
    };
    let state = feedback.joints[selected_index];

    SafetyAbortTelemetrySample {
        telemetry: commissioning_telemetry_sample(
            joint,
            feedback,
            selected_index,
            initial_q,
            measured_q[selected_index],
            commanded_q,
            target,
        ),
        raw_motor_position_rev: state.position_rev,
        raw_motor_velocity_rev_s: state.velocity_rev_s,
        raw_motor_torque_nm: state.torque_nm,
        safe_target_built,
        target_build_error,
    }
}

#[allow(clippy::too_many_arguments)]
fn log_safety_abort_telemetry(
    abort_stage: &'static str,
    phase: &'static str,
    normalized_time: f32,
    abort_reason: &anyhow::Error,
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    feedback: &FeedbackSnapshot,
    selected_index: usize,
    initial_q: f32,
    commanded_q: f32,
) {
    let joint = &profile.joints[selected_index];
    let abort = safety_abort_telemetry_sample(
        profile,
        dynamics,
        feedback,
        selected_index,
        initial_q,
        commanded_q,
    );
    let sample = abort.telemetry;
    tracing::error!(
        milestone = "safety_abort",
        phase,
        abort_stage,
        abort_reason = %abort_reason,
        joint = %joint.name,
        node_id = joint.node_id,
        normalized_time,
        commanded_q = sample.commanded_q,
        raw_measured_q = sample.measured_q,
        measured_delta = sample.measured_delta,
        raw_measured_velocity = sample.measured_velocity,
        raw_measured_torque = sample.measured_torque,
        raw_motor_position_rev = abort.raw_motor_position_rev,
        raw_motor_velocity_rev_s = abort.raw_motor_velocity_rev_s,
        raw_motor_torque_nm = abort.raw_motor_torque_nm,
        gravity_ff = sample.gravity_ff,
        kp = sample.kp,
        kd = sample.kd,
        estimated_pd_torque = sample.estimated_pd_torque,
        estimated_total_torque = sample.gravity_ff + sample.estimated_pd_torque,
        safe_target_built = abort.safe_target_built,
        target_build_error = abort.target_build_error.as_deref().unwrap_or(""),
        "single-axis commissioning safety abort telemetry (q rad, velocity rad/s, torque Nm, gains per rad)"
    );
}

fn validate_symmetric_enable_drift(initial_q: f32, measured_q: f32) -> Result<()> {
    let drift = measured_q - initial_q;
    anyhow::ensure!(drift.is_finite(), "post-enable drift is non-finite");
    anyhow::ensure!(
        drift.abs() <= ENABLE_STABILITY_POSITION_RAD,
        "post-enable drift {drift:.6} rad exceeds symmetric commissioning guard {ENABLE_STABILITY_POSITION_RAD:.6} rad"
    );
    Ok(())
}

async fn wait_for_safe_feedback(backend: &RealBackend) -> Result<FeedbackSnapshot> {
    let deadline = Instant::now() + Duration::from_secs_f32(FEEDBACK_WAIT_SEC);
    loop {
        anyhow::ensure!(
            !backend.transport_failed(),
            "CAN transport failed while waiting for commissioning feedback"
        );
        let feedback = backend.feedback();
        if let Some((index, code)) = feedback
            .joints
            .iter()
            .enumerate()
            .find_map(|(index, joint)| joint.fault_code.map(|code| (index, code)))
        {
            anyhow::bail!(
                "joint_{} reported drive fault 0x{code:04X} before commissioning",
                index + 1
            );
        }
        if feedback.all_online_and_fresh() {
            return Ok(feedback);
        }
        anyhow::ensure!(
            Instant::now() < deadline,
            "six-axis feedback did not become online and fresh before commissioning timeout"
        );
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

fn validate_feedback(
    profile: &HardwareProfile,
    feedback: &FeedbackSnapshot,
    selected_tracking: Option<(usize, f32, f32)>,
) -> Result<[f32; DOF]> {
    anyhow::ensure!(
        feedback.all_online_and_fresh(),
        "commissioning feedback became offline, stale, or faulted"
    );
    let q = feedback_positions_unchecked(profile, feedback);
    for (index, (state, joint)) in feedback.joints.iter().zip(&profile.joints).enumerate() {
        let position = q[index];
        let velocity = motor_velocity_to_ros(state.velocity_rev_s, joint);
        let torque = motor_torque_to_ros(state.torque_nm, joint);
        anyhow::ensure!(
            position.is_finite() && velocity.is_finite() && torque.is_finite(),
            "{} feedback contains a non-finite physical value",
            joint.name
        );
        anyhow::ensure!(
            position >= joint.limits.measured_position_lower_rad() - LIMIT_EPSILON
                && position <= joint.limits.measured_position_upper_rad() + LIMIT_EPSILON,
            "{} measured position {:.6} rad exceeds read-only feedback envelope [{:.6}, {:.6}] rad",
            joint.name,
            position,
            joint.limits.measured_position_lower_rad(),
            joint.limits.measured_position_upper_rad()
        );
        anyhow::ensure!(
            velocity.abs() <= joint.limits.velocity_rad_s + LIMIT_EPSILON,
            "{} measured velocity {:.6} rad/s exceeds software limit {:.6} rad/s",
            joint.name,
            velocity,
            joint.limits.velocity_rad_s
        );
        anyhow::ensure!(
            torque.abs() <= joint.limits.torque_nm + LIMIT_EPSILON,
            "{} measured torque {:.6} Nm exceeds software limit {:.6} Nm",
            joint.name,
            torque,
            joint.limits.torque_nm
        );
        anyhow::ensure!(
            state.fault_code.is_none(),
            "{} reported drive fault {:?}",
            joint.name,
            state.fault_code
        );
    }
    if let Some((selected_index, commanded_position, delta_rad)) = selected_tracking {
        let maximum_error = delta_rad.abs() + TRACKING_ERROR_MARGIN_RAD;
        let error = (q[selected_index] - commanded_position).abs();
        anyhow::ensure!(
            error <= maximum_error,
            "{} tracking error {:.6} rad exceeds commissioning guard {:.6} rad",
            profile.joints[selected_index].name,
            error,
            maximum_error
        );
    }
    Ok(q)
}

fn build_safe_hold_targets(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    measured_q: &[f32; DOF],
    selected_index: usize,
    selected_position: f32,
) -> Result<[MotorTarget; DOF]> {
    build_safe_hold_targets_with_selected_gravity_scale(
        profile,
        dynamics,
        measured_q,
        selected_index,
        selected_position,
        None,
    )
}

fn build_joint1_first_position_trajectory_targets(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    measured_q: &[f32; DOF],
    commanded_q: f32,
    normalized_time: f32,
) -> Result<([MotorTarget; DOF], f32, f32)> {
    anyhow::ensure!(
        normalized_time.is_finite() && (0.0..=1.0).contains(&normalized_time),
        "joint_1 compensated trajectory time {normalized_time:?} is outside [0,1]"
    );
    let mut targets = build_safe_hold_targets(
        profile,
        dynamics,
        measured_q,
        J1_FIRST_POSITION_INDEX,
        commanded_q,
    )?;
    let joint = &profile.joints[J1_FIRST_POSITION_INDEX];
    let model_gravity_ff_nm =
        motor_torque_to_ros(targets[J1_FIRST_POSITION_INDEX].torque_nm, joint);
    anyhow::ensure!(
        model_gravity_ff_nm.abs() <= J1_FIRST_POSITION_MAX_GRAVITY_NM,
        "joint_1 model feed-forward {model_gravity_ff_nm:.6} Nm exceeds {J1_FIRST_POSITION_MAX_GRAVITY_NM:.6} Nm"
    );
    let friction_compensation_nm = joint1_first_position_friction_compensation(normalized_time);
    let total_feedforward_nm = model_gravity_ff_nm + friction_compensation_nm;
    anyhow::ensure!(
        total_feedforward_nm.abs() <= J1_FIRST_POSITION_MAX_COMPENSATED_FEEDFORWARD_NM,
        "joint_1 compensated feed-forward {total_feedforward_nm:.6} Nm exceeds {J1_FIRST_POSITION_MAX_COMPENSATED_FEEDFORWARD_NM:.6} Nm"
    );
    targets[J1_FIRST_POSITION_INDEX] = ros_target_to_motor(
        RosTarget {
            position_rad: commanded_q,
            velocity_rad_s: 0.0,
            torque_nm: total_feedforward_nm,
            kp_nm_rad: J1_FIRST_POSITION_TRAJECTORY_KP,
            kd_nm_s_rad: J1_FIRST_POSITION_TRAJECTORY_KD,
        },
        joint,
    );
    Ok((targets, model_gravity_ff_nm, friction_compensation_nm))
}

fn build_joint5_first_position_trajectory_targets(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    measured_q: &[f32; DOF],
    commanded_q: f32,
) -> Result<[MotorTarget; DOF]> {
    let mut targets = build_safe_hold_targets(
        profile,
        dynamics,
        measured_q,
        J5_FIRST_POSITION_INDEX,
        commanded_q,
    )?;
    let joint = &profile.joints[J5_FIRST_POSITION_INDEX];
    let model_feedforward_nm =
        motor_torque_to_ros(targets[J5_FIRST_POSITION_INDEX].torque_nm, joint);
    anyhow::ensure!(
        model_feedforward_nm.abs() <= J5_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        "joint_5 model feed-forward {model_feedforward_nm:.6} Nm exceeds {J5_FIRST_POSITION_MAX_FEEDFORWARD_NM:.6} Nm"
    );
    targets[J5_FIRST_POSITION_INDEX] = ros_target_to_motor(
        RosTarget {
            position_rad: commanded_q,
            velocity_rad_s: 0.0,
            torque_nm: model_feedforward_nm,
            kp_nm_rad: J5_FIRST_POSITION_TRAJECTORY_KP,
            kd_nm_s_rad: J5_FIRST_POSITION_TRAJECTORY_KD,
        },
        joint,
    );
    Ok(targets)
}

fn build_joint4_first_position_trajectory_targets(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    measured_q: &[f32; DOF],
    commanded_q: f32,
) -> Result<[MotorTarget; DOF]> {
    let mut targets = build_safe_hold_targets(
        profile,
        dynamics,
        measured_q,
        J4_FIRST_POSITION_INDEX,
        commanded_q,
    )?;
    let joint = &profile.joints[J4_FIRST_POSITION_INDEX];
    let model_feedforward_nm =
        motor_torque_to_ros(targets[J4_FIRST_POSITION_INDEX].torque_nm, joint);
    anyhow::ensure!(
        model_feedforward_nm.abs() <= J4_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        "joint_4 model feed-forward {model_feedforward_nm:.6} Nm exceeds {J4_FIRST_POSITION_MAX_FEEDFORWARD_NM:.6} Nm"
    );
    targets[J4_FIRST_POSITION_INDEX] = ros_target_to_motor(
        RosTarget {
            position_rad: commanded_q,
            velocity_rad_s: 0.0,
            torque_nm: model_feedforward_nm,
            kp_nm_rad: J4_FIRST_POSITION_TRAJECTORY_KP,
            kd_nm_s_rad: J4_FIRST_POSITION_TRAJECTORY_KD,
        },
        joint,
    );
    Ok(targets)
}

fn build_joint4_assisted_position_trajectory_targets(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    measured_q: &[f32; DOF],
    commanded_q: f32,
    assistance_nm: f32,
) -> Result<[MotorTarget; DOF]> {
    anyhow::ensure!(
        assistance_nm.is_finite()
            && (J4_ASSISTED_POSITION_PEAK_TORQUE_NM..=0.0).contains(&assistance_nm),
        "joint_4 trajectory assistance {assistance_nm:.6} Nm is outside [{:.6}, 0] Nm",
        J4_ASSISTED_POSITION_PEAK_TORQUE_NM
    );
    let mut targets =
        build_joint4_first_position_trajectory_targets(profile, dynamics, measured_q, commanded_q)?;
    let joint = &profile.joints[J4_FIRST_POSITION_INDEX];
    let model_feedforward_nm =
        motor_torque_to_ros(targets[J4_FIRST_POSITION_INDEX].torque_nm, joint);
    let total_feedforward_nm = model_feedforward_nm + assistance_nm;
    anyhow::ensure!(
        total_feedforward_nm.abs() <= J4_ASSISTED_POSITION_MAX_FEEDFORWARD_NM,
        "joint_4 assisted feed-forward {total_feedforward_nm:.6} Nm exceeds {:.6} Nm",
        J4_ASSISTED_POSITION_MAX_FEEDFORWARD_NM
    );
    targets[J4_FIRST_POSITION_INDEX] = ros_target_to_motor(
        RosTarget {
            position_rad: commanded_q,
            velocity_rad_s: 0.0,
            torque_nm: total_feedforward_nm,
            kp_nm_rad: J4_FIRST_POSITION_TRAJECTORY_KP,
            kd_nm_s_rad: J4_FIRST_POSITION_TRAJECTORY_KD,
        },
        joint,
    );
    Ok(targets)
}

fn build_joint6_first_position_trajectory_targets(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    measured_q: &[f32; DOF],
    commanded_q: f32,
) -> Result<[MotorTarget; DOF]> {
    let mut targets = build_safe_hold_targets(
        profile,
        dynamics,
        measured_q,
        J6_FIRST_POSITION_INDEX,
        commanded_q,
    )?;
    let joint = &profile.joints[J6_FIRST_POSITION_INDEX];
    let model_feedforward_nm =
        motor_torque_to_ros(targets[J6_FIRST_POSITION_INDEX].torque_nm, joint);
    anyhow::ensure!(
        model_feedforward_nm.abs() <= J6_FIRST_POSITION_MAX_FEEDFORWARD_NM,
        "joint_6 model feed-forward {model_feedforward_nm:.6} Nm exceeds {J6_FIRST_POSITION_MAX_FEEDFORWARD_NM:.6} Nm"
    );
    targets[J6_FIRST_POSITION_INDEX] = ros_target_to_motor(
        RosTarget {
            position_rad: commanded_q,
            velocity_rad_s: 0.0,
            torque_nm: model_feedforward_nm,
            kp_nm_rad: J6_FIRST_POSITION_TRAJECTORY_KP,
            kd_nm_s_rad: J6_FIRST_POSITION_TRAJECTORY_KD,
        },
        joint,
    );
    Ok(targets)
}

fn build_position_diagnostic_hold_targets(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    measured_q: &[f32; DOF],
    selected_index: usize,
    selected_position: f32,
    selected_gravity_scale: f32,
) -> Result<[MotorTarget; DOF]> {
    anyhow::ensure!(
        selected_index == DIAGNOSTIC_JOINT_INDEX,
        "position diagnostic gravity override is restricted to joint_2"
    );
    anyhow::ensure!(
        selected_gravity_scale.is_finite()
            && (POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE
                ..=POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE)
                .contains(&selected_gravity_scale),
        "position diagnostic gravity scale must remain in [{POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE:.2}, {POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE:.2}]"
    );
    build_safe_hold_targets_with_selected_gravity_scale(
        profile,
        dynamics,
        measured_q,
        selected_index,
        selected_position,
        Some(selected_gravity_scale),
    )
}

fn build_safe_hold_targets_with_selected_gravity_scale(
    profile: &HardwareProfile,
    dynamics: &hex_arm_dynamics::ArmDynamics,
    measured_q: &[f32; DOF],
    selected_index: usize,
    selected_position: f32,
    selected_gravity_scale: Option<f32>,
) -> Result<[MotorTarget; DOF]> {
    validate_commissioning_gains(profile)?;
    let gravity_torque = dynamics.gravity_torque_with(measured_q, profile.gravity_vector_base_m_s2);
    anyhow::ensure!(
        gravity_torque.len() == DOF,
        "dynamics model did not return six gravity torques"
    );
    let ros_targets: [RosTarget; DOF] = array::from_fn(|index| {
        let joint = &profile.joints[index];
        let gravity_scale = if index == selected_index {
            selected_gravity_scale.unwrap_or(joint.gravity_compensation_scale)
        } else {
            joint.gravity_compensation_scale
        };
        RosTarget {
            position_rad: if index == selected_index {
                selected_position
            } else {
                // Non-selected axes remain strictly non-torque. If a passive
                // hard-stop reading is inside its explicit measurement margin
                // but just outside the command range, publish the nearest
                // legal dormant target instead of widening command authority.
                measured_q[index].clamp(
                    joint.limits.position_lower_rad,
                    joint.limits.position_upper_rad,
                )
            },
            velocity_rad_s: 0.0,
            torque_nm: gravity_torque[index] * gravity_scale,
            kp_nm_rad: joint.default_kp,
            kd_nm_s_rad: joint.default_kd,
        }
    });
    for (target, joint) in ros_targets.iter().zip(&profile.joints) {
        anyhow::ensure!(
            target.position_rad.is_finite()
                && target.torque_nm.is_finite()
                && target.kp_nm_rad.is_finite()
                && target.kd_nm_s_rad.is_finite(),
            "{} commissioning target contains a non-finite value",
            joint.name
        );
        anyhow::ensure!(
            (joint.limits.position_lower_rad..=joint.limits.position_upper_rad)
                .contains(&target.position_rad),
            "{} commissioning target {:.6} rad exceeds position limit",
            joint.name,
            target.position_rad
        );
        anyhow::ensure!(
            target.torque_nm.abs() <= joint.limits.torque_nm,
            "{} gravity feedforward {:.6} Nm exceeds profile torque limit {:.6} Nm",
            joint.name,
            target.torque_nm,
            joint.limits.torque_nm
        );
    }
    Ok(array::from_fn(|index| {
        ros_target_to_motor(ros_targets[index], &profile.joints[index])
    }))
}

fn validate_commissioning_gains(profile: &HardwareProfile) -> Result<()> {
    for joint in &profile.joints {
        anyhow::ensure!(
            joint.default_kp <= COMMISSION_KP_HARD_MAX_NM_RAD,
            "{} commissioning Kp {:.6} exceeds hard limit {:.6} Nm/rad",
            joint.name,
            joint.default_kp,
            COMMISSION_KP_HARD_MAX_NM_RAD
        );
        anyhow::ensure!(
            joint.default_kd <= COMMISSION_KD_HARD_MAX_NM_S_RAD,
            "{} commissioning Kd {:.6} exceeds hard limit {:.6} Nm*s/rad",
            joint.name,
            joint.default_kd,
            COMMISSION_KD_HARD_MAX_NM_S_RAD
        );
    }
    Ok(())
}

fn round_trip_phase(normalized_time: f32) -> f32 {
    let t = normalized_time.clamp(0.0, 1.0);
    0.5 * (1.0 - (TAU * t).cos())
}

fn joint1_first_position_friction_compensation(normalized_time: f32) -> f32 {
    let t = normalized_time.clamp(0.0, 1.0);
    let signed_velocity_envelope = (TAU * t).sin();
    if signed_velocity_envelope >= 0.0 {
        J1_FIRST_POSITION_POSITIVE_COMPENSATION_NM * signed_velocity_envelope
    } else {
        J1_FIRST_POSITION_NEGATIVE_COMPENSATION_NM * signed_velocity_envelope
    }
}

fn return_tolerance_rad(requested_delta_rad: f32) -> f32 {
    (requested_delta_rad.abs() * 0.25).clamp(0.001, 0.005)
}

#[derive(Debug, Clone, Copy)]
struct MeasuredExcursion {
    start_position_rad: f32,
    requested_delta_rad: f32,
    most_positive_delta_rad: f32,
    most_negative_delta_rad: f32,
}

impl MeasuredExcursion {
    fn new(start_position_rad: f32, requested_delta_rad: f32) -> Self {
        Self {
            start_position_rad,
            requested_delta_rad,
            most_positive_delta_rad: 0.0,
            most_negative_delta_rad: 0.0,
        }
    }

    fn observe(&mut self, measured_position_rad: f32) -> Result<()> {
        let delta = measured_position_rad - self.start_position_rad;
        anyhow::ensure!(
            delta.is_finite(),
            "measured commissioning excursion is non-finite"
        );
        let maximum_excursion = self.requested_delta_rad.abs() + EXCURSION_MARGIN_RAD;
        anyhow::ensure!(
            delta.abs() <= maximum_excursion,
            "measured excursion {delta:.6} rad exceeds requested magnitude plus guard {maximum_excursion:.6} rad"
        );
        if self.requested_delta_rad.is_sign_positive() {
            anyhow::ensure!(
                delta >= -OPPOSITE_DIRECTION_MARGIN_RAD,
                "measured excursion {delta:.6} rad is opposite the requested positive direction"
            );
        } else {
            anyhow::ensure!(
                delta <= OPPOSITE_DIRECTION_MARGIN_RAD,
                "measured excursion {delta:.6} rad is opposite the requested negative direction"
            );
        }
        self.most_positive_delta_rad = self.most_positive_delta_rad.max(delta);
        self.most_negative_delta_rad = self.most_negative_delta_rad.min(delta);
        Ok(())
    }

    fn validate_completed(&self) -> Result<f32> {
        let signed_peak = if self.requested_delta_rad.is_sign_positive() {
            self.most_positive_delta_rad
        } else {
            self.most_negative_delta_rad
        };
        let minimum_excursion =
            MINIMUM_REQUESTED_EXCURSION_FRACTION * self.requested_delta_rad.abs();
        anyhow::ensure!(
            signed_peak.signum() == self.requested_delta_rad.signum()
                && signed_peak.abs() >= minimum_excursion,
            "measured signed peak excursion {signed_peak:.6} rad did not reach {minimum_excursion:.6} rad in the requested direction; max positive excursion {:.6} rad, max negative excursion {:.6} rad",
            self.most_positive_delta_rad,
            self.most_negative_delta_rad
        );
        Ok(signed_peak)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::{
        BusProfile, BusTransport, ControllerProfile, HardwareProfile, IdentityFingerprint,
        JointLimits, JointProfile,
    };

    fn profile_with_velocity_limit(velocity_rad_s: f32) -> HardwareProfile {
        HardwareProfile {
            schema_version: 2,
            validated: true,
            calibrated: false,
            robot_prefix: "test/arm".into(),
            urdf_path: "/unused/in/pure/request/tests.urdf".into(),
            gravity_vector_base_m_s2: [0.0, 0.0, -9.81],
            tip_payload: None,
            bus: BusProfile {
                transport: BusTransport::GsUsb,
                interface: String::new(),
                channel: 0,
                adapter_vid: 0x1209,
                adapter_pid: 0x2323,
                heartbeat_node_id: 16,
                hardware_timestamp: false,
                direct_joint_mapping: true,
                auxiliary_node_ids: vec![15],
                expected_link: None,
            },
            controller: ControllerProfile {
                loop_hz: 1000,
                state_publish_hz: 50,
                discovery_timeout_ms: 2000,
                feedback_timeout_ms: 100,
                command_watchdog_ms: 100,
            },
            joints: (0..DOF)
                .map(|index| JointProfile {
                    name: format!("joint_{}", index + 1),
                    node_id: (index + 1) as u8,
                    identity: IdentityFingerprint {
                        vendor_id: 1,
                        product_code: 2,
                        revision: 3,
                        serial_number: (index + 1) as u32,
                        model: "test".into(),
                    },
                    direction: 1,
                    zero_offset_rad: 0.0,
                    torque_scale: 1.0,
                    gravity_compensation_scale: 1.0,
                    torque_permille: 100,
                    kp_kd_torque_permille: 100,
                    limits: JointLimits {
                        position_lower_rad: -1.0,
                        position_upper_rad: 1.0,
                        measured_position_margin_rad: 0.0,
                        velocity_rad_s,
                        acceleration_rad_s2: 0.1,
                        torque_nm: 5.0,
                    },
                    default_kp: 2.0,
                    default_kd: 0.3,
                })
                .collect(),
        }
    }

    fn zero_gravity_dynamics() -> hex_arm_dynamics::ArmDynamics {
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        hex_arm_dynamics::ArmDynamics::from_parts(
            vec![([0.0; 3], identity, [0.0, 0.0, 1.0]); DOF],
            vec![(0.0, [0.0; 3]); DOF],
            [0.0, 0.0, -9.81],
        )
    }

    fn position_gravity_dynamics() -> hex_arm_dynamics::ArmDynamics {
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut joints = vec![([0.0; 3], identity, [0.0, 0.0, 1.0]); DOF];
        joints[DIAGNOSTIC_JOINT_INDEX].2 = [0.0, 1.0, 0.0];
        let mut links = vec![(0.0, [0.0; 3]); DOF];
        links[DIAGNOSTIC_JOINT_INDEX] = (0.3, [0.5, 0.0, 0.0]);
        hex_arm_dynamics::ArmDynamics::from_parts(joints, links, [0.0, 0.0, -9.81])
    }

    fn healthy_diagnostic_feedback() -> FeedbackSnapshot {
        let mut feedback = FeedbackSnapshot::default();
        for joint in &mut feedback.joints {
            joint.online = true;
            joint.fresh = true;
            joint.driver_temperature_c = 35.0;
            joint.motor_temperature_c = 36.0;
            joint.temperature_c = joint.motor_temperature_c;
        }
        feedback
    }

    fn low_diagnostic_limits() -> DiagnosticTorqueLimits {
        SingleAxisDiagnosticRequest {
            selected_index: DIAGNOSTIC_JOINT_INDEX,
            mode: SingleAxisDiagnosticMode::TauFfStaircase {
                peak_additive_torque_nm: LOW_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM,
                step_torque_nm: MIN_DIAGNOSTIC_TORQUE_STEP_NM,
                dwell_sec: 0.20,
            },
            high_torque_authorized: false,
            censored_gravity_hold_authorized: false,
        }
        .torque_limits()
    }

    fn high_diagnostic_limits() -> DiagnosticTorqueLimits {
        SingleAxisDiagnosticRequest {
            selected_index: DIAGNOSTIC_JOINT_INDEX,
            mode: SingleAxisDiagnosticMode::TauFfStaircase {
                peak_additive_torque_nm: 1.5,
                step_torque_nm: 0.25,
                dwell_sec: 0.20,
            },
            high_torque_authorized: true,
            censored_gravity_hold_authorized: false,
        }
        .torque_limits()
    }

    fn position_diagnostic_profile() -> HardwareProfile {
        let mut profile = profile_with_velocity_limit(0.1);
        let joint = &mut profile.joints[DIAGNOSTIC_JOINT_INDEX];
        joint.gravity_compensation_scale = POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE;
        joint.default_kp = POSITION_DIAGNOSTIC_EXPECTED_KP;
        joint.default_kd = POSITION_DIAGNOSTIC_EXPECTED_KD;
        joint.torque_permille = POSITION_DIAGNOSTIC_EXPECTED_TORQUE_PERMILLE;
        joint.kp_kd_torque_permille = POSITION_DIAGNOSTIC_EXPECTED_KP_KD_TORQUE_PERMILLE;
        profile
    }

    fn joint1_first_position_profile() -> HardwareProfile {
        let mut profile =
            profile_with_velocity_limit(J1_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S);
        let joint = &mut profile.joints[J1_FIRST_POSITION_INDEX];
        joint.direction = -1;
        joint.zero_offset_rad = J1_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD;
        joint.torque_scale = J1_FIRST_POSITION_EXPECTED_TORQUE_SCALE;
        joint.gravity_compensation_scale = J1_FIRST_POSITION_EXPECTED_GRAVITY_SCALE;
        joint.default_kp = J1_FIRST_POSITION_EXPECTED_KP;
        joint.default_kd = J1_FIRST_POSITION_EXPECTED_KD;
        joint.torque_permille = J1_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE;
        joint.kp_kd_torque_permille = J1_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE;
        joint.limits = JointLimits {
            position_lower_rad: J1_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD,
            position_upper_rad: J1_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD,
            measured_position_margin_rad: 0.0,
            velocity_rad_s: J1_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S,
            acceleration_rad_s2: J1_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2,
            torque_nm: J1_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM,
        };
        profile
    }

    fn joint1_first_position_request(authorized: bool) -> Joint1FirstPositionDiagnosticRequest {
        Joint1FirstPositionDiagnosticRequest { authorized }
    }

    fn joint5_first_position_profile() -> HardwareProfile {
        let mut profile =
            profile_with_velocity_limit(J5_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S);
        let joint = &mut profile.joints[J5_FIRST_POSITION_INDEX];
        joint.direction = 1;
        joint.zero_offset_rad = J5_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD;
        joint.torque_scale = J5_FIRST_POSITION_EXPECTED_TORQUE_SCALE;
        joint.gravity_compensation_scale = J5_FIRST_POSITION_EXPECTED_GRAVITY_SCALE;
        joint.default_kp = J5_FIRST_POSITION_EXPECTED_KP;
        joint.default_kd = J5_FIRST_POSITION_EXPECTED_KD;
        joint.torque_permille = J5_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE;
        joint.kp_kd_torque_permille = J5_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE;
        joint.limits = JointLimits {
            position_lower_rad: J5_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD,
            position_upper_rad: J5_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD,
            measured_position_margin_rad: 0.0,
            velocity_rad_s: J5_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S,
            acceleration_rad_s2: J5_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2,
            torque_nm: J5_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM,
        };
        profile
    }

    fn joint5_first_position_request(authorized: bool) -> Joint5FirstPositionDiagnosticRequest {
        Joint5FirstPositionDiagnosticRequest { authorized }
    }

    fn joint4_first_position_profile() -> HardwareProfile {
        let mut profile =
            profile_with_velocity_limit(J4_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S);
        let joint = &mut profile.joints[J4_FIRST_POSITION_INDEX];
        joint.direction = 1;
        joint.zero_offset_rad = J4_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD;
        joint.torque_scale = J4_FIRST_POSITION_EXPECTED_TORQUE_SCALE;
        joint.gravity_compensation_scale = J4_FIRST_POSITION_EXPECTED_GRAVITY_SCALE;
        joint.default_kp = J4_FIRST_POSITION_EXPECTED_KP;
        joint.default_kd = J4_FIRST_POSITION_EXPECTED_KD;
        joint.torque_permille = J4_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE;
        joint.kp_kd_torque_permille = J4_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE;
        joint.limits = JointLimits {
            position_lower_rad: J4_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD,
            position_upper_rad: J4_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD,
            measured_position_margin_rad: 0.0,
            velocity_rad_s: J4_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S,
            acceleration_rad_s2: J4_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2,
            torque_nm: J4_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM,
        };
        profile
    }

    fn joint3_gravity_unload_profile() -> HardwareProfile {
        let mut profile = profile_with_velocity_limit(0.1);
        let joint = &mut profile.joints[J3_GRAVITY_UNLOAD_INDEX];
        joint.direction = 1;
        joint.zero_offset_rad = J3_GRAVITY_UNLOAD_EXPECTED_ZERO_OFFSET_RAD;
        joint.torque_scale = J3_GRAVITY_UNLOAD_EXPECTED_TORQUE_SCALE;
        joint.gravity_compensation_scale = J3_GRAVITY_UNLOAD_EXPECTED_GRAVITY_SCALE;
        joint.default_kp = J3_GRAVITY_UNLOAD_EXPECTED_KP;
        joint.default_kd = J3_GRAVITY_UNLOAD_EXPECTED_KD;
        joint.torque_permille = J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_TORQUE_PERMILLE;
        joint.kp_kd_torque_permille = J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE;
        joint.limits = JointLimits {
            position_lower_rad: J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_LOWER_RAD,
            position_upper_rad: J3_GRAVITY_UNLOAD_EXPECTED_PROFILE_UPPER_RAD,
            measured_position_margin_rad: 0.0,
            velocity_rad_s: 0.1,
            acceleration_rad_s2: 0.1,
            torque_nm: 7.5,
        };
        profile
    }

    fn joint3_assisted_position_profile() -> HardwareProfile {
        let mut profile = joint3_gravity_unload_profile();
        let joint = &mut profile.joints[J3_GRAVITY_UNLOAD_INDEX];
        joint.default_kp = J3_ASSISTED_POSITION_EXPECTED_KP;
        joint.default_kd = J3_ASSISTED_POSITION_EXPECTED_KD;
        joint.torque_permille = J3_ASSISTED_POSITION_EXPECTED_TORQUE_PERMILLE;
        joint.kp_kd_torque_permille = J3_ASSISTED_POSITION_EXPECTED_KP_KD_TORQUE_PERMILLE;
        joint.limits = JointLimits {
            position_lower_rad: J3_ASSISTED_POSITION_EXPECTED_LOWER_RAD,
            position_upper_rad: J3_ASSISTED_POSITION_EXPECTED_UPPER_RAD,
            measured_position_margin_rad: 0.0,
            velocity_rad_s: J3_ASSISTED_POSITION_EXPECTED_VELOCITY_RAD_S,
            acceleration_rad_s2: J3_ASSISTED_POSITION_EXPECTED_ACCELERATION_RAD_S2,
            torque_nm: J3_ASSISTED_POSITION_EXPECTED_TORQUE_NM,
        };
        profile
    }

    fn joint4_first_position_request(authorized: bool) -> Joint4FirstPositionDiagnosticRequest {
        Joint4FirstPositionDiagnosticRequest { authorized }
    }

    fn joint4_censored_torque_request(authorized: bool) -> Joint4CensoredTorqueDiagnosticRequest {
        Joint4CensoredTorqueDiagnosticRequest { authorized }
    }

    fn joint3_gravity_unload_request(authorized: bool) -> Joint3GravityUnloadDiagnosticRequest {
        Joint3GravityUnloadDiagnosticRequest { authorized }
    }

    fn joint3_assisted_position_request(
        authorized: bool,
    ) -> Joint3AssistedPositionDiagnosticRequest {
        Joint3AssistedPositionDiagnosticRequest { authorized }
    }

    fn joint4_assisted_position_request(
        authorized: bool,
    ) -> Joint4AssistedPositionDiagnosticRequest {
        Joint4AssistedPositionDiagnosticRequest { authorized }
    }

    fn joint6_first_position_profile() -> HardwareProfile {
        let mut profile =
            profile_with_velocity_limit(J6_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S);
        let joint = &mut profile.joints[J6_FIRST_POSITION_INDEX];
        joint.direction = 1;
        joint.zero_offset_rad = J6_FIRST_POSITION_EXPECTED_ZERO_OFFSET_RAD;
        joint.torque_scale = J6_FIRST_POSITION_EXPECTED_TORQUE_SCALE;
        joint.gravity_compensation_scale = J6_FIRST_POSITION_EXPECTED_GRAVITY_SCALE;
        joint.default_kp = J6_FIRST_POSITION_EXPECTED_KP;
        joint.default_kd = J6_FIRST_POSITION_EXPECTED_KD;
        joint.torque_permille = J6_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_PERMILLE;
        joint.kp_kd_torque_permille = J6_FIRST_POSITION_EXPECTED_PROFILE_KP_KD_TORQUE_PERMILLE;
        joint.limits = crate::profile::JointLimits {
            position_lower_rad: J6_FIRST_POSITION_EXPECTED_PROFILE_LOWER_RAD,
            position_upper_rad: J6_FIRST_POSITION_EXPECTED_PROFILE_UPPER_RAD,
            measured_position_margin_rad: 0.0,
            velocity_rad_s: J6_FIRST_POSITION_EXPECTED_PROFILE_VELOCITY_RAD_S,
            acceleration_rad_s2: J6_FIRST_POSITION_EXPECTED_PROFILE_ACCELERATION_RAD_S2,
            torque_nm: J6_FIRST_POSITION_EXPECTED_PROFILE_TORQUE_NM,
        };
        profile
    }

    fn joint6_first_position_request(authorized: bool) -> Joint6FirstPositionDiagnosticRequest {
        Joint6FirstPositionDiagnosticRequest { authorized }
    }

    fn joint1_censored_torque_request(authorized: bool) -> Joint1CensoredTorqueDiagnosticRequest {
        Joint1CensoredTorqueDiagnosticRequest { authorized }
    }

    fn joint1_negative_censored_torque_request(
        authorized: bool,
    ) -> Joint1NegativeCensoredTorqueDiagnosticRequest {
        Joint1NegativeCensoredTorqueDiagnosticRequest { authorized }
    }

    fn position_diagnostic_request(authorized: bool) -> SingleAxisDiagnosticRequest {
        SingleAxisDiagnosticRequest {
            selected_index: DIAGNOSTIC_JOINT_INDEX,
            mode: SingleAxisDiagnosticMode::PositionRoundTrip {
                delta_rad: POSITION_DIAGNOSTIC_DELTA_RAD,
                duration_sec: POSITION_DIAGNOSTIC_DURATION_SEC,
            },
            high_torque_authorized: authorized,
            censored_gravity_hold_authorized: false,
        }
    }

    fn censored_gravity_hold_profile() -> HardwareProfile {
        let mut profile = position_diagnostic_profile();
        profile.joints[DIAGNOSTIC_JOINT_INDEX].gravity_compensation_scale =
            CENSORED_GRAVITY_HOLD_PROFILE_SCALE;
        profile
    }

    fn censored_gravity_hold_request(
        high_torque_authorized: bool,
        censored_gravity_hold_authorized: bool,
    ) -> SingleAxisDiagnosticRequest {
        SingleAxisDiagnosticRequest {
            selected_index: DIAGNOSTIC_JOINT_INDEX,
            mode: SingleAxisDiagnosticMode::GravityHoldCensored,
            high_torque_authorized,
            censored_gravity_hold_authorized,
        }
    }

    fn censored_gravity_hold_observation(
        measured_delta: f32,
        measured_velocity: f32,
    ) -> DiagnosticObservation {
        DiagnosticObservation {
            telemetry: CommissioningTelemetrySample {
                commanded_q: 0.0,
                measured_q: measured_delta,
                measured_delta,
                measured_velocity,
                measured_torque: 1.0,
                gravity_ff: 1.1,
                kp: POSITION_DIAGNOSTIC_EXPECTED_KP,
                kd: POSITION_DIAGNOSTIC_EXPECTED_KD,
                estimated_pd_torque: -0.1,
            },
            driver_temperature_c: 35.0,
            motor_temperature_c: 36.0,
            breakaway_detected: false,
        }
    }

    #[test]
    fn joint1_first_position_request_locks_every_reviewed_profile_field() {
        let profile = joint1_first_position_profile();
        let request = joint1_first_position_request(true);
        request.validate(&profile).unwrap();
        assert_eq!(request.selected_index(), J1_FIRST_POSITION_INDEX);
        assert!(joint1_first_position_request(false)
            .validate(&profile)
            .is_err());

        for mutate in 0..15 {
            let mut invalid = joint1_first_position_profile();
            let joint = &mut invalid.joints[J1_FIRST_POSITION_INDEX];
            match mutate {
                0 => joint.name = "not_joint_1".into(),
                1 => joint.node_id = 7,
                2 => joint.direction = 1,
                3 => joint.zero_offset_rad += 1.0e-6,
                4 => joint.default_kp -= 1.0,
                5 => joint.default_kd -= 0.1,
                6 => joint.torque_scale -= 0.01,
                7 => joint.gravity_compensation_scale -= 0.01,
                8 => joint.torque_permille -= 1,
                9 => joint.kp_kd_torque_permille -= 1,
                10 => joint.limits.position_lower_rad -= 0.01,
                11 => joint.limits.position_upper_rad += 0.01,
                12 => joint.limits.velocity_rad_s += 0.01,
                13 => joint.limits.acceleration_rad_s2 += 0.01,
                14 => joint.limits.measured_position_margin_rad = 0.001,
                _ => unreachable!(),
            }
            assert!(
                request.validate(&invalid).is_err(),
                "profile mutation {mutate} unexpectedly passed"
            );
        }

        let mut invalid_torque = joint1_first_position_profile();
        invalid_torque.joints[J1_FIRST_POSITION_INDEX]
            .limits
            .torque_nm -= 0.1;
        assert!(request.validate(&invalid_torque).is_err());
        let mut indirect_mapping = joint1_first_position_profile();
        indirect_mapping.bus.direct_joint_mapping = false;
        assert!(request.validate(&indirect_mapping).is_err());
    }

    #[test]
    fn joint5_first_position_request_locks_profile_direction_and_negative_path() {
        let profile = joint5_first_position_profile();
        let request = joint5_first_position_request(true);
        request.validate(&profile).unwrap();
        assert_eq!(request.selected_index(), J5_FIRST_POSITION_INDEX);
        assert!(joint5_first_position_request(false)
            .validate(&profile)
            .is_err());

        for mutate in 0..16 {
            let mut invalid = joint5_first_position_profile();
            let joint = &mut invalid.joints[J5_FIRST_POSITION_INDEX];
            match mutate {
                0 => joint.name = "not_joint_5".into(),
                1 => joint.node_id = 1,
                2 => joint.direction = -1,
                3 => joint.zero_offset_rad += 1.0e-6,
                4 => joint.default_kp += 1.0,
                5 => joint.default_kd += 0.1,
                6 => joint.torque_scale -= 0.01,
                7 => joint.gravity_compensation_scale -= 0.01,
                8 => joint.torque_permille -= 1,
                9 => joint.kp_kd_torque_permille -= 1,
                10 => joint.limits.position_lower_rad -= 0.01,
                11 => joint.limits.position_upper_rad += 0.01,
                12 => joint.limits.velocity_rad_s += 0.01,
                13 => joint.limits.acceleration_rad_s2 += 0.01,
                14 => joint.limits.torque_nm += 0.1,
                15 => joint.limits.measured_position_margin_rad = 0.001,
                _ => unreachable!(),
            }
            assert!(
                request.validate(&invalid).is_err(),
                "joint_5 profile mutation {mutate} unexpectedly passed"
            );
        }
        let mut indirect = joint5_first_position_profile();
        indirect.bus.direct_joint_mapping = false;
        assert!(request.validate(&indirect).is_err());

        let joint = &profile.joints[J5_FIRST_POSITION_INDEX];
        validate_joint5_first_position_dynamic_window(joint, 0.02).unwrap();
        assert!(validate_joint5_first_position_dynamic_window(joint, -0.249).is_err());
        assert!(J5_FIRST_POSITION_DELTA_RAD.is_sign_negative());
        assert!(
            (J5_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV
                - J5_FIRST_POSITION_DELTA_RAD.abs() / TAU * 0.5)
                .abs()
                <= 1.0e-9
        );
    }

    #[test]
    fn joint5_followup_uses_fixed_temporary_gain_without_mutating_profile() {
        let profile = joint5_first_position_profile();
        let measured_q = [0.0; DOF];
        let targets = build_joint5_first_position_trajectory_targets(
            &profile,
            &zero_gravity_dynamics(),
            &measured_q,
            J5_FIRST_POSITION_DELTA_RAD,
        )
        .unwrap();
        let joint = &profile.joints[J5_FIRST_POSITION_INDEX];
        let target = targets[J5_FIRST_POSITION_INDEX];
        assert!((motor_kp_to_ros(target.kp_nm_rev, joint) - 50.0).abs() <= 1.0e-5);
        assert!((motor_kd_to_ros(target.kd_nm_s_rev, joint) - 2.0).abs() <= 1.0e-5);
        assert!((motor_position_to_ros(target.position_rev, joint) + 0.005).abs() <= 1.0e-6);
        assert_eq!(joint.default_kp.to_bits(), 30.0_f32.to_bits());
        assert_eq!(joint.default_kd.to_bits(), 1.0_f32.to_bits());
    }

    #[test]
    fn joint4_first_position_request_locks_profile_and_negative_path() {
        let profile = joint4_first_position_profile();
        let request = joint4_first_position_request(true);
        request.validate(&profile).unwrap();
        assert_eq!(request.selected_index(), J4_FIRST_POSITION_INDEX);
        assert!(joint4_first_position_request(false)
            .validate(&profile)
            .is_err());

        for mutate in 0..16 {
            let mut invalid = joint4_first_position_profile();
            let joint = &mut invalid.joints[J4_FIRST_POSITION_INDEX];
            match mutate {
                0 => joint.name = "not_joint_4".into(),
                1 => joint.node_id = 1,
                2 => joint.direction = -1,
                3 => joint.zero_offset_rad += 1.0e-6,
                4 => joint.default_kp += 1.0,
                5 => joint.default_kd += 0.1,
                6 => joint.torque_scale -= 0.01,
                7 => joint.gravity_compensation_scale += 0.01,
                8 => joint.torque_permille -= 1,
                9 => joint.kp_kd_torque_permille -= 1,
                10 => joint.limits.position_lower_rad -= 0.01,
                11 => joint.limits.position_upper_rad += 0.01,
                12 => joint.limits.velocity_rad_s += 0.01,
                13 => joint.limits.acceleration_rad_s2 += 0.01,
                14 => joint.limits.torque_nm += 0.1,
                15 => joint.limits.measured_position_margin_rad = 0.001,
                _ => unreachable!(),
            }
            assert!(
                request.validate(&invalid).is_err(),
                "joint_4 profile mutation {mutate} unexpectedly passed"
            );
        }
        let mut indirect = joint4_first_position_profile();
        indirect.bus.direct_joint_mapping = false;
        assert!(request.validate(&indirect).is_err());

        let joint = &profile.joints[J4_FIRST_POSITION_INDEX];
        validate_joint4_first_position_dynamic_window(joint, 0.01).unwrap();
        assert!(validate_joint4_first_position_dynamic_window(joint, -0.249).is_err());
        assert!(J4_FIRST_POSITION_DELTA_RAD.is_sign_negative());
        assert!(
            (J4_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV
                - J4_FIRST_POSITION_DELTA_RAD.abs() / TAU * 0.5)
                .abs()
                <= 1.0e-9
        );
    }

    #[test]
    fn joint4_followup_uses_fixed_temporary_gain_without_mutating_profile() {
        let profile = joint4_first_position_profile();
        let measured_q = [0.0; DOF];
        let targets = build_joint4_first_position_trajectory_targets(
            &profile,
            &zero_gravity_dynamics(),
            &measured_q,
            J4_FIRST_POSITION_DELTA_RAD,
        )
        .unwrap();
        let joint = &profile.joints[J4_FIRST_POSITION_INDEX];
        let target = targets[J4_FIRST_POSITION_INDEX];
        assert!((motor_kp_to_ros(target.kp_nm_rev, joint) - 80.0).abs() <= 1.0e-5);
        assert!((motor_kd_to_ros(target.kd_nm_s_rev, joint) - 4.0).abs() <= 1.0e-5);
        assert!((motor_position_to_ros(target.position_rev, joint) + 0.005).abs() <= 1.0e-6);
        assert_eq!(joint.default_kp.to_bits(), 40.0_f32.to_bits());
        assert_eq!(joint.default_kd.to_bits(), 2.0_f32.to_bits());
    }

    #[test]
    fn joint4_assistance_is_independently_authorized_phase_bounded_and_returns_to_zero() {
        let profile = joint4_first_position_profile();
        joint4_assisted_position_request(true)
            .validate(&profile)
            .unwrap();
        assert!(joint4_assisted_position_request(false)
            .validate(&profile)
            .is_err());

        let measured_q = [0.0; DOF];
        let joint = &profile.joints[J4_FIRST_POSITION_INDEX];
        for (assistance_nm, expected_torque_nm) in [
            (0.0, 0.0),
            (J4_ASSISTED_POSITION_PEAK_TORQUE_NM * 0.5, -0.15),
            (J4_ASSISTED_POSITION_PEAK_TORQUE_NM, -0.30),
        ] {
            let targets = build_joint4_assisted_position_trajectory_targets(
                &profile,
                &zero_gravity_dynamics(),
                &measured_q,
                J4_FIRST_POSITION_DELTA_RAD,
                assistance_nm,
            )
            .unwrap();
            let target = targets[J4_FIRST_POSITION_INDEX];
            assert!(
                (motor_torque_to_ros(target.torque_nm, joint) - expected_torque_nm).abs() <= 1.0e-6
            );
            assert!((motor_kp_to_ros(target.kp_nm_rev, joint) - 80.0).abs() <= 1.0e-5);
            assert!((motor_kd_to_ros(target.kd_nm_s_rev, joint) - 4.0).abs() <= 1.0e-5);
        }
        assert!(build_joint4_assisted_position_trajectory_targets(
            &profile,
            &zero_gravity_dynamics(),
            &measured_q,
            J4_FIRST_POSITION_DELTA_RAD,
            -0.301,
        )
        .is_err());
        assert!(build_joint4_assisted_position_trajectory_targets(
            &profile,
            &zero_gravity_dynamics(),
            &measured_q,
            J4_FIRST_POSITION_DELTA_RAD,
            0.001,
        )
        .is_err());
    }

    #[test]
    fn joint4_assisted_runner_uses_round_trip_phase_and_both_exact_readbacks() {
        let source = include_str!("commissioning.rs");
        let body = source
            .split_once("async fn run_joint4_first_position_round_trip(")
            .unwrap()
            .1
            .split_once("fn validate_joint4_censored_torque_active_time(")
            .unwrap()
            .0;
        let phase = body.find("let path_phase = round_trip_phase").unwrap();
        let assistance = body
            .find("let assistance_nm = path_phase * peak_assistance_nm")
            .unwrap();
        let publish = body.find("backend.set_targets(targets)").unwrap();
        let changed = body.find("first_quantized_changed_position").unwrap();
        let baseline = body.find("returned_persistent_baseline").unwrap();
        assert!(phase < assistance && assistance < publish);
        assert!(publish < changed && changed < baseline);
        assert!(body.contains("backend.set_targets(baseline_targets)"));
    }

    #[test]
    fn joint3_gravity_unload_is_independently_authorized_and_profile_locked() {
        let profile = joint3_gravity_unload_profile();
        joint3_gravity_unload_request(true)
            .validate(&profile)
            .unwrap();
        assert_eq!(joint3_gravity_unload_request(true).selected_index(), 2);
        assert!(joint3_gravity_unload_request(false)
            .validate(&profile)
            .is_err());

        for mutation in 0..8 {
            let mut invalid = joint3_gravity_unload_profile();
            let joint = &mut invalid.joints[J3_GRAVITY_UNLOAD_INDEX];
            match mutation {
                0 => joint.zero_offset_rad += 0.001,
                1 => joint.torque_scale = 1.0,
                2 => joint.gravity_compensation_scale = 0.01,
                3 => joint.default_kp += 1.0,
                4 => joint.default_kd += 0.1,
                5 => joint.torque_permille -= 1,
                6 => joint.kp_kd_torque_permille -= 1,
                7 => joint.limits.torque_nm -= 0.1,
                _ => unreachable!(),
            }
            assert!(joint3_gravity_unload_request(true)
                .validate(&invalid)
                .is_err());
        }
    }

    #[test]
    fn joint3_assisted_position_is_independently_authorized_and_profile_locked() {
        let profile = joint3_assisted_position_profile();
        joint3_assisted_position_request(true)
            .validate(&profile)
            .unwrap();
        assert_eq!(joint3_assisted_position_request(true).selected_index(), 2);
        assert!(joint3_assisted_position_request(false)
            .validate(&profile)
            .is_err());

        for mutation in 0..12 {
            let mut invalid = joint3_assisted_position_profile();
            let joint = &mut invalid.joints[J3_GRAVITY_UNLOAD_INDEX];
            match mutation {
                0 => invalid.bus.direct_joint_mapping = false,
                1 => joint.zero_offset_rad += 0.001,
                2 => joint.torque_scale = 1.0,
                3 => joint.gravity_compensation_scale = 0.01,
                4 => joint.default_kp -= 1.0,
                5 => joint.default_kd -= 0.1,
                6 => joint.torque_permille += 1,
                7 => joint.kp_kd_torque_permille += 1,
                8 => joint.limits.position_lower_rad -= 0.001,
                9 => joint.limits.velocity_rad_s += 0.001,
                10 => joint.limits.acceleration_rad_s2 += 0.001,
                11 => joint.limits.torque_nm += 0.1,
                _ => unreachable!(),
            }
            assert!(joint3_assisted_position_request(true)
                .validate(&invalid)
                .is_err());
        }
    }

    #[test]
    fn joint3_assistance_is_phase_bounded_fixed_gain_and_zero_at_both_ends() {
        let profile = joint3_assisted_position_profile();
        let joint = &profile.joints[J3_GRAVITY_UNLOAD_INDEX];
        let baseline = array::from_fn(|index| {
            ros_target_to_motor(
                RosTarget {
                    position_rad: if index == J3_GRAVITY_UNLOAD_INDEX {
                        3.139
                    } else {
                        0.0
                    },
                    velocity_rad_s: 0.0,
                    torque_nm: 0.0,
                    kp_nm_rad: profile.joints[index].default_kp,
                    kd_nm_s_rad: profile.joints[index].default_kd,
                },
                &profile.joints[index],
            )
        });

        for (normalized_time, expected_phase) in [(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)] {
            let phase = round_trip_phase(normalized_time);
            assert!((phase - expected_phase).abs() <= 1.0e-6);
            let commanded_q = 3.139 + phase * J3_ASSISTED_POSITION_DELTA_RAD;
            let targets = apply_joint3_phase_assistance(
                &profile,
                baseline,
                commanded_q,
                phase * J3_ASSISTED_POSITION_PEAK_TORQUE_NM,
            )
            .unwrap();
            let target = targets[J3_GRAVITY_UNLOAD_INDEX];
            assert!(
                (motor_position_to_ros(target.position_rev, joint) - commanded_q).abs() <= 1.0e-6
            );
            assert!(
                (motor_torque_to_ros(target.torque_nm, joint)
                    - phase * J3_ASSISTED_POSITION_PEAK_TORQUE_NM)
                    .abs()
                    <= 1.0e-6
            );
            assert!(
                (motor_kp_to_ros(target.kp_nm_rev, joint) - J3_ASSISTED_POSITION_EXPECTED_KP).abs()
                    <= 1.0e-5
            );
            assert!(
                (motor_kd_to_ros(target.kd_nm_s_rev, joint) - J3_ASSISTED_POSITION_EXPECTED_KD)
                    .abs()
                    <= 1.0e-5
            );
            for index in [0, 1, 3, 4, 5] {
                assert_eq!(targets[index], baseline[index]);
            }
        }
        assert!(apply_joint3_phase_assistance(&profile, baseline, 3.139, -0.251).is_err());
        assert!(apply_joint3_phase_assistance(&profile, baseline, 3.139, 0.001).is_err());
    }

    #[test]
    fn joint3_assisted_runner_has_fixed_round_trip_and_no_caller_selected_parameters() {
        let source = include_str!("commissioning.rs");
        let body = source
            .split_once("pub async fn run_joint3_assisted_position_diagnostic(")
            .unwrap()
            .1
            .split_once("async fn run_single_axis_commissioning_inner(")
            .unwrap()
            .0;
        assert!(body.contains("J3_ASSISTED_POSITION_DELTA_RAD"));
        assert!(body.contains("J3_ASSISTED_POSITION_DURATION_SEC"));
        assert!(body.contains("Some(J3_ASSISTED_POSITION_PEAK_TORQUE_NM)"));
        let inner = source
            .split_once("async fn run_single_axis_commissioning_inner(")
            .unwrap()
            .1
            .split_once("fn apply_joint3_phase_assistance(")
            .unwrap()
            .0;
        let phase = inner.find("let phase = round_trip_phase").unwrap();
        let assistance = inner.find("apply_joint3_phase_assistance").unwrap();
        let publish = inner.find("backend.set_targets(targets)").unwrap();
        assert!(phase < assistance && assistance < publish);
        assert!(inner.contains("register_single_axis_diagnostic_baseline"));
        assert!(inner.contains("enable_joint3_gravity_unload_diagnostic_axis"));
        assert!(inner.contains("validate_joint3_assisted_active_time"));
        assert!(inner.contains("ensure_single_axis_commissioning_state"));
        assert!(inner.contains("validate_six_axis_diagnostic_temperatures"));
    }

    #[test]
    fn joint3_assisted_temperature_gate_checks_both_sensors_on_every_axis() {
        let profile = joint3_assisted_position_profile();
        let mut feedback = healthy_diagnostic_feedback();
        let baseline = joint1_first_position_temperature_baseline(&profile, &feedback).unwrap();
        validate_six_axis_diagnostic_temperatures(
            &profile,
            &feedback,
            baseline,
            "joint_3 assisted-position",
        )
        .unwrap();

        feedback.joints[0].driver_temperature_c = 35.0 + MAX_DIAGNOSTIC_TEMPERATURE_RISE_C + 0.01;
        let error = format!(
            "{:#}",
            validate_six_axis_diagnostic_temperatures(
                &profile,
                &feedback,
                baseline,
                "joint_3 assisted-position",
            )
            .unwrap_err()
        );
        assert!(error.contains("joint_1 driver temperature rose"));
        feedback.joints[0].driver_temperature_c = 35.0;

        feedback.joints[5].motor_temperature_c = f32::NAN;
        let error = format!(
            "{:#}",
            validate_six_axis_diagnostic_temperatures(
                &profile,
                &feedback,
                baseline,
                "joint_3 assisted-position",
            )
            .unwrap_err()
        );
        assert!(error.contains("joint_6 motor temperature"));
    }

    #[test]
    fn joint3_gravity_unload_levels_and_censors_are_fixed() {
        assert_eq!(joint3_gravity_unload_feedforward(1).unwrap(), -0.05);
        assert_eq!(
            joint3_gravity_unload_feedforward(J3_GRAVITY_UNLOAD_LEVELS).unwrap(),
            -0.75
        );
        assert!(joint3_gravity_unload_feedforward(0).is_err());
        assert!(joint3_gravity_unload_feedforward(J3_GRAVITY_UNLOAD_LEVELS + 1).is_err());

        let mut observation = DiagnosticObservation {
            telemetry: CommissioningTelemetrySample {
                commanded_q: 0.0,
                measured_q: 0.0,
                measured_delta: 0.0,
                measured_velocity: 0.0,
                measured_torque: 0.0,
                gravity_ff: 0.0,
                kp: J3_GRAVITY_UNLOAD_EXPECTED_KP,
                kd: J3_GRAVITY_UNLOAD_EXPECTED_KD,
                estimated_pd_torque: 0.0,
            },
            driver_temperature_c: 30.0,
            motor_temperature_c: 30.0,
            breakaway_detected: false,
        };
        observation.telemetry.measured_delta = J3_GRAVITY_UNLOAD_TRIGGER_RAD + 1.0e-7;
        observation.telemetry.measured_velocity = 0.0;
        assert!(!joint3_gravity_unload_should_censor(observation));
        observation.telemetry.measured_delta = J3_GRAVITY_UNLOAD_TRIGGER_RAD;
        assert!(joint3_gravity_unload_should_censor(observation));
        observation.telemetry.measured_delta = 0.0;
        observation.telemetry.measured_velocity = J3_GRAVITY_UNLOAD_TRIGGER_VELOCITY_RAD_S;
        assert!(joint3_gravity_unload_should_censor(observation));
    }

    #[test]
    fn joint3_gravity_unload_changes_only_feedforward_at_fixed_position() {
        let profile = joint3_gravity_unload_profile();
        let baseline = array::from_fn(|index| {
            ros_target_to_motor(
                RosTarget {
                    position_rad: if index == J3_GRAVITY_UNLOAD_INDEX {
                        3.139
                    } else {
                        0.0
                    },
                    velocity_rad_s: 0.0,
                    torque_nm: 0.0,
                    kp_nm_rad: profile.joints[index].default_kp,
                    kd_nm_s_rad: profile.joints[index].default_kd,
                },
                &profile.joints[index],
            )
        });
        let targets =
            build_joint3_gravity_unload_targets(&profile, baseline, 3.139, -0.10).unwrap();
        let joint = &profile.joints[J3_GRAVITY_UNLOAD_INDEX];
        assert_eq!(
            compressed_target_position_code(&targets[J3_GRAVITY_UNLOAD_INDEX], joint),
            compressed_target_position_code(&baseline[J3_GRAVITY_UNLOAD_INDEX], joint)
        );
        assert!((motor_torque_to_ros(targets[2].torque_nm, joint) + 0.10).abs() < 1.0e-6);
        for index in [0, 1, 3, 4, 5] {
            assert_eq!(targets[index], baseline[index]);
        }
    }

    #[test]
    fn joint3_gravity_unload_source_has_no_position_trajectory_branch() {
        let source = include_str!("commissioning.rs");
        let body = source
            .split_once("pub async fn run_joint3_gravity_unload_diagnostic(")
            .unwrap()
            .1
            .split_once("struct Joint4VerifiedTorqueTarget")
            .unwrap()
            .0;
        assert!(body.contains("run_joint3_gravity_unload_ramp"));
        assert!(!body.contains("round_trip"));
        assert!(!body.contains("delta_rad: -0.005"));
        let ramp = source
            .split_once("async fn run_joint3_gravity_unload_ramp(")
            .unwrap()
            .1
            .split_once("async fn run_joint3_gravity_unload_frozen_hold(")
            .unwrap()
            .0;
        assert!(ramp.contains("inward_motion_censor"));
        assert!(
            ramp.contains("restore zero-feed-forward joint_3 baseline after first-level censor")
        );
        assert!(!ramp.contains("moved inward before any nonzero wire-distinct level completed"));
    }

    #[test]
    fn joint4_censored_torque_is_independently_authorized_negative_and_fixed_position() {
        let profile = joint4_first_position_profile();
        joint4_censored_torque_request(true)
            .validate(&profile)
            .unwrap();
        assert!(joint4_censored_torque_request(false)
            .validate(&profile)
            .is_err());

        assert_eq!(joint4_censored_torque_additive(1).unwrap(), -0.025);
        assert_eq!(
            joint4_censored_torque_additive(J4_CENSORED_TORQUE_LEVELS).unwrap(),
            -0.45
        );
        assert!(joint4_censored_torque_additive(0).is_err());
        assert!(joint4_censored_torque_additive(J4_CENSORED_TORQUE_LEVELS + 1).is_err());
        assert!(!joint4_censored_torque_should_censor(-0.000_299));
        assert!(joint4_censored_torque_should_censor(-0.000_300));

        let baseline = build_safe_hold_targets(
            &profile,
            &zero_gravity_dynamics(),
            &[0.0; DOF],
            J4_FIRST_POSITION_INDEX,
            0.0,
        )
        .unwrap();
        let target =
            build_joint4_censored_torque_targets(&profile, baseline, 0.0, 0.0, -0.225).unwrap();
        let joint = &profile.joints[J4_FIRST_POSITION_INDEX];
        assert_eq!(
            compressed_target_position_code(&baseline[J4_FIRST_POSITION_INDEX], joint),
            compressed_target_position_code(&target[J4_FIRST_POSITION_INDEX], joint)
        );
        assert_ne!(
            compressed_target_words(&baseline[J4_FIRST_POSITION_INDEX], joint),
            compressed_target_words(&target[J4_FIRST_POSITION_INDEX], joint)
        );
        assert!(
            (motor_torque_to_ros(target[J4_FIRST_POSITION_INDEX].torque_nm, joint) + 0.225).abs()
                <= 1.0e-6
        );
        assert!(
            build_joint4_censored_torque_targets(&profile, baseline, 0.0, 0.0, 0.025,).is_err()
        );
    }

    #[test]
    fn joint4_censored_torque_source_censors_before_promotion_and_has_no_trajectory() {
        let source = include_str!("commissioning.rs");
        let body = source
            .split_once("pub async fn run_joint4_censored_torque_diagnostic(")
            .unwrap()
            .1
            .split_once("pub async fn run_joint6_first_position_diagnostic(")
            .unwrap()
            .0;
        assert!(!body.contains("round_trip_phase"));
        let ramp = source
            .split_once("async fn run_joint4_censored_torque_ramp(")
            .unwrap()
            .1
            .split_once("async fn run_joint4_censored_torque_frozen_hold(")
            .unwrap()
            .0;
        let censor = ramp.find("joint4_censored_torque_should_censor").unwrap();
        let promote = ramp
            .find("promote completed feedback-proven joint_4 torque level")
            .unwrap();
        assert!(censor < promote);
        assert!(ramp.contains("frozen_words != target_words && frozen_words != baseline_words"));
        assert!(ramp.contains("oldest_tpdo1_at"));
    }

    #[test]
    fn joint6_first_position_request_locks_profile_and_negative_path() {
        let profile = joint6_first_position_profile();
        let request = joint6_first_position_request(true);
        request.validate(&profile).unwrap();
        assert_eq!(request.selected_index(), J6_FIRST_POSITION_INDEX);
        assert!(joint6_first_position_request(false)
            .validate(&profile)
            .is_err());

        for mutate in 0..16 {
            let mut invalid = joint6_first_position_profile();
            let joint = &mut invalid.joints[J6_FIRST_POSITION_INDEX];
            match mutate {
                0 => joint.name = "not_joint_6".into(),
                1 => joint.node_id = 1,
                2 => joint.direction = -1,
                3 => joint.zero_offset_rad += 1.0e-6,
                4 => joint.default_kp += 1.0,
                5 => joint.default_kd += 0.1,
                6 => joint.torque_scale -= 0.01,
                7 => joint.gravity_compensation_scale -= 0.01,
                8 => joint.torque_permille -= 1,
                9 => joint.kp_kd_torque_permille -= 1,
                10 => joint.limits.position_lower_rad -= 0.01,
                11 => joint.limits.position_upper_rad += 0.01,
                12 => joint.limits.velocity_rad_s += 0.01,
                13 => joint.limits.acceleration_rad_s2 += 0.01,
                14 => joint.limits.torque_nm += 0.1,
                15 => joint.limits.measured_position_margin_rad = 0.001,
                _ => unreachable!(),
            }
            assert!(
                request.validate(&invalid).is_err(),
                "joint_6 profile mutation {mutate} unexpectedly passed"
            );
        }
        let mut indirect = joint6_first_position_profile();
        indirect.bus.direct_joint_mapping = false;
        assert!(request.validate(&indirect).is_err());

        let joint = &profile.joints[J6_FIRST_POSITION_INDEX];
        validate_joint6_first_position_dynamic_window(joint, 0.13).unwrap();
        assert!(validate_joint6_first_position_dynamic_window(joint, -0.249).is_err());
        assert!(J6_FIRST_POSITION_DELTA_RAD.is_sign_negative());
        assert!(
            (J6_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV
                - J6_FIRST_POSITION_DELTA_RAD.abs() / TAU * 0.5)
                .abs()
                <= 1.0e-9
        );
    }

    #[test]
    fn joint6_followup_uses_fixed_temporary_gain_without_mutating_profile() {
        let profile = joint6_first_position_profile();
        let measured_q = [0.0; DOF];
        let targets = build_joint6_first_position_trajectory_targets(
            &profile,
            &zero_gravity_dynamics(),
            &measured_q,
            J6_FIRST_POSITION_DELTA_RAD,
        )
        .unwrap();
        let joint = &profile.joints[J6_FIRST_POSITION_INDEX];
        let target = targets[J6_FIRST_POSITION_INDEX];
        assert!((motor_kp_to_ros(target.kp_nm_rev, joint) - 50.0).abs() <= 1.0e-5);
        assert!((motor_kd_to_ros(target.kd_nm_s_rev, joint) - 2.0).abs() <= 1.0e-5);
        assert!((motor_position_to_ros(target.position_rev, joint) + 0.005).abs() <= 1.0e-6);
        assert_eq!(joint.default_kp.to_bits(), 25.0_f32.to_bits());
        assert_eq!(joint.default_kd.to_bits(), 1.0_f32.to_bits());
    }

    #[test]
    fn joint1_censored_torque_request_has_independent_authority_and_same_profile_locks() {
        let profile = joint1_first_position_profile();
        joint1_censored_torque_request(true)
            .validate(&profile)
            .unwrap();
        assert!(joint1_censored_torque_request(false)
            .validate(&profile)
            .is_err());

        let mut wrong_zero = joint1_first_position_profile();
        wrong_zero.joints[J1_FIRST_POSITION_INDEX].zero_offset_rad += 1.0e-6;
        assert!(joint1_censored_torque_request(true)
            .validate(&wrong_zero)
            .is_err());
        let mut indirect = joint1_first_position_profile();
        indirect.bus.direct_joint_mapping = false;
        assert!(joint1_censored_torque_request(true)
            .validate(&indirect)
            .is_err());

        joint1_negative_censored_torque_request(true)
            .validate(&profile)
            .unwrap();
        assert!(joint1_negative_censored_torque_request(false)
            .validate(&profile)
            .is_err());
        assert!(joint1_negative_censored_torque_request(true)
            .validate(&wrong_zero)
            .is_err());
    }

    #[test]
    fn joint1_censored_torque_levels_are_fixed_directional_and_wire_distinct() {
        let profile = joint1_first_position_profile();
        let joint = &profile.joints[J1_FIRST_POSITION_INDEX];
        let dynamics = zero_gravity_dynamics();
        let q = [0.0; DOF];
        let baseline = build_safe_hold_targets(
            &profile,
            &dynamics,
            &q,
            J1_FIRST_POSITION_INDEX,
            q[J1_FIRST_POSITION_INDEX],
        )
        .unwrap();
        let baseline_position_code =
            compressed_target_position_code(&baseline[J1_FIRST_POSITION_INDEX], joint);
        for direction in [
            Joint1CensoredTorqueDirection::Positive,
            Joint1CensoredTorqueDirection::Negative,
        ] {
            let mut previous_words =
                compressed_target_words(&baseline[J1_FIRST_POSITION_INDEX], joint);
            for level in 1..=J1_CENSORED_TORQUE_LEVELS {
                let additive = joint1_censored_torque_additive(level, direction).unwrap();
                assert_eq!(
                    additive.to_bits(),
                    (direction.sign() * level as f32 * 0.025).to_bits()
                );
                let targets = build_joint1_censored_torque_targets(
                    &profile, baseline, 0.0, 0.0, additive, direction,
                )
                .unwrap();
                assert_eq!(
                    compressed_target_position_code(&targets[J1_FIRST_POSITION_INDEX], joint),
                    baseline_position_code
                );
                let words = compressed_target_words(&targets[J1_FIRST_POSITION_INDEX], joint);
                assert_ne!(words, previous_words, "level {level} repeated wire words");
                previous_words = words;
            }
            assert_eq!(
                joint1_censored_torque_additive(J1_CENSORED_TORQUE_LEVELS, direction)
                    .unwrap()
                    .to_bits(),
                (direction.sign() * J1_CENSORED_TORQUE_CAP_NM).to_bits()
            );
            assert!(joint1_censored_torque_additive(0, direction).is_err());
            assert!(
                joint1_censored_torque_additive(J1_CENSORED_TORQUE_LEVELS + 1, direction).is_err()
            );
        }
    }

    #[test]
    fn joint1_censored_torque_identification_boundaries_fail_closed() {
        let profile = joint1_first_position_profile();
        let dynamics = zero_gravity_dynamics();
        let initial_q = [0.0; DOF];
        let joint = &profile.joints[J1_FIRST_POSITION_INDEX];
        let initial_motor = ros_target_to_motor(
            RosTarget {
                position_rad: 0.0,
                ..RosTarget::default()
            },
            joint,
        );
        let baseline_targets = build_safe_hold_targets(
            &profile,
            &dynamics,
            &initial_q,
            J1_FIRST_POSITION_INDEX,
            0.0,
        )
        .unwrap();
        let target = build_joint1_censored_torque_targets(
            &profile,
            baseline_targets,
            0.0,
            0.0,
            J1_CENSORED_TORQUE_STEP_NM,
            Joint1CensoredTorqueDirection::Positive,
        )
        .unwrap()[J1_FIRST_POSITION_INDEX];
        let mut feedback = healthy_diagnostic_feedback();
        let baseline = joint1_first_position_temperature_baseline(&profile, &feedback).unwrap();
        let observe = |feedback: &FeedbackSnapshot| {
            joint1_censored_torque_observation(
                &profile,
                feedback,
                &initial_q,
                initial_motor.position_rev,
                baseline,
                target,
                Joint1CensoredTorqueDirection::Positive,
            )
        };

        let set_state = |feedback: &mut FeedbackSnapshot, q: f32, dq: f32| {
            let motor = ros_target_to_motor(
                RosTarget {
                    position_rad: q,
                    velocity_rad_s: dq,
                    ..RosTarget::default()
                },
                joint,
            );
            feedback.joints[J1_FIRST_POSITION_INDEX].position_rev = motor.position_rev;
            feedback.joints[J1_FIRST_POSITION_INDEX].velocity_rev_s = motor.velocity_rev_s;
        };
        set_state(&mut feedback, J1_CENSORED_TORQUE_TRIGGER_RAD - 1.0e-7, 0.0);
        let below = observe(&feedback).unwrap();
        assert!(!joint1_censored_torque_should_censor(
            below,
            Joint1CensoredTorqueDirection::Positive
        ));
        set_state(&mut feedback, J1_CENSORED_TORQUE_TRIGGER_RAD + 1.0e-6, 0.0);
        let trigger = observe(&feedback).unwrap();
        assert!(joint1_censored_torque_should_censor(
            trigger,
            Joint1CensoredTorqueDirection::Positive
        ));
        set_state(
            &mut feedback,
            J1_CENSORED_TORQUE_HARD_POSITIVE_RAD + 1.0e-6,
            0.0,
        );
        assert!(observe(&feedback).is_err());
        set_state(&mut feedback, J1_CENSORED_TORQUE_HARD_NEGATIVE_RAD, 0.0);
        observe(&feedback).unwrap();
        set_state(
            &mut feedback,
            J1_CENSORED_TORQUE_HARD_NEGATIVE_RAD - 1.0e-6,
            0.0,
        );
        assert!(observe(&feedback).is_err());
        set_state(
            &mut feedback,
            0.0,
            J1_CENSORED_TORQUE_HARD_VELOCITY_RAD_S + 1.0e-5,
        );
        assert!(observe(&feedback).is_err());
    }

    #[test]
    fn joint1_negative_censored_torque_mirrors_censor_and_hard_boundaries() {
        let profile = joint1_first_position_profile();
        let dynamics = zero_gravity_dynamics();
        let initial_q = [0.0; DOF];
        let joint = &profile.joints[J1_FIRST_POSITION_INDEX];
        let initial_motor = ros_target_to_motor(
            RosTarget {
                position_rad: 0.0,
                ..RosTarget::default()
            },
            joint,
        );
        let baseline_targets = build_safe_hold_targets(
            &profile,
            &dynamics,
            &initial_q,
            J1_FIRST_POSITION_INDEX,
            0.0,
        )
        .unwrap();
        let direction = Joint1CensoredTorqueDirection::Negative;
        let target = build_joint1_censored_torque_targets(
            &profile,
            baseline_targets,
            0.0,
            0.0,
            -J1_CENSORED_TORQUE_STEP_NM,
            direction,
        )
        .unwrap()[J1_FIRST_POSITION_INDEX];
        let mut feedback = healthy_diagnostic_feedback();
        let baseline = joint1_first_position_temperature_baseline(&profile, &feedback).unwrap();
        let observe = |feedback: &FeedbackSnapshot| {
            joint1_censored_torque_observation(
                &profile,
                feedback,
                &initial_q,
                initial_motor.position_rev,
                baseline,
                target,
                direction,
            )
        };
        let set_state = |feedback: &mut FeedbackSnapshot, q: f32, dq: f32| {
            let motor = ros_target_to_motor(
                RosTarget {
                    position_rad: q,
                    velocity_rad_s: dq,
                    ..RosTarget::default()
                },
                joint,
            );
            feedback.joints[J1_FIRST_POSITION_INDEX].position_rev = motor.position_rev;
            feedback.joints[J1_FIRST_POSITION_INDEX].velocity_rev_s = motor.velocity_rev_s;
        };

        set_state(&mut feedback, -J1_CENSORED_TORQUE_TRIGGER_RAD + 1.0e-7, 0.0);
        assert!(!joint1_censored_torque_should_censor(
            observe(&feedback).unwrap(),
            direction
        ));
        set_state(&mut feedback, -J1_CENSORED_TORQUE_TRIGGER_RAD - 1.0e-6, 0.0);
        assert!(joint1_censored_torque_should_censor(
            observe(&feedback).unwrap(),
            direction
        ));
        set_state(
            &mut feedback,
            -J1_CENSORED_TORQUE_HARD_POSITIVE_RAD - 1.0e-6,
            0.0,
        );
        assert!(observe(&feedback).is_err());
        set_state(&mut feedback, -J1_CENSORED_TORQUE_HARD_NEGATIVE_RAD, 0.0);
        observe(&feedback).unwrap();
        set_state(
            &mut feedback,
            -J1_CENSORED_TORQUE_HARD_NEGATIVE_RAD + 1.0e-6,
            0.0,
        );
        assert!(observe(&feedback).is_err());
    }

    #[test]
    fn joint1_censored_torque_source_has_no_position_trajectory_and_promotes_after_dwell() {
        let source = include_str!("commissioning.rs");
        let body = source
            .split_once("pub async fn run_joint1_censored_torque_diagnostic(")
            .unwrap()
            .1
            .split_once("async fn wait_for_joint1_first_position_enable_stability(")
            .unwrap()
            .0;
        assert!(!body.contains("run_joint1_first_position_round_trip"));
        assert!(!body.contains("round_trip_phase"));
        let ramp = body
            .split_once("async fn run_joint1_censored_torque_ramp(")
            .unwrap()
            .1;
        let trigger = ramp.find("joint1_censored_torque_should_censor").unwrap();
        let proof = ramp.find("feedback_at > previous_feedback_at").unwrap();
        let dwell = ramp.find("Instant::now() >= level_deadline").unwrap();
        let promote = ramp
            .find("promote completed feedback-proven joint_1 torque level")
            .unwrap();
        assert!(trigger < proof && proof < dwell && dwell < promote);
        assert!(ramp.contains("verified_nonzero.context("));
        assert!(ramp.contains("frozen_words != target_words && frozen_words != baseline_words"));
    }

    #[test]
    fn joint1_first_position_run_checks_strict_zero_backend_before_policy_or_motion() {
        let source = include_str!("commissioning.rs");
        let body = source
            .split_once("pub async fn run_joint1_first_position_diagnostic(")
            .unwrap()
            .1
            .split_once("async fn wait_for_joint1_first_position_enable_stability(")
            .unwrap()
            .0;
        let strict_zero = body
            .find("ensure_joint1_first_position_strict_session(profile)")
            .unwrap();
        let request_validation = body.find("request.validate(profile)").unwrap();
        let enable = body
            .find("enable_joint1_first_position_diagnostic_axis")
            .unwrap();
        assert!(strict_zero < request_validation);
        assert!(request_validation < enable);
    }

    #[test]
    fn joint1_first_position_round_trip_proves_changed_target_and_returned_baseline_exactly() {
        let source = include_str!("commissioning.rs");
        let round_trip = source
            .split_once("async fn run_joint1_first_position_round_trip(")
            .unwrap()
            .1
            .split_once("async fn confirm_joint1_first_position_target_readback_while_guarded(")
            .unwrap()
            .0;
        assert_eq!(
            round_trip
                .matches("confirm_joint1_first_position_target_readback_while_guarded(")
                .count(),
            2
        );
        assert!(round_trip.contains("first_quantized_changed_position"));
        assert!(round_trip.contains("returned_persistent_baseline"));
        assert!(round_trip.contains("changed_target_readback"));

        let guarded_readback = source
            .split_once("async fn confirm_joint1_first_position_target_readback_while_guarded(")
            .unwrap()
            .1
            .split_once("fn validate_joint1_first_position_readback_poll(")
            .unwrap()
            .0;
        assert!(guarded_readback.contains("confirm_commissioning_target_readback"));
        assert!(guarded_readback.contains("tokio::time::sleep(LOOP_PERIOD)"));
    }

    #[test]
    fn joint1_first_position_q0_and_dynamic_window_are_both_hard_gates() {
        let profile = joint1_first_position_profile();
        let joint = &profile.joints[J1_FIRST_POSITION_INDEX];
        validate_joint1_first_position_dynamic_window(joint, J1_FIRST_POSITION_INITIAL_Q_LOWER_RAD)
            .unwrap();
        validate_joint1_first_position_dynamic_window(joint, J1_FIRST_POSITION_INITIAL_Q_UPPER_RAD)
            .unwrap();
        assert!(validate_joint1_first_position_dynamic_window(
            joint,
            J1_FIRST_POSITION_INITIAL_Q_LOWER_RAD - 1.0e-4,
        )
        .is_err());
        assert!(validate_joint1_first_position_dynamic_window(
            joint,
            J1_FIRST_POSITION_INITIAL_Q_UPPER_RAD + 1.0e-4,
        )
        .is_err());

        let mut narrow = joint.clone();
        narrow.limits.position_upper_rad = J1_FIRST_POSITION_INITIAL_Q_UPPER_RAD
            + J1_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD
            - 1.0e-4;
        assert!(validate_joint1_first_position_dynamic_window(
            &narrow,
            J1_FIRST_POSITION_INITIAL_Q_UPPER_RAD,
        )
        .is_err());
    }

    #[test]
    fn joint1_first_position_observation_checks_both_temperatures_on_all_six_axes() {
        let profile = joint1_first_position_profile();
        let dynamics = zero_gravity_dynamics();
        let initial_q = [0.0; DOF];
        let mut feedback = healthy_diagnostic_feedback();
        feedback.joints[J1_FIRST_POSITION_INDEX].position_rev = ros_target_to_motor(
            RosTarget {
                position_rad: initial_q[J1_FIRST_POSITION_INDEX],
                ..RosTarget::default()
            },
            &profile.joints[J1_FIRST_POSITION_INDEX],
        )
        .position_rev;
        let baseline = joint1_first_position_temperature_baseline(&profile, &feedback).unwrap();
        let targets = build_safe_hold_targets(
            &profile,
            &dynamics,
            &initial_q,
            J1_FIRST_POSITION_INDEX,
            0.0,
        )
        .unwrap();
        let observe = |feedback: &FeedbackSnapshot| {
            joint1_first_position_observation(
                &profile,
                feedback,
                J1_FIRST_POSITION_INDEX,
                &initial_q,
                0.0,
                baseline,
                targets[J1_FIRST_POSITION_INDEX],
                0.0,
            )
        };
        observe(&feedback).unwrap();

        feedback.joints[4].driver_temperature_c = 35.0 + MAX_DIAGNOSTIC_TEMPERATURE_RISE_C + 0.01;
        let error = observe(&feedback).unwrap_err().to_string();
        assert!(error.contains("joint_5 driver temperature rose"));
        feedback.joints[4].driver_temperature_c = 35.0;
        feedback.joints[5].motor_temperature_c = 36.0 + MAX_DIAGNOSTIC_TEMPERATURE_RISE_C + 0.01;
        let error = observe(&feedback).unwrap_err().to_string();
        assert!(error.contains("joint_6 motor temperature rose"));
        feedback.joints[5].motor_temperature_c = 36.0;
        feedback.joints[2].driver_temperature_c = f32::NAN;
        let error = observe(&feedback).unwrap_err().to_string();
        assert!(error.contains("joint_3 driver temperature"));
    }

    #[test]
    fn joint1_first_position_acceptance_locks_logical_and_raw_direction_evidence() {
        validate_joint1_first_position_acceptance(
            J1_FIRST_POSITION_REQUIRED_PEAK_RAD,
            -J1_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV,
        )
        .unwrap();
        validate_joint1_first_position_acceptance(
            J1_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD,
            -J1_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV,
        )
        .unwrap();
        assert!(validate_joint1_first_position_acceptance(
            J1_FIRST_POSITION_REQUIRED_PEAK_RAD - 1.0e-6,
            -J1_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV,
        )
        .is_err());
        assert!(validate_joint1_first_position_acceptance(
            J1_FIRST_POSITION_WINDOW_UPPER_OFFSET_RAD + 1.0e-6,
            -J1_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV,
        )
        .is_err());
        assert!(validate_joint1_first_position_acceptance(
            J1_FIRST_POSITION_REQUIRED_PEAK_RAD,
            -J1_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV + 1.0e-7,
        )
        .is_err());

        let profile = joint1_first_position_profile();
        let joint = &profile.joints[J1_FIRST_POSITION_INDEX];
        let start = ros_target_to_motor(
            RosTarget {
                position_rad: 0.0,
                ..RosTarget::default()
            },
            joint,
        );
        let accepted_peak = ros_target_to_motor(
            RosTarget {
                position_rad: J1_FIRST_POSITION_REQUIRED_PEAK_RAD,
                ..RosTarget::default()
            },
            joint,
        );
        let raw_delta = accepted_peak.position_rev - start.position_rev;
        assert!(raw_delta.is_sign_negative());
        assert!(raw_delta <= -J1_FIRST_POSITION_REQUIRED_RAW_NEGATIVE_REV);
    }

    #[test]
    fn j2_diagnostic_request_is_explicit_bounded_and_positive_only() {
        let profile = profile_with_velocity_limit(0.1);
        SingleAxisDiagnosticRequest {
            selected_index: DIAGNOSTIC_JOINT_INDEX,
            mode: SingleAxisDiagnosticMode::Hold { duration_sec: 0.5 },
            high_torque_authorized: false,
            censored_gravity_hold_authorized: false,
        }
        .validate(&profile)
        .unwrap();
        SingleAxisDiagnosticRequest {
            selected_index: DIAGNOSTIC_JOINT_INDEX,
            mode: SingleAxisDiagnosticMode::TauFfStaircase {
                peak_additive_torque_nm: 0.25,
                step_torque_nm: 0.025,
                dwell_sec: 0.20,
            },
            high_torque_authorized: false,
            censored_gravity_hold_authorized: false,
        }
        .validate(&profile)
        .unwrap();

        for invalid in [
            SingleAxisDiagnosticRequest {
                selected_index: 0,
                mode: SingleAxisDiagnosticMode::Hold { duration_sec: 0.5 },
                high_torque_authorized: false,
                censored_gravity_hold_authorized: false,
            },
            SingleAxisDiagnosticRequest {
                selected_index: DIAGNOSTIC_JOINT_INDEX,
                mode: SingleAxisDiagnosticMode::TauFfStaircase {
                    peak_additive_torque_nm: -0.10,
                    step_torque_nm: 0.025,
                    dwell_sec: 0.20,
                },
                high_torque_authorized: false,
                censored_gravity_hold_authorized: false,
            },
            SingleAxisDiagnosticRequest {
                selected_index: DIAGNOSTIC_JOINT_INDEX,
                mode: SingleAxisDiagnosticMode::TauFfStaircase {
                    peak_additive_torque_nm: LOW_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM + 0.001,
                    step_torque_nm: 0.025,
                    dwell_sec: 0.20,
                },
                high_torque_authorized: false,
                censored_gravity_hold_authorized: false,
            },
        ] {
            assert!(invalid.validate(&profile).is_err());
        }
    }

    #[test]
    fn high_torque_tier_requires_second_ack_and_preserves_all_hard_caps() {
        let profile = profile_with_velocity_limit(0.1);
        let request = |peak, step, dwell, authorized| SingleAxisDiagnosticRequest {
            selected_index: DIAGNOSTIC_JOINT_INDEX,
            mode: SingleAxisDiagnosticMode::TauFfStaircase {
                peak_additive_torque_nm: peak,
                step_torque_nm: step,
                dwell_sec: dwell,
            },
            high_torque_authorized: authorized,
            censored_gravity_hold_authorized: false,
        };

        request(1.50, 0.25, 0.20, true).validate(&profile).unwrap();
        request(1.50, 0.10, 0.20, true).validate(&profile).unwrap();
        let planned_levels = diagnostic_torque_levels(1.50, 0.10).unwrap();
        assert_eq!(planned_levels.len(), 15);
        assert_eq!(planned_levels.last().copied(), Some(1.50));
        request(1.75, 0.25, 0.25, true).validate(&profile).unwrap();
        let missing_ack = request(0.251, 0.25, 0.20, false)
            .validate(&profile)
            .unwrap_err()
            .to_string();
        assert!(missing_ack.contains("explicit high-torque authorization"));
        assert!(request(1.751, 0.25, 0.20, true).validate(&profile).is_err());
        assert!(request(1.50, 0.251, 0.20, true).validate(&profile).is_err());
        assert!(request(1.50, 0.25, 0.251, true).validate(&profile).is_err());
        assert!(request(1.75, 0.10, 0.25, true).validate(&profile).is_err());

        let hold_with_high_ack = SingleAxisDiagnosticRequest {
            selected_index: DIAGNOSTIC_JOINT_INDEX,
            mode: SingleAxisDiagnosticMode::Hold { duration_sec: 0.5 },
            high_torque_authorized: true,
            censored_gravity_hold_authorized: false,
        };
        assert!(hold_with_high_ack.validate(&profile).is_err());

        let acknowledged_low = request(0.25, 0.25, 0.20, true).torque_limits();
        assert!(!acknowledged_low.high_tier);
        assert_eq!(
            acknowledged_low.max_total_torque_nm,
            LOW_TIER_MAX_DIAGNOSTIC_TOTAL_TORQUE_NM
        );
    }

    #[test]
    fn position_round_trip_is_fixed_high_tier_and_locks_the_reviewed_j2_profile() {
        let profile = position_diagnostic_profile();
        let request = position_diagnostic_request(true);
        request.validate(&profile).unwrap();
        assert!(request.uses_high_torque_tier());
        let limits = request.torque_limits();
        assert!(limits.high_tier);
        assert_eq!(
            limits.max_total_torque_nm,
            POSITION_DIAGNOSTIC_MAX_TOTAL_TORQUE_NM
        );
        assert_eq!(limits.max_active_sec, POSITION_DIAGNOSTIC_MAX_ACTIVE_SEC);

        assert!(position_diagnostic_request(false)
            .validate(&profile)
            .is_err());
        for mode in [
            SingleAxisDiagnosticMode::PositionRoundTrip {
                delta_rad: -POSITION_DIAGNOSTIC_DELTA_RAD,
                duration_sec: POSITION_DIAGNOSTIC_DURATION_SEC,
            },
            SingleAxisDiagnosticMode::PositionRoundTrip {
                delta_rad: POSITION_DIAGNOSTIC_DELTA_RAD - 0.0001,
                duration_sec: POSITION_DIAGNOSTIC_DURATION_SEC,
            },
            SingleAxisDiagnosticMode::PositionRoundTrip {
                delta_rad: POSITION_DIAGNOSTIC_DELTA_RAD,
                duration_sec: POSITION_DIAGNOSTIC_DURATION_SEC + 0.1,
            },
        ] {
            assert!(SingleAxisDiagnosticRequest {
                selected_index: DIAGNOSTIC_JOINT_INDEX,
                mode,
                high_torque_authorized: true,
                censored_gravity_hold_authorized: false,
            }
            .validate(&profile)
            .is_err());
        }

        for mutate in 0..5 {
            let mut invalid = position_diagnostic_profile();
            let joint = &mut invalid.joints[DIAGNOSTIC_JOINT_INDEX];
            match mutate {
                0 => joint.gravity_compensation_scale += 0.01,
                1 => joint.default_kp -= 1.0,
                2 => joint.default_kd -= 0.1,
                3 => joint.torque_permille -= 1,
                4 => joint.kp_kd_torque_permille -= 1,
                _ => unreachable!(),
            }
            assert!(request.validate(&invalid).is_err());
        }
    }

    #[test]
    fn censored_gravity_hold_is_fixed_high_tier_and_locks_the_exercised_baseline() {
        let profile = censored_gravity_hold_profile();
        let request = censored_gravity_hold_request(true, true);
        request.validate(&profile).unwrap();
        assert!(request.uses_high_torque_tier());

        let limits = request.torque_limits();
        assert!(limits.high_tier);
        assert_eq!(limits.max_additive_torque_nm, 0.0);
        assert_eq!(
            limits.max_total_torque_nm,
            POSITION_DIAGNOSTIC_MAX_TOTAL_TORQUE_NM
        );
        assert_eq!(limits.max_active_sec, CENSORED_GRAVITY_HOLD_MAX_ACTIVE_SEC);
        assert!(censored_gravity_hold_request(false, true)
            .validate(&profile)
            .is_err());
        assert!(censored_gravity_hold_request(true, false)
            .validate(&profile)
            .is_err());
        let mut unrelated_mode_with_censored_ack = position_diagnostic_request(true);
        unrelated_mode_with_censored_ack.censored_gravity_hold_authorized = true;
        assert!(unrelated_mode_with_censored_ack
            .validate(&position_diagnostic_profile())
            .is_err());

        for mutate in 0..5 {
            let mut invalid = censored_gravity_hold_profile();
            let joint = &mut invalid.joints[DIAGNOSTIC_JOINT_INDEX];
            match mutate {
                0 => joint.gravity_compensation_scale = CENSORED_GRAVITY_HOLD_CAP_SCALE,
                1 => joint.default_kp -= 1.0,
                2 => joint.default_kd -= 0.1,
                3 => joint.torque_permille -= 1,
                4 => joint.kp_kd_torque_permille -= 1,
                _ => unreachable!(),
            }
            assert!(request.validate(&invalid).is_err());
        }
    }

    #[test]
    fn censored_gravity_hold_scale_has_fixed_endpoints_monotonicity_and_step_cap() {
        let first = censored_gravity_hold_scale(0).unwrap();
        let last = censored_gravity_hold_scale(CENSORED_GRAVITY_HOLD_STEPS).unwrap();
        assert_eq!(
            first.to_bits(),
            CENSORED_GRAVITY_HOLD_PROFILE_SCALE.to_bits()
        );
        assert_eq!(last.to_bits(), CENSORED_GRAVITY_HOLD_CAP_SCALE.to_bits());
        assert!(censored_gravity_hold_scale(CENSORED_GRAVITY_HOLD_STEPS + 1).is_err());

        let mut previous = first;
        let mut maximum_step = 0.0_f32;
        for step in 1..=CENSORED_GRAVITY_HOLD_STEPS {
            let current = censored_gravity_hold_scale(step).unwrap();
            assert!(
                current >= previous,
                "step {step} moved gravity scale backwards"
            );
            assert!(
                current <= CENSORED_GRAVITY_HOLD_CAP_SCALE,
                "step {step} exceeded the fixed gravity scale cap"
            );
            maximum_step = maximum_step.max(current - previous);
            previous = current;
        }
        assert!(maximum_step <= CENSORED_GRAVITY_HOLD_MAX_SCALE_STEP);
        assert!(
            (LOOP_PERIOD.as_secs_f32() * CENSORED_GRAVITY_HOLD_STEPS as f32
                - CENSORED_GRAVITY_HOLD_NOMINAL_DURATION_SEC)
                .abs()
                <= f32::EPSILON
        );
        assert!(POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY < LOOP_PERIOD);
    }

    #[test]
    fn censored_gravity_hold_displacement_milestones_are_boundary_exact_and_deduplicated() {
        let mut milestones = CensoredGravityHoldDisplacementMilestones::default();
        assert_eq!(milestones.take_due(0.000_099_9), None);
        assert_eq!(
            milestones.take_due(0.000_100),
            Some((0.000_100, "positive_0p1_mrad"))
        );
        assert_eq!(milestones.take_due(0.000_100), None);

        // A single fresh sample may cross both remaining boundaries. Calling
        // take_due repeatedly records each one exactly once before the censor
        // decision can reject the currently published target.
        assert_eq!(
            milestones.take_due(0.000_300),
            Some((0.000_200, "positive_0p2_mrad"))
        );
        assert_eq!(
            milestones.take_due(0.000_300),
            Some((0.000_300, "positive_0p3_mrad_censor"))
        );
        assert_eq!(milestones.take_due(0.000_300), None);
        assert_eq!(milestones.take_due(0.001), None);
    }

    #[test]
    fn censored_gravity_hold_rejects_identification_boundaries_before_freezing() {
        assert_eq!(
            censored_gravity_hold_decision(censored_gravity_hold_observation(
                CENSORED_GRAVITY_HOLD_TRIGGER_RAD - 1.0e-7,
                0.0,
            ))
            .unwrap(),
            CensoredGravityHoldDecision::Advance
        );
        assert_eq!(
            censored_gravity_hold_decision(censored_gravity_hold_observation(
                CENSORED_GRAVITY_HOLD_TRIGGER_RAD,
                0.0,
            ))
            .unwrap(),
            CensoredGravityHoldDecision::FreezePrevious
        );
        assert_eq!(
            censored_gravity_hold_poll_decision(
                censored_gravity_hold_observation(CENSORED_GRAVITY_HOLD_TRIGGER_RAD, 0.0),
                false,
            )
            .unwrap(),
            CensoredGravityHoldPollDecision::FreezePrevious,
            "the displacement censor must not wait for the all-six target proof"
        );
        assert_eq!(
            censored_gravity_hold_poll_decision(
                censored_gravity_hold_observation(CENSORED_GRAVITY_HOLD_TRIGGER_RAD - 1.0e-7, 0.0,),
                false,
            )
            .unwrap(),
            CensoredGravityHoldPollDecision::AwaitTargetProof
        );
        assert_eq!(
            censored_gravity_hold_poll_decision(
                censored_gravity_hold_observation(CENSORED_GRAVITY_HOLD_TRIGGER_RAD - 1.0e-7, 0.0,),
                true,
            )
            .unwrap(),
            CensoredGravityHoldPollDecision::ConfirmPublished
        );

        for (position, velocity, expected) in [
            (
                CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD,
                0.0,
                "displacement",
            ),
            (
                CENSORED_GRAVITY_HOLD_TRIGGER_RAD,
                CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S,
                "velocity",
            ),
            (
                CENSORED_GRAVITY_HOLD_TRIGGER_RAD,
                -CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S,
                "velocity",
            ),
        ] {
            let error = censored_gravity_hold_decision(censored_gravity_hold_observation(
                position, velocity,
            ))
            .unwrap_err()
            .to_string();
            assert!(error.contains(expected), "unexpected error: {error}");
            assert!(
                censored_gravity_hold_poll_decision(
                    censored_gravity_hold_observation(position, velocity),
                    false,
                )
                .is_err(),
                "identification exit must not wait for all-six target proof"
            );
        }
    }

    #[test]
    fn censored_gravity_hold_censor_keeps_previous_verified_cleanup_target() {
        let targets = |torque_nm| {
            array::from_fn(|_| MotorTarget {
                torque_nm,
                ..MotorTarget::default()
            })
        };
        let previous_verified = targets(0.27);
        let currently_published = targets(0.28);
        let next_prospective = targets(0.29);
        let mut progress = PositionRampTargetProgress::new(currently_published, previous_verified);

        let decision = censored_gravity_hold_decision(censored_gravity_hold_observation(
            CENSORED_GRAVITY_HOLD_TRIGGER_RAD,
            0.0,
        ))
        .unwrap();
        assert_eq!(decision, CensoredGravityHoldDecision::FreezePrevious);
        // The runtime intentionally does not call confirm_published() on the
        // triggering sample. Signal/error cleanup therefore remains exactly
        // one feedback-verified target behind the rejected wire target.
        assert_eq!(progress.verified, previous_verified);
        assert_eq!(progress.published, currently_published);

        assert_eq!(
            censored_gravity_hold_decision(censored_gravity_hold_observation(0.000_299, 0.0))
                .unwrap(),
            CensoredGravityHoldDecision::Advance
        );
        assert_eq!(progress.confirm_published(), currently_published);
        progress.publish(next_prospective);
        assert_eq!(progress.verified, currently_published);
        assert_eq!(progress.published, next_prospective);
    }

    #[test]
    fn censored_gravity_hold_rollback_skips_repeated_quantized_words() {
        let profile = censored_gravity_hold_profile();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let joint = &profile.joints[selected];
        let targets = |torque_nm| {
            array::from_fn(|_| MotorTarget {
                torque_nm,
                ..MotorTarget::default()
            })
        };
        let configured = targets(0.10);
        let distinct_prior = targets(0.20);
        let latest_verified = targets(0.30);
        let rejected_same_words = latest_verified;
        assert_ne!(
            compressed_target_words(&configured[selected], joint),
            compressed_target_words(&distinct_prior[selected], joint)
        );
        assert_ne!(
            compressed_target_words(&distinct_prior[selected], joint),
            compressed_target_words(&latest_verified[selected], joint)
        );

        let mut history = CensoredVerifiedTargetHistory::default();
        history.record(
            0,
            CENSORED_GRAVITY_HOLD_PROFILE_SCALE,
            configured,
            selected,
            joint,
        );
        history.record(10, 0.30, distinct_prior, selected, joint);
        history.record(11, 0.31, latest_verified, selected, joint);
        // A floating-point microstep that compresses to the same words is not
        // a new hardware rollback candidate.
        history.record(12, 0.32, rejected_same_words, selected, joint);
        assert_eq!(history.distinct.len(), 3);

        let rollback = history
            .rollback_candidate(&rejected_same_words, &configured, selected, joint)
            .unwrap();
        assert_eq!(rollback.step, 10);
        assert_eq!(rollback.scale, 0.30);
        assert_eq!(rollback.targets, distinct_prior);
        assert_ne!(
            compressed_target_words(&rollback.targets[selected], joint),
            compressed_target_words(&rejected_same_words[selected], joint)
        );
        assert_ne!(
            compressed_target_words(&rollback.targets[selected], joint),
            compressed_target_words(&configured[selected], joint)
        );
    }

    #[test]
    fn censored_gravity_hold_has_no_rollback_candidate_at_first_microstep() {
        let profile = censored_gravity_hold_profile();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let joint = &profile.joints[selected];
        let targets = |torque_nm| {
            array::from_fn(|_| MotorTarget {
                torque_nm,
                ..MotorTarget::default()
            })
        };
        let configured = targets(0.10);
        let stable_point_two_five = targets(0.15);
        let first_microstep = targets(0.20);
        let mut history = CensoredVerifiedTargetHistory::default();
        history.record(
            0,
            CENSORED_GRAVITY_HOLD_PROFILE_SCALE,
            configured,
            selected,
            joint,
        );
        history.record(
            0,
            CENSORED_GRAVITY_HOLD_PROFILE_SCALE,
            stable_point_two_five,
            selected,
            joint,
        );
        assert!(history
            .rollback_candidate(&first_microstep, &configured, selected, joint)
            .is_none());
    }

    #[test]
    fn censored_half_cosine_contains_repeated_quantized_steps_but_fixed_position_codes() {
        let profile = censored_gravity_hold_profile();
        let dynamics = position_gravity_dynamics();
        let q = [0.0; DOF];
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let joint = &profile.joints[selected];
        let mut previous_words = None;
        let mut previous_position_code = None;
        let mut repeated_adjacent_words = 0_usize;

        for step in 0..=CENSORED_GRAVITY_HOLD_STEPS {
            let targets = build_position_diagnostic_hold_targets(
                &profile,
                &dynamics,
                &q,
                selected,
                q[selected],
                censored_gravity_hold_scale(step).unwrap(),
            )
            .unwrap();
            let words = compressed_target_words(&targets[selected], joint);
            let position_code = compressed_target_position_code(&targets[selected], joint);
            if previous_words == Some(words) {
                repeated_adjacent_words += 1;
            }
            if let Some(previous_position_code) = previous_position_code {
                assert_eq!(position_code, previous_position_code);
            }
            previous_words = Some(words);
            previous_position_code = Some(position_code);
        }
        assert!(
            repeated_adjacent_words > 0,
            "the regression fixture must exercise adjacent floating steps with identical 0x2004 words"
        );
    }

    #[test]
    fn censored_gravity_hold_changes_only_feedforward_at_fixed_position() {
        let profile = censored_gravity_hold_profile();
        let dynamics = position_gravity_dynamics();
        let q = [0.0; DOF];
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let joint = &profile.joints[selected];
        let initial = build_position_diagnostic_hold_targets(
            &profile,
            &dynamics,
            &q,
            selected,
            q[selected],
            CENSORED_GRAVITY_HOLD_PROFILE_SCALE,
        )
        .unwrap();
        let cap = build_position_diagnostic_hold_targets(
            &profile,
            &dynamics,
            &q,
            selected,
            q[selected],
            CENSORED_GRAVITY_HOLD_CAP_SCALE,
        )
        .unwrap();

        assert_eq!(
            compressed_target_position_code(&initial[selected], joint),
            compressed_target_position_code(&cap[selected], joint)
        );
        assert_eq!(initial[selected].position_rev, cap[selected].position_rev);
        assert_eq!(
            initial[selected].velocity_rev_s,
            cap[selected].velocity_rev_s
        );
        assert_eq!(initial[selected].kp_nm_rev, cap[selected].kp_nm_rev);
        assert_eq!(initial[selected].kd_nm_s_rev, cap[selected].kd_nm_s_rev);
        assert_ne!(initial[selected].torque_nm, cap[selected].torque_nm);
        assert_ne!(
            compressed_target_words(&initial[selected], joint),
            compressed_target_words(&cap[selected], joint)
        );
    }

    #[test]
    fn censored_gravity_hold_statistics_require_fresh_samples_and_terminal_stability() {
        let origin = Instant::now();
        let build_statistics = |terminal_span_rad: f32, terminal_velocity_rad_s: f32| {
            let mut statistics = CensoredHoldStatistics::default();
            for index in 0..CENSORED_GRAVITY_HOLD_MIN_STATISTICS_SAMPLES {
                let fraction =
                    index as f32 / (CENSORED_GRAVITY_HOLD_MIN_STATISTICS_SAMPLES - 1) as f32;
                let position =
                    CENSORED_GRAVITY_HOLD_TRIGGER_RAD + terminal_span_rad * (fraction - 0.5);
                statistics
                    .observe(
                        origin + LOOP_PERIOD * index as u32,
                        censored_gravity_hold_observation(position, terminal_velocity_rad_s),
                    )
                    .unwrap();
            }
            statistics
        };

        let stable = build_statistics(
            CENSORED_GRAVITY_HOLD_TERMINAL_POSITION_SPAN_RAD * 0.5,
            CENSORED_GRAVITY_HOLD_TERMINAL_VELOCITY_RAD_S * 0.5,
        )
        .summarize()
        .unwrap();
        assert_eq!(
            stable.sample_count,
            CENSORED_GRAVITY_HOLD_MIN_STATISTICS_SAMPLES
        );
        assert!(stable.terminal_sample_count >= CENSORED_GRAVITY_HOLD_MIN_TERMINAL_SAMPLES);
        assert!(stable.terminal_stable);

        let excessive_span =
            build_statistics(CENSORED_GRAVITY_HOLD_TERMINAL_POSITION_SPAN_RAD * 3.0, 0.0)
                .summarize()
                .unwrap();
        assert!(!excessive_span.terminal_stable);
        let excessive_velocity =
            build_statistics(0.0, CENSORED_GRAVITY_HOLD_TERMINAL_VELOCITY_RAD_S + 1.0e-6)
                .summarize()
                .unwrap();
        assert!(!excessive_velocity.terminal_stable);

        let mut insufficient = CensoredHoldStatistics::default();
        insufficient
            .observe(origin, censored_gravity_hold_observation(0.0, 0.0))
            .unwrap();
        assert!(insufficient
            .observe(origin, censored_gravity_hold_observation(0.0, 0.0))
            .is_err());
        assert!(insufficient.summarize().is_err());
    }

    #[test]
    fn position_gravity_ramp_has_fixed_endpoints_monotonicity_and_step_cap() {
        let first = position_diagnostic_gravity_ramp_scale(0).unwrap();
        let last =
            position_diagnostic_gravity_ramp_scale(POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS).unwrap();
        assert_eq!(
            first.to_bits(),
            POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE.to_bits()
        );
        assert!((last - POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE).abs() <= f32::EPSILON);
        assert!(
            position_diagnostic_gravity_ramp_scale(POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS + 1)
                .is_err()
        );

        let mut previous = first;
        let mut maximum_step = 0.0_f32;
        for step in 1..=POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS {
            let current = position_diagnostic_gravity_ramp_scale(step).unwrap();
            assert!(
                current >= previous,
                "step {step} moved gravity scale backwards"
            );
            maximum_step = maximum_step.max(current - previous);
            previous = current;
        }
        assert!(maximum_step <= POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_SCALE_STEP);
        assert!(
            (LOOP_PERIOD.as_secs_f32() * POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS as f32
                - POSITION_DIAGNOSTIC_GRAVITY_RAMP_DURATION_SEC)
                .abs()
                <= f32::EPSILON
        );
        assert!(POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY < LOOP_PERIOD);

        let previous_feedback_at = Instant::now();
        let target_proof_after =
            previous_feedback_at + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
        assert!(!position_gravity_ramp_feedback_proves_target(
            target_proof_after,
            previous_feedback_at,
            target_proof_after,
        ));
        assert!(position_gravity_ramp_feedback_proves_target(
            target_proof_after + Duration::from_nanos(1),
            previous_feedback_at,
            target_proof_after,
        ));
    }

    #[test]
    fn position_gravity_ramp_uses_one_ms_tpdo1_progress_not_twenty_ms_tpdo2_cadence() {
        fn simulated_endpoint(use_required_tpdo_timestamp: bool) -> Duration {
            const TPDO1_PERIOD_MS: u64 = 1;
            const TPDO2_PERIOD_MS: u64 = 20;

            let origin = Instant::now();
            let mut elapsed = LOOP_PERIOD;
            let mut previous_feedback_at = origin;
            let mut target_proof_after = origin + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
            let mut step = 0_usize;

            loop {
                // The ramp task observes the 1 ms and 20 ms streams at its
                // fixed 4 ms loop cadence. `captured_at` is intentionally the
                // older required stream, matching RealBackend aggregation.
                let elapsed_ms = elapsed.as_millis() as u64;
                let last_tpdo1_ms = elapsed_ms / TPDO1_PERIOD_MS * TPDO1_PERIOD_MS;
                let last_tpdo2_ms = elapsed_ms / TPDO2_PERIOD_MS * TPDO2_PERIOD_MS;
                let feedback = FeedbackSnapshot {
                    oldest_tpdo1_at: Some(origin + Duration::from_millis(last_tpdo1_ms)),
                    captured_at: Some(
                        origin + Duration::from_millis(last_tpdo1_ms.min(last_tpdo2_ms)),
                    ),
                    ..FeedbackSnapshot::default()
                };
                let feedback_at = if use_required_tpdo_timestamp {
                    feedback.captured_at.unwrap()
                } else {
                    feedback.oldest_tpdo1_at.unwrap()
                };

                if position_gravity_ramp_feedback_proves_target(
                    feedback_at,
                    previous_feedback_at,
                    target_proof_after,
                ) {
                    previous_feedback_at = feedback_at;
                    if step == POSITION_DIAGNOSTIC_GRAVITY_RAMP_STEPS {
                        return elapsed;
                    }
                    step += 1;
                    // Model the actual contract: a newly published target is
                    // never provable until strictly after two 1 kHz sender
                    // periods, even when TPDO1 itself advances every 1 ms.
                    target_proof_after =
                        origin + elapsed + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY;
                }

                elapsed += LOOP_PERIOD;
                assert!(elapsed <= Duration::from_secs(8));
            }
        }

        let tpdo1_driven_endpoint = simulated_endpoint(false);
        assert_eq!(tpdo1_driven_endpoint, Duration::from_millis(1_504));

        let required_tpdo_driven_endpoint = simulated_endpoint(true);
        assert_eq!(required_tpdo_driven_endpoint, Duration::from_millis(7_520));
        assert!(required_tpdo_driven_endpoint >= Duration::from_millis(7_500));
    }

    #[test]
    fn tpdo1_ramp_progress_does_not_bypass_a_stale_tpdo2_feedback_gate() {
        let profile = position_diagnostic_profile();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let joint = &profile.joints[selected];
        let initial = joint.limits.position_lower_rad + 0.001;
        let mut initial_q = [0.0; DOF];
        initial_q[selected] = initial;
        let baseline = DiagnosticTemperatureBaseline {
            driver_c: 35.0,
            motor_c: 36.0,
        };
        let target = ros_target_to_motor(
            RosTarget {
                position_rad: initial,
                velocity_rad_s: 0.0,
                torque_nm: 0.0,
                kp_nm_rad: joint.default_kp,
                kd_nm_s_rad: joint.default_kd,
            },
            joint,
        );
        let limits = position_diagnostic_request(true).torque_limits();
        let origin = Instant::now();
        let mut feedback = healthy_diagnostic_feedback();
        feedback.joints[selected].position_rev = initial / TAU;
        feedback.oldest_tpdo1_at = Some(origin + LOOP_PERIOD);
        // The slower TPDO2 holds the ordinary required-stream timestamp back.
        feedback.captured_at = Some(origin);

        assert!(position_gravity_ramp_feedback_proves_target(
            feedback.oldest_tpdo1_at.unwrap(),
            origin,
            origin + POSITION_DIAGNOSTIC_GRAVITY_RAMP_PROOF_DELAY,
        ));
        position_diagnostic_observation(
            &profile, &feedback, selected, &initial_q, baseline, target, initial, limits,
        )
        .unwrap();

        // RealBackend derives `fresh` from required_tpdos_fresh(), which
        // includes TPDO2. A new TPDO1 progression timestamp therefore cannot
        // make a stale TPDO2 sample pass the unchanged safety observation.
        feedback.joints[4].fresh = false;
        let error = position_diagnostic_observation(
            &profile, &feedback, selected, &initial_q, baseline, target, initial, limits,
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("offline, stale, or faulted"));
    }

    #[test]
    fn position_pre_enable_target_uses_point_two_five_scale_without_changing_hold_gains() {
        let profile = position_diagnostic_profile();
        let dynamics = position_gravity_dynamics();
        let q = [0.0; DOF];
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let initial = build_position_diagnostic_hold_targets(
            &profile,
            &dynamics,
            &q,
            selected,
            q[selected],
            POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE,
        )
        .unwrap();
        let endpoint = build_position_diagnostic_hold_targets(
            &profile,
            &dynamics,
            &q,
            selected,
            q[selected],
            POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE,
        )
        .unwrap();
        let ordinary_profile_target =
            build_safe_hold_targets(&profile, &dynamics, &q, selected, q[selected]).unwrap();
        let joint = &profile.joints[selected];
        let model_gravity =
            dynamics.gravity_torque_with(&q, profile.gravity_vector_base_m_s2)[selected];
        let initial_ff = motor_torque_to_ros(initial[selected].torque_nm, joint);
        let endpoint_ff = motor_torque_to_ros(endpoint[selected].torque_nm, joint);

        assert!(
            (initial_ff - model_gravity * POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE).abs() < 1.0e-6
        );
        assert!(
            (endpoint_ff - model_gravity * POSITION_DIAGNOSTIC_EXPECTED_GRAVITY_SCALE).abs()
                < 1.0e-6
        );
        assert_eq!(endpoint[selected], ordinary_profile_target[selected]);
        assert_eq!(
            initial[selected].position_rev,
            endpoint[selected].position_rev
        );
        assert_eq!(
            initial[selected].velocity_rev_s,
            endpoint[selected].velocity_rev_s
        );
        assert_eq!(initial[selected].kp_nm_rev, endpoint[selected].kp_nm_rev);
        assert_eq!(
            initial[selected].kd_nm_s_rev,
            endpoint[selected].kd_nm_s_rev
        );
        assert_ne!(initial[selected].torque_nm, endpoint[selected].torque_nm);
    }

    #[test]
    fn gravity_ramp_persistent_baseline_lags_until_published_target_is_verified() {
        let targets = |torque_nm| {
            array::from_fn(|_| MotorTarget {
                torque_nm,
                ..MotorTarget::default()
            })
        };
        let verified = targets(0.25);
        let published = targets(0.30);
        let prospective = targets(0.35);
        let mut progress = PositionRampTargetProgress::new(published, verified);

        assert_eq!(progress.verified, verified);
        assert_eq!(progress.published, published);
        assert_eq!(progress.confirm_published(), published);
        progress.publish(prospective);
        assert_eq!(progress.verified, published);
        assert_eq!(progress.published, prospective);
        assert_eq!(progress.confirm_published(), prospective);
    }

    #[test]
    fn gravity_ramp_rejects_position_velocity_and_soft_go_boundaries() {
        let profile = position_diagnostic_profile();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let lower = profile.joints[selected].limits.position_lower_rad;
        let observation =
            |measured_q: f32, measured_delta: f32, measured_velocity: f32| DiagnosticObservation {
                telemetry: CommissioningTelemetrySample {
                    commanded_q: measured_q - measured_delta,
                    measured_q,
                    measured_delta,
                    measured_velocity,
                    measured_torque: 1.0,
                    gravity_ff: 1.0,
                    kp: POSITION_DIAGNOSTIC_EXPECTED_KP,
                    kd: POSITION_DIAGNOSTIC_EXPECTED_KD,
                    estimated_pd_torque: 0.0,
                },
                driver_temperature_c: 35.0,
                motor_temperature_c: 36.0,
                breakaway_detected: false,
            };

        validate_position_gravity_ramp_observation(
            &profile,
            selected,
            observation(
                lower + POSITION_DIAGNOSTIC_GRAVITY_RAMP_ABSOLUTE_UPPER_OFFSET_RAD,
                POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_POSITIVE_RAD,
                MAX_DIAGNOSTIC_VELOCITY_RAD_S,
            ),
        )
        .unwrap();
        validate_position_gravity_ramp_observation(
            &profile,
            selected,
            observation(
                lower,
                -POSITION_DIAGNOSTIC_MAX_OPPOSITE_POSITION_RAD,
                -MAX_DIAGNOSTIC_VELOCITY_RAD_S,
            ),
        )
        .unwrap();

        for (measured_q, delta, velocity, expected) in [
            (
                lower + 0.003,
                POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_POSITIVE_RAD + 1.0e-6,
                0.0,
                "excursion",
            ),
            (
                lower + 0.003,
                -POSITION_DIAGNOSTIC_MAX_OPPOSITE_POSITION_RAD - 1.0e-6,
                0.0,
                "opposite",
            ),
            (
                lower + POSITION_DIAGNOSTIC_GRAVITY_RAMP_ABSOLUTE_UPPER_OFFSET_RAD + 1.0e-6,
                0.001,
                0.0,
                "absolute position",
            ),
            (
                lower + 0.003,
                0.001,
                MAX_DIAGNOSTIC_VELOCITY_RAD_S + 1.0e-6,
                "velocity",
            ),
        ] {
            let error = validate_position_gravity_ramp_observation(
                &profile,
                selected,
                observation(measured_q, delta, velocity),
            )
            .unwrap_err()
            .to_string();
            assert!(error.contains(expected), "unexpected error: {error}");
        }

        validate_position_gravity_ramp_go_peaks(
            POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_POSITION_RAD,
            POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_VELOCITY_RAD_S,
        )
        .unwrap();
        assert!(validate_position_gravity_ramp_go_peaks(
            POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_POSITION_RAD + 1.0e-6,
            0.0,
        )
        .is_err());
        assert!(validate_position_gravity_ramp_go_peaks(
            0.0,
            POSITION_DIAGNOSTIC_GRAVITY_RAMP_GO_VELOCITY_RAD_S + 1.0e-6,
        )
        .is_err());
    }

    #[test]
    fn j2_diagnostic_start_gate_is_asymmetric_and_tier_specific() {
        let profile = profile_with_velocity_limit(0.1);
        let joint = &profile.joints[DIAGNOSTIC_JOINT_INDEX];
        for offset in [-0.0009, 0.0, 0.0009] {
            validate_diagnostic_start_position(
                joint,
                joint.limits.position_lower_rad + offset,
                LOW_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD,
            )
            .unwrap();
        }
        for offset in [-0.0011, 0.0011] {
            assert!(validate_diagnostic_start_position(
                joint,
                joint.limits.position_lower_rad + offset,
                LOW_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD,
            )
            .is_err());
        }

        for offset in [-0.0009, 0.001_153, 0.0019] {
            validate_diagnostic_start_position(
                joint,
                joint.limits.position_lower_rad + offset,
                HIGH_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD,
            )
            .unwrap();
        }
        for offset in [-0.0011, 0.0021] {
            assert!(validate_diagnostic_start_position(
                joint,
                joint.limits.position_lower_rad + offset,
                HIGH_TIER_DIAGNOSTIC_LOWER_BOUND_PROXIMITY_RAD,
            )
            .is_err());
        }
    }

    #[test]
    fn lowest_probe_is_quantized_differently_from_configured_initial_hold() {
        let profile = profile_with_velocity_limit(0.1);
        let dynamics = zero_gravity_dynamics();
        let q = [0.0; DOF];
        let limits = low_diagnostic_limits();
        let (initial, gravity) = build_diagnostic_hold_targets(
            &profile,
            &dynamics,
            &q,
            DIAGNOSTIC_JOINT_INDEX,
            0.0,
            0.0,
            limits,
        )
        .unwrap();
        let (probe, probe_gravity) = build_diagnostic_hold_targets(
            &profile,
            &dynamics,
            &q,
            DIAGNOSTIC_JOINT_INDEX,
            0.0,
            MIN_DIAGNOSTIC_TORQUE_STEP_NM,
            limits,
        )
        .unwrap();
        let (peak, _) = build_diagnostic_hold_targets(
            &profile,
            &dynamics,
            &q,
            DIAGNOSTIC_JOINT_INDEX,
            0.0,
            LOW_TIER_MAX_DIAGNOSTIC_ADDITIVE_TORQUE_NM,
            limits,
        )
        .unwrap();
        let joint = &profile.joints[DIAGNOSTIC_JOINT_INDEX];

        assert_eq!(gravity, 0.0);
        assert_eq!(probe_gravity, 0.0);
        assert_eq!(
            initial[DIAGNOSTIC_JOINT_INDEX].position_rev,
            probe[DIAGNOSTIC_JOINT_INDEX].position_rev
        );
        assert_eq!(
            initial[DIAGNOSTIC_JOINT_INDEX].kp_nm_rev,
            probe[DIAGNOSTIC_JOINT_INDEX].kp_nm_rev
        );
        assert_eq!(
            initial[DIAGNOSTIC_JOINT_INDEX].kd_nm_s_rev,
            probe[DIAGNOSTIC_JOINT_INDEX].kd_nm_s_rev
        );
        assert_ne!(
            compressed_target_words(&initial[DIAGNOSTIC_JOINT_INDEX], joint),
            compressed_target_words(&probe[DIAGNOSTIC_JOINT_INDEX], joint),
            "the first SDO readback target must differ bitwise from the value written during configure"
        );
        assert_ne!(
            compressed_target_words(&initial[DIAGNOSTIC_JOINT_INDEX], joint),
            compressed_target_words(&peak[DIAGNOSTIC_JOINT_INDEX], joint),
            "the peak SDO readback target must differ bitwise from the value written during configure"
        );
    }

    #[test]
    fn diagnostic_guards_both_temperatures_and_stops_at_small_breakaway() {
        let profile = profile_with_velocity_limit(0.1);
        let mut feedback = healthy_diagnostic_feedback();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let joint = &profile.joints[selected];
        let limits = low_diagnostic_limits();
        let target = ros_target_to_motor(
            RosTarget {
                position_rad: 0.0,
                velocity_rad_s: 0.0,
                torque_nm: 0.2,
                kp_nm_rad: joint.default_kp,
                kd_nm_s_rad: joint.default_kd,
            },
            joint,
        );
        feedback.joints[selected].position_rev = 0.0011 / TAU;
        let observation = diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &[0.0; DOF],
            DiagnosticTemperatureBaseline {
                driver_c: 35.0,
                motor_c: 36.0,
            },
            target,
            0.025,
            limits,
        )
        .unwrap();
        assert!(observation.breakaway_detected);

        feedback.joints[selected].position_rev = -0.0006 / TAU;
        let wrong_direction = diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &[0.0; DOF],
            DiagnosticTemperatureBaseline {
                driver_c: 35.0,
                motor_c: 36.0,
            },
            target,
            0.025,
            limits,
        )
        .unwrap_err()
        .to_string();
        assert!(wrong_direction.contains("opposite"));

        feedback.joints[selected].position_rev = 0.0;
        feedback.joints[selected].motor_temperature_c = 38.1;
        let error = diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &[0.0; DOF],
            DiagnosticTemperatureBaseline {
                driver_c: 35.0,
                motor_c: 36.0,
            },
            target,
            0.025,
            limits,
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("motor temperature rose"));
    }

    #[test]
    fn high_tier_enforces_absolute_position_and_two_point_five_nm_total_caps() {
        let profile = profile_with_velocity_limit(0.1);
        let dynamics = zero_gravity_dynamics();
        let limits = high_diagnostic_limits();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let joint = &profile.joints[selected];
        let mut q = [0.0; DOF];
        q[selected] = joint.limits.position_lower_rad + 0.002;

        let (allowed, _) = build_diagnostic_hold_targets(
            &profile,
            &dynamics,
            &q,
            selected,
            q[selected],
            1.75,
            limits,
        )
        .unwrap();
        assert!((motor_torque_to_ros(allowed[selected].torque_nm, joint) - 1.75).abs() < 1.0e-6);

        let mut feedback = healthy_diagnostic_feedback();
        feedback.joints[selected].position_rev = (joint.limits.position_lower_rad + 0.0041) / TAU;
        let absolute_error = diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &q,
            DiagnosticTemperatureBaseline {
                driver_c: 35.0,
                motor_c: 36.0,
            },
            allowed[selected],
            1.75,
            limits,
        )
        .unwrap_err()
        .to_string();
        assert!(absolute_error.contains("high-tier absolute position"));

        feedback.joints[selected].position_rev = q[selected] / TAU;
        feedback.joints[selected].torque_nm = 2.501;
        let measured_torque_error = diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &q,
            DiagnosticTemperatureBaseline {
                driver_c: 35.0,
                motor_c: 36.0,
            },
            allowed[selected],
            1.75,
            limits,
        )
        .unwrap_err()
        .to_string();
        assert!(measured_torque_error.contains("measured torque"));
    }

    #[test]
    fn position_round_trip_observation_enforces_position_velocity_torque_and_return_caps() {
        let profile = position_diagnostic_profile();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let joint = &profile.joints[selected];
        let initial = joint.limits.position_lower_rad + 0.001;
        let mut initial_q = [0.0; DOF];
        initial_q[selected] = initial;
        let baseline = DiagnosticTemperatureBaseline {
            driver_c: 35.0,
            motor_c: 36.0,
        };
        let limits = position_diagnostic_request(true).torque_limits();
        let target_for = |position_rad, torque_nm| {
            ros_target_to_motor(
                RosTarget {
                    position_rad,
                    velocity_rad_s: 0.0,
                    torque_nm,
                    kp_nm_rad: joint.default_kp,
                    kd_nm_s_rad: joint.default_kd,
                },
                joint,
            )
        };
        let target = target_for(initial + POSITION_DIAGNOSTIC_DELTA_RAD, 1.2);
        let mut feedback = healthy_diagnostic_feedback();
        feedback.joints[selected].position_rev = (initial + 0.003) / TAU;
        feedback.joints[selected].torque_nm = 1.4;
        let allowed = position_diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &initial_q,
            baseline,
            target,
            initial + POSITION_DIAGNOSTIC_DELTA_RAD,
            limits,
        )
        .unwrap();
        assert!((allowed.telemetry.measured_delta - 0.003).abs() < 1.0e-6);

        feedback.joints[selected].position_rev = (initial - 0.0006) / TAU;
        assert!(position_diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &initial_q,
            baseline,
            target,
            initial + POSITION_DIAGNOSTIC_DELTA_RAD,
            limits,
        )
        .unwrap_err()
        .to_string()
        .contains("opposite"));

        feedback.joints[selected].position_rev = (initial + 0.003) / TAU;
        feedback.joints[selected].velocity_rev_s = 0.021 / TAU;
        assert!(position_diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &initial_q,
            baseline,
            target,
            initial + POSITION_DIAGNOSTIC_DELTA_RAD,
            limits,
        )
        .is_err());

        feedback.joints[selected].velocity_rev_s = 0.0;
        feedback.joints[selected].torque_nm = 2.001;
        assert!(position_diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &initial_q,
            baseline,
            target,
            initial + POSITION_DIAGNOSTIC_DELTA_RAD,
            limits,
        )
        .is_err());

        feedback.joints[selected].torque_nm = 1.0;
        let excessive_ff = target_for(initial + POSITION_DIAGNOSTIC_DELTA_RAD, 1.351);
        assert!(position_diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &initial_q,
            baseline,
            excessive_ff,
            initial + POSITION_DIAGNOSTIC_DELTA_RAD,
            limits,
        )
        .unwrap_err()
        .to_string()
        .contains("feed-forward"));

        feedback.joints[selected].position_rev = (initial + 0.001) / TAU;
        let returned = position_diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &initial_q,
            baseline,
            target_for(initial, 1.2),
            initial,
            limits,
        )
        .unwrap();
        validate_position_return_observation(returned).unwrap();
        feedback.joints[selected].velocity_rev_s = 0.0101 / TAU;
        let moving = position_diagnostic_observation(
            &profile,
            &feedback,
            selected,
            &initial_q,
            baseline,
            target_for(initial, 1.2),
            initial,
            limits,
        )
        .unwrap();
        assert!(validate_position_return_observation(moving).is_err());
    }

    #[test]
    fn position_round_trip_peak_changes_the_compressed_position_field() {
        let profile = position_diagnostic_profile();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let joint = &profile.joints[selected];
        let initial = joint.limits.position_lower_rad + 0.001;
        let baseline = ros_target_to_motor(
            RosTarget {
                position_rad: initial,
                velocity_rad_s: 0.0,
                torque_nm: 1.2,
                kp_nm_rad: joint.default_kp,
                kd_nm_s_rad: joint.default_kd,
            },
            joint,
        );
        let peak = ros_target_to_motor(
            RosTarget {
                position_rad: initial + POSITION_DIAGNOSTIC_DELTA_RAD,
                velocity_rad_s: 0.0,
                torque_nm: 1.2,
                kp_nm_rad: joint.default_kp,
                kd_nm_s_rad: joint.default_kd,
            },
            joint,
        );
        assert_ne!(
            compressed_target_position_code(&baseline, joint),
            compressed_target_position_code(&peak, joint)
        );
    }

    #[test]
    fn high_tier_breakaway_and_error_use_immediate_baseline_restore_only() {
        let high = high_diagnostic_limits();
        let low = low_diagnostic_limits();
        assert!(requires_fast_diagnostic_restore(
            high,
            &Ok(DiagnosticLevelOutcome::BreakawayDetected)
        ));
        assert!(requires_fast_diagnostic_restore(
            high,
            &Err(anyhow::anyhow!("injected guard failure"))
        ));
        assert!(!requires_fast_diagnostic_restore(
            high,
            &Ok(DiagnosticLevelOutcome::Completed)
        ));
        assert!(!requires_fast_diagnostic_restore(
            low,
            &Ok(DiagnosticLevelOutcome::BreakawayDetected)
        ));
    }

    #[test]
    fn high_tier_enable_stability_uses_the_same_position_velocity_and_temperature_envelope() {
        let profile = profile_with_velocity_limit(0.1);
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let lower = profile.joints[selected].limits.position_lower_rad;
        let mut feedback = healthy_diagnostic_feedback();
        let initial = lower + 0.001_153;
        let guards = DiagnosticStabilityGuards {
            torque_limits: high_diagnostic_limits(),
            temperature_baseline: DiagnosticTemperatureBaseline {
                driver_c: 35.0,
                motor_c: 36.0,
            },
            active_started: Instant::now(),
            position_round_trip: false,
            censored_gravity_hold: false,
            selected_gravity_scale: None,
        };
        validate_diagnostic_enable_stability_guards(
            &profile, &feedback, selected, initial, initial, guards,
        )
        .unwrap();

        let absolute = validate_diagnostic_enable_stability_guards(
            &profile,
            &feedback,
            selected,
            initial,
            lower + 0.0041,
            guards,
        )
        .unwrap_err()
        .to_string();
        assert!(absolute.contains("high-tier absolute position"));

        let opposite = validate_diagnostic_enable_stability_guards(
            &profile,
            &feedback,
            selected,
            initial,
            initial - 0.0006,
            guards,
        )
        .unwrap_err()
        .to_string();
        assert!(opposite.contains("opposite"));

        feedback.joints[selected].velocity_rev_s = 0.021 / TAU;
        let velocity = validate_diagnostic_enable_stability_guards(
            &profile, &feedback, selected, initial, initial, guards,
        )
        .unwrap_err()
        .to_string();
        assert!(velocity.contains("velocity"));
    }

    #[test]
    fn position_round_trip_uses_its_half_milliradian_and_point_zero_one_stability_gate() {
        let profile = position_diagnostic_profile();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let lower = profile.joints[selected].limits.position_lower_rad;
        let feedback = healthy_diagnostic_feedback();
        let guards = DiagnosticStabilityGuards {
            torque_limits: position_diagnostic_request(true).torque_limits(),
            temperature_baseline: DiagnosticTemperatureBaseline {
                driver_c: 35.0,
                motor_c: 36.0,
            },
            active_started: Instant::now(),
            position_round_trip: true,
            censored_gravity_hold: false,
            selected_gravity_scale: Some(POSITION_DIAGNOSTIC_ENABLE_GRAVITY_SCALE),
        };
        assert_eq!(
            enable_stability_limits(Some(guards)),
            (
                POSITION_DIAGNOSTIC_STABILITY_POSITION_RAD,
                POSITION_DIAGNOSTIC_STABILITY_VELOCITY_RAD_S,
            )
        );
        assert_eq!(
            enable_stability_limits(None),
            (ENABLE_STABILITY_POSITION_RAD, STABLE_VELOCITY_RAD_S)
        );

        let initial = lower + 0.002;
        validate_diagnostic_enable_stability_guards(
            &profile,
            &feedback,
            selected,
            initial,
            lower + POSITION_DIAGNOSTIC_GRAVITY_RAMP_ABSOLUTE_UPPER_OFFSET_RAD - 1.0e-6,
            guards,
        )
        .unwrap();
        assert!(validate_diagnostic_enable_stability_guards(
            &profile,
            &feedback,
            selected,
            initial,
            initial + POSITION_DIAGNOSTIC_GRAVITY_RAMP_MAX_POSITIVE_RAD + 1.0e-6,
            guards,
        )
        .is_err());
        assert!(validate_diagnostic_enable_stability_guards(
            &profile,
            &feedback,
            selected,
            initial,
            initial - POSITION_DIAGNOSTIC_MAX_OPPOSITE_POSITION_RAD - 1.0e-6,
            guards,
        )
        .is_err());
    }

    #[test]
    fn censored_gravity_hold_enforces_identification_layer_during_enable_stability() {
        let profile = censored_gravity_hold_profile();
        let selected = DIAGNOSTIC_JOINT_INDEX;
        let lower = profile.joints[selected].limits.position_lower_rad;
        let initial = lower + 0.001;
        let mut feedback = healthy_diagnostic_feedback();
        let guards = DiagnosticStabilityGuards {
            torque_limits: censored_gravity_hold_request(true, true).torque_limits(),
            temperature_baseline: DiagnosticTemperatureBaseline {
                driver_c: 35.0,
                motor_c: 36.0,
            },
            active_started: Instant::now(),
            position_round_trip: true,
            censored_gravity_hold: true,
            selected_gravity_scale: Some(CENSORED_GRAVITY_HOLD_PROFILE_SCALE),
        };
        assert_eq!(
            enable_stability_limits(Some(guards)),
            (
                CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD,
                CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S,
            )
        );

        validate_diagnostic_enable_stability_guards(
            &profile,
            &feedback,
            selected,
            initial,
            initial + CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD - 1.0e-7,
            guards,
        )
        .unwrap();
        let position_error = validate_diagnostic_enable_stability_guards(
            &profile,
            &feedback,
            selected,
            initial,
            initial + CENSORED_GRAVITY_HOLD_IDENTIFICATION_POSITION_RAD,
            guards,
        )
        .unwrap_err()
        .to_string();
        assert!(position_error.contains("identification exit layer"));

        feedback.joints[selected].velocity_rev_s =
            CENSORED_GRAVITY_HOLD_IDENTIFICATION_VELOCITY_RAD_S / TAU;
        let velocity_error = validate_diagnostic_enable_stability_guards(
            &profile, &feedback, selected, initial, initial, guards,
        )
        .unwrap_err()
        .to_string();
        assert!(velocity_error.contains("identification exit layer"));
    }

    #[test]
    fn commissioning_gains_use_reviewed_profile_values_and_reject_excess() {
        let mut profile = profile_with_velocity_limit(0.1);
        profile.joints[5].default_kp = 10.0;
        profile.joints[5].default_kd = 0.5;
        validate_commissioning_gains(&profile).unwrap();

        let target = ros_target_to_motor(
            RosTarget {
                position_rad: 0.0,
                velocity_rad_s: 0.0,
                torque_nm: 0.0,
                kp_nm_rad: profile.joints[5].default_kp,
                kd_nm_s_rad: profile.joints[5].default_kd,
            },
            &profile.joints[5],
        );
        assert!((target.kp_nm_rev / TAU - 10.0).abs() < 1.0e-6);
        assert!((target.kd_nm_s_rev / TAU - 0.5).abs() < 1.0e-6);

        profile.joints[5].default_kp = COMMISSION_KP_HARD_MAX_NM_RAD;
        profile.joints[5].default_kd = COMMISSION_KD_HARD_MAX_NM_S_RAD;
        validate_commissioning_gains(&profile)
            .expect("the evidence-backed hard gain boundaries must be accepted");

        profile.joints[5].default_kp = COMMISSION_KP_HARD_MAX_NM_RAD + 0.1;
        let error = validate_commissioning_gains(&profile)
            .unwrap_err()
            .to_string();
        assert!(error.contains("joint_6 commissioning Kp"));

        profile.joints[5].default_kp = COMMISSION_KP_HARD_MAX_NM_RAD;
        profile.joints[5].default_kd = COMMISSION_KD_HARD_MAX_NM_S_RAD + 0.1;
        let error = validate_commissioning_gains(&profile)
            .unwrap_err()
            .to_string();
        assert!(error.contains("joint_6 commissioning Kd"));
    }

    #[test]
    fn continuous_stability_gate_resets_the_entire_dwell_window() {
        let mut gate =
            ContinuousStabilityGate::new(Duration::from_millis(250), Duration::from_secs(2));

        assert_eq!(
            gate.observe(Duration::ZERO, true),
            StabilityGateStatus::Waiting
        );
        assert_eq!(
            gate.observe(Duration::from_millis(249), true),
            StabilityGateStatus::Waiting
        );
        assert_eq!(
            gate.observe(Duration::from_millis(250), false),
            StabilityGateStatus::Waiting
        );
        assert_eq!(
            gate.observe(Duration::from_millis(499), true),
            StabilityGateStatus::Waiting
        );
        assert_eq!(
            gate.observe(Duration::from_millis(748), true),
            StabilityGateStatus::Waiting
        );
        assert_eq!(
            gate.observe(Duration::from_millis(749), true),
            StabilityGateStatus::Stable
        );
    }

    #[test]
    fn continuous_stability_gate_times_out_when_dwell_is_incomplete() {
        let mut gate =
            ContinuousStabilityGate::new(Duration::from_millis(250), Duration::from_secs(1));

        assert_eq!(
            gate.observe(Duration::from_millis(800), true),
            StabilityGateStatus::Waiting
        );
        assert_eq!(
            gate.observe(Duration::from_millis(999), true),
            StabilityGateStatus::Waiting
        );
        assert_eq!(
            gate.observe(Duration::from_secs(1), true),
            StabilityGateStatus::TimedOut
        );

        let mut late_gate =
            ContinuousStabilityGate::new(Duration::from_millis(250), Duration::from_secs(1));
        assert_eq!(
            late_gate.observe(Duration::from_millis(700), true),
            StabilityGateStatus::Waiting
        );
        assert_eq!(
            late_gate.observe(Duration::from_millis(1_100), true),
            StabilityGateStatus::TimedOut,
            "a late sample must not succeed after the phase deadline"
        );
    }

    #[test]
    fn symmetric_enable_drift_guard_accepts_both_boundaries_and_rejects_both_sides() {
        validate_symmetric_enable_drift(0.0, ENABLE_STABILITY_POSITION_RAD).unwrap();
        validate_symmetric_enable_drift(0.0, -ENABLE_STABILITY_POSITION_RAD).unwrap();

        let excess = ENABLE_STABILITY_POSITION_RAD + 1.0e-6;
        for measured in [excess, -excess] {
            let error = validate_symmetric_enable_drift(0.0, measured)
                .unwrap_err()
                .to_string();
            assert!(error.contains("exceeds symmetric commissioning guard"));
        }
        assert!(validate_symmetric_enable_drift(0.0, f32::NAN).is_err());
    }

    #[test]
    fn safe_hold_scales_each_joint_gravity_feedforward() {
        let mut profile = profile_with_velocity_limit(0.1);
        profile.gravity_vector_base_m_s2 = [0.0, 0.0, 9.81];
        profile.joints[0].gravity_compensation_scale = 0.25;
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let dynamics = hex_arm_dynamics::ArmDynamics::from_parts(
            vec![([0.0; 3], identity, [0.0, 1.0, 0.0]); DOF],
            vec![
                (0.2, [1.0, 0.0, 0.0]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
            ],
            [0.0, 0.0, -9.81],
        );

        let targets = build_safe_hold_targets(&profile, &dynamics, &[0.0; DOF], 0, 0.0)
            .expect("scaled gravity target should remain inside the test torque limit");

        // Commissioning uses the profile's +Z vector instead of the dynamics
        // object's -Z default, then applies the selected axis scale.
        assert!((targets[0].torque_nm - (1.962 * 0.25)).abs() < 1.0e-5);
        assert!(targets[1..]
            .iter()
            .all(|target| target.torque_nm.abs() < 1.0e-6));
    }

    #[test]
    fn passive_measurement_margin_clamps_only_nonselected_dormant_targets() {
        let mut profile = profile_with_velocity_limit(0.1);
        let passive = 1;
        profile.joints[passive].limits.measured_position_margin_rad = 0.001;
        let mut measured_q = [0.0; DOF];
        measured_q[passive] = profile.joints[passive].limits.position_upper_rad + 0.0008;
        let dynamics = zero_gravity_dynamics();

        let targets = build_safe_hold_targets(&profile, &dynamics, &measured_q, 0, measured_q[0])
            .expect("a nonselected, non-torque axis may use a clamped dormant command target");
        let passive_target =
            motor_position_to_ros(targets[passive].position_rev, &profile.joints[passive]);
        assert!(
            (passive_target - profile.joints[passive].limits.position_upper_rad).abs() < 1.0e-6
        );

        let error = build_safe_hold_targets(
            &profile,
            &dynamics,
            &measured_q,
            passive,
            measured_q[passive],
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("joint_2 commissioning target"));
        assert!(error.contains("exceeds position limit"));
    }

    #[test]
    fn request_accepts_uncalibrated_but_validated_profile_and_safe_motion() {
        let profile = profile_with_velocity_limit(0.1);
        assert!(!profile.calibrated);
        CommissioningRequest {
            selected_index: 2,
            delta_rad: -0.02,
            duration_sec: 2.0,
        }
        .validate(&profile)
        .unwrap();
    }

    #[test]
    fn request_rejects_excess_delta_and_too_fast_smooth_round_trip() {
        let profile = profile_with_velocity_limit(0.1);
        assert!(CommissioningRequest {
            selected_index: 0,
            delta_rad: 0.031,
            duration_sec: 2.0,
        }
        .validate(&profile)
        .is_err());
        let error = CommissioningRequest {
            selected_index: 0,
            delta_rad: 0.03,
            duration_sec: 0.5,
        }
        .validate(&profile)
        .unwrap_err();
        assert!(error.to_string().contains("peak velocity"));

        let error = CommissioningRequest {
            selected_index: 0,
            delta_rad: 0.01,
            duration_sec: 1.0,
        }
        .validate(&profile)
        .unwrap_err();
        assert!(error.to_string().contains("peak acceleration"));
    }

    #[test]
    fn round_trip_phase_starts_returns_and_reaches_endpoint_only_midway() {
        assert!(round_trip_phase(0.0).abs() < 1.0e-6);
        assert!((round_trip_phase(0.5) - 1.0).abs() < 1.0e-6);
        assert!(round_trip_phase(1.0).abs() < 1.0e-6);
        assert!(round_trip_phase(0.25) > 0.0 && round_trip_phase(0.25) < 1.0);
    }

    #[test]
    fn joint1_compensation_is_directional_bounded_and_zero_at_stationary_points() {
        for t in [0.0, 0.5, 1.0] {
            assert!(joint1_first_position_friction_compensation(t).abs() < 1.0e-6);
        }
        assert!(
            (joint1_first_position_friction_compensation(0.25)
                - J1_FIRST_POSITION_POSITIVE_COMPENSATION_NM)
                .abs()
                < 1.0e-6
        );
        assert!(
            (joint1_first_position_friction_compensation(0.75)
                + J1_FIRST_POSITION_NEGATIVE_COMPENSATION_NM)
                .abs()
                < 1.0e-6
        );
        for step in 0..=1_000 {
            let compensation = joint1_first_position_friction_compensation(step as f32 / 1_000.0);
            assert!((-J1_FIRST_POSITION_NEGATIVE_COMPENSATION_NM
                ..=J1_FIRST_POSITION_POSITIVE_COMPENSATION_NM)
                .contains(&compensation));
        }
    }

    #[test]
    fn joint1_compensated_target_uses_temporary_gain_without_mutating_profile_policy() {
        let profile = joint1_first_position_profile();
        let joint = &profile.joints[J1_FIRST_POSITION_INDEX];
        assert_eq!(joint.default_kp, J1_FIRST_POSITION_EXPECTED_KP);
        let q = [0.0; DOF];
        let (outbound, model, compensation) = build_joint1_first_position_trajectory_targets(
            &profile,
            &zero_gravity_dynamics(),
            &q,
            J1_FIRST_POSITION_DELTA_RAD * 0.5,
            0.25,
        )
        .unwrap();
        assert!(model.abs() < 1.0e-6);
        assert!((compensation - J1_FIRST_POSITION_POSITIVE_COMPENSATION_NM).abs() < 1.0e-6);
        assert!(
            (motor_kp_to_ros(outbound[J1_FIRST_POSITION_INDEX].kp_nm_rev, joint)
                - J1_FIRST_POSITION_TRAJECTORY_KP)
                .abs()
                < 1.0e-4
        );
        assert!(
            (motor_kd_to_ros(outbound[J1_FIRST_POSITION_INDEX].kd_nm_s_rev, joint)
                - J1_FIRST_POSITION_TRAJECTORY_KD)
                .abs()
                < 1.0e-4
        );
        assert!(
            (motor_torque_to_ros(outbound[J1_FIRST_POSITION_INDEX].torque_nm, joint)
                - J1_FIRST_POSITION_POSITIVE_COMPENSATION_NM)
                .abs()
                < 1.0e-6
        );
        let (returning, _, return_compensation) = build_joint1_first_position_trajectory_targets(
            &profile,
            &zero_gravity_dynamics(),
            &q,
            J1_FIRST_POSITION_DELTA_RAD * 0.5,
            0.75,
        )
        .unwrap();
        assert!((return_compensation + J1_FIRST_POSITION_NEGATIVE_COMPENSATION_NM).abs() < 1.0e-6);
        assert!(
            (motor_torque_to_ros(returning[J1_FIRST_POSITION_INDEX].torque_nm, joint)
                + J1_FIRST_POSITION_NEGATIVE_COMPENSATION_NM)
                .abs()
                < 1.0e-6
        );
        assert_eq!(profile.joints[J1_FIRST_POSITION_INDEX].default_kp, 60.0);
    }

    #[test]
    fn return_tolerance_scales_with_motion_and_stays_strict() {
        assert!((return_tolerance_rad(0.01) - 0.0025).abs() < 1.0e-6);
        assert!((return_tolerance_rad(-0.03) - 0.005).abs() < 1.0e-6);
        assert!((return_tolerance_rad(0.001) - 0.001).abs() < 1.0e-6);
    }

    #[test]
    fn motion_telemetry_milestones_emit_once_even_when_a_sample_crosses_several() {
        let mut milestones = MotionTelemetryMilestones::default();
        assert_eq!(milestones.take_due(0.249), None);
        assert_eq!(milestones.take_due(0.250), Some("quarter"));
        assert_eq!(milestones.take_due(0.250), None);

        // A delayed iteration may cross both remaining thresholds. Repeated
        // calls for that one sample recover both labels, but never repeat them.
        assert_eq!(milestones.take_due(0.80), Some("peak"));
        assert_eq!(milestones.take_due(0.80), Some("three_quarter"));
        assert_eq!(milestones.take_due(0.80), None);
        assert_eq!(milestones.take_due(1.0), None);
    }

    #[test]
    fn telemetry_sample_reports_ros_units_and_estimated_pd_torque() {
        let mut profile = profile_with_velocity_limit(0.1);
        // J2 deliberately has a negative direction and a non-unit torque
        // scale, so this catches both gain-domain and sign regressions.
        let selected_index = DIAGNOSTIC_JOINT_INDEX;
        profile.joints[selected_index].direction = -1;
        profile.joints[selected_index].torque_scale = 0.85;
        let joint = &profile.joints[selected_index];
        let measured_motor = ros_target_to_motor(
            RosTarget {
                position_rad: 0.12,
                velocity_rad_s: 0.02,
                torque_nm: 0.30,
                kp_nm_rad: 0.0,
                kd_nm_s_rad: 0.0,
            },
            joint,
        );
        let target = ros_target_to_motor(
            RosTarget {
                position_rad: 0.15,
                velocity_rad_s: 0.0,
                torque_nm: 0.40,
                kp_nm_rad: 2.0,
                kd_nm_s_rad: 0.3,
            },
            joint,
        );
        let mut feedback = FeedbackSnapshot::default();
        feedback.joints[selected_index].position_rev = measured_motor.position_rev;
        feedback.joints[selected_index].velocity_rev_s = measured_motor.velocity_rev_s;
        feedback.joints[selected_index].torque_nm = measured_motor.torque_nm;

        let sample = commissioning_telemetry_sample(
            joint,
            &feedback,
            selected_index,
            0.10,
            0.12,
            0.15,
            target,
        );
        assert!((sample.commanded_q - 0.15).abs() < 1.0e-6);
        assert!((sample.measured_q - 0.12).abs() < 1.0e-6);
        assert!((sample.measured_delta - 0.02).abs() < 1.0e-6);
        assert!((sample.measured_velocity - 0.02).abs() < 1.0e-6);
        assert!((sample.measured_torque - 0.30).abs() < 1.0e-6);
        assert!((sample.gravity_ff - 0.40).abs() < 1.0e-6);
        assert!((sample.kp - 2.0).abs() < 1.0e-6);
        assert!((sample.kd - 0.3).abs() < 1.0e-6);
        // 2*(0.15-0.12) + 0.3*(0.0-0.02) = 0.054 Nm.
        assert!((sample.estimated_pd_torque - 0.054).abs() < 1.0e-6);
    }

    #[test]
    fn safety_abort_telemetry_keeps_available_fields_when_safe_target_build_fails() {
        let mut profile = profile_with_velocity_limit(0.1);
        profile.gravity_vector_base_m_s2 = [0.0, 0.0, 9.81];
        profile.joints[0].limits.torque_nm = 0.1;
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let dynamics = hex_arm_dynamics::ArmDynamics::from_parts(
            vec![([0.0; 3], identity, [0.0, 1.0, 0.0]); DOF],
            vec![
                (0.2, [1.0, 0.0, 0.0]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
                (0.0, [0.0; 3]),
            ],
            [0.0, 0.0, -9.81],
        );
        let mut feedback = FeedbackSnapshot::default();
        feedback.joints[0].velocity_rev_s = 0.01;
        feedback.joints[0].torque_nm = 0.2;

        let abort = safety_abort_telemetry_sample(&profile, &dynamics, &feedback, 0, 0.0, 0.01);

        assert!(!abort.safe_target_built);
        assert!(abort
            .target_build_error
            .as_deref()
            .is_some_and(|error| error.contains("gravity feedforward")));
        assert!((abort.telemetry.commanded_q - 0.01).abs() < 1.0e-6);
        assert!(abort.telemetry.measured_q.abs() < 1.0e-6);
        assert!((abort.telemetry.measured_velocity - TAU * 0.01).abs() < 1.0e-6);
        assert!((abort.telemetry.measured_torque - 0.2).abs() < 1.0e-6);
        assert!((abort.telemetry.gravity_ff - 1.962).abs() < 1.0e-5);
        assert!(abort.telemetry.estimated_pd_torque.is_finite());
        assert_eq!(abort.raw_motor_velocity_rev_s, 0.01);
        assert_eq!(abort.raw_motor_torque_nm, 0.2);
    }

    #[test]
    fn measured_excursion_accepts_positive_and_negative_requested_directions() {
        let mut positive = MeasuredExcursion::new(1.0, 0.02);
        for position in [1.004, 1.012, 1.006, 1.0] {
            positive.observe(position).unwrap();
        }
        assert!((positive.validate_completed().unwrap() - 0.012).abs() < 1.0e-6);

        let mut negative = MeasuredExcursion::new(1.0, -0.02);
        for position in [0.996, 0.988, 0.994, 1.0] {
            negative.observe(position).unwrap();
        }
        assert!((negative.validate_completed().unwrap() + 0.012).abs() < 1.0e-6);
    }

    #[test]
    fn measured_excursion_rejects_no_motion_wrong_direction_and_overshoot() {
        let mut stationary = MeasuredExcursion::new(0.0, 0.02);
        stationary.observe(0.0).unwrap();
        let error = stationary.validate_completed().unwrap_err().to_string();
        assert!(error.contains("max positive excursion 0.000000 rad"));
        assert!(error.contains("max negative excursion 0.000000 rad"));

        let mut wrong_direction = MeasuredExcursion::new(0.0, 0.02);
        assert!(wrong_direction.observe(-0.004).is_err());

        let mut overshoot = MeasuredExcursion::new(0.0, -0.02);
        assert!(overshoot.observe(-0.026).is_err());
    }

    #[test]
    fn safety_abort_hook_runs_once_and_preserves_the_original_error() {
        let mut direct = MeasuredExcursion::new(0.0, 0.02);
        let expected_error = direct.observe(-0.004).unwrap_err().to_string();

        let mut guarded = MeasuredExcursion::new(0.0, 0.02);
        let mut abort_calls = 0;
        let actual_error = result_with_safety_abort_hook(guarded.observe(-0.004), |error| {
            abort_calls += 1;
            assert_eq!(error.to_string(), expected_error);
        })
        .unwrap_err()
        .to_string();

        assert_eq!(abort_calls, 1);
        assert_eq!(actual_error, expected_error);

        let mut accepted = MeasuredExcursion::new(0.0, 0.02);
        let mut unexpected_abort_calls = 0;
        result_with_safety_abort_hook(accepted.observe(0.004), |_| {
            unexpected_abort_calls += 1;
        })
        .unwrap();
        assert_eq!(unexpected_abort_calls, 0);
    }
}
