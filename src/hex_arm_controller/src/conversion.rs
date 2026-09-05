use std::f32::consts::TAU;

use crate::profile::JointProfile;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RosTarget {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub torque_nm: f32,
    pub kp_nm_rad: f32,
    pub kd_nm_s_rad: f32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MotorTarget {
    pub position_rev: f32,
    pub velocity_rev_s: f32,
    pub torque_nm: f32,
    pub kp_nm_rev: f32,
    pub kd_nm_s_rev: f32,
}

pub fn motor_position_to_ros(position_rev: f32, joint: &JointProfile) -> f32 {
    joint.direction as f32 * TAU * position_rev + joint.zero_offset_rad
}

pub fn motor_velocity_to_ros(velocity_rev_s: f32, joint: &JointProfile) -> f32 {
    joint.direction as f32 * TAU * velocity_rev_s
}

pub fn motor_torque_to_ros(torque_nm: f32, joint: &JointProfile) -> f32 {
    joint.direction as f32 * torque_nm / joint.torque_scale
}

pub fn motor_kp_to_ros(kp_nm_rev: f32, joint: &JointProfile) -> f32 {
    kp_nm_rev / (TAU * joint.torque_scale)
}

pub fn motor_kd_to_ros(kd_nm_s_rev: f32, joint: &JointProfile) -> f32 {
    kd_nm_s_rev / (TAU * joint.torque_scale)
}

pub fn ros_target_to_motor(target: RosTarget, joint: &JointProfile) -> MotorTarget {
    let direction = joint.direction as f32;
    let torque_scale = joint.torque_scale;
    MotorTarget {
        position_rev: direction * (target.position_rad - joint.zero_offset_rad) / TAU,
        velocity_rev_s: direction * target.velocity_rad_s / TAU,
        torque_nm: direction * target.torque_nm * torque_scale,
        // Firmware sums feed-forward and PD in the same motor-side Nm domain,
        // so every torque-producing coefficient uses the same calibration.
        kp_nm_rev: target.kp_nm_rad * TAU * torque_scale,
        kd_nm_s_rev: target.kd_nm_s_rad * TAU * torque_scale,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::{IdentityFingerprint, JointLimits};

    fn joint(direction: i8) -> JointProfile {
        JointProfile {
            name: "joint_1".into(),
            node_id: 1,
            identity: IdentityFingerprint::test_value(),
            direction,
            zero_offset_rad: 0.25,
            torque_scale: 0.8,
            gravity_compensation_scale: 1.0,
            torque_permille: 250,
            kp_kd_torque_permille: 250,
            limits: JointLimits {
                position_lower_rad: -2.0,
                position_upper_rad: 2.0,
                measured_position_margin_rad: 0.0,
                velocity_rad_s: 3.0,
                acceleration_rad_s2: 4.0,
                torque_nm: 4.0,
            },
            default_kp: 10.0,
            default_kd: 1.5,
        }
    }

    #[test]
    fn target_conversion_closes_in_ros_units_for_both_directions() {
        for direction in [-1, 1] {
            let joint = joint(direction);
            let target = RosTarget {
                position_rad: 1.1,
                velocity_rad_s: -0.7,
                torque_nm: 2.0,
                kp_nm_rad: 5.0,
                kd_nm_s_rad: 0.4,
            };
            let motor = ros_target_to_motor(target, &joint);
            assert!(
                (motor_position_to_ros(motor.position_rev, &joint) - target.position_rad).abs()
                    < 1.0e-6
            );
            assert!(
                (motor_velocity_to_ros(motor.velocity_rev_s, &joint) - target.velocity_rad_s).abs()
                    < 1.0e-6
            );
            assert!(
                (motor_torque_to_ros(motor.torque_nm, &joint) - target.torque_nm).abs() < 1.0e-6
            );
            assert!((motor.kp_nm_rev - target.kp_nm_rad * TAU * joint.torque_scale).abs() < 1.0e-6);
            assert!(
                (motor.kd_nm_s_rev - target.kd_nm_s_rad * TAU * joint.torque_scale).abs() < 1.0e-6
            );
            assert!((motor_kp_to_ros(motor.kp_nm_rev, &joint) - target.kp_nm_rad).abs() < 1.0e-6);
            assert!(
                (motor_kd_to_ros(motor.kd_nm_s_rev, &joint) - target.kd_nm_s_rad).abs() < 1.0e-6
            );

            let measured_position_rad = 0.7;
            let measured_velocity_rad_s = 0.2;
            let measured_position_rev =
                direction as f32 * (measured_position_rad - joint.zero_offset_rad) / TAU;
            let measured_velocity_rev_s = direction as f32 * measured_velocity_rad_s / TAU;
            let motor_pd_torque = motor.kp_nm_rev * (motor.position_rev - measured_position_rev)
                + motor.kd_nm_s_rev * (motor.velocity_rev_s - measured_velocity_rev_s);
            let expected_ros_pd_torque = target.kp_nm_rad
                * (target.position_rad - measured_position_rad)
                + target.kd_nm_s_rad * (target.velocity_rad_s - measured_velocity_rad_s);
            assert!(
                (motor_torque_to_ros(motor_pd_torque, &joint) - expected_ros_pd_torque).abs()
                    < 1.0e-5
            );
        }
    }
}
