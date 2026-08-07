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

pub fn ros_target_to_motor(target: RosTarget, joint: &JointProfile) -> MotorTarget {
    let direction = joint.direction as f32;
    MotorTarget {
        position_rev: direction * (target.position_rad - joint.zero_offset_rad) / TAU,
        velocity_rev_s: direction * target.velocity_rad_s / TAU,
        torque_nm: direction * target.torque_nm * joint.torque_scale,
        kp_nm_rev: target.kp_nm_rad * TAU,
        kd_nm_s_rev: target.kd_nm_s_rad * TAU,
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
            torque_permille: 250,
            kp_kd_torque_permille: 250,
            limits: JointLimits {
                position_lower_rad: -2.0,
                position_upper_rad: 2.0,
                velocity_rad_s: 3.0,
                torque_nm: 4.0,
            },
            default_kp: 10.0,
            default_kd: 1.5,
        }
    }

    #[test]
    fn position_round_trip_both_directions() {
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
            assert!((motor.kp_nm_rev - target.kp_nm_rad * TAU).abs() < 1.0e-6);
        }
    }
}
