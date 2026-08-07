use std::collections::HashSet;
use std::f32::consts::TAU;
use std::path::Path;
use std::time::Duration;

use anyhow::{Context, Result};
use hex_motor::cia402::CompressedMitMapping;
use serde::Deserialize;

pub const JOINT_NAMES: [&str; 6] = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
];

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareProfile {
    pub schema_version: u32,
    pub validated: bool,
    pub calibrated: bool,
    pub robot_prefix: String,
    pub urdf_path: String,
    pub bus: BusProfile,
    pub controller: ControllerProfile,
    pub joints: Vec<JointProfile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusProfile {
    pub channel: u16,
    pub adapter_vid: u16,
    pub adapter_pid: u16,
    pub heartbeat_node_id: u8,
    pub hardware_timestamp: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerProfile {
    pub loop_hz: u32,
    pub state_publish_hz: u32,
    pub discovery_timeout_ms: u64,
    pub feedback_timeout_ms: u64,
    pub command_watchdog_ms: u64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointProfile {
    pub name: String,
    pub node_id: u8,
    pub identity: IdentityFingerprint,
    pub direction: i8,
    pub zero_offset_rad: f32,
    pub torque_scale: f32,
    pub torque_permille: u16,
    pub kp_kd_torque_permille: u16,
    pub limits: JointLimits,
    pub default_kp: f32,
    pub default_kd: f32,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IdentityFingerprint {
    pub vendor_id: u32,
    pub product_code: u32,
    pub revision: u32,
    pub serial_number: u32,
    pub model: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointLimits {
    pub position_lower_rad: f32,
    pub position_upper_rad: f32,
    pub velocity_rad_s: f32,
    pub torque_nm: f32,
}

impl HardwareProfile {
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let contents = std::fs::read_to_string(path)
            .with_context(|| format!("read hardware profile {}", path.display()))?;
        let profile: Self = serde_yaml::from_str(&contents)
            .with_context(|| format!("parse hardware profile {}", path.display()))?;
        profile.validate()?;
        Ok(profile)
    }

    pub fn validate(&self) -> Result<()> {
        anyhow::ensure!(
            self.schema_version == 1,
            "unsupported hardware profile schema_version"
        );
        anyhow::ensure!(self.validated, "profile is not marked validated");
        anyhow::ensure!(
            !self.robot_prefix.trim_matches('/').is_empty(),
            "robot_prefix is empty"
        );
        anyhow::ensure!(!self.urdf_path.is_empty(), "urdf_path is required");
        anyhow::ensure!(
            Path::new(&self.urdf_path).is_file(),
            "urdf_path does not exist"
        );
        anyhow::ensure!(
            (1..=127).contains(&self.bus.heartbeat_node_id),
            "invalid host heartbeat node id"
        );
        anyhow::ensure!(
            self.bus.adapter_vid != 0 && self.bus.adapter_pid != 0,
            "USB VID/PID are required"
        );
        anyhow::ensure!(
            self.controller.loop_hz == 1000,
            "motor loop must be configured at 1000 Hz"
        );
        anyhow::ensure!(
            self.controller.state_publish_hz > 0 && self.controller.state_publish_hz <= 200,
            "invalid state publish rate"
        );
        anyhow::ensure!(
            self.controller.discovery_timeout_ms >= 500,
            "discovery timeout is too short"
        );
        anyhow::ensure!(
            (10..=500).contains(&self.controller.feedback_timeout_ms),
            "feedback timeout outside 10..500 ms"
        );
        anyhow::ensure!(
            (20..=1000).contains(&self.controller.command_watchdog_ms),
            "command watchdog outside 20..1000 ms"
        );
        anyhow::ensure!(
            self.joints.len() == 6,
            "exactly six joint entries are required"
        );

        let mut nodes = HashSet::new();
        for (index, joint) in self.joints.iter().enumerate() {
            anyhow::ensure!(
                joint.name == JOINT_NAMES[index],
                "joints must use canonical joint_1..joint_6 order"
            );
            anyhow::ensure!(
                (1..=127).contains(&joint.node_id),
                "{} has invalid node id",
                joint.name
            );
            anyhow::ensure!(
                joint.node_id != self.bus.heartbeat_node_id,
                "{} conflicts with host heartbeat node",
                joint.name
            );
            anyhow::ensure!(
                nodes.insert(joint.node_id),
                "duplicate CANopen node id {}",
                joint.node_id
            );
            anyhow::ensure!(
                joint.identity.vendor_id != 0 && joint.identity.product_code != 0,
                "{} identity fingerprint is incomplete",
                joint.name
            );
            anyhow::ensure!(
                joint.identity.revision != 0 && joint.identity.serial_number != 0,
                "{} revision/serial fingerprint is incomplete",
                joint.name
            );
            anyhow::ensure!(
                !joint.identity.model.is_empty(),
                "{} motor model is empty",
                joint.name
            );
            anyhow::ensure!(
                matches!(joint.direction, -1 | 1),
                "{} direction must be -1 or +1",
                joint.name
            );
            anyhow::ensure!(
                joint.zero_offset_rad.is_finite(),
                "{} zero offset is non-finite",
                joint.name
            );
            anyhow::ensure!(
                joint.torque_scale.is_finite()
                    && joint.torque_scale > 0.0
                    && joint.torque_scale <= 1.0,
                "{} torque scale must be in (0, 1]",
                joint.name
            );
            anyhow::ensure!(
                (1..=1000).contains(&joint.torque_permille),
                "{} torque_permille is invalid",
                joint.name
            );
            anyhow::ensure!(
                (1..=1000).contains(&joint.kp_kd_torque_permille),
                "{} kp/kd torque limit is invalid",
                joint.name
            );
            anyhow::ensure!(
                joint.limits.position_lower_rad.is_finite()
                    && joint.limits.position_upper_rad.is_finite(),
                "{} position limit is non-finite",
                joint.name
            );
            anyhow::ensure!(
                joint.limits.position_lower_rad < joint.limits.position_upper_rad,
                "{} position limits are reversed",
                joint.name
            );
            anyhow::ensure!(
                joint.limits.velocity_rad_s.is_finite() && joint.limits.velocity_rad_s > 0.0,
                "{} velocity limit is invalid",
                joint.name
            );
            anyhow::ensure!(
                joint.limits.torque_nm.is_finite() && joint.limits.torque_nm > 0.0,
                "{} torque limit is invalid",
                joint.name
            );
            anyhow::ensure!(
                joint.default_kp.is_finite() && joint.default_kp >= 0.0,
                "{} default kp is invalid",
                joint.name
            );
            anyhow::ensure!(
                joint.default_kd.is_finite() && joint.default_kd >= 0.0,
                "{} default kd is invalid",
                joint.name
            );
        }
        Ok(())
    }

    pub fn feedback_timeout(&self) -> Duration {
        Duration::from_millis(self.controller.feedback_timeout_ms)
    }

    pub fn command_watchdog(&self) -> Duration {
        Duration::from_millis(self.controller.command_watchdog_ms)
    }
}

impl JointProfile {
    pub fn compressed_mapping(&self) -> CompressedMitMapping {
        let a =
            (self.limits.position_lower_rad - self.zero_offset_rad) / (self.direction as f32 * TAU);
        let b =
            (self.limits.position_upper_rad - self.zero_offset_rad) / (self.direction as f32 * TAU);
        CompressedMitMapping {
            position_min: a.min(b),
            position_max: a.max(b),
            velocity_min: -self.limits.velocity_rad_s / TAU,
            velocity_max: self.limits.velocity_rad_s / TAU,
            torque_min: -self.limits.torque_nm * self.torque_scale,
            torque_max: self.limits.torque_nm * self.torque_scale,
            kp_min: 0.0,
            kp_max: (self.default_kp.max(1.0) * 2.0) * TAU,
            kd_min: 0.0,
            kd_max: (self.default_kd.max(0.1) * 2.0) * TAU,
        }
    }
}

#[cfg(test)]
impl IdentityFingerprint {
    pub fn test_value() -> Self {
        Self {
            vendor_id: 1,
            product_code: 2,
            revision: 3,
            serial_number: 4,
            model: "test".into(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_profile(urdf_path: String) -> HardwareProfile {
        HardwareProfile {
            schema_version: 1,
            validated: true,
            calibrated: false,
            robot_prefix: "hexmeow/test/arm0".into(),
            urdf_path,
            bus: BusProfile {
                channel: 0,
                adapter_vid: 1,
                adapter_pid: 2,
                heartbeat_node_id: 16,
                hardware_timestamp: true,
            },
            controller: ControllerProfile {
                loop_hz: 1000,
                state_publish_hz: 100,
                discovery_timeout_ms: 1000,
                feedback_timeout_ms: 100,
                command_watchdog_ms: 100,
            },
            joints: JOINT_NAMES
                .iter()
                .enumerate()
                .map(|(index, name)| JointProfile {
                    name: (*name).into(),
                    node_id: (index + 1) as u8,
                    identity: IdentityFingerprint::test_value(),
                    direction: 1,
                    zero_offset_rad: 0.0,
                    torque_scale: 1.0,
                    torque_permille: 100,
                    kp_kd_torque_permille: 100,
                    limits: JointLimits {
                        position_lower_rad: -1.0,
                        position_upper_rad: 1.0,
                        velocity_rad_s: 1.0,
                        torque_nm: 1.0,
                    },
                    default_kp: 10.0,
                    default_kd: 1.0,
                })
                .collect(),
        }
    }

    #[test]
    fn rejects_duplicate_nodes_and_unvalidated_profile() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.joints[1].node_id = profile.joints[0].node_id;
        assert!(profile.validate().is_err());
        profile.joints[1].node_id = 2;
        profile.validated = false;
        assert!(profile.validate().is_err());
    }

    #[test]
    fn mapping_accounts_for_negative_direction() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.joints[0].direction = -1;
        let mapping = profile.joints[0].compressed_mapping();
        assert!(mapping.position_min < mapping.position_max);
    }
}
