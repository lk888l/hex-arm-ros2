use std::collections::HashSet;
use std::f32::consts::TAU;
use std::path::Path;
use std::time::Duration;

use anyhow::{Context, Result};
#[cfg(not(feature = "legacy"))]
use hex_meow_motor as mapping_motor;
#[cfg(feature = "legacy")]
use hex_motor as mapping_motor;
use mapping_motor::cia402::CompressedMitMapping;
use serde::Deserialize;

use crate::single_turn::validate_single_turn_command_window;

pub const JOINT_NAMES: [&str; 6] = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
];
pub const HARDWARE_PROFILE_SCHEMA_VERSION: u32 = 3;
pub const JOINT_COORDINATE_VERSION: u32 = 2;
pub const SINGLE_TURN_COMMAND_SEAM_GUARD_REV: f32 = 0.01;

fn default_gravity_compensation_scale() -> f32 {
    1.0
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HardwareProfile {
    pub schema_version: u32,
    pub joint_coordinate_version: u32,
    pub validated: bool,
    pub calibrated: bool,
    pub robot_prefix: String,
    pub urdf_path: String,
    /// Gravity expressed in the URDF base-link frame. This safety-critical
    /// installation parameter is mandatory in schema v3.
    pub gravity_vector_base_m_s2: [f32; 3],
    /// Optional fixed payload whose mass properties are expressed directly in
    /// the serial arm tip frame.  This does not make the auxiliary device a
    /// controlled seventh axis.
    #[serde(default)]
    pub tip_payload: Option<TipPayloadProfile>,
    pub bus: BusProfile,
    pub controller: ControllerProfile,
    pub joints: Vec<JointProfile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TipPayloadProfile {
    pub mount_link: String,
    pub auxiliary_node_id: u8,
    pub identity: IdentityFingerprint,
    pub mass_kg: f32,
    pub center_of_mass_xyz_m: [f32; 3],
    pub source_urdf_sha256: String,
    pub inertial_calibrated: bool,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusProfile {
    #[serde(default)]
    pub protocol: MotorProtocol,
    #[serde(default)]
    pub transport: BusTransport,
    #[serde(default)]
    pub interface: String,
    pub channel: u16,
    pub adapter_vid: u16,
    pub adapter_pid: u16,
    pub heartbeat_node_id: u8,
    pub hardware_timestamp: bool,
    /// When enabled, joint_N must use CANopen node N. This locks the verified
    /// Firefly Y6 field wiring without preventing other profiles from using an
    /// explicitly reviewed non-direct assignment.
    #[serde(default)]
    pub direct_joint_mapping: bool,
    #[serde(default)]
    pub auxiliary_node_ids: Vec<u8>,
    #[serde(default)]
    pub expected_link: Option<ExpectedSocketCanLink>,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ExpectedSocketCanLink {
    pub nominal_bitrate: u32,
    pub nominal_sample_point_permille: u16,
    pub nominal_sjw: u16,
    pub data_bitrate: u32,
    pub data_sample_point_permille: u16,
    pub data_sjw: u16,
    pub fd: bool,
    pub restart_ms: u32,
    pub adapter: SocketCanAdapterFingerprint,
}

#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct SocketCanAdapterFingerprint {
    pub driver: String,
    pub vendor_id: u16,
    pub product_id: u16,
    pub serial: String,
    pub channel: u16,
}

#[derive(Debug, Clone, Copy, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum BusTransport {
    #[default]
    GsUsb,
    #[serde(alias = "socket-can", alias = "socketcan")]
    SocketCan,
}

#[derive(Debug, Clone, Copy, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MotorProtocol {
    #[default]
    Cia402,
    Meow,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerProfile {
    pub loop_hz: u32,
    pub state_publish_hz: u32,
    pub discovery_timeout_ms: u64,
    pub feedback_timeout_ms: u64,
    pub command_watchdog_ms: u64,
    /// Enable-time gravity ramp in joint-side Nm/s. Once settled, gravity
    /// follows feedback directly, without a continuous slew limiter.
    #[serde(default)]
    pub gravity_startup_slew_rate_nm_s: Option<f32>,
    /// Explicitly enabled, bounded pre-disable damping after a verified ready pose.
    #[serde(default)]
    pub shutdown_damping: Option<ShutdownDamping>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShutdownDamping {
    pub kd_nm_s_rad: [f32; 6],
    pub unload_sec: f32,
    pub timeout_sec: f32,
    pub settle_sec: f32,
}

impl ShutdownDamping {
    pub fn validate(&self) -> Result<()> {
        anyhow::ensure!(
            self.kd_nm_s_rad
                .iter()
                .all(|v| v.is_finite() && *v > 0.0 && *v <= 100.0),
            "invalid shutdown damping gains"
        );
        anyhow::ensure!(
            self.unload_sec.is_finite() && (1.0..=10.0).contains(&self.unload_sec),
            "invalid shutdown unload time"
        );
        anyhow::ensure!(
            self.settle_sec.is_finite() && (0.5..=2.0).contains(&self.settle_sec),
            "invalid shutdown settle time"
        );
        anyhow::ensure!(
            self.timeout_sec.is_finite()
                && self.timeout_sec >= self.unload_sec + self.settle_sec + 1.0
                && self.timeout_sec <= 30.0,
            "invalid shutdown timeout"
        );
        Ok(())
    }
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
    #[serde(default = "default_gravity_compensation_scale")]
    pub gravity_compensation_scale: f32,
    /// Independent clamp on scaled gravity, before motor-unit conversion.
    #[serde(default)]
    pub gravity_compensation_limit_nm: Option<f32>,
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
    /// Additional read-only feedback envelope beyond the strict command
    /// limits. This accommodates measured hard-stop compliance/backlash
    /// without authorizing a command outside `position_lower/upper_rad`.
    #[serde(default)]
    pub measured_position_margin_rad: f32,
    pub velocity_rad_s: f32,
    pub acceleration_rad_s2: f32,
    pub torque_nm: f32,
}

impl JointLimits {
    pub fn measured_position_lower_rad(&self) -> f32 {
        self.position_lower_rad - self.measured_position_margin_rad
    }

    pub fn measured_position_upper_rad(&self) -> f32 {
        self.position_upper_rad + self.measured_position_margin_rad
    }
}

pub(crate) fn validate_gravity_vector(gravity: [f32; 3]) -> Result<()> {
    anyhow::ensure!(
        gravity.iter().all(|value| value.is_finite()),
        "gravity vector is non-finite"
    );
    let norm = gravity
        .iter()
        .map(|value| value * value)
        .sum::<f32>()
        .sqrt();
    anyhow::ensure!(
        (8.0..=12.0).contains(&norm),
        "gravity magnitude must be within 8..12 m/s^2"
    );
    Ok(())
}

impl HardwareProfile {
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self> {
        Self::from_path_with_urdf(path, None)
    }

    /// Only the model location may be overridden by a relocatable deployment.
    /// All numeric calibration and motion authority still come from the profile.
    pub fn from_path_with_urdf(path: impl AsRef<Path>, urdf: Option<&Path>) -> Result<Self> {
        let path = path.as_ref();
        let contents = std::fs::read_to_string(path)
            .with_context(|| format!("read hardware profile {}", path.display()))?;
        let mut profile: Self = serde_yaml::from_str(&contents)
            .with_context(|| format!("parse hardware profile {}", path.display()))?;
        if let Some(urdf) = urdf {
            profile.urdf_path = urdf.to_string_lossy().into_owned();
        }
        profile.validate()?;
        Ok(profile)
    }

    pub fn validate(&self) -> Result<()> {
        if let Some(damping) = &self.controller.shutdown_damping {
            damping.validate()?;
            anyhow::ensure!(
                self.bus.protocol == MotorProtocol::Meow,
                "shutdown damping requires Meow"
            );
        }
        anyhow::ensure!(
            self.schema_version == HARDWARE_PROFILE_SCHEMA_VERSION,
            "unsupported hardware profile schema_version {}; expected {HARDWARE_PROFILE_SCHEMA_VERSION}",
            self.schema_version
        );
        anyhow::ensure!(
            self.joint_coordinate_version == JOINT_COORDINATE_VERSION,
            "unsupported joint_coordinate_version {}; expected {JOINT_COORDINATE_VERSION} with centered J3 coordinates",
            self.joint_coordinate_version
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
        validate_gravity_vector(self.gravity_vector_base_m_s2)
            .context("gravity_vector_base_m_s2 is invalid")?;
        anyhow::ensure!(
            (1..=127).contains(&self.bus.heartbeat_node_id),
            "invalid host heartbeat node id"
        );
        anyhow::ensure!(
            self.bus.protocol != MotorProtocol::Meow
                || self.bus.transport == BusTransport::SocketCan,
            "Meow motor protocol requires socket_can transport"
        );
        match self.bus.transport {
            BusTransport::GsUsb => {
                anyhow::ensure!(
                    self.bus.adapter_vid != 0 && self.bus.adapter_pid != 0,
                    "USB VID/PID are required for gs_usb"
                );
                anyhow::ensure!(
                    self.bus.expected_link.is_none(),
                    "expected_link is only valid for SocketCAN"
                );
            }
            BusTransport::SocketCan => {
                anyhow::ensure!(
                    valid_socketcan_interface_name(&self.bus.interface),
                    "SocketCAN interface must be a conventional Linux interface name"
                );
                let expected_link = self
                    .bus
                    .expected_link
                    .as_ref()
                    .context("SocketCAN expected_link is required")?;
                expected_link.validate_field_link()?;
                anyhow::ensure!(
                    self.bus.channel == expected_link.adapter.channel,
                    "SocketCAN bus.channel {} does not match expected adapter channel {}",
                    self.bus.channel,
                    expected_link.adapter.channel
                );
            }
        }
        anyhow::ensure!(
            self.controller.loop_hz == 1000
                || (self.bus.protocol == MotorProtocol::Meow && self.controller.loop_hz == 500),
            "motor loop must be configured at 1000 Hz (Meow also supports 500 Hz)"
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
        if let Some(rate) = self.controller.gravity_startup_slew_rate_nm_s {
            anyhow::ensure!(
                rate.is_finite() && rate > 0.0,
                "gravity_startup_slew_rate_nm_s must be finite and positive"
            );
        }
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
            if self.bus.direct_joint_mapping {
                anyhow::ensure!(
                    joint.node_id == (index + 1) as u8,
                    "{} must map directly to CANopen node {} while bus.direct_joint_mapping is true",
                    joint.name,
                    index + 1
                );
            }
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
                joint.gravity_compensation_scale.is_finite()
                    && (0.0..=2.0).contains(&joint.gravity_compensation_scale),
                "{} gravity compensation scale must be finite and in [0, 2]",
                joint.name
            );
            anyhow::ensure!(
                self.bus.protocol != MotorProtocol::Meow || joint.torque_scale == 1.0,
                "{} Meow protocol requires torque_scale=1; calibrate gravity with gravity_compensation_scale",
                joint.name
            );
            if let Some(limit) = joint.gravity_compensation_limit_nm {
                anyhow::ensure!(
                    limit.is_finite() && limit >= 0.0 && limit <= joint.limits.torque_nm,
                    "{} gravity_compensation_limit_nm must be finite, non-negative and no greater than the joint torque limit",
                    joint.name
                );
            }
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
                joint.limits.measured_position_margin_rad.is_finite()
                    && (0.0..=0.01).contains(&joint.limits.measured_position_margin_rad),
                "{} measured position margin must be finite and within 0..=0.01 rad",
                joint.name
            );
            anyhow::ensure!(
                joint.limits.measured_position_upper_rad()
                    - joint.limits.measured_position_lower_rad()
                    < TAU,
                "{} measured position envelope spans a full turn and is ambiguous",
                joint.name
            );
            anyhow::ensure!(
                joint.limits.velocity_rad_s.is_finite() && joint.limits.velocity_rad_s > 0.0,
                "{} velocity limit is invalid",
                joint.name
            );
            anyhow::ensure!(
                joint.limits.acceleration_rad_s2.is_finite()
                    && joint.limits.acceleration_rad_s2 > 0.0,
                "{} acceleration limit is invalid",
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
        let mut auxiliary_nodes = HashSet::new();
        for node_id in &self.bus.auxiliary_node_ids {
            anyhow::ensure!(
                (1..=127).contains(node_id),
                "invalid auxiliary CANopen node id {node_id}"
            );
            anyhow::ensure!(
                *node_id != self.bus.heartbeat_node_id,
                "auxiliary node {node_id} conflicts with host heartbeat node"
            );
            anyhow::ensure!(
                !nodes.contains(node_id),
                "auxiliary node {node_id} is already assigned to an arm joint"
            );
            anyhow::ensure!(
                auxiliary_nodes.insert(*node_id),
                "duplicate auxiliary CANopen node id {node_id}"
            );
        }
        if let Some(payload) = &self.tip_payload {
            anyhow::ensure!(
                payload.mount_link == "link_6",
                "tip payload mount_link must be the serial arm tip link_6"
            );
            anyhow::ensure!(
                auxiliary_nodes.contains(&payload.auxiliary_node_id),
                "tip payload auxiliary node {} must be present in bus.auxiliary_node_ids",
                payload.auxiliary_node_id
            );
            anyhow::ensure!(
                payload.identity.vendor_id != 0 && payload.identity.product_code != 0,
                "tip payload identity fingerprint is incomplete"
            );
            anyhow::ensure!(
                payload.identity.revision != 0 && payload.identity.serial_number != 0,
                "tip payload revision/serial fingerprint is incomplete"
            );
            anyhow::ensure!(
                !payload.identity.model.trim().is_empty(),
                "tip payload model is empty"
            );
            anyhow::ensure!(
                payload.mass_kg.is_finite() && payload.mass_kg > 0.0 && payload.mass_kg <= 5.0,
                "tip payload mass_kg must be finite and in (0, 5]"
            );
            anyhow::ensure!(
                payload
                    .center_of_mass_xyz_m
                    .iter()
                    .all(|value| value.is_finite() && value.abs() <= 1.0),
                "tip payload center_of_mass_xyz_m must be finite and within +/-1 m"
            );
            anyhow::ensure!(
                payload.source_urdf_sha256.len() == 64
                    && payload
                        .source_urdf_sha256
                        .bytes()
                        .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
                "tip payload source_urdf_sha256 must be a 64-character lowercase SHA-256"
            );
            anyhow::ensure!(
                !self.calibrated || payload.inertial_calibrated,
                "calibrated hardware profile requires calibrated tip payload inertial data"
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

    /// Validate protocol-specific command representation before enabling.
    /// CiA402 compressed MIT keeps its verified single-turn seam guard;
    /// Meow uses signed Q8.24 multi-turn position targets.
    pub fn validate_single_turn_command_windows(&self) -> Result<()> {
        for joint in &self.joints {
            let mapping = joint.compressed_mapping();
            if self.bus.protocol == MotorProtocol::Meow {
                anyhow::ensure!(
                    [mapping.position_min, mapping.position_max]
                        .iter()
                        .all(|value| value.is_finite() && (-128.0..128.0).contains(value)),
                    "{} Meow position window exceeds signed Q8.24 range [-128, 128) rev",
                    joint.name
                );
                continue;
            }
            validate_single_turn_command_window(
                mapping.position_min,
                mapping.position_max,
                SINGLE_TURN_COMMAND_SEAM_GUARD_REV,
            )
            .with_context(|| {
                format!(
                    "{} command range is not a verified single-turn commissioning window",
                    joint.name
                )
            })?;
        }
        Ok(())
    }
}

fn valid_socketcan_interface_name(interface: &str) -> bool {
    !interface.is_empty()
        && interface.len() <= 15
        && interface
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.'))
}

impl ExpectedSocketCanLink {
    fn validate_field_link(&self) -> Result<()> {
        anyhow::ensure!(
            self.nominal_bitrate == 1_000_000,
            "SocketCAN nominal bitrate must be 1000000"
        );
        anyhow::ensure!(
            self.nominal_sample_point_permille == 800,
            "SocketCAN nominal sample point must be 800 permille"
        );
        anyhow::ensure!(self.nominal_sjw == 5, "SocketCAN nominal SJW must be 5");
        anyhow::ensure!(
            self.data_bitrate == 4_000_000,
            "SocketCAN data bitrate must be 4000000"
        );
        anyhow::ensure!(
            self.data_sample_point_permille == 800,
            "SocketCAN data sample point must be 800 permille"
        );
        anyhow::ensure!(self.data_sjw == 3, "SocketCAN data SJW must be 3");
        anyhow::ensure!(self.fd, "SocketCAN CAN-FD mode must be enabled");
        anyhow::ensure!(
            self.restart_ms == 0,
            "SocketCAN restart_ms must remain 0 for fail-closed recovery"
        );
        anyhow::ensure!(
            self.adapter.driver == "gs_usb",
            "SocketCAN adapter driver must be gs_usb"
        );
        anyhow::ensure!(
            self.adapter.vendor_id == 0x1209 && self.adapter.product_id == 0x2323,
            "SocketCAN adapter must use USB VID:PID 1209:2323"
        );
        anyhow::ensure!(
            self.adapter.serial.len() == 32
                && self
                    .adapter
                    .serial
                    .bytes()
                    .all(|byte| byte.is_ascii_hexdigit()),
            "SocketCAN adapter serial must be the exact 32-digit USB serial"
        );
        anyhow::ensure!(
            self.adapter.channel <= 3,
            "SocketCAN adapter channel must be in 0..=3"
        );
        Ok(())
    }
}

impl JointProfile {
    pub fn clamp_gravity_feedforward(&self, torque_nm: f32) -> f32 {
        // Preserve non-finite dynamics output for the caller's fault gate.
        match self.gravity_compensation_limit_nm {
            Some(limit) if torque_nm.is_finite() => torque_nm.clamp(-limit, limit),
            _ => torque_nm,
        }
    }

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
            kp_max: (self.default_kp.max(1.0) * 2.0) * TAU * self.torque_scale,
            kd_min: 0.0,
            kd_max: (self.default_kd.max(0.1) * 2.0) * TAU * self.torque_scale,
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
    #[cfg(feature = "legacy")]
    use crate::conversion::{ros_target_to_motor, RosTarget};
    #[cfg(feature = "legacy")]
    use mapping_motor::cia402::{compressed_mit::packed_target_words, CompressedMitTarget};

    #[test]
    fn shutdown_damping_configuration_is_bounded() {
        let valid = ShutdownDamping {
            kd_nm_s_rad: [15.0; 6],
            unload_sec: 3.0,
            timeout_sec: 20.0,
            settle_sec: 0.5,
        };
        valid.validate().unwrap();
        for value in [0.0, -1.0, 100.1, f32::NAN, f32::INFINITY] {
            let mut bad = valid.clone();
            bad.kd_nm_s_rad[2] = value;
            assert!(bad.validate().is_err());
        }
        for (unload, timeout, settle) in [
            (0.0, 20.0, 0.5),
            (3.0, 31.0, 0.5),
            (3.0, 4.0, 0.5),
            (3.0, 20.0, 0.0),
        ] {
            let mut bad = valid.clone();
            bad.unload_sec = unload;
            bad.timeout_sec = timeout;
            bad.settle_sec = settle;
            assert!(bad.validate().is_err());
        }
    }

    fn valid_profile(urdf_path: String) -> HardwareProfile {
        HardwareProfile {
            schema_version: HARDWARE_PROFILE_SCHEMA_VERSION,
            joint_coordinate_version: JOINT_COORDINATE_VERSION,
            validated: true,
            calibrated: false,
            robot_prefix: "hexmeow/test/arm0".into(),
            urdf_path,
            gravity_vector_base_m_s2: [0.0, 0.0, -9.81],
            tip_payload: None,
            bus: BusProfile {
                protocol: MotorProtocol::Cia402,
                transport: BusTransport::GsUsb,
                interface: String::new(),
                channel: 0,
                adapter_vid: 1,
                adapter_pid: 2,
                heartbeat_node_id: 16,
                hardware_timestamp: true,
                direct_joint_mapping: false,
                auxiliary_node_ids: Vec::new(),
                expected_link: None,
            },
            controller: ControllerProfile {
                loop_hz: 1000,
                state_publish_hz: 100,
                discovery_timeout_ms: 1000,
                feedback_timeout_ms: 100,
                command_watchdog_ms: 100,
                gravity_startup_slew_rate_nm_s: None,
                shutdown_damping: None,
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
                    gravity_compensation_scale: 1.0,
                    gravity_compensation_limit_nm: None,
                    torque_permille: 100,
                    kp_kd_torque_permille: 100,
                    limits: JointLimits {
                        position_lower_rad: -1.0,
                        position_upper_rad: 1.0,
                        measured_position_margin_rad: 0.0,
                        velocity_rad_s: 1.0,
                        acceleration_rad_s2: 1.0,
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
    fn direct_joint_mapping_is_optional_but_strict_when_enabled() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.joints.swap(0, 1);
        profile.joints[0].name = JOINT_NAMES[0].into();
        profile.joints[1].name = JOINT_NAMES[1].into();

        // A reviewed generic profile may deliberately assign nodes in another
        // order, but the field Firefly profile opts into the direct invariant.
        assert!(profile.validate().is_ok());
        profile.bus.direct_joint_mapping = true;
        let error = profile.validate().unwrap_err().to_string();
        assert!(error.contains("joint_1 must map directly to CANopen node 1"));
    }

    #[test]
    fn schema_v3_requires_coordinate_version_and_base_frame_gravity_vector() {
        let yaml = include_str!("../test/firefly_y6.mock.yaml")
            .lines()
            .filter(|line| !line.trim_start().starts_with("gravity_vector_base_m_s2:"))
            .collect::<Vec<_>>()
            .join("\n");
        let error = serde_yaml::from_str::<HardwareProfile>(&yaml)
            .unwrap_err()
            .to_string();
        assert!(error.contains("missing field `gravity_vector_base_m_s2`"));

        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.schema_version = 2;
        let error = profile.validate().unwrap_err().to_string();
        assert!(error.contains("expected 3"));
        profile.schema_version = HARDWARE_PROFILE_SCHEMA_VERSION;
        profile.joint_coordinate_version = 1;
        let error = profile.validate().unwrap_err().to_string();
        assert!(error.contains("expected 2 with centered J3 coordinates"));
    }

    #[test]
    fn schema_v3_requires_finite_positive_joint_acceleration_limits() {
        let yaml = include_str!("../test/firefly_y6.mock.yaml").replacen(
            " acceleration_rad_s2: 10.0,",
            "",
            1,
        );
        let error = serde_yaml::from_str::<HardwareProfile>(&yaml)
            .unwrap_err()
            .to_string();
        assert!(error.contains("missing field `acceleration_rad_s2`"));

        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        for invalid in [0.0, -0.1, f32::NAN, f32::INFINITY] {
            profile.joints[0].limits.acceleration_rad_s2 = invalid;
            let error = profile.validate().unwrap_err().to_string();
            assert!(error.contains("joint_1 acceleration limit is invalid"));
        }
    }

    #[test]
    fn measured_position_margin_is_read_only_bounded_and_defaults_to_zero() {
        let yaml = include_str!("../test/firefly_y6.mock.yaml").replacen(
            " measured_position_margin_rad: 0.0,",
            "",
            1,
        );
        let parsed: HardwareProfile = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(
            parsed.joints[0]
                .limits
                .measured_position_margin_rad
                .to_bits(),
            0.0_f32.to_bits()
        );

        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.joints[0].limits.measured_position_margin_rad = 0.001;
        profile.validate().unwrap();
        for invalid in [-0.0001, 0.0101, f32::NAN, f32::INFINITY] {
            profile.joints[0].limits.measured_position_margin_rad = invalid;
            let error = profile.validate().unwrap_err().to_string();
            assert!(error.contains("joint_1 measured position margin"));
        }

        profile.joints[0].limits.position_lower_rad = -std::f32::consts::PI;
        profile.joints[0].limits.position_upper_rad = std::f32::consts::PI;
        profile.joints[0].limits.measured_position_margin_rad = 0.01;
        let error = profile.validate().unwrap_err().to_string();
        assert!(error.contains("measured position envelope spans a full turn"));
    }

    #[test]
    fn base_frame_gravity_vector_is_finite_and_has_earth_gravity_magnitude() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());

        for invalid in [
            [f32::NAN, 0.0, 9.81],
            [f32::INFINITY, 0.0, 9.81],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 7.99],
            [0.0, 0.0, 12.01],
        ] {
            profile.gravity_vector_base_m_s2 = invalid;
            let error = profile.validate().unwrap_err().to_string();
            assert!(
                error.contains("gravity_vector_base_m_s2 is invalid"),
                "unexpected validation error for {invalid:?}: {error}"
            );
        }

        for valid in [[0.0, 0.0, -9.81], [0.0, 0.0, 9.81], [0.0, 9.81, 0.0]] {
            profile.gravity_vector_base_m_s2 = valid;
            profile.validate().unwrap();
        }
    }

    #[test]
    fn gravity_compensation_scale_defaults_to_one_for_legacy_yaml() {
        let joint: JointProfile = serde_yaml::from_str(
            r#"
name: joint_1
node_id: 1
identity: {vendor_id: 1, product_code: 2, revision: 3, serial_number: 4, model: test}
direction: 1
zero_offset_rad: 0.0
torque_scale: 1.0
torque_permille: 100
kp_kd_torque_permille: 100
limits: {position_lower_rad: -1.0, position_upper_rad: 1.0, velocity_rad_s: 0.1, acceleration_rad_s2: 0.1, torque_nm: 1.0}
default_kp: 2.0
default_kd: 0.3
"#,
        )
        .unwrap();

        assert_eq!(joint.gravity_compensation_scale, 1.0);
        assert_eq!(joint.gravity_compensation_limit_nm, None);
    }

    #[test]
    fn gravity_compensation_scale_is_finite_and_bounded() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());

        for invalid in [-0.01, 2.01, f32::NAN, f32::INFINITY] {
            profile.joints[3].gravity_compensation_scale = invalid;
            let error = profile.validate().unwrap_err().to_string();
            assert!(
                error.contains("joint_4 gravity compensation scale"),
                "unexpected validation error for {invalid:?}: {error}"
            );
        }

        for valid in [0.0, 1.0, 2.0] {
            profile.joints[3].gravity_compensation_scale = valid;
            profile.validate().unwrap();
        }
    }

    #[test]
    fn gravity_limits_and_startup_rate_are_optional_and_validated() {
        let legacy: HardwareProfile =
            serde_yaml::from_str(include_str!("../test/firefly_y6.mock.yaml")).unwrap();
        assert_eq!(legacy.bus.protocol, MotorProtocol::Cia402);
        assert_eq!(legacy.controller.gravity_startup_slew_rate_nm_s, None);
        assert!(legacy
            .joints
            .iter()
            .all(|joint| joint.gravity_compensation_limit_nm.is_none()));
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        for invalid in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            profile.controller.gravity_startup_slew_rate_nm_s = Some(invalid);
            assert!(profile
                .validate()
                .unwrap_err()
                .to_string()
                .contains("gravity_startup_slew_rate_nm_s"));
        }
        profile.controller.gravity_startup_slew_rate_nm_s = Some(5.0);
        for invalid in [-0.1, 1.01, f32::NAN, f32::INFINITY] {
            profile.joints[0].gravity_compensation_limit_nm = Some(invalid);
            assert!(profile
                .validate()
                .unwrap_err()
                .to_string()
                .contains("gravity_compensation_limit_nm"));
        }
        for valid in [0.0, 0.2, 1.0] {
            profile.joints[0].gravity_compensation_limit_nm = Some(valid);
            profile.validate().unwrap();
        }
        let joint = &profile.joints[0];
        assert_eq!(joint.clamp_gravity_feedforward(3.0), 1.0);
        assert_eq!(joint.clamp_gravity_feedforward(-3.0), -1.0);
        assert_eq!(joint.clamp_gravity_feedforward(0.2), 0.2);
        assert!(joint.clamp_gravity_feedforward(f32::NAN).is_nan());
        assert_eq!(
            joint.clamp_gravity_feedforward(f32::INFINITY),
            f32::INFINITY
        );
    }

    #[test]
    fn meow_profile_uses_socketcan_si_calibration_and_500_or_1000_hz() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.bus.protocol = MotorProtocol::Meow;
        assert!(profile
            .validate()
            .unwrap_err()
            .to_string()
            .contains("socket_can"));
        profile.bus.transport = BusTransport::SocketCan;
        profile.bus.interface = "can0".into();
        profile.bus.expected_link = Some(expected_socketcan_link());
        for loop_hz in [500, 1000] {
            profile.controller.loop_hz = loop_hz;
            profile.validate().unwrap();
        }
        profile.joints[0].torque_scale = 0.85;
        assert!(profile
            .validate()
            .unwrap_err()
            .to_string()
            .contains("torque_scale=1"));
        profile.joints[0].torque_scale = 1.0;
        profile.controller.loop_hz = 500;
        profile.bus.protocol = MotorProtocol::Cia402;
        assert!(profile.validate().is_err());
    }

    #[test]
    fn meow_command_window_uses_multi_turn_q8_24_instead_of_single_turn_seam() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.joints[0].zero_offset_rad = 2.0 * TAU;
        assert!(profile.validate_single_turn_command_windows().is_err());
        profile.bus.protocol = MotorProtocol::Meow;
        profile.validate_single_turn_command_windows().unwrap();
        profile.joints[0].zero_offset_rad = 129.0 * TAU;
        assert!(profile
            .validate_single_turn_command_windows()
            .unwrap_err()
            .to_string()
            .contains("Q8.24"));
        profile.joints[0].zero_offset_rad = f32::NAN;
        assert!(profile.validate_single_turn_command_windows().is_err());
    }

    fn trial_tip_payload() -> TipPayloadProfile {
        TipPayloadProfile {
            mount_link: "link_6".into(),
            auxiliary_node_id: 15,
            identity: IdentityFingerprint {
                vendor_id: 0x4859_444c,
                product_code: 0xaaaa_0001,
                revision: 9,
                serial_number: 0x2573_510b,
                model: "GR80 trial".into(),
            },
            mass_kg: 0.41,
            center_of_mass_xyz_m: [0.005_537_805, 0.000_026_829, 0.048_889_972],
            source_urdf_sha256: "f74b3e76b14175c788c5ef70dd0c1941958d229a8461da6f20e17543e4ba1114"
                .into(),
            inertial_calibrated: false,
        }
    }

    #[test]
    fn tip_payload_requires_serial_tip_auxiliary_allowlist_and_finite_bounds() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.bus.auxiliary_node_ids = vec![15];
        profile.tip_payload = Some(trial_tip_payload());
        profile.validate().unwrap();

        profile.tip_payload.as_mut().unwrap().mount_link = "link_5".into();
        assert!(profile
            .validate()
            .unwrap_err()
            .to_string()
            .contains("link_6"));
        profile.tip_payload.as_mut().unwrap().mount_link = "link_6".into();

        profile.bus.auxiliary_node_ids.clear();
        assert!(profile
            .validate()
            .unwrap_err()
            .to_string()
            .contains("bus.auxiliary_node_ids"));
        profile.bus.auxiliary_node_ids = vec![15];

        profile.tip_payload.as_mut().unwrap().mass_kg = f32::NAN;
        assert!(profile
            .validate()
            .unwrap_err()
            .to_string()
            .contains("mass_kg"));
        profile.tip_payload.as_mut().unwrap().mass_kg = 0.0;
        assert!(profile
            .validate()
            .unwrap_err()
            .to_string()
            .contains("mass_kg"));
        profile.tip_payload.as_mut().unwrap().mass_kg = 0.41;
        profile.tip_payload.as_mut().unwrap().center_of_mass_xyz_m[2] = f32::INFINITY;
        assert!(profile
            .validate()
            .unwrap_err()
            .to_string()
            .contains("center_of_mass_xyz_m"));
    }

    #[test]
    fn uncalibrated_trial_payload_cannot_be_in_a_calibrated_profile() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.bus.auxiliary_node_ids = vec![15];
        profile.tip_payload = Some(trial_tip_payload());
        profile.calibrated = true;
        let error = profile.validate().unwrap_err().to_string();
        assert!(error.contains("calibrated tip payload inertial data"));

        profile.tip_payload.as_mut().unwrap().inertial_calibrated = true;
        profile.validate().unwrap();
    }

    #[test]
    fn tip_payload_source_hash_is_exact_lowercase_sha256() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.bus.auxiliary_node_ids = vec![15];
        profile.tip_payload = Some(trial_tip_payload());
        profile.tip_payload.as_mut().unwrap().source_urdf_sha256 = "F".repeat(64);
        assert!(profile
            .validate()
            .unwrap_err()
            .to_string()
            .contains("lowercase SHA-256"));
    }

    #[test]
    fn mapping_accounts_for_negative_direction() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.joints[0].direction = -1;
        let mapping = profile.joints[0].compressed_mapping();
        assert!(mapping.position_min < mapping.position_max);
    }

    #[test]
    #[cfg(feature = "legacy")]
    fn compressed_gain_mapping_uses_motor_torque_scale_and_two_times_default_boundary() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        let joint = &mut profile.joints[0];
        joint.torque_scale = 0.8;
        joint.default_kp = 10.0;
        joint.default_kd = 1.5;

        let mapping = joint.compressed_mapping();
        assert!(
            (mapping.kp_max - 2.0 * joint.default_kp * TAU * joint.torque_scale).abs() < 1.0e-5
        );
        assert!(
            (mapping.kd_max - 2.0 * joint.default_kd * TAU * joint.torque_scale).abs() < 1.0e-5
        );

        let gain_codes = |kp_nm_rad: f32, kd_nm_s_rad: f32| {
            let motor = ros_target_to_motor(
                RosTarget {
                    position_rad: 0.0,
                    velocity_rad_s: 0.0,
                    torque_nm: 0.0,
                    kp_nm_rad,
                    kd_nm_s_rad,
                },
                joint,
            );
            let target = CompressedMitTarget {
                position: motor.position_rev,
                velocity: motor.velocity_rev_s,
                torque: motor.torque_nm,
                kp: motor.kp_nm_rev,
                kd: motor.kd_nm_s_rev,
            };
            let (lower, upper) = packed_target_words(&target, &mapping);
            let kp_code = (((upper & 0x0f) << 8) | ((lower >> 24) & 0xff)) as u16;
            let kd_code = ((lower >> 12) & 0x0fff) as u16;
            (motor, kp_code, kd_code)
        };

        let (default, default_kp_code, default_kd_code) =
            gain_codes(joint.default_kp, joint.default_kd);
        assert_eq!((default_kp_code, default_kd_code), (2047, 2047));
        assert!(default.kp_nm_rev < mapping.kp_max);
        assert!(default.kd_nm_s_rev < mapping.kd_max);

        let (boundary, boundary_kp_code, boundary_kd_code) =
            gain_codes(2.0 * joint.default_kp, 2.0 * joint.default_kd);
        assert_eq!((boundary_kp_code, boundary_kd_code), (4095, 4095));
        assert!((boundary.kp_nm_rev - mapping.kp_max).abs() < 1.0e-5);
        assert!((boundary.kd_nm_s_rev - mapping.kd_max).abs() < 1.0e-5);

        let (outside, _, _) = gain_codes(2.001 * joint.default_kp, 2.001 * joint.default_kd);
        assert!(outside.kp_nm_rev > mapping.kp_max);
        assert!(outside.kd_nm_s_rev > mapping.kd_max);
    }

    #[test]
    fn real_command_window_check_accepts_corrected_joint_2_and_rejects_old_fit() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        assert!(profile.validate().is_ok());
        profile.joints[1].direction = -1;
        profile.joints[1].zero_offset_rad = 0.010_217;
        profile.joints[1].limits.position_lower_rad = -1.57;
        profile.joints[1].limits.position_upper_rad = 2.09;
        assert!(profile.validate_single_turn_command_windows().is_ok());

        profile.joints[1].zero_offset_rad = 3.150_187;
        assert!(profile.validate_single_turn_command_windows().is_err());

        for joint in &mut profile.joints {
            joint.direction = 1;
            joint.zero_offset_rad = 0.0;
            joint.limits.position_lower_rad = -0.25;
            joint.limits.position_upper_rad = 0.25;
        }
        assert!(profile.validate_single_turn_command_windows().is_ok());
    }

    fn expected_socketcan_link() -> ExpectedSocketCanLink {
        ExpectedSocketCanLink {
            nominal_bitrate: 1_000_000,
            nominal_sample_point_permille: 800,
            nominal_sjw: 5,
            data_bitrate: 4_000_000,
            data_sample_point_permille: 800,
            data_sjw: 3,
            fd: true,
            restart_ms: 0,
            adapter: SocketCanAdapterFingerprint {
                driver: "gs_usb".into(),
                vendor_id: 0x1209,
                product_id: 0x2323,
                serial: "0123456789ABCDEF0123456789ABCDEF".into(),
                channel: 0,
            },
        }
    }

    #[test]
    fn socketcan_profile_requires_the_field_timing_and_adapter_fingerprint() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let mut profile = valid_profile(file.path().display().to_string());
        profile.bus.transport = BusTransport::SocketCan;
        profile.bus.interface = "can0".into();
        profile.bus.expected_link = Some(expected_socketcan_link());
        assert!(profile.validate().is_ok());

        profile.bus.expected_link.as_mut().unwrap().data_bitrate = 5_000_000;
        assert!(profile.validate().is_err());
        profile.bus.expected_link = Some(expected_socketcan_link());
        profile.bus.expected_link.as_mut().unwrap().adapter.serial = "REPLACE_ME".into();
        assert!(profile.validate().is_err());

        profile.bus.expected_link = Some(expected_socketcan_link());
        profile.bus.channel = 2;
        let error = profile.validate().unwrap_err().to_string();
        assert!(error.contains("bus.channel 2"));
        assert!(error.contains("adapter channel 0"));
    }
}
