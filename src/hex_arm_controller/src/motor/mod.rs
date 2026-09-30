//! Motor boundary. No ROS, trajectory scheduling or startup orchestration lives here.
use crate::conversion::MotorTarget;
use anyhow::Result;
use async_trait::async_trait;
use std::time::Instant;

#[cfg(feature = "legacy")]
pub(crate) mod legacy;
mod meow;
mod mock;

#[cfg(feature = "legacy")]
pub use legacy::{CompressedTargetReadback, RealBackend};
pub use meow::MeowBackend;
pub use mock::MockBackend;

pub const DOF: usize = 6;

#[derive(Debug, Clone, Default)]
pub struct MotorIdentitySnapshot {
    pub node_id: u8,
    pub vendor_id: u32,
    pub product_code: u32,
    pub revision: u32,
    pub serial_number: u32,
    pub model: String,
    pub identity_verified: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct JointFeedback {
    pub position_rev: f32,
    pub velocity_rev_s: f32,
    pub torque_nm: f32,
    /// Compatibility temperature used by the ROS diagnostic surface (motor
    /// temperature when available, otherwise driver temperature).
    pub temperature_c: f32,
    /// Raw TPDO2 temperatures retained separately for short hardware
    /// diagnostics.  The initial feedback gate requires both to be present.
    pub driver_temperature_c: f32,
    pub motor_temperature_c: f32,
    pub online: bool,
    pub fresh: bool,
    pub fault_code: Option<u16>,
}

#[derive(Debug, Clone)]
pub struct FeedbackSnapshot {
    pub joints: [JointFeedback; DOF],
    /// Oldest receive timestamp of TPDO1 across all six required joints.
    ///
    /// This is a motion-feedback progression clock only.  It must never
    /// replace `captured_at` or the per-joint `fresh` bit for gates that also
    /// require TPDO2 status and temperature data.
    pub oldest_tpdo1_at: Option<Instant>,
    /// Oldest required-TPDO timestamp across all six joints.  Each joint's
    /// required timestamp is the older of TPDO1 and TPDO2, so this retains the
    /// full feedback/status/temperature freshness contract.
    pub captured_at: Option<Instant>,
}

impl Default for FeedbackSnapshot {
    fn default() -> Self {
        Self {
            joints: [JointFeedback::default(); DOF],
            oldest_tpdo1_at: None,
            captured_at: None,
        }
    }
}

impl FeedbackSnapshot {
    pub fn all_online_and_fresh(&self) -> bool {
        self.joints
            .iter()
            .all(|joint| joint.online && joint.fresh && joint.fault_code.is_none())
    }
}

#[async_trait]
pub trait MotorBackend: Send + Sync {
    async fn discover(&self, refresh: bool) -> Result<Vec<MotorIdentitySnapshot>>;
    async fn initialize_disabled(&self) -> Result<()>;
    /// Validate firmware encoding/authority using read-back calibration before
    /// any enable or target update. Legacy backends retain their own checks.
    fn validate_targets(&self, _targets: [MotorTarget; DOF]) -> Result<()> {
        Ok(())
    }
    /// Optional total feedback torque envelope in joint-side Nm. Meow's
    /// profile torque_nm bounds host feed-forward only, so its total PD + Tff
    /// envelope comes from the drive limit and read-back factory calibration.
    fn measured_torque_limit_nm(&self, _index: usize) -> Option<f32> {
        None
    }
    async fn enable_compressed_mit(&self, initial_targets: [MotorTarget; DOF]) -> Result<()>;
    async fn set_targets(&self, targets: [MotorTarget; DOF]) -> Result<()>;
    async fn disable_all(&self) -> Result<()>;
    /// Orderly process exit. Real hardware must keep producing the host
    /// heartbeat until every drive is disabled and its 0x1016 consumer is
    /// explicitly disarmed.
    async fn shutdown(&self) -> Result<()>;
    async fn clear_faults(&self) -> Result<()>;
    fn feedback(&self) -> FeedbackSnapshot;
    fn transport_failed(&self) -> bool;
}
