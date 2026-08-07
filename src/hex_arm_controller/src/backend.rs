use std::array;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use async_trait::async_trait;
use can_transport::gs_usb::{GsUsbBus, GsUsbConfig};
use can_transport::{CanBus, CanFrame};
use hex_motor::cia402::{self, Cia402Manager, Cia402ManagerOptions, CompressedMitTarget, Logic};
use parking_lot::RwLock;
use tokio::time::MissedTickBehavior;

use crate::conversion::MotorTarget;
use crate::profile::HardwareProfile;

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
    pub temperature_c: f32,
    pub online: bool,
    pub fresh: bool,
    pub fault_code: Option<u16>,
}

#[derive(Debug, Clone)]
pub struct FeedbackSnapshot {
    pub joints: [JointFeedback; DOF],
    pub captured_at: Option<Instant>,
}

impl Default for FeedbackSnapshot {
    fn default() -> Self {
        Self {
            joints: [JointFeedback::default(); DOF],
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
    async fn enable_compressed_mit(&self) -> Result<()>;
    async fn set_targets(&self, targets: [MotorTarget; DOF]) -> Result<()>;
    async fn disable_all(&self) -> Result<()>;
    async fn clear_faults(&self) -> Result<()>;
    fn feedback(&self) -> FeedbackSnapshot;
    fn transport_failed(&self) -> bool;
}

pub struct MockBackend {
    targets: RwLock<[MotorTarget; DOF]>,
    feedback: RwLock<FeedbackSnapshot>,
    enabled: AtomicBool,
}

impl Default for MockBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl MockBackend {
    pub fn new() -> Self {
        let mut feedback = FeedbackSnapshot::default();
        for joint in &mut feedback.joints {
            joint.online = true;
            joint.fresh = true;
        }
        Self {
            targets: RwLock::new([MotorTarget::default(); DOF]),
            feedback: RwLock::new(feedback),
            enabled: AtomicBool::new(false),
        }
    }
}

#[async_trait]
impl MotorBackend for MockBackend {
    async fn discover(&self, _refresh: bool) -> Result<Vec<MotorIdentitySnapshot>> {
        Ok((0..DOF)
            .map(|index| MotorIdentitySnapshot {
                node_id: (index + 1) as u8,
                vendor_id: 0x0068_6578,
                product_code: 0xAAAA_0002,
                revision: 1,
                serial_number: (index + 1) as u32,
                model: "mock HexMeow Motor".into(),
                identity_verified: true,
            })
            .collect())
    }

    async fn initialize_disabled(&self) -> Result<()> {
        Ok(())
    }

    async fn enable_compressed_mit(&self) -> Result<()> {
        self.enabled.store(true, Ordering::Release);
        Ok(())
    }

    async fn set_targets(&self, targets: [MotorTarget; DOF]) -> Result<()> {
        *self.targets.write() = targets;
        if self.enabled.load(Ordering::Acquire) {
            let mut feedback = self.feedback.write();
            for (joint, target) in feedback.joints.iter_mut().zip(targets) {
                joint.position_rev += (target.position_rev - joint.position_rev) * 0.08;
                joint.velocity_rev_s = target.velocity_rev_s;
                joint.torque_nm = target.torque_nm;
                joint.fresh = true;
                joint.online = true;
            }
            feedback.captured_at = Some(Instant::now());
        }
        Ok(())
    }

    async fn disable_all(&self) -> Result<()> {
        self.enabled.store(false, Ordering::Release);
        Ok(())
    }

    async fn clear_faults(&self) -> Result<()> {
        Ok(())
    }
    fn feedback(&self) -> FeedbackSnapshot {
        self.feedback.read().clone()
    }
    fn transport_failed(&self) -> bool {
        false
    }
}

pub struct RealBackend {
    profile: Arc<HardwareProfile>,
    manager: Arc<Cia402Manager>,
    targets: Arc<RwLock<[CompressedMitTarget; DOF]>>,
    transport_failed: Arc<AtomicBool>,
}

impl RealBackend {
    pub async fn open(profile: Arc<HardwareProfile>) -> Result<Self> {
        let config = GsUsbConfig::fd_1m_5m()
            .with_channel(profile.bus.channel)
            .with_hw_timestamp(profile.bus.hardware_timestamp);
        let bus = Arc::new(
            GsUsbBus::open_vid_pid(profile.bus.adapter_vid, profile.bus.adapter_pid, config)
                .await
                .context("open userspace gs_usb adapter at 1M/5M CAN-FD")?,
        );
        anyhow::ensure!(
            bus.capabilities().fd,
            "selected gs_usb adapter does not report CAN-FD support"
        );

        let options = Cia402ManagerOptions {
            heartbeat_node_id: profile.bus.heartbeat_node_id,
            initialized_stale_threshold: profile.feedback_timeout(),
            ..Default::default()
        };
        let manager = Arc::new(Cia402Manager::new(bus.clone(), options)?);
        let targets = Arc::new(RwLock::new([CompressedMitTarget::ZERO; DOF]));
        let mappings: Arc<Vec<_>> = Arc::new(
            profile
                .joints
                .iter()
                .map(|joint| joint.compressed_mapping())
                .collect(),
        );
        let transport_failed = Arc::new(AtomicBool::new(false));

        {
            let bus: Arc<dyn CanBus> = bus;
            let targets = targets.clone();
            let mappings = mappings.clone();
            let failed = transport_failed.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_micros(1000));
                interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
                loop {
                    interval.tick().await;
                    let payload =
                        cia402::compressed_mit::pack_shared_frame(&*targets.read(), &mappings);
                    let frame =
                        match CanFrame::new_fd(cia402::DEFAULT_SHARED_COB_ID, &payload, true) {
                            Ok(frame) => frame,
                            Err(_) => {
                                failed.store(true, Ordering::Release);
                                break;
                            }
                        };
                    match tokio::time::timeout(Duration::from_millis(10), bus.send(frame)).await {
                        Ok(Ok(())) => {}
                        _ => {
                            failed.store(true, Ordering::Release);
                            break;
                        }
                    }
                }
            });
        }

        Ok(Self {
            profile,
            manager,
            targets,
            transport_failed,
        })
    }

    fn verify_identity(&self, node_id: u8, actual: &hex_motor::types::MotorIdentity) -> bool {
        let expected = self
            .profile
            .joints
            .iter()
            .find(|joint| joint.node_id == node_id);
        expected.is_some_and(|joint| {
            actual.vendor_id == joint.identity.vendor_id
                && actual.product_code == joint.identity.product_code
                && actual.revision_number == joint.identity.revision
                && actual.serial_number == joint.identity.serial_number
        })
    }
}

#[async_trait]
impl MotorBackend for RealBackend {
    async fn discover(&self, refresh: bool) -> Result<Vec<MotorIdentitySnapshot>> {
        if refresh {
            let nodes: Vec<_> = self
                .manager
                .list()
                .into_iter()
                .map(|motor| motor.node_id)
                .collect();
            for node in nodes {
                let _ = self.manager.identify(node).await;
            }
        }
        Ok(self
            .manager
            .list()
            .into_iter()
            .map(|motor| {
                let identity_verified = motor
                    .identity
                    .as_ref()
                    .is_some_and(|identity| self.verify_identity(motor.node_id, identity));
                MotorIdentitySnapshot {
                    node_id: motor.node_id,
                    vendor_id: motor
                        .identity
                        .as_ref()
                        .map_or(0, |identity| identity.vendor_id),
                    product_code: motor
                        .identity
                        .as_ref()
                        .map_or(0, |identity| identity.product_code),
                    revision: motor
                        .identity
                        .as_ref()
                        .map_or(0, |identity| identity.revision_number),
                    serial_number: motor
                        .identity
                        .as_ref()
                        .map_or(0, |identity| identity.serial_number),
                    model: motor.friendly_name(),
                    identity_verified,
                }
            })
            .collect())
    }

    async fn initialize_disabled(&self) -> Result<()> {
        let deadline =
            Instant::now() + Duration::from_millis(self.profile.controller.discovery_timeout_ms);
        loop {
            let motors = self.discover(false).await?;
            let ready = self.profile.joints.iter().all(|joint| {
                motors
                    .iter()
                    .any(|motor| motor.node_id == joint.node_id && motor.identity_verified)
            });
            if ready {
                break;
            }
            anyhow::ensure!(
                Instant::now() < deadline,
                "six expected motor identities were not discovered before timeout"
            );
            tokio::time::sleep(Duration::from_millis(50)).await;
        }

        for joint in &self.profile.joints {
            self.manager
                .initialize(joint.node_id)
                .await
                .with_context(|| format!("initialize CANopen node {}", joint.node_id))?;
            self.manager
                .disable(joint.node_id)
                .await
                .with_context(|| format!("keep CANopen node {} disabled", joint.node_id))?;
        }
        Ok(())
    }

    async fn enable_compressed_mit(&self) -> Result<()> {
        *self.targets.write() = [CompressedMitTarget::ZERO; DOF];
        let bus = self.manager.bus();
        for (slot, joint) in self.profile.joints.iter().enumerate() {
            cia402::compressed_mit::configure(
                bus.as_ref(),
                joint.node_id,
                slot as u8,
                DOF as u8,
                cia402::DEFAULT_SHARED_COB_ID,
                &joint.compressed_mapping(),
                joint.torque_permille,
                joint.kp_kd_torque_permille,
                Some(self.manager.options().sdo_timeout),
            )
            .await
            .with_context(|| format!("configure compressed MIT for {}", joint.name))?;
        }
        Ok(())
    }

    async fn set_targets(&self, targets: [MotorTarget; DOF]) -> Result<()> {
        let packed = targets.map(|target| CompressedMitTarget {
            position: target.position_rev,
            velocity: target.velocity_rev_s,
            torque: target.torque_nm,
            kp: target.kp_nm_rev,
            kd: target.kd_nm_s_rev,
        });
        *self.targets.write() = packed;
        Ok(())
    }

    async fn disable_all(&self) -> Result<()> {
        *self.targets.write() = [CompressedMitTarget::ZERO; DOF];
        let mut errors = Vec::new();
        for joint in &self.profile.joints {
            if let Err(error) = self.manager.disable(joint.node_id).await {
                errors.push(format!("{}: {error}", joint.name));
            }
        }
        anyhow::ensure!(errors.is_empty(), "disable failures: {}", errors.join(", "));
        Ok(())
    }

    async fn clear_faults(&self) -> Result<()> {
        for joint in &self.profile.joints {
            self.manager
                .clear_error(joint.node_id)
                .await
                .with_context(|| format!("clear fault on {}", joint.name))?;
        }
        Ok(())
    }

    fn feedback(&self) -> FeedbackSnapshot {
        let now = Instant::now();
        let mut oldest_tpdo = Some(now);
        let joints = array::from_fn(|index| {
            let node = self.profile.joints[index].node_id;
            let status = self.manager.status(node);
            let last_tpdo = status.connection.last_tpdo;
            oldest_tpdo = match (oldest_tpdo, last_tpdo) {
                (Some(oldest), Some(stamp)) => Some(oldest.min(stamp)),
                _ => None,
            };
            let measurement = status.measurements;
            JointFeedback {
                position_rev: measurement.position_rev.unwrap_or_default(),
                velocity_rev_s: measurement.velocity_rev_per_s.unwrap_or_default(),
                torque_nm: measurement.torque_nm.unwrap_or_default(),
                temperature_c: measurement
                    .motor_temp_c
                    .or(measurement.driver_temp_c)
                    .unwrap_or_default(),
                online: status.connection.online,
                fresh: last_tpdo.is_some_and(|stamp| {
                    now.duration_since(stamp) <= self.profile.feedback_timeout()
                }),
                fault_code: match status.logic {
                    Some(Logic::Error { raw_code, .. }) => Some(raw_code),
                    _ => None,
                },
            }
        });
        FeedbackSnapshot {
            joints,
            captured_at: oldest_tpdo,
        }
    }

    fn transport_failed(&self) -> bool {
        self.transport_failed.load(Ordering::Acquire)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn mock_bus_follows_targets_only_when_enabled() {
        let backend = MockBackend::new();
        let mut targets = [MotorTarget::default(); DOF];
        targets[0].position_rev = 1.0;
        backend.set_targets(targets).await.unwrap();
        assert_eq!(backend.feedback().joints[0].position_rev, 0.0);
        backend.enable_compressed_mit().await.unwrap();
        backend.set_targets(targets).await.unwrap();
        assert!(backend.feedback().joints[0].position_rev > 0.0);
    }

    #[test]
    fn compressed_shared_frame_is_exactly_48_bytes() {
        let targets = [CompressedMitTarget::ZERO; DOF];
        let mappings = [hex_motor::cia402::CompressedMitMapping::default(); DOF];
        assert_eq!(
            cia402::compressed_mit::pack_shared_frame(&targets, &mappings).len(),
            48
        );
    }
}
