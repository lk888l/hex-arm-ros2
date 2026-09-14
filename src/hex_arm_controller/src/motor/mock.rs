use super::*;
use parking_lot::RwLock;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

pub struct MockBackend {
    targets: RwLock<[MotorTarget; DOF]>,
    feedback: RwLock<FeedbackSnapshot>,
    enabled: AtomicBool,
    target_failures_remaining: AtomicUsize,
    disable_failures_remaining: AtomicUsize,
    disable_attempts: AtomicUsize,
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
            target_failures_remaining: AtomicUsize::new(0),
            disable_failures_remaining: AtomicUsize::new(0),
            disable_attempts: AtomicUsize::new(0),
        }
    }

    #[cfg(test)]
    pub(crate) fn fail_next_disables(&self, count: usize) {
        self.disable_failures_remaining
            .store(count, Ordering::Release);
    }

    #[cfg(test)]
    pub(crate) fn fail_next_target_updates(&self, count: usize) {
        self.target_failures_remaining
            .store(count, Ordering::Release);
    }

    #[cfg(test)]
    pub(crate) fn disable_attempts(&self) -> usize {
        self.disable_attempts.load(Ordering::Acquire)
    }

    #[cfg(test)]
    pub(crate) fn is_enabled(&self) -> bool {
        self.enabled.load(Ordering::Acquire)
    }

    #[cfg(test)]
    pub(crate) fn targets(&self) -> [MotorTarget; DOF] {
        *self.targets.read()
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

    async fn enable_compressed_mit(&self, initial_targets: [MotorTarget; DOF]) -> Result<()> {
        *self.targets.write() = initial_targets;
        self.enabled.store(true, Ordering::Release);
        Ok(())
    }

    async fn set_targets(&self, targets: [MotorTarget; DOF]) -> Result<()> {
        if self
            .target_failures_remaining
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |remaining| {
                if remaining > 0 {
                    Some(remaining - 1)
                } else {
                    None
                }
            })
            .is_ok()
        {
            anyhow::bail!("injected mock target update failure");
        }
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
            let captured_at = Instant::now();
            feedback.oldest_tpdo1_at = Some(captured_at);
            feedback.captured_at = Some(captured_at);
        }
        Ok(())
    }

    async fn disable_all(&self) -> Result<()> {
        self.disable_attempts.fetch_add(1, Ordering::AcqRel);
        if self
            .disable_failures_remaining
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |remaining| {
                if remaining > 0 {
                    Some(remaining - 1)
                } else {
                    None
                }
            })
            .is_ok()
        {
            anyhow::bail!("injected mock disable failure");
        }
        self.enabled.store(false, Ordering::Release);
        Ok(())
    }

    async fn shutdown(&self) -> Result<()> {
        self.disable_all().await
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
