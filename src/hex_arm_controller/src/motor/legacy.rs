use std::array;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use async_trait::async_trait;
use can_transport::gs_usb::{GsUsbBus, GsUsbConfig};
use can_transport::socketcan::SocketCanBus;
use can_transport::{CanBus, CanFilter, CanFrame, CanId, CanRx};
use hex_motor::cia402::{
    self, Cia402Manager, Cia402ManagerOptions, CompressedMitMapping, CompressedMitTarget,
    DriveDiagnostic, LiveState, Logic,
};
use hex_motor::types::MotorMode;
use parking_lot::{Mutex, RwLock};
use tokio::time::MissedTickBehavior;

use crate::conversion::MotorTarget;
use crate::profile::{BusTransport, HardwareProfile, JointProfile};
use crate::single_turn::{BranchWindow, SingleTurnUnwrapper};
use crate::socketcan_preflight::{
    preflight_socketcan, preflight_socketcan_for_diagnostic, validate_runtime_socketcan,
    HistoricalCanXStatsAcknowledgement,
};

#[cfg(test)]
use super::MockBackend;
use super::{FeedbackSnapshot, JointFeedback, MotorBackend, MotorIdentitySnapshot, DOF};
const MIT_MODE_DISPLAY: u8 = 5;
const POSITION_BRANCH_TOLERANCE_REV: f32 = 1.0e-4;
const POSITION_UNWRAP_STEP_MARGIN_REV: f32 = 1.0e-4;
const DIAGNOSTIC_BASELINE_REPUBLISH_SETTLE: Duration = Duration::from_millis(2);
pub(crate) const J1_FIRST_POSITION_TORQUE_PERMILLE: u16 = 30;
pub(crate) const J1_FIRST_POSITION_KP_KD_TORQUE_PERMILLE: u16 = 20;
pub(crate) const J3_GRAVITY_UNLOAD_TORQUE_PERMILLE: u16 = 30;
pub(crate) const J3_GRAVITY_UNLOAD_KP_KD_TORQUE_PERMILLE: u16 = 20;
pub(crate) const J4_FIRST_POSITION_TORQUE_PERMILLE: u16 = 60;
pub(crate) const J4_FIRST_POSITION_KP_KD_TORQUE_PERMILLE: u16 = 50;
pub(crate) const J4_ASSISTED_POSITION_TORQUE_PERMILLE: u16 = 90;
pub(crate) const J4_ASSISTED_POSITION_KP_KD_TORQUE_PERMILLE: u16 = 50;
pub(crate) const J5_FIRST_POSITION_TORQUE_PERMILLE: u16 = 50;
pub(crate) const J5_FIRST_POSITION_KP_KD_TORQUE_PERMILLE: u16 = 30;
pub(crate) const J6_FIRST_POSITION_TORQUE_PERMILLE: u16 = 50;
pub(crate) const J6_FIRST_POSITION_KP_KD_TORQUE_PERMILLE: u16 = 30;

/// Exact post-quantization contents of the selected drive's compressed-MIT
/// command object.  These words are read, never written, by the diagnostic
/// path after a deliberately distinct RPDO target has been held constant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompressedTargetReadback {
    pub expected_lower: u32,
    pub expected_upper: u32,
    pub actual_lower: u32,
    pub actual_upper: u32,
}

#[derive(Clone)]
struct SharedCommandState {
    targets: Arc<RwLock<[CompressedMitTarget; DOF]>>,
    enabled: Arc<AtomicBool>,
}

impl SharedCommandState {
    fn new() -> Self {
        Self {
            targets: Arc::new(RwLock::new([CompressedMitTarget::ZERO; DOF])),
            enabled: Arc::new(AtomicBool::new(false)),
        }
    }

    fn install_hold_and_enable(&self, targets: [CompressedMitTarget; DOF]) {
        *self.targets.write() = targets;
        self.enabled.store(true, Ordering::Release);
    }

    fn update_targets(&self, targets: [CompressedMitTarget; DOF]) {
        *self.targets.write() = targets;
    }

    fn snapshot_for_send(&self) -> Option<[CompressedMitTarget; DOF]> {
        self.enabled
            .load(Ordering::Acquire)
            .then(|| *self.targets.read())
    }

    fn stop_and_clear(&self) {
        self.enabled.store(false, Ordering::Release);
        *self.targets.write() = [CompressedMitTarget::ZERO; DOF];
    }
}

pub struct RealBackend {
    profile: Arc<HardwareProfile>,
    manager: Arc<Cia402Manager>,
    /// Records the preflight authority used to open this backend. J1's fixed
    /// first-position policy accepts only the ordinary strict-zero open and
    /// rechecks this at both its library entry point and immediately before
    /// its dedicated drive configuration/enable sequence.
    used_historical_can_xstats_acknowledgement: bool,
    shared_commands: SharedCommandState,
    transport_failed: Arc<AtomicBool>,
    /// Set only after strict Operation Enabled and `0x6061=MIT` have both been
    /// confirmed. While set, every later TPDO2 must remain strict OE.
    mit_operation_expected: RwLock<[bool; DOF]>,
    position_unwrappers: Mutex<[Option<SingleTurnUnwrapper>; DOF]>,
    /// Becomes true only after all six drives have completed the disabled
    /// initialization/feedback gate.  Diagnostic shutdown uses this to keep a
    /// cancelled, fully initialized session out of the partial-init cleanup
    /// path, which is allowed to disarm heartbeat consumers one at a time.
    initialization_complete: AtomicBool,
    /// Zero-additive diagnostic hold retained outside the cancellable motion
    /// future.  A signal can therefore restore the last reviewed baseline by
    /// updating the 1 kHz shared sender before any blocking SDO disable.
    diagnostic_baseline: Mutex<Option<DiagnosticBaseline>>,
}

#[derive(Debug, Clone, Copy)]
struct DiagnosticBaseline {
    selected_index: usize,
    targets: [CompressedMitTarget; DOF],
}

impl RealBackend {
    pub async fn open(profile: Arc<HardwareProfile>) -> Result<Self> {
        anyhow::ensure!(
            profile.bus.protocol == crate::profile::MotorProtocol::Cia402,
            "legacy CiA402 backend cannot open a Meow protocol profile"
        );
        Self::open_with_diagnostic_xstats_acknowledgement(profile, None).await
    }

    /// Open the same real backend for the isolated J2 diagnostic, optionally
    /// accepting two exact historical SocketCAN xstats values.  Ordinary
    /// [`Self::open`] has no way to opt into this acknowledgement.
    pub async fn open_for_single_axis_diagnostic(
        profile: Arc<HardwareProfile>,
        acknowledgement: Option<HistoricalCanXStatsAcknowledgement>,
    ) -> Result<Self> {
        Self::open_with_diagnostic_xstats_acknowledgement(profile, acknowledgement).await
    }

    async fn open_with_diagnostic_xstats_acknowledgement(
        profile: Arc<HardwareProfile>,
        acknowledgement: Option<HistoricalCanXStatsAcknowledgement>,
    ) -> Result<Self> {
        anyhow::ensure!(
            acknowledgement.is_none() || profile.bus.transport == BusTransport::SocketCan,
            "historical CAN xstats acknowledgement is valid only for SocketCAN diagnostics"
        );
        let used_historical_can_xstats_acknowledgement = acknowledgement.is_some();
        let mut socketcan_runtime_baseline = None;
        let bus: Arc<dyn CanBus> = match profile.bus.transport {
            BusTransport::GsUsb => {
                let config = GsUsbConfig::fd_1m_5m()
                    .with_channel(profile.bus.channel)
                    .with_hw_timestamp(profile.bus.hardware_timestamp);
                Arc::new(
                    GsUsbBus::open_vid_pid(
                        profile.bus.adapter_vid,
                        profile.bus.adapter_pid,
                        config,
                    )
                    .await
                    .context("open userspace gs_usb adapter at 1M/5M CAN-FD")?,
                )
            }
            BusTransport::SocketCan => {
                let expected_link = profile
                    .bus
                    .expected_link
                    .as_ref()
                    .context("SocketCAN expected_link is required before opening the bus")?;
                let preflight = match acknowledgement {
                    Some(acknowledgement) => preflight_socketcan_for_diagnostic(
                        &profile.bus.interface,
                        expected_link,
                        acknowledgement,
                    ),
                    None => preflight_socketcan(&profile.bus.interface, expected_link),
                };
                socketcan_runtime_baseline = Some(preflight.with_context(|| {
                    format!(
                        "refuse to open SocketCAN interface {} before a clean 1M/4M preflight",
                        profile.bus.interface
                    )
                })?);
                Arc::new(SocketCanBus::open(&profile.bus.interface).with_context(|| {
                    format!(
                        "open SocketCAN interface {}; configure its CAN-FD bitrates before launch",
                        profile.bus.interface
                    )
                })?)
            }
        };
        anyhow::ensure!(
            bus.capabilities().fd,
            "selected CAN transport does not report CAN-FD support"
        );

        let options = Cia402ManagerOptions {
            heartbeat_node_id: profile.bus.heartbeat_node_id,
            initialized_stale_threshold: profile.feedback_timeout(),
            ..Default::default()
        };
        let manager = Arc::new(Cia402Manager::new(bus.clone(), options)?);
        let shared_commands = SharedCommandState::new();
        let mappings: Arc<Vec<_>> = Arc::new(
            profile
                .joints
                .iter()
                .map(|joint| joint.compressed_mapping())
                .collect(),
        );
        let transport_failed = Arc::new(AtomicBool::new(false));

        if let Some(baseline) = socketcan_runtime_baseline {
            let failed = transport_failed.clone();
            let interface = profile.bus.interface.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_secs(1));
                interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
                loop {
                    interval.tick().await;
                    let query_interface = interface.clone();
                    let query_baseline = baseline.clone();
                    let query = tokio::task::spawn_blocking(move || {
                        validate_runtime_socketcan(&query_interface, &query_baseline)
                    });
                    let health = match tokio::time::timeout(Duration::from_millis(500), query).await
                    {
                        Ok(Ok(result)) => result.with_context(|| {
                            format!("SocketCAN runtime inspection failed for {interface}")
                        }),
                        Ok(Err(error)) => Err(anyhow::anyhow!(
                            "SocketCAN runtime inspection worker failed for {interface}: {error}"
                        )),
                        Err(_) => Err(anyhow::anyhow!(
                            "SocketCAN runtime inspection timed out for {interface}"
                        )),
                    };
                    if let Err(error) = health {
                        if !failed.swap(true, Ordering::AcqRel) {
                            tracing::error!(%error, interface, "SocketCAN health monitor latched transport failure");
                        }
                        break;
                    }
                }
            });
        }

        {
            let bus = bus.clone();
            let shared_commands = shared_commands.clone();
            let mappings = mappings.clone();
            let failed = transport_failed.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_micros(1000));
                interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
                loop {
                    interval.tick().await;
                    let Some(targets) = shared_commands.snapshot_for_send() else {
                        continue;
                    };
                    let payload = cia402::compressed_mit::pack_shared_frame(&targets, &mappings);
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
            used_historical_can_xstats_acknowledgement,
            shared_commands,
            transport_failed,
            mit_operation_expected: RwLock::new([false; DOF]),
            position_unwrappers: Mutex::new(array::from_fn(|_| None)),
            initialization_complete: AtomicBool::new(false),
            diagnostic_baseline: Mutex::new(None),
        })
    }

    fn clear_mit_operation_expectations(&self) {
        *self.mit_operation_expected.write() = [false; DOF];
    }

    fn clear_mit_operation_expectation(&self, index: usize) {
        self.mit_operation_expected.write()[index] = false;
    }

    fn expect_mit_operation_on_axis(&self, index: usize) {
        self.mit_operation_expected.write()[index] = true;
    }

    fn ensure_expected_mit_axes_remain_operation_enabled(&self) -> Result<()> {
        let expected = *self.mit_operation_expected.read();
        if !expected.into_iter().any(|value| value) {
            return Ok(());
        }
        let status_words = array::from_fn(|index| {
            self.manager
                .status(self.profile.joints[index].node_id)
                .measurements
                .status_word
        });
        if let Some((index, status_word)) =
            first_expected_mit_axis_not_operation_enabled(&expected, &status_words)
        {
            let joint = &self.profile.joints[index];
            let status = status_word
                .map(|word| format!("0x{word:04X}"))
                .unwrap_or_else(|| "missing".into());
            anyhow::bail!(
                "{} (CANopen node {}) left strict Operation Enabled after MIT mode confirmation; latest TPDO2 status={status}",
                joint.name,
                joint.node_id
            );
        }
        Ok(())
    }

    fn latch_expected_mit_drive_state_failure(&self) {
        if let Err(error) = self.ensure_expected_mit_axes_remain_operation_enabled() {
            if !self.transport_failed.swap(true, Ordering::AcqRel) {
                tracing::error!(%error, "expected MIT drive state was lost; fail-closed disable required");
            }
        }
    }

    fn verify_identity(&self, node_id: u8, actual: &hex_motor::types::MotorIdentity) -> bool {
        let expected_joint = self
            .profile
            .joints
            .iter()
            .find(|joint| joint.node_id == node_id);
        expected_joint.is_some_and(|joint| {
            actual.vendor_id == joint.identity.vendor_id
                && actual.product_code == joint.identity.product_code
                && actual.revision_number == joint.identity.revision
                && actual.serial_number == joint.identity.serial_number
        }) || self.profile.tip_payload.as_ref().is_some_and(|payload| {
            node_id == payload.auxiliary_node_id
                && actual.vendor_id == payload.identity.vendor_id
                && actual.product_code == payload.identity.product_code
                && actual.revision_number == payload.identity.revision
                && actual.serial_number == payload.identity.serial_number
        })
    }

    fn unexpected_node_ids(&self) -> Vec<u8> {
        let expected: std::collections::HashSet<_> = self
            .profile
            .joints
            .iter()
            .map(|joint| joint.node_id)
            .chain(self.profile.bus.auxiliary_node_ids.iter().copied())
            .collect();
        self.manager
            .list()
            .into_iter()
            .filter_map(|motor| (!expected.contains(&motor.node_id)).then_some(motor.node_id))
            .collect()
    }

    fn ensure_no_unexpected_nodes(&self) -> Result<()> {
        let unexpected = self.unexpected_node_ids();
        anyhow::ensure!(
            unexpected.is_empty(),
            "unexpected CANopen nodes discovered: {} (add intentional non-arm devices to bus.auxiliary_node_ids)",
            unexpected
                .iter()
                .map(|node_id| node_id.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        );
        Ok(())
    }

    async fn wait_for_verified_joint_identities(&self) -> Result<()> {
        let deadline =
            Instant::now() + Duration::from_millis(self.profile.controller.discovery_timeout_ms);
        loop {
            let motors = self.discover(false).await?;
            let ready = self.profile.joints.iter().all(|joint| {
                motors
                    .iter()
                    .any(|motor| motor.node_id == joint.node_id && motor.identity_verified)
            });
            if Instant::now() >= deadline {
                anyhow::ensure!(
                    ready,
                    "six expected motor identities were not discovered before timeout"
                );
                validate_required_tip_payload_identity(&self.profile, &motors)?;
                self.ensure_no_unexpected_nodes()?;
                return Ok(());
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    }

    async fn joint_drive_diagnostics(&self) -> Result<[DriveDiagnostic; DOF]> {
        let mut diagnostics = [DriveDiagnostic {
            error_code: 0,
            status_word: 0,
        }; DOF];
        for (index, joint) in self.profile.joints.iter().enumerate() {
            diagnostics[index] = self
                .manager
                .drive_diagnostic(joint.node_id)
                .await
                .with_context(|| {
                    format!(
                        "read authoritative fault/status words from {} (node {})",
                        joint.name, joint.node_id
                    )
                })?;
        }
        Ok(diagnostics)
    }

    /// Clear a deliberately selected set of current host-heartbeat faults. This
    /// path never initializes, configures RPDO, changes mode, or enables a
    /// drive. The caller must still run the final heartbeat-disarm cleanup on
    /// every exit path before this backend is dropped.
    pub async fn recover_heartbeat_lost(&self, requested_nodes: &BTreeSet<u8>) -> Result<()> {
        self.wait_for_verified_joint_identities().await?;
        self.ensure_no_unexpected_nodes()?;

        let before = self.joint_drive_diagnostics().await?;
        validate_heartbeat_recovery_selection(&self.profile, requested_nodes, &before)?;
        let bus = self.manager.bus();
        let mut emcy_rx = bus
            .subscribe(CanFilter::standard(0x080, 0x780))
            .await
            .context("subscribe to arm EMCY frames before heartbeat fault reset")?;
        let controlled_nodes: BTreeSet<_> = self
            .profile
            .joints
            .iter()
            .map(|joint| joint.node_id)
            .collect();

        let reset_and_confirm = async {
            for node_id in requested_nodes {
                self.manager
                    .recover_heartbeat_lost_disabled(*node_id)
                    .await
                    .with_context(|| {
                        format!("explicitly clear heartbeat-lost fault on CANopen node {node_id}")
                    })?;
            }

            // 0x603F can retain 0x8130 as last-error history after a successful
            // reset. Current fault state is authoritative only in a TPDO2 status
            // word strictly newer than every reset request.
            let reset_completed_at = Instant::now();
            self.wait_for_post_recovery_status(reset_completed_at)
                .await?;
            let after = self.joint_drive_diagnostics().await?;
            validate_recovery_result_disabled_fault_free(&self.profile, &after)?;

            // Establish a new, explicit all-axis CiA402 Shutdown state. The final
            // CLI cleanup may remove 0x1016 only after a post-command TPDO2 proves
            // every axis stayed non-OE.
            for joint in &self.profile.joints {
                self.manager
                    .disable_identified(joint.node_id)
                    .await
                    .with_context(|| {
                        format!(
                            "request disabled state on {} (node {}) after heartbeat recovery",
                            joint.name, joint.node_id
                        )
                    })?;
            }
            let disable_completed_at = Instant::now();
            self.wait_for_disabled_feedback(disable_completed_at)
                .await
                .context("post-recovery all-axis disable was not confirmed")?;
            Ok(())
        };

        tokio::select! {
            biased;
            emcy = wait_for_controlled_emcy(emcy_rx.as_mut(), &controlled_nodes) => emcy,
            result = reset_and_confirm => result,
        }
    }

    async fn wait_for_post_recovery_status(&self, reset_completed_at: Instant) -> Result<()> {
        let deadline = Instant::now() + self.manager.options().mode_confirm_timeout;
        loop {
            let mut pending = Vec::new();
            for joint in &self.profile.joints {
                let status = self.manager.status(joint.node_id);
                let tpdo2_new = status
                    .connection
                    .last_tpdo2
                    .is_some_and(|stamp| stamp > reset_completed_at);

                if tpdo2_new {
                    let status_word = status.measurements.status_word.context(format!(
                        "{} post-reset TPDO2 has no status word",
                        joint.name
                    ))?;
                    anyhow::ensure!(
                        !cia402::codec::status_word_has_fault(status_word)
                            && cia402::codec::status_word_is_confirmed_non_torque(status_word),
                        "{} post-reset TPDO2 is Fault or not in a confirmed non-torque state (0x6041=0x{status_word:04X})",
                        joint.name
                    );
                }

                if !tpdo2_new {
                    pending.push(joint.name.as_str());
                }
            }
            if pending.is_empty() {
                return Ok(());
            }
            anyhow::ensure!(
                Instant::now() < deadline,
                "post-reset TPDO2 status confirmation timed out for: {}",
                pending.join(", ")
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    /// While the host heartbeat broadcaster is still running, disarm its
    /// consumer on all six arm nodes and independently prove each drive is not
    /// OperationEnabled. Auxiliary devices are intentionally untouched.
    pub async fn disarm_heartbeat_consumers_confirmed_disabled(&self) -> Result<()> {
        let mut errors = Vec::new();
        for joint in &self.profile.joints {
            if let Err(error) = self
                .manager
                .disarm_consumer_heartbeat_disabled(joint.node_id)
                .await
            {
                errors.push(format!("{} (node {}): {error}", joint.name, joint.node_id));
            }
        }
        if errors.is_empty() {
            self.shared_commands.stop_and_clear();
            Ok(())
        } else {
            anyhow::bail!(
                "heartbeat consumer disarm/disabled confirmation failures: {}",
                errors.join(", ")
            )
        }
    }

    pub async fn disarm_heartbeat_consumers_with_retry(&self, attempts: usize) -> Result<()> {
        anyhow::ensure!(
            attempts > 0,
            "heartbeat disarm retry count must be positive"
        );
        let mut errors = Vec::new();
        for attempt in 1..=attempts {
            match self.disarm_heartbeat_consumers_confirmed_disabled().await {
                Ok(()) => return Ok(()),
                Err(error) => {
                    errors.push(format!("attempt {attempt}: {error:#}"));
                    if attempt < attempts {
                        tokio::time::sleep(Duration::from_millis(50)).await;
                    }
                }
            }
        }
        anyhow::bail!(
            "heartbeat consumer disarm remained unconfirmed: {}",
            errors.join("; ")
        )
    }

    /// Clean only 0x1016 consumers that this process attempted to arm before
    /// initialization failed or was cancelled.  Each manager call is a no-op
    /// for untouched nodes; touched nodes are independently disabled and
    /// authoritatively proven non-OE before their watchdog is removed.
    async fn cleanup_partial_initialization_heartbeat_consumers(&self) -> Result<()> {
        let mut errors = Vec::new();
        for joint in &self.profile.joints {
            match self
                .manager
                .cleanup_session_heartbeat_consumer_disabled(joint.node_id)
                .await
            {
                Ok(true) => tracing::info!(
                    joint = %joint.name,
                    node_id = joint.node_id,
                    "cleaned heartbeat consumer left by partial initialization"
                ),
                Ok(false) => {}
                Err(error) => {
                    errors.push(format!("{} (node {}): {error}", joint.name, joint.node_id))
                }
            }
        }
        anyhow::ensure!(
            errors.is_empty(),
            "partial-initialization heartbeat cleanup failures: {}",
            errors.join(", ")
        );
        self.shared_commands.stop_and_clear();
        Ok(())
    }

    async fn cleanup_partial_initialization_heartbeat_consumers_with_retry(
        &self,
        attempts: usize,
    ) -> Result<()> {
        anyhow::ensure!(attempts > 0, "partial cleanup retry count must be positive");
        let mut errors = Vec::new();
        for attempt in 1..=attempts {
            match self
                .cleanup_partial_initialization_heartbeat_consumers()
                .await
            {
                Ok(()) => return Ok(()),
                Err(error) => {
                    errors.push(format!("attempt {attempt}: {error:#}"));
                    if attempt < attempts {
                        tokio::time::sleep(Duration::from_millis(50)).await;
                    }
                }
            }
        }
        anyhow::bail!(
            "partial-initialization heartbeat cleanup remained unconfirmed: {}",
            errors.join("; ")
        )
    }

    async fn wait_for_disabled_feedback(&self, command_completed_at: Instant) -> Result<()> {
        let deadline = Instant::now() + self.manager.options().mode_confirm_timeout;
        loop {
            let pending: Vec<_> = self
                .profile
                .joints
                .iter()
                .filter_map(|joint| {
                    let status = self.manager.status(joint.node_id);
                    (!disable_feedback_confirmed(&status, command_completed_at))
                        .then_some(joint.name.as_str())
                })
                .collect();
            if pending.is_empty() {
                return Ok(());
            }
            anyhow::ensure!(
                Instant::now() < deadline,
                "disable was not confirmed by a post-command TPDO2 for: {}",
                pending.join(", ")
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    async fn confirm_compressed_mit_enabled(
        &self,
        node_id: u8,
        command_completed_at: Instant,
    ) -> Result<()> {
        let deadline = Instant::now() + self.manager.options().mode_confirm_timeout;
        loop {
            let status = self.manager.status(node_id);
            if let Some(Logic::Error { raw_code, .. }) = status.logic.as_ref() {
                anyhow::bail!(
                    "node {node_id} entered fault 0x{raw_code:04X} while confirming compressed MIT"
                );
            }
            if operation_enabled_after(&status, command_completed_at) {
                break;
            }
            anyhow::ensure!(
                Instant::now() < deadline,
                "node {node_id} compressed MIT enable was not confirmed by a post-command TPDO2"
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }

        // Default TPDO2 does not carry 0x6061, so read the drive's actual mode
        // explicitly instead of trusting manager.target_mode. This is the
        // authoritative mode-display value used in the combined confirmation.
        let bus = self.manager.bus();
        let mode_display = hex_motor::canopen::sdo::upload_u8(
            bus.as_ref(),
            node_id,
            0x6061,
            0,
            Some(self.manager.options().sdo_timeout),
        )
        .await
        .with_context(|| format!("read mode display 0x6061 from CANopen node {node_id}"))?;
        let status = self.manager.status(node_id);
        anyhow::ensure!(
            compressed_mit_enable_confirmed(&status, command_completed_at, Some(mode_display)),
            "node {node_id} compressed MIT confirmation mismatch: mode_display={mode_display}, status_word={:?}",
            status.measurements.status_word
        );
        let status_word = status
            .measurements
            .status_word
            .context("confirmed compressed MIT status has no TPDO2 status word")?;
        let control_word_readback = status
            .measurements
            .control_word_readback
            .context("confirmed compressed MIT status has no TPDO2 control-word readback")?;
        tracing::info!(
            node_id,
            status_word = %format_args!("0x{status_word:04X}"),
            mode_display,
            control_word_readback = %format_args!("0x{control_word_readback:04X}"),
            "compressed MIT enable confirmed by post-command TPDO2 and mode display"
        );
        Ok(())
    }

    fn prepare_single_axis_operation(
        &self,
        selected_index: usize,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        self.clear_mit_operation_expectations();
        anyhow::ensure!(
            selected_index < DOF,
            "commissioning joint index {selected_index} is outside 0..{DOF}"
        );
        self.ensure_no_unexpected_nodes()?;
        anyhow::ensure!(
            !self.transport_failed.load(Ordering::Acquire),
            "CAN transport is already in a failed state"
        );
        self.profile.validate_single_turn_command_windows()?;
        self.ensure_all_axes_disabled_for_commissioning(selected_index)?;
        for (index, joint) in self.profile.joints.iter().enumerate() {
            validate_compressed_target(
                &initial_targets[index],
                &joint.compressed_mapping(),
                &joint.name,
            )?;
        }

        let packed = initial_targets.map(compressed_target);
        // This ordering is a safety invariant: a fully populated hold frame is
        // live before the selected node's RPDO is configured or enabled.
        self.shared_commands.install_hold_and_enable(packed);

        Ok(())
    }

    async fn begin_single_axis_operation(
        &self,
        selected_index: usize,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        self.prepare_single_axis_operation(selected_index, initial_targets)?;

        activate_only_selected_axis(self, selected_index, initial_targets[selected_index]).await
    }

    /// Enable exactly one joint for the deliberately narrow commissioning
    /// path. All six slots contain feedback-derived hold targets before the
    /// shared sender is released, but only the selected drive is configured
    /// to consume the frame or asked to enter MIT mode.
    pub async fn enable_commissioning_axis(
        &self,
        selected_index: usize,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        let activation = self
            .begin_single_axis_operation(selected_index, initial_targets)
            .await;
        if let Err(activation_error) = activation {
            return match self.disable_all_with_retry(3).await {
                Ok(()) => Err(activation_error),
                Err(rollback_error) => Err(anyhow::anyhow!(
                    "{activation_error:#}; confirmed rollback disable also failed after retries: {rollback_error:#}"
                )),
            };
        }
        Ok(())
    }

    /// Explicit supported commissioning only. Configure and read back all
    /// requested caps while disabled; install holds before any enable. The
    /// caller monitors feedback throughout this future and always shuts down.
    pub async fn enable_supported_commissioning_axes(
        &self,
        order: &[usize],
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        let mask = supported_axis_mask(order)?;
        anyhow::ensure!(
            !self.used_historical_can_xstats_acknowledgement,
            "supported commissioning requires strict-zero CAN preflight"
        );
        self.prepare_single_axis_operation(order[0], initial_targets)?;
        for &index in order {
            self.configure_commissioning_axis(index, initial_targets[index])
                .await?;
            let joint = &self.profile.joints[index];
            self.confirm_fixed_position_drive_caps(
                index,
                "supported commissioning",
                joint.torque_permille,
                joint.kp_kd_torque_permille,
            )
            .await?;
        }
        let mut expected = [false; DOF];
        for &index in order {
            self.ensure_supported_commissioning_state(expected)?;
            anyhow::ensure!(
                !self.transport_failed(),
                "CAN failed during group activation"
            );
            let node = self.profile.joints[index].node_id;
            self.manager.set_mode(node, MotorMode::Mit).await?;
            self.confirm_compressed_mit_enabled(node, Instant::now())
                .await?;
            self.expect_mit_operation_on_axis(index);
            expected[index] = true;
            self.ensure_supported_commissioning_state(expected)?;
        }
        self.ensure_supported_commissioning_state(mask)
    }

    pub fn ensure_supported_commissioning_state(&self, active: [bool; DOF]) -> Result<()> {
        anyhow::ensure!(
            *self.mit_operation_expected.read() == active,
            "supported commissioning enable mask changed"
        );
        self.ensure_no_unexpected_nodes()?;
        let now = Instant::now();
        let statuses: [_; DOF] =
            array::from_fn(|i| self.manager.status(self.profile.joints[i].node_id));
        for (joint, status) in self.profile.joints.iter().zip(&statuses) {
            anyhow::ensure!(
                status
                    .connection
                    .required_tpdos_fresh(now, self.profile.feedback_timeout())
                    && !matches!(status.logic, Some(Logic::Error { .. })),
                "{} has stale or faulted group feedback",
                joint.name
            );
        }
        validate_supported_status_words(active, statuses.map(|s| s.measurements.status_word))
    }

    /// Diagnostic activation deliberately leaves every error/cancellation to
    /// the outer selected-first guard. Calling the ordinary rollback here
    /// would issue J1's Shutdown before J2 and violate the high-tier contract.
    pub async fn enable_diagnostic_axis(
        &self,
        selected_index: usize,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        self.begin_single_axis_operation(selected_index, initial_targets)
            .await
    }

    /// Configure and enable only J1 for the compile-time-fixed first-position
    /// diagnostic. The drive-side limits are deliberately lower than the
    /// profile values and are uploaded bit-for-bit while every drive is still
    /// disabled. No failure in either write or readback can reach set_mode.
    ///
    /// These conservative OD values remain on the disabled drive after the
    /// diagnostic. The in-memory/on-disk profile is never changed; a later
    /// ordinary configure call rewrites its own reviewed profile values.
    pub async fn enable_joint1_first_position_diagnostic_axis(
        &self,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        const SELECTED_INDEX: usize = 0;
        self.ensure_joint1_first_position_strict_zero_open()?;
        self.prepare_single_axis_operation(SELECTED_INDEX, initial_targets)?;
        activate_joint1_first_position_axis(
            self,
            initial_targets[SELECTED_INDEX],
            J1_FIRST_POSITION_TORQUE_PERMILLE,
            J1_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
    }

    /// Configure and enable only J3 for its fixed-position gravity-unload
    /// identification. Conservative 30/20-permille torque/PD caps are read
    /// back while all axes are disabled.
    pub async fn enable_joint3_gravity_unload_diagnostic_axis(
        &self,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        const SELECTED_INDEX: usize = 2;
        self.ensure_joint3_gravity_unload_strict_zero_open()?;
        self.prepare_single_axis_operation(SELECTED_INDEX, initial_targets)?;
        self.configure_commissioning_axis_with_caps(
            SELECTED_INDEX,
            initial_targets[SELECTED_INDEX],
            J3_GRAVITY_UNLOAD_TORQUE_PERMILLE,
            J3_GRAVITY_UNLOAD_KP_KD_TORQUE_PERMILLE,
        )
        .await?;
        self.confirm_fixed_position_drive_caps(
            SELECTED_INDEX,
            "joint_3 gravity-unload diagnostic",
            J3_GRAVITY_UNLOAD_TORQUE_PERMILLE,
            J3_GRAVITY_UNLOAD_KP_KD_TORQUE_PERMILLE,
        )
        .await?;
        self.enable_and_confirm_commissioning_axis(SELECTED_INDEX)
            .await
    }

    /// Configure and enable only J5 for its compile-time-fixed first-motion
    /// survey. Fixed conservative caps are read back while all axes remain
    /// disabled; no write/readback failure can reach the enable call.
    pub async fn enable_joint5_first_position_diagnostic_axis(
        &self,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        const SELECTED_INDEX: usize = 4;
        self.ensure_joint5_first_position_strict_zero_open()?;
        self.prepare_single_axis_operation(SELECTED_INDEX, initial_targets)?;
        activate_joint5_first_position_axis(
            self,
            initial_targets[SELECTED_INDEX],
            J5_FIRST_POSITION_TORQUE_PERMILLE,
            J5_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
    }

    /// Configure and enable only J4 for its compile-time-fixed first-motion
    /// survey. The fixed conservative caps are read back before enable.
    pub async fn enable_joint4_first_position_diagnostic_axis(
        &self,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        const SELECTED_INDEX: usize = 3;
        self.ensure_joint4_first_position_strict_zero_open()?;
        self.prepare_single_axis_operation(SELECTED_INDEX, initial_targets)?;
        activate_joint4_first_position_axis(
            self,
            initial_targets[SELECTED_INDEX],
            J4_FIRST_POSITION_TORQUE_PERMILLE,
            J4_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
    }

    /// Configure and enable only J4 for the independently authorized
    /// trajectory-synchronous assistance survey. The still-bounded 90/50
    /// caps are still fixed in code and are proved by SDO readback before OE.
    pub async fn enable_joint4_assisted_position_diagnostic_axis(
        &self,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        const SELECTED_INDEX: usize = 3;
        self.ensure_joint4_first_position_strict_zero_open()?;
        self.prepare_single_axis_operation(SELECTED_INDEX, initial_targets)?;
        activate_joint4_assisted_position_axis(
            self,
            initial_targets[SELECTED_INDEX],
            J4_ASSISTED_POSITION_TORQUE_PERMILLE,
            J4_ASSISTED_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
    }

    /// Configure and enable only J6 for its compile-time-fixed first-motion
    /// survey. The conservative 50/30 limits are proved by SDO readback while
    /// every axis is disabled; no failure can reach the enable operation.
    pub async fn enable_joint6_first_position_diagnostic_axis(
        &self,
        initial_targets: [MotorTarget; DOF],
    ) -> Result<()> {
        const SELECTED_INDEX: usize = 5;
        self.ensure_joint6_first_position_strict_zero_open()?;
        self.prepare_single_axis_operation(SELECTED_INDEX, initial_targets)?;
        activate_joint6_first_position_axis(
            self,
            initial_targets[SELECTED_INDEX],
            J6_FIRST_POSITION_TORQUE_PERMILLE,
            J6_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
    }

    /// Prove that this backend was opened without the J2-only historical CAN
    /// xstats acknowledgement. This is a library-level authority boundary:
    /// a caller cannot bypass the independent J1 CLI by constructing its
    /// fixed request against an acknowledged-history backend.
    pub(crate) fn ensure_joint1_first_position_strict_zero_open(&self) -> Result<()> {
        validate_joint1_first_position_strict_zero_open(
            self.used_historical_can_xstats_acknowledgement,
        )
    }

    /// Bind J1's fixed policy to the exact profile allocation that was used
    /// to build this backend's motor mappings and limits. Value equality is
    /// insufficient because a caller could otherwise construct targets from
    /// a distinct profile object after the hardware session was opened.
    pub(crate) fn ensure_joint1_first_position_strict_session(
        &self,
        supplied_profile: &HardwareProfile,
    ) -> Result<()> {
        validate_joint1_first_position_strict_session(
            self.used_historical_can_xstats_acknowledgement,
            self.profile.as_ref(),
            supplied_profile,
        )
    }

    pub(crate) fn ensure_joint5_first_position_strict_zero_open(&self) -> Result<()> {
        validate_joint5_first_position_strict_zero_open(
            self.used_historical_can_xstats_acknowledgement,
        )
    }

    pub(crate) fn ensure_joint4_first_position_strict_zero_open(&self) -> Result<()> {
        validate_joint4_first_position_strict_zero_open(
            self.used_historical_can_xstats_acknowledgement,
        )
    }

    pub(crate) fn ensure_joint3_gravity_unload_strict_zero_open(&self) -> Result<()> {
        validate_joint3_gravity_unload_strict_zero_open(
            self.used_historical_can_xstats_acknowledgement,
        )
    }

    pub(crate) fn ensure_joint3_gravity_unload_strict_session(
        &self,
        supplied_profile: &HardwareProfile,
    ) -> Result<()> {
        validate_joint3_gravity_unload_strict_session(
            self.used_historical_can_xstats_acknowledgement,
            self.profile.as_ref(),
            supplied_profile,
        )
    }

    pub(crate) fn ensure_joint4_first_position_strict_session(
        &self,
        supplied_profile: &HardwareProfile,
    ) -> Result<()> {
        validate_joint4_first_position_strict_session(
            self.used_historical_can_xstats_acknowledgement,
            self.profile.as_ref(),
            supplied_profile,
        )
    }

    pub(crate) fn ensure_joint5_first_position_strict_session(
        &self,
        supplied_profile: &HardwareProfile,
    ) -> Result<()> {
        validate_joint5_first_position_strict_session(
            self.used_historical_can_xstats_acknowledgement,
            self.profile.as_ref(),
            supplied_profile,
        )
    }

    pub(crate) fn ensure_joint6_first_position_strict_zero_open(&self) -> Result<()> {
        validate_joint6_first_position_strict_zero_open(
            self.used_historical_can_xstats_acknowledgement,
        )
    }

    pub(crate) fn ensure_joint6_first_position_strict_session(
        &self,
        supplied_profile: &HardwareProfile,
    ) -> Result<()> {
        validate_joint6_first_position_strict_session(
            self.used_historical_can_xstats_acknowledgement,
            self.profile.as_ref(),
            supplied_profile,
        )
    }

    /// Retain a zero-additive diagnostic hold outside the cancellable motion
    /// future. This performs no CAN I/O. The caller installs the first
    /// feedback-derived baseline before enable and may replace it with the
    /// post-stability baseline before publishing any torque staircase level.
    pub fn register_single_axis_diagnostic_baseline(
        &self,
        selected_index: usize,
        targets: [MotorTarget; DOF],
    ) -> Result<()> {
        anyhow::ensure!(
            selected_index < DOF,
            "diagnostic baseline joint index {selected_index} is outside 0..{DOF}"
        );
        for (index, joint) in self.profile.joints.iter().enumerate() {
            validate_compressed_target(&targets[index], &joint.compressed_mapping(), &joint.name)?;
        }
        *self.diagnostic_baseline.lock() = Some(DiagnosticBaseline {
            selected_index,
            targets: targets.map(compressed_target),
        });
        Ok(())
    }

    fn restore_registered_diagnostic_baseline(&self, selected_index: usize) -> Result<()> {
        let baseline = *self.diagnostic_baseline.lock();
        match baseline {
            Some(baseline) => {
                anyhow::ensure!(
                    baseline.selected_index == selected_index,
                    "registered diagnostic baseline belongs to joint index {}, not selected index {selected_index}",
                    baseline.selected_index
                );
                // This is intentionally a non-blocking in-memory sender update:
                // signal cleanup must not wait behind an SDO before restoring
                // the latest feedback-verified target. For a censored ramp
                // that target may deliberately retain a bounded nonzero term
                // until the selected drive is confirmed non-torque.
                self.shared_commands.update_targets(baseline.targets);
                tracing::warn!(
                    joint = %self.profile.joints[selected_index].name,
                    "restored registered feedback-verified diagnostic baseline through the shared RPDO sender"
                );
                Ok(())
            }
            None => {
                let expectations = *self.mit_operation_expected.read();
                anyhow::ensure!(
                    !expectations[selected_index],
                    "selected diagnostic axis is expected Operation Enabled but no registered baseline is available"
                );
                // Cancellation immediately after initialize_disabled but before
                // diagnostic feedback/enable legitimately has no baseline.
                Ok(())
            }
        }
    }

    async fn wait_for_axis_disabled_feedback(
        &self,
        index: usize,
        command_completed_at: Instant,
    ) -> Result<()> {
        let joint = &self.profile.joints[index];
        let deadline = Instant::now() + self.manager.options().mode_confirm_timeout;
        loop {
            let status = self.manager.status(joint.node_id);
            if disable_feedback_confirmed(&status, command_completed_at) {
                return Ok(());
            }
            anyhow::ensure!(
                Instant::now() < deadline,
                "{} (node {}) did not provide a post-command TPDO2 confirming a strict non-torque state",
                joint.name,
                joint.node_id
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    async fn disable_axis_confirmed_with_retry(&self, index: usize, attempts: usize) -> Result<()> {
        anyhow::ensure!(
            index < DOF,
            "disable joint index {index} is outside 0..{DOF}"
        );
        anyhow::ensure!(attempts > 0, "disable retry count must be positive");
        self.clear_mit_operation_expectation(index);
        let joint = &self.profile.joints[index];
        let mut errors = Vec::new();
        for attempt in 1..=attempts {
            let result: Result<()> = async {
                self.manager
                    .disable(joint.node_id)
                    .await
                    .with_context(|| format!("send Shutdown to {}", joint.name))?;
                // A cached TPDO2 from during the SDO write cannot satisfy this
                // proof; each attempt requires a strictly post-completion frame.
                let command_completed_at = Instant::now();
                self.wait_for_axis_disabled_feedback(index, command_completed_at)
                    .await
            }
            .await;
            match result {
                Ok(()) => return Ok(()),
                Err(error) => {
                    errors.push(format!("attempt {attempt}: {error:#}"));
                    if attempt < attempts {
                        tokio::time::sleep(Duration::from_millis(50)).await;
                    }
                }
            }
        }
        anyhow::bail!(
            "{} selected-first disable remained unconfirmed: {}",
            joint.name,
            errors.join("; ")
        )
    }

    /// Diagnostic-only shutdown. A fully initialized session must never enter
    /// the partial-initialization fallback, because that path may clear one
    /// heartbeat consumer while the selected axis has not been proven
    /// non-torque. The selected axis is always handled first, all remaining
    /// axes are still attempted after any failure, and 0x1016 is touched only
    /// after all six post-command TPDO2 confirmations succeed.
    pub async fn shutdown_single_axis_diagnostic_selected_first(
        &self,
        selected_index: usize,
    ) -> Result<()> {
        anyhow::ensure!(
            selected_index < DOF,
            "diagnostic shutdown joint index {selected_index} is outside 0..{DOF}"
        );
        if !self.initialization_complete.load(Ordering::Acquire) {
            return MotorBackend::shutdown(self).await;
        }
        run_selected_first_diagnostic_shutdown(self, selected_index).await
    }

    /// Prove that the selected drive consumed the currently held shared RPDO
    /// target by uploading `0x2004:02/03` and comparing the two quantized words
    /// bit-for-bit.  This is intentionally read-only and is only admitted when
    /// exactly the selected commissioning axis is expected to remain in MIT
    /// Operation Enabled.
    pub async fn confirm_commissioning_target_readback(
        &self,
        selected_index: usize,
        expected_target: MotorTarget,
    ) -> Result<CompressedTargetReadback> {
        anyhow::ensure!(
            selected_index < DOF,
            "commissioning readback joint index {selected_index} is outside 0..{DOF}"
        );
        let expectations = *self.mit_operation_expected.read();
        anyhow::ensure!(
            expectations[selected_index]
                && expectations.iter().filter(|expected| **expected).count() == 1,
            "compressed target readback requires exactly the selected commissioning axis to be expected in MIT Operation Enabled"
        );
        self.latch_expected_mit_drive_state_failure();
        anyhow::ensure!(
            !self.transport_failed.load(Ordering::Acquire),
            "CAN transport failed before compressed target readback"
        );
        self.ensure_expected_mit_axes_remain_operation_enabled()?;

        let joint = &self.profile.joints[selected_index];
        let mapping = joint.compressed_mapping();
        validate_compressed_target(&expected_target, &mapping, &joint.name)?;
        let (expected_lower, expected_upper) = cia402::compressed_mit::packed_target_words(
            &compressed_target(expected_target),
            &mapping,
        );
        let bus = self.manager.bus();
        let timeout = Some(self.manager.options().sdo_timeout);
        let actual_lower =
            hex_motor::canopen::sdo::upload_u32(bus.as_ref(), joint.node_id, 0x2004, 0x02, timeout)
                .await
                .with_context(|| {
                    format!(
                        "read compressed-MIT lower target 0x2004:02 from {} (node {})",
                        joint.name, joint.node_id
                    )
                })?;
        let actual_upper =
            hex_motor::canopen::sdo::upload_u32(bus.as_ref(), joint.node_id, 0x2004, 0x03, timeout)
                .await
                .with_context(|| {
                    format!(
                        "read compressed-MIT upper target 0x2004:03 from {} (node {})",
                        joint.name, joint.node_id
                    )
                })?;

        self.latch_expected_mit_drive_state_failure();
        anyhow::ensure!(
            !self.transport_failed.load(Ordering::Acquire),
            "CAN transport failed during compressed target readback"
        );
        self.ensure_expected_mit_axes_remain_operation_enabled()?;
        anyhow::ensure!(
            (actual_lower, actual_upper) == (expected_lower, expected_upper),
            "{} (node {}) did not expose the held RPDO target exactly: expected 0x2004:03/02=0x{expected_upper:08X}{expected_lower:08X}, got 0x{actual_upper:08X}{actual_lower:08X}",
            joint.name,
            joint.node_id
        );
        Ok(CompressedTargetReadback {
            expected_lower,
            expected_upper,
            actual_lower,
            actual_upper,
        })
    }

    /// Check the cached TPDO state contract for isolated single-axis
    /// diagnostics without performing SDO I/O.  The selected joint must remain
    /// strict Operation Enabled; every other arm joint must remain fresh,
    /// fault-free, and in a confirmed drive-function-disabled state.
    pub fn ensure_single_axis_commissioning_state(&self, selected_index: usize) -> Result<()> {
        anyhow::ensure!(
            selected_index < DOF,
            "single-axis state check index {selected_index} is outside 0..{DOF}"
        );
        let expectations = *self.mit_operation_expected.read();
        anyhow::ensure!(
            expectations[selected_index]
                && expectations.iter().filter(|expected| **expected).count() == 1,
            "single-axis diagnostics require exactly the selected joint to be expected in MIT Operation Enabled"
        );
        let now = Instant::now();
        let statuses =
            array::from_fn(|index| self.manager.status(self.profile.joints[index].node_id));
        for (joint, status) in self.profile.joints.iter().zip(&statuses) {
            anyhow::ensure!(
                status
                    .connection
                    .required_tpdos_fresh(now, self.profile.feedback_timeout()),
                "{} (node {}) TPDO state is not fresh during single-axis diagnostics",
                joint.name,
                joint.node_id
            );
            anyhow::ensure!(
                !matches!(status.logic, Some(Logic::Error { .. })),
                "{} (node {}) faulted during single-axis diagnostics",
                joint.name,
                joint.node_id
            );
            anyhow::ensure!(
                status
                    .measurements
                    .status_word
                    .is_some_and(|word| !cia402::codec::status_word_has_fault(word)),
                "{} (node {}) has a missing or faulted status word during single-axis diagnostics",
                joint.name,
                joint.node_id
            );
        }
        let status_words = statuses.map(|status| status.measurements.status_word);
        validate_single_axis_commissioning_status_words(
            &self.profile,
            selected_index,
            &status_words,
        )
    }

    /// Retry the normal all-axis confirmed disable sequence. The shared
    /// command sender remains active until a disable is positively confirmed,
    /// so a transient SDO/TPDO failure never turns into an uncommanded drop of
    /// stiffness while the process is still alive.
    pub async fn disable_all_with_retry(&self, attempts: usize) -> Result<()> {
        anyhow::ensure!(attempts > 0, "disable retry count must be positive");
        let mut errors = Vec::new();
        for attempt in 1..=attempts {
            match MotorBackend::disable_all(self).await {
                Ok(()) => return Ok(()),
                Err(error) => {
                    errors.push(format!("attempt {attempt}: {error:#}"));
                    if attempt < attempts {
                        tokio::time::sleep(Duration::from_millis(50)).await;
                    }
                }
            }
        }
        anyhow::bail!("disable remained unconfirmed: {}", errors.join("; "))
    }

    async fn configure_commissioning_axis_with_caps(
        &self,
        selected_index: usize,
        initial_target: MotorTarget,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()> {
        let joint = &self.profile.joints[selected_index];
        cia402::compressed_mit::configure(
            self.manager.bus().as_ref(),
            joint.node_id,
            selected_index as u8,
            DOF as u8,
            cia402::DEFAULT_SHARED_COB_ID,
            &joint.compressed_mapping(),
            &compressed_target(initial_target),
            torque_permille,
            kp_kd_torque_permille,
            Some(self.manager.options().sdo_timeout),
        )
        .await
        .with_context(|| {
            format!(
                "configure selected commissioning axis {} (CANopen node {})",
                joint.name, joint.node_id
            )
        })
    }

    async fn configure_commissioning_axis(
        &self,
        selected_index: usize,
        initial_target: MotorTarget,
    ) -> Result<()> {
        let joint = &self.profile.joints[selected_index];
        self.configure_commissioning_axis_with_caps(
            selected_index,
            initial_target,
            joint.torque_permille,
            joint.kp_kd_torque_permille,
        )
        .await
    }

    async fn confirm_fixed_position_drive_caps(
        &self,
        selected_index: usize,
        diagnostic_name: &str,
        expected_torque_permille: u16,
        expected_kp_kd_torque_permille: u16,
    ) -> Result<()> {
        self.ensure_all_axes_disabled_for_commissioning(selected_index)?;
        anyhow::ensure!(
            !self.transport_failed.load(Ordering::Acquire),
            "CAN transport failed before {diagnostic_name} drive-cap readback"
        );
        let joint = &self.profile.joints[selected_index];
        let bus = self.manager.bus();
        let timeout = Some(self.manager.options().sdo_timeout);
        let actual_torque_permille =
            hex_motor::canopen::sdo::upload_u16(bus.as_ref(), joint.node_id, 0x6072, 0, timeout)
                .await
                .with_context(|| format!("read {diagnostic_name} torque cap 0x6072:00"))?;
        let actual_kp_kd_torque_permille =
            hex_motor::canopen::sdo::upload_u16(bus.as_ref(), joint.node_id, 0x2004, 0x0E, timeout)
                .await
                .with_context(|| format!("read {diagnostic_name} Kp/Kd torque cap 0x2004:0E"))?;
        self.ensure_all_axes_disabled_for_commissioning(selected_index)?;
        anyhow::ensure!(
            !self.transport_failed.load(Ordering::Acquire),
            "CAN transport failed during {diagnostic_name} drive-cap readback"
        );
        anyhow::ensure!(
            actual_torque_permille == expected_torque_permille
                && actual_kp_kd_torque_permille == expected_kp_kd_torque_permille,
            "{diagnostic_name} drive-cap readback mismatch: expected 0x6072:00/0x2004:0E={expected_torque_permille}/{expected_kp_kd_torque_permille}, got {actual_torque_permille}/{actual_kp_kd_torque_permille}"
        );
        tracing::info!(
            joint = %joint.name,
            node_id = joint.node_id,
            torque_permille = actual_torque_permille,
            kp_kd_torque_permille = actual_kp_kd_torque_permille,
            "fixed diagnostic drive caps were configured and read back while disabled; conservative OD values persist until the next configure"
        );
        Ok(())
    }

    async fn enable_and_confirm_commissioning_axis(&self, selected_index: usize) -> Result<()> {
        let joint = &self.profile.joints[selected_index];
        // Configuration is required to be disabled-only, but verify that fact
        // from fresh drive feedback again immediately before the sole
        // set_mode(MIT) call. This also proves that none of the other five
        // nodes became Operation Enabled during configuration.
        self.ensure_all_axes_disabled_for_commissioning(selected_index)?;
        anyhow::ensure!(
            !self.transport_failed.load(Ordering::Acquire),
            "shared compressed-MIT command transport failed before enabling {}",
            joint.name
        );
        self.manager
            .set_mode(joint.node_id, MotorMode::Mit)
            .await
            .with_context(|| {
                format!("set selected commissioning axis {} to MIT mode", joint.name)
            })?;
        let command_completed_at = Instant::now();
        self.confirm_compressed_mit_enabled(joint.node_id, command_completed_at)
            .await
            .with_context(|| {
                format!(
                    "confirm selected commissioning axis {} by TPDO2 and 0x6061",
                    joint.name
                )
            })?;
        self.expect_mit_operation_on_axis(selected_index);
        self.ensure_expected_mit_axes_remain_operation_enabled()?;
        Ok(())
    }

    fn ensure_all_axes_disabled_for_commissioning(&self, selected_index: usize) -> Result<()> {
        let now = Instant::now();
        for (index, joint) in self.profile.joints.iter().enumerate() {
            let status = self.manager.status(joint.node_id);
            anyhow::ensure!(
                status
                    .connection
                    .required_tpdos_fresh(now, self.profile.feedback_timeout()),
                "{} feedback is not fresh while verifying the commissioning disabled state",
                joint.name
            );
            anyhow::ensure!(
                status
                    .measurements
                    .status_word
                    .is_some_and(cia402::codec::status_word_is_confirmed_non_torque),
                "{}{} is not in a confirmed non-torque state before single-axis commissioning",
                joint.name,
                if index == selected_index {
                    " (selected axis)"
                } else {
                    " (unselected axis)"
                }
            );
            anyhow::ensure!(
                !matches!(status.logic, Some(Logic::Error { .. })),
                "{} is faulted before single-axis commissioning",
                joint.name
            );
        }
        Ok(())
    }
}

#[async_trait]
trait DiagnosticShutdownOperations: Sync {
    fn restore_baseline(&self, selected_index: usize) -> Result<()>;
    async fn disable_axis_confirmed(&self, index: usize) -> Result<()>;
    fn stop_command_sender(&self);
    async fn disarm_all_heartbeat_consumers(&self) -> Result<()>;
}

#[async_trait]
impl DiagnosticShutdownOperations for RealBackend {
    fn restore_baseline(&self, selected_index: usize) -> Result<()> {
        self.restore_registered_diagnostic_baseline(selected_index)
    }

    async fn disable_axis_confirmed(&self, index: usize) -> Result<()> {
        self.disable_axis_confirmed_with_retry(index, 3).await
    }

    fn stop_command_sender(&self) {
        self.shared_commands.stop_and_clear();
    }

    async fn disarm_all_heartbeat_consumers(&self) -> Result<()> {
        self.disarm_heartbeat_consumers_with_retry(3).await
    }
}

async fn run_selected_first_diagnostic_shutdown(
    operations: &impl DiagnosticShutdownOperations,
    selected_index: usize,
) -> Result<()> {
    anyhow::ensure!(
        selected_index < DOF,
        "diagnostic shutdown joint index {selected_index} is outside 0..{DOF}"
    );

    // Baseline restore is best-effort and deliberately precedes every SDO.
    // Even a failed/missing restore must not postpone the selected-axis
    // Shutdown request. A successful update gets two 1 kHz sender periods,
    // not the normal diagnostic dwell/readback delay.
    let baseline_error = operations.restore_baseline(selected_index).err();
    if baseline_error.is_none() {
        tokio::time::sleep(DIAGNOSTIC_BASELINE_REPUBLISH_SETTLE).await;
    }

    let mut disable_errors = Vec::new();
    if let Err(error) = operations.disable_axis_confirmed(selected_index).await {
        disable_errors.push(format!("selected joint index {selected_index}: {error:#}"));
    }
    for index in 0..DOF {
        if index == selected_index {
            continue;
        }
        if let Err(error) = operations.disable_axis_confirmed(index).await {
            disable_errors.push(format!("joint index {index}: {error:#}"));
        }
    }

    if !disable_errors.is_empty() {
        let baseline = baseline_error
            .map(|error| format!("; baseline restore also failed: {error:#}"))
            .unwrap_or_default();
        anyhow::bail!(
            "selected-first diagnostic disable was not confirmed for every axis: {}{}; retaining the shared sender and every 0x1016 consumer",
            disable_errors.join(", "),
            baseline
        );
    }

    // Every drive has supplied its own post-command strict non-torque TPDO2.
    // It is now safe to stop the RPDO sender and only then begin clearing
    // heartbeat consumers.
    operations.stop_command_sender();
    let disarm = operations.disarm_all_heartbeat_consumers().await;
    match (baseline_error, disarm) {
        (None, Ok(())) => Ok(()),
        (Some(baseline_error), Ok(())) => Err(baseline_error)
            .context("all axes were disabled/disarmed but diagnostic baseline restore failed"),
        (None, Err(disarm_error)) => Err(disarm_error),
        (Some(baseline_error), Err(disarm_error)) => Err(anyhow::anyhow!(
            "diagnostic baseline restore failed: {baseline_error:#}; all axes were disabled but heartbeat disarm also failed: {disarm_error:#}"
        )),
    }
}

/// Require the exact 0x1018 identity associated with a configured fixed tool
/// payload.  This is intentionally a pure gate over discovery snapshots: the
/// auxiliary node is never initialized, disabled, or otherwise controlled.
fn validate_required_tip_payload_identity(
    profile: &HardwareProfile,
    motors: &[MotorIdentitySnapshot],
) -> Result<()> {
    let Some(payload) = &profile.tip_payload else {
        return Ok(());
    };
    let actual = motors
        .iter()
        .find(|motor| motor.node_id == payload.auxiliary_node_id)
        .with_context(|| {
            format!(
                "tip payload auxiliary node {} was not discovered",
                payload.auxiliary_node_id
            )
        })?;
    anyhow::ensure!(
        actual.vendor_id == payload.identity.vendor_id
            && actual.product_code == payload.identity.product_code
            && actual.revision == payload.identity.revision
            && actual.serial_number == payload.identity.serial_number,
        "tip payload auxiliary node {} 0x1018 identity mismatch: expected \
         vendor=0x{:08X} product=0x{:08X} revision={} serial=0x{:08X}, got \
         vendor=0x{:08X} product=0x{:08X} revision={} serial=0x{:08X}",
        payload.auxiliary_node_id,
        payload.identity.vendor_id,
        payload.identity.product_code,
        payload.identity.revision,
        payload.identity.serial_number,
        actual.vendor_id,
        actual.product_code,
        actual.revision,
        actual.serial_number
    );
    Ok(())
}

fn compressed_target(target: MotorTarget) -> CompressedMitTarget {
    CompressedMitTarget {
        position: target.position_rev,
        velocity: target.velocity_rev_s,
        torque: target.torque_nm,
        kp: target.kp_nm_rev,
        kd: target.kd_nm_s_rev,
    }
}

#[async_trait]
trait CommissioningAxisOperations: Sync {
    async fn configure_selected(&self, selected_index: usize, target: MotorTarget) -> Result<()>;
    async fn enable_selected(&self, selected_index: usize) -> Result<()>;
}

#[async_trait]
impl CommissioningAxisOperations for RealBackend {
    async fn configure_selected(&self, selected_index: usize, target: MotorTarget) -> Result<()> {
        self.configure_commissioning_axis(selected_index, target)
            .await
    }

    async fn enable_selected(&self, selected_index: usize) -> Result<()> {
        self.enable_and_confirm_commissioning_axis(selected_index)
            .await
    }
}

async fn activate_only_selected_axis(
    operations: &impl CommissioningAxisOperations,
    selected_index: usize,
    target: MotorTarget,
) -> Result<()> {
    operations
        .configure_selected(selected_index, target)
        .await?;
    operations.enable_selected(selected_index).await
}

#[async_trait]
trait Joint1FirstPositionActivationOperations: Sync {
    async fn configure_joint1(
        &self,
        target: MotorTarget,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()>;
    async fn confirm_joint1_caps(
        &self,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()>;
    async fn enable_joint1(&self) -> Result<()>;
}

#[async_trait]
impl Joint1FirstPositionActivationOperations for RealBackend {
    async fn configure_joint1(
        &self,
        target: MotorTarget,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()> {
        self.configure_commissioning_axis_with_caps(
            0,
            target,
            torque_permille,
            kp_kd_torque_permille,
        )
        .await
    }

    async fn confirm_joint1_caps(
        &self,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()> {
        self.confirm_fixed_position_drive_caps(
            0,
            "joint_1 fixed diagnostic",
            torque_permille,
            kp_kd_torque_permille,
        )
        .await
    }

    async fn enable_joint1(&self) -> Result<()> {
        self.enable_and_confirm_commissioning_axis(0).await
    }
}

#[async_trait]
trait Joint5FirstPositionActivationOperations: Sync {
    async fn configure_joint5(
        &self,
        target: MotorTarget,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()>;
    async fn confirm_joint5_caps(
        &self,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()>;
    async fn enable_joint5(&self) -> Result<()>;
}

#[async_trait]
trait Joint4FirstPositionActivationOperations: Sync {
    async fn configure_joint4(
        &self,
        target: MotorTarget,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()>;
    async fn confirm_joint4_caps(
        &self,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()>;
    async fn enable_joint4(&self) -> Result<()>;
}

#[async_trait]
impl Joint4FirstPositionActivationOperations for RealBackend {
    async fn configure_joint4(
        &self,
        target: MotorTarget,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()> {
        self.configure_commissioning_axis_with_caps(
            3,
            target,
            torque_permille,
            kp_kd_torque_permille,
        )
        .await
    }

    async fn confirm_joint4_caps(
        &self,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()> {
        self.confirm_fixed_position_drive_caps(
            3,
            "joint_4 fixed diagnostic",
            torque_permille,
            kp_kd_torque_permille,
        )
        .await
    }

    async fn enable_joint4(&self) -> Result<()> {
        self.enable_and_confirm_commissioning_axis(3).await
    }
}

#[async_trait]
impl Joint5FirstPositionActivationOperations for RealBackend {
    async fn configure_joint5(
        &self,
        target: MotorTarget,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()> {
        self.configure_commissioning_axis_with_caps(
            4,
            target,
            torque_permille,
            kp_kd_torque_permille,
        )
        .await
    }

    async fn confirm_joint5_caps(
        &self,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()> {
        self.confirm_fixed_position_drive_caps(
            4,
            "joint_5 fixed diagnostic",
            torque_permille,
            kp_kd_torque_permille,
        )
        .await
    }

    async fn enable_joint5(&self) -> Result<()> {
        self.enable_and_confirm_commissioning_axis(4).await
    }
}

#[async_trait]
trait Joint6FirstPositionActivationOperations: Sync {
    async fn configure_joint6(
        &self,
        target: MotorTarget,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()>;
    async fn confirm_joint6_caps(
        &self,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()>;
    async fn enable_joint6(&self) -> Result<()>;
}

#[async_trait]
impl Joint6FirstPositionActivationOperations for RealBackend {
    async fn configure_joint6(
        &self,
        target: MotorTarget,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()> {
        self.configure_commissioning_axis_with_caps(
            5,
            target,
            torque_permille,
            kp_kd_torque_permille,
        )
        .await
    }

    async fn confirm_joint6_caps(
        &self,
        torque_permille: u16,
        kp_kd_torque_permille: u16,
    ) -> Result<()> {
        self.confirm_fixed_position_drive_caps(
            5,
            "joint_6 fixed diagnostic",
            torque_permille,
            kp_kd_torque_permille,
        )
        .await
    }

    async fn enable_joint6(&self) -> Result<()> {
        self.enable_and_confirm_commissioning_axis(5).await
    }
}

async fn activate_joint5_first_position_axis(
    operations: &impl Joint5FirstPositionActivationOperations,
    target: MotorTarget,
    torque_permille: u16,
    kp_kd_torque_permille: u16,
) -> Result<()> {
    anyhow::ensure!(
        torque_permille == J5_FIRST_POSITION_TORQUE_PERMILLE
            && kp_kd_torque_permille == J5_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        "joint_5 first-position activation accepts only the fixed 50/30 drive caps"
    );
    operations
        .configure_joint5(target, torque_permille, kp_kd_torque_permille)
        .await?;
    operations
        .confirm_joint5_caps(torque_permille, kp_kd_torque_permille)
        .await?;
    operations.enable_joint5().await
}

async fn activate_joint4_first_position_axis(
    operations: &impl Joint4FirstPositionActivationOperations,
    target: MotorTarget,
    torque_permille: u16,
    kp_kd_torque_permille: u16,
) -> Result<()> {
    anyhow::ensure!(
        torque_permille == J4_FIRST_POSITION_TORQUE_PERMILLE
            && kp_kd_torque_permille == J4_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        "joint_4 first-position activation accepts only the fixed 60/50 drive caps"
    );
    operations
        .configure_joint4(target, torque_permille, kp_kd_torque_permille)
        .await?;
    operations
        .confirm_joint4_caps(torque_permille, kp_kd_torque_permille)
        .await?;
    operations.enable_joint4().await
}

async fn activate_joint4_assisted_position_axis(
    operations: &impl Joint4FirstPositionActivationOperations,
    target: MotorTarget,
    torque_permille: u16,
    kp_kd_torque_permille: u16,
) -> Result<()> {
    anyhow::ensure!(
        torque_permille == J4_ASSISTED_POSITION_TORQUE_PERMILLE
            && kp_kd_torque_permille == J4_ASSISTED_POSITION_KP_KD_TORQUE_PERMILLE,
        "joint_4 assisted-position activation accepts only the fixed 90/50 drive caps"
    );
    operations
        .configure_joint4(target, torque_permille, kp_kd_torque_permille)
        .await?;
    operations
        .confirm_joint4_caps(torque_permille, kp_kd_torque_permille)
        .await?;
    operations.enable_joint4().await
}

async fn activate_joint6_first_position_axis(
    operations: &impl Joint6FirstPositionActivationOperations,
    target: MotorTarget,
    torque_permille: u16,
    kp_kd_torque_permille: u16,
) -> Result<()> {
    anyhow::ensure!(
        torque_permille == J6_FIRST_POSITION_TORQUE_PERMILLE
            && kp_kd_torque_permille == J6_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        "joint_6 first-position activation accepts only the fixed 50/30 drive caps"
    );
    operations
        .configure_joint6(target, torque_permille, kp_kd_torque_permille)
        .await?;
    operations
        .confirm_joint6_caps(torque_permille, kp_kd_torque_permille)
        .await?;
    operations.enable_joint6().await
}

async fn activate_joint1_first_position_axis(
    operations: &impl Joint1FirstPositionActivationOperations,
    target: MotorTarget,
    torque_permille: u16,
    kp_kd_torque_permille: u16,
) -> Result<()> {
    anyhow::ensure!(
        torque_permille == J1_FIRST_POSITION_TORQUE_PERMILLE
            && kp_kd_torque_permille == J1_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        "joint_1 first-position activation accepts only the fixed 30/20 drive caps"
    );
    operations
        .configure_joint1(target, torque_permille, kp_kd_torque_permille)
        .await?;
    operations
        .confirm_joint1_caps(torque_permille, kp_kd_torque_permille)
        .await?;
    operations.enable_joint1().await
}

fn validate_joint1_first_position_strict_zero_open(
    used_historical_can_xstats_acknowledgement: bool,
) -> Result<()> {
    anyhow::ensure!(
        !used_historical_can_xstats_acknowledgement,
        "joint_1 first-position diagnostic requires a strict-zero backend open and rejects every historical CAN xstats acknowledgement"
    );
    Ok(())
}

fn validate_joint1_first_position_strict_session(
    used_historical_can_xstats_acknowledgement: bool,
    backend_profile: &HardwareProfile,
    supplied_profile: &HardwareProfile,
) -> Result<()> {
    validate_joint1_first_position_strict_zero_open(used_historical_can_xstats_acknowledgement)?;
    anyhow::ensure!(
        std::ptr::eq(backend_profile, supplied_profile),
        "joint_1 first-position diagnostic requires the exact HardwareProfile allocation used to open the backend"
    );
    Ok(())
}

fn validate_joint5_first_position_strict_zero_open(
    used_historical_can_xstats_acknowledgement: bool,
) -> Result<()> {
    anyhow::ensure!(
        !used_historical_can_xstats_acknowledgement,
        "joint_5 first-position diagnostic requires a strict-zero backend open and rejects every historical CAN xstats acknowledgement"
    );
    Ok(())
}

fn validate_joint4_first_position_strict_zero_open(
    used_historical_can_xstats_acknowledgement: bool,
) -> Result<()> {
    anyhow::ensure!(
        !used_historical_can_xstats_acknowledgement,
        "joint_4 first-position diagnostic requires a strict-zero backend open and rejects every historical CAN xstats acknowledgement"
    );
    Ok(())
}

fn validate_joint3_gravity_unload_strict_zero_open(
    used_historical_can_xstats_acknowledgement: bool,
) -> Result<()> {
    anyhow::ensure!(
        !used_historical_can_xstats_acknowledgement,
        "joint_3 gravity-unload diagnostic requires a strict-zero backend open and rejects every historical CAN xstats acknowledgement"
    );
    Ok(())
}

fn validate_joint3_gravity_unload_strict_session(
    used_historical_can_xstats_acknowledgement: bool,
    backend_profile: &HardwareProfile,
    supplied_profile: &HardwareProfile,
) -> Result<()> {
    validate_joint3_gravity_unload_strict_zero_open(used_historical_can_xstats_acknowledgement)?;
    anyhow::ensure!(
        std::ptr::eq(backend_profile, supplied_profile),
        "joint_3 gravity-unload diagnostic requires the exact HardwareProfile allocation used to open the backend"
    );
    Ok(())
}

fn validate_joint4_first_position_strict_session(
    used_historical_can_xstats_acknowledgement: bool,
    backend_profile: &HardwareProfile,
    supplied_profile: &HardwareProfile,
) -> Result<()> {
    validate_joint4_first_position_strict_zero_open(used_historical_can_xstats_acknowledgement)?;
    anyhow::ensure!(
        std::ptr::eq(backend_profile, supplied_profile),
        "joint_4 first-position diagnostic requires the exact HardwareProfile allocation used to open the backend"
    );
    Ok(())
}

fn validate_joint5_first_position_strict_session(
    used_historical_can_xstats_acknowledgement: bool,
    backend_profile: &HardwareProfile,
    supplied_profile: &HardwareProfile,
) -> Result<()> {
    validate_joint5_first_position_strict_zero_open(used_historical_can_xstats_acknowledgement)?;
    anyhow::ensure!(
        std::ptr::eq(backend_profile, supplied_profile),
        "joint_5 first-position diagnostic requires the exact HardwareProfile allocation used to open the backend"
    );
    Ok(())
}

fn validate_joint6_first_position_strict_zero_open(
    used_historical_can_xstats_acknowledgement: bool,
) -> Result<()> {
    anyhow::ensure!(
        !used_historical_can_xstats_acknowledgement,
        "joint_6 first-position diagnostic requires a strict-zero backend open and rejects every historical CAN xstats acknowledgement"
    );
    Ok(())
}

fn validate_joint6_first_position_strict_session(
    used_historical_can_xstats_acknowledgement: bool,
    backend_profile: &HardwareProfile,
    supplied_profile: &HardwareProfile,
) -> Result<()> {
    validate_joint6_first_position_strict_zero_open(used_historical_can_xstats_acknowledgement)?;
    anyhow::ensure!(
        std::ptr::eq(backend_profile, supplied_profile),
        "joint_6 first-position diagnostic requires the exact HardwareProfile allocation used to open the backend"
    );
    Ok(())
}

fn first_expected_mit_axis_not_operation_enabled(
    expected: &[bool; DOF],
    status_words: &[Option<u16>; DOF],
) -> Option<(usize, Option<u16>)> {
    expected
        .iter()
        .zip(status_words)
        .enumerate()
        .find_map(|(index, (expected, status_word))| {
            (*expected && !status_word.is_some_and(cia402::codec::status_word_is_operation_enabled))
                .then_some((index, *status_word))
        })
}

fn supported_axis_mask(order: &[usize]) -> Result<[bool; DOF]> {
    anyhow::ensure!(
        (2..=DOF).contains(&order.len()),
        "supported operation requires 2..6 distinct axes"
    );
    let mut mask = [false; DOF];
    for &index in order {
        anyhow::ensure!(
            index < DOF && !mask[index],
            "invalid/duplicate supported axis {index}"
        );
        mask[index] = true;
    }
    Ok(mask)
}

fn validate_supported_status_words(active: [bool; DOF], words: [Option<u16>; DOF]) -> Result<()> {
    for i in 0..DOF {
        let valid = if active[i] {
            words[i].is_some_and(cia402::codec::status_word_is_operation_enabled)
        } else {
            words[i].is_some_and(cia402::codec::status_word_is_confirmed_non_torque)
        };
        anyhow::ensure!(
            valid && words[i].is_some_and(|word| !cia402::codec::status_word_has_fault(word)),
            "joint {} group state mismatch: active={} status={:?}",
            i + 1,
            active[i],
            words[i]
        );
    }
    Ok(())
}

#[cfg(test)]
mod supported_activation_tests {
    use super::*;
    #[test]
    fn mask_and_status_contract_rejects_missing_unexpected_and_quickstop_axes() {
        let mask = supported_axis_mask(&[3, 2]).unwrap();
        for order in [vec![], vec![2], vec![2, 2], vec![2, 6]] {
            assert!(supported_axis_mask(&order).is_err());
        }
        let mut words = [Some(0x0231); DOF];
        words[2] = Some(0x0027);
        words[3] = Some(0x0027);
        validate_supported_status_words(mask, words).unwrap();
        for (i, word) in [
            (0, Some(0x0027)),
            (2, Some(0x0231)),
            (3, None),
            (3, Some(0x0007)),
            (5, Some(0x0008)),
        ] {
            let mut bad = words;
            bad[i] = word;
            assert!(validate_supported_status_words(mask, bad).is_err());
        }
    }
}

fn validate_single_axis_commissioning_status_words(
    profile: &HardwareProfile,
    selected_index: usize,
    status_words: &[Option<u16>; DOF],
) -> Result<()> {
    anyhow::ensure!(
        selected_index < DOF,
        "single-axis status index {selected_index} is outside 0..{DOF}"
    );
    for (index, (joint, status_word)) in profile.joints.iter().zip(status_words).enumerate() {
        let rendered = status_word
            .map(|word| format!("0x{word:04X}"))
            .unwrap_or_else(|| "missing".into());
        if index == selected_index {
            anyhow::ensure!(
                status_word.is_some_and(cia402::codec::status_word_is_operation_enabled),
                "{} (node {}) is not strict Operation Enabled during isolated diagnostics (status {rendered})",
                joint.name,
                joint.node_id
            );
        } else {
            anyhow::ensure!(
                status_word.is_some_and(cia402::codec::status_word_is_confirmed_non_torque),
                "unselected {} (node {}) is not in a confirmed non-torque state during isolated diagnostics (status {rendered})",
                joint.name,
                joint.node_id
            );
        }
    }
    Ok(())
}

fn disable_feedback_confirmed(status: &LiveState, command_completed_at: Instant) -> bool {
    status
        .connection
        .last_tpdo2
        .is_some_and(|stamp| stamp > command_completed_at)
        && status
            .measurements
            .status_word
            .is_some_and(cia402::codec::status_word_is_confirmed_non_torque)
}

fn operation_enabled_after(status: &LiveState, command_completed_at: Instant) -> bool {
    status
        .connection
        .last_tpdo2
        .is_some_and(|stamp| stamp > command_completed_at)
        && status
            .measurements
            .status_word
            .is_some_and(cia402::codec::status_word_is_operation_enabled)
}

fn compressed_mit_enable_confirmed(
    status: &LiveState,
    command_completed_at: Instant,
    mode_display: Option<u8>,
) -> bool {
    operation_enabled_after(status, command_completed_at) && mode_display == Some(MIT_MODE_DISPLAY)
}

/// Validate the first complete feedback snapshot without performing any I/O.
///
/// The returned array is intentionally all-or-nothing: callers install it
/// only after every joint has passed the same online/fresh/finite/branch gate.
fn build_initial_feedback_unwrappers(
    joints: &[JointProfile],
    statuses: &[LiveState; DOF],
    now: Instant,
    maximum_age: Duration,
) -> Result<[SingleTurnUnwrapper; DOF]> {
    anyhow::ensure!(
        joints.len() == DOF,
        "initial feedback gate requires exactly {DOF} joints, got {}",
        joints.len()
    );

    let mut candidates = Vec::with_capacity(DOF);
    for (joint, status) in joints.iter().zip(statuses) {
        candidates.push(validate_initial_joint_feedback(
            joint,
            status,
            now,
            maximum_age,
        )?);
    }
    candidates.try_into().map_err(|candidates: Vec<_>| {
        anyhow::anyhow!(
            "initial feedback gate constructed {} single-turn states instead of {DOF}",
            candidates.len()
        )
    })
}

fn validate_initial_joint_feedback(
    joint: &JointProfile,
    status: &LiveState,
    now: Instant,
    maximum_age: Duration,
) -> Result<SingleTurnUnwrapper> {
    let label = format!("{} (CANopen node {})", joint.name, joint.node_id);
    anyhow::ensure!(
        status.connection.online,
        "{label} is offline at the initial feedback gate"
    );
    ensure_initial_tpdo_fresh(
        &label,
        "TPDO1",
        status.connection.last_tpdo1,
        now,
        maximum_age,
    )?;
    ensure_initial_tpdo_fresh(
        &label,
        "TPDO2",
        status.connection.last_tpdo2,
        now,
        maximum_age,
    )?;

    let measurements = &status.measurements;
    let tpdo1_last_error = measurements.tpdo1_error_code.ok_or_else(|| {
        anyhow::anyhow!(
            "{label} TPDO1 has no decoded error-code field at the initial feedback gate"
        )
    })?;
    let tpdo2_last_error = measurements.tpdo2_error_code.ok_or_else(|| {
        anyhow::anyhow!(
            "{label} TPDO2 has no decoded error-code field at the initial feedback gate"
        )
    })?;

    let status_word = measurements.status_word.ok_or_else(|| {
        anyhow::anyhow!("{label} TPDO2 has no decoded status word at the initial feedback gate")
    })?;
    if cia402::codec::status_word_has_fault(status_word) {
        let last_error = [tpdo2_last_error, tpdo1_last_error]
            .into_iter()
            .find(|code| *code != 0)
            .unwrap_or(0);
        anyhow::bail!(
            "{label} has a current CiA402 Fault at the initial feedback gate \
             (status 0x{status_word:04X}, last error 0x{last_error:04X})"
        );
    }
    anyhow::ensure!(
        cia402::codec::status_word_is_confirmed_non_torque(status_word),
        "{label} is not in a confirmed non-torque state at the initial feedback gate (status 0x{status_word:04X})"
    );

    let raw_position = require_finite_initial_measurement(
        &label,
        "TPDO1 position_rev",
        measurements.position_rev,
    )?;
    require_finite_initial_measurement(
        &label,
        "TPDO1 velocity_rev_per_s",
        measurements.velocity_rev_per_s,
    )?;
    require_finite_initial_measurement(&label, "TPDO1 torque_nm", measurements.torque_nm)?;
    require_finite_initial_measurement(&label, "TPDO2 driver_temp_c", measurements.driver_temp_c)?;
    require_finite_initial_measurement(&label, "TPDO2 motor_temp_c", measurements.motor_temp_c)?;
    anyhow::ensure!(
        measurements.timestamp_us.is_some(),
        "{label} TPDO1 has no decoded timestamp at the initial feedback gate"
    );

    let mapping = joint.compressed_mapping();
    let window = BranchWindow {
        lower_rev: mapping.position_min,
        upper_rev: mapping.position_max,
        tolerance_rev: POSITION_BRANCH_TOLERANCE_REV
            + joint.limits.measured_position_margin_rad / std::f32::consts::TAU,
    };
    SingleTurnUnwrapper::initialize(raw_position, window, None).map_err(|error| {
        anyhow::anyhow!(
            "{label} cannot initialize its single-turn feedback window from raw position {raw_position} rev: {error}"
        )
    })
}

fn ensure_initial_tpdo_fresh(
    label: &str,
    stream: &str,
    stamp: Option<Instant>,
    now: Instant,
    maximum_age: Duration,
) -> Result<()> {
    let stamp = stamp.ok_or_else(|| {
        anyhow::anyhow!("{label} has not received {stream} at the initial feedback gate")
    })?;
    let age = now.saturating_duration_since(stamp);
    anyhow::ensure!(
        age <= maximum_age,
        "{label} {stream} is stale at the initial feedback gate (age {:.3} ms, limit {:.3} ms)",
        age.as_secs_f64() * 1_000.0,
        maximum_age.as_secs_f64() * 1_000.0
    );
    Ok(())
}

fn require_finite_initial_measurement(label: &str, field: &str, value: Option<f32>) -> Result<f32> {
    let value = value.ok_or_else(|| {
        anyhow::anyhow!("{label} has no decoded {field} at the initial feedback gate")
    })?;
    anyhow::ensure!(
        value.is_finite(),
        "{label} {field} is non-finite ({value}) at the initial feedback gate"
    );
    Ok(value)
}

fn validate_compressed_target(
    target: &MotorTarget,
    mapping: &CompressedMitMapping,
    joint_name: &str,
) -> Result<()> {
    let values = [
        (
            "position",
            target.position_rev,
            mapping.position_min,
            mapping.position_max,
        ),
        (
            "velocity",
            target.velocity_rev_s,
            mapping.velocity_min,
            mapping.velocity_max,
        ),
        (
            "torque",
            target.torque_nm,
            mapping.torque_min,
            mapping.torque_max,
        ),
        ("kp", target.kp_nm_rev, mapping.kp_min, mapping.kp_max),
        ("kd", target.kd_nm_s_rev, mapping.kd_min, mapping.kd_max),
    ];
    for (name, value, lower, upper) in values {
        anyhow::ensure!(
            value.is_finite() && (lower..=upper).contains(&value),
            "{joint_name} compressed-MIT {name} target {value} is outside [{lower}, {upper}]"
        );
    }
    Ok(())
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
        self.initialization_complete.store(false, Ordering::Release);
        *self.diagnostic_baseline.lock() = None;
        self.clear_mit_operation_expectations();
        let initialize_result: Result<()> = async {
            self.wait_for_verified_joint_identities().await?;
            for node_id in &self.profile.bus.auxiliary_node_ids {
                if self
                    .manager
                    .list()
                    .iter()
                    .any(|motor| motor.node_id == *node_id)
                {
                    tracing::info!(
                        node_id,
                        "allowed auxiliary CANopen node discovered and excluded from arm control"
                    );
                }
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
            self.disable_all_with_retry(3)
                .await
                .context("confirm that all six CANopen drives are disabled after initialization")?;

            // Do not advertise a ready API from identity/lifecycle state alone.
            // A complete, current pair of TPDO streams and an unambiguous initial
            // single-turn branch are required for every joint while all drives are
            // still confirmed disabled.  Build the six unwrappers off to the side
            // and commit them atomically so a failure on a later joint cannot
            // leave a partially initialized feedback state behind.
            let now = Instant::now();
            let statuses =
                array::from_fn(|index| self.manager.status(self.profile.joints[index].node_id));
            let unwrappers = build_initial_feedback_unwrappers(
                &self.profile.joints,
                &statuses,
                now,
                self.profile.feedback_timeout(),
            )
            .context("initial six-axis feedback gate rejected controller startup")?;
            *self.position_unwrappers.lock() = unwrappers.map(Some);
            Ok(())
        }
        .await;

        match initialize_result {
            Ok(()) => {
                self.initialization_complete.store(true, Ordering::Release);
                Ok(())
            }
            Err(initialize_error) => {
                self.transport_failed.store(true, Ordering::Release);
                match self
                    .cleanup_partial_initialization_heartbeat_consumers_with_retry(3)
                    .await
                {
                    Ok(()) => Err(initialize_error),
                    Err(cleanup_error) => Err(anyhow::anyhow!(
                        "{initialize_error:#}; partial-initialization heartbeat cleanup also failed: {cleanup_error:#}"
                    )),
                }
            }
        }
    }

    async fn enable_compressed_mit(&self, initial_targets: [MotorTarget; DOF]) -> Result<()> {
        self.clear_mit_operation_expectations();
        self.ensure_no_unexpected_nodes()?;
        anyhow::ensure!(
            !self.transport_failed.load(Ordering::Acquire),
            "CAN transport is already in a failed state"
        );
        self.profile.validate_single_turn_command_windows()?;
        for (index, joint) in self.profile.joints.iter().enumerate() {
            let mapping = joint.compressed_mapping();
            validate_compressed_target(&initial_targets[index], &mapping, &joint.name)?;
        }
        let initial_targets = initial_targets.map(|target| CompressedMitTarget {
            position: target.position_rev,
            velocity: target.velocity_rev_s,
            torque: target.torque_nm,
            kp: target.kp_nm_rev,
            kd: target.kd_nm_s_rev,
        });
        // Publish the feedback-derived hold target before the first RPDO is
        // enabled. The sender gate is released only after the target is live.
        self.shared_commands
            .install_hold_and_enable(initial_targets);

        let bus = self.manager.bus();
        // Configure every RPDO/range while all drives remain disabled. Only
        // after all six configurations succeed do we begin the confirmed
        // CiA402 set_mode enable ramp.
        for (slot, joint) in self.profile.joints.iter().enumerate() {
            let configure_result = cia402::compressed_mit::configure(
                bus.as_ref(),
                joint.node_id,
                slot as u8,
                DOF as u8,
                cia402::DEFAULT_SHARED_COB_ID,
                &joint.compressed_mapping(),
                &initial_targets[slot],
                joint.torque_permille,
                joint.kp_kd_torque_permille,
                Some(self.manager.options().sdo_timeout),
            )
            .await;
            if let Err(configure_error) = configure_result {
                let configure_error = anyhow::anyhow!(
                    "configure compressed MIT for {}: {}",
                    joint.name,
                    configure_error
                );
                return match self.disable_all().await {
                    Ok(()) => Err(configure_error),
                    Err(rollback_error) => Err(anyhow::anyhow!(
                        "{configure_error:#}; rollback disable also failed: {rollback_error:#}"
                    )),
                };
            }
        }

        for (index, joint) in self.profile.joints.iter().enumerate() {
            if let Err(error) = self.ensure_expected_mit_axes_remain_operation_enabled() {
                self.latch_expected_mit_drive_state_failure();
                return match self.disable_all().await {
                    Ok(()) => Err(error),
                    Err(rollback_error) => Err(anyhow::anyhow!(
                        "{error:#}; rollback disable also failed: {rollback_error:#}"
                    )),
                };
            }
            if self.transport_failed.load(Ordering::Acquire) {
                let configure_error = anyhow::anyhow!(
                    "shared compressed-MIT command transport failed before enabling {}",
                    joint.name
                );
                return match self.disable_all().await {
                    Ok(()) => Err(configure_error),
                    Err(rollback_error) => Err(anyhow::anyhow!(
                        "{configure_error:#}; rollback disable also failed: {rollback_error:#}"
                    )),
                };
            }

            let enable_result = async {
                self.manager
                    .set_mode(joint.node_id, MotorMode::Mit)
                    .await
                    .with_context(|| format!("set {} to confirmed MIT mode", joint.name))?;
                // Require a TPDO2 newer than the completed set_mode call; a
                // frame seen during the SDO ramp cannot satisfy confirmation.
                let command_completed_at = Instant::now();
                self.confirm_compressed_mit_enabled(joint.node_id, command_completed_at)
                    .await
                    .with_context(|| format!("confirm compressed MIT for {}", joint.name))
            }
            .await;
            if let Err(enable_error) = enable_result {
                return match self.disable_all().await {
                    Ok(()) => Err(enable_error),
                    Err(rollback_error) => Err(anyhow::anyhow!(
                        "{enable_error:#}; rollback disable also failed: {rollback_error:#}"
                    )),
                };
            }
            self.expect_mit_operation_on_axis(index);
        }
        if let Err(error) = self.ensure_expected_mit_axes_remain_operation_enabled() {
            self.latch_expected_mit_drive_state_failure();
            return match self.disable_all().await {
                Ok(()) => Err(error),
                Err(rollback_error) => Err(anyhow::anyhow!(
                    "{error:#}; rollback disable also failed: {rollback_error:#}"
                )),
            };
        }
        Ok(())
    }

    async fn set_targets(&self, targets: [MotorTarget; DOF]) -> Result<()> {
        self.latch_expected_mit_drive_state_failure();
        anyhow::ensure!(
            !self.transport_failed.load(Ordering::Acquire),
            "CAN transport is in a failed state"
        );
        for (index, joint) in self.profile.joints.iter().enumerate() {
            validate_compressed_target(&targets[index], &joint.compressed_mapping(), &joint.name)?;
        }
        let packed = targets.map(|target| CompressedMitTarget {
            position: target.position_rev,
            velocity: target.velocity_rev_s,
            torque: target.torque_nm,
            kp: target.kp_nm_rev,
            kd: target.kd_nm_s_rev,
        });
        self.shared_commands.update_targets(packed);
        Ok(())
    }

    async fn disable_all(&self) -> Result<()> {
        // A transition out of OE is intentional from this point onward. Clear
        // the runtime expectation before issuing CW=0x06, then require the
        // independent post-command non-torque confirmation below.
        self.clear_mit_operation_expectations();
        let mut errors = Vec::new();
        for joint in &self.profile.joints {
            if let Err(error) = self.manager.disable(joint.node_id).await {
                errors.push(format!("{}: {error}", joint.name));
            }
        }
        // Keep publishing the last hold target until every SDO is acknowledged
        // and a newer TPDO2 confirms every axis left Operation Enabled. If any
        // step fails, zeroing or stopping could drop stiffness on a live axis.
        anyhow::ensure!(errors.is_empty(), "disable failures: {}", errors.join(", "));
        self.wait_for_disabled_feedback(Instant::now()).await?;
        self.shared_commands.stop_and_clear();
        Ok(())
    }

    async fn shutdown(&self) -> Result<()> {
        // 0x1016 is itself a safety watchdog. Never remove it after an
        // unconfirmed disable: on that path the process exit must deliberately
        // let the drive's heartbeat-loss protection fire.
        match self.disable_all_with_retry(3).await {
            Ok(()) => self.disarm_heartbeat_consumers_with_retry(3).await,
            Err(disable_error) => {
                // A cancelled/failed initialize leaves some nodes Identified
                // rather than Initialized, so the normal all-axis path cannot
                // address them.  Fall back only to consumers recorded by this
                // session; each one gets its own Shutdown + authoritative
                // non-OE proof before 0x1016 is removed.
                match self
                    .cleanup_partial_initialization_heartbeat_consumers_with_retry(3)
                    .await
                {
                    Ok(()) => Err(disable_error).context(
                        "normal all-axis disable was not available; partial-init heartbeat consumers were cleaned, but the disable error is preserved",
                    ),
                    Err(cleanup_error) => Err(anyhow::anyhow!(
                        "normal confirmed disable failed: {disable_error:#}; refusing incomplete partial-init heartbeat cleanup: {cleanup_error:#}"
                    )),
                }
            }
        }
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
        self.latch_expected_mit_drive_state_failure();
        let now = Instant::now();
        let mut oldest_tpdo = Some(now);
        let mut oldest_tpdo1 = Some(now);
        let mut position_unwrappers = self.position_unwrappers.lock();
        let joints = array::from_fn(|index| {
            let joint = &self.profile.joints[index];
            let node = joint.node_id;
            let status = self.manager.status(node);
            let required_tpdo = status.connection.required_tpdo_stamp();
            oldest_tpdo = match (oldest_tpdo, required_tpdo) {
                (Some(oldest), Some(stamp)) => Some(oldest.min(stamp)),
                _ => None,
            };
            oldest_tpdo1 = match (oldest_tpdo1, status.connection.last_tpdo1) {
                (Some(oldest), Some(stamp)) => Some(oldest.min(stamp)),
                _ => None,
            };
            let mut fresh = status
                .connection
                .required_tpdos_fresh(now, self.profile.feedback_timeout());
            let measurement = status.measurements;
            let position_rev = measurement.position_rev.map_or(0.0, |raw_rev| {
                let mapping = joint.compressed_mapping();
                let window = BranchWindow {
                    lower_rev: mapping.position_min,
                    upper_rev: mapping.position_max,
                    tolerance_rev: POSITION_BRANCH_TOLERANCE_REV
                        + joint.limits.measured_position_margin_rad / std::f32::consts::TAU,
                };
                let max_step_rev = joint.limits.velocity_rad_s
                    * self.profile.feedback_timeout().as_secs_f32()
                    / std::f32::consts::TAU
                    + POSITION_UNWRAP_STEP_MARGIN_REV;
                let result = match &mut position_unwrappers[index] {
                    Some(unwrapper) => unwrapper.update(raw_rev, max_step_rev),
                    slot @ None => match SingleTurnUnwrapper::initialize(raw_rev, window, None) {
                        Ok(unwrapper) => {
                            let position_rev = unwrapper.position_rev();
                            *slot = Some(unwrapper);
                            Ok(position_rev)
                        }
                        Err(error) => Err(error),
                    },
                };
                match result {
                    Ok(position_rev) => position_rev,
                    Err(error) => {
                        fresh = false;
                        if !self.transport_failed.swap(true, Ordering::AcqRel) {
                            tracing::error!(
                                joint = %joint.name,
                                node_id = node,
                                raw_position_rev = raw_rev,
                                %error,
                                "single-turn feedback unwrapping failed closed"
                            );
                        }
                        f32::NAN
                    }
                }
            });
            JointFeedback {
                position_rev,
                velocity_rev_s: measurement.velocity_rev_per_s.unwrap_or_default(),
                torque_nm: measurement.torque_nm.unwrap_or_default(),
                temperature_c: measurement
                    .motor_temp_c
                    .or(measurement.driver_temp_c)
                    .unwrap_or_default(),
                driver_temperature_c: measurement.driver_temp_c.unwrap_or_default(),
                motor_temperature_c: measurement.motor_temp_c.unwrap_or_default(),
                online: status.connection.online,
                fresh,
                fault_code: match status.logic {
                    Some(Logic::Error { raw_code, .. }) => Some(raw_code),
                    _ => None,
                },
            }
        });
        FeedbackSnapshot {
            joints,
            oldest_tpdo1_at: oldest_tpdo1,
            captured_at: oldest_tpdo,
        }
    }

    fn transport_failed(&self) -> bool {
        self.latch_expected_mit_drive_state_failure();
        if let Err(error) = self.ensure_no_unexpected_nodes() {
            if !self.transport_failed.swap(true, Ordering::AcqRel) {
                tracing::error!(%error, "CANopen node allowlist violation");
            }
        }
        self.transport_failed.load(Ordering::Acquire)
    }
}

fn validate_heartbeat_recovery_selection(
    profile: &HardwareProfile,
    requested_nodes: &BTreeSet<u8>,
    diagnostics: &[DriveDiagnostic; DOF],
) -> Result<()> {
    anyhow::ensure!(
        profile.bus.direct_joint_mapping,
        "heartbeat-lost recovery requires direct joint_N -> node N mapping"
    );
    anyhow::ensure!(
        !requested_nodes.is_empty(),
        "heartbeat-lost recovery requires at least one explicit node"
    );
    let controlled_nodes: BTreeSet<_> = profile.joints.iter().map(|joint| joint.node_id).collect();
    anyhow::ensure!(
        controlled_nodes == (1..=DOF as u8).collect(),
        "heartbeat-lost recovery is restricted to arm nodes 1..=6"
    );
    anyhow::ensure!(
        requested_nodes.is_subset(&controlled_nodes),
        "heartbeat-lost recovery request contains a node outside arm nodes 1..=6"
    );

    let mut actual_heartbeat_faults = BTreeSet::new();
    for (joint, diagnostic) in profile.joints.iter().zip(diagnostics) {
        anyhow::ensure!(
            cia402::codec::status_word_is_confirmed_non_torque(diagnostic.status_word),
            "{} (node {}) is not in a confirmed non-torque state; refusing any heartbeat fault reset",
            joint.name,
            joint.node_id
        );
        if cia402::codec::status_word_has_fault(diagnostic.status_word) {
            anyhow::ensure!(
                diagnostic.error_code == 0x8130,
                "{} (node {}) has a current non-heartbeat fault (last error 0x{:04X}); refusing fault reset",
                joint.name,
                joint.node_id,
                diagnostic.error_code
            );
            actual_heartbeat_faults.insert(joint.node_id);
        }
    }
    anyhow::ensure!(
        actual_heartbeat_faults == *requested_nodes,
        "explicit recovery set {requested_nodes:?} does not exactly match live 0x8130 fault set {actual_heartbeat_faults:?}"
    );
    Ok(())
}

async fn wait_for_controlled_emcy(
    rx: &mut dyn CanRx,
    controlled_nodes: &BTreeSet<u8>,
) -> Result<()> {
    loop {
        let frame = rx
            .recv()
            .await
            .context("receive arm EMCY while heartbeat recovery is active")?;
        let CanId::Standard(cob_id) = frame.id() else {
            continue;
        };
        if !(0x081..=0x0FF).contains(&cob_id) {
            continue;
        }
        let node_id = (cob_id - 0x080) as u8;
        if !controlled_nodes.contains(&node_id) {
            continue;
        }
        let error_code = frame
            .data()
            .get(..2)
            .map(|bytes| u16::from_le_bytes([bytes[0], bytes[1]]));
        anyhow::bail!(
            "CANopen node {node_id} emitted a new EMCY during heartbeat recovery (error {:?}); refusing success",
            error_code.map(|code| format!("0x{code:04X}"))
        );
    }
}

fn validate_recovery_result_disabled_fault_free(
    profile: &HardwareProfile,
    diagnostics: &[DriveDiagnostic; DOF],
) -> Result<()> {
    for (joint, diagnostic) in profile.joints.iter().zip(diagnostics) {
        anyhow::ensure!(
            !cia402::codec::status_word_has_fault(diagnostic.status_word)
                && cia402::codec::status_word_is_confirmed_non_torque(diagnostic.status_word),
            "{} (node {}) was not confirmed fault-free and disabled after recovery \
             (0x603F=0x{:04X}, 0x6041=0x{:04X})",
            joint.name,
            joint.node_id,
            diagnostic.error_code,
            diagnostic.status_word
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn recovery_profile() -> HardwareProfile {
        HardwareProfile {
            schema_version: crate::profile::HARDWARE_PROFILE_SCHEMA_VERSION,
            joint_coordinate_version: crate::profile::JOINT_COORDINATE_VERSION,
            validated: true,
            calibrated: false,
            robot_prefix: "test/firefly_y6".into(),
            urdf_path: "unused-by-recovery-selection-tests".into(),
            gravity_vector_base_m_s2: [0.0, 0.0, -9.81],
            tip_payload: None,
            bus: crate::profile::BusProfile {
                protocol: crate::profile::MotorProtocol::Cia402,
                transport: BusTransport::GsUsb,
                interface: String::new(),
                channel: 0,
                adapter_vid: 1,
                adapter_pid: 1,
                heartbeat_node_id: 16,
                hardware_timestamp: false,
                direct_joint_mapping: true,
                auxiliary_node_ids: vec![15],
                expected_link: None,
            },
            controller: crate::profile::ControllerProfile {
                max_measured_temperature_c: None,
                gravity_startup_slew_rate_nm_s: None,
                hand_guiding_velocity_limits_rad_s: None,
                hand_guiding_position_margin_rad: None,
                shutdown_damping: None,
                loop_hz: 1000,
                state_publish_hz: 100,
                discovery_timeout_ms: 500,
                feedback_timeout_ms: 100,
                command_watchdog_ms: 100,
            },
            joints: initial_gate_joints(),
        }
    }

    fn heartbeat_lost_diagnostics() -> [DriveDiagnostic; DOF] {
        [DriveDiagnostic {
            error_code: 0x8130,
            status_word: 0x0008,
        }; DOF]
    }

    fn initial_gate_joints() -> Vec<JointProfile> {
        (0..DOF)
            .map(|index| JointProfile {
                name: format!("joint_{}", index + 1),
                node_id: (index + 1) as u8,
                identity: crate::profile::IdentityFingerprint::test_value(),
                direction: 1,
                zero_offset_rad: 0.0,
                torque_scale: 1.0,
                gravity_compensation_scale: 1.0,
                gravity_compensation_limit_nm: None,
                motion_feedforward: None,
                torque_permille: 200,
                kp_kd_torque_permille: 100,
                meow_torque_budget: Default::default(),
                limits: crate::profile::JointLimits {
                    position_lower_rad: -1.0,
                    position_upper_rad: 1.0,
                    measured_position_margin_rad: 0.0,
                    measured_velocity_margin_rad_s: 0.0,
                    velocity_rad_s: 0.1,
                    acceleration_rad_s2: 0.1,
                    torque_nm: 1.0,
                },
                default_kp: 1.0,
                default_kd: 0.1,
            })
            .collect()
    }

    fn profile_with_trial_payload() -> HardwareProfile {
        let mut profile = recovery_profile();
        profile.tip_payload = Some(crate::profile::TipPayloadProfile {
            mount_link: "link_6".into(),
            auxiliary_node_id: 15,
            identity: crate::profile::IdentityFingerprint {
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
        });
        profile
    }

    fn trial_payload_snapshot() -> MotorIdentitySnapshot {
        MotorIdentitySnapshot {
            node_id: 15,
            vendor_id: 0x4859_444c,
            product_code: 0xaaaa_0001,
            revision: 9,
            serial_number: 0x2573_510b,
            model: "CiA402 HEX-4310".into(),
            identity_verified: true,
        }
    }

    #[test]
    fn required_tip_payload_identity_rejects_missing_or_wrong_auxiliary_node() {
        let profile = profile_with_trial_payload();
        let missing = validate_required_tip_payload_identity(&profile, &[])
            .unwrap_err()
            .to_string();
        assert!(missing.contains("node 15 was not discovered"));

        let mut wrong = trial_payload_snapshot();
        wrong.serial_number ^= 1;
        let mismatch = validate_required_tip_payload_identity(&profile, &[wrong])
            .unwrap_err()
            .to_string();
        assert!(mismatch.contains("0x1018 identity mismatch"));

        validate_required_tip_payload_identity(&profile, &[trial_payload_snapshot()]).unwrap();
    }

    #[test]
    fn recovery_selection_accepts_only_exact_active_8130_fault_set() {
        let profile = recovery_profile();
        let requested = (1..=6).collect();
        validate_heartbeat_recovery_selection(&profile, &requested, &heartbeat_lost_diagnostics())
            .unwrap();
    }

    #[test]
    fn recovery_selection_rejects_partial_ack_other_fault_and_operation_enabled() {
        let profile = recovery_profile();
        let partial = [1, 2, 3].into_iter().collect();
        let error = validate_heartbeat_recovery_selection(
            &profile,
            &partial,
            &heartbeat_lost_diagnostics(),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("does not exactly match"));

        let all = (1..=6).collect();
        let mut other_fault = heartbeat_lost_diagnostics();
        other_fault[2].error_code = 0x2310;
        assert!(validate_heartbeat_recovery_selection(&profile, &all, &other_fault).is_err());

        let mut operation_enabled = heartbeat_lost_diagnostics();
        operation_enabled[5].status_word = 0x0027;
        assert!(validate_heartbeat_recovery_selection(&profile, &all, &operation_enabled).is_err());

        for unsafe_status in [0x0007, 0x000F] {
            let mut braking_state = heartbeat_lost_diagnostics();
            braking_state[5].status_word = unsafe_status;
            assert!(validate_heartbeat_recovery_selection(&profile, &all, &braking_state).is_err());
        }

        let retained_history = [DriveDiagnostic {
            error_code: 0x8130,
            status_word: 0x0231,
        }; DOF];
        assert!(validate_heartbeat_recovery_selection(&profile, &all, &retained_history).is_err());
    }

    #[test]
    fn recovery_result_requires_fault_free_confirmed_non_torque_state() {
        let profile = recovery_profile();
        let clean = [DriveDiagnostic {
            error_code: 0x8130,
            status_word: 0x0231,
        }; DOF];
        validate_recovery_result_disabled_fault_free(&profile, &clean).unwrap();

        for diagnostic in [
            DriveDiagnostic {
                error_code: 0,
                status_word: 0x0008,
            },
            DriveDiagnostic {
                error_code: 0,
                status_word: 0x0007,
            },
            DriveDiagnostic {
                error_code: 0,
                status_word: 0x000F,
            },
            DriveDiagnostic {
                error_code: 0,
                status_word: 0x0027,
            },
        ] {
            let mut invalid = clean;
            invalid[0] = diagnostic;
            assert!(validate_recovery_result_disabled_fault_free(&profile, &invalid).is_err());
        }
    }

    #[test]
    fn orderly_shutdown_cannot_disarm_watchdog_before_confirmed_disable() {
        let source = include_str!("legacy.rs");
        let body = source
            .split_once("impl MotorBackend for RealBackend")
            .unwrap()
            .1
            .split_once("#[cfg(test)]")
            .unwrap()
            .0;
        let confirmed_disable = body.find("disable_all_with_retry(3)").unwrap();
        let disarm = body
            .find("disarm_heartbeat_consumers_with_retry(3)")
            .unwrap();
        assert!(confirmed_disable < disarm);
        assert!(body[..disarm].contains("?;"));
    }

    #[test]
    fn partial_initialize_cleanup_is_targeted_and_preserves_disable_errors() {
        let source = include_str!("legacy.rs");
        let body = source
            .split_once("impl MotorBackend for RealBackend")
            .unwrap()
            .1
            .split_once("#[cfg(test)]")
            .unwrap()
            .0;
        assert!(body.contains("cleanup_partial_initialization_heartbeat_consumers_with_retry(3)"));
        assert!(body.contains("Ok(()) => Err(disable_error).context("));
        assert!(body.contains("partial-initialization heartbeat cleanup also failed"));
    }

    fn healthy_initial_status(now: Instant, raw_position: f32) -> LiveState {
        let mut status = LiveState::empty(now);
        status.connection.online = true;
        status.connection.last_tpdo1 = Some(now - Duration::from_millis(1));
        status.connection.last_tpdo2 = Some(now - Duration::from_millis(1));
        status.logic = Some(Logic::Disabled);
        status.measurements.position_rev = Some(raw_position);
        status.measurements.velocity_rev_per_s = Some(0.0);
        status.measurements.torque_nm = Some(0.0);
        status.measurements.driver_temp_c = Some(30.0);
        status.measurements.motor_temp_c = Some(31.0);
        status.measurements.status_word = Some(0x0231);
        status.measurements.tpdo1_error_code = Some(0);
        status.measurements.tpdo2_error_code = Some(0);
        status.measurements.timestamp_us = Some(123_456);
        status
    }

    fn healthy_initial_statuses(now: Instant) -> [LiveState; DOF] {
        array::from_fn(|_| healthy_initial_status(now, 0.05))
    }

    #[test]
    fn initial_feedback_gate_builds_all_six_unwrappers_atomically() {
        let now = Instant::now();
        let unwrappers = build_initial_feedback_unwrappers(
            &initial_gate_joints(),
            &healthy_initial_statuses(now),
            now,
            Duration::from_millis(100),
        )
        .unwrap();

        assert_eq!(unwrappers.len(), DOF);
        assert!(unwrappers
            .iter()
            .all(|unwrapper| (unwrapper.position_rev() - 0.05).abs() < f32::EPSILON));
    }

    #[test]
    fn initial_feedback_gate_keeps_8130_history_but_uses_current_fault_bit() {
        let now = Instant::now();
        let joints = initial_gate_joints();
        let mut retained_history = healthy_initial_statuses(now);
        for status in &mut retained_history {
            status.measurements.tpdo1_error_code = Some(0x8130);
            status.measurements.tpdo2_error_code = Some(0x8130);
            status.measurements.status_word = Some(0x0231);
            status.logic = Some(Logic::Disabled);
        }
        build_initial_feedback_unwrappers(
            &joints,
            &retained_history,
            now,
            Duration::from_millis(100),
        )
        .unwrap();

        retained_history[3].measurements.status_word = Some(0x0008);
        retained_history[3].logic = Some(Logic::Error {
            kind: hex_motor::types::MotorErrorKind::HeartbeatLost,
            raw_code: 0x8130,
        });
        let error = build_initial_feedback_unwrappers(
            &joints,
            &retained_history,
            now,
            Duration::from_millis(100),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("joint_4"));
        assert!(error.contains("current CiA402 Fault"));
        assert!(error.contains("0x8130"));
    }

    #[test]
    fn initial_feedback_gate_rejects_joint_branch_before_ready() {
        let now = Instant::now();
        let mut statuses = healthy_initial_statuses(now);
        // The fixture joint window is about [-0.159, 0.159] rev, so this
        // canonical raw sample has no permissible continuous branch.
        statuses[2].measurements.position_rev = Some(0.4);

        let error = build_initial_feedback_unwrappers(
            &initial_gate_joints(),
            &statuses,
            now,
            Duration::from_millis(100),
        )
        .unwrap_err()
        .to_string();

        assert!(error.contains("joint_3 (CANopen node 3)"));
        assert!(error.contains("single-turn feedback window"));
        assert!(error.contains("no periodic branch"));
    }

    #[test]
    fn initial_feedback_margin_accepts_hard_stop_measurement_without_widening_commands() {
        let now = Instant::now();
        let mut joints = initial_gate_joints();
        joints[1].limits.measured_position_margin_rad = 0.001;
        let command_mapping = joints[1].compressed_mapping();
        assert!((command_mapping.position_max - 1.0 / std::f32::consts::TAU).abs() < 1.0e-7);

        let mut within_margin = healthy_initial_statuses(now);
        within_margin[1].measurements.position_rev = Some(1.0008 / std::f32::consts::TAU);
        build_initial_feedback_unwrappers(&joints, &within_margin, now, Duration::from_millis(100))
            .unwrap();

        let mut outside_margin = healthy_initial_statuses(now);
        outside_margin[1].measurements.position_rev = Some(1.002 / std::f32::consts::TAU);
        let error = build_initial_feedback_unwrappers(
            &joints,
            &outside_margin,
            now,
            Duration::from_millis(100),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("joint_2"));
        assert!(error.contains("no periodic branch"));
    }

    #[test]
    fn initial_feedback_gate_requires_online_and_both_fresh_tpdo_streams() {
        let now = Instant::now();
        let joints = initial_gate_joints();

        let mut offline = healthy_initial_statuses(now);
        offline[0].connection.online = false;
        let error =
            build_initial_feedback_unwrappers(&joints, &offline, now, Duration::from_millis(100))
                .unwrap_err()
                .to_string();
        assert!(error.contains("joint_1 (CANopen node 1) is offline"));

        let mut no_tpdo1 = healthy_initial_statuses(now);
        no_tpdo1[1].connection.last_tpdo1 = None;
        let error =
            build_initial_feedback_unwrappers(&joints, &no_tpdo1, now, Duration::from_millis(100))
                .unwrap_err()
                .to_string();
        assert!(error.contains("joint_2 (CANopen node 2) has not received TPDO1"));

        let mut stale_tpdo2 = healthy_initial_statuses(now);
        stale_tpdo2[3].connection.last_tpdo2 = Some(now - Duration::from_millis(101));
        let error = build_initial_feedback_unwrappers(
            &joints,
            &stale_tpdo2,
            now,
            Duration::from_millis(100),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("joint_4 (CANopen node 4) TPDO2 is stale"));
    }

    #[test]
    fn initial_feedback_gate_rejects_missing_or_nonfinite_measurements() {
        let now = Instant::now();
        let joints = initial_gate_joints();

        let mut nonfinite_position = healthy_initial_statuses(now);
        nonfinite_position[4].measurements.position_rev = Some(f32::NAN);
        let error = build_initial_feedback_unwrappers(
            &joints,
            &nonfinite_position,
            now,
            Duration::from_millis(100),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("joint_5 (CANopen node 5) TPDO1 position_rev is non-finite"));

        let mut missing_velocity = healthy_initial_statuses(now);
        missing_velocity[5].measurements.velocity_rev_per_s = None;
        let error = build_initial_feedback_unwrappers(
            &joints,
            &missing_velocity,
            now,
            Duration::from_millis(100),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("joint_6 (CANopen node 6) has no decoded TPDO1 velocity_rev_per_s"));

        let mut nonfinite_temperature = healthy_initial_statuses(now);
        nonfinite_temperature[1].measurements.motor_temp_c = Some(f32::INFINITY);
        let error = build_initial_feedback_unwrappers(
            &joints,
            &nonfinite_temperature,
            now,
            Duration::from_millis(100),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("joint_2 (CANopen node 2) TPDO2 motor_temp_c is non-finite"));
    }

    #[derive(Default)]
    struct RecordingCommissioningOperations {
        configured: Mutex<Vec<usize>>,
        enabled: Mutex<Vec<usize>>,
    }

    #[async_trait]
    impl CommissioningAxisOperations for RecordingCommissioningOperations {
        async fn configure_selected(
            &self,
            selected_index: usize,
            _target: MotorTarget,
        ) -> Result<()> {
            self.configured.lock().push(selected_index);
            Ok(())
        }

        async fn enable_selected(&self, selected_index: usize) -> Result<()> {
            self.enabled.lock().push(selected_index);
            Ok(())
        }
    }

    struct RecordingJoint1ActivationOperations {
        events: Mutex<Vec<String>>,
        fail_configure: bool,
        fail_readback: bool,
    }

    #[async_trait]
    impl Joint1FirstPositionActivationOperations for RecordingJoint1ActivationOperations {
        async fn configure_joint1(
            &self,
            _target: MotorTarget,
            torque_permille: u16,
            kp_kd_torque_permille: u16,
        ) -> Result<()> {
            self.events.lock().push(format!(
                "configure:{torque_permille}/{kp_kd_torque_permille}"
            ));
            anyhow::ensure!(!self.fail_configure, "injected configure failure");
            Ok(())
        }

        async fn confirm_joint1_caps(
            &self,
            torque_permille: u16,
            kp_kd_torque_permille: u16,
        ) -> Result<()> {
            self.events.lock().push(format!(
                "readback:{torque_permille}/{kp_kd_torque_permille}"
            ));
            anyhow::ensure!(!self.fail_readback, "injected readback failure");
            Ok(())
        }

        async fn enable_joint1(&self) -> Result<()> {
            self.events.lock().push("enable".into());
            Ok(())
        }
    }

    struct RecordingJoint5ActivationOperations {
        events: Mutex<Vec<String>>,
        fail_configure: bool,
        fail_readback: bool,
    }

    #[async_trait]
    impl Joint5FirstPositionActivationOperations for RecordingJoint5ActivationOperations {
        async fn configure_joint5(
            &self,
            _target: MotorTarget,
            torque_permille: u16,
            kp_kd_torque_permille: u16,
        ) -> Result<()> {
            self.events.lock().push(format!(
                "configure:{torque_permille}/{kp_kd_torque_permille}"
            ));
            anyhow::ensure!(!self.fail_configure, "injected configure failure");
            Ok(())
        }

        async fn confirm_joint5_caps(
            &self,
            torque_permille: u16,
            kp_kd_torque_permille: u16,
        ) -> Result<()> {
            self.events.lock().push(format!(
                "readback:{torque_permille}/{kp_kd_torque_permille}"
            ));
            anyhow::ensure!(!self.fail_readback, "injected readback failure");
            Ok(())
        }

        async fn enable_joint5(&self) -> Result<()> {
            self.events.lock().push("enable".into());
            Ok(())
        }
    }

    struct RecordingJoint4ActivationOperations {
        events: Mutex<Vec<String>>,
        fail_configure: bool,
        fail_readback: bool,
    }

    #[async_trait]
    impl Joint4FirstPositionActivationOperations for RecordingJoint4ActivationOperations {
        async fn configure_joint4(
            &self,
            _target: MotorTarget,
            torque_permille: u16,
            kp_kd_torque_permille: u16,
        ) -> Result<()> {
            self.events.lock().push(format!(
                "configure:{torque_permille}/{kp_kd_torque_permille}"
            ));
            anyhow::ensure!(!self.fail_configure, "injected configure failure");
            Ok(())
        }

        async fn confirm_joint4_caps(
            &self,
            torque_permille: u16,
            kp_kd_torque_permille: u16,
        ) -> Result<()> {
            self.events.lock().push(format!(
                "readback:{torque_permille}/{kp_kd_torque_permille}"
            ));
            anyhow::ensure!(!self.fail_readback, "injected readback failure");
            Ok(())
        }

        async fn enable_joint4(&self) -> Result<()> {
            self.events.lock().push("enable".into());
            Ok(())
        }
    }

    struct RecordingJoint6ActivationOperations {
        events: Mutex<Vec<String>>,
        fail_configure: bool,
        fail_readback: bool,
    }

    #[async_trait]
    impl Joint6FirstPositionActivationOperations for RecordingJoint6ActivationOperations {
        async fn configure_joint6(
            &self,
            _target: MotorTarget,
            torque_permille: u16,
            kp_kd_torque_permille: u16,
        ) -> Result<()> {
            self.events.lock().push(format!(
                "configure:{torque_permille}/{kp_kd_torque_permille}"
            ));
            anyhow::ensure!(!self.fail_configure, "injected configure failure");
            Ok(())
        }

        async fn confirm_joint6_caps(
            &self,
            torque_permille: u16,
            kp_kd_torque_permille: u16,
        ) -> Result<()> {
            self.events.lock().push(format!(
                "readback:{torque_permille}/{kp_kd_torque_permille}"
            ));
            anyhow::ensure!(!self.fail_readback, "injected readback failure");
            Ok(())
        }

        async fn enable_joint6(&self) -> Result<()> {
            self.events.lock().push("enable".into());
            Ok(())
        }
    }

    struct RecordingDiagnosticShutdownOperations {
        events: Mutex<Vec<String>>,
        fail_restore: bool,
        fail_disable: Vec<usize>,
        fail_disarm: bool,
    }

    impl RecordingDiagnosticShutdownOperations {
        fn new(fail_restore: bool, fail_disable: Vec<usize>, fail_disarm: bool) -> Self {
            Self {
                events: Mutex::new(Vec::new()),
                fail_restore,
                fail_disable,
                fail_disarm,
            }
        }
    }

    #[async_trait]
    impl DiagnosticShutdownOperations for RecordingDiagnosticShutdownOperations {
        fn restore_baseline(&self, selected_index: usize) -> Result<()> {
            self.events.lock().push(format!("restore:{selected_index}"));
            anyhow::ensure!(!self.fail_restore, "injected baseline failure");
            Ok(())
        }

        async fn disable_axis_confirmed(&self, index: usize) -> Result<()> {
            self.events.lock().push(format!("disable:{index}"));
            anyhow::ensure!(
                !self.fail_disable.contains(&index),
                "injected disable failure for {index}"
            );
            Ok(())
        }

        fn stop_command_sender(&self) {
            self.events.lock().push("stop_sender".into());
        }

        async fn disarm_all_heartbeat_consumers(&self) -> Result<()> {
            self.events.lock().push("disarm".into());
            anyhow::ensure!(!self.fail_disarm, "injected heartbeat disarm failure");
            Ok(())
        }
    }

    #[tokio::test]
    async fn diagnostic_shutdown_is_selected_first_then_stops_and_disarms() {
        let operations = RecordingDiagnosticShutdownOperations::new(false, Vec::new(), false);
        run_selected_first_diagnostic_shutdown(&operations, 1)
            .await
            .unwrap();
        assert_eq!(
            *operations.events.lock(),
            [
                "restore:1",
                "disable:1",
                "disable:0",
                "disable:2",
                "disable:3",
                "disable:4",
                "disable:5",
                "stop_sender",
                "disarm",
            ]
        );
    }

    #[tokio::test]
    async fn joint1_diagnostic_shutdown_restores_then_disables_j1_before_every_other_axis() {
        let operations = RecordingDiagnosticShutdownOperations::new(false, Vec::new(), false);
        run_selected_first_diagnostic_shutdown(&operations, 0)
            .await
            .unwrap();
        assert_eq!(
            *operations.events.lock(),
            [
                "restore:0",
                "disable:0",
                "disable:1",
                "disable:2",
                "disable:3",
                "disable:4",
                "disable:5",
                "stop_sender",
                "disarm",
            ]
        );
    }

    #[tokio::test]
    async fn joint5_diagnostic_shutdown_restores_then_disables_j5_before_every_other_axis() {
        let operations = RecordingDiagnosticShutdownOperations::new(false, Vec::new(), false);
        run_selected_first_diagnostic_shutdown(&operations, 4)
            .await
            .unwrap();
        assert_eq!(
            *operations.events.lock(),
            [
                "restore:4",
                "disable:4",
                "disable:0",
                "disable:1",
                "disable:2",
                "disable:3",
                "disable:5",
                "stop_sender",
                "disarm",
            ]
        );
    }

    #[tokio::test]
    async fn joint4_diagnostic_shutdown_restores_then_disables_j4_before_every_other_axis() {
        let operations = RecordingDiagnosticShutdownOperations::new(false, Vec::new(), false);
        run_selected_first_diagnostic_shutdown(&operations, 3)
            .await
            .unwrap();
        assert_eq!(
            *operations.events.lock(),
            [
                "restore:3",
                "disable:3",
                "disable:0",
                "disable:1",
                "disable:2",
                "disable:4",
                "disable:5",
                "stop_sender",
                "disarm",
            ]
        );
    }

    #[tokio::test]
    async fn joint6_diagnostic_shutdown_restores_then_disables_j6_before_every_other_axis() {
        let operations = RecordingDiagnosticShutdownOperations::new(false, Vec::new(), false);
        run_selected_first_diagnostic_shutdown(&operations, 5)
            .await
            .unwrap();
        assert_eq!(
            *operations.events.lock(),
            [
                "restore:5",
                "disable:5",
                "disable:0",
                "disable:1",
                "disable:2",
                "disable:3",
                "disable:4",
                "stop_sender",
                "disarm",
            ]
        );
    }

    #[tokio::test]
    async fn diagnostic_shutdown_attempts_every_axis_but_retains_sender_and_watchdogs_on_failure() {
        let operations = RecordingDiagnosticShutdownOperations::new(false, vec![1, 4], false);
        let error = run_selected_first_diagnostic_shutdown(&operations, 1)
            .await
            .unwrap_err()
            .to_string();
        let events = operations.events.lock();
        assert_eq!(
            *events,
            [
                "restore:1",
                "disable:1",
                "disable:0",
                "disable:2",
                "disable:3",
                "disable:4",
                "disable:5",
            ]
        );
        assert!(!events.iter().any(|event| event == "stop_sender"));
        assert!(!events.iter().any(|event| event == "disarm"));
        assert!(error.contains("retaining the shared sender and every 0x1016 consumer"));
    }

    #[tokio::test]
    async fn diagnostic_baseline_failure_never_blocks_selected_first_disable() {
        let operations = RecordingDiagnosticShutdownOperations::new(true, Vec::new(), false);
        let error = run_selected_first_diagnostic_shutdown(&operations, 1)
            .await
            .unwrap_err()
            .to_string();
        let events = operations.events.lock();
        assert_eq!(events[0], "restore:1");
        assert_eq!(events[1], "disable:1");
        assert_eq!(events.last().unwrap(), "disarm");
        assert!(error.contains("baseline restore failed"));
    }

    #[tokio::test]
    async fn mock_bus_follows_targets_only_when_enabled() {
        let backend = MockBackend::new();
        let mut targets = [MotorTarget::default(); DOF];
        targets[0].position_rev = 1.0;
        backend.set_targets(targets).await.unwrap();
        assert_eq!(backend.feedback().joints[0].position_rev, 0.0);
        backend
            .enable_compressed_mit([MotorTarget::default(); DOF])
            .await
            .unwrap();
        backend.set_targets(targets).await.unwrap();
        assert!(backend.feedback().joints[0].position_rev > 0.0);
    }

    #[tokio::test]
    async fn enable_installs_nonzero_initial_targets_before_marking_enabled() {
        let backend = MockBackend::new();
        let mut initial = [MotorTarget::default(); DOF];
        initial[0].position_rev = 0.25;
        initial[0].kp_nm_rev = 12.0;

        backend.enable_compressed_mit(initial).await.unwrap();

        assert!(backend.is_enabled());
        assert_eq!(backend.targets(), initial);
        assert_ne!(backend.targets()[0], MotorTarget::default());
    }

    #[tokio::test]
    async fn commissioning_activation_configures_and_enables_only_the_selected_axis() {
        let operations = RecordingCommissioningOperations::default();

        activate_only_selected_axis(&operations, 3, MotorTarget::default())
            .await
            .unwrap();

        assert_eq!(*operations.configured.lock(), [3]);
        assert_eq!(*operations.enabled.lock(), [3]);
        assert!(!operations.configured.lock().contains(&0));
        assert!(!operations.configured.lock().contains(&1));
        assert!(!operations.configured.lock().contains(&2));
        assert!(!operations.configured.lock().contains(&4));
        assert!(!operations.configured.lock().contains(&5));
    }

    #[tokio::test]
    async fn joint1_fixed_activation_proves_30_20_caps_before_enable() {
        let operations = RecordingJoint1ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        activate_joint1_first_position_axis(
            &operations,
            MotorTarget::default(),
            J1_FIRST_POSITION_TORQUE_PERMILLE,
            J1_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .unwrap();
        assert_eq!(
            *operations.events.lock(),
            ["configure:30/20", "readback:30/20", "enable"]
        );
    }

    #[tokio::test]
    async fn joint1_fixed_activation_never_enables_after_write_or_readback_failure() {
        for (fail_configure, fail_readback, expected) in [
            (true, false, vec!["configure:30/20"]),
            (false, true, vec!["configure:30/20", "readback:30/20"]),
        ] {
            let operations = RecordingJoint1ActivationOperations {
                events: Mutex::new(Vec::new()),
                fail_configure,
                fail_readback,
            };
            assert!(activate_joint1_first_position_axis(
                &operations,
                MotorTarget::default(),
                J1_FIRST_POSITION_TORQUE_PERMILLE,
                J1_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
            )
            .await
            .is_err());
            assert_eq!(*operations.events.lock(), expected);
            assert!(!operations
                .events
                .lock()
                .iter()
                .any(|event| event == "enable"));
        }
    }

    #[tokio::test]
    async fn joint1_fixed_activation_rejects_nonfixed_caps_before_configuration() {
        let operations = RecordingJoint1ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        assert!(activate_joint1_first_position_axis(
            &operations,
            MotorTarget::default(),
            J1_FIRST_POSITION_TORQUE_PERMILLE + 1,
            J1_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .is_err());
        assert!(operations.events.lock().is_empty());
    }

    #[tokio::test]
    async fn joint5_fixed_activation_proves_50_30_caps_and_never_enables_on_failure() {
        let operations = RecordingJoint5ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        activate_joint5_first_position_axis(
            &operations,
            MotorTarget::default(),
            J5_FIRST_POSITION_TORQUE_PERMILLE,
            J5_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .unwrap();
        assert_eq!(
            *operations.events.lock(),
            ["configure:50/30", "readback:50/30", "enable"]
        );

        for (fail_configure, fail_readback, expected) in [
            (true, false, vec!["configure:50/30"]),
            (false, true, vec!["configure:50/30", "readback:50/30"]),
        ] {
            let operations = RecordingJoint5ActivationOperations {
                events: Mutex::new(Vec::new()),
                fail_configure,
                fail_readback,
            };
            assert!(activate_joint5_first_position_axis(
                &operations,
                MotorTarget::default(),
                J5_FIRST_POSITION_TORQUE_PERMILLE,
                J5_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
            )
            .await
            .is_err());
            assert_eq!(*operations.events.lock(), expected);
        }

        let operations = RecordingJoint5ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        assert!(activate_joint5_first_position_axis(
            &operations,
            MotorTarget::default(),
            J5_FIRST_POSITION_TORQUE_PERMILLE + 1,
            J5_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .is_err());
        assert!(operations.events.lock().is_empty());
    }

    #[tokio::test]
    async fn joint4_fixed_activation_proves_60_50_caps_and_never_enables_on_failure() {
        let operations = RecordingJoint4ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        activate_joint4_first_position_axis(
            &operations,
            MotorTarget::default(),
            J4_FIRST_POSITION_TORQUE_PERMILLE,
            J4_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .unwrap();
        assert_eq!(
            *operations.events.lock(),
            ["configure:60/50", "readback:60/50", "enable"]
        );

        for (fail_configure, fail_readback, expected) in [
            (true, false, vec!["configure:60/50"]),
            (false, true, vec!["configure:60/50", "readback:60/50"]),
        ] {
            let operations = RecordingJoint4ActivationOperations {
                events: Mutex::new(Vec::new()),
                fail_configure,
                fail_readback,
            };
            assert!(activate_joint4_first_position_axis(
                &operations,
                MotorTarget::default(),
                J4_FIRST_POSITION_TORQUE_PERMILLE,
                J4_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
            )
            .await
            .is_err());
            assert_eq!(*operations.events.lock(), expected);
        }

        let operations = RecordingJoint4ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        assert!(activate_joint4_first_position_axis(
            &operations,
            MotorTarget::default(),
            J4_FIRST_POSITION_TORQUE_PERMILLE + 1,
            J4_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .is_err());
        assert!(operations.events.lock().is_empty());
    }

    #[tokio::test]
    async fn joint4_assisted_activation_proves_only_90_50_before_enable() {
        let operations = RecordingJoint4ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        activate_joint4_assisted_position_axis(
            &operations,
            MotorTarget::default(),
            J4_ASSISTED_POSITION_TORQUE_PERMILLE,
            J4_ASSISTED_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .unwrap();
        assert_eq!(
            *operations.events.lock(),
            ["configure:90/50", "readback:90/50", "enable"]
        );

        for (fail_configure, fail_readback, expected) in [
            (true, false, vec!["configure:90/50"]),
            (false, true, vec!["configure:90/50", "readback:90/50"]),
        ] {
            let operations = RecordingJoint4ActivationOperations {
                events: Mutex::new(Vec::new()),
                fail_configure,
                fail_readback,
            };
            assert!(activate_joint4_assisted_position_axis(
                &operations,
                MotorTarget::default(),
                J4_ASSISTED_POSITION_TORQUE_PERMILLE,
                J4_ASSISTED_POSITION_KP_KD_TORQUE_PERMILLE,
            )
            .await
            .is_err());
            assert_eq!(*operations.events.lock(), expected);
        }

        let operations = RecordingJoint4ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        assert!(activate_joint4_assisted_position_axis(
            &operations,
            MotorTarget::default(),
            J4_FIRST_POSITION_TORQUE_PERMILLE,
            J4_ASSISTED_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .is_err());
        assert!(operations.events.lock().is_empty());
    }

    #[tokio::test]
    async fn joint6_fixed_activation_proves_50_30_caps_and_never_enables_on_failure() {
        let operations = RecordingJoint6ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        activate_joint6_first_position_axis(
            &operations,
            MotorTarget::default(),
            J6_FIRST_POSITION_TORQUE_PERMILLE,
            J6_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .unwrap();
        assert_eq!(
            *operations.events.lock(),
            ["configure:50/30", "readback:50/30", "enable"]
        );

        for (fail_configure, fail_readback, expected) in [
            (true, false, vec!["configure:50/30"]),
            (false, true, vec!["configure:50/30", "readback:50/30"]),
        ] {
            let operations = RecordingJoint6ActivationOperations {
                events: Mutex::new(Vec::new()),
                fail_configure,
                fail_readback,
            };
            assert!(activate_joint6_first_position_axis(
                &operations,
                MotorTarget::default(),
                J6_FIRST_POSITION_TORQUE_PERMILLE,
                J6_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
            )
            .await
            .is_err());
            assert_eq!(*operations.events.lock(), expected);
        }

        let operations = RecordingJoint6ActivationOperations {
            events: Mutex::new(Vec::new()),
            fail_configure: false,
            fail_readback: false,
        };
        assert!(activate_joint6_first_position_axis(
            &operations,
            MotorTarget::default(),
            J6_FIRST_POSITION_TORQUE_PERMILLE + 1,
            J6_FIRST_POSITION_KP_KD_TORQUE_PERMILLE,
        )
        .await
        .is_err());
        assert!(operations.events.lock().is_empty());
    }

    #[test]
    fn joint1_fixed_library_gate_rejects_every_historical_xstats_open() {
        validate_joint1_first_position_strict_zero_open(false).unwrap();
        let error = validate_joint1_first_position_strict_zero_open(true)
            .unwrap_err()
            .to_string();
        assert!(error.contains("requires a strict-zero backend open"));

        let backend_profile = recovery_profile();
        validate_joint1_first_position_strict_session(false, &backend_profile, &backend_profile)
            .unwrap();
        let value_equal_clone = backend_profile.clone();
        let error = validate_joint1_first_position_strict_session(
            false,
            &backend_profile,
            &value_equal_clone,
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("exact HardwareProfile allocation"));
        assert!(validate_joint1_first_position_strict_session(
            true,
            &backend_profile,
            &backend_profile,
        )
        .is_err());

        let source = include_str!("legacy.rs");
        let enable = source
            .split_once("pub async fn enable_joint1_first_position_diagnostic_axis(")
            .unwrap()
            .1
            .split_once("pub(crate) fn ensure_joint1_first_position_strict_zero_open")
            .unwrap()
            .0;
        assert!(
            enable
                .find("ensure_joint1_first_position_strict_zero_open()")
                .unwrap()
                < enable.find("prepare_single_axis_operation").unwrap()
        );
        let constructor = source
            .split_once("async fn open_with_diagnostic_xstats_acknowledgement(")
            .unwrap()
            .1
            .split_once("fn clear_mit_operation_expectations")
            .unwrap()
            .0;
        assert!(constructor.contains(
            "let used_historical_can_xstats_acknowledgement = acknowledgement.is_some()"
        ));
        assert!(constructor.contains("used_historical_can_xstats_acknowledgement,"));
    }

    #[test]
    fn joint5_fixed_library_gate_rejects_history_and_value_equal_profile_clone() {
        validate_joint5_first_position_strict_zero_open(false).unwrap();
        assert!(validate_joint5_first_position_strict_zero_open(true).is_err());
        let backend_profile = recovery_profile();
        validate_joint5_first_position_strict_session(false, &backend_profile, &backend_profile)
            .unwrap();
        let clone = backend_profile.clone();
        assert!(
            validate_joint5_first_position_strict_session(false, &backend_profile, &clone).is_err()
        );
    }

    #[test]
    fn joint3_unload_library_gate_and_activation_order_are_fail_closed() {
        validate_joint3_gravity_unload_strict_zero_open(false).unwrap();
        assert!(validate_joint3_gravity_unload_strict_zero_open(true).is_err());
        let backend_profile = recovery_profile();
        validate_joint3_gravity_unload_strict_session(false, &backend_profile, &backend_profile)
            .unwrap();
        let clone = backend_profile.clone();
        assert!(
            validate_joint3_gravity_unload_strict_session(false, &backend_profile, &clone).is_err()
        );

        let source = include_str!("legacy.rs");
        let body = source
            .split_once("pub async fn enable_joint3_gravity_unload_diagnostic_axis(")
            .unwrap()
            .1
            .split_once("pub async fn enable_joint5_first_position_diagnostic_axis(")
            .unwrap()
            .0;
        let strict = body
            .find("ensure_joint3_gravity_unload_strict_zero_open")
            .unwrap();
        let prepare = body.find("prepare_single_axis_operation").unwrap();
        let configure = body.find("configure_commissioning_axis_with_caps").unwrap();
        let readback = body.find("confirm_fixed_position_drive_caps").unwrap();
        let enable = body.find("enable_and_confirm_commissioning_axis").unwrap();
        assert!(strict < prepare);
        assert!(prepare < configure);
        assert!(configure < readback);
        assert!(readback < enable);
        assert!(body.contains("J3_GRAVITY_UNLOAD_TORQUE_PERMILLE"));
        assert!(body.contains("J3_GRAVITY_UNLOAD_KP_KD_TORQUE_PERMILLE"));
    }

    #[test]
    fn joint4_fixed_library_gate_rejects_history_and_value_equal_profile_clone() {
        validate_joint4_first_position_strict_zero_open(false).unwrap();
        assert!(validate_joint4_first_position_strict_zero_open(true).is_err());
        let backend_profile = recovery_profile();
        validate_joint4_first_position_strict_session(false, &backend_profile, &backend_profile)
            .unwrap();
        let clone = backend_profile.clone();
        assert!(
            validate_joint4_first_position_strict_session(false, &backend_profile, &clone).is_err()
        );
    }

    #[test]
    fn joint6_fixed_library_gate_rejects_history_and_value_equal_profile_clone() {
        validate_joint6_first_position_strict_zero_open(false).unwrap();
        assert!(validate_joint6_first_position_strict_zero_open(true).is_err());
        let backend_profile = recovery_profile();
        validate_joint6_first_position_strict_session(false, &backend_profile, &backend_profile)
            .unwrap();
        let clone = backend_profile.clone();
        assert!(
            validate_joint6_first_position_strict_session(false, &backend_profile, &clone).is_err()
        );
    }

    #[test]
    fn ordinary_selected_axis_configuration_still_uses_profile_caps() {
        let source = include_str!("legacy.rs");
        let body = source
            .split_once("async fn configure_commissioning_axis(")
            .unwrap()
            .1
            .split_once("async fn confirm_joint1_first_position_drive_caps(")
            .unwrap()
            .0;
        assert!(body.contains("joint.torque_permille"));
        assert!(body.contains("joint.kp_kd_torque_permille"));

        let ordinary_activation = source
            .split_once("async fn activate_only_selected_axis(")
            .unwrap()
            .1
            .split_once("trait Joint1FirstPositionActivationOperations")
            .unwrap()
            .0;
        assert!(!ordinary_activation.contains("J1_FIRST_POSITION_TORQUE_PERMILLE"));
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

    #[test]
    fn compressed_target_validation_rejects_values_the_packer_would_clamp() {
        let mapping = CompressedMitMapping::default();
        let valid = MotorTarget {
            position_rev: 0.25,
            velocity_rev_s: 0.1,
            torque_nm: 1.0,
            kp_nm_rev: 10.0,
            kd_nm_s_rev: 1.0,
        };
        validate_compressed_target(&valid, &mapping, "joint_1").unwrap();

        let mut outside = valid;
        outside.position_rev = mapping.position_max + 0.01;
        let error = validate_compressed_target(&outside, &mapping, "joint_1").unwrap_err();
        assert!(error.to_string().contains("position target"));

        outside = valid;
        outside.kp_nm_rev = mapping.kp_max + 1.0;
        let error = validate_compressed_target(&outside, &mapping, "joint_1").unwrap_err();
        assert!(error.to_string().contains("kp target"));
    }

    #[test]
    fn shared_commands_are_silent_until_hold_is_installed_and_after_disable() {
        let commands = SharedCommandState::new();
        assert!(commands.snapshot_for_send().is_none());

        let mut hold = [CompressedMitTarget::ZERO; DOF];
        hold[0].position = 0.25;
        hold[0].kp = 12.0;
        commands.install_hold_and_enable(hold);
        let first_frame_targets = commands.snapshot_for_send().unwrap();
        assert_eq!(first_frame_targets, hold);
        assert_ne!(first_frame_targets[0], CompressedMitTarget::ZERO);

        commands.stop_and_clear();
        assert!(commands.snapshot_for_send().is_none());
        assert_eq!(*commands.targets.read(), [CompressedMitTarget::ZERO; DOF]);
    }

    #[test]
    fn disable_confirmation_requires_a_new_tpdo2_and_confirmed_non_torque_status() {
        let command_completed_at = Instant::now();
        let mut status = LiveState::empty(command_completed_at);
        status.connection.last_tpdo2 = Some(command_completed_at - Duration::from_millis(1));
        status.measurements.status_word = Some(0x0040);
        assert!(!disable_feedback_confirmed(&status, command_completed_at));

        status.connection.last_tpdo2 = Some(command_completed_at);
        assert!(!disable_feedback_confirmed(&status, command_completed_at));

        status.connection.last_tpdo2 = Some(command_completed_at + Duration::from_millis(1));
        status.measurements.status_word = Some(0x0007);
        assert!(!disable_feedback_confirmed(&status, command_completed_at));

        status.measurements.status_word = Some(0x000F);
        assert!(!disable_feedback_confirmed(&status, command_completed_at));

        status.measurements.status_word = Some(0x0027);
        assert!(!disable_feedback_confirmed(&status, command_completed_at));

        status.measurements.status_word = Some(0x0231);
        assert!(disable_feedback_confirmed(&status, command_completed_at));
    }

    #[test]
    fn runtime_mit_expectation_rejects_every_post_enable_state_drop() {
        let expected = [true, false, false, false, false, false];
        let mut statuses = [Some(0x0237); DOF];
        assert_eq!(
            first_expected_mit_axis_not_operation_enabled(&expected, &statuses),
            None
        );

        for unsafe_status in [0x0007, 0x000F, 0x0023, 0x0021, 0x0040] {
            statuses[0] = Some(unsafe_status);
            assert_eq!(
                first_expected_mit_axis_not_operation_enabled(&expected, &statuses),
                Some((0, Some(unsafe_status)))
            );
        }
        statuses[0] = None;
        assert_eq!(
            first_expected_mit_axis_not_operation_enabled(&expected, &statuses),
            Some((0, None))
        );

        // Non-selected commissioning axes are deliberately ignored by this
        // runtime-OE expectation and remain governed by the non-torque gate.
        statuses = [Some(0x0040); DOF];
        statuses[0] = Some(0x0027);
        assert_eq!(
            first_expected_mit_axis_not_operation_enabled(&expected, &statuses),
            None
        );
    }

    #[test]
    fn isolated_diagnostic_state_requires_selected_oe_and_every_other_axis_non_torque() {
        let profile = recovery_profile();
        let mut statuses = [Some(0x0231); DOF];
        statuses[1] = Some(0x0237);
        validate_single_axis_commissioning_status_words(&profile, 1, &statuses).unwrap();

        statuses[3] = Some(0x0237);
        let unselected_oe = validate_single_axis_commissioning_status_words(&profile, 1, &statuses)
            .unwrap_err()
            .to_string();
        assert!(unselected_oe.contains("unselected joint_4"));

        statuses[3] = Some(0x0007);
        assert!(validate_single_axis_commissioning_status_words(&profile, 1, &statuses).is_err());
        statuses[3] = Some(0x0231);
        statuses[1] = Some(0x000F);
        let selected_not_oe =
            validate_single_axis_commissioning_status_words(&profile, 1, &statuses)
                .unwrap_err()
                .to_string();
        assert!(selected_not_oe.contains("joint_2"));
        assert!(selected_not_oe.contains("not strict Operation Enabled"));
    }

    #[test]
    fn confirmed_mit_enable_requires_post_set_mode_tpdo2_and_actual_mode_display() {
        let command_completed_at = Instant::now();
        let mut status = LiveState::empty(command_completed_at);
        status.connection.last_tpdo2 = Some(command_completed_at - Duration::from_millis(1));
        status.measurements.status_word = Some(0x0027);
        assert!(!compressed_mit_enable_confirmed(
            &status,
            command_completed_at,
            Some(MIT_MODE_DISPLAY)
        ));

        status.connection.last_tpdo2 = Some(command_completed_at);
        assert!(!compressed_mit_enable_confirmed(
            &status,
            command_completed_at,
            Some(MIT_MODE_DISPLAY)
        ));

        status.connection.last_tpdo2 = Some(command_completed_at + Duration::from_millis(1));
        status.measurements.status_word = Some(0x0040);
        assert!(!compressed_mit_enable_confirmed(
            &status,
            command_completed_at,
            Some(MIT_MODE_DISPLAY)
        ));

        status.measurements.status_word = Some(0x0007);
        assert!(!compressed_mit_enable_confirmed(
            &status,
            command_completed_at,
            Some(MIT_MODE_DISPLAY)
        ));
        status.measurements.status_word = Some(0x000F);
        assert!(!compressed_mit_enable_confirmed(
            &status,
            command_completed_at,
            Some(MIT_MODE_DISPLAY)
        ));

        status.measurements.status_word = Some(0x0027);
        assert!(!compressed_mit_enable_confirmed(
            &status,
            command_completed_at,
            None
        ));
        assert!(!compressed_mit_enable_confirmed(
            &status,
            command_completed_at,
            Some(3)
        ));
        assert!(compressed_mit_enable_confirmed(
            &status,
            command_completed_at,
            Some(MIT_MODE_DISPLAY)
        ));
    }
}
