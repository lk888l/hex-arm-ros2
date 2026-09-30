//! Latest Meow Motor firmware, separate from the legacy CiA402 driver.
//! Canonical MIT uses an atomic 20-byte RPDO per axis; initialization never
//! clears faults, saves parameters, or enables a motor.
use super::{FeedbackSnapshot, JointFeedback, MotorBackend, MotorIdentitySnapshot, DOF};
use crate::conversion::MotorTarget;
use crate::discovery::{discover_read_only, DiscoveryOptions};
use crate::profile::{BusTransport, HardwareProfile, MeowPdAllocation, MeowTorqueBudget};
use crate::socketcan_preflight::{preflight_socketcan, validate_runtime_socketcan};
use anyhow::{Context, Result};
use async_trait::async_trait;
use can_transport::socketcan::SocketCanBus;
use can_transport::{CanBus, CanFilter, CanFrame, CanId};
use hex_meow_motor::canopen::rpdo_config::{build_rpdo_config_writes, RpdoRecipe};
use hex_meow_motor::canopen::tpdo_config::{build_tpdo_config_writes, SdoWrite, TpdoEntry};
use hex_meow_motor::canopen::{heartbeat, nmt, sdo};
use hex_meow_motor::cia402::PdoProfile;
use hex_meow_motor::meow_motor::{self, MeowFactoryCalibration, SignedQ8_24};
use hex_meow_motor::types::MotorIdentity;
use parking_lot::RwLock;
use std::array;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::Mutex;
use tokio::task::JoinHandle;
use tokio::time::MissedTickBehavior;
const TIMEOUT: Option<Duration> = Some(Duration::from_millis(250));
const MODE_TIMEOUT: Duration = Duration::from_secs(2);
const HEARTBEAT_TIMEOUT_MS: u16 = 250;
#[derive(Clone, Copy, Debug)]
struct Calibration {
    peak_nm: f32,
    torque_factor: f32,
    gain_factor: f32,
}
#[derive(Clone, Copy, Debug, Default)]
struct Telemetry {
    position: Option<SignedQ8_24>,
    timestamp_us: Option<u16>,
    velocity: Option<f32>,
    torque_permille: i16,
    mode: Option<u8>,
    error: u16,
    driver_c: f32,
    motor_c: f32,
    heartbeat_at: Option<Instant>,
    nmt: Option<nmt::NmtState>,
    tpdo1_at: Option<Instant>,
    tpdo2_at: Option<Instant>,
}
impl Telemetry {
    fn fresh(&self, now: Instant, timeout: Duration) -> bool {
        [self.tpdo1_at, self.tpdo2_at]
            .into_iter()
            .all(|stamp| stamp.is_some_and(|stamp| now.saturating_duration_since(stamp) <= timeout))
            && self.heartbeat_at.is_some_and(|stamp| {
                now.saturating_duration_since(stamp) <= Duration::from_millis(1_500)
            })
            && self.nmt == Some(nmt::NmtState::Operational)
            && self.position.is_some()
            && self.velocity.is_some()
    }
}
pub struct MeowBackend {
    profile: Arc<HardwareProfile>,
    bus: Arc<dyn CanBus>,
    telemetry: Arc<RwLock<[Telemetry; DOF]>>,
    calibration: RwLock<[Option<Calibration>; DOF]>,
    identities: RwLock<Vec<MotorIdentitySnapshot>>,
    touched: RwLock<[bool; DOF]>,
    consumers: RwLock<[bool; DOF]>,
    initialized: AtomicBool,
    heartbeat_enabled: Arc<AtomicBool>,
    stream: Arc<RwLock<Option<[[u8; 20]; DOF]>>>,
    expected_mit: Arc<RwLock<[bool; DOF]>>,
    failed: Arc<AtomicBool>,
    operations: Mutex<()>,
    tasks: Vec<JoinHandle<()>>,
}
impl Drop for MeowBackend {
    fn drop(&mut self) {
        // Failed shutdown leaves 0x1016 armed; stopping the producer then lets
        // the motor's heartbeat protection take effect.
        for task in &self.tasks {
            task.abort();
        }
    }
}
impl MeowBackend {
    pub async fn open(profile: Arc<HardwareProfile>) -> Result<Self> {
        anyhow::ensure!(profile.joints.len() == DOF, "Meow requires six joints");
        anyhow::ensure!(
            profile.bus.transport == BusTransport::SocketCan,
            "Meow hardware requires the verified SocketCAN transport"
        );
        anyhow::ensure!(
            profile.joints.iter().all(|joint| joint.torque_scale == 1.0),
            "Meow torque_scale must be 1; factory calibration is read from each motor"
        );
        anyhow::ensure!(
            matches!(profile.controller.loop_hz, 500 | 1000),
            "Meow loop rate must be 500 or 1000 Hz"
        );
        let expected = profile
            .bus
            .expected_link
            .as_ref()
            .context("SocketCAN expected_link required")?;
        let baseline = preflight_socketcan(&profile.bus.interface, expected)?;
        let bus: Arc<dyn CanBus> = Arc::new(SocketCanBus::open(&profile.bus.interface)?);
        anyhow::ensure!(bus.capabilities().fd, "Meow MIT requires CAN-FD");
        let mut rx = bus.subscribe(CanFilter::standard(0, 0)).await?;
        let telemetry = Arc::new(RwLock::new([Telemetry::default(); DOF]));
        let failed = Arc::new(AtomicBool::new(false));
        let heartbeat_enabled = Arc::new(AtomicBool::new(false));
        let stream = Arc::new(RwLock::new(None::<[[u8; 20]; DOF]>));
        let expected_mit = Arc::new(RwLock::new([false; DOF]));
        let mut tasks = Vec::new();
        {
            let telemetry = telemetry.clone();
            let failed = failed.clone();
            let expected_mit = expected_mit.clone();
            let profile = profile.clone();
            tasks.push(tokio::spawn(async move {
                loop {
                    let frame = match rx.recv().await {
                        Ok(frame) => frame,
                        Err(error) => {
                            tracing::error!(%error, "Meow CAN receive failed");
                            failed.store(true, Ordering::Release);
                            break;
                        }
                    };
                    let CanId::Standard(cob) = frame.id() else {
                        continue;
                    };
                    let now = Instant::now();
                    if (0x701..=0x77f).contains(&cob) {
                        let node = (cob - 0x700) as u8;
                        if node != profile.bus.heartbeat_node_id
                            && !profile.bus.auxiliary_node_ids.contains(&node)
                            && !profile.joints.iter().any(|joint| joint.node_id == node)
                        {
                            failed.store(true, Ordering::Release);
                        }
                    }
                    let Some(index) = profile.joints.iter().position(|joint| {
                        let node = u16::from(joint.node_id);
                        [0x180 + node, 0x280 + node, 0x700 + node, 0x80 + node].contains(&cob)
                    }) else {
                        continue;
                    };
                    let node = u16::from(profile.joints[index].node_id);
                    let mut states = telemetry.write();
                    let state = &mut states[index];
                    if cob == 0x180 + node {
                        if let Ok(sample) = meow_motor::decode_tpdo1(frame.data()) {
                            let contiguous = state.tpdo1_at.is_some_and(|stamp| {
                                now.saturating_duration_since(stamp) <= Duration::from_millis(30)
                            });
                            state.velocity = if contiguous {
                                state
                                    .position
                                    .zip(state.timestamp_us)
                                    .and_then(|(previous, ts)| {
                                        let dt = sample.timestamp_us.wrapping_sub(ts);
                                        (dt > 0).then(|| {
                                            (sample.position.wrapping_delta(previous) * 1_000_000.0
                                                / f64::from(dt))
                                                as f32
                                        })
                                    })
                            } else {
                                None
                            };
                            state.position = Some(sample.position);
                            state.timestamp_us = Some(sample.timestamp_us);
                            state.torque_permille = sample.torque_permille;
                            state.tpdo1_at = Some(now);
                        }
                    } else if cob == 0x280 + node {
                        if let Ok(sample) = meow_motor::decode_tpdo2(frame.data()) {
                            if sample.logic().is_ok() {
                                state.mode = Some(sample.mode_display);
                                state.error = sample.detailed_error;
                                state.driver_c = sample.driver_temp_c();
                                state.motor_c = sample.motor_temp_c();
                                state.tpdo2_at = Some(now);
                                if expected_mit.read()[index]
                                    && (sample.mode_display != 4 || sample.detailed_error != 0)
                                {
                                    failed.store(true, Ordering::Release);
                                }
                            }
                        }
                    } else if cob == 0x700 + node {
                        if let Some((_, nmt)) = nmt::parse_heartbeat(&frame) {
                            state.heartbeat_at = Some(now);
                            state.nmt = Some(nmt);
                            if expected_mit.read()[index] && nmt != nmt::NmtState::Operational {
                                failed.store(true, Ordering::Release);
                            }
                        }
                    } else if cob == 0x80 + node && frame.data().len() >= 2 {
                        let code = u16::from_le_bytes([frame.data()[0], frame.data()[1]]);
                        if code != 0 {
                            state.error = code;
                            if expected_mit.read()[index] {
                                failed.store(true, Ordering::Release);
                            }
                        }
                    }
                }
            }));
        }
        {
            let bus = bus.clone();
            let enabled = heartbeat_enabled.clone();
            let failed = failed.clone();
            let host = profile.bus.heartbeat_node_id;
            tasks.push(tokio::spawn(async move {
                let mut tick = tokio::time::interval(Duration::from_millis(50));
                tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
                loop {
                    tick.tick().await;
                    if enabled.load(Ordering::Acquire) {
                        let frame =
                            heartbeat::build_heartbeat_frame(host, nmt::NmtState::Operational)
                                .unwrap();
                        if !matches!(
                            tokio::time::timeout(Duration::from_millis(10), bus.send(frame)).await,
                            Ok(Ok(()))
                        ) {
                            failed.store(true, Ordering::Release);
                            break;
                        }
                    }
                }
            }));
        }
        {
            let bus = bus.clone();
            let stream = stream.clone();
            let failed = failed.clone();
            let profile = profile.clone();
            tasks.push(tokio::spawn(async move {
                let mut tick = tokio::time::interval(Duration::from_secs_f64(
                    1.0 / f64::from(profile.controller.loop_hz),
                ));
                tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
                loop {
                    tick.tick().await;
                    let payloads = *stream.read();
                    if let Some(payloads) = payloads {
                        if let Err(error) = send_payloads(bus.as_ref(), &profile, &payloads).await {
                            tracing::error!(%error, "Meow MIT PDO stream failed");
                            failed.store(true, Ordering::Release);
                            break;
                        }
                    }
                }
            }));
        }
        {
            let failed = failed.clone();
            let interface = profile.bus.interface.clone();
            tasks.push(tokio::spawn(async move {
                let mut tick = tokio::time::interval(Duration::from_secs(1));
                tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
                loop {
                    tick.tick().await;
                    let interface = interface.clone();
                    let baseline = baseline.clone();
                    let result = tokio::task::spawn_blocking(move || {
                        validate_runtime_socketcan(&interface, &baseline)
                    });
                    if !matches!(
                        tokio::time::timeout(Duration::from_millis(500), result).await,
                        Ok(Ok(Ok(())))
                    ) {
                        failed.store(true, Ordering::Release);
                        break;
                    }
                }
            }));
        }
        Ok(Self {
            profile,
            bus,
            telemetry,
            calibration: RwLock::new([None; DOF]),
            identities: RwLock::new(Vec::new()),
            touched: RwLock::new([false; DOF]),
            consumers: RwLock::new([false; DOF]),
            initialized: AtomicBool::new(false),
            heartbeat_enabled,
            stream,
            expected_mit,
            failed,
            operations: Mutex::new(()),
            tasks,
        })
    }
    async fn discover_inner(&self) -> Result<Vec<MotorIdentitySnapshot>> {
        let report = discover_read_only(
            self.bus.clone(),
            &DiscoveryOptions {
                expected_node_ids: self
                    .profile
                    .joints
                    .iter()
                    .map(|joint| joint.node_id)
                    .collect(),
                auxiliary_node_ids: self
                    .profile
                    .bus
                    .auxiliary_node_ids
                    .iter()
                    .copied()
                    .collect(),
                observe_timeout: Duration::from_millis(
                    self.profile.controller.discovery_timeout_ms,
                ),
                sdo_timeout: TIMEOUT.unwrap(),
            },
        )
        .await?;
        anyhow::ensure!(
            !report.has_failures(),
            "Meow read-only discovery failed: {report:?}"
        );
        if let Some(payload) = &self.profile.tip_payload {
            let actual = report
                .nodes
                .iter()
                .find(|node| node.node_id == payload.auxiliary_node_id)
                .and_then(|node| node.identity.as_ref())
                .context("configured payload node is missing")?;
            let expected = &payload.identity;
            anyhow::ensure!(
                actual.vendor_id == expected.vendor_id
                    && actual.product_code == expected.product_code
                    && actual.revision_number == expected.revision
                    && actual.serial_number == expected.serial_number,
                "payload node identity does not match the mass/inertia configuration"
            );
        }
        let mut snapshots = Vec::with_capacity(DOF);
        for joint in &self.profile.joints {
            let found = report
                .nodes
                .iter()
                .find(|node| node.node_id == joint.node_id)
                .and_then(|node| node.identity.as_ref())
                .context("missing Meow motor identity")?;
            let model = meow_motor::lookup_model(found.vendor_id, found.product_code)
                .with_context(|| {
                    format!(
                        "node {} is not the new Meow Motor protocol family",
                        joint.node_id
                    )
                })?;
            let expected = &joint.identity;
            let verified = found.vendor_id == expected.vendor_id
                && found.product_code == expected.product_code
                && found.revision_number == expected.revision
                && found.serial_number == expected.serial_number;
            snapshots.push(MotorIdentitySnapshot {
                node_id: joint.node_id,
                vendor_id: found.vendor_id,
                product_code: found.product_code,
                revision: found.revision_number,
                serial_number: found.serial_number,
                model: model.display_name().into(),
                identity_verified: verified,
            });
        }
        *self.identities.write() = snapshots.clone();
        Ok(snapshots)
    }
    async fn read_mode(&self, node: u8) -> Result<(u8, u16)> {
        Ok((
            sdo::upload_u8(self.bus.as_ref(), node, 0x4402, 0, TIMEOUT).await?,
            sdo::upload_u16(self.bus.as_ref(), node, 0x453F, 0, TIMEOUT).await?,
        ))
    }
    async fn confirm_mode(&self, node: u8, expected: u8, allow_error: bool) -> Result<()> {
        let deadline = Instant::now() + MODE_TIMEOUT;
        loop {
            let (mode, error) = self.read_mode(node).await?;
            if mode == expected && (allow_error || error == 0) {
                return Ok(());
            }
            if expected == 0 && allow_error && matches!(mode, 0xA1..=0xA6 | 0xAF) {
                return Ok(());
            }
            anyhow::ensure!(Instant::now() < deadline,
                "node {node} mode confirmation failed: expected {expected}, mode=0x{mode:02X}, error=0x{error:04X}");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }
    async fn write_checked(&self, node: u8, write: &SdoWrite) -> Result<()> {
        sdo::download(
            self.bus.as_ref(),
            node,
            write.index,
            write.subindex,
            &write.data,
            TIMEOUT,
        )
        .await?;
        let actual = sdo::upload(
            self.bus.as_ref(),
            node,
            write.index,
            write.subindex,
            TIMEOUT,
        )
        .await?;
        anyhow::ensure!(
            actual == write.data,
            "node {node} readback mismatch at 0x{:04X}:{:02X}",
            write.index,
            write.subindex
        );
        tokio::time::sleep(Duration::from_millis(2)).await;
        Ok(())
    }
    async fn change_nmt(
        &self,
        index: usize,
        command: nmt::NmtCommand,
        expected: nmt::NmtState,
    ) -> Result<()> {
        let since = Instant::now();
        self.bus
            .send(nmt::build_nmt_command(
                command,
                self.profile.joints[index].node_id,
            )?)
            .await?;
        let deadline = Instant::now() + MODE_TIMEOUT;
        loop {
            let state = self.telemetry.read()[index];
            if state.nmt == Some(expected) && state.heartbeat_at.is_some_and(|stamp| stamp > since)
            {
                return Ok(());
            }
            anyhow::ensure!(
                Instant::now() < deadline,
                "Meow NMT state was not confirmed for axis {}",
                index + 1
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }
    async fn fresh_mode_after(&self, index: usize, since: Instant, expected: u8) -> Result<()> {
        let deadline = Instant::now() + MODE_TIMEOUT;
        loop {
            anyhow::ensure!(
                !self.transport_failed(),
                "Meow feedback confirmation interrupted by a latched failure"
            );
            let state = self.telemetry.read()[index];
            if state.fresh(Instant::now(), self.profile.feedback_timeout())
                && state.tpdo1_at.is_some_and(|stamp| stamp > since)
                && state.tpdo2_at.is_some_and(|stamp| stamp > since)
                && state.mode == Some(expected)
                && state.error == 0
            {
                return Ok(());
            }
            anyhow::ensure!(
                Instant::now() < deadline,
                "Meow axis {} lacks fresh confirmed mode {expected} feedback",
                index + 1
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }
    fn encode_targets(&self, targets: [MotorTarget; DOF]) -> Result<[[u8; 20]; DOF]> {
        let calibration = *self.calibration.read();
        let mut payloads = [[0u8; 20]; DOF];
        for (index, joint) in self.profile.joints.iter().enumerate() {
            let position_rad =
                crate::conversion::motor_position_to_ros(targets[index].position_rev, joint);
            let velocity_rad_s =
                crate::conversion::motor_velocity_to_ros(targets[index].velocity_rev_s, joint);
            anyhow::ensure!(
                position_rad >= joint.limits.position_lower_rad - 1.0e-5
                    && position_rad <= joint.limits.position_upper_rad + 1.0e-5
                    && velocity_rad_s.abs() <= joint.limits.velocity_rad_s + 1.0e-5,
                "{} MIT target exceeds configured position/velocity limits",
                joint.name
            );
            payloads[index] = encode_target(
                targets[index],
                calibration[index].context("Meow calibration missing")?,
                joint.torque_permille,
                joint.kp_kd_torque_permille,
                joint.meow_torque_budget,
            )?;
        }
        Ok(payloads)
    }
    async fn disable_inner(&self) -> Result<()> {
        *self.expected_mit.write() = [false; DOF];
        let touched = *self.touched.read();
        let mut errors = Vec::new();
        for (index, joint) in self.profile.joints.iter().enumerate() {
            if !touched[index] {
                continue;
            }
            let outcome = async {
                sdo::download_u8(self.bus.as_ref(), joint.node_id, 0x4401, 0, 0, TIMEOUT).await?;
                self.confirm_mode(joint.node_id, 0, true).await
            }
            .await;
            if let Err(error) = outcome {
                errors.push(format!("{}: {error:#}", joint.name));
            }
        }
        if !errors.is_empty() {
            // The drive watchdog is the final stop if SDO/status confirmation
            // fails. Leave its consumer armed and stop feeding it; a live host
            // heartbeat must not keep an unconfirmed axis powered forever.
            self.failed.store(true, Ordering::Release);
            self.heartbeat_enabled.store(false, Ordering::Release);
            anyhow::bail!(
                "Meow disable not confirmed; heartbeat backstop armed, restart required: {}",
                errors.join("; ")
            );
        }
        // Keep holding until authoritative mode reads prove every touched motor
        // has left its torque-producing mode, including partial initialization.
        *self.stream.write() = None;
        Ok(())
    }
}
#[async_trait]
impl MotorBackend for MeowBackend {
    async fn discover(&self, _refresh: bool) -> Result<Vec<MotorIdentitySnapshot>> {
        let _guard = self.operations.lock().await;
        self.discover_inner().await
    }
    async fn initialize_disabled(&self) -> Result<()> {
        let _guard = self.operations.lock().await;
        anyhow::ensure!(
            !self.initialized.load(Ordering::Acquire),
            "Meow is already initialized"
        );
        anyhow::ensure!(
            !self.failed.load(Ordering::Acquire),
            "Meow backend has failed; restart before initialization"
        );
        let identities = self.discover_inner().await?;
        anyhow::ensure!(identities.iter().all(|id| id.identity_verified),
            "all six exact Meow identities must match the commissioned profile before configuration writes");
        let consumer = heartbeat::encode_consumer_heartbeat_entry(
            self.profile.bus.heartbeat_node_id,
            HEARTBEAT_TIMEOUT_MS,
        );
        for joint in &self.profile.joints {
            let (mode, error) = self.read_mode(joint.node_id).await?;
            anyhow::ensure!(mode == 0 && error == 0,
                "{} must be disabled and fault-free before deployment (mode=0x{mode:02X}, error=0x{error:04X}); stop the GUI or explicitly recover first", joint.name);
            let existing =
                sdo::upload_u32(self.bus.as_ref(), joint.node_id, 0x1016, 1, TIMEOUT).await?;
            anyhow::ensure!(existing == 0 || existing == consumer,
                "{} heartbeat consumer belongs to another host: 0x{existing:08X}; refusing configuration", joint.name);
        }
        anyhow::ensure!(
            !self.failed.load(Ordering::Acquire),
            "Meow bus failed during read-only initialization checks"
        );
        self.heartbeat_enabled.store(true, Ordering::Release);
        for (index, joint) in self.profile.joints.iter().enumerate() {
            let node = joint.node_id;
            self.touched.write()[index] = true;
            sdo::download_u8(self.bus.as_ref(), node, 0x4401, 0, 0, TIMEOUT).await?;
            self.confirm_mode(node, 0, false).await?;
            self.change_nmt(
                index,
                nmt::NmtCommand::EnterPreOperational,
                nmt::NmtState::PreOperational,
            )
            .await?;
            self.confirm_mode(node, 0, false).await?;
            let identity = &identities[index];
            let identity = MotorIdentity {
                node_id: node,
                vendor_id: identity.vendor_id,
                product_code: identity.product_code,
                revision_number: identity.revision,
                serial_number: identity.serial_number,
                product_name: None,
            };
            let factory = meow_motor::calibration::read_from_device(
                self.bus.as_ref(),
                node,
                &identity,
                TIMEOUT.unwrap(),
            )
            .await?;
            let torque_factor = match factory {
                MeowFactoryCalibration::Valid(factory) => factory.torque.factor as f32,
                MeowFactoryCalibration::Missing { reason, .. } => {
                    // Match the proven GUI contract: a successful read with no
                    // valid factory record uses unity; transport errors above
                    // remain fatal and are never converted into a fallback.
                    tracing::warn!(node, %reason, "no valid factory torque record; using GUI unity fallback");
                    1.0
                }
            };
            let gain_factor = sdo::upload_f32(self.bus.as_ref(), node, 0x4102, 7, TIMEOUT).await?;
            let peak_nm = sdo::upload_f32(self.bus.as_ref(), node, 0x4576, 0, TIMEOUT).await?;
            anyhow::ensure!(
                [gain_factor, peak_nm, torque_factor]
                    .into_iter()
                    .all(|v| v.is_finite() && v > f32::EPSILON),
                "{} returned invalid MIT/peak/factory scales",
                joint.name
            );
            self.calibration.write()[index] = Some(Calibration {
                gain_factor,
                peak_nm,
                torque_factor,
            });
            tracing::info!(
                node,
                gain_factor,
                peak_nm,
                torque_factor,
                "Meow calibration read; Tff and gain scales are separate"
            );
            self.write_checked(node, &SdoWrite::u8(0x4103, 1, 0))
                .await?;
            self.write_checked(node, &SdoWrite::u16(0x4572, 0, joint.torque_permille))
                .await?;
            let profile = PdoProfile::from_event_timer_ms(
                (1000 / self.profile.controller.loop_hz).max(1) as u16,
            )?;
            for recipe in [
                meow_motor::device_tpdo1_recipe(node, profile)?,
                meow_motor::device_tpdo2_recipe(node)?,
            ] {
                for write in build_tpdo_config_writes(&recipe)? {
                    self.write_checked(node, &write).await?;
                }
            }
            for write in build_rpdo_config_writes(&mit_recipe(node))? {
                self.write_checked(node, &write).await?;
            }
            let existing = sdo::upload_u32(self.bus.as_ref(), node, 0x1016, 1, TIMEOUT).await?;
            anyhow::ensure!(
                existing == 0 || existing == consumer,
                "{} heartbeat consumer belongs to another host: 0x{existing:08X}",
                joint.name
            );
            self.consumers.write()[index] = true;
            self.write_checked(node, &SdoWrite::u32(0x1016, 1, consumer))
                .await?;
            self.change_nmt(
                index,
                nmt::NmtCommand::StartRemoteNode,
                nmt::NmtState::Operational,
            )
            .await?;
            self.fresh_mode_after(index, Instant::now(), 0).await?;
        }
        anyhow::ensure!(
            !self.failed.load(Ordering::Acquire),
            "Meow transport failed during initialization"
        );
        self.initialized.store(true, Ordering::Release);
        Ok(())
    }
    async fn enable_compressed_mit(&self, initial_targets: [MotorTarget; DOF]) -> Result<()> {
        let _guard = self.operations.lock().await;
        anyhow::ensure!(
            self.initialized.load(Ordering::Acquire),
            "Meow initialization required"
        );
        anyhow::ensure!(!self.transport_failed(), "Meow transport has failed");
        let feedback = self.feedback();
        anyhow::ensure!(
            feedback.all_online_and_fresh(),
            "Meow enable requires fresh feedback for all axes"
        );
        for (index, initial_target) in initial_targets.iter().enumerate() {
            anyhow::ensure!(
                self.telemetry.read()[index].mode == Some(0),
                "Meow axis {} is already enabled",
                index + 1
            );
            anyhow::ensure!(
                (initial_target.position_rev - feedback.joints[index].position_rev).abs() <= 0.002
                    && initial_target.velocity_rev_s.abs() <= 1.0e-6,
                "Meow enable target must hold the current measured position on axis {}",
                index + 1
            );
        }
        let payloads = self.encode_targets(initial_targets)?;
        *self.stream.write() = Some(payloads);
        let result: Result<()> = async {
            send_payloads(self.bus.as_ref(), &self.profile, &payloads).await?;
            // Read the actual RPDO target before mode=MIT; a queued hold packet
            // is not proof that a motor received the safe command.
            for (index, joint) in self.profile.joints.iter().enumerate() {
                for (sub, start, end) in [
                    (1, 0, 4),
                    (2, 4, 8),
                    (3, 8, 12),
                    (4, 12, 14),
                    (5, 14, 16),
                    (6, 16, 18),
                ] {
                    let actual =
                        sdo::upload(self.bus.as_ref(), joint.node_id, 0x4102, sub, TIMEOUT).await?;
                    anyhow::ensure!(
                        actual == payloads[index][start..end],
                        "{} MIT hold readback failed at sub {sub}",
                        joint.name
                    );
                }
            }
            for (index, joint) in self.profile.joints.iter().enumerate() {
                anyhow::ensure!(
                    !self.transport_failed(),
                    "Meow startup interrupted by a transport or enabled-axis fault"
                );
                self.confirm_mode(joint.node_id, 0, false).await?;
                let current = self.telemetry.read()[index];
                anyhow::ensure!(
                    current.fresh(Instant::now(), self.profile.feedback_timeout())
                        && current.position.is_some_and(|p| (p.to_revolutions() as f32
                            - initial_targets[index].position_rev)
                            .abs()
                            <= 0.002),
                    "{} moved during startup; refusing stale hold target",
                    joint.name
                );
                sdo::download_u8(self.bus.as_ref(), joint.node_id, 0x4401, 0, 4, TIMEOUT).await?;
                let since = Instant::now();
                self.confirm_mode(joint.node_id, 4, false).await?;
                self.fresh_mode_after(index, since, 4).await?;
                self.expected_mit.write()[index] = true;
                anyhow::ensure!(
                    !self.transport_failed(),
                    "Meow axis fault during startup confirmation"
                );
            }
            Ok(())
        }
        .await;
        if let Err(error) = result {
            return match self.disable_inner().await {
                Ok(()) => Err(error),
                Err(disable) => Err(anyhow::anyhow!(
                    "Meow enable failed: {error:#}; disable also failed: {disable:#}"
                )),
            };
        }
        Ok(())
    }
    fn validate_targets(&self, targets: [MotorTarget; DOF]) -> Result<()> {
        self.encode_targets(targets).map(|_| ())
    }
    async fn set_targets(&self, targets: [MotorTarget; DOF]) -> Result<()> {
        let _guard = self.operations.lock().await;
        anyhow::ensure!(!self.transport_failed(), "Meow transport has failed");
        anyhow::ensure!(
            self.expected_mit.read().iter().all(|enabled| *enabled),
            "all Meow axes must be enabled before updating targets"
        );
        *self.stream.write() = Some(self.encode_targets(targets)?);
        Ok(())
    }
    async fn disable_all(&self) -> Result<()> {
        let _guard = self.operations.lock().await;
        self.disable_inner().await
    }
    async fn shutdown(&self) -> Result<()> {
        let _guard = self.operations.lock().await;
        self.disable_inner().await?;
        let consumers = *self.consumers.read();
        let expected = heartbeat::encode_consumer_heartbeat_entry(
            self.profile.bus.heartbeat_node_id,
            HEARTBEAT_TIMEOUT_MS,
        );
        for (index, joint) in self.profile.joints.iter().enumerate() {
            if !consumers[index] {
                continue;
            }
            self.confirm_mode(joint.node_id, 0, true).await?;
            let actual =
                sdo::upload_u32(self.bus.as_ref(), joint.node_id, 0x1016, 1, TIMEOUT).await?;
            anyhow::ensure!(
                actual == 0 || actual == expected,
                "refusing to remove another host's heartbeat consumer"
            );
            self.write_checked(joint.node_id, &SdoWrite::u32(0x1016, 1, 0))
                .await?;
            self.consumers.write()[index] = false;
        }
        self.heartbeat_enabled.store(false, Ordering::Release);
        self.initialized.store(false, Ordering::Release);
        Ok(())
    }
    async fn clear_faults(&self) -> Result<()> {
        let _guard = self.operations.lock().await;
        anyhow::ensure!(!self.failed.load(Ordering::Acquire),
            "Meow backend failure is latched; restart the controller before explicit motor recovery");
        let identities = self.discover_inner().await?;
        anyhow::ensure!(
            identities.iter().all(|id| id.identity_verified),
            "fault recovery requires exact commissioned identities"
        );
        for joint in &self.profile.joints {
            let (mode, _) = self.read_mode(joint.node_id).await?;
            anyhow::ensure!(
                mode == 0 || matches!(mode, 0xA1..=0xA6 | 0xAF),
                "explicit fault recovery requires all axes disabled"
            );
        }
        for joint in &self.profile.joints {
            let (mode, error) = self.read_mode(joint.node_id).await?;
            if mode == 0 && error == 0 {
                continue;
            }
            sdo::download_u8(self.bus.as_ref(), joint.node_id, 0x4401, 0, 0, TIMEOUT).await?;
            tokio::time::sleep(Duration::from_millis(10)).await;
            sdo::download_u8(self.bus.as_ref(), joint.node_id, 0x4401, 0, 0xFF, TIMEOUT).await?;
            self.confirm_mode(joint.node_id, 0, false).await?;
        }
        Ok(())
    }
    fn measured_torque_limit_nm(&self, index: usize) -> Option<f32> {
        let calibration = self.calibration.read().get(index).copied().flatten()?;
        let joint = self.profile.joints.get(index)?;
        Some(
            calibration.peak_nm * f32::from(joint.torque_permille)
                / 1000.0
                / calibration.torque_factor
                / joint.torque_scale,
        )
    }

    fn feedback(&self) -> FeedbackSnapshot {
        let now = Instant::now();
        let telemetry = *self.telemetry.read();
        let calibration = *self.calibration.read();
        let oldest = |tpdo1_only: bool| {
            let mut oldest = now;
            for state in telemetry {
                oldest = oldest.min(state.tpdo1_at?);
                if !tpdo1_only {
                    oldest = oldest.min(state.tpdo2_at?);
                }
            }
            Some(oldest)
        };
        let joints = array::from_fn(|index| {
            let state = telemetry[index];
            let torque_nm = calibration[index].map_or(f32::NAN, |calibration| {
                state.torque_permille as f32 * calibration.peak_nm
                    / 1000.0
                    / calibration.torque_factor
            });
            JointFeedback {
                position_rev: state
                    .position
                    .map_or(f32::NAN, |p| p.to_revolutions() as f32),
                velocity_rev_s: state.velocity.unwrap_or(f32::NAN),
                torque_nm,
                temperature_c: state.motor_c,
                driver_temperature_c: state.driver_c,
                motor_temperature_c: state.motor_c,
                online: state.heartbeat_at.is_some_and(|stamp| {
                    now.saturating_duration_since(stamp) <= Duration::from_millis(1_500)
                }),
                fresh: state.fresh(now, self.profile.feedback_timeout()) && torque_nm.is_finite(),
                fault_code: if state.error != 0 {
                    Some(state.error)
                } else if state.mode.is_some_and(|mode| mode > 4) {
                    Some(0xffff)
                } else {
                    None
                },
            }
        });
        FeedbackSnapshot {
            joints,
            oldest_tpdo1_at: oldest(true),
            captured_at: oldest(false),
        }
    }
    fn transport_failed(&self) -> bool {
        let expected = *self.expected_mit.read();
        let telemetry = *self.telemetry.read();
        for index in 0..DOF {
            if expected[index]
                && (!telemetry[index].fresh(Instant::now(), self.profile.feedback_timeout())
                    || telemetry[index].mode != Some(4)
                    || telemetry[index].error != 0)
            {
                self.failed.store(true, Ordering::Release);
            }
        }
        self.failed.load(Ordering::Acquire)
    }
}
fn mit_recipe(node: u8) -> RpdoRecipe {
    RpdoRecipe {
        rpdo_index: 0,
        cob_id: 0x200 + u16::from(node),
        transmission_type: 255,
        entries: vec![
            TpdoEntry {
                index: 0x4102,
                subindex: 1,
                bit_len: 32,
            },
            TpdoEntry {
                index: 0x4102,
                subindex: 2,
                bit_len: 32,
            },
            TpdoEntry {
                index: 0x4102,
                subindex: 3,
                bit_len: 32,
            },
            TpdoEntry {
                index: 0x4102,
                subindex: 4,
                bit_len: 16,
            },
            TpdoEntry {
                index: 0x4102,
                subindex: 5,
                bit_len: 16,
            },
            TpdoEntry {
                index: 0x4102,
                subindex: 6,
                bit_len: 16,
            },
            TpdoEntry {
                index: 0x3000,
                subindex: 2,
                bit_len: 16,
            },
        ],
    }
}
fn encode_target(
    target: MotorTarget,
    calibration: Calibration,
    max_permille: u16,
    pd_permille: u16,
    budget: MeowTorqueBudget,
) -> Result<[u8; 20]> {
    budget.validate()?;
    anyhow::ensure!(
        [
            target.position_rev,
            target.velocity_rev_s,
            target.torque_nm,
            target.kp_nm_rev,
            target.kd_nm_s_rev
        ]
        .into_iter()
        .all(f32::is_finite),
        "MIT target must be finite"
    );
    anyhow::ensure!(
        [
            calibration.gain_factor,
            calibration.torque_factor,
            calibration.peak_nm
        ]
        .into_iter()
        .all(|v| v.is_finite() && v > f32::EPSILON),
        "invalid Meow calibration"
    );
    anyhow::ensure!(
        max_permille <= 1000 && pd_permille <= 1000,
        "invalid Meow torque cap"
    );
    let gain = |value: f32| -> Result<u16> {
        let raw = (f64::from(value) / f64::from(calibration.gain_factor)).round();
        anyhow::ensure!(
            value >= 0.0 && (0.0..=f64::from(u16::MAX)).contains(&raw),
            "MIT gain exceeds u16 firmware range"
        );
        Ok(raw as u16)
    };
    let wire_torque = target.torque_nm * calibration.torque_factor;
    let cap_nm = calibration.peak_nm * f32::from(max_permille) / 1000.0;
    anyhow::ensure!(wire_torque.abs() <= cap_nm + 1.0e-5,
        "Meow physical Tff {} Nm requires {wire_torque} raw Nm, above configured {cap_nm} raw Nm ceiling", target.torque_nm);
    // Allocate in calibrated motor-side units. Existing profiles retain fixed
    // 15% headroom; commissioned profiles may share the remainder with PD.
    let feedforward_budget = (f64::from(wire_torque.abs())
        * (1.0 + budget.feedforward_reserve_ratio)
        / f64::from(calibration.peak_nm)
        * 1000.0)
        .ceil();
    let allocated_pd = match budget.pd_allocation {
        MeowPdAllocation::Fixed => {
            anyhow::ensure!(feedforward_budget + f64::from(pd_permille) <= f64::from(max_permille),
                "Meow torque ceiling lacks PD/gravity headroom: need {:.0} permille (Tff with {:.1}% reserve + PD), configured {max_permille}",
                feedforward_budget + f64::from(pd_permille), budget.feedforward_reserve_ratio * 100.0);
            pd_permille
        }
        MeowPdAllocation::Remaining => {
            anyhow::ensure!(feedforward_budget <= f64::from(max_permille),
                "Meow feed-forward budget needs {feedforward_budget} permille, above configured {max_permille}");
            pd_permille.min(max_permille - feedforward_budget as u16)
        }
    };
    let mut payload = [0; 20];
    payload[0..4].copy_from_slice(&target.position_rev.to_le_bytes());
    payload[4..8].copy_from_slice(&target.velocity_rev_s.to_le_bytes());
    payload[8..12].copy_from_slice(&wire_torque.to_le_bytes());
    payload[12..14].copy_from_slice(&gain(target.kp_nm_rev)?.to_le_bytes());
    payload[14..16].copy_from_slice(&gain(target.kd_nm_s_rev)?.to_le_bytes());
    payload[16..18].copy_from_slice(&allocated_pd.to_le_bytes());
    Ok(payload)
}
async fn send_payloads(
    bus: &dyn CanBus,
    profile: &HardwareProfile,
    payloads: &[[u8; 20]; DOF],
) -> Result<()> {
    for (joint, payload) in profile.joints.iter().zip(payloads) {
        let frame = CanFrame::new_fd(0x200 + u16::from(joint.node_id), payload, true)?;
        tokio::time::timeout(Duration::from_millis(10), bus.send(frame)).await??;
    }
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn canonical_mit_keeps_factory_and_gain_scales_distinct() {
        let target = MotorTarget {
            position_rev: 0.125,
            velocity_rev_s: -0.2,
            torque_nm: 2.0,
            kp_nm_rev: 40.0,
            kd_nm_s_rev: 0.6,
        };
        let payload = encode_target(
            target,
            Calibration {
                peak_nm: 30.0,
                torque_factor: 3.0,
                gain_factor: 0.01,
            },
            1000,
            500,
            MeowTorqueBudget::default(),
        )
        .unwrap();
        assert_eq!(f32::from_le_bytes(payload[8..12].try_into().unwrap()), 6.0);
        assert_eq!(
            u16::from_le_bytes(payload[12..14].try_into().unwrap()),
            4000
        );
        assert_eq!(u16::from_le_bytes(payload[14..16].try_into().unwrap()), 60);
        assert_eq!(u16::from_le_bytes(payload[16..18].try_into().unwrap()), 500);
        assert_eq!(&payload[18..20], &[0, 0]);
        let recipe = mit_recipe(3);
        assert_eq!(recipe.cob_id, 0x203);
        assert_eq!(
            recipe
                .entries
                .iter()
                .map(|e| usize::from(e.bit_len))
                .sum::<usize>(),
            160
        );
        assert!(recipe
            .entries
            .iter()
            .all(|e| e.index == 0x4102 || e.index == 0x3000));
    }
    #[test]
    fn underpowered_and_invalid_wire_targets_fail_without_silent_clamping() {
        let calibration = Calibration {
            peak_nm: 10.0,
            torque_factor: 4.0,
            gain_factor: 0.01,
        };
        let target = MotorTarget {
            torque_nm: 1.0,
            ..Default::default()
        };
        assert!(encode_target(target, calibration, 300, 300, MeowTorqueBudget::default()).is_err());
        let with_pd = MotorTarget {
            torque_nm: 0.75,
            ..Default::default()
        };
        assert!(
            encode_target(with_pd, calibration, 600, 300, MeowTorqueBudget::default()).is_err()
        );
        assert!(encode_target(with_pd, calibration, 700, 300, MeowTorqueBudget::default()).is_ok());
        assert!(encode_target(
            MotorTarget {
                kp_nm_rev: 1000.0,
                ..Default::default()
            },
            calibration,
            1000,
            1000,
            MeowTorqueBudget::default()
        )
        .is_err());
        assert!(encode_target(
            MotorTarget {
                torque_nm: f32::NAN,
                ..Default::default()
            },
            calibration,
            1000,
            1000,
            MeowTorqueBudget::default()
        )
        .is_err());
        assert!(encode_target(
            target,
            Calibration {
                torque_factor: 0.0,
                ..calibration
            },
            1000,
            1000,
            MeowTorqueBudget::default()
        )
        .is_err());
    }
    #[test]
    fn meow_feedback_uses_q8_24_and_sixteen_bit_timestamp_wrap() {
        let previous = SignedQ8_24::from_revolutions(0.15).unwrap();
        let current = SignedQ8_24::from_revolutions(0.151).unwrap();
        let dt = 500u16.wrapping_sub(65_036);
        assert_eq!(dt, 1000);
        let velocity = current.wrapping_delta(previous) * 1_000_000.0 / f64::from(dt);
        assert!((velocity - 1.0).abs() < 0.0001);
    }

    #[test]
    fn remaining_budget_preserves_tff_and_shares_the_calibrated_total_with_pd() {
        let calibration = Calibration {
            peak_nm: 10.0,
            torque_factor: 4.0,
            gain_factor: 0.01,
        };
        let budget = MeowTorqueBudget {
            feedforward_reserve_ratio: 0.0,
            pd_allocation: MeowPdAllocation::Remaining,
        };
        let allocated_pd = |torque_nm, maximum, requested| {
            let payload = encode_target(
                MotorTarget {
                    torque_nm,
                    ..Default::default()
                },
                calibration,
                maximum,
                requested,
                budget,
            )
            .unwrap();
            assert_eq!(
                f32::from_le_bytes(payload[8..12].try_into().unwrap()),
                torque_nm * 4.0
            );
            u16::from_le_bytes(payload[16..18].try_into().unwrap())
        };
        assert_eq!(allocated_pd(0.75, 1000, 1000), 700);
        assert_eq!(allocated_pd(-0.75, 1000, 1000), 700);
        assert_eq!(allocated_pd(0.75, 1000, 500), 500);
        assert_eq!(allocated_pd(0.0, 1000, 1000), 1000);
        assert_eq!(allocated_pd(0.0, 800, 1000), 800);
        assert_eq!(allocated_pd(2.5, 1000, 1000), 0);
        for torque_nm in [-2.51, 2.51, f32::NAN, f32::INFINITY] {
            assert!(encode_target(
                MotorTarget {
                    torque_nm,
                    ..Default::default()
                },
                calibration,
                1000,
                1000,
                budget
            )
            .is_err());
        }
        // Sweep both torque directions and changing drive ceilings. The wire
        // feed-forward stays exact, and the complete granted PD always fits.
        for maximum in [650, 800, 1000] {
            for raw_tenths in -60..=60 {
                let torque_nm = raw_tenths as f32 / 40.0;
                let pd = allocated_pd(torque_nm, maximum, 1000);
                let raw_torque = torque_nm * 4.0;
                assert!(
                    raw_torque.abs() + f32::from(pd) / 100.0 <= f32::from(maximum) / 100.0 + 1.0e-5
                );
            }
        }
    }

    #[test]
    fn reserve_is_configurable_without_bypassing_physical_caps_or_fixed_defaults() {
        let calibration = Calibration {
            peak_nm: 10.0,
            torque_factor: 4.0,
            gain_factor: 0.01,
        };
        let target = MotorTarget {
            torque_nm: 0.75,
            ..Default::default()
        };
        let budget = MeowTorqueBudget {
            pd_allocation: MeowPdAllocation::Remaining,
            ..Default::default()
        };
        let payload = encode_target(target, calibration, 1000, 1000, budget).unwrap();
        assert_eq!(u16::from_le_bytes(payload[16..18].try_into().unwrap()), 655);
        assert!(
            encode_target(target, calibration, 1000, 1000, MeowTorqueBudget::default()).is_err()
        );
        assert!(encode_target(
            MotorTarget {
                torque_nm: 2.5,
                ..Default::default()
            },
            calibration,
            1000,
            1000,
            budget
        )
        .is_err());
        for feedforward_reserve_ratio in [-0.1, 1.01, f64::NAN, f64::INFINITY] {
            assert!(encode_target(
                target,
                calibration,
                1000,
                1000,
                MeowTorqueBudget {
                    feedforward_reserve_ratio,
                    ..budget
                }
            )
            .is_err());
        }
        // Reproduce the field failure: 151 permille reserved for gravity plus
        // a fixed 500 PD required 651 against the former 650 drive ceiling.
        let field_calibration = Calibration {
            peak_nm: 30.0,
            torque_factor: 1.0,
            gain_factor: 0.01,
        };
        let field_target = MotorTarget {
            torque_nm: 3.92,
            ..Default::default()
        };
        let rejected = encode_target(
            field_target,
            field_calibration,
            650,
            500,
            MeowTorqueBudget::default(),
        )
        .unwrap_err();
        assert!(rejected.to_string().contains("need 651 permille"));
        let payload = encode_target(
            field_target,
            field_calibration,
            1000,
            1000,
            MeowTorqueBudget {
                feedforward_reserve_ratio: 0.0,
                ..budget
            },
        )
        .unwrap();
        assert_eq!(u16::from_le_bytes(payload[16..18].try_into().unwrap()), 869);
    }
}

#[cfg(test)]
mod lifecycle_tests {
    use super::*;
    use can_transport::{CanBusState, CanCapabilities, CanIoError, CanRx};

    struct UnavailableSdoBus;
    #[async_trait]
    impl CanBus for UnavailableSdoBus {
        async fn send(&self, _frame: CanFrame) -> std::result::Result<(), CanIoError> {
            Err(CanIoError::Disconnected)
        }
        async fn subscribe(
            &self,
            _filter: CanFilter,
        ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
            Err(CanIoError::Disconnected)
        }
        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
        async fn bus_state(&self) -> std::result::Result<Option<CanBusState>, CanIoError> {
            Err(CanIoError::Disconnected)
        }
    }

    #[tokio::test]
    async fn unconfirmed_disable_stops_heartbeat_preserves_watchdog_and_requires_restart() {
        let profile: HardwareProfile = serde_yaml::from_str(include_str!(
            "../../../../config/hardware/firefly_y6.meow_mit.example.yaml"
        ))
        .unwrap();
        let mut touched = [false; DOF];
        touched[0] = true;
        let hold = [[42u8; 20]; DOF];
        let backend = MeowBackend {
            profile: Arc::new(profile),
            bus: Arc::new(UnavailableSdoBus),
            telemetry: Arc::new(RwLock::new([Telemetry::default(); DOF])),
            calibration: RwLock::new([None; DOF]),
            identities: RwLock::new(Vec::new()),
            touched: RwLock::new(touched),
            consumers: RwLock::new(touched),
            initialized: AtomicBool::new(true),
            heartbeat_enabled: Arc::new(AtomicBool::new(true)),
            stream: Arc::new(RwLock::new(Some(hold))),
            expected_mit: Arc::new(RwLock::new([true; DOF])),
            failed: Arc::new(AtomicBool::new(false)),
            operations: Mutex::new(()),
            tasks: Vec::new(),
        };
        assert_eq!(backend.measured_torque_limit_nm(0), None);
        backend.calibration.write()[0] = Some(Calibration {
            gain_factor: 1.0,
            peak_nm: 30.0,
            torque_factor: 1.5,
        });
        assert_eq!(backend.measured_torque_limit_nm(0), Some(13.0));
        assert_eq!(backend.profile.joints[0].limits.torque_nm, 0.5);
        assert_eq!(backend.measured_torque_limit_nm(DOF), None);
        assert!(backend.disable_all().await.is_err());
        assert!(!backend.heartbeat_enabled.load(Ordering::Acquire));
        assert_eq!(*backend.consumers.read(), touched);
        assert_eq!(*backend.stream.read(), Some(hold));
        assert!(backend.failed.load(Ordering::Acquire));
        assert!(backend
            .clear_faults()
            .await
            .unwrap_err()
            .to_string()
            .contains("restart"));
        assert!(backend.shutdown().await.is_err());
        assert_eq!(*backend.consumers.read(), touched);
    }
}
