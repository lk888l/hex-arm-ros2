use super::*;
use crate::backend::MockBackend;
use crate::profile::{
    BusProfile, BusTransport, ControllerProfile, IdentityFingerprint, JointLimits, JointProfile,
    JOINT_NAMES,
};
use async_trait::async_trait;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

struct DampingBackend {
    inner: ShutdownOrderBackend,
    state: RwLock<FeedbackSnapshot>,
    last: RwLock<Option<[crate::conversion::MotorTarget; DOF]>>,
    fall_to_fold: bool,
    fail_disable: AtomicBool,
}

impl DampingBackend {
    fn new(fall_to_fold: bool) -> Self {
        let inner = ShutdownOrderBackend::new();
        let mut state = inner.feedback.read().clone();
        for (f, q) in state
            .joints
            .iter_mut()
            .zip(crate::startup_recipe::RECIPE.ready())
        {
            f.position_rev = q / std::f32::consts::TAU;
        }
        Self {
            inner,
            state: RwLock::new(state),
            last: RwLock::new(None),
            fall_to_fold,
            fail_disable: AtomicBool::new(false),
        }
    }
}

#[async_trait]
impl MotorBackend for DampingBackend {
    async fn discover(&self, refresh: bool) -> Result<Vec<MotorIdentitySnapshot>> {
        self.inner.discover(refresh).await
    }
    async fn initialize_disabled(&self) -> Result<()> {
        self.inner.initialize_disabled().await
    }
    async fn enable_compressed_mit(&self, t: [crate::conversion::MotorTarget; DOF]) -> Result<()> {
        self.inner.enable_compressed_mit(t).await
    }
    async fn set_targets(&self, t: [crate::conversion::MotorTarget; DOF]) -> Result<()> {
        self.inner.set_targets(t).await?;
        *self.last.write() = Some(t);
        // Model a supported folded resting position only once fully unloaded.
        if self.fall_to_fold && t.iter().all(|t| t.kp_nm_rev == 0.0) {
            for (f, q) in self
                .state
                .write()
                .joints
                .iter_mut()
                .zip(crate::startup_recipe::RECIPE.folded_position_rad)
            {
                f.position_rev = q / std::f32::consts::TAU;
            }
        }
        Ok(())
    }
    async fn disable_all(&self) -> Result<()> {
        anyhow::ensure!(
            !self.fail_disable.load(Ordering::Acquire),
            "mock disable unconfirmed"
        );
        self.inner.disable_all().await
    }
    async fn shutdown(&self) -> Result<()> {
        self.inner.shutdown().await
    }
    async fn clear_faults(&self) -> Result<()> {
        self.inner.clear_faults().await
    }
    fn feedback(&self) -> FeedbackSnapshot {
        self.state.read().clone()
    }
    fn transport_failed(&self) -> bool {
        false
    }
}

async fn damping_runtime(fall: bool) -> (Arc<ArmRuntime>, Arc<DampingBackend>, u32) {
    let backend = Arc::new(DampingBackend::new(fall));
    let mut runtime = runtime_with_backend(backend.clone());
    let p = Arc::get_mut(&mut runtime.profile).unwrap();
    p.bus.protocol = crate::profile::MotorProtocol::Meow;
    p.controller.shutdown_damping = Some(crate::profile::ShutdownDamping {
        kd_nm_s_rad: [1.0; DOF],
        unload_sec: 1.0,
        timeout_sec: 2.5,
        settle_sec: 0.5,
    });
    runtime.initialize().await.unwrap();
    let id = runtime.acquire("damping-test".into()).unwrap().0;
    runtime.set_mode(id, OperatingMode::Active).await.unwrap();
    *backend.last.write() = None;
    (Arc::new(runtime), backend, id)
}

#[tokio::test]
async fn damping_requires_ready_and_stationary_before_taking_ownership() {
    let (rt, b, id) = damping_runtime(true).await;
    b.state.write().joints[2].position_rev += 0.02;
    assert!(rt
        .damped_stop(id)
        .await
        .unwrap_err()
        .to_string()
        .contains("startup_ready"));
    assert!(b.last.read().is_none());
    b.state.write().joints[2].position_rev -= 0.02;
    b.state.write().joints[0].velocity_rev_s = 0.01;
    assert!(rt
        .damped_stop(id)
        .await
        .unwrap_err()
        .to_string()
        .contains("stationary"));
    assert!(b.last.read().is_none());
    rt.shutdown().await.unwrap();
}

#[tokio::test]
async fn damping_unloads_settles_then_disables_and_never_reopens_commands() {
    let (rt, b, id) = damping_runtime(true).await;
    let worker = tokio::spawn(rt.clone().run_control_loop());
    rt.damped_stop(id).await.unwrap();
    let final_targets = b.last.read().unwrap();
    assert!(final_targets.iter().all(|t| t.kp_nm_rev == 0.0
        && t.torque_nm == 0.0
        && t.kd_nm_s_rev > 0.0
        && t.velocity_rev_s == 0.0));
    assert!(!b.inner.enabled.load(Ordering::Acquire));
    assert!(rt.set_mode(id, OperatingMode::Active).await.is_err());
    rt.shutdown().await.unwrap();
    worker.await.unwrap();
    assert!(!b.inner.targets_during_disable.load(Ordering::Acquire));
}

#[tokio::test]
async fn damping_timeout_is_not_success_even_if_motionless() {
    let (rt, b, id) = damping_runtime(false).await;
    let error = rt.damped_stop(id).await.unwrap_err();
    assert!(error.to_string().contains("timed out"));
    assert!(!b.inner.enabled.load(Ordering::Acquire));
    assert_eq!(rt.data.read().safety.mode, OperatingMode::Fault);
}

#[tokio::test]
async fn damping_unconfirmed_disable_latches_fault_and_retries_without_rpc_owner() {
    let (rt, b, id) = damping_runtime(true).await;
    b.fail_disable.store(true, Ordering::Release);
    assert!(rt
        .damped_stop(id)
        .await
        .unwrap_err()
        .to_string()
        .contains("disable unconfirmed"));
    assert_eq!(rt.data.read().safety.mode, OperatingMode::Fault);
    assert!(rt.data.read().disable_pending);
    assert!(b.inner.enabled.load(Ordering::Acquire));
    b.fail_disable.store(false, Ordering::Release);
    let worker = tokio::spawn(rt.clone().run_control_loop());
    tokio::time::timeout(Duration::from_secs(1), async {
        while rt.data.read().disable_pending {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .unwrap();
    assert!(!b.inner.enabled.load(Ordering::Acquire));
    assert!(rt.acquire("late".into()).is_err());
    rt.shutdown().await.unwrap();
    worker.await.unwrap();
}

#[tokio::test]
async fn damping_feedback_failure_and_process_shutdown_interrupt_unloading() {
    for failure in ["signal", "stale", "motor_fault", "overspeed"] {
        let (rt, b, id) = damping_runtime(true).await;
        let task = tokio::spawn({
            let rt = rt.clone();
            async move { rt.damped_stop(id).await }
        });
        while b.last.read().is_none() {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        match failure {
            "signal" => {
                rt.begin_shutdown();
            }
            "stale" => b.state.write().joints[0].fresh = false,
            "motor_fault" => b.state.write().joints[0].fault_code = Some(1),
            "overspeed" => b.state.write().joints[0].velocity_rev_s = 10.0,
            _ => unreachable!(),
        }
        assert!(tokio::time::timeout(Duration::from_millis(100), task)
            .await
            .unwrap()
            .unwrap()
            .is_err());
        assert!(!b.inner.enabled.load(Ordering::Acquire));
        rt.shutdown().await.unwrap();
    }
}

struct ShutdownOrderBackend {
    feedback: RwLock<FeedbackSnapshot>,
    enabled: AtomicBool,
    enable_calls: AtomicUsize,
    shutdown_calls: AtomicUsize,
    shutdown_complete: AtomicBool,
    enable_after_shutdown: AtomicBool,
    reject_nonzero_feedforward: AtomicBool,
    disable_delay_ms: AtomicUsize,
    disable_in_progress: AtomicBool,
    targets_during_disable: AtomicBool,
    discovery_calls: AtomicUsize,
    refresh_delay_ms: AtomicUsize,
    discovery_in_progress: AtomicBool,
    target_calls: AtomicUsize,
    validation_calls: AtomicUsize,
    sent_position_rev: RwLock<Vec<f32>>,
    pause_enable: AtomicBool,
    enable_entered: tokio::sync::Notify,
    resume_enable: tokio::sync::Notify,
    fail_disable: AtomicBool,
}

impl ShutdownOrderBackend {
    fn new() -> Self {
        let mut feedback = FeedbackSnapshot::default();
        for joint in &mut feedback.joints {
            joint.online = true;
            joint.fresh = true;
        }
        feedback.captured_at = Some(Instant::now());
        Self {
            feedback: RwLock::new(feedback),
            enabled: AtomicBool::new(false),
            enable_calls: AtomicUsize::new(0),
            shutdown_calls: AtomicUsize::new(0),
            shutdown_complete: AtomicBool::new(false),
            enable_after_shutdown: AtomicBool::new(false),
            reject_nonzero_feedforward: AtomicBool::new(false),
            disable_delay_ms: AtomicUsize::new(0),
            disable_in_progress: AtomicBool::new(false),
            targets_during_disable: AtomicBool::new(false),
            discovery_calls: AtomicUsize::new(0),
            refresh_delay_ms: AtomicUsize::new(0),
            discovery_in_progress: AtomicBool::new(false),
            target_calls: AtomicUsize::new(0),
            validation_calls: AtomicUsize::new(0),
            sent_position_rev: RwLock::new(Vec::new()),
            pause_enable: AtomicBool::new(false),
            enable_entered: tokio::sync::Notify::new(),
            resume_enable: tokio::sync::Notify::new(),
            fail_disable: AtomicBool::new(false),
        }
    }
}

#[async_trait]
impl MotorBackend for ShutdownOrderBackend {
    fn validate_targets(&self, targets: [crate::conversion::MotorTarget; DOF]) -> Result<()> {
        self.validation_calls.fetch_add(1, Ordering::AcqRel);
        anyhow::ensure!(
            !self.reject_nonzero_feedforward.load(Ordering::Acquire)
                || targets.iter().all(|target| target.torque_nm == 0.0),
            "test motor has insufficient PD/gravity headroom"
        );
        Ok(())
    }

    async fn discover(&self, refresh: bool) -> Result<Vec<MotorIdentitySnapshot>> {
        self.discovery_calls.fetch_add(1, Ordering::AcqRel);
        if refresh {
            self.discovery_in_progress.store(true, Ordering::Release);
            tokio::time::sleep(Duration::from_millis(
                self.refresh_delay_ms.load(Ordering::Acquire) as u64,
            ))
            .await;
            self.discovery_in_progress.store(false, Ordering::Release);
        }
        Ok(vec![MotorIdentitySnapshot {
            node_id: 1,
            vendor_id: 1,
            product_code: 1,
            revision: 1,
            serial_number: 1,
            model: "test".into(),
            identity_verified: true,
        }])
    }

    async fn initialize_disabled(&self) -> Result<()> {
        self.enabled.store(false, Ordering::Release);
        Ok(())
    }

    async fn enable_compressed_mit(
        &self,
        _initial_targets: [crate::conversion::MotorTarget; DOF],
    ) -> Result<()> {
        self.enable_calls.fetch_add(1, Ordering::AcqRel);
        if self.shutdown_complete.load(Ordering::Acquire) {
            self.enable_after_shutdown.store(true, Ordering::Release);
        }
        if self.pause_enable.load(Ordering::Acquire) {
            self.enable_entered.notify_one();
            self.resume_enable.notified().await;
        }
        self.enabled.store(true, Ordering::Release);
        Ok(())
    }

    async fn set_targets(&self, targets: [crate::conversion::MotorTarget; DOF]) -> Result<()> {
        self.sent_position_rev.write().push(targets[0].position_rev);
        self.target_calls.fetch_add(1, Ordering::AcqRel);
        if self.disable_in_progress.load(Ordering::Acquire) {
            self.targets_during_disable.store(true, Ordering::Release);
            anyhow::bail!("test drive is being disabled");
        }
        Ok(())
    }

    async fn disable_all(&self) -> Result<()> {
        anyhow::ensure!(
            !self.fail_disable.load(Ordering::Acquire),
            "mock disable unconfirmed"
        );
        self.disable_in_progress.store(true, Ordering::Release);
        self.enabled.store(false, Ordering::Release);
        tokio::time::sleep(Duration::from_millis(
            self.disable_delay_ms.load(Ordering::Acquire) as u64,
        ))
        .await;
        self.disable_in_progress.store(false, Ordering::Release);
        Ok(())
    }

    async fn shutdown(&self) -> Result<()> {
        self.shutdown_calls.fetch_add(1, Ordering::AcqRel);
        self.enabled.store(false, Ordering::Release);
        self.shutdown_complete.store(true, Ordering::Release);
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

fn runtime_and_backend_for_safety_test() -> (ArmRuntime, Arc<MockBackend>) {
    let backend = Arc::new(MockBackend::new());
    (
        runtime_with_backend(backend.clone() as Arc<dyn MotorBackend>),
        backend,
    )
}

fn runtime_with_backend(backend: Arc<dyn MotorBackend>) -> ArmRuntime {
    let profile = Arc::new(HardwareProfile {
        schema_version: crate::profile::HARDWARE_PROFILE_SCHEMA_VERSION,
        joint_coordinate_version: crate::profile::JOINT_COORDINATE_VERSION,
        validated: true,
        calibrated: true,
        robot_prefix: "hexmeow/test/arm0".into(),
        urdf_path: "unused-by-this-test".into(),
        gravity_vector_base_m_s2: [0.0, 0.0, -9.81],
        tip_payload: None,
        bus: BusProfile {
            protocol: crate::profile::MotorProtocol::Cia402,
            transport: BusTransport::GsUsb,
            interface: String::new(),
            channel: 0,
            adapter_vid: 1,
            adapter_pid: 2,
            heartbeat_node_id: 16,
            hardware_timestamp: false,
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
            max_measured_temperature_c: None,
            gravity_startup_slew_rate_nm_s: None,
            hand_guiding_velocity_limits_rad_s: None,
            hand_guiding_position_margin_rad: None,
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
                motion_feedforward: None,
                torque_permille: 100,
                kp_kd_torque_permille: 100,
                meow_torque_budget: Default::default(),
                limits: JointLimits {
                    position_lower_rad: -2.0,
                    position_upper_rad: 2.0,
                    measured_position_margin_rad: 0.0,
                    measured_velocity_margin_rad_s: 0.0,
                    velocity_rad_s: 0.2,
                    acceleration_rad_s2: 0.1,
                    torque_nm: 1.0,
                },
                default_kp: 5.0,
                default_kd: 0.8,
            })
            .collect(),
    });
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let dynamics = ArmDynamics::from_parts(
        vec![([0.0; 3], identity, [0.0, 0.0, 1.0]); DOF],
        vec![(0.0, [0.0; 3]); DOF],
        [0.0, 0.0, -9.81],
    );
    ArmRuntime::new(profile, backend, dynamics)
}

fn runtime_for_safety_test() -> ArmRuntime {
    runtime_and_backend_for_safety_test().0
}

fn targets(positions: [f32; DOF]) -> Vec<RosTarget> {
    positions
        .into_iter()
        .map(|position_rad| RosTarget {
            position_rad,
            ..Default::default()
        })
        .collect()
}

fn streaming_command(session_id: u32, tau_ff: Vec<f32>) -> pb::JointTrajectory {
    pb::JointTrajectory {
        header: None,
        session_id,
        points: vec![pb::JointSetpoint {
            q: vec![0.0; DOF],
            dq: vec![],
            kp: vec![],
            kd: vec![],
            tau_ff,
        }],
        t_from_start_ns: vec![10_000_000],
        on_timeout: pb::TimeoutBehavior::Fault as i32,
    }
}

fn feedback_from_ros(
    runtime: &ArmRuntime,
    positions_rad: [f32; DOF],
    velocities_rad_s: [f32; DOF],
) -> FeedbackSnapshot {
    let mut feedback = FeedbackSnapshot::default();
    for index in 0..DOF {
        let motor = ros_target_to_motor(
            RosTarget {
                position_rad: positions_rad[index],
                velocity_rad_s: velocities_rad_s[index],
                ..Default::default()
            },
            &runtime.profile.joints[index],
        );
        feedback.joints[index].position_rev = motor.position_rev;
        feedback.joints[index].velocity_rev_s = motor.velocity_rev_s;
        feedback.joints[index].online = true;
        feedback.joints[index].fresh = true;
    }
    feedback.captured_at = Some(Instant::now());
    feedback
}

#[tokio::test]
async fn intentional_disable_excludes_streaming_without_latching_transport_fault() {
    let backend = Arc::new(ShutdownOrderBackend::new());
    backend.disable_delay_ms.store(30, Ordering::Release);
    let runtime = Arc::new(runtime_with_backend(
        backend.clone() as Arc<dyn MotorBackend>
    ));
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("disable-stream-race".into()).unwrap();
    runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap();
    let task = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::sleep(Duration::from_millis(5)).await;
    runtime
        .set_mode(session, OperatingMode::Disabled)
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(5)).await;
    assert!(!backend.targets_during_disable.load(Ordering::Acquire));
    assert_eq!(runtime.data.read().safety.mode, OperatingMode::Disabled);
    runtime.begin_shutdown();
    task.await.unwrap();
    runtime.shutdown().await.unwrap();
}

#[tokio::test]
async fn shutdown_latch_rejects_a_pending_active_transition_before_final_disarm() {
    let backend = Arc::new(ShutdownOrderBackend::new());
    let runtime = Arc::new(runtime_with_backend(
        backend.clone() as Arc<dyn MotorBackend>
    ));
    runtime.initialize().await.unwrap();
    let (session_id, _, _) = runtime.acquire("shutdown-race-test".into()).unwrap();

    // Keep set_mode pending at the same serialization point used by final
    // backend shutdown.  Closing is latched while both operations wait.
    let held_gate = runtime.mode_gate.lock().await;
    let pending_runtime = runtime.clone();
    let mode_task = tokio::spawn(async move {
        pending_runtime
            .set_mode(session_id, OperatingMode::Active)
            .await
    });
    tokio::task::yield_now().await;

    assert!(runtime.begin_shutdown());
    let shutdown_runtime = runtime.clone();
    let shutdown_task = tokio::spawn(async move { shutdown_runtime.shutdown().await });
    drop(held_gate);

    let mode_error = mode_task.await.unwrap().unwrap_err();
    assert!(mode_error.to_string().contains("shutting down"));
    shutdown_task.await.unwrap().unwrap();

    assert_eq!(backend.enable_calls.load(Ordering::Acquire), 0);
    assert_eq!(backend.shutdown_calls.load(Ordering::Acquire), 1);
    assert!(!backend.enabled.load(Ordering::Acquire));
    assert!(!backend.enable_after_shutdown.load(Ordering::Acquire));
}

#[tokio::test]
async fn closing_rejects_all_mutating_api_and_control_loop_is_joinable() {
    let (runtime, _) = runtime_and_backend_for_safety_test();
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let (session_id, _, _) = runtime.acquire("closing-api-test".into()).unwrap();
    let loop_task = tokio::spawn(runtime.clone().run_control_loop());

    assert!(runtime.begin_shutdown());
    assert!(!runtime.begin_shutdown(), "shutdown latch must be one-way");
    tokio::time::timeout(Duration::from_millis(100), loop_task)
        .await
        .expect("control loop ignored shutdown cancellation")
        .expect("control loop task panicked");

    let assert_closing = |error: anyhow::Error| {
        assert!(
            error.to_string().contains("shutting down"),
            "unexpected API rejection: {error:#}"
        );
    };
    assert_closing(runtime.acquire("late-client".into()).unwrap_err());
    assert_closing(
        runtime
            .set_mode(session_id, OperatingMode::Active)
            .await
            .unwrap_err(),
    );
    assert_closing(
        runtime
            .submit_trajectory(streaming_command(session_id, vec![]))
            .unwrap_err(),
    );
    assert_closing(runtime.clear_fault(session_id).await.unwrap_err());
    assert_closing(runtime.release(session_id).await.unwrap_err());
    assert_closing(runtime.discover(true).await.unwrap_err());
    assert_closing(
        runtime
            .set_gravity(session_id, [0.0, 0.0, -9.81])
            .unwrap_err(),
    );

    runtime.shutdown().await.unwrap();
    let state = runtime.driver_state_proto();
    assert!(!state.session_owned);
    assert!(!state.profile_valid);
    assert_eq!(runtime.mode(), OperatingMode::Disabled);
}

#[test]
fn measured_feedback_accepts_configured_position_and_velocity_limits() {
    let runtime = runtime_for_safety_test();
    let feedback = feedback_from_ros(
        &runtime,
        [-2.0, 2.0, -2.0, 2.0, -2.0, 2.0],
        [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
    );

    assert!(runtime.measured_feedback_fault(&feedback).is_none());
}

#[test]
fn measured_position_outside_limit_reports_joint_and_position_fault_code() {
    let runtime = runtime_for_safety_test();
    let mut positions = [0.0; DOF];
    positions[1] = 2.01;
    let feedback = feedback_from_ros(&runtime, positions, [0.0; DOF]);

    let (code, reason) = runtime.measured_feedback_fault(&feedback).unwrap();
    assert_eq!(code, FAULT_MEASURED_POSITION_LIMIT);
    assert!(reason.contains("joint_2 measured position"));
    assert!(reason.contains("feedback envelope [-2.000000, 2.000000]"));
    assert!(reason.contains("command limits remain [-2.000000, 2.000000]"));
}

#[test]
fn measured_position_margin_never_widens_command_validation() {
    let mut runtime = runtime_for_safety_test();
    Arc::make_mut(&mut runtime.profile).joints[1]
        .limits
        .measured_position_margin_rad = 0.001;

    let mut positions = [0.0; DOF];
    positions[1] = 2.0008;
    let feedback = feedback_from_ros(&runtime, positions, [0.0; DOF]);
    assert!(runtime.measured_feedback_fault(&feedback).is_none());

    positions[1] = 2.0012;
    let feedback = feedback_from_ros(&runtime, positions, [0.0; DOF]);
    assert_eq!(
        runtime.measured_feedback_fault(&feedback).unwrap().0,
        FAULT_MEASURED_POSITION_LIMIT
    );

    let mut commands = targets([0.0; DOF]);
    commands[1].position_rad = 2.0001;
    let error = runtime.validate_targets(&commands).unwrap_err().to_string();
    assert!(error.contains("joint_2 position exceeds software limit"));
}

#[test]
fn measured_velocity_margin_never_widens_command_validation() {
    let mut runtime = runtime_for_safety_test();
    Arc::make_mut(&mut runtime.profile).joints[3]
        .limits
        .measured_velocity_margin_rad_s = 0.02;
    let mut velocity = [0.0; DOF];
    velocity[3] = 0.21;
    assert!(runtime
        .measured_feedback_fault(&feedback_from_ros(&runtime, [0.0; DOF], velocity))
        .is_none());
    velocity[3] = 0.221;
    assert_eq!(
        runtime
            .measured_feedback_fault(&feedback_from_ros(&runtime, [0.0; DOF], velocity))
            .unwrap()
            .0,
        FAULT_MEASURED_OVERSPEED
    );
    let mut commands = targets([0.0; DOF]);
    commands[3].velocity_rad_s = 0.21;
    assert!(runtime.validate_targets(&commands).is_err());
}

#[test]
fn measured_overspeed_reports_joint_and_velocity_fault_code() {
    let runtime = runtime_for_safety_test();
    let mut velocities = [0.0; DOF];
    velocities[4] = -0.21;
    let feedback = feedback_from_ros(&runtime, [0.0; DOF], velocities);

    let (code, reason) = runtime.measured_feedback_fault(&feedback).unwrap();
    assert_eq!(code, FAULT_MEASURED_OVERSPEED);
    assert!(reason.contains("joint_5 measured velocity"));
    assert!(reason.contains("exceeds 0.200000 rad/s"));
}

#[test]
fn non_finite_measured_state_cannot_bypass_limits() {
    let runtime = runtime_for_safety_test();
    let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
    feedback.joints[0].position_rev = f32::NAN;
    assert_eq!(
        runtime.measured_feedback_fault(&feedback).unwrap().0,
        FAULT_MEASURED_POSITION_LIMIT
    );

    feedback.joints[0].position_rev = 0.0;
    feedback.joints[0].velocity_rev_s = f32::INFINITY;
    assert_eq!(
        runtime.measured_feedback_fault(&feedback).unwrap().0,
        FAULT_MEASURED_OVERSPEED
    );
}

#[test]
fn measured_torque_limit_uses_joint_units_and_checks_both_signs() {
    let mut runtime = runtime_for_safety_test();
    let joint = &mut Arc::make_mut(&mut runtime.profile).joints[2];
    joint.direction = -1;
    joint.torque_scale = 0.85;
    joint.limits.torque_nm = 7.5;
    let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
    for sign in [-1.0, 1.0] {
        feedback.joints[2].torque_nm = sign * 7.5 * 0.85;
        assert!(runtime.measured_feedback_fault(&feedback).is_none());
        feedback.joints[2].torque_nm = sign * 7.51 * 0.85;
        assert_eq!(
            runtime.measured_feedback_fault(&feedback).unwrap().0,
            FAULT_MEASURED_TORQUE_LIMIT
        );
    }
    feedback.joints[2].torque_nm = f32::NAN;
    assert_eq!(
        runtime.measured_feedback_fault(&feedback).unwrap().0,
        FAULT_MEASURED_TORQUE_LIMIT
    );
}

#[test]
fn measured_temperature_checks_motor_and_driver_independently() {
    let runtime = runtime_for_safety_test();
    for field in 0..3 {
        for value in [70.1, f32::NAN, f32::INFINITY] {
            let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
            let state = &mut feedback.joints[3];
            match field {
                0 => state.temperature_c = value,
                1 => state.motor_temperature_c = value,
                _ => state.driver_temperature_c = value,
            }
            assert_eq!(
                runtime.measured_feedback_fault(&feedback).unwrap().0,
                FAULT_MEASURED_TEMPERATURE
            );
        }
    }
}

#[test]
fn arm_temperature_override_preserves_finite_feedback_checks() {
    let mut runtime = runtime_for_safety_test();
    Arc::make_mut(&mut runtime.profile)
        .controller
        .max_measured_temperature_c = Some(85.0);
    for field in 0..3 {
        for value in [80.0, 85.0, 85.1, f32::NAN] {
            let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
            let state = &mut feedback.joints[3];
            match field {
                0 => state.temperature_c = value,
                1 => state.motor_temperature_c = value,
                _ => state.driver_temperature_c = value,
            }
            let fault = runtime.measured_feedback_fault(&feedback);
            if value.is_finite() && value <= 85.0 {
                assert!(fault.is_none());
            } else {
                assert_eq!(fault.unwrap().0, FAULT_MEASURED_TEMPERATURE);
            }
        }
    }
}

#[tokio::test]
async fn hand_guiding_accepts_position_margin_at_startup() {
    for margin in [None, Some(0.012)] {
        let backend = Arc::new(DampingBackend::new(false));
        let mut runtime = runtime_with_backend(backend.clone());
        let profile = Arc::make_mut(&mut runtime.profile);
        profile.controller.hand_guiding_position_margin_rad = margin;
        profile.joints[2].limits.position_lower_rad = -1.57;
        profile.joints[2].limits.position_upper_rad = 1.57;
        profile.joints[2].limits.measured_position_margin_rad = 0.01;
        let measured = if margin.is_some() { 1.581 } else { 1.570901 };
        backend.state.write().joints[2].position_rev = measured / std::f32::consts::TAU;
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("hand-guiding-boundary".into()).unwrap();
        assert!(runtime
            .set_mode(session, OperatingMode::Active)
            .await
            .is_err());
        assert_eq!(backend.inner.enable_calls.load(Ordering::Acquire), 0);
        runtime
            .start_gravity_comp(session, &[0.5; DOF])
            .await
            .unwrap();
        let sent = backend.last.read().unwrap()[2];
        assert!(
            (motor_position_to_ros(sent.position_rev, &runtime.profile.joints[2]) - 1.57).abs()
                < 1.0e-6
        );
        assert!(sent.kp_nm_rev == 0.0 && sent.velocity_rev_s == 0.0);
        assert!((sent.position_rev - backend.state.read().joints[2].position_rev).abs() <= 0.002);
        assert!((runtime.joint_state_proto().q[2] - measured).abs() < 1.0e-6);
        runtime.shutdown().await.unwrap();
    }
}

#[test]
fn hand_guiding_position_margin_does_not_expand_other_modes_or_commands() {
    let mut runtime = runtime_for_safety_test();
    let profile = Arc::make_mut(&mut runtime.profile);
    profile.controller.hand_guiding_position_margin_rad = Some(0.012);
    for joint in &mut profile.joints {
        joint.limits.measured_position_margin_rad = 0.01;
    }
    for index in 0..DOF {
        for sign in [-1.0, 1.0] {
            let mut position = [0.0; DOF];
            position[index] = sign * 2.011;
            let feedback = feedback_from_ros(&runtime, position, [0.0; DOF]);
            assert!(runtime
                .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                .is_none());
            for mode in [
                OperatingMode::Active,
                OperatingMode::Disabled,
                OperatingMode::Passive,
            ] {
                assert_eq!(
                    runtime
                        .measured_feedback_fault_in_mode(&feedback, mode)
                        .unwrap()
                        .0,
                    FAULT_MEASURED_POSITION_LIMIT
                );
            }
            let targets = runtime.gravity_comp_targets(&feedback, [0.5; DOF]);
            runtime.validate_targets(&targets).unwrap();
            assert_eq!(targets[index].position_rad, sign * 2.0);
            assert!(runtime
                .validate_targets(&runtime.hold_targets(&feedback))
                .is_err());
            let mut external = targets.clone();
            external[index].position_rad = position[index];
            assert!(runtime.validate_targets(&external).is_err());

            position[index] = sign * 2.013;
            let feedback = feedback_from_ros(&runtime, position, [0.0; DOF]);
            assert_eq!(
                runtime
                    .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                    .unwrap()
                    .0,
                FAULT_MEASURED_POSITION_LIMIT
            );
            assert!(runtime
                .validate_targets(&runtime.gravity_comp_targets(&feedback, [0.5; DOF]))
                .is_err());
        }
    }
    for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
        feedback.joints[2].position_rev = invalid;
        assert_eq!(
            runtime
                .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                .unwrap()
                .0,
            FAULT_MEASURED_POSITION_LIMIT
        );
        assert!(runtime
            .validate_targets(&runtime.gravity_comp_targets(&feedback, [0.5; DOF]))
            .is_err());
    }
}

#[test]
fn hand_guiding_boundary_target_keeps_actual_pose_for_gravity() {
    let (mut runtime, _) = runtime_with_startup_gravity();
    let profile = Arc::make_mut(&mut runtime.profile);
    profile.controller.hand_guiding_position_margin_rad = Some(0.012);
    profile.joints[0].limits.position_upper_rad = 1.57;
    let feedback = feedback_from_ros(&runtime, [1.578, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; DOF]);
    let actual_q = runtime.ros_joint_state(&feedback).0;
    let targets = runtime.gravity_comp_targets(&feedback, [0.5; DOF]);
    assert_eq!(targets[0].position_rad, 1.57);
    let gravity = runtime.data.read().gravity;
    let actual_torque = runtime.dynamics.gravity_torque_with(&actual_q, gravity)[0];
    let mut clamped_q = actual_q.clone();
    clamped_q[0] = 1.57;
    let clamped_torque = runtime.dynamics.gravity_torque_with(&clamped_q, gravity)[0];
    let joint = &runtime.profile.joints[0];
    let expected =
        joint.clamp_gravity_feedforward(actual_torque * joint.gravity_compensation_scale);
    assert!((targets[0].torque_nm - expected).abs() < 1.0e-6);
    assert!((actual_torque - clamped_torque).abs() > 1.0e-4);
    runtime.data.write().feedback = feedback;
    assert_eq!(runtime.joint_state_proto().q[0], actual_q[0]);
}

#[tokio::test]
async fn hand_guiding_configured_speed_is_used_by_the_control_loop() {
    let backend = Arc::new(DampingBackend::new(false));
    let mut runtime = runtime_with_backend(backend.clone());
    Arc::make_mut(&mut runtime.profile)
        .controller
        .hand_guiding_velocity_limits_rad_s = Some([2.0; DOF]);
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("hand-guiding-speed".into()).unwrap();
    runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .unwrap();
    // Independent feedback models hand motion without following zero
    // velocity targets, so the loop must really accept speeds above 0.2.
    for state in &mut backend.state.write().joints {
        state.velocity_rev_s = 0.8 / std::f32::consts::TAU;
    }
    *backend.last.write() = None;
    let loop_task = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::timeout(Duration::from_millis(500), async {
        while backend.last.read().is_none() {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .expect("hand guiding did not output at the relaxed speed");
    assert_eq!(runtime.mode(), OperatingMode::GravityComp);
    assert!(backend.inner.enabled.load(Ordering::Acquire));
    assert!(backend
        .last
        .read()
        .unwrap()
        .iter()
        .all(|t| t.kp_nm_rev == 0.0 && t.velocity_rev_s == 0.0));
    runtime.gravity_comp_heartbeat(session, 1).unwrap();

    backend.state.write().joints[1].velocity_rev_s = 2.01 / std::f32::consts::TAU;
    tokio::time::timeout(Duration::from_millis(500), async {
        while runtime.mode() != OperatingMode::Fault
            || backend.inner.enabled.load(Ordering::Acquire)
        {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .expect("configured hand-guiding overspeed must disable");
    let error = runtime
        .gravity_comp_heartbeat(session, 2)
        .unwrap_err()
        .to_string();
    assert!(error.contains("joint_2 measured velocity"));
    assert!(error.contains("exceeds 2.000000 rad/s"));
    runtime.shutdown().await.unwrap();
    loop_task.await.unwrap();
}

#[test]
fn hand_guiding_speed_limits_do_not_change_other_modes_or_commands() {
    let mut runtime = runtime_for_safety_test();
    let limits = [0.8, 1.0, 1.2, 1.4, 1.6, 2.0];
    Arc::make_mut(&mut runtime.profile)
        .controller
        .hand_guiding_velocity_limits_rad_s = Some(limits);
    for index in 0..DOF {
        for sign in [-1.0, 1.0] {
            let mut velocity = [0.0; DOF];
            velocity[index] = sign * limits[index];
            let feedback = feedback_from_ros(&runtime, [0.0; DOF], velocity);
            assert!(runtime
                .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                .is_none());
            for mode in [
                OperatingMode::Active,
                OperatingMode::Disabled,
                OperatingMode::Passive,
            ] {
                assert_eq!(
                    runtime
                        .measured_feedback_fault_in_mode(&feedback, mode)
                        .unwrap()
                        .0,
                    FAULT_MEASURED_OVERSPEED
                );
            }
            let mut commands = targets([0.0; DOF]);
            commands[index].velocity_rad_s = velocity[index];
            assert!(runtime.validate_targets(&commands).is_err());

            velocity[index] = sign * (limits[index] + 0.001);
            let feedback = feedback_from_ros(&runtime, [0.0; DOF], velocity);
            let (code, reason) = runtime
                .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                .unwrap();
            assert_eq!(code, FAULT_MEASURED_OVERSPEED);
            assert!(reason.contains(&runtime.profile.joints[index].name));
        }
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut velocity = [0.0; DOF];
            velocity[index] = invalid;
            let feedback = feedback_from_ros(&runtime, [0.0; DOF], velocity);
            assert_eq!(
                runtime
                    .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
                    .unwrap()
                    .0,
                FAULT_MEASURED_OVERSPEED
            );
        }
    }
    let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.8; DOF]);
    feedback.joints[0].position_rev = 3.0 / std::f32::consts::TAU;
    assert_eq!(
        runtime
            .measured_feedback_fault_in_mode(&feedback, OperatingMode::GravityComp)
            .unwrap()
            .0,
        FAULT_MEASURED_POSITION_LIMIT
    );
}

#[tokio::test]
async fn hand_guiding_rejects_moving_entry_and_retains_overspeed_protection() {
    let backend = ShutdownOrderBackend::new();
    backend.feedback.write().joints[0].velocity_rev_s = 0.03 / std::f32::consts::TAU;
    let backend = Arc::new(backend);
    let mut runtime = runtime_with_backend(backend.clone());
    Arc::make_mut(&mut runtime.profile)
        .controller
        .hand_guiding_velocity_limits_rad_s = Some([2.0; DOF]);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("moving-entry".into()).unwrap();
    assert!(runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .unwrap_err()
        .to_string()
        .contains("entry speed"));
    assert_eq!(backend.enable_calls.load(Ordering::Acquire), 0);

    let (runtime, backend) = runtime_and_backend_for_safety_test();
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("guiding-overspeed".into()).unwrap();
    runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .unwrap();
    let mut moved = backend.targets();
    moved[0].velocity_rev_s = 1.0;
    backend.set_targets(moved).await.unwrap();
    let loop_task = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::timeout(Duration::from_millis(500), async {
        while backend.is_enabled() {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .unwrap();
    assert_eq!(
        runtime.driver_state_proto().fault_code,
        FAULT_MEASURED_OVERSPEED
    );
    runtime.shutdown().await.unwrap();
    loop_task.await.unwrap();
}

#[tokio::test]
async fn hand_guiding_rejects_invalid_gains_and_nonowners_before_enable() {
    let (runtime, backend) = runtime_and_backend_for_safety_test();
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("hand-guiding-test".into()).unwrap();
    for invalid in [
        vec![],
        vec![1.0; 5],
        vec![0.0; DOF],
        vec![-1.0; DOF],
        vec![f32::NAN; DOF],
        vec![f32::INFINITY; DOF],
    ] {
        assert!(runtime.start_gravity_comp(session, &invalid).await.is_err());
        assert!(!backend.is_enabled());
    }
    assert!(runtime
        .start_gravity_comp(session + 1, &[1.0; DOF])
        .await
        .is_err());
    assert!(!backend.is_enabled());
}

#[tokio::test]
async fn hand_guiding_ramps_gravity_without_any_position_stiffness() {
    let (runtime, backend) = runtime_with_startup_gravity();
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("hand-guiding-ramp".into()).unwrap();
    runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .unwrap();
    let initial = backend.targets();
    assert!(initial
        .iter()
        .all(|t| t.kp_nm_rev == 0.0 && t.torque_nm == 0.0 && t.velocity_rev_s == 0.0));
    assert!((initial[0].kd_nm_s_rev - std::f32::consts::TAU * 0.5).abs() < 1.0e-5);
    let t0 = runtime
        .data
        .read()
        .gravity_comp
        .as_ref()
        .unwrap()
        .ramp
        .as_ref()
        .unwrap()
        .last_tick_ns;
    runtime
        .gravity_comp_tick(&backend.feedback(), t0 + 20_000_000)
        .await
        .unwrap();
    assert!((backend.targets()[0].torque_nm - 0.1).abs() < 1.0e-5);
    runtime
        .gravity_comp_tick(&backend.feedback(), t0 + 100_000_000)
        .await
        .unwrap();
    assert!((backend.targets()[0].torque_nm - 0.3).abs() < 1.0e-5);
    assert!(runtime
        .data
        .read()
        .gravity_comp
        .as_ref()
        .unwrap()
        .ramp
        .is_none());
    assert!(runtime
        .event_log_proto()
        .events
        .iter()
        .any(|e| e.code == "hand_guiding_ready"));

    // After the ramp, gravity follows the actual hand-moved pose directly.
    let feedback = feedback_from_ros(
        &runtime,
        [std::f32::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.1; DOF],
    );
    runtime
        .gravity_comp_tick(&feedback, t0 + 102_000_000)
        .await
        .unwrap();
    let targets = backend.targets();
    assert!(targets[0].torque_nm.abs() < 1.0e-5);
    assert!(targets
        .iter()
        .all(|t| t.kp_nm_rev == 0.0 && t.velocity_rev_s == 0.0));
    assert!((targets[0].position_rev - 0.25).abs() < 1.0e-5);
}

#[tokio::test]
async fn hand_guiding_validates_eventual_gravity_before_zero_ramp_enable() {
    let (template, _) = runtime_with_startup_gravity();
    let backend = Arc::new(ShutdownOrderBackend::new());
    // This backend rejects the full target even though the zero ramp frame
    // alone would fit. No enable call may escape this preflight failure.
    backend
        .reject_nonzero_feedforward
        .store(true, Ordering::Release);
    let runtime = ArmRuntime::new(template.profile, backend.clone(), template.dynamics);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("hand-guiding-budget".into()).unwrap();
    assert!(runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .is_err());
    assert_eq!(backend.enable_calls.load(Ordering::Acquire), 0);
}

#[tokio::test]
async fn hand_guiding_is_exclusive_and_cannot_become_a_trajectory() {
    let (runtime, backend) = runtime_and_backend_for_safety_test();
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("hand-guiding-owner".into()).unwrap();
    runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .unwrap();
    assert_eq!(runtime.acquire("competitor".into()).unwrap().0, 0);
    assert!(runtime
        .submit_trajectory(streaming_command(session, vec![]))
        .is_err());
    assert!(runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .is_err());
    assert!(runtime
        .start_gravity_comp(session, &[1.0; DOF])
        .await
        .is_err());
    assert!(runtime.set_gravity(session, [0.0, 0.0, 9.81]).is_err());
    assert!(runtime.release(session + 1).await.is_err());
    assert!(backend.is_enabled());
    runtime.release(session).await.unwrap();
    assert!(!backend.is_enabled());
    assert!(runtime.data.read().gravity_comp.is_none());
    assert!(runtime.gravity_comp_heartbeat(session, 1).is_err());
    let (new_session, _, _) = runtime.acquire("next-owner".into()).unwrap();
    assert_ne!(new_session, session);
    runtime
        .set_mode(new_session, OperatingMode::Active)
        .await
        .unwrap();
}

#[tokio::test]
async fn hand_guiding_heartbeat_cannot_replay_or_revive_an_expired_lease() {
    let (runtime, _) = runtime_and_backend_for_safety_test();
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("hand-guiding-heartbeat".into()).unwrap();
    runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .unwrap();
    assert!(runtime.gravity_comp_heartbeat(session + 1, 1).is_err());
    runtime.gravity_comp_heartbeat(session, 1).unwrap();
    let renewed = runtime
        .data
        .read()
        .gravity_comp
        .as_ref()
        .unwrap()
        .renewed_at;
    assert!(runtime.gravity_comp_heartbeat(session, 1).is_err());
    assert!(runtime.gravity_comp_heartbeat(session, 0).is_err());
    assert_eq!(
        runtime
            .data
            .read()
            .gravity_comp
            .as_ref()
            .unwrap()
            .renewed_at,
        renewed
    );
    runtime
        .data
        .write()
        .gravity_comp
        .as_mut()
        .unwrap()
        .renewed_at = Instant::now() - GRAVITY_COMP_LEASE;
    assert!(runtime
        .gravity_comp_heartbeat(session, 2)
        .unwrap_err()
        .to_string()
        .contains("expired"));
}

#[tokio::test]
async fn hand_guiding_heartbeat_reports_the_original_joint_fault() {
    let (runtime, backend) = runtime_and_backend_for_safety_test();
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("hand-guiding-fault-report".into()).unwrap();
    runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .unwrap();
    let mut overspeed = [crate::conversion::MotorTarget::default(); DOF];
    overspeed[1].velocity_rev_s = 1.0;
    backend.set_targets(overspeed).await.unwrap();
    let loop_task = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::timeout(Duration::from_millis(500), async {
        while runtime.mode() != OperatingMode::Fault || backend.is_enabled() {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .expect("hand-guiding overspeed must fault and disable");
    let state = runtime.driver_state_proto();
    assert_eq!(state.fault_code, FAULT_MEASURED_OVERSPEED);
    let error = runtime
        .gravity_comp_heartbeat(session, 1)
        .unwrap_err()
        .to_string();
    assert!(error.contains("0x1007"));
    assert!(error.contains("joint_2 measured velocity"));
    assert!(error.contains(&state.fault_reason));
    assert!(runtime.data.read().gravity_comp.is_none());
    runtime.shutdown().await.unwrap();
    loop_task.await.unwrap();
}

#[tokio::test]
async fn hand_guiding_deadman_fault_retries_disable_without_position_hold() {
    let (runtime, backend) = runtime_and_backend_for_safety_test();
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("hand-guiding-deadman".into()).unwrap();
    runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .unwrap();
    runtime
        .data
        .write()
        .gravity_comp
        .as_mut()
        .unwrap()
        .renewed_at = Instant::now() - GRAVITY_COMP_LEASE;
    backend.fail_next_disables(1);
    let loop_task = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::timeout(Duration::from_millis(500), async {
        while backend.is_enabled() {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .unwrap();
    assert_eq!(runtime.mode(), OperatingMode::Fault);
    assert_eq!(
        runtime.driver_state_proto().fault_code,
        FAULT_COMMAND_WATCHDOG
    );
    assert!(backend.disable_attempts() >= 2);
    assert!(backend.targets().iter().all(|t| t.kp_nm_rev == 0.0));
    assert!(runtime.gravity_comp_heartbeat(session, 1).is_err());
    runtime.shutdown().await.unwrap();
    loop_task.await.unwrap();
}

#[tokio::test]
async fn hand_guiding_runs_with_heartbeats_and_stops_on_shutdown() {
    let (runtime, backend) = runtime_and_backend_for_safety_test();
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("hand-guiding-live".into()).unwrap();
    runtime
        .start_gravity_comp(session, &[0.5; DOF])
        .await
        .unwrap();
    let loop_task = tokio::spawn(runtime.clone().run_control_loop());
    // Survive both the ordinary 100 ms command watchdog and the 500 ms
    // guiding lease without sending any trajectory/position command.
    for sequence in 1..=14 {
        runtime.gravity_comp_heartbeat(session, sequence).unwrap();
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    assert_eq!(runtime.mode(), OperatingMode::GravityComp);
    assert!(runtime.driver_state_proto().command_age_s < 0.2);
    runtime.shutdown().await.unwrap();
    loop_task.await.unwrap();
    assert!(!backend.is_enabled());
    assert!(runtime.gravity_comp_heartbeat(session, 15).is_err());
}

#[tokio::test]
async fn uncommissioned_continuous_modes_never_enable_the_backend() {
    for requested in [OperatingMode::Passive, OperatingMode::GravityComp] {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        runtime.initialize().await.unwrap();
        let (session_id, _, _) = runtime.acquire("fail-closed-test".into()).unwrap();

        let error = runtime.set_mode(session_id, requested).await.unwrap_err();
        assert!(error.to_string().contains("unavailable"));
        assert_eq!(runtime.mode(), OperatingMode::Disabled);

        // Mock feedback follows setpoints only while enabled. A post-error
        // setpoint therefore also proves that no backend enable occurred.
        let mut probe = [crate::conversion::MotorTarget::default(); DOF];
        probe[0].position_rev = 0.5;
        backend.set_targets(probe).await.unwrap();
        assert_eq!(backend.feedback().joints[0].position_rev, 0.0);
    }
}

#[tokio::test]
async fn failed_mode_transition_latches_fault_after_successful_rollback() {
    let (runtime, backend) = runtime_and_backend_for_safety_test();
    runtime.initialize().await.unwrap();
    let (session_id, _, _) = runtime.acquire("mode-failure-test".into()).unwrap();
    backend.fail_next_target_updates(1);

    let error = runtime
        .set_mode(session_id, OperatingMode::Active)
        .await
        .unwrap_err();

    assert!(error
        .to_string()
        .contains("mode transition to Active failed"));
    assert_eq!(runtime.mode(), OperatingMode::Fault);
    assert_eq!(
        runtime.driver_state_proto().fault_code,
        FAULT_MODE_TRANSITION
    );
    assert!(!backend.is_enabled());
    assert!(runtime
        .event_log_proto()
        .events
        .iter()
        .any(|event| event.code == "fault_disable_confirmed"));
}

#[tokio::test]
async fn failed_mode_transition_and_rollback_retries_until_disabled() {
    let (runtime, backend) = runtime_and_backend_for_safety_test();
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let (session_id, _, _) = runtime.acquire("mode-rollback-retry-test".into()).unwrap();
    backend.fail_next_target_updates(1);
    backend.fail_next_disables(1);

    let error = runtime
        .set_mode(session_id, OperatingMode::Active)
        .await
        .unwrap_err();
    assert!(error.to_string().contains("rollback disable failed"));
    assert_eq!(runtime.mode(), OperatingMode::Fault);
    assert!(
        backend.is_enabled(),
        "injected rollback failure must be observable"
    );

    let loop_task = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::timeout(Duration::from_millis(500), async {
        while backend.is_enabled() {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .expect("fault loop did not retry the failed mode-transition rollback");

    assert!(backend.disable_attempts() >= 2);
    assert_eq!(runtime.mode(), OperatingMode::Fault);
    assert!(runtime
        .event_log_proto()
        .events
        .iter()
        .any(|event| event.code == "fault_disable_confirmed"));

    loop_task.abort();
    let _ = loop_task.await;
}

#[tokio::test]
async fn control_loop_disables_if_an_uncommissioned_mode_is_observed() {
    for requested in [OperatingMode::Passive, OperatingMode::GravityComp] {
        let (runtime, backend) = runtime_and_backend_for_safety_test();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();

        // Simulate an internal invariant violation with hardware already
        // enabled; public set_mode cannot create either of these states.
        backend
            .enable_compressed_mit([Default::default(); DOF])
            .await
            .unwrap();
        runtime.data.write().safety.mode = requested;

        let loop_task = tokio::spawn(runtime.clone().run_control_loop());
        tokio::time::timeout(Duration::from_millis(100), async {
            while runtime.mode() != OperatingMode::Fault {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("control loop did not reject uncommissioned mode");
        tokio::time::sleep(Duration::from_millis(5)).await;

        let stopped_position = backend.feedback().joints[0].position_rev;
        let mut probe = [crate::conversion::MotorTarget::default(); DOF];
        probe[0].position_rev = 0.5;
        backend.set_targets(probe).await.unwrap();
        assert_eq!(backend.feedback().joints[0].position_rev, stopped_position);
        assert!(runtime
            .driver_state_proto()
            .fault_reason
            .contains("unsupported"));

        loop_task.abort();
        let _ = loop_task.await;
    }
}

#[tokio::test]
async fn control_loop_latches_and_disables_on_measured_overspeed() {
    let (runtime, backend) = runtime_and_backend_for_safety_test();
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let (session_id, _, _) = runtime.acquire("overspeed-test".into()).unwrap();
    runtime
        .set_mode(session_id, OperatingMode::Active)
        .await
        .unwrap();

    let mut overspeed = [crate::conversion::MotorTarget::default(); DOF];
    overspeed[0].velocity_rev_s = 1.0;
    backend.set_targets(overspeed).await.unwrap();

    let loop_task = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::timeout(Duration::from_millis(100), async {
        while runtime.mode() != OperatingMode::Fault {
            tokio::task::yield_now().await;
        }
    })
    .await
    .expect("control loop did not latch measured overspeed");
    // Fault state is published before the best-effort hardware disable.
    tokio::time::sleep(Duration::from_millis(5)).await;

    let state = runtime.driver_state_proto();
    assert_eq!(state.fault_code, FAULT_MEASURED_OVERSPEED);
    assert!(state.fault_reason.contains("joint_1 measured velocity"));

    let stopped_position = backend.feedback().joints[0].position_rev;
    let mut post_fault_target = [crate::conversion::MotorTarget::default(); DOF];
    post_fault_target[0].position_rev = 0.5;
    backend.set_targets(post_fault_target).await.unwrap();
    assert_eq!(
        backend.feedback().joints[0].position_rev,
        stopped_position,
        "fault path must disable the backend before returning"
    );

    loop_task.abort();
    let _ = loop_task.await;
}

#[tokio::test]
async fn command_watchdog_holds_latest_feedback_and_retries_until_disabled() {
    let (runtime, backend) = runtime_and_backend_for_safety_test();
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let (session_id, _, _) = runtime.acquire("watchdog-contract-test".into()).unwrap();
    runtime
        .set_mode(session_id, OperatingMode::Active)
        .await
        .unwrap();

    let mut command = streaming_command(session_id, vec![]);
    command.points[0].q[0] = 1.0;
    command.t_from_start_ns[0] = 1_000_000_000;
    runtime.submit_trajectory(command).unwrap();
    backend.fail_next_disables(1);

    let loop_task = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::timeout(Duration::from_millis(500), async {
        while runtime.mode() != OperatingMode::Fault || backend.is_enabled() {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .expect("watchdog did not retry the confirmed disable");

    assert!(backend.disable_attempts() >= 2);
    let feedback = backend.feedback();
    let held = backend.targets();
    for (target, state) in held.iter().zip(feedback.joints) {
        assert!((target.position_rev - state.position_rev).abs() < 1.0e-6);
        assert_eq!(target.velocity_rev_s, 0.0);
    }
    let state = runtime.driver_state_proto();
    assert_eq!(state.fault_code, FAULT_COMMAND_WATCHDOG);
    assert!(state.fault_reason.contains("confirmed disable"));
    let events = runtime.event_log_proto().events;
    assert!(events
        .iter()
        .any(|event| event.code == "watchdog_feedback_hold_applied"));
    assert!(events
        .iter()
        .any(|event| event.code == "fault_disable_confirmed"));

    loop_task.abort();
    let _ = loop_task.await;
}

#[tokio::test]
async fn releasing_and_reacquiring_a_session_does_not_clear_a_latched_fault() {
    let runtime = runtime_for_safety_test();
    runtime.initialize().await.unwrap();
    let (session_id, _, _) = runtime.acquire("first-client".into()).unwrap();
    {
        let mut data = runtime.data.write();
        data.safety
            .latch_fault(FAULT_FEEDBACK_TIMEOUT, "motor feedback timeout/offline");
    }

    runtime.release(session_id).await.unwrap();

    let state = runtime.driver_state_proto();
    assert_eq!(runtime.mode(), OperatingMode::Fault);
    assert!(state.fault_latched);
    assert_eq!(state.fault_code, FAULT_FEEDBACK_TIMEOUT);
    assert_eq!(state.fault_reason, "motor feedback timeout/offline");
    assert!(!state.session_owned);

    let (recovery_session, _, _) = runtime.acquire("recovery-client".into()).unwrap();
    let error = runtime
        .set_mode(recovery_session, OperatingMode::Active)
        .await
        .unwrap_err();
    assert!(error.to_string().contains("fault is latched"));

    runtime.clear_fault(recovery_session).await.unwrap();
    assert_eq!(runtime.mode(), OperatingMode::Disabled);
    assert!(!runtime.driver_state_proto().fault_latched);
    runtime.release(recovery_session).await.unwrap();
}

#[test]
fn empty_tau_ff_delegates_gravity_but_explicit_zero_does_not() {
    let runtime = runtime_for_safety_test();
    let (session_id, _, _) = runtime.acquire("gravity-policy-test".into()).unwrap();
    runtime.data.write().safety.mode = OperatingMode::Active;

    runtime
        .submit_trajectory(streaming_command(session_id, vec![]))
        .unwrap();
    assert!(
        runtime
            .data
            .read()
            .command
            .as_ref()
            .unwrap()
            .automatic_gravity_feedforward
    );

    runtime
        .submit_trajectory(streaming_command(session_id, vec![0.0; DOF]))
        .unwrap();
    assert!(
        !runtime
            .data
            .read()
            .command
            .as_ref()
            .unwrap()
            .automatic_gravity_feedforward
    );
}

#[test]
fn automatic_gravity_feedforward_uses_per_joint_profile_scale() {
    let template = runtime_for_safety_test();
    let mut profile = (*template.profile).clone();
    profile.gravity_vector_base_m_s2 = [0.0, 0.0, 9.81];
    profile.joints[0].gravity_compensation_scale = 0.25;
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let dynamics = ArmDynamics::from_parts(
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
    let runtime = ArmRuntime::new(Arc::new(profile), Arc::new(MockBackend::new()), dynamics);
    let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
    let mut targets = targets([0.0; DOF]);

    runtime.apply_gravity_feedforward(&mut targets, &feedback);

    // The profile's +Z gravity reverses the model's built-in -Z default;
    // per-axis scaling is applied only after that vector-based G(q).
    assert!((targets[0].torque_nm - (1.962 * 0.25)).abs() < 1.0e-5);
    assert!(targets[1..]
        .iter()
        .all(|target| target.torque_nm.abs() < 1.0e-6));
}

#[test]
fn automatic_motion_feedforward_is_not_added_to_explicit_client_torque() {
    let template = runtime_for_safety_test();
    let mut profile = (*template.profile).clone();
    for j in &mut profile.joints {
        j.gravity_compensation_scale = 0.0;
    }
    profile.joints[0].motion_feedforward = Some(crate::profile::MotionFeedforward {
        positive_nm: 0.2,
        negative_nm: 0.4,
        velocity_scale_rad_s: 0.001,
    });
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let dynamics = ArmDynamics::from_parts(
        vec![([0.0; 3], identity, [0.0, 1.0, 0.0]); DOF],
        vec![(0.0, [0.0; 3]); DOF],
        [0.0, 0.0, -9.81],
    );
    let runtime = ArmRuntime::new(Arc::new(profile), Arc::new(MockBackend::new()), dynamics);
    let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
    let mut output = targets([0.0; DOF]);
    output[0].velocity_rad_s = -0.01;
    runtime.apply_command_gravity_feedforward(&mut output, &feedback, true, 0);
    assert!((output[0].torque_nm + 0.4).abs() < 1e-6);
    runtime.apply_command_gravity_feedforward(&mut output, &feedback, true, 1);
    assert!(
        (output[0].torque_nm + 0.4).abs() < 1e-6,
        "must replace, not accumulate"
    );
    output[0].torque_nm = 0.125;
    runtime.apply_command_gravity_feedforward(&mut output, &feedback, false, 2);
    assert_eq!(output[0].torque_nm, 0.125);
    output[0].velocity_rad_s = 0.0;
    runtime.apply_command_gravity_feedforward(&mut output, &feedback, true, 3);
    assert_eq!(output[0].torque_nm, 0.0);
}

fn runtime_with_startup_gravity() -> (ArmRuntime, Arc<MockBackend>) {
    let template = runtime_for_safety_test();
    let mut profile = (*template.profile).clone();
    profile.gravity_vector_base_m_s2 = [0.0, 0.0, 9.81];
    profile.controller.gravity_startup_slew_rate_nm_s = Some(5.0);
    profile.joints[0].gravity_compensation_scale = 0.25;
    profile.joints[0].gravity_compensation_limit_nm = Some(0.3);
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let dynamics = ArmDynamics::from_parts(
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
    let backend = Arc::new(MockBackend::new());
    (
        ArmRuntime::new(Arc::new(profile), backend.clone(), dynamics),
        backend,
    )
}

#[tokio::test]
async fn active_starts_with_zero_gravity_then_tracks_directly_after_bounded_ramp() {
    let (runtime, backend) = runtime_with_startup_gravity();
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("startup-gravity".into()).unwrap();
    runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap();
    assert!(backend
        .feedback()
        .joints
        .iter()
        .all(|joint| joint.torque_nm == 0.0));
    let start = runtime
        .data
        .read()
        .gravity_startup_ramp
        .as_ref()
        .unwrap()
        .last_tick_ns;
    let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
    let mut command = targets([0.0; DOF]);
    runtime.apply_command_gravity_feedforward(&mut command, &feedback, true, start + 20_000_000);
    assert!((command[0].torque_nm - 0.1).abs() < 1.0e-6);
    assert!(runtime.data.read().gravity_startup_ramp.is_some());
    runtime.apply_command_gravity_feedforward(&mut command, &feedback, true, start + 80_000_000);
    assert!((command[0].torque_nm - 0.3).abs() < 1.0e-6);
    assert!(runtime.data.read().gravity_startup_ramp.is_none());

    let mut changed_q = [0.0; DOF];
    changed_q[0] = std::f32::consts::FRAC_PI_2;
    let changed_feedback = feedback_from_ros(&runtime, changed_q, [0.0; DOF]);
    runtime.apply_command_gravity_feedforward(
        &mut command,
        &changed_feedback,
        true,
        start + 81_000_000,
    );
    assert!(
        command[0].torque_nm.abs() < 1.0e-5,
        "settled gravity must not lag feedback behind a continuous slew limiter"
    );

    runtime
        .set_mode(session, OperatingMode::Disabled)
        .await
        .unwrap();
    runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap();
    assert_eq!(
        runtime
            .data
            .read()
            .gravity_startup_ramp
            .as_ref()
            .unwrap()
            .output_nm,
        [0.0; DOF]
    );
}

#[tokio::test]
async fn startup_checks_eventual_firmware_torque_budget_before_any_enable() {
    let (template, _) = runtime_with_startup_gravity();
    let backend = Arc::new(ShutdownOrderBackend::new());
    backend
        .reject_nonzero_feedforward
        .store(true, Ordering::Release);
    let runtime = ArmRuntime::new(template.profile, backend.clone(), template.dynamics);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("insufficient-budget".into()).unwrap();
    let error = runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap_err();
    assert!(error.to_string().contains("headroom"));
    assert_eq!(backend.enable_calls.load(Ordering::Acquire), 0);
    assert!(!backend.enabled.load(Ordering::Acquire));
}

#[test]
fn feedback_hold_snaps_only_endpoint_roundoff_and_external_commands_remain_strict() {
    let template = runtime_for_safety_test();
    let mut profile = (*template.profile).clone();
    let joint = &mut profile.joints[1];
    joint.direction = -1;
    joint.zero_offset_rad = -0.001_000_664_2;
    joint.limits.position_lower_rad = -1.57;
    joint.limits.position_upper_rad = 2.09;
    joint.limits.measured_position_margin_rad = 0.01;
    let runtime = ArmRuntime::new(
        Arc::new(profile),
        Arc::new(MockBackend::new()),
        template.dynamics,
    );
    let mut feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
    // Actual GUI node-2 park reference after signed Q8.24 quantization.
    feedback.joints[1].position_rev = (0.249714_f64 * 16_777_216.0).round() as f32 / 16_777_216.0;
    let joint = &runtime.profile.joints[1];
    let measured = motor_position_to_ros(feedback.joints[1].position_rev, joint);
    assert!(measured < joint.limits.position_lower_rad);
    assert!(runtime.measured_feedback_fault(&feedback).is_none());
    let hold = runtime.hold_targets(&feedback);
    assert_eq!(hold[1].position_rad, joint.limits.position_lower_rad);
    runtime.validate_targets(&hold).unwrap();
    runtime.data.write().feedback = feedback.clone();
    let published = runtime.joint_state_proto();
    assert_eq!(published.q[1], joint.limits.position_lower_rad);
    let mut echoed_hold = hold.clone();
    echoed_hold[1].position_rad = published.q[1];
    runtime.validate_targets(&echoed_hold).unwrap();
    assert_eq!(
        runtime.data.read().feedback.joints[1].position_rev,
        feedback.joints[1].position_rev
    );
    let mut external = hold.clone();
    external[1].position_rad = measured;
    assert!(runtime.validate_targets(&external).is_err());

    let truly_outside = joint.limits.position_lower_rad - 0.001;
    feedback.joints[1].position_rev = ros_target_to_motor(
        RosTarget {
            position_rad: truly_outside,
            ..Default::default()
        },
        joint,
    )
    .position_rev;
    assert!(
        runtime.measured_feedback_fault(&feedback).is_none(),
        "measurement margin is separate from command authority"
    );
    let hold = runtime.hold_targets(&feedback);
    assert!(hold[1].position_rad < joint.limits.position_lower_rad);
    assert!(runtime.validate_targets(&hold).is_err());
}

#[tokio::test]
async fn meow_boundary_entry_keeps_feedback_and_commands_bounded() {
    let backend = Arc::new(DampingBackend::new(false));
    let mut runtime = runtime_with_backend(backend.clone());
    let profile = Arc::make_mut(&mut runtime.profile);
    profile.bus.protocol = MotorProtocol::Meow;
    profile.controller.gravity_startup_slew_rate_nm_s = Some(1.0);
    profile.joints[1].limits.position_lower_rad = -1.57;
    profile.joints[1].limits.position_upper_rad = 2.09;
    profile.joints[2].limits.position_lower_rad = -1.57;
    profile.joints[2].limits.position_upper_rad = 1.57;
    for joint in &mut profile.joints {
        joint.limits.measured_position_margin_rad = 0.01;
    }
    let q = [-0.048_891, -1.570_404, 1.579, 0.001_54, 0.011_08, 0.217_66];
    *backend.state.write() = feedback_from_ros(&runtime, q, [0.0; DOF]);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("meow-fold-boundary".into()).unwrap();
    runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap();
    assert_eq!(backend.inner.enable_calls.load(Ordering::Acquire), 1);
    let published = runtime.joint_state_proto().q;
    assert!((published[1] - q[1]).abs() < 1.0e-6);
    assert!((published[2] - q[2]).abs() < 1.0e-6);
    let sent = backend.last.read().unwrap();
    assert!(
        (motor_position_to_ros(sent[1].position_rev, &runtime.profile.joints[1]) + 1.57).abs()
            < 1.0e-6
    );
    assert!(
        (motor_position_to_ros(sent[2].position_rev, &runtime.profile.joints[2]) - 1.57).abs()
            < 1.0e-6
    );

    let mut echo = streaming_command(session, vec![]);
    echo.points[0].q = published.clone();
    runtime.submit_trajectory(echo.clone()).unwrap();
    let bounded = runtime
        .data
        .read()
        .command
        .as_ref()
        .unwrap()
        .targets
        .clone();
    assert_eq!(bounded[1].position_rad, -1.57);
    assert_eq!(bounded[2].position_rad, 1.57);
    runtime.validate_targets(&bounded).unwrap();
    for fault in [
        "different_pose",
        "velocity",
        "gains",
        "feedforward",
        "other_axis",
    ] {
        let mut bad = echo.clone();
        match fault {
            "different_pose" => bad.points[0].q[1] -= 0.001,
            "velocity" => bad.points[0].dq = vec![0.001; DOF],
            "gains" => bad.points[0].kp = vec![5.0; DOF],
            "feedforward" => bad.points[0].tau_ff = vec![0.0; DOF],
            "other_axis" => bad.points[0].q[0] += 0.01,
            _ => unreachable!(),
        }
        assert!(runtime.submit_trajectory(bad).is_err(), "accepted {fault}");
    }
    let start = runtime
        .data
        .read()
        .gravity_startup_ramp
        .as_ref()
        .unwrap()
        .last_tick_ns;
    runtime.apply_command_gravity_feedforward(
        &mut bounded.clone(),
        &backend.feedback(),
        true,
        start + 100_000_000,
    );
    assert!(runtime.data.read().gravity_startup_ramp.is_none());
    // The controller may still echo its activation reference after gravity
    // settles and before the first bounded FJT waypoint arrives.
    runtime.submit_trajectory(echo.clone()).unwrap();
    let mut motion = streaming_command(session, vec![]);
    motion.points[0].q = bounded.iter().map(|target| target.position_rad).collect();
    motion.points[0].q[0] += 0.01;
    runtime.submit_trajectory(motion).unwrap();
    assert!(runtime.data.read().meow_entry_hold.is_none());
    assert!(runtime.submit_trajectory(echo).is_err());
    assert_eq!(runtime.profile.joints[1].limits.position_lower_rad, -1.57);
    assert!((runtime.joint_state_proto().q[1] - q[1]).abs() < 1.0e-6);
    runtime.shutdown().await.unwrap();
}

#[tokio::test]
async fn meow_boundary_entry_rejects_excess_margin_or_moving_feedback_before_enable() {
    for (protocol, measured, speed, margin) in [
        (MotorProtocol::Meow, -1.581, 0.0, 0.01),
        (MotorProtocol::Meow, -1.570_404, 0.0201, 0.01),
        (MotorProtocol::Meow, -1.570_404, 0.0, 0.0),
        (MotorProtocol::Cia402, -1.570_404, 0.0, 0.01),
    ] {
        let backend = Arc::new(DampingBackend::new(false));
        let mut runtime = runtime_with_backend(backend.clone());
        let profile = Arc::make_mut(&mut runtime.profile);
        profile.bus.protocol = protocol;
        profile.joints[1].limits.position_lower_rad = -1.57;
        profile.joints[1].limits.measured_position_margin_rad = margin;
        let mut q = [0.0; DOF];
        q[1] = measured;
        let mut dq = [0.0; DOF];
        dq[1] = speed;
        *backend.state.write() = feedback_from_ros(&runtime, q, dq);
        runtime.initialize().await.unwrap();
        let (session, _, _) = runtime.acquire("bad-meow-boundary".into()).unwrap();
        assert!(runtime
            .set_mode(session, OperatingMode::Active)
            .await
            .is_err());
        assert_eq!(backend.inner.enable_calls.load(Ordering::Acquire), 0);
        runtime.shutdown().await.unwrap();
    }
}

#[tokio::test]
async fn startup_accepts_hold_heartbeats_but_rejects_motion_until_gravity_ready() {
    let (runtime, _) = runtime_with_startup_gravity();
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("gravity-ready-gate".into()).unwrap();
    runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap();
    runtime
        .submit_trajectory(streaming_command(session, vec![]))
        .unwrap();
    let mut motion = streaming_command(session, vec![]);
    motion.points[0].q[0] = 0.01;
    assert!(runtime
        .submit_trajectory(motion.clone())
        .unwrap_err()
        .to_string()
        .contains("gravity_ready"));
    let mut velocity = streaming_command(session, vec![]);
    velocity.points[0].dq = vec![0.01; DOF];
    assert!(runtime.submit_trajectory(velocity).is_err());
    let start = runtime
        .data
        .read()
        .gravity_startup_ramp
        .as_ref()
        .unwrap()
        .last_tick_ns;
    let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
    let mut command = targets([0.0; DOF]);
    runtime.apply_command_gravity_feedforward(&mut command, &feedback, true, start + 100_000_000);
    assert!(runtime
        .data
        .read()
        .events
        .iter()
        .any(|event| event.code == "gravity_ready"));
    runtime.submit_trajectory(motion).unwrap();
}

#[tokio::test]
async fn startup_zero_frame_does_not_hide_an_unsafe_eventual_gravity_target() {
    let (runtime, backend) = runtime_with_startup_gravity();
    let mut profile = (*runtime.profile).clone();
    profile.joints[0].gravity_compensation_limit_nm = None;
    profile.joints[0].limits.torque_nm = 0.1;
    let runtime = ArmRuntime::new(Arc::new(profile), backend.clone(), runtime.dynamics);
    runtime.initialize().await.unwrap();
    let (session, _, _) = runtime.acquire("bad-gravity-ramp".into()).unwrap();
    assert!(runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap_err()
        .to_string()
        .contains("torque exceeds software limit"));
    assert!(!backend.is_enabled());
    for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut ramp = GravityStartupRamp::new(0, targets([0.0; DOF]));
        let mut command = targets([0.0; DOF]);
        command[0].torque_nm = invalid;
        assert!(!ramp.apply(&mut command, 5.0, 1_000_000));
        assert!(!command[0].torque_nm.is_finite());
    }
}

#[test]
fn startup_ramp_uses_one_tick_for_all_axes_and_explicit_torque_bypasses_it() {
    let mut ramp = GravityStartupRamp::new(10_000_000, targets([0.0; DOF]));
    let mut command = targets([0.0; DOF]);
    command[0].torque_nm = 1.0;
    command[1].torque_nm = -1.0;
    assert!(!ramp.apply(&mut command, 5.0, 30_000_000));
    assert!((command[0].torque_nm - 0.1).abs() < 1.0e-6);
    assert!((command[1].torque_nm + 0.1).abs() < 1.0e-6);
    command[0].torque_nm = 1.0;
    command[1].torque_nm = -1.0;
    assert!(!ramp.apply(&mut command, 5.0, 30_000_000));
    assert!((command[0].torque_nm - 0.1).abs() < 1.0e-6);

    let (runtime, _) = runtime_with_startup_gravity();
    runtime.data.write().gravity_startup_ramp =
        Some(GravityStartupRamp::new(0, targets([0.0; DOF])));
    let feedback = feedback_from_ros(&runtime, [0.0; DOF], [0.0; DOF]);
    let mut explicit = targets([0.0; DOF]);
    explicit[0].torque_nm = -0.7; // Greater than the gravity-only 0.3 Nm clamp.
    runtime.apply_command_gravity_feedforward(&mut explicit, &feedback, false, 1_000_000);
    assert_eq!(explicit[0].torque_nm, -0.7);
    assert!(runtime.data.read().gravity_startup_ramp.is_none());
    runtime.apply_command_gravity_feedforward(&mut explicit, &feedback, true, 2_000_000);
    assert!((explicit[0].torque_nm - 0.3).abs() < 1.0e-6);
}

#[tokio::test]
async fn gravity_override_is_scoped_to_one_exclusive_session() {
    let template = runtime_for_safety_test();
    let mut profile = (*template.profile).clone();
    let profile_gravity = [0.0, 0.0, 9.81];
    profile.gravity_vector_base_m_s2 = profile_gravity;
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let dynamics = ArmDynamics::from_parts(
        vec![([0.0; 3], identity, [0.0, 1.0, 0.0]); DOF],
        vec![(0.0, [0.0; 3]); DOF],
        [0.0, 0.0, -9.81],
    );
    let runtime = ArmRuntime::new(Arc::new(profile), Arc::new(MockBackend::new()), dynamics);
    assert_eq!(runtime.data.read().gravity, profile_gravity);

    let (first_session, _, _) = runtime.acquire("first-gravity-client".into()).unwrap();
    runtime
        .set_gravity(first_session, [0.0, 0.0, -9.81])
        .unwrap();
    assert_eq!(runtime.data.read().gravity, [0.0, 0.0, -9.81]);
    runtime.release(first_session).await.unwrap();
    assert_eq!(runtime.data.read().gravity, profile_gravity);

    // New-session admission also restores the profile value defensively,
    // even if an earlier internal path left a stale runtime value behind.
    runtime.data.write().gravity = [0.0, -9.81, 0.0];
    let (second_session, _, _) = runtime.acquire("second-gravity-client".into()).unwrap();
    assert_eq!(runtime.data.read().gravity, profile_gravity);
    runtime
        .set_gravity(second_session, [0.0, 9.81, 0.0])
        .unwrap();
    runtime.shutdown().await.unwrap();
    assert_eq!(runtime.data.read().gravity, profile_gravity);
}

#[test]
fn unsupported_multi_point_chunks_are_rejected_instead_of_truncated() {
    let runtime = runtime_for_safety_test();
    let (session_id, _, _) = runtime.acquire("chunk-contract-test".into()).unwrap();
    runtime.data.write().safety.mode = OperatingMode::Active;
    let mut command = streaming_command(session_id, vec![]);
    command.points.push(command.points[0].clone());
    command.t_from_start_ns.push(20_000_000);

    let error = runtime.submit_trajectory(command).unwrap_err();
    assert!(error.to_string().contains("exactly one setpoint"));
}

#[test]
fn only_the_advertised_fault_timeout_behavior_is_accepted() {
    let runtime = runtime_for_safety_test();
    let (session_id, _, _) = runtime.acquire("timeout-contract-test".into()).unwrap();
    runtime.data.write().safety.mode = OperatingMode::Active;

    runtime
        .submit_trajectory(streaming_command(session_id, vec![]))
        .unwrap();
    for behavior in [
        pb::TimeoutBehavior::Unspecified,
        pb::TimeoutBehavior::Hold,
        pb::TimeoutBehavior::RampStop,
        pb::TimeoutBehavior::ShortBrake,
    ] {
        let mut command = streaming_command(session_id, vec![]);
        command.on_timeout = behavior as i32;
        let error = runtime.submit_trajectory(command).unwrap_err();
        assert!(error
            .to_string()
            .contains("only TIMEOUT_BEHAVIOR_FAULT is commissioned"));
    }

    let mut command = streaming_command(session_id, vec![]);
    command.on_timeout = i32::MAX;
    assert!(runtime
        .submit_trajectory(command)
        .unwrap_err()
        .to_string()
        .contains("unknown timeout behavior"));
}

#[test]
fn activation_rebases_from_disabled_feedback() {
    let old_positions = [0.8, 0.6, 0.4, 0.2, -0.2, -0.4];
    let moved_positions = [-0.5, -0.3, -0.1, 0.1, 0.3, 0.5];

    let mut interpolator = Interpolator::hold(targets(old_positions), 0);
    let activation = CommandEnvelope {
        generation: 2,
        source_sequence: 0,
        targets: targets(moved_positions),
        duration_ns: 0,
        received_at: Instant::now(),
        rebase_from_feedback: Some(targets(moved_positions)),
        automatic_gravity_feedforward: true,
    };

    activation
        .apply_to_interpolator(&mut interpolator, 20_000_000, &[0.2; DOF], &[0.1; DOF])
        .unwrap();

    let first_active_sample = interpolator.sample(20_000_000);
    for (target, expected_position) in first_active_sample.iter().zip(moved_positions) {
        assert!((target.position_rad - expected_position).abs() < 1e-6);
    }
}

#[test]
fn ordinary_zero_duration_command_remains_rate_limited() {
    let start_positions = [0.0; DOF];
    let goal_positions = [1.0; DOF];
    let mut interpolator = Interpolator::hold(targets(start_positions), 0);
    let command = CommandEnvelope {
        generation: 1,
        source_sequence: 0,
        targets: targets(goal_positions),
        duration_ns: 0,
        received_at: Instant::now(),
        rebase_from_feedback: None,
        automatic_gravity_feedforward: true,
    };

    command
        .apply_to_interpolator(&mut interpolator, 0, &[0.2; DOF], &[0.1; DOF])
        .unwrap();

    for target in interpolator.sample(0) {
        assert_eq!(target.position_rad, 0.0);
    }
    let one_second = interpolator.sample(1_000_000_000);
    assert!(one_second.iter().all(|target| {
        target.position_rad > 0.0 && target.position_rad < 1.0 && target.velocity_rad_s.abs() <= 0.2
    }));
    for index in 0..DOF {
        let (maximum_velocity, maximum_acceleration) = interpolator.segment_bounds(index);
        assert!(maximum_velocity <= 0.2_f32 as f64 * (1.0 + 1.0e-9));
        assert!(maximum_acceleration <= 0.1_f32 as f64 * (1.0 + 1.0e-9));
    }
}

#[test]
fn first_trajectory_preserves_unconsumed_activation_rebase() {
    let old_positions = [-1.0; DOF];
    let feedback_positions = [0.0; DOF];
    let goal_positions = [1.0; DOF];
    let activation = CommandEnvelope {
        generation: 1,
        source_sequence: 0,
        targets: targets(feedback_positions),
        duration_ns: 0,
        received_at: Instant::now(),
        rebase_from_feedback: Some(targets(feedback_positions)),
        automatic_gravity_feedforward: true,
    };
    let queued_trajectory = CommandEnvelope {
        generation: 2,
        source_sequence: 0,
        targets: targets(goal_positions),
        duration_ns: 0,
        received_at: Instant::now(),
        rebase_from_feedback: activation.rebase_from_feedback.clone(),
        automatic_gravity_feedforward: true,
    };
    let mut interpolator = Interpolator::hold(targets(old_positions), 0);

    queued_trajectory
        .apply_to_interpolator(&mut interpolator, 0, &[0.2; DOF], &[0.1; DOF])
        .unwrap();

    for target in interpolator.sample(0) {
        assert_eq!(target.position_rad, 0.0);
    }
    let one_second = interpolator.sample(1_000_000_000);
    assert!(one_second.iter().all(|target| {
        target.position_rad > 0.0 && target.position_rad < 1.0 && target.velocity_rad_s.abs() <= 0.2
    }));
    for index in 0..DOF {
        let (maximum_velocity, maximum_acceleration) = interpolator.segment_bounds(index);
        assert!(maximum_velocity <= 0.2_f32 as f64 * (1.0 + 1.0e-9));
        assert!(maximum_acceleration <= 0.1_f32 as f64 * (1.0 + 1.0e-9));
    }
}

#[tokio::test]
async fn cached_discovery_never_waits_for_mode_or_backend_operations() {
    let backend = Arc::new(ShutdownOrderBackend::new());
    let runtime = Arc::new(runtime_with_backend(backend.clone()));
    assert!(runtime
        .discover(false)
        .await
        .unwrap_err()
        .to_string()
        .contains("not initialized"));
    runtime.initialize().await.unwrap();
    let held_gate = runtime.mode_gate.lock().await;
    let cached = tokio::time::timeout(Duration::from_millis(50), runtime.discover(false))
        .await
        .expect("cached discovery waited on the mode gate")
        .unwrap();
    assert_eq!(cached.len(), 1);
    assert!(cached[0].identity_verified);
    assert_eq!(backend.discovery_calls.load(Ordering::Acquire), 1);
    drop(held_gate);
    runtime.shutdown().await.unwrap();
}

#[tokio::test]
async fn discovery_refresh_rechecks_mode_after_waiting_for_the_gate() {
    let backend = Arc::new(ShutdownOrderBackend::new());
    let runtime = Arc::new(runtime_with_backend(backend.clone()));
    runtime.initialize().await.unwrap();
    let held_gate = runtime.mode_gate.lock().await;
    let worker = tokio::spawn({
        let runtime = runtime.clone();
        async move { runtime.discover(true).await }
    });
    tokio::task::yield_now().await;
    // Represents a mode transition completing while owning the same gate.
    runtime.data.write().safety.mode = OperatingMode::Active;
    drop(held_gate);
    assert!(worker
        .await
        .unwrap()
        .unwrap_err()
        .to_string()
        .contains("DISABLED"));
    assert_eq!(backend.discovery_calls.load(Ordering::Acquire), 1);
    runtime.shutdown().await.unwrap();
}

#[tokio::test]
async fn slow_refresh_is_disabled_only_and_cached_lookup_remains_available() {
    let backend = Arc::new(ShutdownOrderBackend::new());
    backend.refresh_delay_ms.store(100, Ordering::Release);
    let runtime = Arc::new(runtime_with_backend(backend.clone()));
    runtime.initialize().await.unwrap();
    let session = runtime.acquire("slow-discovery".into()).unwrap().0;
    let refresh = tokio::spawn({
        let runtime = runtime.clone();
        async move { runtime.discover(true).await }
    });
    tokio::time::timeout(Duration::from_secs(1), async {
        while !backend.discovery_in_progress.load(Ordering::Acquire) {
            tokio::task::yield_now().await;
        }
    })
    .await
    .unwrap();
    tokio::time::timeout(Duration::from_millis(50), runtime.discover(false))
        .await
        .unwrap()
        .unwrap();
    let enable = tokio::spawn({
        let runtime = runtime.clone();
        async move { runtime.set_mode(session, OperatingMode::Active).await }
    });
    tokio::task::yield_now().await;
    assert_eq!(backend.enable_calls.load(Ordering::Acquire), 0);
    refresh.await.unwrap().unwrap();
    enable.await.unwrap().unwrap();
    let control = tokio::spawn(runtime.clone().run_control_loop());
    let before = backend.target_calls.load(Ordering::Acquire);
    for _ in 0..5 {
        assert!(runtime
            .discover(true)
            .await
            .unwrap_err()
            .to_string()
            .contains("DISABLED"));
        assert_eq!(runtime.discover(false).await.unwrap().len(), 1);
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    assert!(backend.target_calls.load(Ordering::Acquire) > before);
    assert_eq!(backend.discovery_calls.load(Ordering::Acquire), 2);
    runtime.shutdown().await.unwrap();
    control.await.unwrap();
}

#[tokio::test]
async fn refresh_rejects_fault_pending_disable_and_terminal_stop() {
    let (runtime, _) = runtime_and_backend_for_safety_test();
    runtime.initialize().await.unwrap();
    for mode in [
        OperatingMode::Active,
        OperatingMode::GravityComp,
        OperatingMode::Fault,
    ] {
        runtime.data.write().safety.mode = mode;
        assert!(runtime
            .discover(true)
            .await
            .unwrap_err()
            .to_string()
            .contains("DISABLED"));
    }
    runtime.data.write().safety.mode = OperatingMode::Disabled;
    runtime.data.write().disable_pending = true;
    assert!(runtime
        .discover(true)
        .await
        .unwrap_err()
        .to_string()
        .contains("DISABLED"));
    runtime.data.write().disable_pending = false;
    runtime.data.write().damped_stopping = true;
    assert!(runtime
        .discover(true)
        .await
        .unwrap_err()
        .to_string()
        .contains("shutting down"));
    assert!(runtime.discover(false).await.is_err());
    runtime.shutdown().await.unwrap();
}

#[tokio::test]
async fn prepared_old_owner_command_cannot_commit_after_release_and_reacquire() {
    let runtime = Arc::new(runtime_for_safety_test());
    runtime.initialize().await.unwrap();
    let old_session = runtime.acquire("old-owner".into()).unwrap().0;
    runtime
        .set_mode(old_session, OperatingMode::Active)
        .await
        .unwrap();
    let (prepared_tx, prepared_rx) = tokio::sync::oneshot::channel();
    let (resume_tx, resume_rx) = std::sync::mpsc::channel();
    let submission = tokio::task::spawn_blocking({
        let runtime = runtime.clone();
        move || {
            let prepared = runtime
                .prepare_trajectory(streaming_command(old_session, vec![]))
                .unwrap();
            prepared_tx.send(()).unwrap();
            resume_rx.recv().unwrap();
            runtime.commit_trajectory(prepared)
        }
    });
    prepared_rx.await.unwrap();
    runtime.release(old_session).await.unwrap();
    let new_session = runtime.acquire("new-owner".into()).unwrap().0;
    runtime
        .set_mode(new_session, OperatingMode::Active)
        .await
        .unwrap();
    let generation = runtime.data.read().command_generation;
    resume_tx.send(()).unwrap();
    assert!(submission
        .await
        .unwrap()
        .unwrap_err()
        .to_string()
        .contains("exclusive session"));
    {
        let data = runtime.data.read();
        assert_eq!(data.command_generation, generation);
        assert_eq!(data.session.as_ref().unwrap().id, new_session);
    }
    runtime.shutdown().await.unwrap();
}

#[tokio::test]
async fn commit_rechecks_terminal_admission_and_current_mode() {
    let runtime = runtime_for_safety_test();
    runtime.initialize().await.unwrap();
    let session = runtime.acquire("commit-state".into()).unwrap().0;
    runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap();
    for state in ["disabled", "damping", "closing"] {
        let prepared = runtime
            .prepare_trajectory(streaming_command(session, vec![]))
            .unwrap();
        let generation = runtime.data.read().command_generation;
        {
            let mut data = runtime.data.write();
            match state {
                "disabled" => data.safety.mode = OperatingMode::Disabled,
                "damping" => data.damped_stopping = true,
                "closing" => data.closing = true,
                _ => unreachable!(),
            }
        }
        assert!(runtime.commit_trajectory(prepared).is_err());
        let mut data = runtime.data.write();
        assert_eq!(data.command_generation, generation);
        data.safety.mode = OperatingMode::Active;
        data.damped_stopping = false;
        data.closing = false;
    }
    runtime.shutdown().await.unwrap();
}

#[tokio::test]
async fn delayed_source_command_is_still_fresh_by_receive_time() {
    // Characterize current policy; this is evidence for a later source-age design,
    // not permission to silently change watchdog or the wire protocol.
    let runtime = runtime_for_safety_test();
    runtime.initialize().await.unwrap();
    let session = runtime.acquire("delayed-source".into()).unwrap().0;
    runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap();
    let mut command = streaming_command(session, vec![]);
    command.header = Some(pb::Header {
        seq: 42,
        stamp_ns: 0,
        sync_ns: None,
    });
    runtime.submit_trajectory(command).unwrap();
    {
        let data = runtime.data.read();
        let accepted = data.command.as_ref().unwrap();
        assert!(accepted.received_at.elapsed() < runtime.profile.command_watchdog());
        assert_eq!(accepted.source_sequence, 42);
    }
    runtime.shutdown().await.unwrap();
}

#[tokio::test]
async fn suspending_only_control_task_does_not_execute_its_watchdog() {
    let backend = Arc::new(ShutdownOrderBackend::new());
    let mut runtime = runtime_with_backend(backend.clone());
    Arc::get_mut(&mut runtime.profile)
        .unwrap()
        .controller
        .command_watchdog_ms = 10;
    let runtime = Arc::new(runtime);
    runtime.initialize().await.unwrap();
    let session = runtime.acquire("control-freeze".into()).unwrap().0;
    runtime
        .set_mode(session, OperatingMode::Active)
        .await
        .unwrap();
    let control = tokio::spawn(runtime.clone().run_control_loop());
    tokio::task::yield_now().await;
    control.abort();
    let error = control.await.unwrap_err();
    assert!(error.is_cancelled());
    let output_count = backend.target_calls.load(Ordering::Acquire);
    tokio::time::sleep(Duration::from_millis(30)).await;
    assert_eq!(backend.target_calls.load(Ordering::Acquire), output_count);
    assert!(
        runtime
            .data
            .read()
            .command
            .as_ref()
            .unwrap()
            .received_at
            .elapsed()
            > runtime.profile.command_watchdog()
    );
    assert_eq!(runtime.mode(), OperatingMode::Active);
    assert!(backend.enabled.load(Ordering::Acquire));
    // Lifecycle supervision must invoke shutdown; it is not this task's watchdog.
    runtime.shutdown().await.unwrap();
    assert!(!backend.enabled.load(Ordering::Acquire));
}

#[tokio::test]
async fn control_output_from_the_previous_owner_is_discarded_after_gate_wait() {
    let backend = Arc::new(ShutdownOrderBackend::new());
    let runtime = Arc::new(runtime_with_backend(backend.clone()));
    runtime.initialize().await.unwrap();
    let old_session = runtime.acquire("old-output-owner".into()).unwrap().0;
    runtime
        .set_mode(old_session, OperatingMode::Active)
        .await
        .unwrap();
    let held_gate = runtime.mode_gate.lock().await;
    let validation_count = backend.validation_calls.load(Ordering::Acquire);
    let sent_before = backend.sent_position_rev.read().len();
    let control = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::timeout(Duration::from_millis(50), async {
        while backend.validation_calls.load(Ordering::Acquire) == validation_count {
            tokio::task::yield_now().await;
        }
    })
    .await
    .expect("control task did not calculate the old owner's output");
    // The output is now computed and blocked at mode_gate. Model the state
    // commit of release/reacquire+ACTIVE while owning the same gate. Public
    // release/reacquire admission is covered by the command-commit test.
    {
        let mut data = runtime.data.write();
        data.session = Some(SessionLease {
            id: old_session + 1,
            client_name: "new-output-owner".into(),
        });
        data.command_generation += 1;
        let hold = targets([0.5; DOF]);
        data.command = Some(CommandEnvelope {
            generation: data.command_generation,
            source_sequence: 0,
            targets: hold.clone(),
            duration_ns: 0,
            received_at: Instant::now(),
            rebase_from_feedback: Some(hold),
            automatic_gravity_feedforward: true,
        });
    }
    drop(held_gate);
    tokio::time::timeout(Duration::from_millis(50), async {
        while backend.sent_position_rev.read().len() < sent_before + 2 {
            tokio::task::yield_now().await;
        }
    })
    .await
    .expect("new owner's output was never published");
    assert!(
        backend.sent_position_rev.read()[sent_before..]
            .iter()
            .all(|position| (*position * std::f32::consts::TAU - 0.5).abs() < 1.0e-6),
        "an old-owner output entered the new session"
    );
    runtime.shutdown().await.unwrap();
    control.await.unwrap();
}

#[tokio::test]
async fn old_owner_gravity_tick_cannot_advance_or_clear_a_new_startup_ramp() {
    for automatic in [false, true] {
        let (runtime, backend) = runtime_with_startup_gravity();
        let runtime = Arc::new(runtime);
        runtime.initialize().await.unwrap();
        let old_owner = runtime.acquire("old-gravity-owner".into()).unwrap().0;
        runtime
            .set_mode(old_owner, OperatingMode::Active)
            .await
            .unwrap();
        let old_control_owner = runtime.data.read().control_owner();
        let (captured_tx, captured_rx) = tokio::sync::oneshot::channel();
        let (resume_tx, resume_rx) = std::sync::mpsc::channel();
        let tick = tokio::task::spawn_blocking({
            let runtime = runtime.clone();
            let feedback = backend.feedback();
            move || {
                let mut output = targets([0.0; DOF]);
                captured_tx.send(()).unwrap();
                resume_rx.recv().unwrap();
                runtime.apply_command_gravity_feedforward_for_owner(
                    &mut output,
                    &feedback,
                    automatic,
                    1_000_000_000,
                    old_control_owner,
                )
            }
        });
        captured_rx.await.unwrap();
        runtime.release(old_owner).await.unwrap();
        let new_owner = runtime.acquire("new-gravity-owner".into()).unwrap().0;
        runtime
            .set_mode(new_owner, OperatingMode::Active)
            .await
            .unwrap();
        let expected_ramp = {
            let data = runtime.data.read();
            let ramp = data.gravity_startup_ramp.as_ref().unwrap();
            (ramp.last_tick_ns, ramp.output_nm, ramp.hold_targets.clone())
        };
        let events_before = runtime.event_log_proto().events.len();
        resume_tx.send(()).unwrap();
        assert!(!tick.await.unwrap(), "old-owner gravity work was admitted");
        {
            let data = runtime.data.read();
            let ramp = data
                .gravity_startup_ramp
                .as_ref()
                .expect("old tick cleared the new owner's ramp");
            assert_eq!(
                (ramp.last_tick_ns, ramp.output_nm, ramp.hold_targets.clone()),
                expected_ramp
            );
        }
        assert_eq!(runtime.event_log_proto().events.len(), events_before);
        let mut new_output = targets([0.0; DOF]);
        let new_control_owner = runtime.data.read().control_owner();
        assert!(runtime.apply_command_gravity_feedforward_for_owner(
            &mut new_output,
            &backend.feedback(),
            automatic,
            1_000_000_000,
            new_control_owner,
        ));
        runtime.shutdown().await.unwrap();
    }
}

#[tokio::test]
async fn owned_gravity_updates_reject_fault_disable_and_terminal_admission() {
    let (runtime, backend) = runtime_with_startup_gravity();
    runtime.initialize().await.unwrap();
    let owner = runtime.acquire("gravity-admission".into()).unwrap().0;
    runtime
        .set_mode(owner, OperatingMode::Active)
        .await
        .unwrap();
    let expected_owner = runtime.data.read().control_owner();
    let initial_output = runtime
        .data
        .read()
        .gravity_startup_ramp
        .as_ref()
        .unwrap()
        .output_nm;
    for state in ["disabled", "fault", "damping", "closing"] {
        {
            let mut data = runtime.data.write();
            match state {
                "disabled" => data.safety.mode = OperatingMode::Disabled,
                "fault" => data.safety.mode = OperatingMode::Fault,
                "damping" => data.damped_stopping = true,
                "closing" => data.closing = true,
                _ => unreachable!(),
            }
        }
        for automatic in [false, true] {
            assert!(!runtime.apply_command_gravity_feedforward_for_owner(
                &mut targets([0.0; DOF]),
                &backend.feedback(),
                automatic,
                1_000_000_000,
                expected_owner,
            ));
            assert_eq!(
                runtime
                    .data
                    .read()
                    .gravity_startup_ramp
                    .as_ref()
                    .unwrap()
                    .output_nm,
                initial_output
            );
        }
        let mut data = runtime.data.write();
        data.safety.mode = OperatingMode::Active;
        data.damped_stopping = false;
        data.closing = false;
    }
    runtime.shutdown().await.unwrap();
}

#[tokio::test]
async fn successful_hardware_enable_cannot_overwrite_a_fault_latched_in_flight() {
    for requested in [OperatingMode::Active, OperatingMode::GravityComp] {
        for rollback_fails in [false, true] {
            let backend = Arc::new(ShutdownOrderBackend::new());
            backend.pause_enable.store(true, Ordering::Release);
            let runtime = Arc::new(runtime_with_backend(backend.clone()));
            runtime.initialize().await.unwrap();
            let owner = runtime.acquire("fault-during-enable".into()).unwrap().0;
            let transition = tokio::spawn({
                let runtime = runtime.clone();
                async move {
                    if requested == OperatingMode::GravityComp {
                        runtime.start_gravity_comp(owner, &[1.0; DOF]).await
                    } else {
                        runtime.set_mode(owner, requested).await
                    }
                }
            });
            tokio::time::timeout(
                Duration::from_millis(100),
                backend.enable_entered.notified(),
            )
            .await
            .expect("enable did not enter the controlled hardware wait");
            runtime.latch_whole_arm_fault(
                FAULT_MEASURED_OVERSPEED,
                "overspeed observed while enable was pending",
            );
            backend
                .fail_disable
                .store(rollback_fails, Ordering::Release);
            backend.resume_enable.notify_one();
            let error = transition.await.unwrap().unwrap_err();
            assert!(
                error.to_string().contains("latched fault"),
                "unexpected transition result: {error:#}"
            );
            {
                let data = runtime.data.read();
                assert_eq!(data.safety.mode, OperatingMode::Fault);
                assert_eq!(data.safety.fault_code, FAULT_MEASURED_OVERSPEED);
                assert_eq!(
                    data.safety.fault_reason,
                    "overspeed observed while enable was pending"
                );
                assert!(data.command.is_none() && data.gravity_comp.is_none());
                assert_eq!(data.disable_pending, rollback_fails);
                assert_eq!(data.next_disable_retry_at.is_some(), rollback_fails);
            }
            assert_eq!(backend.enabled.load(Ordering::Acquire), rollback_fails);
            if !rollback_fails {
                assert!(runtime
                    .event_log_proto()
                    .events
                    .iter()
                    .any(|event| event.code == "fault_disable_confirmed"));
            } else {
                // Background retry confirms disable without resetting the first fault.
                backend.fail_disable.store(false, Ordering::Release);
                let control = tokio::spawn(runtime.clone().run_control_loop());
                tokio::time::timeout(Duration::from_millis(200), async {
                    while runtime.data.read().disable_pending {
                        tokio::task::yield_now().await;
                    }
                })
                .await
                .expect("fault disable retry never completed");
                assert!(!backend.enabled.load(Ordering::Acquire));
                assert_eq!(runtime.mode(), OperatingMode::Fault);
                runtime.begin_shutdown();
                control.await.unwrap();
                runtime.shutdown().await.unwrap();
                continue;
            }
            assert!(runtime
                .set_mode(owner, OperatingMode::Active)
                .await
                .is_err());
            runtime.clear_fault(owner).await.unwrap();
            assert_eq!(runtime.mode(), OperatingMode::Disabled);
            assert!(!runtime.driver_state_proto().fault_latched);
            runtime.shutdown().await.unwrap();
        }
    }
}

#[tokio::test]
async fn same_owner_reactivation_preserves_the_new_ramp_from_stale_ticks() {
    for disable_first in [false, true] {
        for automatic in [false, true] {
            let (runtime, backend) = runtime_with_startup_gravity();
            let runtime = Arc::new(runtime);
            runtime.initialize().await.unwrap();
            let owner = runtime.acquire("reactivation-owner".into()).unwrap().0;
            runtime
                .set_mode(owner, OperatingMode::Active)
                .await
                .unwrap();
            let old_control_owner = runtime.data.read().control_owner();
            let (captured_tx, captured_rx) = tokio::sync::oneshot::channel();
            let (resume_tx, resume_rx) = std::sync::mpsc::channel();
            let old_tick = tokio::task::spawn_blocking({
                let runtime = runtime.clone();
                let feedback = backend.feedback();
                move || {
                    let mut output = targets([0.0; DOF]);
                    captured_tx.send(()).unwrap();
                    resume_rx.recv().unwrap();
                    runtime.apply_command_gravity_feedforward_for_owner(
                        &mut output,
                        &feedback,
                        automatic,
                        1_000_000_000,
                        old_control_owner,
                    )
                }
            });
            captured_rx.await.unwrap();
            if disable_first {
                runtime
                    .set_mode(owner, OperatingMode::Disabled)
                    .await
                    .unwrap();
            }
            runtime
                .set_mode(owner, OperatingMode::Active)
                .await
                .unwrap();
            let (new_control_owner, expected_ramp) = {
                let data = runtime.data.read();
                let ramp = data.gravity_startup_ramp.as_ref().unwrap();
                (
                    data.control_owner(),
                    (ramp.last_tick_ns, ramp.output_nm, ramp.hold_targets.clone()),
                )
            };
            assert_eq!(new_control_owner.session_id, old_control_owner.session_id);
            assert_ne!(new_control_owner.epoch, old_control_owner.epoch);
            resume_tx.send(()).unwrap();
            assert!(!old_tick.await.unwrap());
            {
                let data = runtime.data.read();
                let ramp = data
                    .gravity_startup_ramp
                    .as_ref()
                    .expect("old activation cleared the new ramp");
                assert_eq!(
                    (ramp.last_tick_ns, ramp.output_nm, ramp.hold_targets.clone()),
                    expected_ramp
                );
            }
            // Ordinary streaming updates within the activation remain valid.
            let generation_before = runtime.data.read().command_generation;
            runtime
                .submit_trajectory(streaming_command(owner, vec![]))
                .unwrap();
            assert!(runtime.data.read().command_generation > generation_before);
            assert_eq!(runtime.data.read().control_owner(), new_control_owner);
            assert!(runtime.apply_command_gravity_feedforward_for_owner(
                &mut targets([0.0; DOF]),
                &backend.feedback(),
                automatic,
                1_000_000_000,
                new_control_owner,
            ));
            runtime.shutdown().await.unwrap();
        }
    }
}

#[tokio::test]
async fn same_owner_mode_commit_discards_an_output_waiting_for_hardware_gate() {
    let backend = Arc::new(ShutdownOrderBackend::new());
    let runtime = Arc::new(runtime_with_backend(backend.clone()));
    runtime.initialize().await.unwrap();
    let owner = runtime.acquire("same-owner-output".into()).unwrap().0;
    runtime
        .set_mode(owner, OperatingMode::Active)
        .await
        .unwrap();
    let old_control_owner = runtime.data.read().control_owner();
    // New activation follows the changed measured pose; the prior calculation
    // still belongs to its original zero-position activation command.
    backend.feedback.write().joints[0].position_rev = 0.5 / std::f32::consts::TAU;
    backend.pause_enable.store(true, Ordering::Release);
    let sent_before = backend.sent_position_rev.read().len();
    let reactivation = tokio::spawn({
        let runtime = runtime.clone();
        async move { runtime.set_mode(owner, OperatingMode::Active).await }
    });
    tokio::time::timeout(
        Duration::from_millis(100),
        backend.enable_entered.notified(),
    )
    .await
    .unwrap();
    let validation_before = backend.validation_calls.load(Ordering::Acquire);
    let control = tokio::spawn(runtime.clone().run_control_loop());
    tokio::time::timeout(Duration::from_millis(50), async {
        while backend.validation_calls.load(Ordering::Acquire) == validation_before {
            tokio::task::yield_now().await;
        }
    })
    .await
    .expect("old activation never calculated an output while mode held the gate");
    backend.resume_enable.notify_one();
    reactivation.await.unwrap().unwrap();
    assert_eq!(
        runtime.data.read().control_owner().session_id,
        old_control_owner.session_id
    );
    assert_ne!(
        runtime.data.read().control_owner().epoch,
        old_control_owner.epoch
    );
    tokio::time::timeout(Duration::from_millis(50), async {
        while backend.sent_position_rev.read().len() < sent_before + 3 {
            tokio::task::yield_now().await;
        }
    })
    .await
    .expect("new activation never produced its measured-pose output");
    assert!(
        backend.sent_position_rev.read()[sent_before..]
            .iter()
            .all(|position| (*position * std::f32::consts::TAU - 0.5).abs() < 1.0e-6),
        "output from the prior activation crossed a same-owner mode commit"
    );
    runtime.shutdown().await.unwrap();
    control.await.unwrap();
}
