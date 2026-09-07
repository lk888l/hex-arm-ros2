use std::collections::BTreeSet;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;
use std::{fmt, future::Future};

use anyhow::{Context, Result};
use can_transport::socketcan::SocketCanBus;
use can_transport::CanBus;
use clap::{Parser, ValueEnum};
use hex_arm_controller::backend::{MockBackend, MotorBackend, RealBackend};
use hex_arm_controller::commissioning::{
    run_joint1_censored_torque_diagnostic, run_joint1_first_position_diagnostic,
    run_joint1_negative_censored_torque_diagnostic, run_joint3_assisted_position_diagnostic,
    run_joint3_gravity_unload_diagnostic, run_joint4_assisted_position_diagnostic,
    run_joint4_censored_torque_diagnostic, run_joint4_first_position_diagnostic,
    run_joint5_first_position_diagnostic, run_joint6_first_position_diagnostic,
    run_single_axis_commissioning, run_single_axis_diagnostic, CommissioningRequest,
    Joint1CensoredTorqueDiagnosticRequest, Joint1FirstPositionDiagnosticRequest,
    Joint1NegativeCensoredTorqueDiagnosticRequest, Joint3AssistedPositionDiagnosticRequest,
    Joint3GravityUnloadDiagnosticRequest, Joint4AssistedPositionDiagnosticRequest,
    Joint4CensoredTorqueDiagnosticRequest, Joint4FirstPositionDiagnosticRequest,
    Joint5FirstPositionDiagnosticRequest, Joint6FirstPositionDiagnosticRequest,
    SingleAxisDiagnosticMode, SingleAxisDiagnosticRequest,
};
use hex_arm_controller::discovery::{discover_read_only, DiscoveryOptions};
use hex_arm_controller::meow_backend::MeowBackend;
use hex_arm_controller::payload_dynamics::load_profile_dynamics;
use hex_arm_controller::profile::{BusTransport, HardwareProfile, MotorProtocol};
use hex_arm_controller::protocol;
use hex_arm_controller::runtime::ArmRuntime;
use hex_arm_controller::socketcan_preflight::HistoricalCanXStatsAcknowledgement;
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
#[command(about = "Firefly Y6 userspace USB/CAN-FD controller")]
struct Arguments {
    #[arg(long)]
    profile: Option<PathBuf>,
    #[arg(long, default_value_t = false)]
    mock: bool,
    #[arg(long, default_value = "")]
    zenoh_connect: String,
    #[arg(long, default_value = "")]
    zenoh_listen: String,
    /// Read heartbeats and CANopen identity only. This branch never loads a
    /// hardware profile, initializes a drive, sends NMT/PDO, or starts control.
    #[arg(long, default_value_t = false)]
    discover_only: bool,
    /// Parse the profile, load the dynamics model, and verify that every real
    /// command window avoids the unverified single-turn seam. Opens no CAN bus.
    #[arg(
        long,
        default_value_t = false,
        conflicts_with = "discover_only",
        conflicts_with = "mock"
    )]
    validate_profile_only: bool,
    /// Optional six-joint URDF pose for an offline gravity/motor-target report.
    /// This computes a report only; it never commands a startup position.
    #[arg(
        long,
        num_args = 6,
        allow_hyphen_values = true,
        requires = "validate_profile_only"
    )]
    check_pose_rad: Option<Vec<f32>>,
    /// Run the isolated real-hardware commissioning path for exactly one
    /// canonical joint. Requires an explicit motion delta, duration, and
    /// --allow-motion acknowledgement; starts neither ROS nor Zenoh.
    #[arg(
        long,
        requires_all = ["delta_rad", "duration_sec", "allow_motion"],
        conflicts_with_all = [
            "discover_only",
            "validate_profile_only",
            "mock",
            "zenoh_connect",
            "zenoh_listen"
        ]
    )]
    commission_axis: Option<String>,
    /// Signed logical-joint displacement. Commissioning always performs a
    /// smooth start -> delta -> start round trip.
    #[arg(long, allow_hyphen_values = true, requires = "commission_axis")]
    delta_rad: Option<f32>,
    /// Total seconds for the complete out-and-back round trip.
    #[arg(long, requires = "commission_axis")]
    duration_sec: Option<f32>,
    /// Explicit acknowledgement that the commissioning command enables one
    /// physical motor and causes a small motion.
    #[arg(long, default_value_t = false, requires = "commission_axis")]
    allow_motion: bool,
    /// Run an explicitly selected bounded single-axis diagnostic. The fixed
    /// J1 first-position mode and the J2 modes never start ROS or Zenoh.
    #[arg(
        long,
        requires_all = ["diagnostic_mode", "allow_diagnostic_motion"],
        conflicts_with_all = [
            "discover_only",
            "validate_profile_only",
            "mock",
            "commission_axis",
            "recover_heartbeat_lost",
            "zenoh_connect",
            "zenoh_listen"
        ]
    )]
    diagnose_axis: Option<String>,
    /// `hold` checks a fixed position hold only. `tau-ff-staircase` adds small
    /// positive feed-forward steps. `position-round-trip` is a fixed J2-only
    /// +0.005 rad / 4.0 s smooth survey. `gravity-hold-censored` is a separate,
    /// compile-time-fixed gravity-ramp/hold identification mode; neither fixed
    /// mode is a general jog command. `joint1-first-position` is a separate,
    /// fixed +0.005 rad / 4.0 s J1-only first-motion survey.
    /// `joint1-torque-censored` and `joint1-negative-torque-censored` are
    /// separately authorized fixed-position follow-ups: each changes only
    /// feed-forward in its named direction and stops at a mirrored feedback
    /// censor.
    #[arg(long, value_enum, requires = "diagnose_axis")]
    diagnostic_mode: Option<DiagnosticCliMode>,
    #[arg(long, requires = "diagnose_axis")]
    diagnostic_hold_sec: Option<f32>,
    #[arg(long, requires = "diagnose_axis")]
    diagnostic_peak_tau_ff_nm: Option<f32>,
    #[arg(long, requires = "diagnose_axis")]
    diagnostic_step_tau_ff_nm: Option<f32>,
    #[arg(long, requires = "diagnose_axis")]
    diagnostic_dwell_sec: Option<f32>,
    #[arg(long, allow_hyphen_values = true, requires = "diagnose_axis")]
    diagnostic_delta_rad: Option<f32>,
    #[arg(long, requires = "diagnose_axis")]
    diagnostic_duration_sec: Option<f32>,
    /// Explicit acknowledgement that a selected J1 or J2 diagnostic enables
    /// one physical motor.
    #[arg(long, default_value_t = false, requires = "diagnose_axis")]
    allow_diagnostic_motion: bool,
    /// Independent acknowledgement required when a tau-ff staircase exceeds
    /// 0.25 Nm and for either compile-time-fixed high-tier diagnostic.
    #[arg(long, default_value_t = false, requires = "diagnose_axis")]
    allow_high_torque_diagnostic: bool,
    /// Independent acknowledgement for the compile-time-fixed censored
    /// gravity hold. It is required only by `gravity-hold-censored` and cannot
    /// authorize any hold, staircase, or position-round-trip request.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_high_torque_diagnostic"],
        required_if_eq("diagnostic_mode", "gravity-hold-censored")
    )]
    acknowledge_censored_gravity_hold: bool,
    /// Independent acknowledgement for the compile-time-fixed J1 first
    /// position survey. It cannot authorize any J2 diagnostic mode.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint1-first-position"),
        conflicts_with_all = [
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint1_first_position: bool,
    /// Independent acknowledgement for the fixed-position J1 positive-torque
    /// identification. It cannot authorize the J1 position survey or J2.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint1-torque-censored"),
        conflicts_with_all = [
            "acknowledge_joint1_first_position",
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint1_torque_censored: bool,
    /// Independent acknowledgement for the mirror-image fixed-position J1
    /// negative-torque identification. It cannot authorize the positive mode,
    /// the J1 position survey, or any J2 mode.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint1-negative-torque-censored"),
        conflicts_with_all = [
            "acknowledge_joint1_first_position",
            "acknowledge_joint1_torque_censored",
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint1_negative_torque_censored: bool,
    /// Independent acknowledgement for the fixed J5 negative 5 mrad first
    /// motion. It cannot authorize J1, J2, or any numeric diagnostic mode.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint5-first-position"),
        conflicts_with_all = [
            "acknowledge_joint1_first_position",
            "acknowledge_joint1_torque_censored",
            "acknowledge_joint1_negative_torque_censored",
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint5_first_position: bool,
    /// Independent acknowledgement for the fixed J4 negative 5 mrad position
    /// channel survey. It cannot authorize gravity identification or another axis.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint4-first-position"),
        conflicts_with_all = [
            "acknowledge_joint1_first_position",
            "acknowledge_joint1_torque_censored",
            "acknowledge_joint1_negative_torque_censored",
            "acknowledge_joint5_first_position",
            "acknowledge_joint6_first_position",
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint4_first_position: bool,
    /// Independent acknowledgement for J4's fixed-position negative-torque
    /// identification. It cannot authorize a position trajectory or another axis.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint4-torque-censored"),
        conflicts_with_all = [
            "acknowledge_joint1_first_position",
            "acknowledge_joint1_torque_censored",
            "acknowledge_joint1_negative_torque_censored",
            "acknowledge_joint4_first_position",
            "acknowledge_joint5_first_position",
            "acknowledge_joint6_first_position",
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint4_torque_censored: bool,
    /// Independent acknowledgement for the fixed J4 -5 mrad survey whose
    /// negative assistance follows the trajectory phase and returns to zero.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint4-assisted-position"),
        conflicts_with_all = [
            "acknowledge_joint1_first_position",
            "acknowledge_joint1_torque_censored",
            "acknowledge_joint1_negative_torque_censored",
            "acknowledge_joint4_first_position",
            "acknowledge_joint4_torque_censored",
            "acknowledge_joint5_first_position",
            "acknowledge_joint6_first_position",
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint4_assisted_position: bool,
    /// Independent acknowledgement for J3's fixed-position negative
    /// gravity-load identification. It cannot authorize a position trajectory,
    /// another axis, numeric torque arguments, or historical CAN counters.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint3-gravity-unload"),
        conflicts_with_all = [
            "acknowledge_joint1_first_position",
            "acknowledge_joint1_torque_censored",
            "acknowledge_joint1_negative_torque_censored",
            "acknowledge_joint4_first_position",
            "acknowledge_joint4_torque_censored",
            "acknowledge_joint4_assisted_position",
            "acknowledge_joint3_assisted_position",
            "acknowledge_joint5_first_position",
            "acknowledge_joint6_first_position",
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint3_gravity_unload: bool,
    /// Independent acknowledgement for the fixed J3 inward 5 mrad survey
    /// with trajectory-synchronous -0.25 Nm peak assistance.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint3-assisted-position"),
        conflicts_with_all = [
            "acknowledge_joint1_first_position",
            "acknowledge_joint1_torque_censored",
            "acknowledge_joint1_negative_torque_censored",
            "acknowledge_joint3_gravity_unload",
            "acknowledge_joint4_first_position",
            "acknowledge_joint4_torque_censored",
            "acknowledge_joint4_assisted_position",
            "acknowledge_joint5_first_position",
            "acknowledge_joint6_first_position",
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint3_assisted_position: bool,
    /// Independent acknowledgement for the fixed J6 negative 5 mrad first
    /// motion. It cannot authorize another axis or a numeric diagnostic mode.
    #[arg(
        long,
        default_value_t = false,
        requires_all = ["diagnose_axis", "allow_diagnostic_motion"],
        required_if_eq("diagnostic_mode", "joint6-first-position"),
        conflicts_with_all = [
            "acknowledge_joint1_first_position",
            "acknowledge_joint1_torque_censored",
            "acknowledge_joint1_negative_torque_censored",
            "acknowledge_joint5_first_position",
            "allow_high_torque_diagnostic",
            "acknowledge_censored_gravity_hold",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    acknowledge_joint6_first_position: bool,
    /// Exact historical SocketCAN error_warning value accepted only for a J2
    /// high-tier diagnostic. Requires the paired passive count and a separate
    /// acknowledgement; ordinary control and commissioning remain strict-zero.
    #[arg(
        long,
        requires_all = [
            "diagnose_axis",
            "diagnostic_can_error_passive_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    diagnostic_can_error_warning_baseline: Option<u32>,
    /// Exact historical SocketCAN error_passive value accepted only for a J2
    /// high-tier diagnostic.
    #[arg(
        long,
        requires_all = [
            "diagnose_axis",
            "diagnostic_can_error_warning_baseline",
            "acknowledge_historical_can_xstats"
        ]
    )]
    diagnostic_can_error_passive_baseline: Option<u32>,
    /// Independent acknowledgement of the two exact historical CAN xstats.
    /// It never permits any other CAN/netdev error counter or a later change.
    #[arg(
        long,
        default_value_t = false,
        requires_all = [
            "diagnose_axis",
            "allow_high_torque_diagnostic",
            "diagnostic_can_error_warning_baseline",
            "diagnostic_can_error_passive_baseline"
        ]
    )]
    acknowledge_historical_can_xstats: bool,
    /// Explicitly clear an existing host-heartbeat-lost fault (0x8130) on the
    /// listed arm nodes. The list must exactly match the live faulted set and
    /// is restricted to nodes 1..=6. Starts neither ROS nor Zenoh and never
    /// initializes or enables a drive.
    #[arg(
        long,
        value_delimiter = ',',
        num_args = 1..,
        requires = "allow_heartbeat_fault_reset",
        conflicts_with_all = [
            "discover_only",
            "validate_profile_only",
            "mock",
            "commission_axis",
            "zenoh_connect",
            "zenoh_listen"
        ]
    )]
    recover_heartbeat_lost: Vec<u8>,
    /// Explicit acknowledgement for the narrowly gated 0x8130 fault reset.
    #[arg(long, default_value_t = false, requires = "recover_heartbeat_lost")]
    allow_heartbeat_fault_reset: bool,
    #[arg(long, value_enum, default_value_t = DiscoveryTransport::SocketCan)]
    transport: DiscoveryTransport,
    #[arg(long, default_value = "can0")]
    interface: String,
    #[arg(long = "expected-node")]
    expected_nodes: Vec<u8>,
    #[arg(long = "auxiliary-node")]
    auxiliary_nodes: Vec<u8>,
    #[arg(long = "timeout", alias = "timeout-sec", default_value_t = 2.0)]
    timeout_sec: f64,
    #[arg(long, default_value_t = 0.25)]
    sdo_timeout_sec: f64,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum DiscoveryTransport {
    SocketCan,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum DiagnosticCliMode {
    Hold,
    TauFfStaircase,
    PositionRoundTrip,
    GravityHoldCensored,
    Joint1FirstPosition,
    Joint1TorqueCensored,
    Joint1NegativeTorqueCensored,
    Joint3GravityUnload,
    Joint3AssistedPosition,
    Joint4FirstPosition,
    Joint4TorqueCensored,
    Joint4AssistedPosition,
    Joint5FirstPosition,
    Joint6FirstPosition,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();
    let arguments = Arguments::parse();
    if arguments.discover_only {
        return run_discovery(&arguments).await;
    }

    let profile_path = arguments
        .profile
        .as_ref()
        .context("real control/recovery requires --profile /absolute/path/to/verified.yaml")?;
    let profile = Arc::new(HardwareProfile::from_path(profile_path)?);
    if !arguments.recover_heartbeat_lost.is_empty()
        || arguments.commission_axis.is_some()
        || arguments.diagnose_axis.is_some()
    {
        anyhow::ensure!(
            profile.bus.protocol == MotorProtocol::Cia402,
            "legacy CiA402 commissioning/recovery commands do not support Meow firmware; use the Meow runtime and supervised ROS trajectory path"
        );
    }
    if !arguments.recover_heartbeat_lost.is_empty() {
        return run_hardware_heartbeat_recovery(&arguments, profile).await;
    }
    let dynamics = load_profile_dynamics(&profile)?;
    anyhow::ensure!(dynamics.dof() == 6, "URDF dynamics model is not six-axis");
    if arguments.validate_profile_only {
        profile.validate_single_turn_command_windows()?;
        if let Some(payload) = &profile.tip_payload {
            println!(
                "tip payload bound: node={} mount={} mass_kg={} inertial_calibrated={} source_sha256={}",
                payload.auxiliary_node_id,
                payload.mount_link,
                payload.mass_kg,
                payload.inertial_calibrated,
                payload.source_urdf_sha256
            );
        }
        println!(
            "motor protocol: {:?}, loop_hz={}",
            profile.bus.protocol, profile.controller.loop_hz
        );
        if let Some(pose) = &arguments.check_pose_rad {
            anyhow::ensure!(
                pose.iter().all(|q| q.is_finite()),
                "check pose must be finite"
            );
            let gravity = dynamics.gravity_torque_with(pose, profile.gravity_vector_base_m_s2);
            for ((joint, q), g) in profile.joints.iter().zip(pose).zip(gravity) {
                anyhow::ensure!(
                    (joint.limits.position_lower_rad..=joint.limits.position_upper_rad).contains(q),
                    "{} check pose is outside command limits",
                    joint.name
                );
                let scaled = g * joint.gravity_compensation_scale;
                let tff = joint
                    .gravity_compensation_limit_nm
                    .map_or(scaled, |limit| scaled.clamp(-limit, limit));
                let target = hex_arm_controller::conversion::ros_target_to_motor(
                    hex_arm_controller::conversion::RosTarget {
                        position_rad: *q,
                        velocity_rad_s: 0.0,
                        torque_nm: tff,
                        kp_nm_rad: joint.default_kp,
                        kd_nm_s_rad: joint.default_kd,
                    },
                    joint,
                );
                println!(
                    "{} q_rad={:.6} motor_rev={:.6} G_nm={:.6} gravity_ff_nm={:.6} motor_ff_nm={:.6} kp_nm_rev={:.6} kd_nm_s_rev={:.6}",
                    joint.name, q, target.position_rev, g, tff, target.torque_nm,
                    target.kp_nm_rev, target.kd_nm_s_rev
                );
                anyhow::ensure!(
                    tff.abs() <= joint.limits.torque_nm,
                    "{} gravity feed-forward exceeds software torque limit",
                    joint.name
                );
            }
        }
        println!(
            "profile validation passed: six joints, dynamics model, and protocol-specific command windows are valid"
        );
        return Ok(());
    }
    if arguments.commission_axis.is_some() {
        return run_hardware_commissioning(&arguments, profile, dynamics).await;
    }
    if arguments.diagnose_axis.is_some() {
        return run_hardware_diagnostic(&arguments, profile, dynamics).await;
    }

    run_normal_control(&arguments, profile, dynamics).await
}

/// Start the control plane before any potentially slow real-CAN work.  The
/// direct listener must exist while the ROS bridge is configuring, but no
/// robot_api queryable or publisher is registered until all six drives have
/// initialized and the runtime is safely DISABLED.
async fn run_normal_control(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let zenoh_config = build_zenoh_config(arguments)?;
    let session = zenoh::open(zenoh_config)
        .await
        .map_err(|error| anyhow::anyhow!("open early Zenoh robot_api transport: {error}"))?;
    tracing::info!("Zenoh transport is open; initializing the arm with API endpoints withheld");
    // Install before RealBackend::open.  A signal arriving during preflight or
    // manager construction is retained and handled as soon as we own the
    // backend, so Docker SIGTERM cannot regain its default immediate-exit path.
    let mut termination_signals = TerminationSignals::install()
        .context("install controller SIGINT/SIGTERM handlers before backend initialization")?;

    let backend: Arc<dyn MotorBackend> = if arguments.mock {
        tracing::warn!("using simulated motor backend; no physical outputs exist");
        Arc::new(MockBackend::new())
    } else {
        match profile.bus.protocol {
            MotorProtocol::Cia402 => Arc::new(RealBackend::open(profile.clone()).await?),
            MotorProtocol::Meow => Arc::new(MeowBackend::open(profile.clone()).await?),
        }
    };
    let runtime = Arc::new(ArmRuntime::new(profile.clone(), backend, dynamics));
    let initialization = tokio::select! {
        biased;
        signal = termination_signals.received() => match signal {
            Ok(signal) => Err(anyhow::anyhow!(
                "controller startup interrupted by {signal} before the API became ready"
            )),
            Err(error) => Err(error.context("controller termination signal monitor failed")),
        },
        result = runtime.initialize() => result.context("initialize arm in DISABLED state"),
    };
    if let Err(error) = initialization {
        tracing::warn!(%error, "controller initialization stopped; starting orderly drive shutdown");
        let shutdown = runtime.shutdown().await;
        return combine_operation_and_cleanup("controller initialization", Err(error), shutdown);
    }

    let protocol_tasks = match async {
        let urdf_xml = std::fs::read_to_string(&profile.urdf_path).context("read URDF resource")?;
        protocol::serve(session, runtime.clone(), urdf_xml).await
    }
    .await
    {
        Ok(tasks) => tasks,
        Err(error) => {
            runtime.begin_shutdown();
            let shutdown = runtime.shutdown().await;
            return combine_operation_and_cleanup("controller API startup", Err(error), shutdown);
        }
    };
    let control_task = tokio::spawn(runtime.clone().run_control_loop());

    tracing::info!(prefix = %profile.robot_prefix, "Firefly Y6 controller ready and DISABLED");
    let operation_result = match termination_signals.received().await {
        Ok(signal) => {
            tracing::info!(%signal, "controller termination signal received");
            Ok(())
        }
        Err(error) => Err(error.context("wait for controller SIGINT/SIGTERM")),
    };

    // Close admission synchronously before either worker is asked to stop.
    // The control loop is joined first. Final runtime shutdown then waits on
    // mode_gate, so every admitted hardware RPC finishes before backend
    // disable/heartbeat-disarm. Only after hardware is physically quiescent do
    // we abort and join possibly stuck Zenoh reply/publication I/O; transport
    // liveness can therefore never postpone the safety shutdown.
    runtime.begin_shutdown();
    tracing::info!("controller stopping; disabling drives and disarming heartbeat consumers");
    let control_shutdown = match control_task.await {
        Ok(()) => Ok(()),
        Err(error) => Err(anyhow::anyhow!("control loop task failed: {error}")),
    };
    let backend_shutdown = runtime.shutdown().await;
    let protocol_shutdown = protocol_tasks.cancel_and_join().await;
    let shutdown = combine_runtime_shutdown(protocol_shutdown, control_shutdown, backend_shutdown);
    combine_operation_and_cleanup("controller runtime", operation_result, shutdown)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TerminationSignal {
    Interrupt,
    Terminate,
}

impl fmt::Display for TerminationSignal {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::Interrupt => "SIGINT",
            Self::Terminate => "SIGTERM",
        })
    }
}

#[cfg(unix)]
struct TerminationSignals {
    interrupt: tokio::signal::unix::Signal,
    terminate: tokio::signal::unix::Signal,
}

#[cfg(unix)]
impl TerminationSignals {
    /// Install both handlers synchronously before drive initialization starts.
    fn install() -> Result<Self> {
        use tokio::signal::unix::{signal, SignalKind};

        Ok(Self {
            interrupt: signal(SignalKind::interrupt()).context("install SIGINT handler")?,
            terminate: signal(SignalKind::terminate()).context("install SIGTERM handler")?,
        })
    }

    async fn received(&mut self) -> Result<TerminationSignal> {
        tokio::select! {
            signal = self.interrupt.recv() => {
                signal.context("SIGINT stream closed")?;
                Ok(TerminationSignal::Interrupt)
            }
            signal = self.terminate.recv() => {
                signal.context("SIGTERM stream closed")?;
                Ok(TerminationSignal::Terminate)
            }
        }
    }
}

#[cfg(not(unix))]
struct TerminationSignals;

#[cfg(not(unix))]
impl TerminationSignals {
    fn install() -> Result<Self> {
        Ok(Self)
    }

    async fn received(&mut self) -> Result<TerminationSignal> {
        tokio::signal::ctrl_c()
            .await
            .context("listen for process interrupt")?;
        Ok(TerminationSignal::Interrupt)
    }
}

fn combine_operation_and_cleanup(
    scope: &str,
    operation: Result<()>,
    cleanup: Result<()>,
) -> Result<()> {
    match (operation, cleanup) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(operation_error), Ok(())) => Err(operation_error),
        (Ok(()), Err(cleanup_error)) => {
            Err(cleanup_error).with_context(|| format!("{scope} cleanup failed"))
        }
        (Err(operation_error), Err(cleanup_error)) => Err(anyhow::anyhow!(
            "{scope} failed: {operation_error:#}; cleanup also failed: {cleanup_error:#}"
        )),
    }
}

fn combine_runtime_shutdown(
    protocol: Result<()>,
    control_loop: Result<()>,
    backend: Result<()>,
) -> Result<()> {
    let mut failures = Vec::new();
    for (scope, result) in [
        ("Zenoh protocol", protocol),
        ("control loop", control_loop),
        ("backend disable/heartbeat disarm", backend),
    ] {
        if let Err(error) = result {
            failures.push(format!("{scope}: {error:#}"));
        }
    }
    anyhow::ensure!(
        failures.is_empty(),
        "controller shutdown failed: {}",
        failures.join("; ")
    );
    Ok(())
}

fn parse_heartbeat_recovery_nodes(values: &[u8]) -> Result<BTreeSet<u8>> {
    anyhow::ensure!(
        !values.is_empty(),
        "--recover-heartbeat-lost requires at least one node"
    );
    let mut nodes = BTreeSet::new();
    for node_id in values {
        anyhow::ensure!(
            (1..=6).contains(node_id),
            "--recover-heartbeat-lost is restricted to arm nodes 1..=6 (got {node_id})"
        );
        anyhow::ensure!(
            nodes.insert(*node_id),
            "duplicate node {node_id} in --recover-heartbeat-lost"
        );
    }
    Ok(nodes)
}

async fn run_hardware_heartbeat_recovery(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
) -> Result<()> {
    anyhow::ensure!(
        arguments.allow_heartbeat_fault_reset,
        "heartbeat-lost recovery requires --allow-heartbeat-fault-reset"
    );
    let requested_nodes = parse_heartbeat_recovery_nodes(&arguments.recover_heartbeat_lost)?;
    let mut termination_signals = TerminationSignals::install()
        .context("install SIGINT/SIGTERM guard before heartbeat recovery opens CAN")?;

    tracing::warn!(
        nodes = ?requested_nodes,
        "starting explicit 0x8130-only recovery; no drive will be initialized or enabled"
    );
    let backend = RealBackend::open(profile)
        .await
        .context("open strictly preflighted CAN backend for heartbeat recovery")?;
    let operation = tokio::select! {
        biased;
        signal = termination_signals.received() => match signal {
            Ok(signal) => Err(anyhow::anyhow!(
                "heartbeat recovery interrupted by {signal}; watchdog remains armed"
            )),
            Err(error) => Err(error.context("heartbeat recovery signal monitor failed")),
        },
        result = backend.recover_heartbeat_lost(&requested_nodes) => result,
    };

    // Disarming 0x1016 is permitted only after the recovery operation has
    // established a new, post-command all-axis disabled confirmation. Any
    // interrupted or failed operation deliberately leaves the watchdog armed.
    operation?;
    let disarm = backend.disarm_heartbeat_consumers_with_retry(3).await;
    combine_operation_and_cleanup("heartbeat-lost recovery", Ok(()), disarm)?;
    tracing::info!(
        nodes = ?requested_nodes,
        "heartbeat-lost recovery confirmed; all six drives are fault-free/disabled and host consumers are disarmed"
    );
    Ok(())
}

async fn run_guarded_commissioning_operation<Operation, Shutdown, Disable, DisableFuture>(
    axis_name: &str,
    operation: Operation,
    shutdown: Shutdown,
    disable: Disable,
) -> Result<()>
where
    Operation: Future<Output = Result<()>>,
    Shutdown: Future<Output = Result<TerminationSignal>>,
    Disable: FnOnce() -> DisableFuture,
    DisableFuture: Future<Output = Result<()>>,
{
    let operation_result = tokio::select! {
        biased;
        signal = shutdown => match signal {
            Ok(signal) => Err(anyhow::anyhow!(
                "commissioning interrupted by {signal}; requesting confirmed all-axis disable"
            )),
            Err(error) => Err(error.context("commissioning termination signal monitor failed")),
        },
        result = operation => result,
    };
    // This call is outside the cancelled operation future.  Initialization,
    // configuration, enable, motion, signal, and ordinary error paths all
    // therefore converge here.
    let shutdown_result = disable().await;
    combine_commissioning_results(axis_name, operation_result, shutdown_result)
}

fn combine_commissioning_results(
    axis_name: &str,
    operation: Result<()>,
    disable: Result<()>,
) -> Result<()> {
    match (operation, disable) {
        (Ok(()), Ok(())) => {
            tracing::info!(joint = axis_name, "commissioning completed and all drives are confirmed disabled");
            Ok(())
        }
        (Err(operation_error), Ok(())) => Err(operation_error),
        (Ok(()), Err(disable_error)) => Err(disable_error).context(
            "commissioning motion completed but final disable/heartbeat-disarm was not confirmed",
        ),
        (Err(operation_error), Err(disable_error)) => Err(anyhow::anyhow!(
            "commissioning failed: {operation_error:#}; final confirmed disable/heartbeat-disarm also failed: {disable_error:#}"
        )),
    }
}

async fn run_hardware_commissioning(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    anyhow::ensure!(
        arguments.allow_motion,
        "single-axis commissioning requires the explicit --allow-motion acknowledgement"
    );
    let axis_name = arguments
        .commission_axis
        .as_deref()
        .context("--commission-axis is required")?;
    let selected_index = parse_commission_axis(axis_name)?;
    let request = CommissioningRequest {
        selected_index,
        delta_rad: arguments.delta_rad.context("--delta-rad is required")?,
        duration_sec: arguments
            .duration_sec
            .context("--duration-sec is required")?,
    };
    request.validate(&profile)?;

    tracing::warn!(
        joint = axis_name,
        node_id = profile.joints[selected_index].node_id,
        calibrated = profile.calibrated,
        "entering isolated single-axis commissioning; no ROS or Zenoh session will be opened"
    );
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strictly preflighted real CAN backend for commissioning")?;
    // Installing the Unix streams changes SIGINT/SIGTERM handling immediately;
    // this happens before initialize_disabled can send its first CAN command.
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install commissioning termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_single_axis_commissioning(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown(),
    )
    .await
}

fn diagnostic_request(arguments: &Arguments) -> Result<SingleAxisDiagnosticRequest> {
    let axis_name = arguments
        .diagnose_axis
        .as_deref()
        .context("--diagnose-axis is required")?;
    let selected_index = parse_commission_axis(axis_name)?;
    let cli_mode = arguments
        .diagnostic_mode
        .context("--diagnostic-mode is required")?;
    anyhow::ensure!(
        matches!(cli_mode, DiagnosticCliMode::GravityHoldCensored)
            || !arguments.acknowledge_censored_gravity_hold,
        "--acknowledge-censored-gravity-hold is valid only with --diagnostic-mode gravity-hold-censored"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position,
        "--acknowledge-joint1-first-position cannot authorize a joint_2 diagnostic"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_torque_censored,
        "--acknowledge-joint1-torque-censored cannot authorize a joint_2 diagnostic"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_negative_torque_censored,
        "--acknowledge-joint1-negative-torque-censored cannot authorize a joint_2 diagnostic"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint4_assisted_position,
        "--acknowledge-joint4-assisted-position cannot authorize a joint_2 diagnostic"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint3_assisted_position,
        "--acknowledge-joint3-assisted-position cannot authorize a joint_2 diagnostic"
    );
    let mode = match cli_mode {
        DiagnosticCliMode::Hold => {
            anyhow::ensure!(
                arguments.diagnostic_peak_tau_ff_nm.is_none()
                    && arguments.diagnostic_step_tau_ff_nm.is_none()
                    && arguments.diagnostic_dwell_sec.is_none()
                    && arguments.diagnostic_delta_rad.is_none()
                    && arguments.diagnostic_duration_sec.is_none(),
                "hold mode does not accept staircase or position-round-trip arguments"
            );
            SingleAxisDiagnosticMode::Hold {
                duration_sec: arguments
                    .diagnostic_hold_sec
                    .context("hold mode requires --diagnostic-hold-sec")?,
            }
        }
        DiagnosticCliMode::TauFfStaircase => {
            anyhow::ensure!(
                arguments.diagnostic_hold_sec.is_none()
                    && arguments.diagnostic_delta_rad.is_none()
                    && arguments.diagnostic_duration_sec.is_none(),
                "tau-ff-staircase mode does not accept hold or position-round-trip arguments"
            );
            SingleAxisDiagnosticMode::TauFfStaircase {
                peak_additive_torque_nm: arguments
                    .diagnostic_peak_tau_ff_nm
                    .context("tau-ff-staircase requires --diagnostic-peak-tau-ff-nm")?,
                step_torque_nm: arguments
                    .diagnostic_step_tau_ff_nm
                    .context("tau-ff-staircase requires --diagnostic-step-tau-ff-nm")?,
                dwell_sec: arguments
                    .diagnostic_dwell_sec
                    .context("tau-ff-staircase requires --diagnostic-dwell-sec")?,
            }
        }
        DiagnosticCliMode::PositionRoundTrip => {
            anyhow::ensure!(
                arguments.diagnostic_hold_sec.is_none()
                    && arguments.diagnostic_peak_tau_ff_nm.is_none()
                    && arguments.diagnostic_step_tau_ff_nm.is_none()
                    && arguments.diagnostic_dwell_sec.is_none(),
                "position-round-trip mode does not accept hold or torque-staircase arguments"
            );
            SingleAxisDiagnosticMode::PositionRoundTrip {
                delta_rad: arguments
                    .diagnostic_delta_rad
                    .context("position-round-trip requires --diagnostic-delta-rad 0.005")?,
                duration_sec: arguments
                    .diagnostic_duration_sec
                    .context("position-round-trip requires --diagnostic-duration-sec 4.0")?,
            }
        }
        DiagnosticCliMode::GravityHoldCensored => {
            anyhow::ensure!(
                arguments.diagnostic_hold_sec.is_none()
                    && arguments.diagnostic_peak_tau_ff_nm.is_none()
                    && arguments.diagnostic_step_tau_ff_nm.is_none()
                    && arguments.diagnostic_dwell_sec.is_none()
                    && arguments.diagnostic_delta_rad.is_none()
                    && arguments.diagnostic_duration_sec.is_none(),
                "gravity-hold-censored uses only compile-time-fixed parameters and does not accept hold, staircase, delta, or duration arguments"
            );
            anyhow::ensure!(
                arguments.allow_high_torque_diagnostic,
                "gravity-hold-censored requires --allow-high-torque-diagnostic"
            );
            anyhow::ensure!(
                arguments.acknowledge_censored_gravity_hold,
                "gravity-hold-censored requires --acknowledge-censored-gravity-hold"
            );
            SingleAxisDiagnosticMode::GravityHoldCensored
        }
        DiagnosticCliMode::Joint1FirstPosition => {
            anyhow::bail!("joint1-first-position uses its independent fixed-policy request path")
        }
        DiagnosticCliMode::Joint1TorqueCensored => {
            anyhow::bail!("joint1-torque-censored uses its independent fixed-policy request path")
        }
        DiagnosticCliMode::Joint1NegativeTorqueCensored => anyhow::bail!(
            "joint1-negative-torque-censored uses its independent fixed-policy request path"
        ),
        DiagnosticCliMode::Joint3GravityUnload => {
            anyhow::bail!("joint3-gravity-unload uses its independent fixed-policy request path")
        }
        DiagnosticCliMode::Joint3AssistedPosition => {
            anyhow::bail!("joint3-assisted-position uses its independent fixed-policy request path")
        }
        DiagnosticCliMode::Joint4FirstPosition => {
            anyhow::bail!("joint4-first-position uses its independent fixed-policy request path")
        }
        DiagnosticCliMode::Joint4TorqueCensored => {
            anyhow::bail!("joint4-torque-censored uses its independent fixed-policy request path")
        }
        DiagnosticCliMode::Joint4AssistedPosition => {
            anyhow::bail!("joint4-assisted-position uses its independent fixed-policy request path")
        }
        DiagnosticCliMode::Joint5FirstPosition => {
            anyhow::bail!("joint5-first-position uses its independent fixed-policy request path")
        }
        DiagnosticCliMode::Joint6FirstPosition => {
            anyhow::bail!("joint6-first-position uses its independent fixed-policy request path")
        }
    };
    Ok(SingleAxisDiagnosticRequest {
        selected_index,
        mode,
        high_torque_authorized: arguments.allow_high_torque_diagnostic,
        censored_gravity_hold_authorized: arguments.acknowledge_censored_gravity_hold,
    })
}

fn joint1_first_position_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint1FirstPositionDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint1FirstPosition)
        ),
        "joint_1 fixed request requires --diagnostic-mode joint1-first-position"
    );
    anyhow::ensure!(
        selected_index == 0,
        "joint1-first-position is restricted to --diagnose-axis joint_1"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint1_first_position,
        "joint1-first-position requires --acknowledge-joint1-first-position"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint1-first-position uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_torque_censored
            && !arguments.acknowledge_joint1_negative_torque_censored
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint1-first-position is strict-zero CAN only and rejects the J1 torque mode plus every J2 high-tier, censored, or historical-xstats authorization"
    );
    Ok(Joint1FirstPositionDiagnosticRequest { authorized: true })
}

fn joint1_censored_torque_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint1CensoredTorqueDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint1TorqueCensored)
        ),
        "joint_1 fixed torque request requires --diagnostic-mode joint1-torque-censored"
    );
    anyhow::ensure!(
        selected_index == 0,
        "joint1-torque-censored is restricted to --diagnose-axis joint_1"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint1_torque_censored,
        "joint1-torque-censored requires --acknowledge-joint1-torque-censored"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint1-torque-censored uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position
            && !arguments.acknowledge_joint1_negative_torque_censored
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint1-torque-censored is strict-zero CAN only and rejects every other motion-mode or historical-xstats authorization"
    );
    Ok(Joint1CensoredTorqueDiagnosticRequest { authorized: true })
}

fn joint1_negative_censored_torque_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint1NegativeCensoredTorqueDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint1NegativeTorqueCensored)
        ),
        "joint_1 negative fixed torque request requires --diagnostic-mode joint1-negative-torque-censored"
    );
    anyhow::ensure!(
        selected_index == 0,
        "joint1-negative-torque-censored is restricted to --diagnose-axis joint_1"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint1_negative_torque_censored,
        "joint1-negative-torque-censored requires --acknowledge-joint1-negative-torque-censored"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint1-negative-torque-censored uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position
            && !arguments.acknowledge_joint1_torque_censored
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint1-negative-torque-censored is strict-zero CAN only and rejects every other motion-mode or historical-xstats authorization"
    );
    Ok(Joint1NegativeCensoredTorqueDiagnosticRequest { authorized: true })
}

fn joint5_first_position_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint5FirstPositionDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint5FirstPosition)
        ),
        "joint_5 fixed request requires --diagnostic-mode joint5-first-position"
    );
    anyhow::ensure!(
        selected_index == 4,
        "joint5-first-position is restricted to --diagnose-axis joint_5"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint5_first_position,
        "joint5-first-position requires --acknowledge-joint5-first-position"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint5-first-position uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position
            && !arguments.acknowledge_joint1_torque_censored
            && !arguments.acknowledge_joint1_negative_torque_censored
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint5-first-position is strict-zero CAN only and rejects every other diagnostic authorization"
    );
    Ok(Joint5FirstPositionDiagnosticRequest { authorized: true })
}

fn joint3_gravity_unload_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint3GravityUnloadDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint3GravityUnload)
        ),
        "joint_3 unload request requires --diagnostic-mode joint3-gravity-unload"
    );
    anyhow::ensure!(
        selected_index == 2,
        "joint3-gravity-unload is restricted to --diagnose-axis joint_3"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint3_gravity_unload,
        "joint3-gravity-unload requires --acknowledge-joint3-gravity-unload"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint3-gravity-unload uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position
            && !arguments.acknowledge_joint1_torque_censored
            && !arguments.acknowledge_joint1_negative_torque_censored
            && !arguments.acknowledge_joint4_first_position
            && !arguments.acknowledge_joint4_torque_censored
            && !arguments.acknowledge_joint4_assisted_position
            && !arguments.acknowledge_joint3_assisted_position
            && !arguments.acknowledge_joint5_first_position
            && !arguments.acknowledge_joint6_first_position
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint3-gravity-unload is strict-zero CAN only and rejects every other diagnostic authorization"
    );
    Ok(Joint3GravityUnloadDiagnosticRequest { authorized: true })
}

fn joint3_assisted_position_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint3AssistedPositionDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint3AssistedPosition)
        ),
        "joint_3 assisted-position request requires --diagnostic-mode joint3-assisted-position"
    );
    anyhow::ensure!(
        selected_index == 2,
        "joint3-assisted-position is restricted to --diagnose-axis joint_3"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint3_assisted_position,
        "joint3-assisted-position requires --acknowledge-joint3-assisted-position"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint3-assisted-position uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position
            && !arguments.acknowledge_joint1_torque_censored
            && !arguments.acknowledge_joint1_negative_torque_censored
            && !arguments.acknowledge_joint3_gravity_unload
            && !arguments.acknowledge_joint4_first_position
            && !arguments.acknowledge_joint4_torque_censored
            && !arguments.acknowledge_joint4_assisted_position
            && !arguments.acknowledge_joint5_first_position
            && !arguments.acknowledge_joint6_first_position
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint3-assisted-position is strict-zero CAN only and rejects every other diagnostic authorization"
    );
    Ok(Joint3AssistedPositionDiagnosticRequest { authorized: true })
}

fn joint4_first_position_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint4FirstPositionDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint4FirstPosition)
        ),
        "joint_4 fixed request requires --diagnostic-mode joint4-first-position"
    );
    anyhow::ensure!(
        selected_index == 3,
        "joint4-first-position is restricted to --diagnose-axis joint_4"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint4_first_position,
        "joint4-first-position requires --acknowledge-joint4-first-position"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint4-first-position uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position
            && !arguments.acknowledge_joint1_torque_censored
            && !arguments.acknowledge_joint1_negative_torque_censored
            && !arguments.acknowledge_joint5_first_position
            && !arguments.acknowledge_joint6_first_position
            && !arguments.acknowledge_joint4_torque_censored
            && !arguments.acknowledge_joint4_assisted_position
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint4-first-position is strict-zero CAN only and rejects every other diagnostic authorization"
    );
    Ok(Joint4FirstPositionDiagnosticRequest { authorized: true })
}

fn joint4_censored_torque_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint4CensoredTorqueDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint4TorqueCensored)
        ),
        "joint_4 torque request requires --diagnostic-mode joint4-torque-censored"
    );
    anyhow::ensure!(
        selected_index == 3,
        "joint4-torque-censored is restricted to --diagnose-axis joint_4"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint4_torque_censored,
        "joint4-torque-censored requires --acknowledge-joint4-torque-censored"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint4-torque-censored uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position
            && !arguments.acknowledge_joint1_torque_censored
            && !arguments.acknowledge_joint1_negative_torque_censored
            && !arguments.acknowledge_joint4_first_position
            && !arguments.acknowledge_joint5_first_position
            && !arguments.acknowledge_joint6_first_position
            && !arguments.acknowledge_joint4_assisted_position
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint4-torque-censored is strict-zero CAN only and rejects every other diagnostic authorization"
    );
    Ok(Joint4CensoredTorqueDiagnosticRequest { authorized: true })
}

fn joint4_assisted_position_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint4AssistedPositionDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint4AssistedPosition)
        ),
        "joint_4 assisted request requires --diagnostic-mode joint4-assisted-position"
    );
    anyhow::ensure!(
        selected_index == 3,
        "joint4-assisted-position is restricted to --diagnose-axis joint_4"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint4_assisted_position,
        "joint4-assisted-position requires --acknowledge-joint4-assisted-position"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint4-assisted-position uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position
            && !arguments.acknowledge_joint1_torque_censored
            && !arguments.acknowledge_joint1_negative_torque_censored
            && !arguments.acknowledge_joint4_first_position
            && !arguments.acknowledge_joint4_torque_censored
            && !arguments.acknowledge_joint5_first_position
            && !arguments.acknowledge_joint6_first_position
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint4-assisted-position is strict-zero CAN only and rejects every other diagnostic authorization"
    );
    Ok(Joint4AssistedPositionDiagnosticRequest { authorized: true })
}

fn joint6_first_position_diagnostic_request(
    arguments: &Arguments,
) -> Result<Joint6FirstPositionDiagnosticRequest> {
    let selected_index = parse_commission_axis(
        arguments
            .diagnose_axis
            .as_deref()
            .context("--diagnose-axis is required")?,
    )?;
    anyhow::ensure!(
        matches!(
            arguments.diagnostic_mode,
            Some(DiagnosticCliMode::Joint6FirstPosition)
        ),
        "joint_6 fixed request requires --diagnostic-mode joint6-first-position"
    );
    anyhow::ensure!(
        selected_index == 5,
        "joint6-first-position is restricted to --diagnose-axis joint_6"
    );
    anyhow::ensure!(
        arguments.acknowledge_joint6_first_position,
        "joint6-first-position requires --acknowledge-joint6-first-position"
    );
    anyhow::ensure!(
        arguments.diagnostic_hold_sec.is_none()
            && arguments.diagnostic_peak_tau_ff_nm.is_none()
            && arguments.diagnostic_step_tau_ff_nm.is_none()
            && arguments.diagnostic_dwell_sec.is_none()
            && arguments.diagnostic_delta_rad.is_none()
            && arguments.diagnostic_duration_sec.is_none(),
        "joint6-first-position uses only compile-time-fixed parameters and accepts no numeric diagnostic arguments"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint1_first_position
            && !arguments.acknowledge_joint1_torque_censored
            && !arguments.acknowledge_joint1_negative_torque_censored
            && !arguments.acknowledge_joint5_first_position
            && !arguments.allow_high_torque_diagnostic
            && !arguments.acknowledge_censored_gravity_hold
            && arguments.diagnostic_can_error_warning_baseline.is_none()
            && arguments.diagnostic_can_error_passive_baseline.is_none()
            && !arguments.acknowledge_historical_can_xstats,
        "joint6-first-position is strict-zero CAN only and rejects every other diagnostic authorization"
    );
    Ok(Joint6FirstPositionDiagnosticRequest { authorized: true })
}

fn diagnostic_historical_can_xstats_acknowledgement(
    arguments: &Arguments,
    request: &SingleAxisDiagnosticRequest,
    transport: BusTransport,
) -> Result<Option<HistoricalCanXStatsAcknowledgement>> {
    let values = match (
        arguments.diagnostic_can_error_warning_baseline,
        arguments.diagnostic_can_error_passive_baseline,
        arguments.acknowledge_historical_can_xstats,
    ) {
        (None, None, false) => return Ok(None),
        (Some(error_warning), Some(error_passive), true) => (error_warning, error_passive),
        _ => anyhow::bail!(
            "historical CAN xstats require both exact warning/passive baselines and --acknowledge-historical-can-xstats"
        ),
    };
    anyhow::ensure!(
        request.selected_index == 1
            && request.high_torque_authorized
            && request.uses_high_torque_tier(),
        "historical CAN xstats acknowledgement is restricted to an explicitly authorized joint_2 high-tier diagnostic"
    );
    anyhow::ensure!(
        transport == BusTransport::SocketCan,
        "historical CAN xstats acknowledgement is valid only with a SocketCAN hardware profile"
    );
    Ok(Some(HistoricalCanXStatsAcknowledgement::new(
        values.0, values.1,
    )))
}

fn require_fixed_high_tier_diagnostic_historical_xstats(
    request: &SingleAxisDiagnosticRequest,
    acknowledgement: Option<HistoricalCanXStatsAcknowledgement>,
) -> Result<()> {
    if matches!(
        request.mode,
        SingleAxisDiagnosticMode::PositionRoundTrip { .. }
            | SingleAxisDiagnosticMode::GravityHoldCensored
    ) {
        anyhow::ensure!(
            acknowledgement.is_some(),
            "joint_2 fixed high-tier diagnostics require both exact historical CAN xstats baselines and --acknowledge-historical-can-xstats"
        );
    }
    Ok(())
}

async fn run_hardware_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    anyhow::ensure!(
        arguments.allow_diagnostic_motion,
        "single-axis diagnostics require --allow-diagnostic-motion"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint3_gravity_unload
            || matches!(
                arguments.diagnostic_mode,
                Some(DiagnosticCliMode::Joint3GravityUnload)
            ),
        "--acknowledge-joint3-gravity-unload cannot authorize any other diagnostic mode"
    );
    anyhow::ensure!(
        !arguments.acknowledge_joint3_assisted_position
            || matches!(
                arguments.diagnostic_mode,
                Some(DiagnosticCliMode::Joint3AssistedPosition)
            ),
        "--acknowledge-joint3-assisted-position cannot authorize any other diagnostic mode"
    );
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint1FirstPosition)
    ) {
        return run_hardware_joint1_first_position_diagnostic(arguments, profile, dynamics).await;
    }
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint1TorqueCensored)
    ) {
        return run_hardware_joint1_censored_torque_diagnostic(arguments, profile, dynamics).await;
    }
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint1NegativeTorqueCensored)
    ) {
        return run_hardware_joint1_negative_censored_torque_diagnostic(
            arguments, profile, dynamics,
        )
        .await;
    }
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint3GravityUnload)
    ) {
        return run_hardware_joint3_gravity_unload_diagnostic(arguments, profile, dynamics).await;
    }
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint3AssistedPosition)
    ) {
        return run_hardware_joint3_assisted_position_diagnostic(arguments, profile, dynamics)
            .await;
    }
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint4FirstPosition)
    ) {
        return run_hardware_joint4_first_position_diagnostic(arguments, profile, dynamics).await;
    }
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint4TorqueCensored)
    ) {
        return run_hardware_joint4_censored_torque_diagnostic(arguments, profile, dynamics).await;
    }
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint4AssistedPosition)
    ) {
        return run_hardware_joint4_assisted_position_diagnostic(arguments, profile, dynamics)
            .await;
    }
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint5FirstPosition)
    ) {
        return run_hardware_joint5_first_position_diagnostic(arguments, profile, dynamics).await;
    }
    if matches!(
        arguments.diagnostic_mode,
        Some(DiagnosticCliMode::Joint6FirstPosition)
    ) {
        return run_hardware_joint6_first_position_diagnostic(arguments, profile, dynamics).await;
    }
    let request = diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let historical_xstats_acknowledgement = diagnostic_historical_can_xstats_acknowledgement(
        arguments,
        &request,
        profile.bus.transport,
    )?;
    require_fixed_high_tier_diagnostic_historical_xstats(
        &request,
        historical_xstats_acknowledgement,
    )?;
    let axis_name = &profile.joints[request.selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[request.selected_index].node_id,
        mode = ?request.mode,
        high_torque_authorized = request.high_torque_authorized,
        "entering bounded single-axis diagnostics; no ROS or Zenoh session will be opened"
    );
    let backend = RealBackend::open_for_single_axis_diagnostic(
        profile.clone(),
        historical_xstats_acknowledgement,
    )
    .await
    .context("open strictly preflighted real CAN backend for joint_2 diagnostics")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install diagnostic termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_single_axis_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(request.selected_index),
    )
    .await
}

async fn run_hardware_joint1_first_position_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint1_first_position_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_delta_rad = 0.005_f32,
        fixed_duration_sec = 4.0_f32,
        "entering strict-zero fixed joint_1 first-position diagnostic; no ROS or Zenoh session will be opened"
    );
    // Ordinary open has no historical-xstats escape hatch. The J1 request
    // rejected all J2 acknowledgements before this strict preflight runs.
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strict-zero real CAN backend for joint_1 first-position diagnostic")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_1 diagnostic termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint1_first_position_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

async fn run_hardware_joint1_censored_torque_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint1_censored_torque_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_position = true,
        torque_step_nm = 0.025_f32,
        torque_cap_nm = 0.60_f32,
        displacement_censor_rad = 0.00030_f32,
        "entering strict-zero fixed-position joint_1 censored-torque diagnostic; no position trajectory, ROS, or Zenoh session will be opened"
    );
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strict-zero real CAN backend for joint_1 censored-torque diagnostic")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_1 censored-torque termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint1_censored_torque_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

async fn run_hardware_joint1_negative_censored_torque_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint1_negative_censored_torque_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_position = true,
        torque_direction = "negative",
        torque_step_nm = 0.025_f32,
        torque_cap_nm = 0.60_f32,
        displacement_censor_rad = -0.00030_f32,
        "entering strict-zero fixed-position joint_1 negative censored-torque diagnostic; no position trajectory, ROS, or Zenoh session will be opened"
    );
    let backend = RealBackend::open(profile.clone()).await.context(
        "open strict-zero real CAN backend for joint_1 negative censored-torque diagnostic",
    )?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_1 negative censored-torque termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint1_negative_censored_torque_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

async fn run_hardware_joint5_first_position_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint5_first_position_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_delta_rad = -0.005_f32,
        fixed_duration_sec = 4.0_f32,
        "entering strict-zero fixed joint_5 first-position diagnostic; no ROS or Zenoh session will be opened"
    );
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strict-zero real CAN backend for joint_5 first-position diagnostic")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_5 diagnostic termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint5_first_position_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

async fn run_hardware_joint3_gravity_unload_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint3_gravity_unload_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_position = true,
        torque_step_nm = -0.05_f32,
        torque_cap_nm = -0.75_f32,
        displacement_censor_rad = -0.00030_f32,
        velocity_censor_rad_s = -0.003_f32,
        "entering strict-zero joint_3 gravity-unload diagnostic; no position trajectory, ROS, or Zenoh session will be opened"
    );
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strict-zero real CAN backend for joint_3 gravity-unload diagnostic")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_3 gravity-unload termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint3_gravity_unload_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

async fn run_hardware_joint3_assisted_position_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint3_assisted_position_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_delta_rad = -0.005_f32,
        fixed_duration_sec = 4.0_f32,
        fixed_peak_assistance_nm = -0.25_f32,
        "entering strict-zero joint_3 trajectory-synchronous assisted position diagnostic"
    );
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strict-zero real CAN backend for joint_3 assisted-position diagnostic")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_3 assisted-position termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint3_assisted_position_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

async fn run_hardware_joint4_first_position_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint4_first_position_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_delta_rad = -0.005_f32,
        fixed_duration_sec = 4.0_f32,
        "entering strict-zero fixed joint_4 position-channel diagnostic; gravity feed-forward remains zero"
    );
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strict-zero real CAN backend for joint_4 first-position diagnostic")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_4 diagnostic termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint4_first_position_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

async fn run_hardware_joint4_censored_torque_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint4_censored_torque_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_torque_step_nm = 0.025_f32,
        fixed_torque_cap_nm = -0.45_f32,
        "entering strict-zero fixed-position joint_4 negative-torque identification; no position trajectory will be constructed"
    );
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strict-zero real CAN backend for joint_4 censored-torque diagnostic")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_4 torque diagnostic termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint4_censored_torque_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

async fn run_hardware_joint4_assisted_position_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint4_assisted_position_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_delta_rad = -0.005_f32,
        fixed_duration_sec = 4.0_f32,
        fixed_peak_assistance_nm = -0.30_f32,
        "entering strict-zero joint_4 trajectory-synchronous assisted position diagnostic"
    );
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strict-zero real CAN backend for joint_4 assisted-position diagnostic")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_4 assisted-position termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint4_assisted_position_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

async fn run_hardware_joint6_first_position_diagnostic(
    arguments: &Arguments,
    profile: Arc<HardwareProfile>,
    dynamics: hex_arm_dynamics::ArmDynamics,
) -> Result<()> {
    let request = joint6_first_position_diagnostic_request(arguments)?;
    request.validate(&profile)?;
    let selected_index = request.selected_index();
    let axis_name = &profile.joints[selected_index].name;
    tracing::warn!(
        joint = %axis_name,
        node_id = profile.joints[selected_index].node_id,
        fixed_delta_rad = -0.005_f32,
        fixed_duration_sec = 4.0_f32,
        "entering strict-zero fixed joint_6 first-position diagnostic; no ROS or Zenoh session will be opened"
    );
    let backend = RealBackend::open(profile.clone())
        .await
        .context("open strict-zero real CAN backend for joint_6 first-position diagnostic")?;
    let mut termination_signals = match TerminationSignals::install() {
        Ok(signals) => signals,
        Err(error) => {
            let disable = backend.shutdown().await;
            return combine_commissioning_results(
                axis_name,
                Err(error.context("install joint_6 diagnostic termination guard")),
                disable,
            );
        }
    };
    let operation = async {
        backend
            .initialize_disabled()
            .await
            .context("verify identities and initialize all six drives disabled")?;
        run_joint6_first_position_diagnostic(&backend, &profile, &dynamics, request).await
    };
    run_guarded_commissioning_operation(
        axis_name,
        operation,
        termination_signals.received(),
        || backend.shutdown_single_axis_diagnostic_selected_first(selected_index),
    )
    .await
}

fn parse_commission_axis(value: &str) -> Result<usize> {
    match value {
        "joint_1" => Ok(0),
        "joint_2" => Ok(1),
        "joint_3" => Ok(2),
        "joint_4" => Ok(3),
        "joint_5" => Ok(4),
        "joint_6" => Ok(5),
        _ => anyhow::bail!("--commission-axis must be exactly joint_1..joint_6"),
    }
}

fn build_zenoh_config(arguments: &Arguments) -> Result<zenoh::Config> {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("mode", "\"peer\"")
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;

    let connect = arguments.zenoh_connect.trim();
    let listen = arguments.zenoh_listen.trim();
    if !connect.is_empty() {
        config
            .insert_json5(
                "connect/endpoints",
                &serde_json::to_string(&[connect]).context("encode Zenoh connect endpoint")?,
            )
            .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    }
    if !listen.is_empty() {
        config
            .insert_json5(
                "listen/endpoints",
                &serde_json::to_string(&[listen]).context("encode Zenoh listen endpoint")?,
            )
            .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    }
    if !connect.is_empty() || !listen.is_empty() {
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    }
    Ok(config)
}

async fn run_discovery(arguments: &Arguments) -> Result<()> {
    anyhow::ensure!(
        arguments.timeout_sec.is_finite() && arguments.timeout_sec > 0.0,
        "--timeout must be finite and positive"
    );
    anyhow::ensure!(
        arguments.sdo_timeout_sec.is_finite() && arguments.sdo_timeout_sec > 0.0,
        "--sdo-timeout-sec must be finite and positive"
    );
    anyhow::ensure!(
        !arguments.interface.trim().is_empty(),
        "--interface must not be empty"
    );
    let expected_nodes = if arguments.expected_nodes.is_empty() {
        (1..=6).collect()
    } else {
        arguments.expected_nodes.iter().copied().collect()
    };
    let auxiliary_nodes = arguments.auxiliary_nodes.iter().copied().collect();
    let bus: Arc<dyn CanBus> = match arguments.transport {
        DiscoveryTransport::SocketCan => Arc::new(
            SocketCanBus::open(&arguments.interface)
                .with_context(|| format!("open SocketCAN interface {}", arguments.interface))?,
        ),
    };

    println!(
        "discover-only transport=socket-can interface={} (heartbeat listen + SDO uploads only)",
        arguments.interface
    );
    let report = discover_read_only(
        bus,
        &DiscoveryOptions {
            expected_node_ids: expected_nodes,
            auxiliary_node_ids: auxiliary_nodes,
            observe_timeout: Duration::from_secs_f64(arguments.timeout_sec),
            sdo_timeout: Duration::from_secs_f64(arguments.sdo_timeout_sec),
        },
    )
    .await?;

    for node in &report.nodes {
        if let Some(identity) = &node.identity {
            println!(
                "node={} role={} nmt={:?} vendor=0x{:08x} product=0x{:08x} revision=0x{:08x} serial=0x{:08x} model={}",
                node.node_id,
                node.role,
                node.nmt_state,
                identity.vendor_id,
                identity.product_code,
                identity.revision_number,
                identity.serial_number,
                identity.product_name.as_deref().unwrap_or("-")
            );
        } else {
            println!(
                "node={} role={} nmt={:?} identity_error={}",
                node.node_id,
                node.role,
                node.nmt_state,
                node.identity_error.as_deref().unwrap_or("unknown")
            );
        }
    }
    if !report.missing_expected.is_empty() {
        println!("missing_expected={:?}", report.missing_expected);
    }
    anyhow::ensure!(
        !report.has_failures(),
        "discover-only found missing, unidentified, or unexpected CANopen nodes"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discover_only_cli_requires_no_profile_and_accepts_explicit_allowlist() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--discover-only",
            "--transport",
            "socket-can",
            "--interface",
            "can0",
            "--expected-node",
            "1",
            "--expected-node",
            "6",
            "--auxiliary-node",
            "15",
            "--timeout",
            "3",
        ])
        .unwrap();

        assert!(arguments.discover_only);
        assert!(arguments.profile.is_none());
        assert_eq!(arguments.expected_nodes, [1, 6]);
        assert_eq!(arguments.auxiliary_nodes, [15]);
        assert_eq!(arguments.timeout_sec, 3.0);
    }

    #[test]
    fn profile_validation_cli_is_offline_and_conflicts_with_mock_control() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--validate-profile-only",
        ])
        .unwrap();
        assert!(arguments.validate_profile_only);
        assert!(!arguments.mock);
        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--validate-profile-only",
            "--mock",
        ])
        .is_err());
    }

    #[test]
    fn commissioning_cli_requires_all_explicit_motion_acknowledgements() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--commission-axis",
            "joint_3",
            "--delta-rad",
            "-0.01",
            "--duration-sec",
            "1.0",
            "--allow-motion",
        ])
        .unwrap();
        assert_eq!(arguments.commission_axis.as_deref(), Some("joint_3"));
        assert_eq!(arguments.delta_rad, Some(-0.01));
        assert_eq!(arguments.duration_sec, Some(1.0));
        assert!(arguments.allow_motion);
        assert_eq!(parse_commission_axis("joint_3").unwrap(), 2);

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--commission-axis",
            "joint_3",
            "--delta-rad",
            "0.01",
            "--duration-sec",
            "1.0",
        ])
        .is_err());
        assert!(parse_commission_axis("joint_15").is_err());
    }

    #[test]
    fn commissioning_cli_cannot_start_ros_zenoh_or_mock_mode() {
        let base = [
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--commission-axis",
            "joint_1",
            "--delta-rad",
            "0.01",
            "--duration-sec",
            "1.0",
            "--allow-motion",
        ];
        for conflicting in ["--mock", "--zenoh-listen", "--zenoh-connect"] {
            let mut argv = base.to_vec();
            argv.push(conflicting);
            if conflicting != "--mock" {
                argv.push("tcp/127.0.0.1:7448");
            }
            assert!(Arguments::try_parse_from(argv).is_err());
        }
    }

    #[test]
    fn diagnostic_cli_requires_explicit_j2_mode_parameters_and_acknowledgement() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "tau-ff-staircase",
            "--diagnostic-peak-tau-ff-nm",
            "0.025",
            "--diagnostic-step-tau-ff-nm",
            "0.025",
            "--diagnostic-dwell-sec",
            "0.2",
            "--allow-diagnostic-motion",
        ])
        .unwrap();
        let request = diagnostic_request(&arguments).unwrap();
        assert_eq!(request.selected_index, 1);
        assert!(!request.high_torque_authorized);
        assert_eq!(
            request.mode,
            SingleAxisDiagnosticMode::TauFfStaircase {
                peak_additive_torque_nm: 0.025,
                step_torque_nm: 0.025,
                dwell_sec: 0.2,
            }
        );

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "hold",
            "--diagnostic-hold-sec",
            "0.5",
        ])
        .is_err());
    }

    #[test]
    fn joint1_first_position_cli_has_an_independent_fixed_authorization_path() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_1",
            "--diagnostic-mode",
            "joint1-first-position",
            "--allow-diagnostic-motion",
            "--acknowledge-joint1-first-position",
        ])
        .unwrap();
        let request = joint1_first_position_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 0);
        assert!(diagnostic_request(&arguments).is_err());

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_1",
            "--diagnostic-mode",
            "joint1-first-position",
            "--allow-diagnostic-motion",
        ])
        .is_err());

        let wrong_axis = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "joint1-first-position",
            "--allow-diagnostic-motion",
            "--acknowledge-joint1-first-position",
        ])
        .unwrap();
        assert!(joint1_first_position_diagnostic_request(&wrong_axis).is_err());
    }

    #[test]
    fn joint1_first_position_cli_rejects_numeric_and_j2_authority_crossovers() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_1",
            "--diagnostic-mode",
            "joint1-first-position",
            "--allow-diagnostic-motion",
            "--acknowledge-joint1-first-position",
        ];
        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.25"),
            ("--diagnostic-step-tau-ff-nm", "0.025"),
            ("--diagnostic-dwell-sec", "0.2"),
            ("--diagnostic-delta-rad", "0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            let error = joint1_first_position_diagnostic_request(&arguments)
                .unwrap_err()
                .to_string();
            assert!(error.contains("accepts no numeric diagnostic arguments"));
        }

        for conflicting in [
            vec!["--allow-high-torque-diagnostic"],
            vec![
                "--allow-high-torque-diagnostic",
                "--acknowledge-censored-gravity-hold",
            ],
            vec![
                "--allow-high-torque-diagnostic",
                "--diagnostic-can-error-warning-baseline",
                "10",
                "--diagnostic-can-error-passive-baseline",
                "10",
                "--acknowledge-historical-can-xstats",
            ],
        ] {
            let mut argv = base.clone();
            argv.extend(conflicting);
            assert!(Arguments::try_parse_from(argv).is_err());
        }

        let j2_with_j1_ack = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "hold",
            "--diagnostic-hold-sec",
            "0.5",
            "--allow-diagnostic-motion",
            "--acknowledge-joint1-first-position",
        ])
        .unwrap();
        assert!(diagnostic_request(&j2_with_j1_ack).is_err());
    }

    #[test]
    fn joint1_censored_torque_cli_has_independent_fixed_authorization() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_1",
            "--diagnostic-mode",
            "joint1-torque-censored",
            "--allow-diagnostic-motion",
            "--acknowledge-joint1-torque-censored",
        ])
        .unwrap();
        let request = joint1_censored_torque_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 0);
        assert!(diagnostic_request(&arguments).is_err());

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_1",
            "--diagnostic-mode",
            "joint1-torque-censored",
            "--allow-diagnostic-motion",
        ])
        .is_err());
    }

    #[test]
    fn joint1_negative_censored_torque_cli_has_independent_fixed_authorization() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_1",
            "--diagnostic-mode",
            "joint1-negative-torque-censored",
            "--allow-diagnostic-motion",
            "--acknowledge-joint1-negative-torque-censored",
        ])
        .unwrap();
        let request = joint1_negative_censored_torque_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 0);
        assert!(joint1_censored_torque_diagnostic_request(&arguments).is_err());
        assert!(diagnostic_request(&arguments).is_err());

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_1",
            "--diagnostic-mode",
            "joint1-negative-torque-censored",
            "--allow-diagnostic-motion",
        ])
        .is_err());
    }

    #[test]
    fn joint1_censored_torque_cli_rejects_parameters_axes_and_cross_authority() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_1",
            "--diagnostic-mode",
            "joint1-torque-censored",
            "--allow-diagnostic-motion",
            "--acknowledge-joint1-torque-censored",
        ];
        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.60"),
            ("--diagnostic-step-tau-ff-nm", "0.025"),
            ("--diagnostic-dwell-sec", "0.1"),
            ("--diagnostic-delta-rad", "0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            assert!(joint1_censored_torque_diagnostic_request(&arguments).is_err());
        }

        let mut wrong_axis = base.clone();
        let axis = wrong_axis
            .iter()
            .position(|value| *value == "joint_1")
            .unwrap();
        wrong_axis[axis] = "joint_2";
        let arguments = Arguments::try_parse_from(wrong_axis).unwrap();
        assert!(joint1_censored_torque_diagnostic_request(&arguments).is_err());

        for conflicting in [
            vec!["--acknowledge-joint1-first-position"],
            vec!["--allow-high-torque-diagnostic"],
            vec![
                "--allow-high-torque-diagnostic",
                "--acknowledge-censored-gravity-hold",
            ],
        ] {
            let mut argv = base.clone();
            argv.extend(conflicting);
            assert!(Arguments::try_parse_from(argv).is_err());
        }
    }

    #[test]
    fn joint1_negative_censored_torque_cli_rejects_parameters_axes_and_cross_authority() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_1",
            "--diagnostic-mode",
            "joint1-negative-torque-censored",
            "--allow-diagnostic-motion",
            "--acknowledge-joint1-negative-torque-censored",
        ];
        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.60"),
            ("--diagnostic-step-tau-ff-nm", "0.025"),
            ("--diagnostic-dwell-sec", "0.1"),
            ("--diagnostic-delta-rad", "-0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            assert!(joint1_negative_censored_torque_diagnostic_request(&arguments).is_err());
        }

        let mut wrong_axis = base.clone();
        let axis = wrong_axis
            .iter()
            .position(|value| *value == "joint_1")
            .unwrap();
        wrong_axis[axis] = "joint_2";
        let arguments = Arguments::try_parse_from(wrong_axis).unwrap();
        assert!(joint1_negative_censored_torque_diagnostic_request(&arguments).is_err());

        for conflicting in [
            vec!["--acknowledge-joint1-first-position"],
            vec!["--acknowledge-joint1-torque-censored"],
            vec!["--allow-high-torque-diagnostic"],
        ] {
            let mut argv = base.clone();
            argv.extend(conflicting);
            assert!(Arguments::try_parse_from(argv).is_err());
        }
    }

    #[test]
    fn joint5_first_position_cli_is_fixed_axis_specific_and_independently_authorized() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_5",
            "--diagnostic-mode",
            "joint5-first-position",
            "--allow-diagnostic-motion",
            "--acknowledge-joint5-first-position",
        ];
        let arguments = Arguments::try_parse_from(base.clone()).unwrap();
        let request = joint5_first_position_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 4);
        assert!(diagnostic_request(&arguments).is_err());

        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.1"),
            ("--diagnostic-delta-rad", "-0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            assert!(joint5_first_position_diagnostic_request(&arguments).is_err());
        }

        let mut wrong_axis = base.clone();
        let axis = wrong_axis
            .iter()
            .position(|value| *value == "joint_5")
            .unwrap();
        wrong_axis[axis] = "joint_1";
        let arguments = Arguments::try_parse_from(wrong_axis).unwrap();
        assert!(joint5_first_position_diagnostic_request(&arguments).is_err());

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_5",
            "--diagnostic-mode",
            "joint5-first-position",
            "--allow-diagnostic-motion",
        ])
        .is_err());
    }

    #[test]
    fn joint5_first_position_source_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint5_first_position_diagnostic(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        let request = body
            .find("joint5_first_position_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body.find("run_joint5_first_position_diagnostic").unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
    }

    #[test]
    fn joint3_gravity_unload_cli_is_fixed_strict_and_independently_authorized() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_3",
            "--diagnostic-mode",
            "joint3-gravity-unload",
            "--allow-diagnostic-motion",
            "--acknowledge-joint3-gravity-unload",
        ];
        let arguments = Arguments::try_parse_from(base.clone()).unwrap();
        let request = joint3_gravity_unload_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 2);
        assert!(diagnostic_request(&arguments).is_err());

        let mut wrong_axis = base.clone();
        let axis = wrong_axis
            .iter()
            .position(|value| *value == "joint_3")
            .unwrap();
        wrong_axis[axis] = "joint_4";
        let arguments = Arguments::try_parse_from(wrong_axis).unwrap();
        assert!(joint3_gravity_unload_diagnostic_request(&arguments).is_err());

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_3",
            "--diagnostic-mode",
            "joint3-gravity-unload",
            "--allow-diagnostic-motion",
        ])
        .is_err());
    }

    #[test]
    fn joint3_gravity_unload_source_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint3_gravity_unload_diagnostic(")
            .unwrap()
            .1
            .split_once("async fn run_hardware_joint4_first_position_diagnostic(")
            .unwrap()
            .0;
        let request = body
            .find("joint3_gravity_unload_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body.find("run_joint3_gravity_unload_diagnostic").unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
    }

    #[test]
    fn joint3_assisted_position_cli_is_fixed_strict_and_independently_authorized() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_3",
            "--diagnostic-mode",
            "joint3-assisted-position",
            "--allow-diagnostic-motion",
            "--acknowledge-joint3-assisted-position",
        ];
        let arguments = Arguments::try_parse_from(base.clone()).unwrap();
        let request = joint3_assisted_position_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 2);
        assert!(diagnostic_request(&arguments).is_err());

        let mut wrong_axis = base.clone();
        let axis = wrong_axis
            .iter()
            .position(|value| *value == "joint_3")
            .unwrap();
        wrong_axis[axis] = "joint_4";
        let arguments = Arguments::try_parse_from(wrong_axis).unwrap();
        assert!(joint3_assisted_position_diagnostic_request(&arguments).is_err());

        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.1"),
            ("--diagnostic-delta-rad", "-0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            assert!(joint3_assisted_position_diagnostic_request(&arguments).is_err());
        }

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_3",
            "--diagnostic-mode",
            "joint3-assisted-position",
            "--allow-diagnostic-motion",
        ])
        .is_err());
    }

    #[test]
    fn joint3_assisted_position_source_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint3_assisted_position_diagnostic(")
            .unwrap()
            .1
            .split_once("async fn run_hardware_joint4_first_position_diagnostic(")
            .unwrap()
            .0;
        let request = body
            .find("joint3_assisted_position_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body
            .find("run_joint3_assisted_position_diagnostic")
            .unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
    }

    #[test]
    fn joint4_first_position_cli_is_fixed_axis_specific_and_independently_authorized() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_4",
            "--diagnostic-mode",
            "joint4-first-position",
            "--allow-diagnostic-motion",
            "--acknowledge-joint4-first-position",
        ];
        let arguments = Arguments::try_parse_from(base.clone()).unwrap();
        let request = joint4_first_position_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 3);
        assert!(diagnostic_request(&arguments).is_err());

        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.1"),
            ("--diagnostic-delta-rad", "-0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            assert!(joint4_first_position_diagnostic_request(&arguments).is_err());
        }

        let mut wrong_axis = base.clone();
        let axis = wrong_axis
            .iter()
            .position(|value| *value == "joint_4")
            .unwrap();
        wrong_axis[axis] = "joint_5";
        let arguments = Arguments::try_parse_from(wrong_axis).unwrap();
        assert!(joint4_first_position_diagnostic_request(&arguments).is_err());

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_4",
            "--diagnostic-mode",
            "joint4-first-position",
            "--allow-diagnostic-motion",
        ])
        .is_err());
    }

    #[test]
    fn joint4_first_position_source_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint4_first_position_diagnostic(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        let request = body
            .find("joint4_first_position_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body.find("run_joint4_first_position_diagnostic").unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
    }

    #[test]
    fn joint4_censored_torque_cli_is_fixed_axis_specific_and_independently_authorized() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_4",
            "--diagnostic-mode",
            "joint4-torque-censored",
            "--allow-diagnostic-motion",
            "--acknowledge-joint4-torque-censored",
        ];
        let arguments = Arguments::try_parse_from(base.clone()).unwrap();
        let request = joint4_censored_torque_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 3);
        assert!(joint4_first_position_diagnostic_request(&arguments).is_err());
        assert!(diagnostic_request(&arguments).is_err());

        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.45"),
            ("--diagnostic-delta-rad", "-0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            assert!(joint4_censored_torque_diagnostic_request(&arguments).is_err());
        }

        let mut wrong_axis = base.clone();
        let axis = wrong_axis
            .iter()
            .position(|value| *value == "joint_4")
            .unwrap();
        wrong_axis[axis] = "joint_5";
        let arguments = Arguments::try_parse_from(wrong_axis).unwrap();
        assert!(joint4_censored_torque_diagnostic_request(&arguments).is_err());

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_4",
            "--diagnostic-mode",
            "joint4-torque-censored",
            "--allow-diagnostic-motion",
        ])
        .is_err());
    }

    #[test]
    fn joint4_censored_torque_source_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint4_censored_torque_diagnostic(")
            .unwrap()
            .1
            .split_once("async fn run_hardware_joint6_first_position_diagnostic(")
            .unwrap()
            .0;
        let request = body
            .find("joint4_censored_torque_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body.find("run_joint4_censored_torque_diagnostic").unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
    }

    #[test]
    fn joint4_assisted_position_cli_is_fixed_and_separately_authorized() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_4",
            "--diagnostic-mode",
            "joint4-assisted-position",
            "--allow-diagnostic-motion",
            "--acknowledge-joint4-assisted-position",
        ];
        let arguments = Arguments::try_parse_from(base.clone()).unwrap();
        let request = joint4_assisted_position_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 3);
        assert!(joint4_first_position_diagnostic_request(&arguments).is_err());
        assert!(joint4_censored_torque_diagnostic_request(&arguments).is_err());
        assert!(diagnostic_request(&arguments).is_err());

        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.15"),
            ("--diagnostic-delta-rad", "-0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            assert!(joint4_assisted_position_diagnostic_request(&arguments).is_err());
        }

        let mut wrong_axis = base.clone();
        let axis = wrong_axis
            .iter()
            .position(|value| *value == "joint_4")
            .unwrap();
        wrong_axis[axis] = "joint_5";
        let arguments = Arguments::try_parse_from(wrong_axis).unwrap();
        assert!(joint4_assisted_position_diagnostic_request(&arguments).is_err());

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_4",
            "--diagnostic-mode",
            "joint4-assisted-position",
            "--allow-diagnostic-motion",
        ])
        .is_err());
    }

    #[test]
    fn joint4_assisted_position_source_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint4_assisted_position_diagnostic(")
            .unwrap()
            .1
            .split_once("async fn run_hardware_joint6_first_position_diagnostic(")
            .unwrap()
            .0;
        let request = body
            .find("joint4_assisted_position_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body
            .find("run_joint4_assisted_position_diagnostic")
            .unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
    }

    #[test]
    fn joint6_first_position_cli_is_fixed_axis_specific_and_independently_authorized() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_6",
            "--diagnostic-mode",
            "joint6-first-position",
            "--allow-diagnostic-motion",
            "--acknowledge-joint6-first-position",
        ];
        let arguments = Arguments::try_parse_from(base.clone()).unwrap();
        let request = joint6_first_position_diagnostic_request(&arguments).unwrap();
        assert!(request.authorized);
        assert_eq!(request.selected_index(), 5);
        assert!(diagnostic_request(&arguments).is_err());

        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.1"),
            ("--diagnostic-delta-rad", "-0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            assert!(joint6_first_position_diagnostic_request(&arguments).is_err());
        }

        let mut wrong_axis = base.clone();
        let axis = wrong_axis
            .iter()
            .position(|value| *value == "joint_6")
            .unwrap();
        wrong_axis[axis] = "joint_5";
        let arguments = Arguments::try_parse_from(wrong_axis).unwrap();
        assert!(joint6_first_position_diagnostic_request(&arguments).is_err());

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_6",
            "--diagnostic-mode",
            "joint6-first-position",
            "--allow-diagnostic-motion",
        ])
        .is_err());
    }

    #[test]
    fn joint6_first_position_source_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint6_first_position_diagnostic(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        let request = body
            .find("joint6_first_position_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body.find("run_joint6_first_position_diagnostic").unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
    }

    #[test]
    fn joint1_censored_torque_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint1_censored_torque_diagnostic(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        let request = body
            .find("joint1_censored_torque_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body.find("run_joint1_censored_torque_diagnostic").unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
        assert!(!body.contains("HistoricalCanXStatsAcknowledgement"));
    }

    #[test]
    fn joint1_negative_censored_torque_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint1_negative_censored_torque_diagnostic(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        let request = body
            .find("joint1_negative_censored_torque_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body
            .find("run_joint1_negative_censored_torque_diagnostic")
            .unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
        assert!(!body.contains("HistoricalCanXStatsAcknowledgement"));
    }

    #[test]
    fn joint1_first_position_uses_strict_open_and_selected_first_cleanup() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_joint1_first_position_diagnostic(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        let request = body
            .find("joint1_first_position_diagnostic_request(arguments)")
            .unwrap();
        let validation = body.find("request.validate(&profile)").unwrap();
        let strict_open = body.find("RealBackend::open(profile.clone())").unwrap();
        let initialize = body.find("initialize_disabled()").unwrap();
        let run = body.find("run_joint1_first_position_diagnostic").unwrap();
        let cleanup = body
            .find("shutdown_single_axis_diagnostic_selected_first(selected_index)")
            .unwrap();
        assert!(request < validation);
        assert!(validation < strict_open);
        assert!(strict_open < initialize);
        assert!(initialize < run);
        assert!(run < cleanup);
        assert!(!body.contains("open_for_single_axis_diagnostic"));
        assert!(!body.contains("HistoricalCanXStatsAcknowledgement"));
    }

    #[test]
    fn high_torque_diagnostic_has_an_independent_cli_ack_before_backend_open() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "tau-ff-staircase",
            "--diagnostic-peak-tau-ff-nm",
            "1.5",
            "--diagnostic-step-tau-ff-nm",
            "0.25",
            "--diagnostic-dwell-sec",
            "0.2",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
        ])
        .unwrap();
        let request = diagnostic_request(&arguments).unwrap();
        assert!(request.high_torque_authorized);

        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_diagnostic(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        assert!(
            body.find("request.validate(&profile)").unwrap()
                < body.find("RealBackend::open").unwrap()
        );
        assert!(body.contains("shutdown_single_axis_diagnostic_selected_first"));
    }

    #[test]
    fn position_round_trip_cli_is_fixed_and_requires_exact_history_before_backend_open() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "position-round-trip",
            "--diagnostic-delta-rad",
            "0.005",
            "--diagnostic-duration-sec",
            "4.0",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
            "--diagnostic-can-error-warning-baseline",
            "10",
            "--diagnostic-can-error-passive-baseline",
            "10",
            "--acknowledge-historical-can-xstats",
        ])
        .unwrap();
        let request = diagnostic_request(&arguments).unwrap();
        assert_eq!(
            request.mode,
            SingleAxisDiagnosticMode::PositionRoundTrip {
                delta_rad: 0.005,
                duration_sec: 4.0,
            }
        );
        assert!(request.high_torque_authorized);
        assert!(!request.censored_gravity_hold_authorized);
        let acknowledgement = diagnostic_historical_can_xstats_acknowledgement(
            &arguments,
            &request,
            BusTransport::SocketCan,
        )
        .unwrap();
        assert_eq!(
            acknowledgement,
            Some(HistoricalCanXStatsAcknowledgement::new(10, 10))
        );
        require_fixed_high_tier_diagnostic_historical_xstats(&request, acknowledgement).unwrap();

        let mut missing_history_argv = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "position-round-trip",
            "--diagnostic-delta-rad",
            "0.005",
            "--diagnostic-duration-sec",
            "4.0",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
        ];
        let missing_history = Arguments::try_parse_from(&missing_history_argv).unwrap();
        let missing_history_request = diagnostic_request(&missing_history).unwrap();
        assert!(require_fixed_high_tier_diagnostic_historical_xstats(
            &missing_history_request,
            diagnostic_historical_can_xstats_acknowledgement(
                &missing_history,
                &missing_history_request,
                BusTransport::SocketCan,
            )
            .unwrap(),
        )
        .is_err());

        missing_history_argv.extend([
            "--diagnostic-can-error-warning-baseline",
            "10",
            "--diagnostic-can-error-passive-baseline",
            "10",
            "--acknowledge-historical-can-xstats",
        ]);
        missing_history_argv.retain(|argument| *argument != "--allow-high-torque-diagnostic");
        assert!(Arguments::try_parse_from(missing_history_argv).is_err());

        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_diagnostic(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        let request_validation = body.find("request.validate(&profile)").unwrap();
        let history_required = body
            .find("require_fixed_high_tier_diagnostic_historical_xstats")
            .unwrap();
        let backend_open = body
            .find("RealBackend::open_for_single_axis_diagnostic")
            .unwrap();
        assert!(request_validation < history_required);
        assert!(history_required < backend_open);
    }

    #[test]
    fn censored_gravity_hold_cli_is_fixed_distinct_and_history_gated() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "gravity-hold-censored",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
            "--acknowledge-censored-gravity-hold",
            "--diagnostic-can-error-warning-baseline",
            "10",
            "--diagnostic-can-error-passive-baseline",
            "10",
            "--acknowledge-historical-can-xstats",
        ])
        .unwrap();
        let request = diagnostic_request(&arguments).unwrap();
        assert_eq!(request.selected_index, 1);
        assert_eq!(request.mode, SingleAxisDiagnosticMode::GravityHoldCensored);
        assert!(request.high_torque_authorized);
        assert!(request.censored_gravity_hold_authorized);
        assert!(request.uses_high_torque_tier());

        let acknowledgement = diagnostic_historical_can_xstats_acknowledgement(
            &arguments,
            &request,
            BusTransport::SocketCan,
        )
        .unwrap();
        assert_eq!(
            acknowledgement,
            Some(HistoricalCanXStatsAcknowledgement::new(10, 10))
        );
        require_fixed_high_tier_diagnostic_historical_xstats(&request, acknowledgement).unwrap();
        assert!(diagnostic_historical_can_xstats_acknowledgement(
            &arguments,
            &request,
            BusTransport::GsUsb,
        )
        .is_err());
    }

    #[test]
    fn censored_gravity_hold_requires_its_independent_authorization() {
        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "gravity-hold-censored",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
        ])
        .is_err());
        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "gravity-hold-censored",
            "--allow-diagnostic-motion",
            "--acknowledge-censored-gravity-hold",
        ])
        .is_err());

        // The censored acknowledgement is not a generic high-tier or motion
        // authorization and must not attach to the position trajectory.
        let wrong_mode = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "position-round-trip",
            "--diagnostic-delta-rad",
            "0.005",
            "--diagnostic-duration-sec",
            "4.0",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
            "--acknowledge-censored-gravity-hold",
        ])
        .unwrap();
        assert!(diagnostic_request(&wrong_mode).is_err());
    }

    #[test]
    fn censored_gravity_hold_rejects_every_numeric_cross_mode_argument() {
        let base = vec![
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "gravity-hold-censored",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
            "--acknowledge-censored-gravity-hold",
        ];
        for (option, value) in [
            ("--diagnostic-hold-sec", "0.5"),
            ("--diagnostic-peak-tau-ff-nm", "0.25"),
            ("--diagnostic-step-tau-ff-nm", "0.025"),
            ("--diagnostic-dwell-sec", "0.2"),
            ("--diagnostic-delta-rad", "0.005"),
            ("--diagnostic-duration-sec", "4.0"),
        ] {
            let mut argv = base.clone();
            argv.extend([option, value]);
            let arguments = Arguments::try_parse_from(argv).unwrap();
            let error = diagnostic_request(&arguments).unwrap_err().to_string();
            assert!(error.contains("compile-time-fixed parameters"));
        }
    }

    #[test]
    fn censored_gravity_hold_cannot_bypass_required_historical_xstats() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "gravity-hold-censored",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
            "--acknowledge-censored-gravity-hold",
        ])
        .unwrap();
        let request = diagnostic_request(&arguments).unwrap();
        let acknowledgement = diagnostic_historical_can_xstats_acknowledgement(
            &arguments,
            &request,
            BusTransport::SocketCan,
        )
        .unwrap();
        assert_eq!(acknowledgement, None);
        assert!(
            require_fixed_high_tier_diagnostic_historical_xstats(&request, acknowledgement)
                .is_err()
        );
    }

    #[test]
    fn historical_can_xstats_cli_is_exact_all_or_none_and_high_tier_only() {
        let high_arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "tau-ff-staircase",
            "--diagnostic-peak-tau-ff-nm",
            "1.5",
            "--diagnostic-step-tau-ff-nm",
            "0.25",
            "--diagnostic-dwell-sec",
            "0.2",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
            "--diagnostic-can-error-warning-baseline",
            "10",
            "--diagnostic-can-error-passive-baseline",
            "10",
            "--acknowledge-historical-can-xstats",
        ])
        .unwrap();
        let high_request = diagnostic_request(&high_arguments).unwrap();
        assert_eq!(
            diagnostic_historical_can_xstats_acknowledgement(
                &high_arguments,
                &high_request,
                BusTransport::SocketCan,
            )
            .unwrap(),
            Some(HistoricalCanXStatsAcknowledgement::new(10, 10))
        );
        assert!(diagnostic_historical_can_xstats_acknowledgement(
            &high_arguments,
            &high_request,
            BusTransport::GsUsb,
        )
        .is_err());

        let high_without_history = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "tau-ff-staircase",
            "--diagnostic-peak-tau-ff-nm",
            "1.5",
            "--diagnostic-step-tau-ff-nm",
            "0.25",
            "--diagnostic-dwell-sec",
            "0.2",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
        ])
        .unwrap();
        let high_without_history_request = diagnostic_request(&high_without_history).unwrap();
        assert_eq!(
            diagnostic_historical_can_xstats_acknowledgement(
                &high_without_history,
                &high_without_history_request,
                BusTransport::SocketCan,
            )
            .unwrap(),
            None
        );

        for incomplete in [
            vec![
                "--diagnostic-can-error-warning-baseline",
                "10",
                "--diagnostic-can-error-passive-baseline",
                "10",
            ],
            vec![
                "--diagnostic-can-error-warning-baseline",
                "10",
                "--acknowledge-historical-can-xstats",
            ],
            vec![
                "--diagnostic-can-error-passive-baseline",
                "10",
                "--acknowledge-historical-can-xstats",
            ],
        ] {
            let mut argv = vec![
                "hex_arm_controller",
                "--profile",
                "/tmp/profile.yaml",
                "--diagnose-axis",
                "joint_2",
                "--diagnostic-mode",
                "tau-ff-staircase",
                "--diagnostic-peak-tau-ff-nm",
                "1.5",
                "--diagnostic-step-tau-ff-nm",
                "0.25",
                "--diagnostic-dwell-sec",
                "0.2",
                "--allow-diagnostic-motion",
                "--allow-high-torque-diagnostic",
            ];
            argv.extend(incomplete);
            assert!(Arguments::try_parse_from(argv).is_err());
        }

        let low_arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "tau-ff-staircase",
            "--diagnostic-peak-tau-ff-nm",
            "0.25",
            "--diagnostic-step-tau-ff-nm",
            "0.25",
            "--diagnostic-dwell-sec",
            "0.2",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
            "--diagnostic-can-error-warning-baseline",
            "10",
            "--diagnostic-can-error-passive-baseline",
            "10",
            "--acknowledge-historical-can-xstats",
        ])
        .unwrap();
        let low_request = diagnostic_request(&low_arguments).unwrap();
        assert!(!low_request.uses_high_torque_tier());
        assert!(diagnostic_historical_can_xstats_acknowledgement(
            &low_arguments,
            &low_request,
            BusTransport::SocketCan,
        )
        .is_err());

        let hold_arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "hold",
            "--diagnostic-hold-sec",
            "0.5",
            "--allow-diagnostic-motion",
            "--allow-high-torque-diagnostic",
            "--diagnostic-can-error-warning-baseline",
            "10",
            "--diagnostic-can-error-passive-baseline",
            "10",
            "--acknowledge-historical-can-xstats",
        ])
        .unwrap();
        let hold_request = diagnostic_request(&hold_arguments).unwrap();
        assert!(diagnostic_historical_can_xstats_acknowledgement(
            &hold_arguments,
            &hold_request,
            BusTransport::SocketCan,
        )
        .is_err());

        // Without --diagnose-axis the acknowledgement cannot be attached to
        // normal control, commissioning, or heartbeat recovery CLI paths.
        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnostic-can-error-warning-baseline",
            "10",
            "--diagnostic-can-error-passive-baseline",
            "10",
            "--acknowledge-historical-can-xstats",
            "--allow-high-torque-diagnostic",
        ])
        .is_err());

        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_diagnostic(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        let request_validation = body.find("request.validate(&profile)").unwrap();
        let xstats_validation = body
            .find("diagnostic_historical_can_xstats_acknowledgement")
            .unwrap();
        let backend_open = body
            .find("RealBackend::open_for_single_axis_diagnostic")
            .unwrap();
        assert!(request_validation < xstats_validation);
        assert!(xstats_validation < backend_open);
    }

    #[test]
    fn diagnostic_cli_isolated_from_normal_and_commissioning_modes() {
        let base = [
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--diagnose-axis",
            "joint_2",
            "--diagnostic-mode",
            "hold",
            "--diagnostic-hold-sec",
            "0.5",
            "--allow-diagnostic-motion",
        ];
        for conflicting in ["--mock", "--commission-axis", "--zenoh-listen"] {
            let mut argv = base.to_vec();
            argv.push(conflicting);
            match conflicting {
                "--commission-axis" => {
                    argv.extend(["joint_1", "--delta-rad", "0.01", "--duration-sec", "1.0"]);
                    argv.push("--allow-motion");
                }
                "--zenoh-listen" => argv.push("tcp/127.0.0.1:7448"),
                _ => {}
            }
            assert!(Arguments::try_parse_from(argv).is_err());
        }
    }

    #[test]
    fn heartbeat_recovery_cli_requires_explicit_nodes_ack_and_excludes_aux() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--recover-heartbeat-lost",
            "1,2,3,4,5,6",
            "--allow-heartbeat-fault-reset",
        ])
        .unwrap();
        assert_eq!(
            parse_heartbeat_recovery_nodes(&arguments.recover_heartbeat_lost).unwrap(),
            (1..=6).collect()
        );
        assert!(arguments.allow_heartbeat_fault_reset);

        assert!(Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--recover-heartbeat-lost",
            "1,2,3,4,5,6",
        ])
        .is_err());
        assert!(parse_heartbeat_recovery_nodes(&[15]).is_err());
        assert!(parse_heartbeat_recovery_nodes(&[1, 1]).is_err());
    }

    #[test]
    fn heartbeat_recovery_cli_cannot_start_normal_or_commissioning_modes() {
        let base = [
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--recover-heartbeat-lost",
            "1",
            "--allow-heartbeat-fault-reset",
        ];
        for conflicting in ["--mock", "--discover-only", "--validate-profile-only"] {
            let mut argv = base.to_vec();
            argv.push(conflicting);
            assert!(Arguments::try_parse_from(argv).is_err());
        }

        let mut commissioning = base.to_vec();
        commissioning.extend([
            "--commission-axis",
            "joint_1",
            "--delta-rad",
            "0.01",
            "--duration-sec",
            "1.0",
            "--allow-motion",
        ]);
        assert!(Arguments::try_parse_from(commissioning).is_err());
    }

    #[test]
    fn cleanup_error_is_never_silently_discarded() {
        let error = combine_operation_and_cleanup(
            "controller runtime",
            Ok(()),
            Err(anyhow::anyhow!("0x1016 readback failed")),
        )
        .unwrap_err();
        let error = format!("{error:#}");
        assert!(error.contains("controller runtime cleanup failed"));
        assert!(error.contains("0x1016 readback failed"));
    }

    #[test]
    fn zenoh_listen_cli_builds_a_direct_non_multicast_peer() {
        let arguments = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--zenoh-listen",
            "tcp/127.0.0.1:7448",
        ])
        .unwrap();
        let config = build_zenoh_config(&arguments).unwrap();

        assert_eq!(config.get_json("mode").unwrap(), "\"peer\"");
        assert_eq!(
            config.get_json("listen/endpoints").unwrap(),
            "[\"tcp/127.0.0.1:7448\"]"
        );
        assert_eq!(
            config.get_json("scouting/multicast/enabled").unwrap(),
            "false"
        );
    }

    #[test]
    fn zenoh_connect_is_direct_but_endpoint_free_mock_keeps_scouting_default() {
        let connected = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--zenoh-connect",
            "tcp/router.example:7447",
        ])
        .unwrap();
        let connected_config = build_zenoh_config(&connected).unwrap();
        assert_eq!(
            connected_config.get_json("connect/endpoints").unwrap(),
            "[\"tcp/router.example:7447\"]"
        );
        assert_eq!(
            connected_config
                .get_json("scouting/multicast/enabled")
                .unwrap(),
            "false"
        );

        let default_mock = Arguments::try_parse_from([
            "hex_arm_controller",
            "--profile",
            "/tmp/profile.yaml",
            "--mock",
        ])
        .unwrap();
        let default_config = build_zenoh_config(&default_mock).unwrap();
        assert_eq!(
            default_config
                .get_json("scouting/multicast/enabled")
                .unwrap(),
            "null"
        );
    }

    #[test]
    fn normal_startup_opens_zenoh_before_real_backend_and_withholds_api_until_ready() {
        // This is intentionally a source-order contract: moving any one of
        // these side effects changes live startup semantics even though the
        // individual functions would still type-check and unit-test normally.
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_normal_control(")
            .unwrap()
            .1
            .split_once("#[derive(Debug, Clone, Copy, PartialEq, Eq)]")
            .unwrap()
            .0;
        let zenoh_open = body.find("let session = zenoh::open").unwrap();
        let backend_open = body.find("Arc::new(RealBackend::open").unwrap();
        let runtime_initialized = body.find("runtime.initialize()").unwrap();
        let api_registered = body.find("protocol::serve").unwrap();

        assert!(zenoh_open < backend_open);
        assert!(backend_open < runtime_initialized);
        assert!(runtime_initialized < api_registered);
    }

    #[test]
    fn normal_shutdown_quiesces_hardware_before_aborting_protocol_io() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_normal_control(")
            .unwrap()
            .1
            .split_once("#[derive(Debug, Clone, Copy, PartialEq, Eq)]")
            .unwrap()
            .0;
        let closing_latched = body.rfind("runtime.begin_shutdown()").unwrap();
        let control_joined = body.find("control_task.await").unwrap();
        let backend_shutdown = body.rfind("runtime.shutdown().await").unwrap();
        let protocol_joined = body.find("protocol_tasks.cancel_and_join().await").unwrap();

        assert!(closing_latched < control_joined);
        assert!(control_joined < backend_shutdown);
        assert!(backend_shutdown < protocol_joined);
    }

    #[tokio::test]
    async fn commissioning_signal_cancels_operation_then_always_disables() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let disable_calls = AtomicUsize::new(0);
        let result = run_guarded_commissioning_operation(
            "joint_6",
            std::future::pending::<Result<()>>(),
            std::future::ready(Ok(TerminationSignal::Terminate)),
            || async {
                disable_calls.fetch_add(1, Ordering::SeqCst);
                Ok(())
            },
        )
        .await;

        assert_eq!(disable_calls.load(Ordering::SeqCst), 1);
        assert!(result.unwrap_err().to_string().contains("SIGTERM"));
    }

    #[tokio::test]
    async fn commissioning_success_also_disables_exactly_once() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let disable_calls = AtomicUsize::new(0);
        run_guarded_commissioning_operation(
            "joint_6",
            std::future::ready(Ok(())),
            std::future::pending::<Result<TerminationSignal>>(),
            || async {
                disable_calls.fetch_add(1, Ordering::SeqCst);
                Ok(())
            },
        )
        .await
        .unwrap();

        assert_eq!(disable_calls.load(Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn commissioning_guard_preserves_operation_and_disable_errors() {
        let result = run_guarded_commissioning_operation(
            "joint_6",
            std::future::ready(Err(anyhow::anyhow!("operation failed"))),
            std::future::pending::<Result<TerminationSignal>>(),
            || async { Err(anyhow::anyhow!("disable failed")) },
        )
        .await;
        let error = format!("{:#}", result.unwrap_err());

        assert!(error.contains("operation failed"));
        assert!(error.contains("disable failed"));
    }

    #[test]
    fn commissioning_motion_does_not_compete_for_process_signals() {
        assert!(!include_str!("commissioning.rs").contains("tokio::signal"));
    }

    #[test]
    fn commissioning_cleanup_uses_orderly_backend_shutdown() {
        let source = include_str!("main.rs");
        let body = source
            .split_once("async fn run_hardware_commissioning(")
            .unwrap()
            .1
            .split_once("fn parse_commission_axis")
            .unwrap()
            .0;
        assert!(body.contains("|| backend.shutdown()"));
        assert!(!body.contains("|| backend.disable_all_with_retry"));
    }
}

#[cfg(test)]
mod offline_pose_arguments {
    use super::*;

    #[test]
    fn pose_report_requires_offline_validation_and_six_joint_angles() {
        let args = Arguments::try_parse_from([
            "controller",
            "--profile",
            "candidate.yaml",
            "--validate-profile-only",
            "--check-pose-rad",
            "0",
            "-1.57",
            "3.14",
            "0",
            "0",
            "0",
        ])
        .unwrap();
        assert_eq!(args.check_pose_rad.as_ref().unwrap().len(), 6);
        assert!(Arguments::try_parse_from([
            "controller",
            "--profile",
            "candidate.yaml",
            "--check-pose-rad",
            "0",
            "-1.57",
            "3.14",
            "0",
            "0",
            "0",
        ])
        .is_err());
        assert!(Arguments::try_parse_from([
            "controller",
            "--profile",
            "candidate.yaml",
            "--validate-profile-only",
            "--check-pose-rad",
            "0",
            "-1.57",
            "3.14",
            "0",
            "0",
        ])
        .is_err());
    }
}
