//! Production driver process: profile -> motor backend -> runtime -> protocol.
//! Historical motion/recovery commands live in the opt-in hex_arm_commission binary.
use anyhow::{Context, Result};
use clap::Parser;
use hex_arm_controller::{
    motor::{MeowBackend, MockBackend, MotorBackend},
    payload_dynamics::load_profile_dynamics,
    profile::{HardwareProfile, MotorProtocol},
    protocol,
    runtime::ArmRuntime,
};
use std::{fmt, path::PathBuf, sync::Arc};
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
#[command(about = "Firefly Y6 Meow / CiA402 production driver")]
struct Arguments {
    #[arg(long)]
    profile: PathBuf,
    /// Simulated motor IO; never opens CAN.
    #[arg(long)]
    mock: bool,
    #[arg(long, default_value = "")]
    zenoh_connect: String,
    #[arg(long, default_value = "")]
    zenoh_listen: String,
    /// Offline profile/model validation; never opens CAN or Zenoh.
    #[arg(long, conflicts_with = "mock")]
    validate_profile_only: bool,
    #[arg(
        long,
        num_args = 6,
        allow_hyphen_values = true,
        requires = "validate_profile_only"
    )]
    check_pose_rad: Option<Vec<f32>>,
    /// Relocatable installed URDF, replacing only the profile's model path.
    #[arg(long)]
    urdf: Option<PathBuf>,
    /// Optional atomic JSON stop acknowledgement. Its absence is NOT confirmation.
    #[arg(long)]
    shutdown_report: Option<PathBuf>,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();
    let arguments = Arguments::parse();
    hex_arm_controller::trace::init();
    if !arguments.validate_profile_only {
        hex_arm_controller::shutdown_report::write(
            arguments.shutdown_report.as_deref(),
            "starting",
            None,
        )?;
    }
    let profile = Arc::new(HardwareProfile::from_path_with_urdf(
        &arguments.profile,
        arguments.urdf.as_deref(),
    )?);
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

    let result = run_normal_control(&arguments, profile, dynamics).await;
    hex_arm_controller::trace::flush();
    result
}

async fn shutdown_with_report(runtime: &ArmRuntime, path: Option<&std::path::Path>) -> Result<()> {
    let shutdown = runtime.shutdown().await;
    let error = shutdown.as_ref().err().map(|e| format!("{e:#}"));
    let report = hex_arm_controller::shutdown_report::write(
        path,
        if shutdown.is_ok() {
            "disabled_confirmed"
        } else {
            "disable_unconfirmed"
        },
        error.as_deref(),
    );
    combine_operation_and_cleanup("stop acknowledgement", shutdown, report)
}

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
            #[cfg(feature = "legacy")]
            MotorProtocol::Cia402 => Arc::new(hex_arm_controller::motor::RealBackend::open(profile.clone()).await?),
            #[cfg(not(feature = "legacy"))]
            MotorProtocol::Cia402 => anyhow::bail!("production driver supports Meow / SocketCAN only; legacy CiA402 requires --features legacy"),
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
        let shutdown = shutdown_with_report(&runtime, arguments.shutdown_report.as_deref()).await;
        return combine_operation_and_cleanup("controller initialization", Err(error), shutdown);
    }

    let mut protocol_tasks = match tokio::select! {
        signal = termination_signals.received() => Err(anyhow::anyhow!("API startup interrupted: {:?}", signal)),
        result = async {
        let urdf_xml = std::fs::read_to_string(&profile.urdf_path).context("read URDF resource")?;
        protocol::serve(session, runtime.clone(), urdf_xml).await
        } => result,
    } {
        Ok(tasks) => tasks,
        Err(error) => {
            runtime.begin_shutdown();
            let shutdown =
                shutdown_with_report(&runtime, arguments.shutdown_report.as_deref()).await;
            return combine_operation_and_cleanup("controller API startup", Err(error), shutdown);
        }
    };
    let mut control_task = tokio::spawn(runtime.clone().run_control_loop());

    tracing::info!(prefix = %profile.robot_prefix, "Firefly Y6 controller ready and DISABLED");
    let mut joined_control = None;
    let operation_result = tokio::select! {
        signal = termination_signals.received() => signal.map(|signal| {
            tracing::info!(%signal, "controller termination signal received");
        }),
        error = protocol_tasks.wait_for_failure() => Err(error),
        outcome = &mut control_task => {
            joined_control = Some(outcome);
            Err(anyhow::anyhow!("control loop exited unexpectedly"))
        },
    };

    // Close admission synchronously before either worker is asked to stop.
    // The control loop is joined first. Final runtime shutdown then waits on
    // mode_gate, so every admitted hardware RPC finishes before backend
    // disable/heartbeat-disarm. Only after hardware is physically quiescent do
    // we abort and join possibly stuck Zenoh reply/publication I/O; transport
    // liveness can therefore never postpone the safety shutdown.
    runtime.begin_shutdown();
    tracing::info!("controller stopping; disabling drives and disarming heartbeat consumers");
    let control_outcome = match joined_control {
        Some(outcome) => outcome,
        None => control_task.await,
    };
    let control_shutdown = match control_outcome {
        Ok(()) => Ok(()),
        Err(error) => Err(anyhow::anyhow!("control loop task failed: {error}")),
    };
    let backend_shutdown =
        shutdown_with_report(&runtime, arguments.shutdown_report.as_deref()).await;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn production_cli_rejects_historical_motion_flags() {
        for flag in [
            "--commission-axis",
            "--diagnose-axis",
            "--startup-sequence",
            "--recover-heartbeat-lost",
        ] {
            assert!(
                Arguments::try_parse_from(["driver", "--profile", "profile.yaml", flag]).is_err()
            );
        }
    }

    #[test]
    fn validation_and_mock_are_mutually_exclusive() {
        assert!(Arguments::try_parse_from([
            "driver",
            "--profile",
            "p",
            "--mock",
            "--validate-profile-only"
        ])
        .is_err());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn api_startup_failure_still_writes_confirmed_disable_receipt() {
        let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let profile = Arc::new(
            HardwareProfile::from_path_with_urdf(
                manifest.join("test/firefly_y6.mock.yaml"),
                Some(&manifest.join("../xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf")),
            )
            .unwrap(),
        );
        let dynamics = load_profile_dynamics(&profile).unwrap();
        let runtime = Arc::new(ArmRuntime::new(
            profile,
            Arc::new(MockBackend::new()),
            dynamics,
        ));
        runtime.initialize().await.unwrap();
        let directory = tempfile::tempdir().unwrap();
        let receipt = directory.path().join("driver-shutdown.json");
        hex_arm_controller::shutdown_report::write(Some(&receipt), "starting", None).unwrap();
        let mut config = zenoh::Config::default();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        config.insert_json5("listen/endpoints", "[]").unwrap();
        let session = zenoh::open(config).await.unwrap();
        session.close().await.unwrap();
        let operation = match protocol::serve(session, runtime.clone(), String::new()).await {
            Ok(_) => panic!("closed transport accepted API startup"),
            Err(error) => error,
        };
        runtime.begin_shutdown();
        let cleanup = shutdown_with_report(&runtime, Some(&receipt)).await;
        assert!(
            combine_operation_and_cleanup("controller API startup", Err(operation), cleanup)
                .is_err()
        );
        let report: serde_json::Value =
            serde_json::from_slice(&std::fs::read(receipt).unwrap()).unwrap();
        assert_eq!(report["state"], "disabled_confirmed");
        assert_eq!(report["pid"], std::process::id());
        assert!(report["error"].is_null());
    }

    #[test]
    fn cleanup_errors_are_never_hidden_by_operation_errors() {
        let err = combine_operation_and_cleanup(
            "test",
            Err(anyhow::anyhow!("operation")),
            Err(anyhow::anyhow!("cleanup")),
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("operation") && err.contains("cleanup"));
        assert!(combine_runtime_shutdown(Ok(()), Ok(()), Err(anyhow::anyhow!("disable"))).is_err());
    }
}
