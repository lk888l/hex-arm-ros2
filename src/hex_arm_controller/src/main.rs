use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result};
use clap::Parser;
use hex_arm_controller::backend::{MockBackend, MotorBackend, RealBackend};
use hex_arm_controller::profile::HardwareProfile;
use hex_arm_controller::protocol;
use hex_arm_controller::runtime::ArmRuntime;
use tracing_subscriber::EnvFilter;

#[derive(Debug, Parser)]
#[command(about = "Firefly Y6 userspace USB/CAN-FD controller")]
struct Arguments {
    #[arg(long)]
    profile: PathBuf,
    #[arg(long, default_value_t = false)]
    mock: bool,
    #[arg(long, default_value = "")]
    zenoh_connect: String,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();
    let arguments = Arguments::parse();
    let profile = Arc::new(HardwareProfile::from_path(&arguments.profile)?);
    let dynamics = hex_arm_dynamics::ArmDynamics::from_urdf_file(&profile.urdf_path)
        .context("load fixed hex-arm-dynamics model")?;
    anyhow::ensure!(dynamics.dof() == 6, "URDF dynamics model is not six-axis");

    let backend: Arc<dyn MotorBackend> = if arguments.mock {
        tracing::warn!("using simulated motor backend; no physical outputs exist");
        Arc::new(MockBackend::new())
    } else {
        Arc::new(RealBackend::open(profile.clone()).await?)
    };
    let runtime = Arc::new(ArmRuntime::new(profile.clone(), backend, dynamics));
    runtime
        .initialize()
        .await
        .context("initialize arm in DISABLED state")?;

    let mut zenoh_config = zenoh::Config::default();
    zenoh_config
        .insert_json5("mode", "\"peer\"")
        .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    if !arguments.zenoh_connect.is_empty() {
        zenoh_config
            .insert_json5(
                "connect/endpoints",
                &format!("[\"{}\"]", arguments.zenoh_connect),
            )
            .map_err(|error| anyhow::anyhow!(error.to_string()))?;
    }
    let session = zenoh::open(zenoh_config)
        .await
        .map_err(|error| anyhow::anyhow!("open Zenoh robot_api session: {error}"))?;
    let urdf_xml = std::fs::read_to_string(&profile.urdf_path).context("read URDF resource")?;
    protocol::serve(session, runtime.clone(), urdf_xml).await?;
    tokio::spawn(runtime.clone().run_control_loop());

    tracing::info!(prefix = %profile.robot_prefix, "Firefly Y6 controller ready and DISABLED");
    tokio::signal::ctrl_c()
        .await
        .context("wait for shutdown signal")?;
    tracing::info!("shutdown requested; disabling all motors");
    runtime.shutdown().await;
    Ok(())
}
