//! Process-boundary regressions. Uses loopback Zenoh and MockBackend only.
use std::{
    fs,
    io::{BufRead, BufReader},
    net::TcpListener,
    process::{Child, Command, Stdio},
    sync::mpsc,
    time::{Duration, Instant},
};

struct ProcessGuard(Child);
impl Drop for ProcessGuard {
    fn drop(&mut self) {
        if self.0.try_wait().ok().flatten().is_none() {
            let _ = self.0.kill();
        }
        let _ = self.0.wait();
    }
}

#[cfg(unix)]
#[test]
fn sigint_and_sigterm_both_acknowledge_confirmed_mock_disable() {
    for signal in ["INT", "TERM"] {
        let directory = tempfile::tempdir().unwrap();
        let report_path = directory.path().join("driver-shutdown.json");
        let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        drop(listener);
        let mut process = ProcessGuard(
            Command::new(env!("CARGO_BIN_EXE_hex_arm_controller"))
                .arg("--profile")
                .arg(manifest.join("test/firefly_y6.mock.yaml"))
                .arg("--urdf")
                .arg(manifest.join("../xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf"))
                .args(["--mock", "--zenoh-listen", &format!("tcp/127.0.0.1:{port}")])
                .arg("--shutdown-report")
                .arg(&report_path)
                .stdout(Stdio::piped())
                .stderr(Stdio::inherit())
                .spawn()
                .unwrap(),
        );
        let stdout = process.0.stdout.take().unwrap();
        let (tx, rx) = mpsc::channel();
        let reader = std::thread::spawn(move || {
            for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                if line.contains("controller ready and DISABLED") {
                    let _ = tx.send(());
                }
            }
        });
        rx.recv_timeout(Duration::from_secs(15))
            .expect("mock driver did not become ready");
        let starting: serde_json::Value =
            serde_json::from_slice(&fs::read(&report_path).unwrap()).unwrap();
        assert_eq!(starting["state"], "starting");
        assert!(Command::new("kill")
            .args(["-s", signal, &process.0.id().to_string()])
            .status()
            .unwrap()
            .success());
        let deadline = Instant::now() + Duration::from_secs(10);
        let status = loop {
            if let Some(status) = process.0.try_wait().unwrap() {
                break status;
            }
            assert!(Instant::now() < deadline, "mock driver did not stop");
            std::thread::sleep(Duration::from_millis(10));
        };
        assert!(status.success());
        reader.join().unwrap();
        let report: serde_json::Value =
            serde_json::from_slice(&fs::read(&report_path).unwrap()).unwrap();
        assert_eq!(report["schema_version"], 1);
        assert_eq!(report["pid"], process.0.id());
        assert_eq!(report["state"], "disabled_confirmed");
        assert!(report["error"].is_null());
    }
}
