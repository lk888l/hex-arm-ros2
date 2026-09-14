//! Non-realtime, atomic process shutdown acknowledgement for deployment tooling.
use anyhow::{Context, Result};
use std::{
    fs::{self, OpenOptions},
    io::Write,
    path::Path,
    time::{SystemTime, UNIX_EPOCH},
};

pub fn write(path: Option<&Path>, state: &str, error: Option<&str>) -> Result<()> {
    let Some(path) = path else {
        return Ok(());
    };
    anyhow::ensure!(
        matches!(
            state,
            "starting" | "disabled_confirmed" | "disable_unconfirmed"
        ),
        "invalid shutdown state"
    );
    let tmp = path.with_extension(format!("tmp.{}", std::process::id()));
    let result = (|| -> Result<()> {
        let mut file = OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&tmp)
            .with_context(|| format!("create shutdown acknowledgement {}", tmp.display()))?;
        let value = serde_json::json!({
            "schema_version": 1,
            "pid": std::process::id(),
            "state": state,
            "error": error,
            "unix_time_ms": SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis(),
        });
        serde_json::to_writer(&mut file, &value)?;
        file.write_all(b"\n")?;
        file.sync_all()?;
        fs::rename(&tmp, path)?;
        Ok(())
    })();
    // Do not remove a temporary file that we did not create.
    // A failed create_new indicates an operator-visible stale/colliding writer.
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn new_run_invalidates_old_confirmation_and_stop_is_structured() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("stop.json");
        write(Some(&path), "disabled_confirmed", None).unwrap();
        write(Some(&path), "starting", None).unwrap();
        let read = || {
            serde_json::from_str::<serde_json::Value>(&fs::read_to_string(&path).unwrap()).unwrap()
        };
        assert_eq!(read()["state"], "starting");
        write(Some(&path), "disable_unconfirmed", Some("bus lost")).unwrap();
        assert_eq!(read()["error"], "bus lost");
        assert_eq!(read()["schema_version"], 1);
    }
    #[test]
    fn report_failure_is_an_error_and_cannot_claim_success() {
        assert!(write(
            Some(Path::new("/nonexistent/hex-arm/stop.json")),
            "disabled_confirmed",
            None
        )
        .is_err());
        assert!(write(None, "disabled_confirmed", None).is_ok());
    }
}
