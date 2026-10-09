//! Optional, bounded transport tracing. Hot paths never perform file I/O.
//! CLOCK_MONOTONIC is shared with Linux C++/Python clients on the same host;
//! ROS stamps and ArmRuntime's process-relative clock are not comparable to it.
use std::future::Future;
use std::io::{BufWriter, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, Arc, OnceLock};
use std::time::Duration;

const CAPACITY: usize = 8192;
static LOGGER: OnceLock<Option<Logger>> = OnceLock::new();
static NEXT_SPAN_ID: AtomicU64 = AtomicU64::new(1);
tokio::task_local! { static CONTEXT: (u64, u64); }

#[derive(Clone, Copy)]
struct Row {
    timestamp_ns: u64,
    stage: &'static str,
    seq: u64,
    generation: u64,
    boundary: Option<bool>,
    span_id: u64,
}

enum Message {
    Record(Row),
    Flush(mpsc::Sender<()>),
}

struct Logger {
    sender: mpsc::SyncSender<Message>,
    dropped: Arc<AtomicU64>,
}

pub fn monotonic_ns() -> u64 {
    let mut time = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // Both fields are valid output storage for the POSIX call.
    if unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut time) } != 0 {
        return 0;
    }
    (time.tv_sec as u64) * 1_000_000_000 + time.tv_nsec as u64
}

/// Initialize before starting periodic tasks. Tracing is disabled by default.
pub fn init() {
    LOGGER.get_or_init(|| {
        let directory = std::env::var_os("HEX_ARM_TRACE_DIR")?;
        let directory = std::path::PathBuf::from(directory);
        let result = (|| -> std::io::Result<Logger> {
            std::fs::create_dir_all(&directory)?;
            let pid = std::process::id();
            let path = directory.join(format!("rust-{pid}-{}.csv", monotonic_ns()));
            let file = std::fs::File::create(path)?;
            let mut output = BufWriter::new(file);
            writeln!(
                output,
                "timestamp_ns,pid,stage,seq,generation,source_stamp_ns,span_id"
            )?;
            let (sender, receiver) = mpsc::sync_channel(CAPACITY);
            let dropped = Arc::new(AtomicU64::new(0));
            let worker_dropped = dropped.clone();
            std::thread::Builder::new()
                .name("transport-trace".into())
                .spawn(move || {
                    let mut reported_drops = 0;
                    let mut last_flush = std::time::Instant::now();
                    loop {
                        match receiver.recv_timeout(Duration::from_millis(250)) {
                            Ok(Message::Record(row)) => {
                                if write_row(&mut output, pid, row).is_err() {
                                    break;
                                }
                            }
                            Ok(Message::Flush(acknowledge)) => {
                                let drops = worker_dropped.load(Ordering::Relaxed);
                                if drops != reported_drops {
                                    let _ = write_row(
                                        &mut output,
                                        pid,
                                        Row {
                                            timestamp_ns: monotonic_ns(),
                                            stage: "trace_dropped",
                                            seq: drops,
                                            generation: 0,
                                            boundary: None,
                                            span_id: 0,
                                        },
                                    );
                                    reported_drops = drops;
                                }
                                let _ = output.flush();
                                let _ = acknowledge.send(());
                            }
                            Err(mpsc::RecvTimeoutError::Disconnected) => break,
                            Err(mpsc::RecvTimeoutError::Timeout) => {
                                let drops = worker_dropped.load(Ordering::Relaxed);
                                if drops != reported_drops {
                                    let _ = write_row(
                                        &mut output,
                                        pid,
                                        Row {
                                            timestamp_ns: monotonic_ns(),
                                            stage: "trace_dropped",
                                            seq: drops,
                                            generation: 0,
                                            boundary: None,
                                            span_id: 0,
                                        },
                                    );
                                    reported_drops = drops;
                                }
                                if output.flush().is_err() {
                                    break;
                                }
                            }
                        }
                        if last_flush.elapsed() >= Duration::from_millis(250) {
                            let drops = worker_dropped.load(Ordering::Relaxed);
                            if drops != reported_drops {
                                let _ = write_row(
                                    &mut output,
                                    pid,
                                    Row {
                                        timestamp_ns: monotonic_ns(),
                                        stage: "trace_dropped",
                                        seq: drops,
                                        generation: 0,
                                        boundary: None,
                                        span_id: 0,
                                    },
                                );
                                reported_drops = drops;
                            }
                            if output.flush().is_err() {
                                break;
                            }
                            last_flush = std::time::Instant::now();
                        }
                    }
                    let _ = output.flush();
                })?;
            Ok(Logger { sender, dropped })
        })();
        match result {
            Ok(logger) => Some(logger),
            Err(error) => {
                tracing::warn!(%error, "transport tracing unavailable");
                None
            }
        }
    });
}

fn write_row(output: &mut impl Write, pid: u32, row: Row) -> std::io::Result<()> {
    let boundary = match row.boundary {
        Some(true) => "_begin",
        Some(false) => "_end",
        None => "",
    };
    writeln!(
        output,
        "{},{},{}{},{},{},0,{}",
        row.timestamp_ns, pid, row.stage, boundary, row.seq, row.generation, row.span_id
    )
}

pub fn enabled() -> bool {
    LOGGER.get().is_some_and(Option::is_some)
}

fn submit(stage: &'static str, seq: u64, generation: u64, boundary: Option<bool>, span_id: u64) {
    if let Some(Some(logger)) = LOGGER.get() {
        if logger
            .sender
            .try_send(Message::Record(Row {
                timestamp_ns: monotonic_ns(),
                stage,
                seq,
                generation,
                boundary,
                span_id,
            }))
            .is_err()
        {
            logger.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }
}

pub fn record(stage: &'static str, seq: u64, generation: u64) {
    submit(stage, seq, generation, None, 0);
}

pub struct Span {
    stage: &'static str,
    seq: u64,
    generation: u64,
    span_id: u64,
}
impl Span {
    pub fn new(stage: &'static str, seq: u64, generation: u64) -> Self {
        let span_id = if enabled() {
            NEXT_SPAN_ID.fetch_add(1, Ordering::Relaxed)
        } else {
            0
        };
        submit(stage, seq, generation, Some(true), span_id);
        Self {
            stage,
            seq,
            generation,
            span_id,
        }
    }
}
impl Drop for Span {
    fn drop(&mut self) {
        submit(
            self.stage,
            self.seq,
            self.generation,
            Some(false),
            self.span_id,
        );
    }
}

pub async fn with_context<T>(seq: u64, generation: u64, operation: impl Future<Output = T>) -> T {
    if enabled() {
        CONTEXT.scope((seq, generation), operation).await
    } else {
        operation.await
    }
}
pub fn context() -> (u64, u64) {
    CONTEXT.try_with(|context| *context).unwrap_or_default()
}

/// Final process cleanup only; never call from a control tick.
pub fn flush() {
    if let Some(Some(logger)) = LOGGER.get() {
        let (send, receive) = mpsc::channel();
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        let mut message = Message::Flush(send);
        loop {
            match logger.sender.try_send(message) {
                Ok(()) => {
                    let _ = receive.recv_timeout(
                        deadline.saturating_duration_since(std::time::Instant::now()),
                    );
                    break;
                }
                Err(mpsc::TrySendError::Disconnected(_)) => break,
                Err(mpsc::TrySendError::Full(pending)) => {
                    if std::time::Instant::now() >= deadline {
                        break;
                    }
                    message = pending;
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn trace_clock_is_monotonic_and_rows_keep_command_identity() {
        let first = monotonic_ns();
        assert!(first > 0);
        assert!(monotonic_ns() >= first);
        let mut output = Vec::new();
        write_row(
            &mut output,
            12,
            Row {
                timestamp_ns: first,
                stage: "gate_wait",
                seq: 7,
                generation: 9,
                boundary: Some(false),
                span_id: 42,
            },
        )
        .unwrap();
        assert_eq!(
            String::from_utf8(output).unwrap(),
            format!("{first},12,gate_wait_end,7,9,0,42\n")
        );
    }
    #[tokio::test]
    async fn trace_context_is_scoped_to_the_operation_and_task() {
        CONTEXT
            .scope((7, 9), async {
                tokio::task::yield_now().await;
                assert_eq!(context(), (7, 9));
                assert_eq!(tokio::spawn(async { context() }).await.unwrap(), (0, 0));
            })
            .await;
        assert_eq!(context(), (0, 0));
    }
}
