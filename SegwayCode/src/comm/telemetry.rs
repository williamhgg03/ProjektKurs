// Telemetry out: batches the control core's samples and POSTs them to the server. Adapted from
// the accel_And_gyro branch's IMU uploader. Also logs the control loop's Status, so the
// control thread itself never has to wait on the console.

use std::sync::mpsc::Receiver;
use std::thread;
use std::time::Instant;

use esp_idf_svc::hal::cpu::{self, Core};
use esp_idf_svc::http::Method;
use esp_idf_svc::http::client::EspHttpConnection;
use serde::Serialize;

use super::{SERVER_URL, new_http};
use crate::config::{
    MAX_BATCH, MAX_PENDING, NET_TX_PRIORITY, NET_TX_STACK, RETRY_DELAY, STATUS_LOG_INTERVAL,
    UPLOAD_INTERVAL, UPLOAD_TIMEOUT,
};
use crate::rtos;
use crate::shared::{Mode, SHARED, Telemetry};

/// Body of `POST /device/telemetry`.
#[derive(Serialize)]
struct Batch<'a> {
    samples: &'a [Telemetry],
}

/// Start the uploader thread on core 0.
pub fn spawn(rx: Receiver<Telemetry>) -> anyhow::Result<()> {
    rtos::spawn_pinned(
        c"net-tx",
        Core::Core0,
        NET_TX_PRIORITY,
        NET_TX_STACK,
        move || run(rx),
    )?;
    Ok(())
}

fn run(rx: Receiver<Telemetry>) {
    let url = format!("{SERVER_URL}/device/telemetry");
    let mut http: Option<EspHttpConnection> = None;
    let mut pending: Vec<Telemetry> = Vec::with_capacity(MAX_PENDING);
    let mut failing = false;
    let mut status_log = StatusLog::new();

    log::info!(
        "Telemetry uploader running on {:?}, posting to {url}",
        cpu::core()
    );
    loop {
        thread::sleep(UPLOAD_INTERVAL);
        status_log.poll();

        // Drain the channel before every batch, so it never fills up during a long upload round
        while collect(&rx, &mut pending) {
            let conn = match &mut http {
                Some(conn) => conn,
                None => match new_http(UPLOAD_TIMEOUT) {
                    Ok(conn) => http.insert(conn),
                    Err(e) => {
                        log::error!("Creating HTTP connection failed: {e:?}");
                        break;
                    }
                },
            };

            let n = pending.len().min(MAX_BATCH);
            match post_batch(conn, &url, &pending[..n]) {
                Ok(()) => {
                    pending.drain(..n);
                    SHARED.link.mark_alive();
                    if failing {
                        log::info!("Telemetry upload working again");
                        failing = false;
                    }
                }
                Err(e) => {
                    if !failing {
                        log::warn!(
                            "Telemetry upload failed, keeping the newest {MAX_PENDING} samples: {e:?}"
                        );
                        failing = true;
                    }
                    // The connection may be left mid-request, start over with a fresh one
                    http = None;
                    thread::sleep(RETRY_DELAY);
                    break;
                }
            }
        }
    }
}

/// Move new samples from the channel into `pending`, dropping the oldest beyond
/// `MAX_PENDING`. Returns whether there is anything to upload.
fn collect(rx: &Receiver<Telemetry>, pending: &mut Vec<Telemetry>) -> bool {
    pending.extend(rx.try_iter());
    if pending.len() > MAX_PENDING {
        let excess = pending.len() - MAX_PENDING;
        pending.drain(..excess);
    }
    !pending.is_empty()
}

fn post_batch(
    http: &mut EspHttpConnection,
    url: &str,
    samples: &[Telemetry],
) -> anyhow::Result<()> {
    let body = serde_json::to_vec(&Batch { samples })?;
    let len = body.len().to_string();

    http.initiate_request(
        Method::Post,
        url,
        &[
            ("content-type", "application/json"),
            ("content-length", &len),
        ],
    )?;
    http.write_all(&body)?;
    http.initiate_response()?;
    let status = http.status();

    // Read the (empty) response body so the connection can be reused
    let mut buf = [0u8; 64];
    while http.read(&mut buf)? > 0 {}

    anyhow::ensure!(
        (200..300).contains(&status),
        "server returned HTTP {status}"
    );
    Ok(())
}

/// Logs the control core's Status: every change of mode or link, plus a periodic summary.
struct StatusLog {
    last_mode: Option<Mode>,
    last_link_ok: bool,
    last_summary: Instant,
}

impl StatusLog {
    fn new() -> Self {
        Self {
            last_mode: None,
            last_link_ok: false,
            last_summary: Instant::now(),
        }
    }

    fn poll(&mut self) {
        let s = SHARED.status.read();

        if self.last_mode != Some(s.mode) {
            log::info!("Control mode {:?} (pitch {:+.1} deg)", s.mode, s.pitch_deg);
            self.last_mode = Some(s.mode);
        }
        if self.last_link_ok != s.link_ok {
            if s.link_ok {
                log::info!("Server link up, motion commands active");
            } else {
                log::warn!("Server link lost, lean/turn zeroed (still balancing)");
            }
            self.last_link_ok = s.link_ok;
        }

        if self.last_summary.elapsed() >= STATUS_LOG_INTERVAL {
            self.last_summary = Instant::now();
            log::info!(
                "control: {:?} pitch {:+.1} deg | dt {}..{} us, compute max {} us | imu errors {}, telemetry dropped {}",
                s.mode,
                s.pitch_deg,
                s.dt_min_us,
                s.dt_max_us,
                s.loop_max_us,
                s.imu_errors,
                s.telemetry_dropped,
            );
        }
    }
}
