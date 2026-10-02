// Reads the MPU-6050 on its data-ready interrupt and uploads the samples to the server.
//
// Sampling and uploading run on separate threads so a slow or failing HTTP request
// never delays sampling: sampler -> bounded channel -> uploader -> POST /device/imu

use std::num::NonZeroU32;
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::thread;
use std::time::Duration;

use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::gpio::{Input, InterruptType, PinDriver};
use esp_idf_svc::hal::task::notification::Notification;
use esp_idf_svc::http::client::EspHttpConnection;
use esp_idf_svc::http::Method;
use serde::Serialize;

use crate::mpu6050::{Mpu6050, Sample};

/// 2.5s of samples at 100 Hz, enough to cover an upload that takes a while.
const CHANNEL_CAP: usize = 256;
const UPLOAD_INTERVAL: Duration = Duration::from_millis(500);
/// Samples per POST, keeps the JSON body around 16 KB.
const MAX_BATCH: usize = 100;
/// Unsent samples kept while the server is unreachable (10s), oldest dropped first.
const MAX_PENDING: usize = 1000;
/// Data-ready fires every 10ms, not seeing it for this long means INT is not connected.
const INT_TIMEOUT: Duration = Duration::from_millis(100);
/// Fail a stuck POST quickly, while it blocks samples are only buffered by the channel.
const HTTP_TIMEOUT: Duration = Duration::from_secs(5);
const STACK_SIZE: usize = 8192;

#[derive(Serialize)]
struct Batch<'a> {
    samples: &'a [Sample],
}

pub fn start(mpu: Mpu6050<'static>, int: PinDriver<'static, Input>) -> anyhow::Result<()> {
    let (tx, rx) = mpsc::sync_channel(CHANNEL_CAP);

    thread::Builder::new()
        .name("imu-sample".into())
        .stack_size(STACK_SIZE)
        .spawn(move || {
            if let Err(e) = sample_loop(mpu, int, tx) {
                log::error!("IMU sampling stopped: {e:?}");
            }
        })?;

    thread::Builder::new()
        .name("imu-upload".into())
        .stack_size(STACK_SIZE)
        .spawn(move || upload_loop(rx))?;

    Ok(())
}

fn sample_loop(
    mut mpu: Mpu6050<'static>,
    mut int: PinDriver<'static, Input>,
    tx: SyncSender<Sample>,
) -> anyhow::Result<()> {
    // A notification wakes the task that created it, so it must be created on this thread
    let notification = Notification::new();
    let notifier = notification.notifier();

    int.set_interrupt_type(InterruptType::PosEdge)?;
    // SAFETY: notifying a task is ISR-safe, and the pin is unsubscribed below before
    // this thread (the notified task) can end.
    unsafe {
        int.subscribe(move || {
            notifier.notify_and_yield(NonZeroU32::MIN);
        })?;
    }

    let result = run_sampler(&mut mpu, &mut int, &notification, &tx);
    int.unsubscribe()?;
    result
}

fn run_sampler(
    mpu: &mut Mpu6050<'static>,
    int: &mut PinDriver<'static, Input>,
    notification: &Notification,
    tx: &SyncSender<Sample>,
) -> anyhow::Result<()> {
    let timeout = TickType::from(INT_TIMEOUT).ticks();
    let mut int_ok = true;
    let mut channel_full = false;

    loop {
        // The driver disables the interrupt each time it fires
        int.enable_interrupt()?;
        let fired = notification.wait(timeout).is_some();
        if fired != int_ok {
            int_ok = fired;
            if fired {
                log::info!("MPU-6050 data-ready interrupt working");
            } else {
                log::warn!("No data-ready interrupt from the MPU-6050, check the INT wire. Reading every {INT_TIMEOUT:?} instead");
            }
        }

        let sample = match mpu.read() {
            Ok(s) => s,
            Err(e) => {
                log::warn!("MPU-6050 read failed: {e}");
                continue;
            }
        };

        match tx.try_send(sample) {
            Ok(()) => channel_full = false,
            Err(TrySendError::Full(_)) => {
                if !channel_full {
                    log::warn!("IMU upload is behind, dropping samples");
                }
                channel_full = true;
            }
            Err(TrySendError::Disconnected(_)) => anyhow::bail!("uploader thread ended"),
        }
    }
}

fn upload_loop(rx: Receiver<Sample>) {
    let url = format!("{}/device/imu", crate::SERVER_URL);
    let mut http: Option<EspHttpConnection> = None;
    let mut pending: Vec<Sample> = Vec::new();

    log::info!("Uploading IMU samples to {url}");
    loop {
        thread::sleep(UPLOAD_INTERVAL);

        // Drain the channel before every batch, so it never fills up during a long upload round
        while collect(&rx, &mut pending) {
            let conn = match &mut http {
                Some(c) => c,
                None => match crate::new_http(HTTP_TIMEOUT) {
                    Ok(c) => http.insert(c),
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
                }
                Err(e) => {
                    log::warn!("IMU upload failed: {e:?}");
                    // The connection may be left mid-request, start over with a fresh one
                    http = None;
                    thread::sleep(crate::RETRY_DELAY);
                    break;
                }
            }
        }
    }
}

/// Move new samples from the channel into `pending`, dropping the oldest beyond
/// `MAX_PENDING`. Returns whether there is anything to upload.
fn collect(rx: &Receiver<Sample>, pending: &mut Vec<Sample>) -> bool {
    pending.extend(rx.try_iter());
    if pending.len() > MAX_PENDING {
        let excess = pending.len() - MAX_PENDING;
        pending.drain(..excess);
        log::warn!("Server unreachable, dropped {excess} old IMU samples");
    }
    !pending.is_empty()
}

fn post_batch(http: &mut EspHttpConnection, url: &str, samples: &[Sample]) -> anyhow::Result<()> {
    let body = serde_json::to_vec(&Batch { samples })?;
    let len = body.len().to_string();

    http.initiate_request(
        Method::Post,
        url,
        &[("content-type", "application/json"), ("content-length", &len)],
    )?;
    http.write_all(&body)?;
    http.initiate_response()?;
    let status = http.status();

    // Read the (empty) response body so the connection can be reused
    let mut buf = [0u8; 64];
    while http.read(&mut buf)? > 0 {}

    anyhow::ensure!((200..300).contains(&status), "server returned HTTP {status}");
    Ok(())
}
