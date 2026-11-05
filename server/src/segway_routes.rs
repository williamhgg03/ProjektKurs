// HTTP routes for the Segway (firmware in SegwayCode/).
//
// Robot-facing:
//   GET  /device/segway?since=N    long-poll for the Command (waits up to LONG_POLL for a change)
//   POST /device/telemetry         {"samples": [...]} about every 250 ms
// Web page:
//   GET  /segway                   control and telemetry page
//   GET  /api/segway               current Command
//   PUT  /api/segway               partial update: enabled, trim_deg, kp, ki, kd
//   POST /api/segway/drive         {lean_deg, turn}, repeat while driving (dead-man, DRIVE_TIMEOUT)
//   POST /api/segway/stop          emergency stop: disable the motors
//   GET  /api/segway/telemetry     ?since=&limit= -> {total, age_ms, samples}
//   GET  /api/segway/telemetry.csv the full CSV log

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::extract::{ConnectInfo, Query, State};
use axum::http::{StatusCode, header};
use axum::response::{Html, IntoResponse};
use axum::routing::{get, post};
use axum::{Json, Router};
use serde::{Deserialize, Serialize};
use tokio::sync::watch;

use crate::segway::{
    self, Command, CommandError, CommandUpdate, Drive, Sample, Settings, TelemetryLog,
};

/// How long `/device/segway?since=` waits for a change before answering anyway. The firmware's
/// HTTP timeout (LONG_POLL_TIMEOUT, 15 s) must be longer.
const LONG_POLL: Duration = Duration::from_secs(10);

/// Driving stops when the web page hasn't repeated its drive command for this long, so a closed
/// tab or a lost phone connection can't leave the robot driving off.
const DRIVE_TIMEOUT: Duration = Duration::from_millis(1000);

/// Most samples `/api/segway/telemetry` returns at once, 10 s at 50 Hz.
const MAX_TELEMETRY_QUERY: usize = 500;

/// A pause this long between uploads is logged as the telemetry stream (re)starting.
const RESUME_AFTER: Duration = Duration::from_secs(5);

#[derive(Clone)]
pub struct SegwayState {
    /// The watch channel holds the command itself, so every change wakes the robot's long-poll.
    command: Arc<watch::Sender<Command>>,
    /// When the web page last sent a drive command, for the dead-man timeout.
    last_drive: Arc<Mutex<Instant>>,
    telemetry: Arc<Mutex<TelemetryLog>>,
    settings_path: Arc<PathBuf>,
}

impl SegwayState {
    pub fn new(settings_path: PathBuf, telemetry: TelemetryLog) -> Self {
        let settings = segway::load_settings(&settings_path).unwrap_or_else(|e| {
            tracing::warn!("reading {} failed, starting with zero gains: {e}", settings_path.display());
            Settings::default()
        });
        tracing::info!(?settings, "Segway settings loaded, robot starts disabled");
        Self {
            command: Arc::new(watch::Sender::new(Command::from_settings(settings))),
            last_drive: Arc::new(Mutex::new(Instant::now())),
            telemetry: Arc::new(Mutex::new(telemetry)),
            settings_path: Arc::new(settings_path),
        }
    }

    /// Apply a change, bump the version and wake the robot only if something changed.
    fn update(&self, f: impl FnOnce(&mut Command) -> Result<bool, CommandError>) -> ApiResult {
        let mut result = Ok(false);
        self.command.send_if_modified(|c| {
            result = f(c);
            let changed = matches!(result, Ok(true));
            if changed {
                c.version += 1;
            }
            changed
        });
        if let Err(e) = result {
            tracing::warn!("rejected Segway update: {e}");
            return Err((StatusCode::BAD_REQUEST, e.to_string()));
        }
        Ok(Json(*self.command.borrow()))
    }

    fn save_settings(&self, settings: Settings) {
        if let Err(e) = segway::save_settings(&self.settings_path, &settings) {
            tracing::warn!("saving {} failed: {e}", self.settings_path.display());
        }
    }
}

type ApiResult = Result<Json<Command>, (StatusCode, String)>;

pub fn router(state: SegwayState) -> Router {
    Router::new()
        .route("/segway", get(page))
        .route("/api/segway", get(get_command).put(put_command))
        .route("/api/segway/drive", post(post_drive))
        .route("/api/segway/stop", post(post_stop))
        .route("/api/segway/telemetry", get(get_telemetry))
        .route("/api/segway/telemetry.csv", get(telemetry_csv))
        .route("/device/segway", get(device_segway))
        .route("/device/telemetry", post(device_telemetry))
        .with_state(state)
}

/// Background task: zero lean/turn when the web page stops repeating its drive command.
pub fn spawn_drive_deadman(state: SegwayState) {
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_millis(100));
        loop {
            tick.tick().await;
            let stale = state.last_drive.lock().unwrap().elapsed() > DRIVE_TIMEOUT;
            if stale && state.command.borrow().is_driving() {
                tracing::warn!("no drive command for {DRIVE_TIMEOUT:?}, stopping the Segway's motion");
                let _ = state.update(|c| Ok(c.halt()));
            }
        }
    });
}

async fn page() -> Html<&'static str> {
    Html(include_str!("../static/segway.html"))
}

async fn get_command(State(state): State<SegwayState>) -> Json<Command> {
    Json(*state.command.borrow())
}

async fn put_command(State(state): State<SegwayState>, Json(body): Json<CommandUpdate>) -> ApiResult {
    let before = *state.command.borrow();
    let result = state.update(|c| c.apply(&body))?;
    let after = result.0;
    if after.enabled != before.enabled {
        tracing::info!(enabled = after.enabled, "Segway motors {}", if after.enabled { "enabled" } else { "disabled" });
    }
    if after.settings() != before.settings() {
        tracing::info!(trim_deg = after.trim_deg, gains = ?after.gains, "Segway settings changed");
        state.save_settings(after.settings());
    }
    Ok(result)
}

async fn post_drive(State(state): State<SegwayState>, Json(body): Json<Drive>) -> ApiResult {
    *state.last_drive.lock().unwrap() = Instant::now();
    state.update(|c| c.drive(body))
}

async fn post_stop(State(state): State<SegwayState>) -> ApiResult {
    tracing::warn!("Segway emergency stop");
    state.update(|c| Ok(c.stop()))
}

#[derive(Deserialize)]
struct DeviceQuery {
    since: Option<u64>,
}

/// Robot-facing endpoint. With `since` equal to the current version it blocks until the
/// command changes (or LONG_POLL passes), otherwise it answers at once.
async fn device_segway(State(state): State<SegwayState>, Query(q): Query<DeviceQuery>) -> Json<Command> {
    let mut rx = state.command.subscribe();
    let current = rx.borrow_and_update().version;
    if q.since == Some(current) {
        let _ = tokio::time::timeout(LONG_POLL, rx.changed()).await;
    }
    Json(*rx.borrow())
}

#[derive(Deserialize)]
struct TelemetryBatch {
    samples: Vec<Sample>,
}

/// Robot-facing endpoint, receives a batch of samples about four times a second.
async fn device_telemetry(
    State(state): State<SegwayState>,
    ConnectInfo(client): ConnectInfo<SocketAddr>,
    Json(body): Json<TelemetryBatch>,
) -> StatusCode {
    let mut log = state.telemetry.lock().unwrap();
    if log.age().is_none_or(|age| age > RESUME_AFTER) {
        tracing::info!(%client, "Segway telemetry arriving");
    }
    if let Err(e) = log.push_batch(&body.samples) {
        tracing::warn!("writing Segway telemetry CSV failed: {e}");
    }
    tracing::debug!(count = body.samples.len(), total = log.total(), "Segway telemetry batch");
    StatusCode::NO_CONTENT
}

#[derive(Deserialize)]
struct TelemetryQuery {
    since: Option<u64>,
    limit: Option<usize>,
}

#[derive(Serialize)]
struct TelemetryResponse {
    /// Pass back as `since` to only get newer samples.
    total: u64,
    /// Milliseconds since the robot last uploaded, `null` if it never has.
    age_ms: Option<u64>,
    samples: Vec<Sample>,
}

async fn get_telemetry(
    State(state): State<SegwayState>,
    Query(q): Query<TelemetryQuery>,
) -> Json<TelemetryResponse> {
    let log = state.telemetry.lock().unwrap();
    let limit = q.limit.unwrap_or(MAX_TELEMETRY_QUERY).min(MAX_TELEMETRY_QUERY);
    Json(TelemetryResponse {
        total: log.total(),
        age_ms: log.age().map(|a| a.as_millis() as u64),
        samples: log.since(q.since, limit),
    })
}

async fn telemetry_csv(State(state): State<SegwayState>) -> Result<impl IntoResponse, (StatusCode, String)> {
    let path = state.telemetry.lock().unwrap().csv_path().map(|p| p.to_path_buf());
    let path = path.ok_or((StatusCode::NOT_FOUND, "CSV logging is disabled".to_string()))?;
    let body = tokio::fs::read(&path)
        .await
        .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, format!("reading {}: {e}", path.display())))?;
    Ok((
        [
            (header::CONTENT_TYPE, "text/csv"),
            (header::CONTENT_DISPOSITION, "attachment; filename=\"segway_telemetry.csv\""),
        ],
        body,
    ))
}
