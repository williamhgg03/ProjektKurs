use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::extract::{ConnectInfo, Query, Request, State};
use axum::http::{header, StatusCode};
use axum::response::{Html, IntoResponse};
use axum::routing::{get, post, put};
use axum::{Json, Router};
use serde::{Deserialize, Serialize};
use tokio::sync::watch;
use tower_http::trace::{DefaultOnResponse, TraceLayer};
use tracing::Level;

use crate::imu::{ImuLog, Sample};
use crate::led::{ConfigUpdate, LedError, LedState, Rgb};

#[derive(Clone)]
pub struct AppState {
    /// The watch channel holds the LED state itself, so every change wakes long-polling clients.
    pub led: Arc<watch::Sender<LedState>>,
    pub imu: Arc<Mutex<ImuLog>>,
}

/// How long `/device/led?since=` waits for a change before answering anyway.
const LONG_POLL: Duration = Duration::from_secs(25);

/// Most samples `/api/imu` returns at once, 10s at 100 Hz.
const MAX_IMU_QUERY: usize = 1000;

/// A pause this long between uploads is logged as the IMU stream (re)starting.
const IMU_RESUME_AFTER: Duration = Duration::from_secs(5);

type ApiResult = Result<Json<LedState>, (StatusCode, String)>;

pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/api/led", get(get_led))
        .route("/api/led/config", put(put_config))
        .route("/api/led/frame", post(post_frame))
        .route("/api/led/fill", post(post_fill))
        .route("/api/led/off", post(post_off))
        .route("/device/led", get(device_led))
        .route("/api/imu/csv", get(imu_csv))
        .layer(
            // One log line per request: method, path and client in the span, status and latency on response.
            TraceLayer::new_for_http()
                .make_span_with(|req: &Request| {
                    let client = req
                        .extensions()
                        .get::<ConnectInfo<SocketAddr>>()
                        .map(|ConnectInfo(addr)| addr.to_string())
                        .unwrap_or_default();
                    tracing::info_span!("request", method = %req.method(), uri = %req.uri(), %client)
                })
                .on_response(DefaultOnResponse::new().level(Level::INFO)),
        )
        // Added after the trace layer on purpose: the chip posts twice a second and the
        // IMU page polls four times a second, which would drown out everything else.
        .route("/imu", get(imu_page))
        .route("/api/imu", get(get_imu))
        .route("/device/imu", post(device_imu))
        .with_state(state)
}

async fn index() -> Html<&'static str> {
    Html(include_str!("../static/index.html"))
}

/// Apply a change and notify waiters only if it succeeded.
fn update(state: &watch::Sender<LedState>, f: impl FnOnce(&mut LedState) -> Result<(), LedError>) -> ApiResult {
    let mut result = Ok(());
    state.send_if_modified(|s| {
        result = f(s);
        result.is_ok()
    });
    if let Err(e) = result {
        tracing::warn!("rejected update: {e}");
        return Err((StatusCode::BAD_REQUEST, e.to_string()));
    }
    let s = state.borrow().clone();
    tracing::info!(version = s.version, "state updated");
    Ok(Json(s))
}

async fn get_led(State(state): State<AppState>) -> Json<LedState> {
    Json(state.led.borrow().clone())
}

async fn put_config(State(state): State<AppState>, Json(body): Json<ConfigUpdate>) -> ApiResult {
    update(&state.led, |s| s.apply_config(body))
}

#[derive(Deserialize)]
struct FrameBody {
    leds: Vec<Rgb>,
}

async fn post_frame(State(state): State<AppState>, Json(body): Json<FrameBody>) -> ApiResult {
    update(&state.led, |s| s.set_frame(body.leds))
}

async fn post_fill(State(state): State<AppState>, Json(color): Json<Rgb>) -> ApiResult {
    update(&state.led, |s| {
        s.fill(color);
        Ok(())
    })
}

async fn post_off(State(state): State<AppState>) -> ApiResult {
    update(&state.led, |s| {
        s.fill(Rgb::OFF);
        Ok(())
    })
}

#[derive(Deserialize)]
struct DeviceQuery {
    since: Option<u64>,
}

#[derive(Serialize)]
struct DeviceFrame {
    version: u64,
    leds: Vec<Rgb>,
}

/// Chip-facing endpoint. With `since` equal to the current version it blocks
/// until the state changes (or LONG_POLL passes), otherwise it answers at once.
async fn device_led(State(state): State<AppState>, Query(q): Query<DeviceQuery>) -> Json<DeviceFrame> {
    let mut rx = state.led.subscribe();
    let current = rx.borrow_and_update().version;
    if q.since == Some(current) {
        let _ = tokio::time::timeout(LONG_POLL, rx.changed()).await;
    }
    let s = rx.borrow();
    Json(DeviceFrame {
        version: s.version,
        leds: s.output_frame(),
    })
}

async fn imu_page() -> Html<&'static str> {
    Html(include_str!("../static/imu.html"))
}

#[derive(Deserialize)]
struct ImuBatch {
    samples: Vec<Sample>,
}

/// Chip-facing endpoint, receives a batch of samples about twice a second.
async fn device_imu(
    State(state): State<AppState>,
    ConnectInfo(client): ConnectInfo<SocketAddr>,
    Json(body): Json<ImuBatch>,
) -> StatusCode {
    let mut imu = state.imu.lock().unwrap();
    if imu.age().is_none_or(|age| age > IMU_RESUME_AFTER) {
        tracing::info!(%client, "IMU data arriving");
    }
    if let Err(e) = imu.push_batch(&body.samples) {
        tracing::warn!("writing IMU CSV failed: {e}");
    }
    tracing::debug!(count = body.samples.len(), total = imu.total(), "IMU batch");
    StatusCode::NO_CONTENT
}

#[derive(Deserialize)]
struct ImuQuery {
    since: Option<u64>,
    limit: Option<usize>,
}

#[derive(Serialize)]
struct ImuResponse {
    /// Pass back as `since` to only get newer samples.
    total: u64,
    /// Milliseconds since the chip last uploaded, `null` if it never has.
    age_ms: Option<u64>,
    samples: Vec<Sample>,
}

async fn get_imu(State(state): State<AppState>, Query(q): Query<ImuQuery>) -> Json<ImuResponse> {
    let imu = state.imu.lock().unwrap();
    let limit = q.limit.unwrap_or(MAX_IMU_QUERY).min(MAX_IMU_QUERY);
    Json(ImuResponse {
        total: imu.total(),
        age_ms: imu.age().map(|a| a.as_millis() as u64),
        samples: imu.since(q.since, limit),
    })
}

async fn imu_csv(State(state): State<AppState>) -> Result<impl IntoResponse, (StatusCode, String)> {
    let path = state.imu.lock().unwrap().csv_path().map(|p| p.to_path_buf());
    let path = path.ok_or((StatusCode::NOT_FOUND, "CSV logging is disabled".to_string()))?;
    let body = tokio::fs::read(&path)
        .await
        .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, format!("reading {}: {e}", path.display())))?;
    Ok((
        [
            (header::CONTENT_TYPE, "text/csv"),
            (header::CONTENT_DISPOSITION, "attachment; filename=\"imu.csv\""),
        ],
        body,
    ))
}
