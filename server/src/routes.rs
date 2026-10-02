use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use axum::body::Bytes;
use axum::extract::{ConnectInfo, DefaultBodyLimit, Query, Request, State};
use axum::http::StatusCode;
use axum::response::Html;
use axum::routing::{get, post, put};
use axum::{Json, Router};
use serde::{Deserialize, Serialize};
use tokio::sync::watch;
use tower_http::trace::{DefaultOnResponse, TraceLayer};
use tracing::Level;

use crate::led::{ConfigUpdate, LedState, Rgb};
use crate::melody::{self, MelodyState, Note};

/// Each watch channel holds its state itself, so every change wakes long-polling clients.
pub struct Shared {
    pub led: watch::Sender<LedState>,
    pub melody: watch::Sender<MelodyState>,
}

pub type AppState = Arc<Shared>;

/// How long `/device/...?since=` waits for a change before answering anyway.
const LONG_POLL: Duration = Duration::from_secs(25);

/// Audio uploads are far bigger than axum's 2 MB default body limit.
const MAX_UPLOAD: usize = 25 * 1024 * 1024;
const DEFAULT_OCTAVE: u8 = 1;

type ApiResult<S> = Result<Json<S>, (StatusCode, String)>;

/// States that bump a version on every change, which long-polling relies on.
pub trait Versioned {
    fn version(&self) -> u64;
}

impl Versioned for LedState {
    fn version(&self) -> u64 {
        self.version
    }
}

impl Versioned for MelodyState {
    fn version(&self) -> u64 {
        self.version
    }
}

pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/api/led", get(get_led))
        .route("/api/led/config", put(put_config))
        .route("/api/led/frame", post(post_frame))
        .route("/api/led/fill", post(post_fill))
        .route("/api/led/off", post(post_off))
        .route(
            "/api/melody",
            get(get_melody)
                .post(post_melody)
                .layer(DefaultBodyLimit::max(MAX_UPLOAD)),
        )
        .route("/api/melody/play", post(post_play))
        .route("/api/melody/stop", post(post_stop))
        .route("/device/led", get(device_led))
        .route("/device/melody", get(device_melody))
        .with_state(state)
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
}

async fn index() -> Html<&'static str> {
    Html(include_str!("../static/index.html"))
}

/// Apply a change and notify waiters only if it succeeded.
fn update<S, E>(tx: &watch::Sender<S>, f: impl FnOnce(&mut S) -> Result<(), E>) -> ApiResult<S>
where
    S: Clone + Versioned,
    E: std::fmt::Display,
{
    let mut result = Ok(());
    tx.send_if_modified(|s| {
        result = f(s);
        result.is_ok()
    });
    if let Err(e) = result {
        tracing::warn!("rejected update: {e}");
        return Err((StatusCode::BAD_REQUEST, e.to_string()));
    }
    let s = tx.borrow().clone();
    tracing::info!(version = s.version(), "state updated");
    Ok(Json(s))
}

/// With `since` equal to the current version, wait until the state changes
/// (or LONG_POLL passes). The returned receiver holds the state to answer with.
async fn long_poll<S: Versioned>(tx: &watch::Sender<S>, since: Option<u64>) -> watch::Receiver<S> {
    let mut rx = tx.subscribe();
    let current = rx.borrow_and_update().version();
    if since == Some(current) {
        let _ = tokio::time::timeout(LONG_POLL, rx.changed()).await;
    }
    rx
}

async fn get_led(State(state): State<AppState>) -> Json<LedState> {
    Json(state.led.borrow().clone())
}

async fn put_config(State(state): State<AppState>, Json(body): Json<ConfigUpdate>) -> ApiResult<LedState> {
    update(&state.led, |s| s.apply_config(body))
}

#[derive(Deserialize)]
struct FrameBody {
    leds: Vec<Rgb>,
}

async fn post_frame(State(state): State<AppState>, Json(body): Json<FrameBody>) -> ApiResult<LedState> {
    update(&state.led, |s| s.set_frame(body.leds))
}

async fn post_fill(State(state): State<AppState>, Json(color): Json<Rgb>) -> ApiResult<LedState> {
    update(&state.led, |s| {
        s.fill(color);
        Ok::<_, std::convert::Infallible>(())
    })
}

async fn post_off(State(state): State<AppState>) -> ApiResult<LedState> {
    update(&state.led, |s| {
        s.fill(Rgb::OFF);
        Ok::<_, std::convert::Infallible>(())
    })
}

async fn get_melody(State(state): State<AppState>) -> Json<MelodyState> {
    Json(state.melody.borrow().clone())
}

#[derive(Deserialize)]
struct UploadQuery {
    octave: Option<u8>,
    name: Option<String>,
}

/// Body is the raw audio file. Decoding takes a while, so it runs off the async workers.
async fn post_melody(
    State(state): State<AppState>,
    Query(q): Query<UploadQuery>,
    body: Bytes,
) -> ApiResult<MelodyState> {
    let octave = q.octave.unwrap_or(DEFAULT_OCTAVE);
    let size = body.len();
    let result = tokio::task::spawn_blocking(move || melody::transcribe(body, octave))
        .await
        .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e.to_string()))?;
    if let Ok(t) = &result {
        tracing::info!(size, octave, notes = t.notes.len(), truncated = t.truncated, "melody transcribed");
    }
    update(&state.melody, |s| result.map(|t| s.load(q.name, t)))
}

async fn post_play(State(state): State<AppState>) -> ApiResult<MelodyState> {
    update(&state.melody, |s| s.play())
}

async fn post_stop(State(state): State<AppState>) -> ApiResult<MelodyState> {
    update(&state.melody, |s| {
        s.stop();
        Ok::<_, std::convert::Infallible>(())
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
    let rx = long_poll(&state.led, q.since).await;
    let s = rx.borrow();
    Json(DeviceFrame {
        version: s.version,
        leds: s.output_frame(),
    })
}

#[derive(Serialize)]
struct DeviceMelody {
    version: u64,
    playing: bool,
    /// Only sent while playing, the chip has nothing to do with them otherwise.
    notes: Vec<Note>,
}

/// Chip-facing endpoint for the buzzer, long-polls like `/device/led`.
async fn device_melody(State(state): State<AppState>, Query(q): Query<DeviceQuery>) -> Json<DeviceMelody> {
    let rx = long_poll(&state.melody, q.since).await;
    let s = rx.borrow();
    Json(DeviceMelody {
        version: s.version,
        playing: s.playing,
        notes: if s.playing { s.notes.clone() } else { Vec::new() },
    })
}
