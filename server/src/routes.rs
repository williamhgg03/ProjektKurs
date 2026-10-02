use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use axum::extract::{ConnectInfo, Query, Request, State};
use axum::http::StatusCode;
use axum::response::Html;
use axum::routing::{get, post, put};
use axum::{Json, Router};
use serde::{Deserialize, Serialize};
use tokio::sync::watch;
use tower_http::trace::{DefaultOnResponse, TraceLayer};
use tracing::Level;

use crate::led::{ConfigUpdate, LedError, LedState, Rgb};

/// The watch channel holds the state itself, so every change wakes long-polling clients.
pub type AppState = Arc<watch::Sender<LedState>>;

/// How long `/device/led?since=` waits for a change before answering anyway.
const LONG_POLL: Duration = Duration::from_secs(25);

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
fn update(state: &AppState, f: impl FnOnce(&mut LedState) -> Result<(), LedError>) -> ApiResult {
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
    Json(state.borrow().clone())
}

async fn put_config(State(state): State<AppState>, Json(body): Json<ConfigUpdate>) -> ApiResult {
    update(&state, |s| s.apply_config(body))
}

#[derive(Deserialize)]
struct FrameBody {
    leds: Vec<Rgb>,
}

async fn post_frame(State(state): State<AppState>, Json(body): Json<FrameBody>) -> ApiResult {
    update(&state, |s| s.set_frame(body.leds))
}

async fn post_fill(State(state): State<AppState>, Json(color): Json<Rgb>) -> ApiResult {
    update(&state, |s| {
        s.fill(color);
        Ok(())
    })
}

async fn post_off(State(state): State<AppState>) -> ApiResult {
    update(&state, |s| {
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
    let mut rx = state.subscribe();
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
