use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use axum::extract::{ConnectInfo, Query, Request, State};
use axum::http::StatusCode;
use axum::response::Html;
use axum::routing::{get, post};
use axum::{Json, Router};
use serde::Deserialize;
use tokio::sync::watch;
use tower_http::trace::{DefaultOnResponse, TraceLayer};
use tracing::Level;

use crate::motor::{MotorError, MotorState, StateUpdate};

/// The watch channel holds the state itself, so every change wakes long-polling clients.
pub type AppState = Arc<watch::Sender<MotorState>>;

/// How long `/device/motors?since=` waits for a change before answering anyway.
const LONG_POLL: Duration = Duration::from_secs(10);

type ApiResult = Result<Json<MotorState>, (StatusCode, String)>;

pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/api/motors", get(get_motors).put(put_motors))
        .route("/api/motors/stop", post(post_stop))
        .route("/device/motors", get(device_motors))
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
fn update(state: &AppState, f: impl FnOnce(&mut MotorState) -> Result<(), MotorError>) -> ApiResult {
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

async fn get_motors(State(state): State<AppState>) -> Json<MotorState> {
    Json(state.borrow().clone())
}

async fn put_motors(State(state): State<AppState>, Json(body): Json<StateUpdate>) -> ApiResult {
    update(&state, |s| s.apply(body))
}

async fn post_stop(State(state): State<AppState>) -> ApiResult {
    update(&state, |s| {
        s.stop();
        Ok(())
    })
}

#[derive(Deserialize)]
struct DeviceQuery {
    since: Option<u64>,
}

/// Chip-facing endpoint. With `since` equal to the current version it blocks
/// until the state changes (or LONG_POLL passes), otherwise it answers at once.
async fn device_motors(State(state): State<AppState>, Query(q): Query<DeviceQuery>) -> Json<MotorState> {
    let mut rx = state.subscribe();
    let current = rx.borrow_and_update().version;
    if q.since == Some(current) {
        let _ = tokio::time::timeout(LONG_POLL, rx.changed()).await;
    }
    let s = rx.borrow().clone();
    Json(s)
}
