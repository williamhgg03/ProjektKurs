mod led;
mod routes;
mod segway;
mod segway_routes;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use tokio::sync::watch;

use crate::led::LedState;
use crate::segway::TelemetryLog;
use crate::segway_routes::SegwayState;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();

    // Port 80 needs root or CAP_NET_BIND_SERVICE, use PORT=8080 for development
    let port = std::env::var("PORT")
        .ok()
        .and_then(|p| p.parse().ok())
        .unwrap_or(80);
    let addr = SocketAddr::from(([0, 0, 0, 0], port));

    let state = Arc::new(watch::Sender::new(LedState::default()));

    // Segway: tuning (trim, gains) survives restarts in a JSON file, telemetry goes to a CSV
    let settings_path = env_path("SEGWAY_SETTINGS", "segway_settings.json");
    let csv_path = env_path("SEGWAY_LOG", "segway_telemetry.csv");
    let telemetry = TelemetryLog::with_csv(&csv_path).unwrap_or_else(|e| {
        tracing::warn!("can't open {}, telemetry is kept in memory only: {e}", csv_path.display());
        TelemetryLog::default()
    });
    let segway = SegwayState::new(settings_path, telemetry);
    segway_routes::spawn_drive_deadman(segway.clone());

    let app = routes::router(state).merge(segway_routes::router(segway));

    let listener = match tokio::net::TcpListener::bind(addr).await {
        Ok(l) => l,
        Err(e) => {
            eprintln!("failed to bind {addr}: {e}");
            if e.kind() == std::io::ErrorKind::PermissionDenied {
                eprintln!("hint: run with PORT=8080, or grant the binary cap_net_bind_service");
            }
            std::process::exit(1);
        }
    };

    tracing::info!("listening on http://{addr}");
    axum::serve(listener, app.into_make_service_with_connect_info::<SocketAddr>())
        .await.expect("server error");
}

fn env_path(var: &str, default: &str) -> PathBuf {
    PathBuf::from(std::env::var(var).unwrap_or_else(|_| default.into()))
}
