mod imu;
mod led;
mod routes;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use tokio::sync::watch;

use crate::imu::ImuLog;
use crate::led::LedState;
use crate::routes::AppState;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();

    // Port 80 needs root or CAP_NET_BIND_SERVICE, use PORT=8080 for development
    let port = std::env::var("PORT")
        .ok()
        .and_then(|p| p.parse().ok())
        .unwrap_or(80);
    let addr = SocketAddr::from(([0, 0, 0, 0], port));

    // Every IMU sample is appended here, set IMU_LOG to change the path
    let csv_path = PathBuf::from(std::env::var("IMU_LOG").unwrap_or_else(|_| "imu.csv".into()));
    let imu = match ImuLog::with_csv(&csv_path) {
        Ok(log) => {
            tracing::info!("logging IMU samples to {}", csv_path.display());
            log
        }
        Err(e) => {
            tracing::warn!("cannot open {}: {e}, IMU samples will not be saved", csv_path.display());
            ImuLog::default()
        }
    };

    let state = AppState {
        led: Arc::new(watch::Sender::new(LedState::default())),
        imu: Arc::new(Mutex::new(imu)),
    };
    let app = routes::router(state);

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
