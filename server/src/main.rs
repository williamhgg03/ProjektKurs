mod led;
mod routes;

use std::net::SocketAddr;
use std::sync::Arc;

use tokio::sync::watch;

use crate::led::LedState;

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
    axum::serve(listener, app).await.expect("server error");
}
