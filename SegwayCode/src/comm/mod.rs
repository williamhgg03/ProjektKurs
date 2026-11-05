// The networking core (core 0): WiFi, setpoints in, telemetry out. Best effort: nothing here
// may stop the control loop, and nothing here may return or panic (that would reboot the chip
// and drop the robot), so every error is logged and retried.

pub mod setpoints;
pub mod telemetry;
pub mod wifi;

use std::time::Duration;

use esp_idf_svc::http::client::{Configuration as HttpConfiguration, EspHttpConnection};

/// Injected by build.rs from wifi.env or the environment, never stored in source.
pub const SERVER_URL: &str = env!("SERVER_URL");

fn new_http(timeout: Duration) -> anyhow::Result<EspHttpConnection> {
    Ok(EspHttpConnection::new(&HttpConfiguration {
        timeout: Some(timeout),
        ..Default::default()
    })?)
}
