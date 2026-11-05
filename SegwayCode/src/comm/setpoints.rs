// Setpoints in: long-polls the server for the current Command and hands it to the control core.
// Adapted from the DC_testing branch's motor polling, with one difference: a network error never
// touches the motors. The control core zeroes lean/turn by itself when the link goes quiet
// (LINK_TIMEOUT_MS) and keeps balancing.

use std::thread;

use esp_idf_svc::hal::cpu;
use esp_idf_svc::http::Method;
use esp_idf_svc::http::client::EspHttpConnection;

use super::wifi::{self, Wifi};
use super::{SERVER_URL, new_http};
use crate::config::{LONG_POLL_TIMEOUT, RETRY_DELAY};
use crate::shared::{Command, SHARED};

/// Runs on the main thread (core 0) forever.
pub fn run(mut wifi: Wifi) -> ! {
    log::info!("Setpoint poller running on {:?}", cpu::core());
    while let Err(e) = wifi::connect(&mut wifi) {
        log::error!("WiFi connect failed, retrying: {e:?}");
        thread::sleep(RETRY_DELAY);
    }

    let url = format!("{SERVER_URL}/device/segway");
    let mut http: Option<EspHttpConnection> = None;
    let mut since: Option<u64> = None;

    log::info!("Polling {url}");
    loop {
        if !wifi.is_connected().unwrap_or(false) {
            log::warn!("WiFi lost, reconnecting");
            http = None;
            if let Err(e) = wifi::connect(&mut wifi) {
                log::error!("Reconnect failed: {e:?}");
                thread::sleep(RETRY_DELAY);
                continue;
            }
        }

        let conn = match &mut http {
            Some(conn) => conn,
            None => match new_http(LONG_POLL_TIMEOUT) {
                Ok(conn) => http.insert(conn),
                Err(e) => {
                    log::error!("Creating HTTP connection failed: {e:?}");
                    thread::sleep(RETRY_DELAY);
                    continue;
                }
            },
        };

        match fetch_command(conn, &url, since) {
            Ok(cmd) => {
                SHARED.link.mark_alive();
                if since != Some(cmd.version) {
                    log::info!("Command v{}: {cmd:?}", cmd.version);
                    SHARED.commands.publish(cmd);
                }
                since = Some(cmd.version);
            }
            Err(e) => {
                log::warn!("Fetching command failed: {e:?}");
                since = None;
                // The connection may be left mid-request, start over with a fresh one
                http = None;
                thread::sleep(RETRY_DELAY);
            }
        }
    }
}

/// GET the current command. With `since` set the server waits until the command changes (or
/// its long-poll timeout passes) before answering.
fn fetch_command(
    http: &mut EspHttpConnection,
    url: &str,
    since: Option<u64>,
) -> anyhow::Result<Command> {
    let url = match since {
        Some(v) => format!("{url}?since={v}"),
        None => url.to_string(),
    };

    http.initiate_request(Method::Get, &url, &[("accept", "application/json")])?;
    http.initiate_response()?;
    let status = http.status();

    let mut body = Vec::new();
    let mut buf = [0u8; 512];
    loop {
        let n = http.read(&mut buf)?;
        if n == 0 {
            break;
        }
        body.extend_from_slice(&buf[..n]);
    }

    anyhow::ensure!(status == 200, "server returned HTTP {status}");
    Ok(serde_json::from_slice(&body)?)
}
