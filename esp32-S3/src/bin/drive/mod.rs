// Sends the held button to the server as a Segway drive command.
//
// The server zeroes the motion if it hasn't heard from us for 1 s (its dead-man
// timeout), so while a button is held the command is repeated every RESEND, and
// a zero command is sent once when the button is released.

#![allow(
    clippy::large_stack_frames,
    reason = "the task's state lives in static task storage, not on a stack"
)]

use defmt::{info, warn};
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::TcpClient;
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use reqwless::client::{HttpClient, HttpResource};
use reqwless::headers::ContentType;
use reqwless::request::RequestBuilder;

use crate::controller::{DRIVE_SIGNAL, Direction, STOP_BODY};

const SERVER_URL: &str = env!("SERVER_URL");
const DRIVE_PATH: &str = "/api/segway/drive";

/// How often the command is repeated while a button is held. Must be well under
/// the server's 1 s dead-man timeout.
const RESEND: Duration = Duration::from_millis(200);
const RECONNECT_DELAY: Duration = Duration::from_secs(1);

/// Room for the server's response headers; the drive endpoint answers with a
/// small JSON body.
pub const RX_BUF_SIZE: usize = 1024;

#[embassy_executor::task]
pub async fn drive_task(
    tcp_client: TcpClient<'static, 1, 1500, 1500>,
    dns_client: DnsSocket<'static>,
    rx_buf: &'static mut [u8; RX_BUF_SIZE],
) {
    let mut held: Option<Direction> = None;
    // A command that still has to reach the server, kept across reconnects.
    let mut pending = false;

    loop {
        let mut client = HttpClient::new(&tcp_client, &dns_client);
        let mut resource = match client.resource(SERVER_URL).await {
            Ok(resource) => resource,
            Err(e) => {
                warn!("Connecting to the server failed: {:?}", e);
                Timer::after(RECONNECT_DELAY).await;
                continue;
            }
        };
        info!("Connected to {}", SERVER_URL);

        loop {
            if !pending {
                let event = match held {
                    // Holding: resend when nothing changes for RESEND
                    Some(_) => with_timeout(RESEND, DRIVE_SIGNAL.wait()).await.ok(),
                    None => Some(DRIVE_SIGNAL.wait().await),
                };
                if let Some(direction) = event {
                    held = direction;
                }
            }
            // A newer touch state beats a command that failed to send
            if let Some(direction) = DRIVE_SIGNAL.try_take() {
                held = direction;
            }

            let body = held.map_or(STOP_BODY, Direction::drive_body);
            pending = true;
            match post_drive(&mut resource, body, rx_buf).await {
                Ok(status) => {
                    pending = false;
                    if !(200..300).contains(&status) {
                        warn!("Server rejected the drive command: HTTP {}", status);
                    }
                }
                Err(e) => {
                    warn!("Sending the drive command failed, reconnecting: {:?}", e);
                    Timer::after(RECONNECT_DELAY).await;
                    break;
                }
            }
        }
    }
}

/// POST `body` to the drive endpoint. Returns the HTTP status code.
async fn post_drive<C: Read + Write>(
    resource: &mut HttpResource<'_, C>,
    body: &'static str,
    rx_buf: &mut [u8],
) -> Result<u16, reqwless::Error> {
    let response = resource
        .post(DRIVE_PATH)
        .content_type(ContentType::ApplicationJson)
        .body(body.as_bytes())
        .send(rx_buf)
        .await?;
    let status = response.status.0;
    // Drain the body so the connection can be reused for the next command
    response.body().read_to_end().await?;
    Ok(status)
}
