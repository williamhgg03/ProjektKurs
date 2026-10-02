mod buzzer;
mod led_driver;

use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::thread;
use std::time::{Duration, Instant};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::http::client::{Configuration as HttpConfiguration, EspHttpConnection};
use esp_idf_svc::http::Method;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::{self, esp};
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use serde::de::DeserializeOwned;
use serde::Deserialize;

use crate::buzzer::{Buzzer, Note};
use crate::led_driver::{LedStrip, Rgb};

// Injected by build.rs from wifi.env or the environment, never stored in source
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");
const SERVER_URL: &str = env!("SERVER_URL");
// Only set for WPA2-Enterprise networks (eduroam); unset means normal WPA2-Personal
const WIFI_EAP_USERNAME: Option<&str> = option_env!("WIFI_EAP_USERNAME");
const WIFI_EAP_IDENTITY: Option<&str> = option_env!("WIFI_EAP_IDENTITY");

/// The server holds a long-poll open for up to 25s, so this must be longer.
const HTTP_TIMEOUT: Duration = Duration::from_secs(35);
const RETRY_DELAY: Duration = Duration::from_secs(2);
/// Spawned threads default to a 4K stack, too small for HTTP plus JSON.
const THREAD_STACK: usize = 8 * 1024;

/// Response of the server's `GET /device/led`.
#[derive(Deserialize)]
struct DeviceFrame {
    version: u64,
    leds: Vec<Rgb>,
}

/// Response of the server's `GET /device/melody`.
#[derive(Deserialize)]
struct DeviceMelody {
    version: u64,
    playing: bool,
    /// Empty unless playing.
    notes: Vec<Note>,
}

/// Sent from the melody poller to the player thread.
enum Command {
    Play(Vec<Note>),
    Stop,
}

fn main() -> anyhow::Result<()> {
    // Required patching for esp-idf runtime
    esp_idf_svc::sys::link_patches();

    // Initialize logger
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut strip = LedStrip::new(peripherals.pins.gpio1)?;
    let buzzer = Buzzer::new(
        peripherals.ledc.channel0,
        peripherals.ledc.timer0,
        peripherals.pins.gpio2,
    )?;

    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?,
        sysloop,
    )?;
    connect_wifi(&mut wifi)?;

    // The buzzer gets its own threads, so a song plays while the LED loop
    // below sits in a long-poll
    let (commands, command_rx) = mpsc::channel();
    thread::Builder::new()
        .name("buzzer".into())
        .stack_size(THREAD_STACK)
        .spawn(move || {
            if let Err(e) = play_songs(buzzer, command_rx) {
                log::error!("Buzzer stopped: {e:?}");
            }
        })?;
    thread::Builder::new()
        .name("melody-poll".into())
        .stack_size(THREAD_STACK)
        .spawn(move || {
            if let Err(e) = poll_melody(commands) {
                log::error!("Melody polling stopped: {e:?}");
            }
        })?;

    let mut http = new_http()?;
    let mut since: Option<u64> = None;

    log::info!("Polling {SERVER_URL}/device/led");
    loop {
        if !wifi.is_connected().unwrap_or(false) {
            log::warn!("WiFi lost, reconnecting");
            if let Err(e) = wifi.connect().and_then(|_| wifi.wait_netif_up()) {
                log::error!("Reconnect failed: {e}");
                thread::sleep(RETRY_DELAY);
                continue;
            }
        }

        match fetch_json::<DeviceFrame>(&mut http, &poll_url("/device/led", since)) {
            Ok(frame) => {
                if since != Some(frame.version) {
                    log::info!("Frame v{} ({} LEDs)", frame.version, frame.leds.len());
                    if let Err(e) = strip.write(&frame.leds) {
                        log::error!("LED write failed: {e}");
                    }
                }
                since = Some(frame.version);
            }
            Err(e) => {
                log::warn!("Fetching frame failed: {e:?}");
                since = None;
                // The connection may be left mid-request, start over with a fresh one
                http = new_http()?;
                thread::sleep(RETRY_DELAY);
            }
        }
    }
}

fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<()> {
    let ssid = WIFI_SSID
        .try_into()
        .map_err(|_| anyhow::anyhow!("SSID too long"))?;

    if let Some(username) = WIFI_EAP_USERNAME {
        wifi.set_configuration(&Configuration::Client(ClientConfiguration {
            ssid,
            auth_method: AuthMethod::WPA2Enterprise,
            ..Default::default()
        }))?;
        enable_enterprise(WIFI_EAP_IDENTITY.unwrap_or(username), username, WIFI_PASS)?;
        log::info!("Connecting to WiFi (WPA2-Enterprise)...");
    } else {
        wifi.set_configuration(&Configuration::Client(ClientConfiguration {
            ssid,
            password: WIFI_PASS
                .try_into()
                .map_err(|_| anyhow::anyhow!("password too long"))?,
            ..Default::default()
        }))?;
        log::info!("Connecting to WiFi (WPA2-Personal)...");
    }

    wifi.start()?;
    wifi.connect()?;
    wifi.wait_netif_up()?;

    let ip = wifi.wifi().sta_netif().get_ip_info()?;
    log::info!("Connected, IP: {}", ip.ip);
    Ok(())
}

/// Sets up 802.1X (PEAP/TTLS with MSCHAPv2) credentials for networks like eduroam.
/// No CA cert is set, so the RADIUS server's certificate is not verified; add
/// `esp_eap_client_set_ca_cert` with the institution's CA to enable that.
fn enable_enterprise(identity: &str, username: &str, password: &str) -> anyhow::Result<()> {
    // ESP-IDF copies the buffers, so they only need to live for the call
    unsafe {
        esp!(sys::esp_eap_client_set_identity(identity.as_ptr(), identity.len() as _))?;
        esp!(sys::esp_eap_client_set_username(username.as_ptr(), username.len() as _))?;
        esp!(sys::esp_eap_client_set_password(password.as_ptr(), password.len() as _))?;
        esp!(sys::esp_wifi_sta_enterprise_enable())?;
    }
    Ok(())
}

fn new_http() -> anyhow::Result<EspHttpConnection> {
    Ok(EspHttpConnection::new(&HttpConfiguration {
        timeout: Some(HTTP_TIMEOUT),
        ..Default::default()
    })?)
}

/// `SERVER_URL` + `path`, asking the server to wait for a change past `since`.
fn poll_url(path: &str, since: Option<u64>) -> String {
    match since {
        Some(v) => format!("{SERVER_URL}{path}?since={v}"),
        None => format!("{SERVER_URL}{path}"),
    }
}

/// GET `url` and parse the JSON body. With `?since=` in the URL the server
/// waits until its state changes (or its long-poll timeout passes) before answering.
fn fetch_json<T: DeserializeOwned>(http: &mut EspHttpConnection, url: &str) -> anyhow::Result<T> {
    http.initiate_request(Method::Get, url, &[("accept", "application/json")])?;
    http.initiate_response()?;
    let status = http.status();

    let mut body = Vec::new();
    let mut buf = [0u8; 1024];
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

/// Long-poll `/device/melody` and hand every change to the player.
/// Reconnecting WiFi is left to the main loop.
fn poll_melody(commands: Sender<Command>) -> anyhow::Result<()> {
    let mut http = new_http()?;
    let mut since: Option<u64> = None;

    loop {
        match fetch_json::<DeviceMelody>(&mut http, &poll_url("/device/melody", since)) {
            Ok(m) => {
                if since != Some(m.version) {
                    log::info!("Melody v{} (playing: {}, {} notes)", m.version, m.playing, m.notes.len());
                    commands.send(if m.playing {
                        Command::Play(m.notes)
                    } else {
                        Command::Stop
                    })?;
                }
                since = Some(m.version);
            }
            Err(e) => {
                // Unlike the LED loop, `since` is kept: re-fetching the same
                // version would restart the song after every network hiccup
                log::warn!("Fetching melody failed: {e:?}");
                http = new_http()?;
                thread::sleep(RETRY_DELAY);
            }
        }
    }
}

/// Play songs as commands arrive. A new command cuts the current song short.
fn play_songs(mut buzzer: Buzzer<'_>, commands: Receiver<Command>) -> anyhow::Result<()> {
    let mut pending = None;
    loop {
        let command = match pending.take() {
            Some(c) => c,
            None => commands.recv()?,
        };
        if let Command::Play(notes) = command {
            log::info!("Playing {} notes", notes.len());
            pending = play(&mut buzzer, &notes, &commands)?;
        }
        buzzer.silence()?;
    }
}

/// Play `notes` to the end, or until a command arrives, which is returned.
fn play(buzzer: &mut Buzzer<'_>, notes: &[Note], commands: &Receiver<Command>) -> anyhow::Result<Option<Command>> {
    // Absolute deadlines: each wait is rounded to the FreeRTOS tick, but the
    // error doesn't add up over the song
    let mut deadline = Instant::now();
    for note in notes {
        match note.freq {
            0 => buzzer.silence()?,
            hz => buzzer.tone(hz.into())?,
        }
        deadline += Duration::from_millis(note.ms.into());
        match commands.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
            Ok(c) => return Ok(Some(c)),
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => anyhow::bail!("melody poller is gone"),
        }
    }
    Ok(None)
}
