mod motor_driver;

use std::thread;
use std::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::http::client::{Configuration as HttpConfiguration, EspHttpConnection};
use esp_idf_svc::http::Method;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::{self, esp};
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use serde::Deserialize;

use crate::motor_driver::{Channel, Motor, Tb6612};

// Injected by build.rs from wifi.env or the environment, never stored in source
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");
const SERVER_URL: &str = env!("SERVER_URL");
// Only set for WPA2-Enterprise networks (eduroam); unset means normal WPA2-Personal
const WIFI_EAP_USERNAME: Option<&str> = option_env!("WIFI_EAP_USERNAME");
const WIFI_EAP_IDENTITY: Option<&str> = option_env!("WIFI_EAP_IDENTITY");

/// The server holds a long-poll open for up to 10s, so this must be longer. It
/// also bounds how long the motors keep running after the network silently drops.
const HTTP_TIMEOUT: Duration = Duration::from_secs(15);
const RETRY_DELAY: Duration = Duration::from_secs(2);

/// Response of the server's `GET /device/motors`.
#[derive(Deserialize)]
struct DeviceState {
    version: u64,
    enabled: bool,
    a: Motor,
    b: Motor,
}

fn main() -> anyhow::Result<()> {
    // Required patching for esp-idf runtime
    esp_idf_svc::sys::link_patches();

    // Initialize logger
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // Arduino Nano ESP32 pin labels -> ESP32-S3 GPIOs
    let pins = peripherals.pins;
    let timer = motor_driver::pwm_timer(peripherals.ledc.timer0)?;
    let a = Channel::new(
        peripherals.ledc.channel0,
        &timer,
        pins.gpio1,  // PWMA = A0
        pins.gpio14, // AIN1 = A7
        pins.gpio4,  // AIN2 = A3
    )?;
    let b = Channel::new(
        peripherals.ledc.channel1,
        &timer,
        pins.gpio2,  // PWMB = A1
        pins.gpio12, // BIN1 = A5
        pins.gpio13, // BIN2 = A6
    )?;
    let mut motors = Tb6612::new(timer, pins.gpio11 /* STBY = A4 */, a, b)?;

    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?,
        sysloop,
    )?;
    connect_wifi(&mut wifi)?;

    let mut http = new_http()?;
    let mut since: Option<u64> = None;

    log::info!("Polling {SERVER_URL}/device/motors");
    loop {
        if !wifi.is_connected().unwrap_or(false) {
            log::warn!("WiFi lost, stopping motors and reconnecting");
            failsafe(&mut motors);
            if let Err(e) = wifi.connect().and_then(|_| wifi.wait_netif_up()) {
                log::error!("Reconnect failed: {e}");
                thread::sleep(RETRY_DELAY);
                continue;
            }
        }

        match fetch_state(&mut http, since) {
            Ok(state) => {
                if since != Some(state.version) {
                    log::info!(
                        "State v{}: enabled={} A={:?} B={:?}",
                        state.version, state.enabled, state.a, state.b
                    );
                    if let Err(e) = motors.set(state.enabled, state.a, state.b) {
                        log::error!("Motor update failed: {e}");
                    }
                }
                since = Some(state.version);
            }
            Err(e) => {
                log::warn!("Fetching state failed, stopping motors: {e:?}");
                failsafe(&mut motors);
                since = None;
                // The connection may be left mid-request, start over with a fresh one
                http = new_http()?;
                thread::sleep(RETRY_DELAY);
            }
        }
    }
}

/// Never keep driving blind: stop whenever the server can't tell us what to do.
fn failsafe(motors: &mut Tb6612) {
    if let Err(e) = motors.stop() {
        log::error!("Stopping motors failed: {e}");
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

/// GET the current motor state. With `since` set the server waits until the
/// state changes (or its long-poll timeout passes) before answering.
fn fetch_state(http: &mut EspHttpConnection, since: Option<u64>) -> anyhow::Result<DeviceState> {
    let url = match since {
        Some(v) => format!("{SERVER_URL}/device/motors?since={v}"),
        None => format!("{SERVER_URL}/device/motors"),
    };

    http.initiate_request(Method::Get, &url, &[("accept", "application/json")])?;
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
