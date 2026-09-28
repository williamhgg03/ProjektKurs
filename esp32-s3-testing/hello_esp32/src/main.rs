mod imu_logger;
mod led_driver;
mod mpu6050;

use std::thread;
use std::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::gpio::{Input, PinDriver, Pull};
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::http::client::{Configuration as HttpConfiguration, EspHttpConnection};
use esp_idf_svc::http::Method;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::EspError;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use serde::Deserialize;

use crate::led_driver::{LedStrip, Rgb};
use crate::mpu6050::Mpu6050;

// Injected by build.rs from wifi.env or the environment, never stored in source
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");
const SERVER_URL: &str = env!("SERVER_URL");

/// The server holds a long-poll open for up to 25s, so this must be longer.
const HTTP_TIMEOUT: Duration = Duration::from_secs(35);
const RETRY_DELAY: Duration = Duration::from_secs(2);
/// MPU-6050 maximum I2C clock.
const I2C_BAUDRATE: Hertz = Hertz(400_000);

/// Response of the server's `GET /device/led`.
#[derive(Deserialize)]
struct DeviceFrame {
    version: u64,
    leds: Vec<Rgb>,
}

fn main() -> anyhow::Result<()> {
    // Required patching for esp-idf runtime
    esp_idf_svc::sys::link_patches();

    // Initialize logger
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut strip = LedStrip::new(peripherals.pins.gpio0)?;

    // MPU-6050 wiring: SDA, SCL and INT (data ready). Change the pins here.
    let imu = init_imu(
        I2cDriver::new(
            peripherals.i2c0,
            peripherals.pins.gpio4,
            peripherals.pins.gpio5,
            &I2cConfig::new().baudrate(I2C_BAUDRATE),
        ),
        PinDriver::input(peripherals.pins.gpio6, Pull::Down),
    );

    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?,
        sysloop,
    )?;
    connect_wifi(&mut wifi)?;

    // Keep the LED strip working even if the sensor is missing or miswired
    match imu {
        Ok((mpu, int)) => imu_logger::start(mpu, int)?,
        Err(e) => log::error!("MPU-6050 not available, IMU logging disabled: {e:?}"),
    }

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

        match fetch_frame(&mut http, since) {
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

fn init_imu(
    i2c: Result<I2cDriver<'static>, EspError>,
    int: Result<PinDriver<'static, Input>, EspError>,
) -> anyhow::Result<(Mpu6050<'static>, PinDriver<'static, Input>)> {
    Ok((Mpu6050::new(i2c?)?, int?))
}

fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<()> {
    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: WIFI_SSID
            .try_into()
            .map_err(|_| anyhow::anyhow!("SSID too long"))?,
        password: WIFI_PASS
            .try_into()
            .map_err(|_| anyhow::anyhow!("password too long"))?,
        ..Default::default()
    }))?;

    wifi.start()?;
    log::info!("Connecting to WiFi...");
    wifi.connect()?;
    wifi.wait_netif_up()?;

    let ip = wifi.wifi().sta_netif().get_ip_info()?;
    log::info!("Connected, IP: {}", ip.ip);
    Ok(())
}

fn new_http() -> anyhow::Result<EspHttpConnection> {
    Ok(EspHttpConnection::new(&HttpConfiguration {
        timeout: Some(HTTP_TIMEOUT),
        ..Default::default()
    })?)
}

/// GET the current frame. With `since` set the server waits until the frame
/// changes (or its long-poll timeout passes) before answering.
fn fetch_frame(http: &mut EspHttpConnection, since: Option<u64>) -> anyhow::Result<DeviceFrame> {
    let url = match since {
        Some(v) => format!("{SERVER_URL}/device/led?since={v}"),
        None => format!("{SERVER_URL}/device/led"),
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
