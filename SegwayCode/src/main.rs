// Segway firmware for the Arduino Nano ESP32 (ESP32-S3), std Rust on ESP-IDF.
//
// The two cores have separate jobs:
//
//   core 0  networking, best effort   WiFi + lwIP (ESP-IDF), comm::setpoints (this main thread,
//                                     long-polls the server), comm::telemetry ("net-tx" thread,
//                                     uploads samples and logs the control status)
//   core 1  control, hard real time   control ("control" thread, alone on the core): MPU-6050 ->
//                                     estimator -> controller -> TB6612, at the IMU's 200 Hz
//
// They only talk through shared.rs (latest-value mailboxes, an atomic heartbeat) and one
// telemetry channel, all non-blocking on the control side. sdkconfig.defaults keeps the ESP-IDF
// system tasks off core 1.

mod comm;
mod config;
mod control;
mod rtos;
mod shared;

use std::sync::mpsc;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};

use crate::control::ControlHw;

fn main() -> anyhow::Result<()> {
    // Required patching for esp-idf runtime
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // ============================================================================================
    // PIN BLOCK: every GPIO the firmware uses. The only place to edit when the wiring changes.
    //
    // !!! PLACEHOLDERS: these are NOT the Segway's wiring yet. Set them before connecting motors.
    //
    // Arduino Nano ESP32 label -> GPIO:
    //   D2=5 D3=6 D4=7 D5=8 D6=9 D7=10 D8=17 D9=18 D10=21 D11=38 D12=47 D13=48
    //   A0=1 A1=2 A2=3 A3=4 A4=11 A5=12 A6=13 A7=14
    // Avoid: GPIO43/44 (D1/D0, UART console), GPIO19/20 (USB), GPIO0/45/46 (strapping, RGB LED).
    // ============================================================================================
    let pins = peripherals.pins;
    let control_hw = ControlHw {
        i2c: peripherals.i2c0,
        imu_sda: pins.gpio47.into(), // MPU-6050 SDA  (D12)  
        imu_scl: pins.gpio21.into(), // MPU-6050 SCL  (D10)  
        imu_int: pins.gpio38.into(),  // MPU-6050 INT (D11)  

        pwm_timer: peripherals.ledc.timer0,
        left_pwm_channel: peripherals.ledc.channel0,
        right_pwm_channel: peripherals.ledc.channel1,
        left_pwm: pins.gpio1.into(),    // TB6612 PWMA (A0)  
        left_in1: pins.gpio7.into(),    // TB6612 AIN1 (D4)  
        left_in2: pins.gpio8.into(),    // TB6612 AIN2 (D5)  
        right_pwm: pins.gpio2.into(),   // TB6612 PWMB (A1)  
        right_in1: pins.gpio9.into(),  // TB6612 BIN1 (D6)  
        right_in2: pins.gpio10.into(),  // TB6612 BIN2 (D7)  
        motor_stby: pins.gpio6.into(), // TB6612 STBY (D3)  
    };
    // ============================================================================================

    // Everything that can fail on bad configuration happens before the control thread starts,
    // so an error here reboots before the motors were ever enabled
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?,
        sysloop,
    )?;
    comm::wifi::start(&mut wifi)?;

    let (telemetry_tx, telemetry_rx) = mpsc::sync_channel(config::TELEMETRY_CHANNEL_CAP);

    // Core 1. Calibrates the gyro while core 0 connects to WiFi
    control::spawn(control_hw, telemetry_tx)?;
    // Core 0
    comm::telemetry::spawn(telemetry_rx)?;
    comm::setpoints::run(wifi)
}
