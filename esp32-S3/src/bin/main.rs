#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

// Segway remote control: four arrow buttons on a round touch screen. A held
// button is sent to the server as a drive command, which the Segway polls for.
//
//   touch_task  --BUTTON_SIGNAL-->  display_task  (highlights the button)
//               --DRIVE_SIGNAL--->  drive_task    (POST /api/segway/drive)

use alloc::string::ToString;
use defmt::info;
use embassy_executor::Spawner;
use embassy_net::{
    Runner, StackResources,
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println as _;
use esp_radio::wifi::sta::eap::EapStationConfig;
use esp_radio::wifi::{
    AuthenticationMethod, Config as WifiConfig, ControllerConfig, Interface, WifiController,
    sta::StationConfig,
};

extern crate alloc;

mod controller;
mod drive;
mod lcd;
mod system;
mod touch;

macro_rules! mk_static {
    ($t:ty, $value:expr) => {{
        static CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        CELL.uninit().write($value)
    }};
}

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");
const WIFI_EAP_USERNAME: Option<&str> = option_env!("WIFI_EAP_USERNAME");
const WIFI_EAP_IDENTITY: Option<&str> = option_env!("WIFI_EAP_IDENTITY");

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32s3 -o unstable-hal -o alloc -o wifi -o ci -o neovim -o defmt -o esp-backtrace -o embassy -o esp

    let m_peripherals = system::hardware::init();
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let m_timg0 = system::timer::init(m_peripherals.TIMG0);
    let m_sw_interrupt = system::interrupt::init(m_peripherals.SW_INTERRUPT);
    esp_rtos::start(m_timg0.timer0, m_sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // Pin map for the Waveshare ESP32-S3-Zero. Only pins on the two side
    // headers (GP1-GP13) are used. Avoided: GP0/GP3/GP45/GP46 (strapping),
    // GP21 (onboard WS2812 LED), GP19/GP20 (USB), GP43/GP44 (UART0 TX/RX).
    //
    //   1.28" Touch LCD  ->  ESP32-S3-Zero
    //   LCD_DIN/MOSI     ->  GP11
    //   LCD_CLK/SCLK     ->  GP12
    //   LCD_CS           ->  GP10
    //   LCD_DC           ->  GP9
    //   LCD_RST          ->  GP8
    //   LCD_BL           ->  GP7
    //   TP_SDA           ->  GP6
    //   TP_SCL           ->  GP5
    //   TP_INT           ->  GP4  (wired, not used by the firmware yet)
    //   TP_RST           ->  GP2
    let lcd = lcd::display::Lcd::new(
        m_peripherals.SPI2,
        m_peripherals.GPIO11, // MOSI
        m_peripherals.GPIO12, // SCLK
        m_peripherals.GPIO10, // LCD_CS
        m_peripherals.GPIO9,  // LCD_DC
        m_peripherals.GPIO8,  // LCD_RST
        m_peripherals.GPIO7,  // LCD_BL
    );

    let touch = touch::touch::Touch::new(
        m_peripherals.I2C0,
        m_peripherals.GPIO6, // TP_SDA
        m_peripherals.GPIO5, // TP_SCL
        m_peripherals.GPIO2, // TP_RST
    );

    // The screen and touch work before (and without) the network
    spawner.spawn(lcd::display_task(lcd).expect("Failed to spawn LCD task"));
    spawner.spawn(touch::touch_task(touch).expect("Failed to spawn touch task"));

    let wifi_config = if let Some(username) = WIFI_EAP_USERNAME {
        let identity = WIFI_EAP_IDENTITY.unwrap_or(username);
        info!("Configuring WPA2-Enterprise Wi-Fi");
        WifiConfig::EapStation(
            EapStationConfig::default()
                .with_ssid(WIFI_SSID)
                .with_identity(identity.to_string())
                .with_username(username.to_string())
                .with_password(WIFI_PASS.to_string()),
        )
    } else {
        info!("Configuring WPA2-Personal Wi-Fi");
        WifiConfig::Station(
            StationConfig::default()
                .with_ssid(WIFI_SSID)
                .with_auth_method(AuthenticationMethod::Wpa2Personal)
                .with_password(WIFI_PASS.to_string()),
        )
    };

    let (wifi_controller, interfaces) = esp_radio::wifi::new(
        m_peripherals.WIFI,
        ControllerConfig::default().with_initial_config(wifi_config),
    )
    .expect("Failed to initialize Wi-Fi controller");
    let wifi_interface = interfaces.station;

    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let rng = esp_hal::rng::Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(wifi_connection(wifi_controller).expect("Failed to spawn Wi-Fi connection task"));
    spawner.spawn(network_task(runner).expect("Failed to spawn network task"));

    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        info!("Network ready with IPv4 address {}", config.address);
    }

    let tcp_client = TcpClient::new(
        stack,
        mk_static!(
            TcpClientState<1, 1500, 1500>,
            TcpClientState::<1, 1500, 1500>::new()
        ),
    );
    let dns_client = DnsSocket::new(stack);
    let rx_buf = mk_static!([u8; drive::RX_BUF_SIZE], [0u8; drive::RX_BUF_SIZE]);
    spawner.spawn(
        drive::drive_task(tcp_client, dns_client, rx_buf).expect("Failed to spawn drive task"),
    );

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}

#[embassy_executor::task]
async fn wifi_connection(mut controller: WifiController<'static>) {
    loop {
        match controller.connect_async().await {
            Ok(info) => {
                info!("Wi-Fi connected: {:?}", info);
                let _ = controller.wait_for_disconnect_async().await;
                info!("Wi-Fi disconnected");
            }
            Err(error) => {
                info!("Wi-Fi connection failed: {:?}", error);
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn network_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await;
}
