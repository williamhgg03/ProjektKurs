#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Runner, StackResources,
};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println as _;
use esp_radio::wifi::{
    AuthenticationMethod, Config as WifiConfig, ControllerConfig, Interface, WifiController,
    sta::StationConfig,
};
use alloc::string::ToString;
use reqwless::{
    client::HttpClient,
    request::Method,
};

extern crate alloc;

mod system;
mod lcd;
mod touch;

macro_rules! mk_static {
    ($t:ty, $value:expr) => {{
        static CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        CELL.uninit().write($value)
    }};
}

const WIFI_SSID: &str = match option_env!("WIFI_SSID") {
    Some(value) => value,
    None => "",
};
const WIFI_PASS: &str = match option_env!("WIFI_PASS") {
    Some(value) => value,
    None => "",
};
const SERVER_URL: &str = match option_env!("SERVER_URL") {
    Some(value) => value,
    None => "http://sjumilaskogen.se/",
};

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

    let m_Peripherals = system::hardware::init();
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let m_Timg0 = system::timer::init(m_Peripherals.TIMG0);
    let m_SwInterrupt = system::interrupt::init(m_Peripherals.SW_INTERRUPT);
    esp_rtos::start(m_Timg0.timer0, m_SwInterrupt.software_interrupt0);

    info!("Embassy initialized!");

    if WIFI_SSID.is_empty() || WIFI_PASS.is_empty() {
        panic!("WIFI_SSID and WIFI_PASS must be set when building the firmware");
    }

    let wifi_config = WifiConfig::Station(
        StationConfig::default()
            .with_ssid(WIFI_SSID)
            .with_auth_method(AuthenticationMethod::Wpa2Personal)
            .with_password(WIFI_PASS.to_string()),
    );

    let (wifi_controller, interfaces) = esp_radio::wifi::new(
        m_Peripherals.WIFI,
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

    spawner.spawn(
        wifi_connection(wifi_controller)
            .expect("Failed to spawn Wi-Fi connection task"),
    );
    spawner.spawn(
        network_task(runner)
            .expect("Failed to spawn network task"),
    );

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
    spawner.spawn(
        http_probe(tcp_client, dns_client)
            .expect("Failed to spawn HTTP task"),
    );

    let lcd = lcd::display::Lcd::new(
        m_Peripherals.SPI2,
        m_Peripherals.GPIO47,
        m_Peripherals.GPIO38,
        m_Peripherals.GPIO21,
        m_Peripherals.GPIO18,
        m_Peripherals.GPIO17,
        m_Peripherals.GPIO10,
    );

    let touch = touch::touch::Touch::new(
        m_Peripherals.I2C0,
        m_Peripherals.GPIO6,
        m_Peripherals.GPIO7,
        m_Peripherals.GPIO13,
    );

    // TODO: Spawn some tasks
    spawner
        .spawn(lcd::display_task(lcd)
        .expect("Failed to spawn LCD task")
    );

    spawner.spawn(
        touch::touch_task(touch)
            .expect("Failed to create touch task"),
    );

    info!("LCD task spawned");

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
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

#[embassy_executor::task]
async fn http_probe(
    tcp_client: TcpClient<'static, 1, 1500, 1500>,
    dns_client: DnsSocket<'static>,
) {
    let mut client = HttpClient::new(&tcp_client, &dns_client);

    loop {
        let mut response_buffer = [0u8; 1024];
        match client.request(Method::GET, SERVER_URL).await {
            Ok(mut request) => match request.send(&mut response_buffer).await {
                Ok(response) => {
                    info!("HTTP server responded with status {}", response.status.0);
                    let _ = response.body().discard().await;
                }
                Err(_) => {
                    info!("HTTP request failed");
                }
            },
            Err(_) => {
                info!("HTTP request failed");
            }
        }

        Timer::after(Duration::from_secs(30)).await;
    }
}
