use core::time::Duration;
// removed unused io imports

fn main() {
    // Required patching for esp-idf runtime
    esp_idf_svc::sys::link_patches();

    // Initialize logger
    esp_idf_svc::log::EspLogger::initialize_default();

    // Edit these with your network credentials
    const WIFI_SSID: &str = "Kalle";
    const WIFI_PASS: &str = "ugaf465k";

    log::info!("Starting WiFi connect + ping example");

    // Initialize NVS and event loop (netif will be created by EspWifi)
    let _nvs = esp_idf_svc::nvs::EspDefaultNvsPartition::take_with(false).expect("failed to init NVS partition");
    let sysloop = esp_idf_svc::eventloop::EspSystemEventLoop::take().expect("failed to take sysloop");

    // Acquire modem peripheral and create the Wifi service
    let modem = unsafe { esp_idf_svc::hal::modem::Modem::steal() };
    let mut wifi = esp_idf_svc::wifi::EspWifi::new(modem, sysloop, Some(_nvs)).expect("failed to create wifi");

    use esp_idf_svc::wifi::{Configuration, ClientConfiguration};


    let mut client_conf = ClientConfiguration::default();
    client_conf.ssid = WIFI_SSID.try_into().expect("SSID too long");
    client_conf.password = WIFI_PASS.try_into().expect("Password too long");

    wifi.set_configuration(&Configuration::Client(client_conf)).expect("failed to set wifi config");
    wifi.start().expect("failed to start wifi");
    wifi.connect().expect("failed to connect");

    log::info!("Waiting for connection (30s)...");
    let start = std::time::Instant::now();
    while start.elapsed() < Duration::from_secs(30) {
        if wifi.is_connected().unwrap_or(false) {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    if !wifi.is_connected().unwrap_or(false) {
        log::error!("Failed to connect to WiFi");
        return;
    }

    let ip = wifi.sta_netif().get_ip_info().expect("failed to get ip info");
    log::info!("Connected, IP: {:?}", ip.ip);

    // Use EspPing to ping Google DNS (8.8.8.8)
    let idx = wifi.sta_netif().get_index();
    let mut pinger = esp_idf_svc::ping::EspPing::new(idx);
    let cfg = esp_idf_svc::ping::Configuration { count: 4, ..Default::default() };

    match pinger.ping(esp_idf_svc::ipv4::Ipv4Addr::new(8, 8, 8, 8), &cfg) {
        Ok(summary) => log::info!("Ping summary: transmitted {}, received {}", summary.transmitted, summary.received),
        Err(e) => log::error!("Ping failed: {}", e),
    }

    log::info!("Done");
}
