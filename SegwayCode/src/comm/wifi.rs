// WiFi station setup, from esp32-s3-testing/hello_esp32 (WPA2-Personal or WPA2-Enterprise).

use esp_idf_svc::sys::{self, esp};
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};

// Injected by build.rs from wifi.env or the environment, never stored in source
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");
// Only set for WPA2-Enterprise networks (eduroam); unset means normal WPA2-Personal
const WIFI_EAP_USERNAME: Option<&str> = option_env!("WIFI_EAP_USERNAME");
const WIFI_EAP_IDENTITY: Option<&str> = option_env!("WIFI_EAP_IDENTITY");

pub type Wifi = BlockingWifi<EspWifi<'static>>;

/// Configure and start the radio. Fails only on bad configuration, so it runs once at boot.
pub fn start(wifi: &mut Wifi) -> anyhow::Result<()> {
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
        log::info!("WiFi: WPA2-Enterprise, SSID {WIFI_SSID}");
    } else {
        wifi.set_configuration(&Configuration::Client(ClientConfiguration {
            ssid,
            password: WIFI_PASS
                .try_into()
                .map_err(|_| anyhow::anyhow!("password too long"))?,
            ..Default::default()
        }))?;
        log::info!("WiFi: WPA2-Personal, SSID {WIFI_SSID}");
    }

    wifi.start()?;
    // Modem sleep adds 100-300 ms latency per exchange
    esp!(unsafe { sys::esp_wifi_set_ps(sys::wifi_ps_type_t_WIFI_PS_NONE) })?;
    Ok(())
}

/// Connect (or reconnect) and wait for an IP address.
pub fn connect(wifi: &mut Wifi) -> anyhow::Result<()> {
    wifi.connect()?;
    wifi.wait_netif_up()?;
    let ip = wifi.wifi().sta_netif().get_ip_info()?;
    log::info!("WiFi connected, IP: {}", ip.ip);
    Ok(())
}

/// Sets up 802.1X (PEAP/TTLS with MSCHAPv2) credentials for networks like eduroam.
/// No CA cert is set, so the RADIUS server's certificate is not verified; add
/// `esp_eap_client_set_ca_cert` with the institution's CA to enable that.
fn enable_enterprise(identity: &str, username: &str, password: &str) -> anyhow::Result<()> {
    // ESP-IDF copies the buffers, so they only need to live for the call
    unsafe {
        esp!(sys::esp_eap_client_set_identity(
            identity.as_ptr(),
            identity.len() as _
        ))?;
        esp!(sys::esp_eap_client_set_username(
            username.as_ptr(),
            username.len() as _
        ))?;
        esp!(sys::esp_eap_client_set_password(
            password.as_ptr(),
            password.len() as _
        ))?;
        esp!(sys::esp_wifi_sta_enterprise_enable())?;
    }
    Ok(())
}
