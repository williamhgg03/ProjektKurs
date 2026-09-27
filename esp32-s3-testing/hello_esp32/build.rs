// Injects WiFi credentials and the server URL at compile time so they never
// live in source control. Values come from the environment first, then from
// the gitignored `wifi.env` file (see `wifi.env.example`).

use std::collections::HashMap;

const SECRETS_FILE: &str = "wifi.env";
const KEYS: [&str; 3] = ["WIFI_SSID", "WIFI_PASS", "SERVER_URL"];

fn main() {
    embuild::espidf::sysenv::output();

    println!("cargo:rerun-if-changed={SECRETS_FILE}");
    let file = std::fs::read_to_string(SECRETS_FILE)
        .map(|s| parse_env(&s))
        .unwrap_or_default();

    for key in KEYS {
        println!("cargo:rerun-if-env-changed={key}");
        let value = std::env::var(key)
            .ok()
            .or_else(|| file.get(key).cloned())
            .unwrap_or_else(|| {
                panic!("{key} is not set: copy wifi.env.example to wifi.env and fill it in, or export {key}")
            });
        let value = if key == "SERVER_URL" {
            value.trim_end_matches('/').to_string()
        } else {
            value
        };
        println!("cargo:rustc-env={key}={value}");
    }
}

/// Minimal KEY=VALUE parser: skips blank lines and `#` comments, strips optional quotes.
fn parse_env(s: &str) -> HashMap<String, String> {
    s.lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter_map(|l| l.split_once('='))
        .map(|(k, v)| (k.trim().to_string(), v.trim().trim_matches('"').to_string()))
        .collect()
}
