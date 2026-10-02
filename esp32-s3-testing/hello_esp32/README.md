# WIFI_test_esp32

Rust (std, ESP-IDF v5.5.3) firmware for the ESP32-S3.

## One-time setup

```sh
cargo install espup ldproxy espflash
espup install        # installs the `esp` Rust toolchain for Xtensa
```

`rust-toolchain.toml` selects the `esp` toolchain automatically. There is no need to
source `~/export-esp.sh`: ESP-IDF and its compilers are downloaded into `.embuild/`
on the first build (slow, needs internet, plus `git`, `cmake` and `python`).

## Build / flash

```sh
cargo build                # debug build
cargo build --release      # release build
cargo run --release        # flash with espflash and open the serial monitor
```

VS Code: `Tasks: Run Task` → `Build (esp)`, `Build (esp) Release` or `Flash + Monitor (esp)`.

## WiFi

Credentials go in `wifi.env` (copy `wifi.env.example`). It selects the network type:

- **Normal WiFi**: set `WIFI_SSID` and `WIFI_PASS`.
- **Eduroam / WPA2-Enterprise**: also set `WIFI_EAP_USERNAME` (usually `user@university.se`),
  with `WIFI_PASS` as the eduroam password. `WIFI_EAP_IDENTITY` optionally sets an
  anonymous outer identity. Remove `WIFI_EAP_USERNAME` to go back to normal WiFi.

Rebuild after changing `wifi.env`. The RADIUS server certificate is not verified.
On eduroam, other devices (e.g. the laptop running `server/`) may not be reachable
from the chip, so `SERVER_URL` might need to point at a public server.

## Buzzer

A passive piezo buzzer on GPIO2 plays melodies uploaded in the server's web UI
(Buzzer section). The server turns the MP3 (or WAV/OGG/FLAC) into a list of notes
and the chip plays them as square waves with LEDC, while the LED loop keeps running.

Wiring: GPIO2 → ~100 Ω → piezo +, piezo − → GND. A 3-pin passive buzzer module
goes S → GPIO2, plus 3V3 and GND. An active buzzer (one that beeps on plain DC)
will not work.
