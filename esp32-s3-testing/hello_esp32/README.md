# WIFI_test_esp32

Rust (std, ESP-IDF v5.5.3) firmware for an Arduino Nano ESP32 (ESP32-S3) that drives two
DC motors through a TB6612FNG driver. It long-polls `server/` for the motor state set on
the website and stops both motors (STBY low) whenever the server can't be reached.

## Wiring

| TB6612 | Nano ESP32 | GPIO   |
|--------|------------|--------|
| PWMA   | A0         | GPIO1  |
| AIN1   | A7         | GPIO14 |
| AIN2   | A3         | GPIO4  |
| PWMB   | A1         | GPIO2  |
| BIN1   | A5         | GPIO12 |
| BIN2   | A6         | GPIO13 |
| STBY   | A4         | GPIO11 |

The pins are set in `main.rs`. If a motor spins the wrong way, swap its IN1/IN2 pins
there (or swap the motor's wires). PWM runs at 20 kHz.

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
