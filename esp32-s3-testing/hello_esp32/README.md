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
