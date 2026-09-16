# WIFI_test_esp32 — build & flash helper

Quick commands to build/flash the ESP32-S3 (requires `espup` setup and `source ~/export-esp.sh`).

Makefile targets (from project root):

- `make build` — build (debug) using the `esp` toolchain
- `make release` — build release
- `make flash` — build release and flash (runs the project's runner)
 - `make monitor` — open serial monitor (targets the `WIFI_test_esp32` release binary)

Scripts:

- `./scripts/build.sh` — same as `make build`
- `./scripts/flash.sh` — flash the release binary (now named `WIFI_test_esp32`)

VS Code: open the Command Palette -> `Tasks: Run Task` and pick `Build (esp)`, `Flash (esp)` or `Monitor (espflash)`.
