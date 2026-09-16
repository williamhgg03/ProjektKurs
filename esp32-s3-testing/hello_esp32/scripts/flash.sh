#!/usr/bin/env bash
set -euo pipefail

# Flash the release build to the connected ESP32-S3
source "${HOME}/export-esp.sh"
rustup run esp cargo run --release
