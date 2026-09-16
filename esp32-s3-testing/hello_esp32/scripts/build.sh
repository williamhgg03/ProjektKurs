#!/usr/bin/env bash
set -euo pipefail

# Build with the 'esp' toolchain. Pass extra cargo args through.
source "${HOME}/export-esp.sh"
rustup run esp cargo build "$@"
