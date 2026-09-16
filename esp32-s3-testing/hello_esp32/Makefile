SHELL := /bin/bash

.PHONY: build release flash monitor clean

build:
	source ~/export-esp.sh && rustup run esp cargo build

release:
	source ~/export-esp.sh && rustup run esp cargo build --release

flash: release
	# Uses the project's runner which calls espflash
	source ~/export-esp.sh && rustup run esp cargo run --release

monitor:
	# Open serial monitor (adjust port if needed)
	source ~/export-esp.sh && espflash monitor target/xtensa-esp32s3-espidf/release/WIFI_test_esp32 || true

clean:
	cargo clean
