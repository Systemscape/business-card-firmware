# BLE HID business card firmware (nRF52810)

default:
    @just --list

# Build the firmware
build:
    cargo build --release

# Build, flash and stream defmt logs via probe-rs
run:
    cargo run --release

# Print flash/RAM usage
size: build
    size target/thumbv7em-none-eabi/release/business-card-firmware

# Erase the whole chip (also clears stored bonds in the STORAGE region)
erase:
    probe-rs erase --chip nRF52810_xxAA --allow-erase-all

# Check formatting and build without flashing
check:
    cargo fmt --check
    cargo build --release
