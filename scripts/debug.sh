#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Resolve the Nix store path before sudo resets PATH.
sudo "$(command -v openocd)" -f "${SCRIPT_DIR}/openocd-stlink.ocd" -c 'init; rtt start; rtt server start 6969 0; reset run'

# In another terminal: cargo run --release (for standalone) or just connect to RTT.
# To see defmt output, use the ELF that matches what is running:
#   Standalone (cargo run --release): nc localhost 6969 | defmt-print -e target/thumbv7em-none-eabihf/release/kongle
#   MCUBoot (flashed via flash_app.sh): nc localhost 6969 | defmt-print -e target/mcuboot/thumbv7em-none-eabihf/release/kongle
