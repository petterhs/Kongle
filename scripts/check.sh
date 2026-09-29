#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

cargo fmt --check
cargo clippy --locked --release --features mcuboot
cargo build --locked --release
bash scripts/build-mcuboot-image.sh
imgtool verify target/mcuboot/thumbv7em-none-eabihf/release/mcuboot/pinetime-mcuboot-app-image.bin
bash -n scripts/*.sh
sha256sum --check bootloader/SHA256SUMS
