#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

defmt-print --version
cargo fmt --check
cargo fmt --manifest-path tests/host/Cargo.toml --check
cargo test --locked --manifest-path tests/host/Cargo.toml --target "$(rustc -vV | sed -n 's/^host: //p')"
cargo clippy --locked --release --features mcuboot
cargo build --locked --release
bash scripts/build-mcuboot-image.sh
defmt-print -e target/thumbv7em-none-eabihf/release/kongle </dev/null
defmt-print -e target/mcuboot/thumbv7em-none-eabihf/release/kongle </dev/null
imgtool verify target/mcuboot/thumbv7em-none-eabihf/release/mcuboot/pinetime-mcuboot-app-image.bin
for script in scripts/*.sh; do
    bash -n "$script"
done
sha256sum --check bootloader/SHA256SUMS
