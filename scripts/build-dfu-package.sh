#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

bash scripts/build-mcuboot-image.sh
image=target/mcuboot/thumbv7em-none-eabihf/release/mcuboot/pinetime-mcuboot-app-image.bin
version="$(python -c 'import tomllib; print(tomllib.load(open("Cargo.toml", "rb"))["package"]["version"])')"
output="target/mcuboot/thumbv7em-none-eabihf/release/mcuboot/kongle-mcuboot-app-dfu-${version}.zip"
python scripts/build-dfu-package.py "$image" "$output"
