#!/bin/bash
set -euo pipefail

if [[ ${EUID} -eq 0 ]]; then
    echo "Do not run this script with sudo. It will use sudo only for OpenOCD when needed." >&2
    echo "Run: bash scripts/flash_app.sh" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# build-mcuboot-image.sh will run "cargo build --release --features mcuboot" itself

# Build a MCUBoot-wrapped image (as current user, so devenv env like LIBCLANG_PATH is available).
cd "${REPO_ROOT}"
bash "${SCRIPT_DIR}/build-mcuboot-image.sh"

# Only openocd needs sudo (hardware access).
sudo openocd -f "${SCRIPT_DIR}/openocd-stlink.ocd" -f "${SCRIPT_DIR}/flash_application.ocd"