#!/usr/bin/env bash
set -euo pipefail

# Build a MCUBoot-wrapped image for the Kongle firmware.
#
# This script builds the firmware with --features mcuboot (FLASH at 0x08200),
# then wraps it with imgtool. It expects "imgtool" and the cross toolchain on PATH.
#
# It produces:
# - target/mcuboot/thumbv7em-none-eabihf/release/mcuboot/kongle.bin
# - target/mcuboot/thumbv7em-none-eabihf/release/mcuboot/pinetime-mcuboot-app-image.bin
#   (MCUBoot image to be flashed at 0x00008000)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TARGET=thumbv7em-none-eabihf
APP_NAME=kongle

# Use a separate target dir for MCUBoot so we don't overwrite the standalone release binary.
MCUBOOT_TARGET_DIR="${REPO_ROOT}/target/mcuboot"
ELF="${MCUBOOT_TARGET_DIR}/${TARGET}/release/${APP_NAME}"
OUT_DIR="${MCUBOOT_TARGET_DIR}/${TARGET}/release/mcuboot"
RAW_BIN="${OUT_DIR}/${APP_NAME}.bin"
IMAGE_BIN="${OUT_DIR}/pinetime-mcuboot-app-image.bin"

# If target/mcuboot was created by a previous "sudo flash_app.sh" run, the current user can't write to it.
if [[ -d "${MCUBOOT_TARGET_DIR}" ]] && ! [[ -w "${MCUBOOT_TARGET_DIR}" ]]; then
  echo "Build directory is not writable: ${MCUBOOT_TARGET_DIR}" >&2
  echo "Restore its ownership before building. Run this script without sudo." >&2
  exit 1
fi

echo "[build-mcuboot-image] Building release firmware with mcuboot layout..."
(cd "${REPO_ROOT}" && CARGO_TARGET_DIR="${MCUBOOT_TARGET_DIR}" cargo build --locked --release --features mcuboot)

if [[ ! -f "${ELF}" ]]; then
  echo "Release ELF not found at: ${ELF}" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"

# Pick an objcopy implementation provided by the toolchain.
if command -v arm-none-eabi-objcopy >/dev/null 2>&1; then
  OBJCOPY="arm-none-eabi-objcopy"
elif command -v llvm-objcopy >/dev/null 2>&1; then
  OBJCOPY="llvm-objcopy"
else
  echo "Neither arm-none-eabi-objcopy nor llvm-objcopy found on PATH." >&2
  echo "Make sure you are running inside the devenv shell so the cross toolchain is available." >&2
  exit 1
fi

echo "[build-mcuboot-image] Using objcopy: ${OBJCOPY}"

"${OBJCOPY}" -O binary "${ELF}" "${RAW_BIN}"

# MCUBoot slot parameters for PineTime:
# - Primary slot base:  0x00008000
# - Slot size:          0x74000 (464 KiB)
# - Header size:        0x200   (512 bytes)
#
# We create an *unsigned* MCUBoot image suitable for development.

if ! command -v imgtool >/dev/null 2>&1; then
  echo "'imgtool' command not found." >&2
  echo "Run this script inside devenv shell, which provides imgtool." >&2
  exit 1
fi

echo "[build-mcuboot-image] Creating MCUBoot image..."

VERSION=$(python -c 'import sys, tomllib; print(tomllib.load(open(sys.argv[1], "rb"))["package"]["version"])' "${REPO_ROOT}/Cargo.toml")

imgtool create \
  --align 4 \
  --header-size 0x200 \
  --pad-header \
  --slot-size 0x74000 \
  --version "${VERSION}" \
  "${RAW_BIN}" \
  "${IMAGE_BIN}"

echo "[build-mcuboot-image] Done."
echo "MCUBoot image: ${IMAGE_BIN}"
echo "Flash with: scripts/flash_app.sh"
