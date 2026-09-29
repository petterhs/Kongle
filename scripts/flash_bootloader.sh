#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"
openocd -f "${SCRIPT_DIR}/openocd-stlink.ocd" -f "${SCRIPT_DIR}/flash_bootloader.ocd"
