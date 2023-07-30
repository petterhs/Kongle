#!/bin/bash
set -euo pipefail

# Check if the application is built
if [ ! -f ../target/thumbv7em-none-eabihf/release/kongle ]
then
    echo "Application not built. Run \"cargo build --release\" first."
    exit 1
fi

# TODO: Create a binary that can be flashed to the device
echo "App that supports the bootloader is not implemented yet."
exit 1

openocd -f ./openocd-stlink.ocd -f ./flash_application.ocd