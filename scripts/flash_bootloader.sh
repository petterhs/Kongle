#!/bin/bash
set -euo pipefail

openocd -f ./openocd-stlink.ocd -f ./flash_bootloader.ocd