#!/bin/bash
set -euo pipefail

#openocd -f ./openocd-stlink.ocd -f ./flash_application.ocd

openocd -f ./openocd-stlink.ocd -c 'init; reset; halt'
