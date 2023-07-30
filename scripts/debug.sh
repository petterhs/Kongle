#!/bin/bash
set -euo pipefail

openocd -f ./openocd-stlink.ocd -c 'init; rtt start; rtt server start 6969 0; reset; halt'

# Run "cargo run --release" in another terminal.

# Run "telnet localhost 6969" in another terminal. To see the output of the
# application.