# Kongle

> **Kongle** *(Norwegian, noun)*  
> **Meaning**: pinecone; cone (the fruit of conifer trees).

Kongle is an [Embassy](https://embassy.dev/)-based Rust firmware for the [PineTime smartwatch](https://wiki.pine64.org/wiki/PineTime) by [Pine64](https://www.pine64.org/), focused on async, low-power operation and experimentation with modern embedded Rust.

## Feature Implementation

- [x] Simple watchface
- [x] BLE
  - [x] trouble-host using apache-nimble
  - [x] Simple time sync
  - [ ] Battery reporting
  - [ ] Steps
  - [ ] Notifications
  - [ ] Heartbeat sensor
  - [ ] Music Service
  - [ ] OTA updates (DFU)
- [x] InfiniTime bootloader development support (MCUBoot boot and debugger flashing; BLE DFU and image confirmation remain incomplete)
- [ ] Sleep mode and wake logic
- [ ] Touch and gestures
- [ ] Display menu and settings page
- [ ] Stopwatch
- [ ] Alarms
- [ ] Sensor services
- [ ] Vibration

## Toolchain

```bash
# Load the pinned toolchain, Cortex-M4F target, and host tools
devenv shell

# Build release firmware
cargo build --release --locked

# (Optional) Run on a connected PineTime via your debug setup
cargo run --release
```

## Debug

Enter `devenv shell`; it provides the pinned `defmt-print` RTT decoder and its
runtime libraries through Nix. Run the scripts below without sudo;
they resolve OpenOCD from devenv before using sudo for hardware access.

Terminals:

  1. `bash scripts/debug.sh` (uses sudo only for OpenOCD)
  2. `cargo run --release`
  3. `nc localhost 6969 | defmt-print -e target/thumbv7em-none-eabihf/release/kongle`

## Two ways to run on dev kit with debugger

- **Standalone (no bootloader)** – for rapid development: `cargo run --release` builds and flashes the firmware at `0x00000000`. Use this with the debug workflow above.
- **With MCUBoot bootloader** – for testing with the bootloader use `scripts/flash_app.sh`, which builds with `--features mcuboot` (FLASH at `0x08200`) and flashes the wrapped image at `0x00008000`.

## InfiniTime MCUBoot bootloader support

Kongle can be run under the [Pinetime MCUBoot bootloader](https://github.com/InfiniTimeOrg/pinetime-mcuboot-bootloader).

- **Flash the bootloader (once per device)**
  From the repository root, with your debugger connected:

  ```bash
  bash scripts/flash_bootloader.sh
  ```

- **Build and flash the application image**

  ```bash
  bash scripts/flash_app.sh
  ```

  (Run without `sudo` so the build sees your devenv env; the script uses `sudo` only for OpenOCD.)

  This builds the firmware with `--features mcuboot` (separate from the standalone build), wraps it as an **unsigned MCUBoot image** (header `0x200`, slot `0x74000`, version from `Cargo.toml`), and programs and verifies it at `0x00008000` via OpenOCD. To build without flashing, run `bash scripts/build-mcuboot-image.sh`. The flash script resets the watch and exits. For logs, run `bash scripts/debug.sh` in one terminal, then connect to RTT from another:

  ```bash
  nc localhost 6969 | defmt-print -e target/mcuboot/thumbv7em-none-eabihf/release/kongle
  ```

On reset, the MCUBoot bootloader should show its logo and then start Kongle. Immediately before handing off to the application, the bootloader starts a 7-second hardware watchdog; Kongle feeds it every second when it detects the bootloader has already started it.

This is a debugger-flashed development baseline. With the opt-in `ota-staging`
feature, Kongle can receive and verify a Nordic legacy DFU image in external
flash, but it deliberately does not activate that image or confirm a trial boot.
Watchdog feeding does not confirm an update. Use
`scripts/build-dfu-package.sh` to create a local test ZIP; ordinary builds do
not enable staging.

The [OTA development sequence](docs/ota.md) describes the opt-in BLE staging
receiver, watch progress UI, long-hold restart, and the gated activation code.
Staging deliberately stops before installation; Kongle-to-Kongle OTA is not yet
complete.

## Contributing

Run `devenv shell -- bash scripts/check.sh` for the same checks as CI: formatting,
host regression tests for CTS parsing and display bounds, Clippy, both release layouts, image hash verification, shell syntax, and bootloader
checksum. PRs and pushes to `master` upload the MCUBoot BIN and its matching ELF
as development artifacts. CI also launches the RTT decoder to check its host
runtime. These are not Nordic DFU ZIPs or published releases.

Before merging boot or flashing changes, use the ST-Link development device to
check standalone boot, MCUBoot boot, display and BLE time sync, and operation
past the bootloader watchdog timeout. These hardware checks are separate from CI.

Use feature branches and pull requests targeting `master`; the maintainer reviews and merges changes. Keep hardware results separate from build checks in PR descriptions. Run `cargo fmt --check`, `cargo clippy --locked --release --features mcuboot`, and both standalone and MCUBoot builds inside `devenv shell`. The lockfile pins the development environment; after upgrading the devenv CLI, `devenv update devenv` updates its modules without intentionally upgrading the other inputs.

## Inspiration

- [InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime)
- [lupyuen](https://github.com/lupyuen/pinetime-rust-mynewt)
- [pinetime-rs (jonlamb-gh)](https://github.com/jonlamb-gh/pinetime-rs)
- [pinetime-rs (Robbe7730)](https://github.com/Robbe7730/pinetime-rs)
- [watchful](https://github.com/lulf/watchful)
