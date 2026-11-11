# Kongle

> **Kongle** *(Norwegian, noun)*  
> **Meaning**: pinecone; cone (the fruit of conifer trees).

Kongle is an [Embassy](https://embassy.dev/)-based Rust firmware for the [PineTime smartwatch](https://wiki.pine64.org/wiki/PineTime) by [Pine64](https://www.pine64.org/), focused on async, low-power operation and experimentation with modern embedded Rust.

## Feature Implementation

- [x] Simple watchface
- [x] BLE
  - [x] trouble-host using apache-nimble
  - [x] Simple time sync
  - [x] Battery reporting (BAS percentage and Kongle voltage/charging history)
  - [ ] Steps
  - [x] Notifications (volatile four-alert inbox through Furu)
  - [ ] Heartbeat sensor
  - [ ] Music Service
  - [ ] OTA updates (DFU)
- [x] InfiniTime bootloader development support (MCUBoot boot and debugger flashing; BLE DFU and image confirmation remain incomplete)
- [ ] Sleep mode and wake logic
- [x] Touch and gestures (development hardware validation pending)
- [x] Basic launcher, notifications, settings, and About pages
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

# (Optional) Flash standalone firmware on a development board. This replaces
# any bootloader at address 0x00000000.
cargo run --release
```

## Debug

Enter `devenv shell`; it provides the pinned `defmt-print` RTT decoder and its
runtime libraries through Nix. Run the scripts below without sudo;
they resolve OpenOCD from devenv before using sudo for hardware access.

Standalone firmware terminals:

  1. `bash scripts/debug.sh` (uses sudo only for OpenOCD)
  2. `cargo run --release`
  3. `nc localhost 6969 | defmt-print -e target/thumbv7em-none-eabihf/release/kongle`

The small `B` at the top-right of the watch appears while a phone has an
active GATT connection. RTT logs `[adv]` connection setup, `[gatt] disconnected`
with the host's reason, and advertising restart. On a development board,
capture RTT alongside Android logcat to distinguish link loss from a watch
reset (`Kongle started` appears again). Closed watches have no RTT access.

Connection feedback uses one 180 ms vibration on connect and two short 50 ms
vibrations on disconnect, including while the screen sleeps. The Settings
Vibration switch controls these and notification vibrations. Device Information
also exposes PINE64, PineTime, the hardware family, and Kongle software version;
the exact PCB revision is not inferred.

The clock face shows 24-pixel battery-level and Bluetooth icons. Battery
percentage is shown by default; tap its row in Settings to toggle it for the
current session. Settings also offers Low, Medium, and High backlight levels;
tap a level to preview it. Brightness defaults to High and resets on reboot.
The About page shows the firmware
version, short build commit, battery percentage, and voltage. The firmware
version is also available through the standard BLE Device Information Service
(Firmware Revision String). The display and backlight sleep after 30 seconds of
inactivity. A short button press toggles the display; a three-second hold still
restarts into the MCUBoot button menu. BLE, the clock, battery sampling, and the
bootloader watchdog continue while the display is asleep. An incoming OTA
transfer wakes the display to show progress and keeps it awake through
validation. Measure battery runtime on hardware before relying on this as a
full low-power mode.

Kongle enables the nRF52832's REG1 DC/DC converter, as InfiniTime does. The
[PineTime V1 schematic](https://files.pine64.org/doc/PineTime/PinTime%20Schematic-V1.0-20191020.pdf)
shows the required DCC inductor. The effect on whole-watch battery life has
not yet been measured; compare similar battery-discharge runs in Furu.

The planned screen and menu architecture, input sequence, and graphics crate
choices are recorded in [Watch UI direction](docs/ui.md).

### Touch navigation

The CST816S touch controller is active while the screen is awake. From the
clock, swipe up for Launcher, down for Notifications, right for Settings, or
left for About. Swipe down to go back from Launcher, Settings, or About; swipe
up to return from Notifications. In Launcher, tap Settings or About. These pages
provide basic navigation and status. Furu can forward Android alerts to Kongle's
Alert Notification Service (New Alert, 0x2A46). A new alert wakes the display
and opens the Notifications page for 30 seconds, except during OTA. Swipe left
for older alerts, right for newer alerts, or up to return to the clock. The
inbox holds the four newest alerts in RAM and clears on reboot. Notification
content is clipped to the screen and non-ASCII characters display as `?`.
Enable notification access and forwarding for the Kongle device in Furu.
Persistent settings are future work. A short button
press still toggles the display, and a three-second hold restarts into the
MCUBoot menu.

The optional `touch-wake-debug` feature keeps touch scanning while the display
is asleep, so a tap or swipe can wake a development watch with a stuck button.
Its power use has not been measured, so release OTA packages leave it off.
Flash a ST-Link development watch that can also receive OTA updates with:

```bash
KONGLE_FEATURES=mcuboot,ota-activation,touch-wake-debug bash scripts/flash_app.sh
```

`touch-wake-debug` is independent of OTA support; a build with only
`mcuboot,touch-wake-debug` still rejects OTA. After flashing, use the MCUBoot
RTT command below. Look for `Touch CST816S initialized`, `Touch gesture`, and
`Touch wake` logs while checking all four swipe directions, taps, sleep/wake,
and the button hold. Test this on the ST-Link development watch before using it
on a closed device.

## Two ways to run on dev kit with debugger

- **Standalone (no bootloader)** – for rapid development: `cargo run --release` builds and flashes the firmware at `0x00000000`, replacing the bootloader if one is installed. Use this with the standalone debug workflow above.
- **With MCUBoot bootloader** – for testing with the bootloader use `scripts/flash_app.sh`, which builds with `--features mcuboot` (FLASH at `0x08200`) and flashes the wrapped image at `0x00008000`.

Do not run `cargo run --release` when debugging the MCUBoot image: GDB's `load`
command flashes the standalone firmware at `0x00000000` and replaces the
bootloader. Use the MCUBoot log procedure below instead. If you accidentally
ran it, recover the ST-Link development board from `devenv shell` with:

```bash
bash scripts/flash_bootloader.sh
bash scripts/flash_app.sh
```

Run these one at a time, with no other OpenOCD session using the ST-Link. The
bootloader should show its logo at startup. If it does not, inspect the
OpenOCD output and do not attempt an OTA transfer yet.

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

  This builds the firmware with `--features mcuboot,ota-activation` (separate from the standalone build), wraps it as an **unsigned MCUBoot image** (header `0x200`, slot `0x74000`, version from `Cargo.toml`), verifies that the pinned bootloader is installed, and programs and verifies the application at `0x00008000` via OpenOCD. To build without flashing, run `bash scripts/build-mcuboot-image.sh`. Set `KONGLE_FEATURES` explicitly for a different feature set; `mcuboot` alone rejects DFU staging requests. The flash script resets the watch and exits. For logs, run `bash scripts/debug.sh` in one terminal, then connect to RTT from another:

  ```bash
  nc localhost 6969 | defmt-print -e target/mcuboot/thumbv7em-none-eabihf/release/kongle
  ```

On reset, the MCUBoot bootloader should show its logo and then start Kongle. Kongle feeds the bootloader's hardware watchdog every second if it finds one already running. A log saying the watchdog is not running does not, by itself, establish whether MCUBoot ran.

This is a debugger-flashed development baseline. Activation requests an MCUBoot
test swap; Kongle does not confirm the new image, so resetting should revert.
For a package with matching features, run `bash scripts/build-dfu-package.sh`.
Use `KONGLE_FEATURES=mcuboot,ota-staging` only when testing reception without
activation; that build rejects final validation.

The [OTA development sequence](docs/ota.md) describes the opt-in BLE staging
receiver, watch progress UI, long-hold restart, and trial activation. Manual
confirmation is still to be implemented after rollback testing.

## Contributing

For host tests alone, enter `devenv shell` and select the host target explicitly:

```bash
cargo test --locked --manifest-path tests/host/Cargo.toml \
  --target "$(rustc -vV | sed -n 's/^host: //p')"
```

Plain `cargo test` inherits the embedded ARM target from `.cargo/config.toml`,
which cannot run the host tests or provide `std`. Pulse trace replay and capture
regressions are described in [docs/heart-rate.md](docs/heart-rate.md).

Run `devenv shell -- bash scripts/check.sh` for the same checks as CI: formatting,
host regression tests for CTS parsing and display bounds, Clippy, both release layouts, image hash verification, shell syntax, and bootloader
checksum. PRs and pushes to `master` upload the MCUBoot BIN and its matching ELF
as development artifacts. CI also launches the RTT decoder to check its host
runtime. These are not Nordic DFU ZIPs or published releases.

Before merging boot or flashing changes, use the ST-Link development device to
check standalone boot, MCUBoot boot, display and BLE time sync, and operation
past the bootloader watchdog timeout. These hardware checks are separate from CI.

Use feature branches and pull requests targeting `master`; the maintainer reviews and merges changes. Keep hardware results separate from build checks in PR descriptions. Run `cargo fmt --check`, `cargo clippy --locked --release --features mcuboot`, and both standalone and MCUBoot builds inside `devenv shell`. The lockfile pins the development environment; after upgrading the devenv CLI, `devenv update devenv` updates its modules without intentionally upgrading the other inputs.

CI DFU packages, tagged release candidates, stable releases, and changelog generation are described in [Firmware builds and releases](docs/releases.md).

## Inspiration

- [InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime)
- [lupyuen](https://github.com/lupyuen/pinetime-rust-mynewt)
- [pinetime-rs (jonlamb-gh)](https://github.com/jonlamb-gh/pinetime-rs)
- [pinetime-rs (Robbe7730)](https://github.com/Robbe7730/pinetime-rs)
- [watchful](https://github.com/lulf/watchful)
