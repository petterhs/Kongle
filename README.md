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
- [ ] InfiniTime bootloader support
- [ ] Sleep mode and wake logic
- [ ] Touch and gestures
- [ ] Display menu and settings page
- [ ] Stopwatch
- [ ] Alarms
- [ ] Sensor services
- [ ] Vibration


## Toolchain

```bash
# Install the Cortex-M4F cross-compilation target
rustup target add thumbv7em-none-eabihf

# Build release firmware
cargo build --release

# (Optional) Run on a connected PineTime via your debug setup
cargo run --release
```

## Debug

Terminals:

  1. `sudo bash ./debug.sh`
  2. `cargo run --release`
  3. `nc localhost 6969 | defmt-print -e target/thumbv7em-none-eabihf/release/kongle`

## Inspiration

- [InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime)
- [lupyuen](https://github.com/lupyuen/pinetime-rust-mynewt)
- [pinetime-rs (jonlamb-gh)](https://github.com/jonlamb-gh/pinetime-rs)
- [pinetime-rs (Robbe7730)](https://github.com/Robbe7730/pinetime-rs)
- [watchful](https://github.com/lulf/watchful)
