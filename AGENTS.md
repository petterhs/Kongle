# AGENTS

## Purpose

This document is for automated agents, tooling, and contributors that interact with this repository programmatically.

It explains the technologies used and how the development environment is configured, with a special note about `devenv` and command availability.

## Technologies

- **Language**: Rust 2021, targeting embedded ARM (`thumbv7em-none-eabihf`) for the PineTime smartwatch (nRF52832).
- **Async framework**: `embassy-executor`, `embassy-nrf`, `embassy-time`, `embassy-sync`, `embassy-futures`, `embassy-embedded-hal`.
- **BLE stack**: `apache-nimble`, `trouble-host`, `trouble-host-macros`, `bt-hci`.
- **Display and graphics**: `embedded-graphics`, `st7789`, `mipidsi`.
- **Embedded HALs and utilities**: `embedded-hal`, `embedded-hal-async`, `embedded-hal-bus`, `embedded-io`, `embedded-io-async`, `embedded-storage`, `embedded-storage-async`, `heapless`, `chrono`, `futures`, `static_cell`, `byte-slice-cast`, `debouncr`.
- **Debugging and runtimes**: `defmt`, `defmt-rtt`, `panic-probe`, `cortex-m`, `cortex-m-rt`.

See `Cargo.toml` for the complete and authoritative list.

## Development environment (devenv)

This repository uses [`devenv`](https://devenv.sh/) (Nix-based) to provide a reproducible development environment.

- **Rust**: configured via `devenv.nix` to use a nightly toolchain with the `thumbv7em-none-eabihf` target.
- **Tooling**: packages such as `git`, `openocd`, `gcc-arm-embedded-13`, `probe-rs-tools`, `cargo-edit`, `clang-tools`, and `libclang` are provided by `devenv`, not assumed to be globally installed.
- **Tasks**: `devenv.nix` defines some helper tasks (for example running `openocd`), but these are **experimental** and not the primary, documented workflow. Prefer the debug steps in `README.md`.

## Debug and run workflow

The canonical, currently used workflow for running and debugging the firmware is documented in `README.md` under `## Debug` and uses three terminals:

1. `sudo bash scripts/debug.sh`
2. `cargo run --release`
3. `nc localhost 6969 | defmt-print -e target/thumbv7em-none-eabihf/release/kongle`

Notes for agents and tools:

- Treat these commands as the source of truth for how the firmware is currently tested on real hardware.
- They assume a non-sandboxed environment with access to USB/JTAG and the necessary host tools; many automated environments will **not** be able to execute them successfully.
- When possible, reason about the code rather than relying on actually running this full debug flow.

### Formatting and linting

- Use `cargo fmt` to format Rust code.
- Use `cargo clippy` for linting and catching common issues.
- These commands should be run **inside** the `devenv` environment so that the correct toolchain and targets are available.

### Important note for agents and sandboxed environments

- **Do not assume** that commands like `cargo`, `rustup`, `openocd`, or `probe-rs` are available in a generic or sandboxed shell.
- These commands are only guaranteed to exist **inside** the `devenv` environment.
- Human contributors should enter the environment using something like:

  ```bash
  devenv shell
  ```

- Automated agents (e.g. IDE-integrated or remote tools running in a sandbox) should:
  - Prefer reasoning about the code and configuration rather than relying on running build/flash/debug commands.
  - If commands must be run, invoke them through `devenv`, and **not** assume `cargo` or other tools are on the PATH by default.
  - Additional tools should be added to `devenv.nix` so they are available consistently for all contributors.

### MCUBoot bootloader (PineTime)

With `--features mcuboot`, Kongle is linked at `0x00008200` (see `memory_mcuboot.x`) for the [PineTime MCUBoot bootloader](https://github.com/InfiniTimeOrg/pinetime-mcuboot-bootloader). Without that feature, it uses `memory_standalone.x` at address zero. Run `bash scripts/build-mcuboot-image.sh` or `bash scripts/flash_app.sh` **inside** `devenv shell`; devenv provides objcopy and imgtool. The bootloader starts a watchdog (~7 s) before jumping to the application; Kongle adopts and feeds it. Feeding the watchdog is separate from confirming a trial image. BLE DFU reception and image confirmation are not implemented yet.

## Collaboration

Use feature branches and PRs targeting `master`; the maintainer reviews and merges manually. Use the configured Git identity without agent co-author trailers. Use the ST-Link-connected development device for hardware validation; closed devices include an InfiniTime daily driver. Report build checks and hardware checks separately, and review the collaboration process after the first few PRs.
