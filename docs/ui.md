# Watch UI direction

Kongle uses `embedded-graphics` for drawing and `mipidsi` for the ST7789
controller. Keep those as the rendering foundation. The `st7789` crate is still
listed in `Cargo.toml` but no firmware source imports it; remove it during a
dependency cleanup after checking the lockfile and both build layouts.

The nRF52832 has 64 KiB of RAM. A 240 × 240 RGB565 framebuffer alone would
need 115,200 bytes. The current firmware also uses RAM for BLE, async tasks,
and DFU. Draw directly to the display, retaining partial redraws for the clock
and connection status. Do not introduce a full framebuffer or allocator as a
default UI requirement.

## Screens and ownership

The main application task owns the display. Other tasks publish data or events
through Embassy watches and channels. The UI owns a `Page` enum (`Clock`,
`Launcher`, `Notifications`, `Settings`, and `About`) and a fixed two-entry
navigation stack. Screen transitions use match-based dispatch, without heap
allocation or a trait object.

Drawing and last-rendered values live in `src/ui.rs`. The application task
translates clock, battery, BLE, OTA, button, and touch events into UI state
changes. On a screen transition or wake, it clears and draws while the
backlight is off. On the clock page, it redraws time when hours or minutes
change, battery when sampled, and BLE state on connection changes. OTA
activity returns to the clock so progress remains visible; a transfer
percentage does not imply that validation or activation passed.

## Input order

`src/device/input.rs` configures the PineTime CST816S on TWIM1 and reads its
interrupt-driven gesture reports. `src/touch.rs` decodes taps and all four
swipes independently of hardware; `src/ui/navigation.rs` routes them between
pages. From the clock, up opens Launcher, down Notifications, right Settings,
and left About. Swipe down returns from Launcher, Settings, or About; swipe up
returns from Notifications. Launcher taps open Settings or
About. Short button press toggles the display; a three-second hold restarts
into the bootloader menu on MCUBoot builds. These inputs need validation on
the ST-Link watch before using this firmware on a closed watch.

By default, touch scanning stops when the display sleeps, then the controller
is reset and reinitialized after button wake. The `touch-wake-debug` feature
keeps scanning and wakes on a touch report. It cannot be combined with
`ota-staging` or `ota-activation`, so OTA packages cannot include it. Measure
its battery cost on the development watch before deciding whether a lower
power touch-wake mode should be offered to daily-driver devices.

## Dependency choices

- `embedded-graphics` already provides text, circles, arcs, and rounded
  rectangles without a framebuffer or allocator. Use these before adding a
  widget framework.
- `embedded-layout` 0.4.2 is an optional helper for alignment and linear
  layouts; add it when a concrete menu layout needs it.
- `embedded-text` 0.7.3 supports `embedded-graphics` 0.8 and is a candidate for
  wrapped notification text. Keep its optional ANSI support disabled unless
  needed.
- `embedded-graphics-simulator` 0.8 can be a host-only tool for screen previews
  and PNG exports. It does not belong in firmware dependencies.
- Defer `embedded-menu` and other widget crates until their interaction model
  matches the touch driver and proposed navigation. Avoid adopting a library
  solely to render a short list.
- Do not revive the old LVGL branch: it predates the present Embassy, BLE, and
  MCUBoot integration. Slint supports microcontrollers, including line-based
  rendering, but its bare-metal setup requires an allocator and a platform
  integration; it is a larger change than Kongle currently needs.

## Visual direction to decide next

- A seven-segment clock face is closest to a classic Casio LCD. The
  [`eg-seven-segment`](https://docs.rs/eg-seven-segment/0.2.0/eg_seven_segment/)
  crate draws scalable digits with `embedded-graphics` 0.8. A first prototype
  should draw `HH:MM` into the existing clock rectangle, then compare flash
  size and redraw time on hardware.
- A compact pixel font from `embedded-graphics` provides a dot-matrix look
  with no new crate. This is appropriate for small labels and a more restrained
  clock face.
- A custom bitmap font can recreate a specific bedside alarm-clock display,
  but it needs a licensed source font and a host conversion step. Keep its
  glyphs limited to digits and punctuation to control flash use.

The clock status row now uses 24-pixel battery-level and Bluetooth icons from
[`embedded-iconoir`](https://docs.rs/embedded-iconoir/0.2.3/embedded_iconoir/).
Only the 24-pixel resolution is enabled, and only referenced icons are linked.
Battery percentage remains beside the icon by default and can be hidden from
Settings for the current session. This setting is held in RAM and resets after
a reboot; persistent storage needs a separate design that does not interfere
with the bootloader or OTA slots. Review icon legibility and disconnected-state
contrast on the ST7789 panel before expanding the icon set. Host-rendered PNG
previews can help choose a direction, but the watch should decide final sizes.

References: [embedded-graphics](https://docs.rs/embedded-graphics/0.8.2/embedded_graphics/),
[embedded-layout](https://docs.rs/embedded-layout/0.4.2/embedded_layout/),
[embedded-text](https://docs.rs/embedded-text/0.7.3/embedded_text/),
[embedded-graphics-simulator](https://docs.rs/embedded-graphics-simulator/0.8.0/embedded_graphics_simulator/),
[Slint on microcontrollers](https://docs.slint.dev/latest/docs/rust/slint/docs/mcu/).
