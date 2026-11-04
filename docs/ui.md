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

Keep one owner of the display in the main application task. Other tasks publish
data or events through the existing Embassy watches and channels. The UI owner
will hold a small `Screen` enum (`Clock`, `Launcher`, `Settings`, `About`, and
`Update`) and screen-local state such as menu selection. A fixed-capacity
navigation stack is enough for returning to earlier screens; it cannot grow
without bound. Use an enum and match-based dispatch first. A trait for every
screen can be added if concrete screen implementations become unwieldy.

Move drawing and its last-rendered values from `main.rs` into a `ui` module.
The application task should translate clock, battery, BLE, OTA, button, and
future touch events into UI state changes, then ask the active screen to draw.
On a screen transition or wake, clear and draw the new screen while the
backlight is off. Within a screen, invalidate only changed fields: for
example, seconds each tick, the date at midnight, battery when sampled, and
BLE state on connection changes. OTA progress is a separate UI state; a
transfer percentage must not imply that image validation or activation passed.

## Input order

The current touch implementation in `src/device/input.rs` is a placeholder and
never produces events. Preserve the present button behavior: short press
toggles the display, and a three-second hold restarts into the bootloader menu
on MCUBoot builds. Implement and hardware-test the PineTime touch controller
before making a launcher or settings screen the main navigation path. Normalize
touch samples into tap and swipe events at the driver boundary, then route them
through the screen state machine. A button press can act as Back when a page is
open, after that behavior has been reviewed on hardware.

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

## Review sequence

1. Extract the existing clock drawing into a UI module without changing its
   appearance or sleep behavior. Keep the watch-face date work in this PR
   independent of that refactor.
2. Add the touch driver and a small host-tested input-to-navigation state
   machine. Test tap, swipes, wake, and bootloader hold on the development
   watch. Keep the daily driver on a known-good build until this is stable.
3. Add the launcher and About screen; then settings and notifications. Measure
   flash size and static RAM on every firmware PR, and use host-rendered
   previews for layout review where useful.

References: [embedded-graphics](https://docs.rs/embedded-graphics/0.8.2/embedded_graphics/),
[embedded-layout](https://docs.rs/embedded-layout/0.4.2/embedded_layout/),
[embedded-text](https://docs.rs/embedded-text/0.7.3/embedded_text/),
[embedded-graphics-simulator](https://docs.rs/embedded-graphics-simulator/0.8.0/embedded_graphics_simulator/),
[Slint on microcontrollers](https://docs.slint.dev/latest/docs/rust/slint/docs/mcu/).
