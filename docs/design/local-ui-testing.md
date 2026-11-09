# First local Mocha firmware iteration

Development branch: `design/mocha-tui-exploration`. No PR or release is intended
until the layout/input behavior has been tested on the ST-Link development watch.

## Implemented

- Black background with Catppuccin Mocha foregrounds, two-pixel rounded panel
  outlines and ISO-8859-1 10 × 20 text, revised after the first LCD test.
- After the next LCD reviews: mauve clock digits without inactive segments, mauve
  panel headings, blue outlines and colored status icons.
  Ordinary text stays light and unavailable controls remain muted.
- Status icons appear only on the clock. Other pages use a full-height frame
  with its title at the top; background BLE/notification updates cannot overwrite
  that header. About still shows battery details in its content area.
- Seven-segment clock, combined weekday/date title and an unread-notification bell
  without a count. The bell clears after explicit inbox navigation/interaction
  or returning home from the notification preview; stored messages remain.
- Only the step field is shown by default (`--` until the step source exists).
  Temperature/humidity placeholders are removed; future optional fields fill
  the two-column layout when their integrations are enabled.
- Full-height pages now reflow their contents too: settings hit targets move
  with the labels, home buttons become taller and notifications get a fourth
  body line. Firmware confirmation retains two distinct targets using shared
  bounds for rendering and hit testing.
- Home controls to the left, media to the right, notifications above and settings
  below the clock. These refer to page positions; e.g. swipe left opens Media.
- Step area opens activity; the bell opens notifications.
- Bluetooth is blue while connected and hidden while disconnected. About shows
  the same factory address bytes used by the BLE stack, in phone display order.
- Firmware base version is now 0.4.0. See the [release gates](../release-notes/v0.4.0.md)
  before publishing a stable release or tagging a reviewed master commit as RC.
- Double button press opens the media page. Single press wakes/returns home;
  from the awake clock, sleep waits up to 350 ms to allow a second press.
- Existing three-second recovery hold remains reserved. A button held at boot
  must still be released before recovery input is armed.
- Settings have brightness and vibration; Display has battery percentage and
  English/Bokmål selection. Preferences currently live in RAM and reset on boot.
- Latin-1 notification characters survive parsing and wrap at UTF-8 boundaries.
  Unsupported glyphs get `?`. The initial language pass translates common UI
  labels/date; some firmware diagnostics remain English.

## Intentionally not connected yet

The Home and Media pages currently show unavailable/unconfigured states. They
do not execute actions. No new BLE service has been added in this iteration.
InfiniTime-compatible media commands and Furu's Android media bridge are the
next implementation step after basic screen/button validation. Home Assistant,
real remote sensor values, step sampling and shared LittleFS writes are later.
Home controls remain reachable in this local prototype. When the Furu integration
is implemented, hide that page unless enabled for this watch; disabled clock
slots must not navigate to an unconfigured integration page.

## Build and flash

Inside devenv, `bash scripts/flash_app.sh` builds `mcuboot,ota-activation` by
default, checks the bootloader, erases the full application slot and programs
the image. It uses sudo only for OpenOCD. Do not run the entire script as root.
The agent session can see ST-Link but currently cannot write its root-owned USB
device, and its sudo invocation requires a password. Hardware flash therefore
needs the maintainer's terminal unless USB permissions are configured separately.

This work does not require `battery-persist-experimental`. Optional
`touch-wake-debug` is useful when testing without a working button, but keep it
out of release/OTA artifacts.

## Suggested physical checks

1. Verify colors, panel borders, date fit and 20 px font legibility at all three
   brightness levels. Assess inactive segments on the LCD, not just a PNG.
2. Let the watch sleep, then wake with one press: clock appears without waiting
   for the double-press timeout. Double press from both asleep/awake states and
   from another page opens Media. Single press from Media returns to the clock.
3. Single press on the awake clock sleeps after the short double-press window.
   Hold still enters bootloader recovery without opening Media first.
4. Check home/notification reverse swipes and the step target. In notifications,
   right swipe goes to a newer item, then home when already at the newest item.
5. Change brightness/vibration/battery percentage: only the affected row should
   redraw. Language change deliberately redraws the page once.
6. Send a Furu test notification containing `Blåbær: På vei hjem. Æ Ø Å é ü ñ`.
   Verify accent characters and the unread bell; verify BLE stays connected
   during sleep and still supports CTS/DFU.
7. Verify System → Firmware still requires two distinct confirmation taps;
   leave/back/home cancels the confirmation dialog. Do not confirm a trial
   merely to test navigation.

## Host checks and pixel previews

`devenv shell -- bash scripts/check.sh` runs the normal firmware matrix plus
renderer/input tests. Host tests now compile the actual UI renderer and its
existing graphics dependencies from the shared lockfile. No simulator or new
runtime dependency was added to firmware.

For PPM images generated from the actual renderer:

```sh
devenv shell -- env KONGLE_UI_PREVIEW_DIR=/tmp/kongle-ui-render \
  cargo test --locked --manifest-path tests/host/Cargo.toml \
  --target x86_64-unknown-linux-gnu renderer_fits_pages
```

These validate placement and field redraws. They cannot validate LCD settling,
perceived brightness, real switch bounce or the bootloader recovery menu.
