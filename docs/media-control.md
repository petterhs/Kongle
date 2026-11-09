# Media control (0.4.0 development)

The Media page now receives real phone metadata and sends previous, play/pause
and next events. Reach it by swiping left from the clock or double-pressing the
button. No phone/player means disabled controls. Metadata changes redraw only
the content/controls, without clearing the page or toggling the backlight.

## BLE compatibility

Use the [InfiniTime music protocol](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/doc/ble.md#music-control),
service `00000000-78fc-48fe-8e23-433b3a1942d0`. Characteristic suffixes below
replace the last two digits of the first UUID group:

| Suffix | Value | Access |
| --- | --- | --- |
| 01 | One-byte event | notify |
| 02 | Playing: 0 or 1 | read/write |
| 03/04/05 | Artist/title/album, up to 40 UTF-8 bytes | read/write |
| 06/07 | Position/duration, u32 big-endian seconds | read/write |
| 08/09 | Track number/track count, u32 big-endian | read/write |
| 0a | Speed ×100, u32 big-endian | read/write |
| 0b/0c | Repeat/shuffle, one-byte boolean | read/write |

Events used by the UI: play `0x00`, pause `0x01`, next `0x03`, previous `0x04`,
screen-open `0xe0`. Volume events are not exposed by this first UI. Numeric field
lengths and boolean values are validated; strings preserve supported Latin-1
text without splitting UTF-8. The service is appended after existing services.

Phone metadata is volatile and reset per BLE connection. Button requests carry
an internal connection generation, and requests from an old connection are
discarded. The GATT listener remains alive while CTS/battery updates and media
notifications run concurrently, preserving the DFU receive-loop fix.

## Furu and physical checks

Use Furu's `feature/media-control` build and enable Android notification access
for the installed Furu variant. This access is needed for Android media sessions;
global notification forwarding need not be enabled. Built-in profiles include
`ble.media_control`; custom profiles can select it explicitly.

Start Spotify or Absorb, then connect/open Media. Furu sends metadata only when
it changes, and forwards standard Android previous/next media keys. The player
decides whether that means tracks, chapters or a skip interval. Kongle never
hardcodes ten-second seeking or controls the phone over Bluetooth audio profiles.

Test title/status, all three buttons, phone lock/background, no-player state,
player switch, reconnect and OTA. ST-Link flashing can leave Android's GATT
cache stale; use Furu's Repair connection if the appended service is missing.
No APK/device playback test is implied by host, ARM or Kotlin compilation.
