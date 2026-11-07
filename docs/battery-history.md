# Battery history and LittleFS trial

Kongle exposes standard Battery Service `0x180F` / Battery Level `0x2A19`.
Furu also reads the Kongle service `6b6f0000-1886-4b40-ae7d-00a11ce50001`:

| Characteristic suffix | Access | Value |
| --- | --- | --- |
| `0001` | read | voltage in mV (little-endian u16), percentage u8, charging u8 |
| `0002` | read | versioned history page |
| `0003` | write | u32 sequence cursor for the next page |
| `0004` | write | UTC epoch seconds (u32), anchoring watch uptime |

Page v1 contains version u8, first sequence u32, next sequence u32, count
u8, current watch uptime seconds u32, then up to 18 records. Each record has
uptime seconds u32, UTC epoch seconds u32 (zero if unsynced), voltage mV u16,
percentage u8 and charging u8. All multi-byte fields are little-endian.
The reader writes cursor zero initially, then each returned `next` until a
zero-count page. It may restart from zero after a disconnect; Furu deduplicates
samples near the same timestamp. No records are deleted on read.

The RAM ring keeps 512 samples, recorded every five minutes and on charging
state changes. This covers at least 42 hours at the base cadence, less when
many charging transitions occur. Furu anchors unsynced uptime samples to
phone time on reconnect. The percentage estimate now has 1% resolution, using
the original linear approximation from 0% at 3.2 V to 100% at 4.2 V. It is not
yet a calibrated lithium-battery curve. The clock, BLE and history all use this
same estimate; recording every five minutes is independent of its resolution.
Keep the voltage trend alongside it when assessing battery life and refining
the curve.

`battery-persist-experimental` mounts the existing InfiniTime LittleFS at
`0x0B4000..0x400000`. It **never formats** the partition. A mount failure
leaves the RAM history available and logs a warning. After Furu supplies UTC
time, new samples are appended to `kongle-battery.bin` with a CRC. The file
is rotated after 64 KiB, retaining the newest 512 records. Previous records
are reloaded after a watch reset. Samples taken before the first phone time
anchor are only in RAM; a reset before reconnect loses those samples.

Build for the **ST-Link development board** with
`KONGLE_FEATURES=mcuboot,ota-activation,battery-persist-experimental bash scripts/flash_app.sh`.
The normal OTA and release builds do not include flash persistence yet. First
test mounting, writes, reboot backfill, and InfiniTime rollback on the dev
board. Check RTT for mount/write errors and verify that InfiniTime's files
remain readable after rollback before enabling this feature in releases.
The optional C LittleFS binding is compiled with the pinned Arm toolchain in
`devenv.nix`; the ordinary build remains unchanged.

InfiniTime's [filesystem layout and configuration](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/fs/FS.cpp)
are the source of truth for this partition. Do not write raw sectors inside it.
