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
phone time on reconnect. The percentage has 1% resolution and is interpolated
between approximate discharge-curve points, rather than mapping 3.2–4.2 V
linearly. The clock, BLE and new history samples use the same estimate. Raw
millivolts are retained; previous recorded percentages are not rewritten.

## Percentage model and measurement

The initial reference points are (mV, %): (3500, 0), (3616, 3), (3723, 22),
(3776, 48), (3979, 79), (4180, 100). These numeric observations come from the
[InfiniTime 1.16.1 battery controller](https://github.com/InfiniTimeOrg/InfiniTime/blob/1.16.1/src/components/battery/BatteryController.cpp),
following its [measured discharge-curve work](https://github.com/InfiniTimeOrg/InfiniTime/pull/1444).
Kongle independently implements bounded integer interpolation. This supplies a
PineTime-specific starting point, not a curve fitted to this user's battery.
For example, 3.600 V changes from the old 40% to about 2%; it is not a newly
introduced shutdown threshold. No low-voltage BLE disconnect or reset is added.

Unlike InfiniTime, this first change does not latch the displayed percentage to
only decrease while discharging, use its power-present/full-charge pins, or
assume a measured hours-remaining value. Voltage rebound after load and charging
voltage can still affect the estimate. A worn battery, temperature, load and
measurement error can change usable capacity. Better percentage estimation does
not increase physical runtime or diagnose a disconnect by itself.

The [PineTime divider](https://pine64.org/documentation/PineTime/Hardware/) uses
two 1 MOhm resistors, equivalent to about 500 kOhm source impedance. The SAADC
channel now acquires for 40 us, as required for that impedance by
[Nordic's nRF52832 specification](https://docs.nordicsemi.com/r/bundle/ps_nrf52832/page/saadc.html)
(10 us supports up to 100 kOhm; 40 us up to 800 kOhm). Gain, reference and
12-bit resolution remain unchanged. The integer ADC conversion multiplies before
dividing to retain single-millivolt steps. Sampling/history cadence is unchanged.
The full-charge ADC conversion is not a voltage calibration against a meter.

## Validation and further calibration

Compare before/after raw voltage near the last reported 3.600 V. The maintainer
found the watch empty roughly an hour after seeing it alive; 3.600 V was the last
value in Furu, not a measured shutdown voltage. Do not infer a cutoff from it.
Record whether future failures are power loss, reset, or BLE-only disconnects. Keep the
initial stable release separate from this untested measurement/model change.
Host checks cover curve anchors, interpolation, monotonicity, limits, all 101
percentage values, and ADC conversion; ARM build checks the channel setting.
These checks cannot validate battery capacity or the physical ADC on a watch.

Record several full, uninterrupted discharge runs with the firmware version,
charging transitions, millivolts and the real last-alive endpoint, ideally on more
than one device. A disconnect alone must not be labelled empty. Use those runs
to compare this reference curve with Kongle's measured voltage-versus-time shape.
For a future time-remaining estimate in Furu, segment at charging/reboots and
large data gaps, use actual mV history rather than old percentages from a changed
curve, and report insufficient data until a useful discharge segment exists.
Linear extrapolation of percentage or voltage across the whole cycle is not a
reliable lithium-battery runtime model.

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
