# Step measurement for Kongle 0.4.0

Status: the maintainer's read-only dev-watch probe returned chip ID `0x11`.
`feature/sensor-foundation` now initializes the matching Bosch engine and
displays raw cumulative counts. This active build still needs hardware testing;
daily totals, persistence and BLE reporting are not implemented yet.
Goal: reliable wrist-worn counting with low idle cost,
documented limitations and reproducible tests. Improvement over another firmware
must be demonstrated on measured cases, not inferred from a rewritten driver.

## What the sensor does

Bosch's [BMA421 product brief, BST-BMA421-FL000-00](https://files.pine64.org/doc/datasheet/pinetime/BST-BMA421-FL000.pdf)
describes on-sensor step counting, event interrupts, readable step registers and
a FIFO for acceleration samples. This lets the application sleep instead of
continuously processing raw acceleration. The brief is not a full register or
feature-configuration specification. Its current figures describe specified
sensor modes, not measured consumption of our complete watch.

The [InfiniTime driver](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/drivers/Bma421.cpp)
already reads the Bosch hardware counter, loads feature configuration, handles
BMA421/BMA425 identification and selects 100 Hz/±2g. This is a useful comparison
baseline, not the automatic choice of configuration for Kongle. Do not assume
an arbitrary lower sampling rate preserves step-engine accuracy.

The [Bosch-authored source distributed with InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/drivers/Bma421_C/bma423.c)
contains variant configuration blobs and a BSD-3-Clause license. Audit the exact
matching driver/header/blob revision and license before reuse. Do not substitute
BMA423/BMA456 configurations just because parts share a family name. The original
public BMA423 repository links were unavailable during this investigation; retain
reproducible source provenance rather than depending on an unverified mirror.

## 1. Identify and initialize our actual hardware

- Read/log chip ID, supported variant and initialization status on the dev watch
  and each battery-powered test watch. Unknown chips fail closed for step data;
  do not return a convincing zero or upload a guessed feature blob.
- Confirm schematic address/IRQ wiring and share I2C with touch and future HRM.
  Touch and motion now share TWIM1 through Embassy's asynchronous I2cDevice;
  transactions are serialized and touch IRQ waits stay outside the bus lock.
  Audit reset/bus recovery behavior, timeouts and
  the effect on the touch controller before adding sensor startup.
- Select a matching Bosch feature configuration with recorded source revision
  and checksum. Check all bus errors and feature-engine initialization status.
  Keep recovery/OTA/watchdog responsiveness during upload; yield between chunks.
- Read back the effective settings and record them with test results. Probe
  supported IRQ/count reporting options; verify behavior on hardware rather
  than importing watermark units or registers from a different Bosch part.

Deliverable: a minimal Rust driver that initializes the identified part and
reports a raw cumulative hardware counter, with independent transport tests.

### Current hardware test

The earlier read-only probe identified the dev sensor as `0x11` at address
`0x18`. The active build calls `Motion::initialize` with an eight-second timeout.
It uses the unchanged 6144-byte streams for IDs 0x11/0x13 with documented hashes
and BSD notices in `src/device/bosch/`. ASIC status, feature enable readback,
acceleration configuration and power registers must pass before counts are
published. Unknown chips receive no writes, including during error cleanup.

Flash from `/tmp/kongle-sensors` using the normal development feature set:

```sh
KONGLE_FEATURES=mcuboot,ota-activation,touch-wake-debug bash scripts/flash_app.sh
```

Capture `[motion] ... ready; config ID ..., accel 28, range 00` followed by
`[motion] raw steps: ...`, or the initialization error. Check touch, BLE and
screen sleep/wake during and after initialization. Test a small counted walk
and wait for delayed/batched sensor reporting. These values are sensor totals,
not daily counts; each firmware boot now resets them. They are intentionally
not exposed as daily steps through BLE yet.

The first active hardware test failed with the old generic `Readback` error;
touch continued to work. Initialization now follows Bosch's APS handoff after
configuration loading before fetching the feature-window pointer. This did not
fix the hardware failure: the next test reported `start: 3064, config_id: 0`,
with feature bytes remaining zero. 3064 is exactly the final upload word address
(`(6144 - 16) / 2`), consistent with attempting to reinitialize an ASIC that
retained its old state across an MCU reset.
Failures now identify `FeatureAddress`, `FeatureReadback` (start pointer,
configuration ID, expected/actual bytes) or `RegisterReadback` (register, mask,
expected/actual value), rather than concealing which check failed. These checks
remain strict; diagnostic detail is not a reason to accept invalid settings.

The shared TWIM buffer now holds 32 bytes. Reads happen every five seconds
awake and thirty seconds asleep, plus screen-state transitions; counting stays
on the sensor. This polling is a baseline, not a final power optimization.
Read errors publish unavailable rather than zero. Three consecutive errors stop
acquisition without resetting the shared bus or touch. Initialization/read
operations and best-effort shutdown have bounded timeouts.

Transport tests exercise both variants, upload failure, bad ASIC status,
unknown chips, feature readback failure and lost engine state. They establish
protocol handling, not physical initialization or counting accuracy. UI tests
check that count updates leave other regions/pages unchanged. Bosch notices
accompany the raw MCUBoot build output and are included in the DFU ZIP.

Startup now probes a known chip and sends soft reset before the shared bus or
touch tasks are created. A temporary TWIM driver uses bounded 20 ms blocking
transactions at this stage, then is dropped even on timeout. Buffers remain
alive until TWIM is disabled. After a 200 ms settling delay, TWIM is recreated
using owned/reborrowed peripherals without register access behind the HAL.
The motion driver requires fresh ASIC status zero before any upload; a stale
success status is `NeedsReset`, not proof that our own initialization succeeded.
Unknown chips receive no reset. Touch still starts if probing fails.

The [Bosch BMA423 data sheet, section 4.2](https://www.bndhep.net/Electronics/Data/BMA423.pdf)
requires INIT_CTRL initialization only once per power-on/soft reset. The
PineTime [reference startup](https://github.com/InfiniTimeOrg/InfiniTime/blob/9ad16304f5fd526b781a760cf1313813d232645e/src/drivers/Bma421.cpp)
also requires sensor reset plus TWI recovery. The changed startup still needs
hardware validation, including repeated MCU resets, touch, BLE and OTA/trial
handoff. Host tests now reject a previously initialized ASIC before any upload.
Do not assume successful cold initialization proves warm startup behavior.

## 2. Establish a baseline before tuning

Keep detection and reporting separate. Prefer cumulative counter reads over
counting interrupt edges: notification batching/lost IRQs must not lose steps.
Evaluate an available counter interrupt/watermark for background operation and
refresh on wake/activity page entry. If a reporting IRQ is unavailable, measure
a bounded low-frequency read fallback. Per-step CPU wakes are not the default.

Record raw hardware count, monotonic time, sensor-reset generation, configuration
ID and public daily total. Optional short raw-acceleration capture uses a bounded
buffer only in a diagnostic build; do not stream all-day raw data over BLE or
write flash for every step. Keep the hardware counter running with display off.

Compare repeatable trials against validated InfiniTime on the same physical
watch, wearing position and route. Record settling time after each trial so
delayed/batched reporting is not mistaken for missed steps. A phone is useful
as a secondary comparison, not the ground-truth counter.

## 3. Accuracy test set

Use manually counted/video-counted steps or a clicker. Run at least three repeats
per case and keep a separate set of trials for validation after tuning.

| Case | What to measure |
| --- | --- |
| 20–50 steps, stops and restarts | Short-bout sensitivity and delayed initial counts |
| 200 and 1000 ordinary steps | Signed count error and repeatability |
| Slow/fast walking and running | Cadence sensitivity, missed/double counts |
| Stairs and uneven ground | Robustness across motion shapes |
| Carrying items / pushing a stroller | Limited arm swing; document physical limits |
| Sitting, typing, cooking, brushing teeth | False positives without walking |
| Travel / vehicle vibration | False positives from non-gait movement |
| Both wrists, different strap tightness | Wearing-position effects |
| Screen off, BLE connected, during reconnect | Independence from UI/phone activity |

Report absolute/signed error per case, false steps per duration, reporting
latency and power cost. A provisional goal for ordinary 1000-step walking is
within 5%, without worsening the negative-motion trials. This is a proposed
acceptance target, not a claim about present performance or all users/gaits.

## 4. Tune only supported parameters, then validate

Determine whether the matched feature engine offers documented wrist-use,
sensitivity or filtering parameters. Change one setting at a time and retain
both baseline and candidate results. Do not infer meanings from opaque blob
bytes or assume exposed parameters guarantee better counting.

Check the accuracy/power tradeoff of documented operating modes. Consider a
host-side correction or raw-data algorithm only if reproducible failures remain
and captured data supports a better result on the held-out trials. An activity
gate must not erase short walks or stationary-arm steps to improve a headline
false-positive score. Keep a small number of tested configurations initially;
do not expose dozens of unexplained sliders to users.

## 5. Daily totals, persistence and Furu

Keep the raw sensor counter independent from the daily total. Track counter
generation/reset explicitly, read errors, unsigned wrap and reboot restores.
Do not subtract a reset count from the previous value and create a huge delta.
An I2C failure is unavailable data, never a real zero. Test these transitions
as pure host logic.

Use a monotonic counter timeline plus wall-clock day boundaries. Handle unset
time, CTS corrections, timezone/DST changes and midnight without duplicate
counting. Read at day boundaries where possible; document uncertainty if a
batch spans midnight without per-step timestamps. Avoid resetting the physical
counter merely to change the displayed date.

Use bounded, versioned checkpoints when persistence is ready, not flash writes
per step. Follow [the LittleFS compatibility plan](shared-littlefs.md); unmodified
InfiniTime does not share a persistent step record with us. Define the maximum
recent progress lost on an unexpected power failure before promising continuity.

Expose the daily count through the InfiniTime-compatible Motion Service already
read by Furu (`00030000-…` / `00030001-…`, u32 LE). Send source/reset/time metadata
through a separately versioned capability if needed for history; Furu must not
double-count the same sample after reconnect. Keep diagnostics available for
test runs without making the ordinary watch UI a debug console.

## 6. Power and release gate

First compare idle battery/voltage trends with step counting off/on, same BLE
connection, brightness, wake usage and duration. A Nordic current monitor, if
available later, can distinguish sensor current from CPU/IRQ/bus wake costs.
Do not translate percentage differences from a short run into precise µA claims.

Before stable 0.4.0: identify supported variants, complete the baseline and
validation trials, test sleep/OTA/reset/date/reconnect cases and document any
remaining accuracy/persistence limitations. Keep a known configuration fallback
and avoid silently changing step behavior between candidates.
