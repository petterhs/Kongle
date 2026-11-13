# Local heart-rate acquisition profile experiment

Build with `KONGLE_FEATURES=mcuboot,ota-activation,hr-test-profiles bash scripts/build-dfu-package.sh`.
The feature is off by default and is not added to release workflows. The watch
starts with A after each reboot; no filesystem or settings storage is changed.

| Button label | Analog gain | LED drive selection |
| --- | ---: | ---: |
| A | x2 | 12.5 mA |
| B | x4 | 12.5 mA |
| C | x2 | 20 mA |
| D | x4 | 20 mA |

Use the bottom button on the pulse page to cycle A → B → C → D → A. It is
locked while Measure is pending or a measurement is active. Stop (and wait for
it to settle) before selecting another profile. Changing the selected profile
hides the old cached result. Each new Measure initializes the sensor and a new
estimator. The driver verifies both gain and current-selection register bits.
Stop/navigation/sleep/OTA and the existing 45-second limit still shut down the
LED driver and sensor. Nothing measures automatically and no remote tuning
characteristic is added. The driver always boots with the old sensor disabled.

Only gain and LED drive change; 15-bit resolution, 50 ms conversion wait,
10 Hz polling, settling time, estimator and confidence thresholds are held
fixed. The LED values are drive selections, not measured average device current.
Register encoding: [HRS3300 datasheet](https://files.pine64.org/doc/datasheet/pinetime/HRS3300%20Heart%20Rate%20Sensor.pdf),
ENABLE 0x01 bit 3 and PDRIVER 0x0c bit 6. Both experiments keep ENABLE 0xd0;
PDRIVER is 0x2f for 12.5 mA and 0x6f for 20 mA. HGAIN 0x17 is 0x04/x2 or
0x08/x4. Runtime gain readback and current-bit readback are required for start.

## Capture identity

Test builds emit diagnostic version 4 (20 bytes, same characteristic and all
other field positions unchanged). The packed u16 at offsets 4–5 keeps its low
12-bit sample count; the high nibble now contains profile ID 0=A through 3=D,
not gain. Each measuring sample therefore identifies its actual configuration.
Normal firmware continues emitting version 3 with gain in that high nibble.
Do not interpret version-4 high bits as gain. The updated offline decoder handles
both versions and emits `profile` and `led_deci_ma` for v4; legacy current is
unknown rather than guessed. Existing Furu versions can record raw v4 bytes,
but the updated capture UI is needed for informative filenames.

Furu reads Firmware Revision directly from the connected watch when recording
starts, retaining that capture's version/hash even if another watch is selected
later. Export names include it and every configuration observed in nonempty
measuring packets; stale idle snapshots are excluded. Mixed-profile recordings
are named with every observed profile. CSV preserves the original three columns
and appends firmware revision and per-diagnostic profile/gain/LED current;
JSON includes capture metadata while retaining the version-1 raw sample format.
Missing firmware/config information is explicitly named unknown. Exports remain
local; no captured optical data is committed or uploaded.

Compare several complete attempts per profile in alternating order with the
same watch, fit, posture and lighting. Track time to first valid reading, coverage,
clipping, gaps and rejection reasons, ideally with an independent reference rate.
Do not choose a profile solely because it displays more values. Periodic sampling
and measuring actual power are separate follow-up experiments.
