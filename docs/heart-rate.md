# On-demand HRS3300 pulse prototype

Branch `feature/heart-rate` extends the tested sensor/steps foundation for 0.4.0.
The development watch's HRS3300 has physically detached. Host/ARM checks cannot
establish sensor operation, signal quality, LED shutdown or accuracy: those
require a closed watch. Do not mark heart-rate release gates complete yet.

## User behavior

Tap the heart field to the right of steps on the clock, then Measure. Opening
the page, connecting Furu, subscribing to HR or reading its characteristic
does not start acquisition. Stop, navigation away, the physical button,
notification preview, display sleep or OTA opening the clock request shutdown.
Commands use a latest-intent signal so Stop supersedes an unconsumed Start.

While measuring, the page stays awake up to the 45-second acquisition limit.
There is no automatic background/interval mode. The owner attempts and verifies
LED-driver/oscillator shutdown and HRS disable before publishing idle. Shutdown
failures retry three times and remain visible as a sensor fault; a failed bus
cannot prove physical LED state. Boot probes a known ID then shuts down a sensor
that a previous firmware may have left running across an MCU reset. Missing or
unexpected IDs receive no writes and do not start a retry loop.

The clock retains a reading for 30 seconds, then shows `--`. The pulse page
retains the last value and its age. BLE transmits only fresh readings during an
active measurement; an idle/cached watch value is never replayed as a new phone
measurement. Stopping or losing a usable signal sends invalid BPM zero, which
Furu hides. No zero is displayed as an actual pulse.

## Transport and signal handling

The independent async Rust driver follows the [TYHX HRS3300 datasheet](https://files.pine64.org/doc/datasheet/pinetime/HRS3300%20Heart%20Rate%20Sensor.pdf):
I2C 0x44, ID 0x21, HEN in 0x01, oscillator/LED drive in 0x0c, resolution 0x16,
gain 0x17 and one eight-byte burst for both packed optical channels. It checks
configuration readback and attempts both shutdown writes even if one fails.
The existing Rust hrs3300 crate uses blocking embedded-hal 0.2; the watch uses
a small transport compatible with the shared bus instead of adding that stack.

Initial settings are gain x1, resolution 0x66, wait 12.5 ms and the lowest LED
drive selection (12.5 mA pulse current, not average watch current). PDRIVER's
low nibble uses the steady-drive setting documented in the PineTime
[reference driver](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/drivers/Hrs3300.cpp).
During the first eight seconds, one-second windows can raise analog gain through
x2/x4/x8 for a weak signal. Windows require at least 20 settled samples. Gain
is reduced if PPG reaches 12000 to leave ADC headroom; separate upward/downward
thresholds avoid oscillation. Each change discards estimator history and waits
200 ms before processing new samples. At most six changes are allowed, and the
original 45-second deadline never moves. LED current and resolution stay fixed;
x64 is not used. This is a hardware starting point, not a calibrated configuration. No upstream
GPL/LGPL pulse-processing code is incorporated. Physical conversion timing,
gain/drive suitability and raw waveform quality still need measurement.

The owner requests samples at 40 ms intervals and uses actual monotonic times.
It does not replay missed samples in a catch-up burst. PPG processing uses an
integer DC estimate, three-sample smoothing, an adaptive amplitude threshold,
four consistent beat intervals and delayed waveform similarity over a bounded
eight-second buffer. At least six seconds of waveform and four qualifying rate
candidates spanning at least three seconds are required before publishing.
Candidates must remain within 10% (minimum allowance 6 BPM) of their first rate;
an inconsistent waveform, large rate change or candidate gap resets confirmation.
Gain changes reset all signal history. Flat/dark/clipped signals, large timing gaps and unstable
intervals are rejected. Rates are restricted to 30–220 BPM. Periodic motion can
still mimic a pulse: these are experimental estimates, not medical measurements.
Synthetic tests are regression checks, not validation on people or movement.
Tests also exercise pulse-shaped waveforms with a reflected secondary peak,
slow baseline drift, small noise and ADC updates held for 40/75/100 ms. These
cases pass, but do not establish the actual device's signal range or quality.

The first closed-watch test started acquisition with a blinking green LED but
ended in No clear signal / No reading yet. Its version-1 diagnostic was
`01-01-00-00-70-02-5D-03-00-00-06-00-00-00-5C-03-60-03-BA-02`:
624 samples, no reported transport error, latest PPG 861, ambient 6,
reporting-window range 860–864, sample age 698 ms. That window is far below
clipping but below the estimator's minimum pulse amplitude. Bounded analog gain
calibration addresses this hypothesis without relaxing pulse-quality checks.
It still requires another physical test: weak optical coupling or absence of a
detectable pulse cannot be fixed by amplifying noise. The configured 14-bit
clipping boundary remains an assumption to validate against real waveforms.

The second closed-watch test after gain calibration produced isolated values
130/88, 188 and 214 BPM reported as too high, with no values visible in Furu.
That is not a successful accuracy test. Sustained confirmation now rejects
short periodic bursts instead of showing a value from a few matching peaks.
It cannot prove physiological origin or fix double-counting without actual
waveforms. Capture the diagnostic stream before further filter tuning.

HR transfers use a mutex-protected bounded port, rather than cancelling a DMA
future while its sample buffer is live. Ordinary small transfers briefly block
for about a millisecond; a transaction has a 10 ms limit. On controller timeout,
the exclusive port disables DMA before its buffers leave scope and recovers
TWIM1, without stealing peripherals or resetting touch. This recovery path and
interaction with existing async clients need physical fault testing. No other
client's transfer is cancelled by HR Start/Stop.

## BLE

The appended Heart Rate Service is 0x180d with live Measurement 0x2a37
(notify and an additional read), plus wrist Body Sensor Location 0x2a38.
Payloads are `[0, bpm]`: uint8 BPM, no claim of skin-contact detection, energy
or RR intervals. The [Bluetooth SIG specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/HRS_v1.0/out/en/index-en.html)
defines these flags. Furu's contact-supported/detected bit mapping is corrected
in the companion branch; built-in Kongle profiles gain the existing `ble.hr`.
The persistent GATT listener remains alive during HR notifications and DFU.

Developer-only read diagnostic: `6b6f180d-1886-4b40-ae7d-00a11ce50001` under
the HR service. Read it in nRF Connect and send its 20 hex bytes if acquisition
fails on a closed watch. It does not itself start measurement. It now supports
opt-in notifications containing raw optical samples during an explicit watch
measurement; an ordinary 0x2a37 subscription does not enable this debug stream.
The latest-value transport can drop intermediate frames if BLE falls behind:
use the sample counter and elapsed sample time to identify gaps. No optical
history is retained after measurement.

| Bytes | Value |
| --- | --- |
| 0 | Version 3 (version 1 used fixed x1 gain) |
| 1 | Phase: 0 idle, 1 measuring, 2 no signal, 3 missing, 4 fault |
| 2 | Live BPM (0 unavailable) |
| 3 | Last error: 0 none, 1 bus, 2 wrong ID, 3 not running, 4 invalid gain, 0x80 OR register address for readback failure |
| 4–5 | u16 LE: low 12 bits sample count (saturated at 4095), high 4 bits gain multiplier (1/2/4/8). Version 1 used the whole u16 for count. |
| 6–9 / 10–13 | Latest PPG / ambient, u32 LE |
| 14–15 / 16–17 | Last reporting window PPG min / max, u16 LE (saturated) |
| 18–19 | Sample elapsed time since measurement start in ms, u16 LE (65535 unavailable). Versions 1/2 used sample age instead. |

To capture a waveform without ST-Link: disconnect Furu, connect nRF Connect,
refresh services if the diagnostic notification property is missing, enable
notifications on the custom diagnostic characteristic and on 0x2a37, then start
Measure on the watch. Keep still for the full measurement and export/share the
nRF Connect log. The custom stream contains optical data and the standard HR
stream tells us which BPM values were actually delivered. Timestamps and gain
are included so gain transitions and missed samples can be excluded in analysis.
In Furu's companion diagnostic build, Logs now records raw HR read/notification
payloads and parsing results. Subscription teardown/setup is serialized so an
outdated async operation cannot unsubscribe a newly installed listener.

## Physical test sequence

1. First use the dev watch to check missing-sensor behavior, touch, steps and
   BLE. It must remain usable without an HR sensor and without automatic retry.
2. OTA the normal activation package (without touch-wake-debug) to a closed
   watch. Keep it unconfirmed until recovery, power and daily-use testing pass.
3. Connect the companion Furu build; repair the GATT cache if an ST-Link flash
   added the new service. Merely connecting must leave the green LED off.
4. Wear it snugly, remain still, open Pulse and start. Compare several repeated
   measurements with a manual count or a trusted reference. Check warmup time,
   stable BPM and live Furu updates. If no value appears, capture diagnostics.
5. Verify LED off after Stop, back navigation/button, a notification preview,
   timeout and OTA. Confirm no repeated measurement or LED activity in sleep.
6. Test removal, ambient light changes, loose fit, movement, reconnect and
   reboot; note false positives, missing values and shutdown errors. Measure
   battery trends before considering any automatic sampling mode.

## Resume point: InfiniTime comparison (2026-11-10)

Maintainer tested `0.4.0+3193736`: Furu now receives repeated
`HR: notification [00 00] -> no valid reading`, but no valid BPM. This confirms
notification delivery to the parser for those packets, not pulse accuracy.
The stricter gate has suppressed results; the underlying optical/DSP problem
is still unresolved. Do not mark HR complete or keep tightening thresholds
without a captured waveform.

Read-only upstream comparison at InfiniTime commit
`9ad16304f5fd526b781a760cf1313813d232645e`:

- [Hrs3300.cpp](https://github.com/InfiniTimeOrg/InfiniTime/blob/9ad16304f5fd526b781a760cf1313813d232645e/src/drivers/Hrs3300.cpp):
  Enable baseline 0x50 (50 ms conversion wait), resolution 0x77 (15-bit),
  gain x1, PDRIVER 0x2f. Kongle uses 0x60, 0x66 and calibrated x1–x8;
  LED drive selection is already the same. Upstream comments describe
  experimentally improved signal quality and an approximately 50 ms ADC
  conversion period at 15 bits. This is not a measured timing guarantee here.
- [Ppg.h](https://github.com/InfiniTimeOrg/InfiniTime/blob/9ad16304f5fd526b781a760cf1313813d232645e/src/components/heartrate/Ppg.h)
  and [HeartRateTask.cpp](https://github.com/InfiniTimeOrg/InfiniTime/blob/9ad16304f5fd526b781a760cf1313813d232645e/src/heartratetask/HeartRateTask.cpp):
  host acquisition at 100 ms (10 Hz), 64 samples (6.4 seconds), five new samples
  per analysis (0.5 seconds). A post-enable settling delay precedes acquisition.
- [Ppg.cpp](https://github.com/InfiniTimeOrg/InfiniTime/blob/9ad16304f5fd526b781a760cf1313813d232645e/src/components/heartrate/Ppg.cpp):
  detrending/differencing, band-pass filtering, Hann window and FFT, spectral
  averaging, dominant-peak quality checks (noise ratio, width, multiple peaks,
  residual DC), then output averaging over up to 20 nonzero results. Ambient
  light changes can reset acquisition. Kongle's local-peak intervals plus
  delayed similarity and sustained confirmation are a different estimator;
  stricter confirmation alone does not correct double counting or weak signal.

Recommended next bounded experiment:

1. Capture a full stationary diagnostic stream with current firmware using
   nRF Connect as above, ideally with an independently counted reference rate.
2. Compare optical acquisition with the upstream 15-bit/50 ms/x1 settings,
   preserving raw timestamps and unchanged LED current. Update clipping bounds
   consistently with resolution. Do not merely change SAMPLE_MS to 100: the
   current smoothing, waveform length and lag calculations assume 25 Hz and
   must be adapted together if sampling changes. Separate sensor and DSP
   experiments so results remain interpretable.
3. Replay the same captured signal through candidate independent Rust filters
   and period/spectral estimators on the host before another closed-watch OTA.
   Inspect harmonics, irregular sample timing, gain transitions and signal/noise.
   FFT is an option, not an automatic requirement; evaluate RAM/flash and CPU
   cost on nRF52832. Do not copy upstream DSP implementation into this project.
4. Recheck repeated readings, removal/movement rejection, LED shutdown and BLE
   delivery. Keep trial/recovery behavior until physical validation passes.

Testing artifacts should be placed in `/home/s27731/sync-phone-apps` per the
maintainer's request; Syncthing transfers this folder to the phone. Current
files there are `kongle-heart-rate-stability-dfu.zip` and
`furu-heart-rate-diagnostics-dev.apk`. The firmware ZIP was already there when
this investigation resumed; the diagnostic APK was copied there now. Continue
on local branches `feature/heart-rate` (`/tmp/kongle-sensors`) and
`feature/kongle-heart-rate` (`/tmp/furu-steps`); this investigation changes docs
only. Next development/testing can resume after the usage quota resets.
