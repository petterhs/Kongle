# On-demand HRS3300 pulse prototype

**Current UI test branch: `feature/heart-rate-ui`**, built on the tested
`test/heart-rate-gain-x2` acquisition experiment. The sensor configuration
below is unchanged by the UI work. The acquisition experiment changes only
gain to fixed x2. LED current remains 12.5 mA, resolution 15-bit, conversion wait
50 ms, sampling 10 Hz and DSP thresholds identical to the x1 baseline `0601db8`.
The diagnostic gain nibble reports 2 after a successfully verified start.
Compare a full stationary capture with the baseline using the same fit/posture;
inspect raw swing, clipping, time to first rate and coverage. Higher gain may
amplify noise too and is not assumed to improve accuracy. Keep the baseline
available. Historical reference-acquisition descriptions below refer to x1.

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
uses large seven-segment digits and `---` without a fresh reading. A small loader fills over an estimated 11.2 seconds, then keeps
moving if no usable reading has arrived. It never delays a valid result or
claims the sensor is ready based on elapsed time alone. Only the loader region
is animated (5 Hz while this page is visible and awaiting a result); the large
digits and button are redrawn only when their content changes. Detailed signal
quality remains available in BLE diagnostics rather than on the pulse page.
BLE transmits only fresh readings during an active measurement; an idle/cached watch value is never replayed as a new phone
measurement. Stopping or losing a usable signal sends invalid BPM zero, which
Furu hides. No zero is displayed as an actual pulse.

## Transport and signal handling

The independent async Rust driver follows the [TYHX HRS3300 datasheet](https://files.pine64.org/doc/datasheet/pinetime/HRS3300%20Heart%20Rate%20Sensor.pdf):
I2C 0x44, ID 0x21, HEN in 0x01, oscillator/LED drive in 0x0c, resolution 0x16,
gain 0x17 and one eight-byte burst for both packed optical channels. It checks
configuration readback and attempts both shutdown writes even if one fails.
The existing Rust hrs3300 crate uses blocking embedded-hal 0.2; the watch uses
a small transport compatible with the shared bus instead of adding that stack.

The current reference-acquisition experiment uses fixed gain x1, resolution
0x77 (15-bit), conversion wait 50 ms (ENABLE baseline 0x50), and the lowest LED
drive selection (12.5 mA pulse current, not average watch current). PDRIVER is
0x2f, as documented in the PineTime
[reference driver](https://github.com/InfiniTimeOrg/InfiniTime/blob/9ad16304f5fd526b781a760cf1313813d232645e/src/drivers/Hrs3300.cpp).
Automatic gain changes are removed from acquisition for this experiment. The
hardware controls remain available in the driver, but are not adjusted while
measuring. Sensor gain/resolution and actual optical quality need physical
validation. The normal 45-second limit and verified shutdown are unchanged.

The owner requests samples every 100 ms (10 Hz), ignores the first 200 ms for
processing, and does not replay missed samples in a catch-up burst. Processing
uses an independent integer implementation: 80 raw samples (eight seconds),
least-squares baseline detrending, forward/backward band-pass filtering, a Hann
window and a Goertzel frequency bank
on a 1 BPM grid from 20 to 280 BPM. Grid spacing is not measurement accuracy.
Fixed mathematical coefficients are generated from the formulas documented in
`src/heart_rate_coefficients.rs`; no upstream DSP implementation/tables or new
DSP dependencies are incorporated. The broader search allows a dominant 240
BPM signal to be rejected rather than reporting a 120 BPM subharmonic.

Every second, a candidate must pass spectral concentration, rejection of
competing peaks and normalized delayed waveform similarity. Every two-second
quarter of the window must have at least two raw ADC levels of variation, so an
old pulse burst cannot keep producing fresh results after the signal goes flat.
Four consistent candidates spanning at least three seconds are required before
publishing: the first reading can arrive after approximately eleven seconds.
The allowed candidate change remains 10% (minimum 6 BPM). Bounded scheduling
jitter is linearly resampled onto a 100 ms grid with Q8 precision. A gap greater
than 250 ms or non-monotonic time resets history. Points interpolated across
intervals greater than 180 ms are marked uncertain; at most four of the 80
window points may be uncertain. Denser stalls are rejected without throwing
away the entire retained window. Read timestamps are taken after the I2C transaction,
including shared-bus wait. Timestamp wrapping is supported. Rates remain restricted to 30–220 BPM;
15-bit clipping bounds track the changed resolution. Analysis taking at least
10 ms is logged over RTT to make CPU/scheduling cost visible in hardware tests.

Flat/dark/clipped signals, large timing gaps, ambiguity and unstable rates are
rejected. Periodic motion can still mimic a pulse: these are experimental
estimates, not medical measurements. Synthetic tests cover 30–220 BPM, pulse
shapes with secondary peaks, drift/noise, ADC update timing, competing peaks,
out-of-range fundamentals, short bursts, missing samples and timestamp wrap.
These checks do not establish operation or accuracy on a person. Real optical
traces remain the next release gate; do not claim a finished HR feature.

The first closed-watch test started acquisition with a blinking green LED but
ended in No clear signal / No reading yet. Its version-1 diagnostic was
`01-01-00-00-70-02-5D-03-00-00-06-00-00-00-5C-03-60-03-BA-02`:
624 samples, no reported transport error, latest PPG 861, ambient 6,
reporting-window range 860–864, sample age 698 ms. That window is far below
clipping but below the old estimator's minimum pulse amplitude. An earlier
bounded gain-calibration experiment then produced the unstable results below.
The current build instead uses reference acquisition with fixed x1 gain;
weak optical coupling cannot be fixed by merely amplifying noise.

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

## Reference-acquisition experiment ready for testing

The source comparison above describes earlier Kongle builds where stated.
Current changes implement the 15-bit/50 ms/x1 sensor setup and 100 ms host
sampling. Host tests showed that merely changing time constants in the old
peak detector lost high-rate/held-ADC readings, so it was replaced with the
independent windowed frequency analysis described above. Regression checks
also caught an old-burst freshness problem; per-quarter signal checks now
reject it. The LED current was not raised. No Furu change is required beyond
`furu-heart-rate-diagnostics-dev.apk` already in the synchronized test folder.

Test the newly named reference OTA ZIP in `/home/s27731/sync-phone-apps`.
Leave the watch trial unconfirmed during hardware validation. First try a
complete still measurement in Furu. Whether or not a BPM appears, capture the
custom diagnostic stream in nRF Connect for a complete still measurement,
as described above, and retain its log for waveform analysis. This is the
first physical test of the reference setup and new estimator, not a validated
accuracy improvement. Do not tune thresholds further without examining data.

### Replay a captured optical trace on the host

Export a chronological text log from nRF Connect or a JSON capture from Furu.
The converter accepts diagnostic version
3 only; older versions encode sample age rather than elapsed sampling time.

```bash
python scripts/hr-diagnostics-to-csv.py /path/to/nrf-log.txt --output /tmp/hr.csv
cargo run --locked --manifest-path tests/host/Cargo.toml \
  --target "$(rustc -vV | sed -n 's/^host: //p')" \
  --example replay-heart-rate -- /tmp/hr.csv
```

Run inside `devenv shell`. The CSV includes raw optical channels, captured BPM,
gain, elapsed time, sample counter and missing-sample counts. Repeated reads of
the same sample are deduplicated; reset markers/counter rollback separate
measurement sessions. The replay tool compiles the actual firmware estimator
and reports qualified BPM separately from the captured device result. It resets
on session/gain changes and uses the same bounded-jitter resampling as the
firmware. Captures must be from the current 10 Hz profile: old 40 ms traces
belong to a different sensor configuration and are not a valid comparison.
Capture gaps or dense interpolation can prevent readings even when the watch
processed a contiguous stream. Replay results alone do not prove accuracy.
Each output row now includes the rejection reason, filled sample count, timing
resets, analysis count, spectral concentration and waveform similarity; a blank
replayed BPM is explicitly not a reading.

## Review corrections and quality diagnostics (2026-11-11)

Review reproduced zero readings from clean 72 BPM with slow baseline drift,
low amplitude, amplitude modulation or one 30 ms scheduling delay every five
seconds. After these changes each of those deterministic 45-second traces gives
35 readings near 72 BPM. These are host regressions, not measurements on skin.

- A 0.35 Hz Butterworth high-pass biquad and a 4 Hz low-pass are applied forward
  and backward to each window. High-pass Q=1/sqrt(2), Q30 coefficients from the
  [standard biquad formula](https://www.w3.org/TR/audio-eq-cookbook/#hpf),
  normalized by a0; low-pass coefficient is 1-exp(-2*pi*4/10), Q15. Q8 states
  retain precision. This avoids treating slow optical baseline changes as a
  dominant out-of-range pulse while keeping a flat low-rate passband.
- Detrended samples are numerically normalized before filtering. This changes
  calculation precision only, not LED current, sensor gain or measured optical
  precision. The arbitrary 16-level cutoff is replaced by the three-level
  (two-level swing) floor plus relative spectral/periodic quality checks.
- Short bursts, actual gaps, clipped/dark input, pure slow drift, tiny seeded
  quantization noise and competing frequency peaks still must be rejected.
  Pulse-shaped tests now include 30/40 BPM to protect low-rate fundamentals.
- The watch displays collection progress, confirmation progress or a short
  rejection explanation on its Pulse page instead of only No reading yet.
  Updates remain restricted to the existing text regions in both languages.

Read the appended quality characteristic
`6b6f180d-1886-4b40-ae7d-00a11ce50002` under HR for additional diagnostics.
Existing 0x2a37 and raw-v3 characteristic handles/payloads are unchanged.
Refresh services in nRF Connect if the new characteristic is not visible.
No companion-app changes are required.

| Bytes | Quality payload (20 bytes, little-endian integers) |
| --- | --- |
| 0 | Version 1 |
| 1 | Reason: 0 collecting, 1 timing gap, 2 non-monotonic time, 3 dark, 4 clipped, 5 low amplitude, 6 poor spectral concentration, 7 competing peaks, 8 out-of-range rate, 9 poor periodicity, 10 confirming, 11 qualified, 12 insufficient real-sample coverage, 13 ambient clipping |
| 2 / 3 | Filled grid samples (0–80) / last candidate BPM (saturated at 255) |
| 4–5 / 6–7 | Timing-history resets / invalid optical samples, saturating counters |
| 8–9 / 10–11 | Analysed windows / qualified readings, saturating counters |
| 12–13 | Last window's raw ADC peak-to-peak range, not pulse amplitude |
| 14–15 | Spectral peak/mean concentration multiplied by 100 (minimum 300; minimum 1000 for marginal periodicity) |
| 16–17 | Delayed waveform similarity on 0–10000 scale (minimum 5500, with stronger spectral evidence below 6500) |
| 18 / 19 | Consistent-candidate count / last input interval in ms (saturated at 255) |

Counters reset only when starting a new measurement. Invalid input and timing
resets clear the window and candidate history but preserve counters. Zero
spectral/similarity values can mean that an earlier check rejected the window;
read the reason first. Current reason/counters also appear in host replay output.
If physical tests still fail, report the Pulse page text and these 20 bytes after
about 15 seconds, alongside raw-v3 data or a full notification trace. The prior
Furu HR subscription now demonstrably delivers zero/valid payloads, so this
iteration remains entirely within Kongle while Furu UI work proceeds separately.

## Follow-up to the captured-session review (2026-11-12)

Furu's `test/ui-plus-hr` / `docs/firmware-review-heart-rate.md` provided a real
capture. The anonymized optical samples are retained in
`tests/fixtures/hr-weak-optical.csv`; see that folder's README for provenance and
limitations. Baseline replay reproduces 21 qualified analyses, first at 19003 ms.
The updated estimator yields 30, first at 12002 ms. These are replay results;
a new watch capture and independent reference are still required.

Periodicity now accepts similarity >=5500. Marginal scores below the old 6500
require spectral concentration >=1000 (10x peak/mean), rather than the normal
300. A single marginal failed analysis with strong spectral evidence may pause
confirmation without publishing a value. Evidence expires after 2500 ms; large
rate changes and hard rejections still restart it. This avoids repeatedly
throwing away progress while preserving noise checks. One positive recording
cannot prove a threshold safe: white/quantized/temporally correlated noise,
competing frequencies, out-of-range signals and bursts remain regression tests.

The reviewer suggested bridging 600 ms and increasing gain/LED current. Neither
is adopted. Short 181–250 ms stalls may be bridged, but uncertain points are
limited to 5% of a window and longer gaps still reset. Frequent stalls report
InsufficientCoverage instead of endless WarmingUp. The actual recorded session
had no such gaps, so this is hardening rather than the explanation for that
capture. Motion polling is unchanged; normal small shared-bus transactions do
not by themselves establish an 180+ ms stall.

Ambient saturation still invalidates a sample, now with reason 13 and
"Too much light" on the watch. PPG saturation retains reason 4. These reasons
are preserved through refilling rather than being immediately overwritten by
WarmingUp on the next non-clipped sample. Gain is already x1 and LED drive is
already the lowest selected setting; changing integration/resolution or LED
power automatically requires separate validation. No fabricated or clipped
reading is published to conceal these conditions.

Qualified readings are sent to the shared BLE state immediately instead of
waiting for the separate one-second housekeeping phase. Raw-v3 and quality-v1
payload sizes/field positions are unchanged; quality reasons 12/13 are additive.
Furu can continue recording raw bytes without an update. No Furu source files
are changed by these firmware corrections.

Furu JSON exports can be replayed directly without an ad-hoc Node decoder:

```bash
python scripts/hr-diagnostics-to-csv.py /path/to/furu-capture.json --output /tmp/hr.csv
```

The same host replay command above then applies. Conversion ignores measurement
and quality packets and exports only measuring raw-v3 frames. Unsupported capture
versions and invalid byte arrays are not guessed. Identity/wall-clock metadata
is omitted from the numeric CSV.

Review source at the time of this investigation:
[Furu firmware review](https://github.com/petterhs/Furu/blob/57a1c0fdf3572b3645f718aeb0396540d5c35792/docs/firmware-review-heart-rate.md).
The review's suggestion to expose warmup/confirmation progress was already
implemented by `4e2340d`. Spectral concentration near 1800 means approximately
18x peak/mean, well above the normal 3x cutoff, so it was not itself a failing
threshold in this trace. It now supplies the additional evidence needed for
marginal periodicity. This does not establish the person's true heart rate.

## Proposed interval measurement experiment (not enabled)

Start with an opt-in watch setting, off by default: one attempt every five
minutes, stop immediately after the first valid reading or after 14 seconds
without one. Keep the manual Measure action and its 45-second limit separate.
The 11.2-second first result in the successful x2 capture is an observation,
not a guarantee; poor contact may produce no reading within a 14-second budget.
Record successful/failed attempts and actual sensor-on duration alongside battery
telemetry, then compare equivalent overnight runs with interval mode off/on.
A 14-second ceiling every 300 seconds gives at most 4.7% sensor-on time, not
4.7% of total watch power. Analog x2 gain does not imply twice the LED current.

Before enabling this mode, move measurement ownership out of page visibility:
manual measurements stop on navigation/sleep, while scheduled measurements must
be able to finish with the display off without waking it. OTA and disabling the
setting must cancel either mode and verify sensor shutdown. Do not schedule
overlapping attempts or catch up missed intervals in a burst. Mark a stored
reading with its age; do not send it again as a new live HRS reading on reconnect.
Reuse the existing settings persistence and include a diagnostic run with missed
readings, navigation, display sleep, reconnect, reboot and OTA cancellation.
