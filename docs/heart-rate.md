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
eight-second buffer. Flat/dark/clipped signals, large timing gaps and unstable
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
fails on a closed watch. It does not itself start measurement or stream raw PPG.

| Bytes | Value |
| --- | --- |
| 0 | Version 2 (version 1 used fixed x1 gain) |
| 1 | Phase: 0 idle, 1 measuring, 2 no signal, 3 missing, 4 fault |
| 2 | Live BPM (0 unavailable) |
| 3 | Last error: 0 none, 1 bus, 2 wrong ID, 3 not running, 4 invalid gain, 0x80 OR register address for readback failure |
| 4–5 | u16 LE: low 12 bits sample count (saturated at 4095), high 4 bits gain multiplier (1/2/4/8). Version 1 used the whole u16 for count. |
| 6–9 / 10–13 | Latest PPG / ambient, u32 LE |
| 14–15 / 16–17 | Last reporting window PPG min / max, u16 LE (saturated) |
| 18–19 | Sample age in ms, u16 LE (65535 if unavailable/too old) |

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
