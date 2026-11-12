# Optical regression trace

`hr-weak-optical.csv` contains 449 measuring frames from Furu's
`docs/furu-hr-capture-2026-11-11T23-12-33.json`, reviewed on `test/ui-plus-hr`.
Only relative elapsed milliseconds and raw PPG/ambient ADC channels remain;
device address, wall-clock timestamps and app metadata are omitted.

The watch ran Kongle `4e2340d`, with 15-bit resolution and fixed x1 gain.
Its signal has roughly 9–22 ADC counts of per-second variation around DC 2000.
Replay at that revision gives 21 qualified analyses, first at 19003 ms.
The review correction gives 30, first at 12002 ms. A host `State::bpm` freshness
model covers about 31.5 seconds instead of 23.5 seconds of the 44.8-second
measuring trace. Those modeled durations are not the same as actual Furu UI
coverage or notification timing. Newly qualified readings are now published
immediately to avoid an additional housekeeping-period delay.

The trace has no independently measured reference pulse. It tests deterministic
replay, continuity, startup delay and absence of large harmonic jumps, not
clinical accuracy. Synthetic negative tests remain necessary alongside this
fixture; one person's recorded signal cannot prove that a lower threshold
rejects noise in general.

## x2-gain physical test

`hr-gain-x2-optical.csv` contains 449 anonymized measuring frames from
Furu's `docs/furu-hr-capture-2026-11-12T07-35-26.csv`, firmware `0311279`.
Diagnostics confirm gain 2, no lost samples and no clipped samples.
Replay gives 30 qualified analyses, starting at 11202 ms.

Direct comparison with the earlier x1 CSV `2026-11-12T00-12-20`:

| Metric | x1 | x2 |
| --- | ---: | ---: |
| First phone rate after estimated measurement start | 24.1 s | 11.2 s |
| Phone time with a rate (held until next zero) | 16.0 s | 32.1 s |
| Positive measurement notifications | 16 | 32 |
| Received BPM min / median / max | 61 / 65 / 72 | 67 / 69 / 71 |
| PPG DC median | 1616 | 4178 |
| Median full-second raw peak-to-peak swing after first 5 s | 10 | 37 |

The x2 phone stream remains valid from capture t=13045 to 45124 ms, with a
short final gap before the end at 46833 ms. No further thresholds were changed
to hide that rejection. Warmup, signal rejection and a reference pulse remain
separate from continuity. Different test times, contact and ambient light mean
the whole amplitude improvement cannot be attributed to analog gain alone.
Several repeat measurements and another closed watch are desirable before
making x2 the production default. No additional LED-current increase is implied.
