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
