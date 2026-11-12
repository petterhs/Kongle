use crate::heart_rate::{Estimator, Phase, Reading, State, MAX_MEASUREMENT_MS, SAMPLE_MS};

#[test]
fn captured_weak_optical_trace_recovers_earlier_without_rate_doubling() {
    // Anonymized capture from Furu, 2026-11-11. These are optical samples,
    // not an independent ground-truth pulse measurement. Avoid overfitting
    // the exact rate; require continuity and no implausible harmonic jumps.
    let mut estimator = Estimator::new();
    let mut state = State {
        phase: Phase::Measuring,
        ..State::EMPTY
    };
    let mut first = None;
    let mut qualified = 0;
    let mut coverage_ms = 0;
    let mut previous = None;
    for row in include_str!("../../fixtures/hr-weak-optical.csv")
        .lines()
        .skip(1)
    {
        let values: Vec<u64> = row.split(',').map(|value| value.parse().unwrap()).collect();
        let at = values[0];
        if let Some(previous) = previous {
            if state.bpm(previous).is_some() {
                coverage_ms += at - previous;
            }
        }
        if let Some(bpm) = estimator.sample(values[1] as u32, values[2] as u32, at) {
            assert!((55..=85).contains(&bpm), "unexpected jump: {bpm}");
            first.get_or_insert(at);
            qualified += 1;
            state.last = Some(Reading { bpm, at_ms: at });
        }
        previous = Some(at);
    }
    assert!(first.unwrap() <= 14000, "{first:?}");
    assert!(qualified >= 30, "{qualified}");
    assert!(coverage_ms >= 31000, "{coverage_ms}");
    assert_eq!(estimator.quality().timing_resets, 0);
}

#[test]
fn confirmation_survives_one_marginal_miss_but_expires_and_rechecks_rate() {
    use crate::heart_rate::RateConfirmation;
    let mut confirmation = RateConfirmation::new();
    assert_eq!(confirmation.candidate(70, 1000), None);
    assert_eq!(confirmation.candidate(70, 2000), None);
    confirmation.miss(3000);
    assert_eq!(confirmation.candidate(70, 4000), None); // Still requires 4 good candidates.
    assert_eq!(confirmation.candidate(71, 5000), Some(71));
    confirmation.miss(6000);
    confirmation.miss(7000);
    confirmation.miss(8000);
    assert_eq!(confirmation.candidate(70, 9000), None);
    assert_eq!(confirmation.candidate(140, 10000), None); // A jump is never inherited.
}

#[test]
fn occasional_short_stalls_keep_the_window_but_long_or_dense_gaps_are_rejected() {
    use crate::heart_rate::Reason;
    for bpm in [40, 72, 160, 220] {
        for isolated in [false, true] {
            let mut estimator = Estimator::new();
            let mut readings = Vec::new();
            for index in 0..450 {
                let now = index * SAMPLE_MS
                    + if isolated {
                        if index >= 200 {
                            150
                        } else {
                            0
                        }
                    } else if index % 50 == 25 {
                        85
                    } else {
                        0
                    };
                let raw = (7000.0
                    + 200.0 * (now as f64 * f64::from(bpm) / 60000.0 * std::f64::consts::TAU).sin())
                    as u32;
                if let Some(found) = estimator.sample(raw, 6, now) {
                    readings.push((now, found));
                }
            }
            assert!(
                readings.len() >= 20,
                "{bpm}, isolated={isolated}: {readings:?}"
            );
            assert!(readings
                .iter()
                .all(|(_, found)| (i32::from(*found) - bpm).abs() <= (bpm / 20).max(3)));
            assert_eq!(estimator.quality().timing_resets, 0);
            if isolated {
                let recovered = readings.iter().find(|(at, _)| *at >= 20150).unwrap().0;
                assert!(recovered <= 23150, "recovery at {recovered}");
            }
        }
    }
    let mut estimator = Estimator::new();
    for index in 0..450 {
        let now = index * SAMPLE_MS + if index % 10 == 5 { 85 } else { 0 };
        let raw =
            (7000.0 + 200.0 * (now as f64 * 72.0 / 60000.0 * std::f64::consts::TAU).sin()) as u32;
        assert_eq!(estimator.sample(raw, 6, now), None);
    }
    assert_eq!(estimator.quality().reason, Reason::InsufficientCoverage);
    assert_eq!(estimator.quality().filled, 80); // Window retained, not endlessly refilled.
    assert_eq!(estimator.quality().timing_resets, 0);
    estimator.sample(7000, 6, 46000);
    assert_eq!(estimator.quality().reason, Reason::TimingGap);
    assert_eq!(estimator.quality().filled, 1);
}

#[test]
fn ambient_and_ppg_saturation_are_distinct_and_do_not_disappear_as_warmup() {
    use crate::heart_rate::Reason;
    let mut estimator = Estimator::new();
    for index in 0..450 {
        let now = index * SAMPLE_MS;
        let raw =
            (30000.0 + 3000.0 * (now as f64 * 72.0 / 60000.0 * std::f64::consts::TAU).sin()) as u32;
        assert_eq!(estimator.sample(raw, 6, now), None);
    }
    assert_eq!(estimator.quality().reason, Reason::Clipped);
    assert!(estimator.quality().invalid_samples > 0);
    estimator.sample(7000, 32760, 45000);
    assert_eq!(estimator.quality().reason, Reason::AmbientClipped);
    estimator.sample(7000, 6, 45100);
    assert_eq!(estimator.quality().reason, Reason::AmbientClipped);
    let mut recovered = 0;
    for index in 452..750 {
        let now = index * SAMPLE_MS;
        let raw =
            (7000.0 + 200.0 * (now as f64 * 72.0 / 60000.0 * std::f64::consts::TAU).sin()) as u32;
        if let Some(bpm) = estimator.sample(raw, 6, now) {
            assert!((70..=74).contains(&bpm));
            recovered += 1;
        }
    }
    assert!(recovered > 10);
}

#[test]
fn review_regressions_keep_pulse_under_drift_jitter_and_changing_amplitude() {
    for case in [
        "clean",
        "weak",
        "baseline_drift",
        "scheduler_jitter",
        "amplitude_modulation",
    ] {
        let mut estimator = Estimator::new();
        let mut readings = Vec::new();
        for i in 0..450 {
            let now = i * 100
                + if case == "scheduler_jitter" && i % 50 == 25 {
                    30
                } else {
                    0
                };
            let t = now as f64 / 1000.0;
            let amplitude = match case {
                "weak" => 2.0,
                "amplitude_modulation" => 12.0 + 8.0 * (t * std::f64::consts::TAU / 8.0).sin(),
                _ => 200.0,
            };
            let baseline = if case == "baseline_drift" {
                7000.0 + 1500.0 * (t * 0.15 * std::f64::consts::TAU).sin()
            } else {
                7000.0
            };
            let raw =
                (baseline + amplitude * (t * 1.2 * std::f64::consts::TAU).sin()).round() as u32;
            if let Some(bpm) = estimator.sample(raw, 6, now) {
                readings.push(bpm);
            }
        }
        assert!(readings.len() >= 20, "{case}: {readings:?}");
        assert!(
            readings.iter().all(|bpm| (70..=74).contains(bpm)),
            "{case}: {readings:?}"
        );
        assert_eq!(estimator.quality().timing_resets, 0, "{case}");
    }
}

#[test]
fn weak_quantization_noise_and_slow_drift_alone_do_not_become_pulse() {
    for seed in 1u32..=16 {
        let mut estimator = Estimator::new();
        let mut random = seed;
        for i in 0..450 {
            random = random.wrapping_mul(1664525u32).wrapping_add(1013904223);
            assert_eq!(
                estimator.sample(859 + random % 5, 6, i * SAMPLE_MS),
                None,
                "seed {seed}"
            );
        }
    }
    // Filtered/temporally correlated noise is harder than independent ADC
    // noise. Marginal periodicity must not bypass spectral evidence.
    for seed in 1u32..=16 {
        let mut estimator = Estimator::new();
        let mut random = seed;
        let mut colored = 0i32;
        for i in 0..450 {
            random = random.wrapping_mul(1664525).wrapping_add(1013904223);
            let white = (random % 1000) as i32 - 500;
            colored += (white - colored) / 2;
            assert_eq!(
                estimator.sample((7000 + colored) as u32, 6, i * SAMPLE_MS),
                None,
                "colored seed {seed}"
            );
        }
    }
    let mut estimator = Estimator::new();
    for i in 0..450 {
        let now = i * SAMPLE_MS;
        let raw =
            (7000.0 + 1500.0 * (now as f64 / 1000.0 * 0.15 * std::f64::consts::TAU).sin()) as u32;
        assert_eq!(estimator.sample(raw, 6, now), None);
    }
}

#[test]
fn quality_explains_rejections_preserves_counters_and_encodes_a_stable_payload() {
    use crate::heart_rate::{Quality, Reason};
    let mut estimator = Estimator::new();
    for i in 0..90 {
        assert_eq!(estimator.sample(7000, 6, i * SAMPLE_MS), None);
    }
    let before = estimator.quality();
    assert_eq!(before.reason, Reason::LowAmplitude);
    assert_eq!(before.filled, 80);
    assert!(before.analyses > 0);
    estimator.sample(7000, 6, 9200); // A real gap clears history and counts it.
    assert_eq!(estimator.quality().reason, Reason::TimingGap);
    assert_eq!(estimator.quality().timing_resets, 1);
    assert_eq!(estimator.quality().filled, 1);
    estimator.sample(7000, 6, 9100);
    assert_eq!(estimator.quality().reason, Reason::NonMonotonic);
    estimator.sample(0, 6, 9300);
    assert_eq!(estimator.quality().reason, Reason::Dark);
    estimator.sample(0x7fff, 6, 9400);
    let after = estimator.quality();
    assert_eq!(after.reason, Reason::Clipped);
    assert_eq!(after.invalid_samples, 2);
    assert_eq!(after.timing_resets, 2);
    assert_eq!(after.analyses, before.analyses);
    let encoded = Quality {
        reason: Reason::Confirming,
        filled: 80,
        candidate: 72,
        timing_resets: 2,
        invalid_samples: 3,
        analyses: 4,
        qualified: 5,
        amplitude: 6,
        concentration: 700,
        similarity: 8000,
        confirmation: 2,
        interval_ms: 130,
    }
    .encode();
    assert_eq!(&encoded[..4], &[1, 10, 80, 72]);
    for (index, value) in [2u16, 3, 4, 5, 6, 700, 8000].iter().enumerate() {
        assert_eq!(
            u16::from_le_bytes(encoded[4 + 2 * index..6 + 2 * index].try_into().unwrap()),
            *value
        );
    }
    assert_eq!(&encoded[18..], &[2, 130]);
}

#[test]
fn isolated_candidates_and_rate_jumps_need_sustained_confirmation() {
    use crate::heart_rate::RateConfirmation;
    let mut confirmation = RateConfirmation::new();
    for (bpm, at) in [
        (130, 1000),
        (130, 1500),
        (88, 2200),
        (188, 4000),
        (214, 5000),
    ] {
        assert_eq!(confirmation.candidate(bpm, at), None);
    }
    let mut confirmation = RateConfirmation::new();
    for at in [0, 700, 1400, 2100, 2800] {
        assert_eq!(confirmation.candidate(88, at), None);
    }
    assert_eq!(confirmation.candidate(88, 3500), Some(88));
    assert_eq!(confirmation.candidate(214, 3800), None);
    assert_eq!(confirmation.candidate(214, 7000), None); // Long gap resets.
    assert_eq!(confirmation.candidate(214, 6900), None); // Invalid time resets.
    assert_eq!(confirmation.candidate(0, 7100), None);
    assert_eq!(confirmation.candidate(88, 7200), None);
}

#[test]
fn short_periodic_bursts_are_not_published_as_stable_pulse() {
    for bpm in [130, 188, 214] {
        let mut estimator = Estimator::new();
        for index in 0..750 {
            let now = index * SAMPLE_MS;
            let wave = if now < 4800 {
                200.0 * (now as f64 * f64::from(bpm) / 60000.0 * std::f64::consts::TAU).sin()
            } else {
                0.0
            };
            assert_eq!(estimator.sample((7000.0 + wave) as u32, 6, now), None);
        }
    }
}

#[test]
fn out_of_range_signals_are_not_reported_as_their_in_range_subharmonics() {
    for bpm in [20, 240, 260] {
        let mut estimator = Estimator::new();
        for index in 0..450 {
            let now = index * SAMPLE_MS;
            let raw = (7000.0
                + 200.0 * (now as f64 * f64::from(bpm) / 60000.0 * std::f64::consts::TAU).sin())
                as u32;
            assert_eq!(
                estimator.sample(raw, 6, now),
                None,
                "{} BPM at {} ms",
                bpm,
                now
            );
        }
    }
}

#[test]
fn competing_frequency_peaks_are_rejected_instead_of_selecting_a_harmonic() {
    for competing_bpm in [110.0, 144.0] {
        let mut estimator = Estimator::new();
        for index in 0..450 {
            let now = index * SAMPLE_MS;
            let phase = now as f64 / 60000.0 * std::f64::consts::TAU;
            // Slightly greater high-frequency amplitude offsets the low-pass
            // attenuation, leaving two competing peaks after filtering.
            let raw = (7000.0
                + 200.0 * (phase * 72.0).sin()
                + 220.0 * (phase * competing_bpm).sin()) as u32;
            assert_eq!(estimator.sample(raw, 6, now), None, "{competing_bpm}");
        }
        assert_eq!(
            estimator.quality().reason,
            crate::heart_rate::Reason::Ambiguous
        );
    }
}

#[test]
fn reference_adc_range_timestamp_wrap_and_missing_samples_are_handled() {
    let mut estimator = Estimator::new();
    let mut readings = Vec::new();
    // Above the old 14-bit clipping boundary, crossing the u16 timestamp wrap.
    for index in 0..300 {
        let now = 65000 + index * SAMPLE_MS;
        let phase = now as f64 * 72.0 / 60000.0 * std::f64::consts::TAU;
        if let Some(bpm) = estimator.sample((24000.0 + 200.0 * phase.sin()) as u32, 6, now) {
            readings.push(bpm);
        }
    }
    assert!(readings.len() > 5);
    assert!(readings.iter().all(|bpm| (70..=74).contains(bpm)));
    let gap_at = 65000 + 303 * SAMPLE_MS;
    assert_eq!(estimator.sample(24000, 6, gap_at), None);
    for index in 1..80 {
        let now = gap_at + index * SAMPLE_MS;
        let phase = now as f64 * 72.0 / 60000.0 * std::f64::consts::TAU;
        assert_eq!(
            estimator.sample((24000.0 + 200.0 * phase.sin()) as u32, 6, now),
            None
        );
    }
    assert_eq!(estimator.sample(0x7fff, 6, gap_at + 80 * SAMPLE_MS), None);
}

#[test]
fn large_legal_adc_waveforms_do_not_overflow_frequency_energy() {
    let mut estimator = Estimator::new();
    let mut readings = Vec::new();
    for index in 0..450 {
        let now = index * SAMPLE_MS;
        let phase = now as f64 * 72.0 / 60000.0 * std::f64::consts::TAU;
        if let Some(bpm) = estimator.sample((16376.0 + 16000.0 * phase.sin()) as u32, 6, now) {
            readings.push(bpm);
        }
    }
    assert!(readings.len() > 5);
    assert!(readings.iter().all(|bpm| (70..=74).contains(bpm)));
}

#[test]
fn weak_quantized_periodic_signals_are_judged_by_quality_not_absolute_amplitude() {
    for gain in [1, 8] {
        let mut estimator = Estimator::new();
        let mut readings = Vec::new();
        for index in 0..1000 {
            let now = index * SAMPLE_MS;
            let phase = now as f64 * 72.0 / 60000.0 * std::f64::consts::TAU;
            // Model analog amplification before ADC quantization, not digital
            // multiplication of already quantized sensor samples.
            let raw = ((861.0 + 2.0 * phase.sin()) * f64::from(gain)).round() as u32;
            if let Some(bpm) = estimator.sample(raw, 6, now) {
                readings.push(bpm);
            }
        }
        {
            assert!(readings.len() >= 3);
            assert!(readings
                .iter()
                .rev()
                .take(5)
                .all(|bpm| (i32::from(*bpm) - 72).abs() <= 3));
        }
    }
}

#[test]
fn synthetic_periodic_signals_have_consistent_rate_without_invented_startup_bpm() {
    for bpm in [30, 40, 60, 72, 90, 120, 160, 200, 220] {
        let mut estimator = Estimator::new();
        let mut results = Vec::new();
        for sample in 0..750 {
            let now = sample * SAMPLE_MS;
            let phase = now as f64 / 60000.0 * f64::from(bpm) * std::f64::consts::TAU;
            let raw = (12000.0 + phase.sin() * 200.0) as u32;
            if let Some(found) = estimator.sample(raw, 10, now) {
                assert!(
                    now >= 8800,
                    "six-second waveform plus sustained candidate confirmation"
                );
                results.push(found);
            }
        }
        assert!(
            results.len() >= 3,
            "no usable synthetic rate for {} BPM",
            bpm
        );
        for found in results.iter().rev().take(5) {
            assert!(
                (i32::from(*found) - bpm).abs() <= (bpm / 20).max(3),
                "target {}, observed {}",
                bpm,
                found
            );
        }
    }
}

#[test]
fn flat_dark_saturated_and_seeded_random_signals_do_not_become_pulse() {
    for raw in [0, 1, 10000, 0x7fff, 0x3ffff] {
        let mut estimator = Estimator::new();
        for i in 0..750 {
            assert_eq!(estimator.sample(raw, 10, i * SAMPLE_MS), None);
        }
    }
    let mut estimator = Estimator::new();
    let mut random = 0x12345678u32;
    for i in 0..1500 {
        random = random.wrapping_mul(1664525).wrapping_add(1013904223);
        let raw = 10000 + random % 1000;
        assert_eq!(
            estimator.sample(raw, 10, i * SAMPLE_MS),
            None,
            "noise at {}",
            i
        );
    }
}

#[test]
fn pulse_shaped_waveforms_with_slow_adc_updates_and_drift_are_not_just_sine_tests() {
    for bpm in [30, 40, 60, 72, 90] {
        for conversion_ms in [40, 75, 100] {
            let mut estimator = Estimator::new();
            let mut readings = Vec::new();
            for index in 0..1000 {
                let now = index * SAMPLE_MS;
                let conversion = now / conversion_ms * conversion_ms;
                let phase = (conversion as f64 * f64::from(bpm) / 60000.0).fract();
                let primary = (-((phase - 0.15) / 0.08).powi(2)).exp();
                let reflected = 0.25 * (-((phase - 0.45) / 0.08).powi(2)).exp();
                let drift = 100.0 * (conversion as f64 / 20000.0 * std::f64::consts::TAU).sin();
                let noise = (index % 7) as f64 - 3.0;
                let raw = (8000.0 + 400.0 * (primary + reflected) + drift + noise) as u32;
                if let Some(found) = estimator.sample(raw, 10, now) {
                    readings.push(found);
                }
            }
            assert!(
                readings.len() >= 3,
                "target {} BPM, ADC period {} ms",
                bpm,
                conversion_ms
            );
            assert!(
                readings
                    .iter()
                    .rev()
                    .take(5)
                    .all(|found| (i32::from(*found) - bpm).abs() <= bpm / 10),
                "target {} BPM, ADC period {} ms; recent {:?}",
                bpm,
                conversion_ms,
                readings.iter().rev().take(5).collect::<Vec<_>>()
            );
        }
    }
}

#[test]
fn stale_previous_session_and_fault_readings_are_not_valid_measurements() {
    let mut state = State {
        phase: Phase::Idle,
        last: Some(Reading {
            bpm: 72,
            at_ms: 1000,
        }),
        started_ms: 0,
    };
    assert_eq!(state.bpm(1000), Some(72));
    assert_eq!(state.measurement(1000), [0, 0]); // Cached watch value is not live BLE data.
    assert_eq!(state.measurement(31001), [0, 0]);
    assert_eq!(state.measurement(999), [0, 0]);
    state.phase = Phase::Measuring;
    state.started_ms = 2000;
    assert_eq!(state.measurement(2000), [0, 0]);
    state.last = Some(Reading {
        bpm: 75,
        at_ms: 3000,
    });
    assert_eq!(state.measurement(3000), [0, 75]);
    assert_eq!(state.measurement(5501), [0, 0]);
    assert!(!state.expired(2000 + MAX_MEASUREMENT_MS - 1));
    assert!(state.expired(2000 + MAX_MEASUREMENT_MS));
    for phase in [Phase::Missing, Phase::Fault, Phase::NoSignal] {
        state.phase = phase;
        assert_eq!(state.measurement(3000), [0, 0]);
    }
}

#[test]
fn timing_gaps_clear_estimator_history_instead_of_compressing_beats() {
    let mut estimator = Estimator::new();
    for i in 0..500 {
        let raw = (12000.0
            + (i as f64 * SAMPLE_MS as f64 / 1000.0 * std::f64::consts::TAU).sin() * 200.0)
            as u32;
        estimator.sample(raw, 10, i * SAMPLE_MS);
    }
    assert_eq!(estimator.sample(12000, 10, 75000), None);
    for i in 1..20 {
        let raw = (12000.0
            + (i as f64 * SAMPLE_MS as f64 / 1000.0 * std::f64::consts::TAU).sin() * 200.0)
            as u32;
        assert_eq!(estimator.sample(raw, 10, 75000 + i * SAMPLE_MS), None);
    }
}

#[test]
fn scheduled_jitter_uses_real_beat_times_and_diagnostics_have_a_versioned_layout() {
    let mut estimator = Estimator::new();
    let mut readings = Vec::new();
    for index in 0..750 {
        let now = index * SAMPLE_MS + if index % 10 == 0 { 20 } else { 0 };
        let raw = (12000.0 + (now as f64 / 1000.0 * std::f64::consts::TAU).sin() * 200.0) as u32;
        if let Some(bpm) = estimator.sample(raw, 10, now) {
            readings.push(bpm);
        }
    }
    assert!(readings.len() > 5);
    assert!(readings
        .iter()
        .rev()
        .take(5)
        .all(|bpm| (58..=62).contains(bpm)));
    let state = State {
        phase: Phase::Measuring,
        last: Some(Reading {
            bpm: 72,
            at_ms: 1000,
        }),
        started_ms: 0,
    };
    let diag = crate::heart_rate::Diagnostics {
        ppg: 12345,
        ambient: 678,
        min: 12000,
        max: 12400,
        samples: 25,
        at_ms: 1000,
        error: 0,
        gain: 4,
        quality: crate::heart_rate::Quality::EMPTY,
    };
    let value = diag.encode(state, 1040);
    assert_eq!(&value[..4], &[3, 1, 72, 0]);
    let packed = u16::from_le_bytes(value[4..6].try_into().unwrap());
    assert_eq!(packed & 0x0fff, 25);
    assert_eq!(packed >> 12, 4);
    assert_eq!(u32::from_le_bytes(value[6..10].try_into().unwrap()), 12345);
    assert_eq!(u32::from_le_bytes(value[10..14].try_into().unwrap()), 678);
    assert_eq!(u16::from_le_bytes(value[14..16].try_into().unwrap()), 12000);
    assert_eq!(u16::from_le_bytes(value[16..18].try_into().unwrap()), 12400);
    assert_eq!(u16::from_le_bytes(value[18..20].try_into().unwrap()), 1000);
}

#[test]
fn packed_channel_fields_decode_all_bits_from_one_frame() {
    let sample = crate::hrs3300::decode([0x5a, 0xbc, 0xed, 0, 0, 0xe7, 0xfa, 0xb6]);
    assert_eq!(sample.ppg, 0x3bcd6);
    assert_eq!(sample.ambient, (0x67 << 11) | (0x5a << 3) | 2);
}

mod transport {
    use crate::hrs3300::{Error, Hrs3300};
    use embedded_hal::i2c::{ErrorKind, ErrorType};
    use embedded_hal_async::i2c::{I2c, Operation};
    use std::{
        cell::RefCell,
        future::Future,
        rc::Rc,
        sync::Arc,
        task::{Context, Poll, Wake, Waker},
    };

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct Fault;
    impl embedded_hal::i2c::Error for Fault {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }
    struct Registers {
        values: [u8; 24],
        writes: Vec<(u8, u8)>,
        missing: bool,
        fail_write: Option<u8>,
        corrupt: bool,
    }
    struct Bus(Rc<RefCell<Registers>>);
    impl ErrorType for Bus {
        type Error = Fault;
    }
    impl I2c for Bus {
        async fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Fault> {
            assert_eq!(address, 0x44);
            let mut registers = self.0.borrow_mut();
            if registers.missing {
                return Err(Fault);
            }
            let mut register = 0;
            for operation in operations {
                match operation {
                    Operation::Write(data) if data.len() == 1 => register = data[0] as usize,
                    Operation::Write(data) => {
                        assert_eq!(data.len(), 2);
                        registers.writes.push((data[0], data[1]));
                        if registers.fail_write == Some(data[0]) {
                            return Err(Fault);
                        }
                        registers.values[data[0] as usize] = data[1];
                    }
                    Operation::Read(data) => {
                        data.copy_from_slice(&registers.values[register..register + data.len()]);
                        if registers.corrupt && register == 1 {
                            data[0] &= !0x80;
                        }
                    }
                }
            }
            Ok(())
        }
    }
    struct NoWake;
    impl Wake for NoWake {
        fn wake(self: Arc<Self>) {}
    }
    fn run<F: Future>(future: F) -> F::Output {
        let waker = Waker::from(Arc::new(NoWake));
        let mut context = Context::from_waker(&waker);
        let mut future = std::pin::pin!(future);
        match future.as_mut().poll(&mut context) {
            Poll::Ready(value) => value,
            Poll::Pending => panic!("mock bus unexpectedly blocked"),
        }
    }
    fn fixture() -> (Hrs3300<Bus>, Rc<RefCell<Registers>>) {
        let mut values = [0; 24];
        values[0] = 0x21;
        // A prior firmware may have left HRS+LED running across MCU reset.
        values[1] = 0xe0;
        values[0x0c] = 0x2f;
        let registers = Rc::new(RefCell::new(Registers {
            values,
            writes: Vec::new(),
            missing: false,
            fail_write: None,
            corrupt: false,
        }));
        (Hrs3300::new(Bus(registers.clone())), registers)
    }

    #[test]
    fn boot_probe_shuts_down_an_inherited_running_sensor_and_start_is_explicit() {
        let (mut sensor, registers) = fixture();
        run(sensor.probe()).unwrap();
        assert_eq!(registers.borrow().values[0x0c], 0);
        assert_eq!(registers.borrow().values[1] & 0x80, 0);
        assert_eq!(run(sensor.sample()), Err(Error::NotRunning));
        run(sensor.start()).unwrap();
        assert_eq!(registers.borrow().values[0x0c] & 0x60, 0x20);
        assert_eq!(registers.borrow().values[1], 0xd0);
        assert_eq!(registers.borrow().values[0x16], 0x77);
        assert_eq!(registers.borrow().values[0x17], 4); // x2; LED still 12.5 mA.
        let diag = crate::heart_rate::Diagnostics {
            gain: crate::hrs3300::MEASUREMENT_GAIN,
            ..crate::heart_rate::Diagnostics::EMPTY
        }
        .encode(crate::heart_rate::State::EMPTY, 0);
        assert_eq!(u16::from_le_bytes([diag[4], diag[5]]) >> 12, 2);
        assert!(run(sensor.sample()).is_ok());
        run(sensor.stop()).unwrap();
        assert_eq!(registers.borrow().values[0x0c], 0);
        assert_eq!(registers.borrow().values[1] & 0x80, 0);
    }

    #[test]
    fn gain_changes_only_touch_gain_register_and_reject_unsupported_values() {
        let (mut sensor, registers) = fixture();
        run(sensor.start()).unwrap();
        registers.borrow_mut().writes.clear();
        for (gain, bits) in [(1, 0), (2, 4), (4, 8), (8, 12)] {
            run(sensor.set_gain(gain)).unwrap();
            assert_eq!(registers.borrow().values[0x17], bits);
        }
        assert_eq!(
            registers.borrow().writes,
            vec![(0x17, 0), (0x17, 4), (0x17, 8), (0x17, 12)]
        );
        for invalid in [0, 3, 16, 64] {
            assert_eq!(
                run(sensor.set_gain(invalid)),
                Err(Error::InvalidGain(invalid))
            );
        }
        assert_eq!(registers.borrow().writes.len(), 4);
        assert!(run(sensor.sample()).is_ok());
        run(sensor.stop()).unwrap();
    }

    #[test]
    fn missing_and_wrong_sensor_ids_do_not_receive_writes() {
        for missing in [false, true] {
            let (mut sensor, registers) = fixture();
            registers.borrow_mut().missing = missing;
            registers.borrow_mut().values[0] = 0x42;
            assert!(run(sensor.start()).is_err());
            run(sensor.stop()).unwrap();
            assert!(registers.borrow().writes.is_empty());
        }
    }

    #[test]
    fn both_shutdown_writes_are_attempted_even_when_one_fails() {
        for failed in [0x0c, 0x01] {
            let (mut sensor, registers) = fixture();
            run(sensor.start()).unwrap();
            registers.borrow_mut().writes.clear();
            registers.borrow_mut().fail_write = Some(failed);
            assert_eq!(run(sensor.stop()), Err(Error::Bus(Fault)));
            assert_eq!(registers.borrow().writes, vec![(0x0c, 0), (1, 0x50)]);
            assert_eq!(run(sensor.sample()), Err(Error::NotRunning));
        }
    }

    #[test]
    fn failed_enable_readback_does_not_allow_sampling_and_cleanup_still_works() {
        let (mut sensor, registers) = fixture();
        registers.borrow_mut().corrupt = true;
        assert!(matches!(
            run(sensor.start()),
            Err(Error::Readback { register: 1, .. })
        ));
        assert_eq!(run(sensor.sample()), Err(Error::NotRunning));
        registers.borrow_mut().corrupt = false;
        run(sensor.stop()).unwrap();
        assert_eq!(registers.borrow().values[0x0c], 0);
    }
}
