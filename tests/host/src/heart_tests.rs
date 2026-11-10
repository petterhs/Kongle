use crate::heart_rate::{Estimator, Phase, Reading, State, MAX_MEASUREMENT_MS};

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
            let now = index * 40;
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
fn analog_gain_calibration_handles_observed_weak_signal_and_retains_headroom() {
    use crate::heart_rate::target_gain;
    for (gain, min, max, expected) in [
        (1, 860, 864, 2),
        (2, 1720, 1728, 4),
        (4, 3440, 3456, 8),
        (8, 6880, 6912, 8),
        (8, 12000, 15000, 4),
        (4, 6000, 7500, 4), // After reducing gain, do not bounce back up.
        (1, 12000, 16383, 1),
        (2, 5500, 5600, 2),
        (8, 0, 0, 8), // No x64 fallback for darkness.
        (1, 864, 860, 1),
        (64, 860, 864, 64),
    ] {
        assert_eq!(target_gain(gain, min, max), expected);
    }
}

#[test]
fn analog_amplification_before_quantization_can_recover_a_weak_periodic_signal() {
    for gain in [1, 8] {
        let mut estimator = Estimator::new();
        let mut readings = Vec::new();
        for index in 0..1000 {
            let now = index * 40;
            let phase = now as f64 * 72.0 / 60000.0 * std::f64::consts::TAU;
            // Model analog amplification before ADC quantization, not digital
            // multiplication of already quantized sensor samples.
            let raw = ((861.0 + 2.0 * phase.sin()) * f64::from(gain)).round() as u32;
            if let Some(bpm) = estimator.sample(raw, 6, now) {
                readings.push(bpm);
            }
        }
        if gain == 1 {
            assert!(readings.is_empty());
        } else {
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
            let now = sample * 40;
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
    for raw in [0, 1, 10000, 0x3fff, 0x3ffff] {
        let mut estimator = Estimator::new();
        for i in 0..750 {
            assert_eq!(estimator.sample(raw, 10, i * 40), None);
        }
    }
    let mut estimator = Estimator::new();
    let mut random = 0x12345678u32;
    for i in 0..1500 {
        random = random.wrapping_mul(1664525).wrapping_add(1013904223);
        let raw = 10000 + random % 1000;
        assert_eq!(estimator.sample(raw, 10, i * 40), None, "noise at {}", i);
    }
}

#[test]
fn pulse_shaped_waveforms_with_slow_adc_updates_and_drift_are_not_just_sine_tests() {
    for bpm in [60, 72, 90] {
        for conversion_ms in [40, 75, 100] {
            let mut estimator = Estimator::new();
            let mut readings = Vec::new();
            for index in 0..1000 {
                let now = index * 40;
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
        let raw = (12000.0 + (i as f64 * 0.04 * std::f64::consts::TAU).sin() * 200.0) as u32;
        estimator.sample(raw, 10, i * 40);
    }
    assert_eq!(estimator.sample(12000, 10, 25000), None);
    for i in 1..60 {
        let raw = (12000.0 + (i as f64 * 0.04 * std::f64::consts::TAU).sin() * 200.0) as u32;
        assert_eq!(estimator.sample(raw, 10, 25000 + i * 40), None);
    }
}

#[test]
fn scheduled_jitter_uses_real_beat_times_and_diagnostics_have_a_versioned_layout() {
    let mut estimator = Estimator::new();
    let mut readings = Vec::new();
    for index in 0..750 {
        let now = index * 40 + if index % 10 == 0 { 20 } else { 0 };
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
        assert_eq!(registers.borrow().values[1] & 0x88, 0x80);
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
            assert_eq!(registers.borrow().writes, vec![(0x0c, 0), (1, 0x60)]);
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
