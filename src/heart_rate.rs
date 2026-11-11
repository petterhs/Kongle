//! On-demand pulse state and an integer, windowed PPG frequency estimator.
//! This is not a medically validated algorithm. Hardware waveform validation
//! and comparison with a reference are required before release.
pub const MAX_MEASUREMENT_MS: u64 = 45_000;
pub const SAMPLE_MS: u64 = 100;
pub const FRESH_MS: u64 = 30_000;
// Keep conversion range and all filter/window time scales aligned with the
// 15-bit / 10 Hz reference acquisition. These are not accuracy guarantees.
const ADC_CLIP: u32 = 0x7ff8;
const WINDOW_SAMPLES: usize = (8000 / SAMPLE_MS) as usize;
const MIN_SIMILARITY: u16 = 5500;
const MARGINAL_CONCENTRATION: u16 = 1000; // Weak periodicity needs >=10x peak/mean.
const MAX_BRIDGE_MS: u64 = 250;
const MAX_UNCERTAIN_SAMPLES: usize = 4; // At most 5% of an eight-second window.
include!("heart_rate_coefficients.rs");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Reason {
    WarmingUp = 0,
    TimingGap = 1,
    NonMonotonic = 2,
    Dark = 3,
    Clipped = 4,
    LowAmplitude = 5,
    PoorSpectrum = 6,
    Ambiguous = 7,
    OutOfRange = 8,
    NotPeriodic = 9,
    Confirming = 10,
    Valid = 11,
    InsufficientCoverage = 12,
    AmbientClipped = 13,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Quality {
    pub reason: Reason,
    pub filled: u8,
    pub candidate: u8,
    pub timing_resets: u16,
    pub invalid_samples: u16,
    pub analyses: u16,
    pub qualified: u16,
    pub amplitude: u16,
    pub concentration: u16,
    pub similarity: u16,
    pub confirmation: u8,
    pub interval_ms: u8,
}
impl Quality {
    pub const EMPTY: Self = Self {
        reason: Reason::WarmingUp,
        filled: 0,
        candidate: 0,
        timing_resets: 0,
        invalid_samples: 0,
        analyses: 0,
        qualified: 0,
        amplitude: 0,
        concentration: 0,
        similarity: 0,
        confirmation: 0,
        interval_ms: 0,
    };
    pub fn encode(self) -> [u8; 20] {
        let mut out = [0; 20];
        out[..4].copy_from_slice(&[1, self.reason as u8, self.filled, self.candidate]);
        for (i, value) in [
            self.timing_resets,
            self.invalid_samples,
            self.analyses,
            self.qualified,
            self.amplitude,
            self.concentration,
            self.similarity,
        ]
        .iter()
        .enumerate()
        {
            out[4 + i * 2..6 + i * 2].copy_from_slice(&value.to_le_bytes());
        }
        out[18] = self.confirmation;
        out[19] = self.interval_ms;
        out
    }
}

#[derive(Clone, Copy)]
pub struct Diagnostics {
    pub ppg: u32,
    pub ambient: u32,
    pub min: u16,
    pub max: u16,
    pub samples: u16,
    pub at_ms: u64,
    pub error: u8,
    pub gain: u8,
    pub quality: Quality,
}

impl Diagnostics {
    pub const EMPTY: Self = Self {
        ppg: 0,
        ambient: 0,
        min: 0,
        max: 0,
        samples: 0,
        at_ms: 0,
        error: 0,
        gain: 1,
        quality: Quality::EMPTY,
    };

    /// Version, phase, live BPM, last error, gain/count LE, PPG/ALS u32 LE,
    /// one-second PPG min/max u16 LE, sample elapsed milliseconds u16 LE.
    pub fn encode(self, state: State, now_ms: u64) -> [u8; 20] {
        let mut bytes = [0; 20];
        bytes[0] = 3;
        bytes[1] = match state.phase {
            Phase::Idle => 0,
            Phase::Measuring => 1,
            Phase::NoSignal => 2,
            Phase::Missing => 3,
            Phase::Fault => 4,
        };
        bytes[2] = state.measurement(now_ms)[1];
        bytes[3] = self.error;
        let count_and_gain = self.samples.min(0x0fff) | (u16::from(self.gain & 15) << 12);
        bytes[4..6].copy_from_slice(&count_and_gain.to_le_bytes());
        bytes[6..10].copy_from_slice(&self.ppg.to_le_bytes());
        bytes[10..14].copy_from_slice(&self.ambient.to_le_bytes());
        bytes[14..16].copy_from_slice(&self.min.to_le_bytes());
        bytes[16..18].copy_from_slice(&self.max.to_le_bytes());
        let elapsed = if self.samples == 0 {
            u16::MAX
        } else {
            self.at_ms
                .saturating_sub(state.started_ms)
                .min(u64::from(u16::MAX)) as u16
        };
        bytes[18..20].copy_from_slice(&elapsed.to_le_bytes());
        bytes
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Idle,
    Measuring,
    NoSignal,
    Missing,
    Fault,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reading {
    pub bpm: u8,
    pub at_ms: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct State {
    pub phase: Phase,
    pub last: Option<Reading>,
    pub started_ms: u64,
}

impl State {
    pub const EMPTY: Self = Self {
        phase: Phase::Idle,
        last: None,
        started_ms: 0,
    };

    pub fn bpm(self, now_ms: u64) -> Option<u8> {
        let reading = self.last?;
        if !(30..=220).contains(&reading.bpm) {
            return None;
        }
        if matches!(self.phase, Phase::Missing | Phase::Fault | Phase::NoSignal) {
            return None;
        }
        let limit = if self.phase == Phase::Measuring {
            if reading.at_ms < self.started_ms {
                return None;
            }
            2500
        } else {
            FRESH_MS
        };
        if now_ms.checked_sub(reading.at_ms)? > limit {
            return None;
        }
        Some(reading.bpm)
    }

    pub fn age_seconds(self, now_ms: u64) -> Option<u64> {
        now_ms.checked_sub(self.last?.at_ms).map(|age| age / 1000)
    }

    /// SIG 0x2A37: live uint8 BPM, contact detection unsupported. Do not replay
    /// a cached watch reading as a new phone measurement. Zero is invalid,
    /// not a real pulse; Furu's parser hides it. Subscription never starts HR.
    pub fn measurement(self, now_ms: u64) -> [u8; 2] {
        [
            0,
            if self.phase == Phase::Measuring {
                self.bpm(now_ms).unwrap_or(0)
            } else {
                0
            },
        ]
    }

    pub fn expired(self, now_ms: u64) -> bool {
        self.phase == Phase::Measuring
            && now_ms.saturating_sub(self.started_ms) >= MAX_MEASUREMENT_MS
    }
}

/// Reject isolated candidates and large rate changes. This confirms temporal
/// consistency, not physiological origin: periodic motion can still pass.
pub struct RateConfirmation {
    anchor: u8,
    count: u8,
    first_ms: u64,
    last_ms: u64,
}

impl RateConfirmation {
    pub const fn new() -> Self {
        Self {
            anchor: 0,
            count: 0,
            first_ms: 0,
            last_ms: 0,
        }
    }

    pub fn candidate(&mut self, bpm: u8, now_ms: u64) -> Option<u8> {
        if !(30..=220).contains(&bpm) {
            *self = Self::new();
            return None;
        }
        if self.count == 0
            || now_ms <= self.last_ms
            || now_ms - self.last_ms > 2500
            || bpm.abs_diff(self.anchor) > (self.anchor / 10).max(6)
        {
            self.anchor = bpm;
            self.count = 1;
            self.first_ms = now_ms;
        } else {
            self.count = self.count.saturating_add(1);
        }
        self.last_ms = now_ms;
        (self.count >= 4 && now_ms.saturating_sub(self.first_ms) >= 3000).then_some(bpm)
    }

    /// One marginal window may interrupt progress, but never produces a
    /// reading. Multiple failures expire evidence; a rate jump still resets.
    pub fn miss(&mut self, now_ms: u64) {
        if now_ms <= self.last_ms || now_ms - self.last_ms > 2500 {
            *self = Self::new();
        }
    }
}

/// Fixed-size window estimator. Independently implemented Goertzel frequency
/// search plus delayed waveform validation, rather than individual peak counts.
/// Ordinary least-squares detrending and windowing use integer arithmetic only.
pub struct Estimator {
    raw: [i32; WINDOW_SAMPLES],
    uncertain: [bool; WINDOW_SAMPLES],
    times: [u16; WINDOW_SAMPLES],
    cursor: usize,
    count: usize,
    previous_ms: Option<u64>,
    previous_ppg: i32,
    next_grid_ms: u64,
    quality: Quality,
    next_analysis_ms: u64,
    confirmation: RateConfirmation,
}

impl Estimator {
    pub const fn new() -> Self {
        Self {
            raw: [0; WINDOW_SAMPLES],
            uncertain: [false; WINDOW_SAMPLES],
            times: [0; WINDOW_SAMPLES],
            cursor: 0,
            count: 0,
            previous_ms: None,
            previous_ppg: 0,
            next_grid_ms: 0,
            quality: Quality::EMPTY,
            next_analysis_ms: 0,
            confirmation: RateConfirmation::new(),
        }
    }

    // Linear interpolation permits sub-sample periods at 10 Hz. The score
    // penalizes energy mismatch as well as phase mismatch, and is bounded by
    // Cauchy-Schwarz. i64 accumulation is safe for 80 signed 16-bit samples.
    fn similarity(wave: &[i16; WINDOW_SAMPLES], period_ms: usize) -> i32 {
        let lag = period_ms / SAMPLE_MS as usize;
        let fraction = period_ms % SAMPLE_MS as usize;
        let skip = lag + usize::from(fraction != 0);
        let mut dot = 0i64;
        let mut energy = 0i64;
        let mut a_sum = 0i64;
        let mut b_sum = 0i64;
        for i in skip..WINDOW_SAMPLES {
            let a = i64::from(wave[i]);
            let near = i64::from(wave[i - lag]);
            let b = if fraction == 0 {
                near
            } else {
                let far = i64::from(wave[i - lag - 1]);
                (near * (SAMPLE_MS as usize - fraction) as i64 + far * fraction as i64)
                    / SAMPLE_MS as i64
            };
            dot += a * b;
            energy += a * a + b * b;
            a_sum += a;
            b_sum += b;
        }
        let n = (WINDOW_SAMPLES - skip) as i64;
        dot -= a_sum * b_sum / n;
        energy -= (a_sum * a_sum + b_sum * b_sum) / n;
        if energy <= 0 || dot <= 0 {
            return 0;
        }
        (dot * 20_000 / energy) as i32
    }

    fn frequency_energy(wave: &[i16; WINDOW_SAMPLES], coefficient: i16) -> u64 {
        let (mut previous, mut before_previous) = (0i64, 0i64);
        for sample in wave {
            let next =
                i64::from(*sample) + ((i64::from(coefficient) * previous) >> 14) - before_previous;
            before_previous = previous;
            previous = next;
        }
        (previous * previous + before_previous * before_previous
            - (((i64::from(coefficient) * previous) >> 14) * before_previous))
            .max(0) as u64
    }

    pub fn quality(&self) -> Quality {
        self.quality
    }

    fn reject(&mut self, reason: Reason) -> Option<u8> {
        self.quality.reason = reason;
        None
    }

    // Zero-phase 0.35 Hz Butterworth high-pass (Q=1/sqrt(2)), then
    // a 4 Hz low-pass in each direction. Q8 states / Q30 HP coefficients.
    // Biquad formulas: https://www.w3.org/TR/audio-eq-cookbook/#hpf
    // A flat passband avoids suppressing the low-rate fundamental in favor
    // of its second harmonic. No sensor/LED gain is changed.
    fn bandpass(wave: &mut [i16; WINDOW_SAMPLES]) {
        const B0: i64 = 919048486;
        const B1: i64 = -1838096971;
        const B2: i64 = 919048486;
        const A1: i64 = -1815693534;
        const A2: i64 = 786758584;
        const LP: i64 = 30114; // 1 - exp(-2*pi*4*0.1), Q15.
        for backwards in [false, true] {
            let edge = if backwards { WINDOW_SAMPLES - 1 } else { 0 };
            let mut x1 = i64::from(wave[edge]) * 256;
            let mut x2 = x1;
            let (mut y1, mut y2) = (0i64, 0i64);
            for step in 0..WINDOW_SAMPLES {
                let index = if backwards {
                    WINDOW_SAMPLES - 1 - step
                } else {
                    step
                };
                let x = i64::from(wave[index]) * 256;
                let y = (B0 * x + B1 * x1 + B2 * x2 - A1 * y1 - A2 * y2) / (1 << 30);
                x2 = x1;
                x1 = x;
                y2 = y1;
                y1 = y;
                wave[index] = (y / 256).clamp(-32767, 32767) as i16;
            }
            let edge = if backwards { WINDOW_SAMPLES - 1 } else { 0 };
            let mut state = i64::from(wave[edge]) * 256;
            for step in 0..WINDOW_SAMPLES {
                let index = if backwards {
                    WINDOW_SAMPLES - 1 - step
                } else {
                    step
                };
                state += LP * (i64::from(wave[index]) * 256 - state) / 32768;
                wave[index] = (state / 256).clamp(-32767, 32767) as i16;
            }
        }
    }

    fn estimate(&mut self) -> Option<u8> {
        self.quality.analyses = self.quality.analyses.saturating_add(1);
        self.quality.candidate = 0;
        self.quality.concentration = 0;
        self.quality.similarity = 0;
        if self
            .uncertain
            .iter()
            .filter(|uncertain| **uncertain)
            .count()
            > MAX_UNCERTAIN_SAMPLES
        {
            return self.reject(Reason::InsufficientCoverage);
        }
        let mut wave = [0i16; WINDOW_SAMPLES];
        let mut sum = 0i64;
        let mut covariance = 0i64;
        let mut x_energy = 0i64;
        let mut min = i32::MAX;
        let mut max = 0i32;
        for i in 0..WINDOW_SAMPLES {
            let raw = self.raw[(self.cursor + i) % WINDOW_SAMPLES];
            let x = 2 * i as i64 - (WINDOW_SAMPLES - 1) as i64;
            sum += i64::from(raw);
            covariance += x * i64::from(raw);
            x_energy += x * x;
            min = min.min(raw);
            max = max.max(raw);
        }
        self.quality.amplitude = ((max - min) / 256).min(i32::from(u16::MAX)) as u16;
        if max - min < 2 * 256 {
            return self.reject(Reason::LowAmplitude);
        }
        // A good burst retained in an old part of the window must not keep
        // producing fresh readings after the optical waveform has gone flat.
        for quarter in 0..4 {
            let mut low = i32::MAX;
            let mut high = 0i32;
            for i in quarter * (WINDOW_SAMPLES / 4)..(quarter + 1) * (WINDOW_SAMPLES / 4) {
                let raw = self.raw[(self.cursor + i) % WINDOW_SAMPLES];
                low = low.min(raw);
                high = high.max(raw);
            }
            if high - low < 2 * 256 {
                return self.reject(Reason::LowAmplitude);
            }
        }
        // Normalize only for numeric precision. All quality metrics remain
        // relative to the measured waveform, never a claim of optical gain.
        let residual = |i: usize| {
            let raw = i64::from(self.raw[(self.cursor + i) % WINDOW_SAMPLES]);
            let x = 2 * i as i64 - (WINDOW_SAMPLES - 1) as i64;
            raw - sum / WINDOW_SAMPLES as i64 - covariance * x / x_energy
        };
        let scale = (0..WINDOW_SAMPLES)
            .map(|i| residual(i).abs())
            .max()
            .unwrap_or(0);
        if scale == 0 {
            return self.reject(Reason::LowAmplitude);
        }
        for (i, value) in wave.iter_mut().enumerate() {
            *value = (residual(i) * 4096 / scale) as i16;
        }
        Self::bandpass(&mut wave);
        let first = self.times[self.cursor];
        let last = self.times[(self.cursor + WINDOW_SAMPLES - 1) % WINDOW_SAMPLES];
        let span_ms = u32::from(last.wrapping_sub(first));
        // A zero/invalid time span must never yield a plausible rate.
        if span_ms == 0 {
            return self.reject(Reason::NonMonotonic);
        }
        // Hann-windowed Goertzel bank at 1 BPM grid spacing. Grid spacing is
        // not an accuracy claim. A broader 20..280 range lets out-of-policy
        // dominant signals be rejected instead of reporting a subharmonic.
        let mut windowed = [0i16; WINDOW_SAMPLES];
        for i in 0..WINDOW_SAMPLES {
            windowed[i] = (i32::from(wave[i]) * i32::from(HANN[i]) / 32767) as i16;
        }
        let mut total = 0u64;
        let mut peak_energy = 0u64;
        let mut peak_bpm = 0u32;
        for (index, coefficient) in COEFFICIENTS.iter().enumerate() {
            let energy = Self::frequency_energy(&windowed, *coefficient);
            total += energy;
            if energy > peak_energy {
                peak_energy = energy;
                peak_bpm = index as u32 + 20;
            }
        }
        self.quality.concentration = (peak_energy * COEFFICIENTS.len() as u64 * 100 / total.max(1))
            .min(u64::from(u16::MAX)) as u16;
        if peak_energy == 0 || peak_energy * (COEFFICIENTS.len() as u64) < total * 3 {
            return self.reject(Reason::PoorSpectrum);
        }
        // Reject competing frequency peaks rather than choosing a rate from
        // an ambiguous waveform. No stored spectrum or heap allocation.
        for (index, coefficient) in COEFFICIENTS.iter().enumerate().step_by(3) {
            let bpm = index as u32 + 20;
            if bpm.abs_diff(peak_bpm) > 15
                && Self::frequency_energy(&windowed, *coefficient) * 5 > peak_energy * 4
            {
                return self.reject(Reason::Ambiguous);
            }
        }
        let corrected_bpm =
            (peak_bpm * (WINDOW_SAMPLES - 1) as u32 * SAMPLE_MS as u32 + span_ms / 2) / span_ms;
        self.quality.candidate = corrected_bpm.min(255) as u8;
        if !(30..=220).contains(&corrected_bpm) {
            return self.reject(Reason::OutOfRange);
        }
        self.quality.similarity =
            Self::similarity(&wave, (60_000 / peak_bpm) as usize).clamp(0, 10000) as u16;
        if self.quality.similarity < MIN_SIMILARITY {
            return self.reject(Reason::NotPeriodic);
        }
        if self.quality.similarity < 6500 && self.quality.concentration < MARGINAL_CONCENTRATION {
            return self.reject(Reason::PoorSpectrum);
        }
        Some(corrected_bpm as u8)
    }

    fn clear_history(&mut self, reason: Reason) {
        let mut quality = self.quality;
        quality.reason = reason;
        quality.filled = 0;
        quality.candidate = 0;
        quality.confirmation = 0;
        quality.amplitude = 0;
        quality.concentration = 0;
        quality.similarity = 0;
        *self = Self::new();
        self.quality = quality;
    }

    fn push(&mut self, ppg: i32, at: u64, uncertain: bool) {
        self.raw[self.cursor] = ppg;
        self.uncertain[self.cursor] = uncertain;
        self.times[self.cursor] = at as u16;
        self.cursor = (self.cursor + 1) % WINDOW_SAMPLES;
        self.count = (self.count + 1).min(WINDOW_SAMPLES);
        self.quality.filled = self.count as u8;
    }

    pub fn sample(&mut self, ppg: u32, ambient: u32, now_ms: u64) -> Option<u8> {
        if ppg <= 8 || ppg >= ADC_CLIP || ambient >= ADC_CLIP {
            self.quality.invalid_samples = self.quality.invalid_samples.saturating_add(1);
            self.clear_history(if ambient >= ADC_CLIP {
                Reason::AmbientClipped
            } else if ppg <= 8 {
                Reason::Dark
            } else {
                Reason::Clipped
            });
            return None;
        }
        let ppg_q8 = (ppg as i32) * 256;
        if let Some(previous) = self.previous_ms {
            if now_ms <= previous || now_ms - previous > MAX_BRIDGE_MS {
                self.quality.timing_resets = self.quality.timing_resets.saturating_add(1);
                self.quality.interval_ms = now_ms.saturating_sub(previous).min(255) as u8;
                self.clear_history(if now_ms <= previous {
                    Reason::NonMonotonic
                } else {
                    Reason::TimingGap
                });
                if now_ms <= previous {
                    return None;
                }
            } else {
                let interval = now_ms - previous;
                self.quality.interval_ms = interval.min(255) as u8;
                // Bridge only brief stalls; mark every point crossing a long
                // interval so an analysis cannot be dominated by interpolation.
                while self.next_grid_ms <= now_ms {
                    let offset = self.next_grid_ms - previous;
                    let interpolated = i64::from(self.previous_ppg)
                        + i64::from(ppg_q8 - self.previous_ppg) * offset as i64 / interval as i64;
                    self.push(interpolated as i32, self.next_grid_ms, interval > 180);
                    self.next_grid_ms += SAMPLE_MS;
                }
            }
        }
        if self.previous_ms.is_none() {
            self.push(ppg_q8, now_ms, false);
            self.next_grid_ms = now_ms + SAMPLE_MS;
        }
        self.previous_ms = Some(now_ms);
        self.previous_ppg = ppg_q8;
        if self.count < WINDOW_SAMPLES {
            if !matches!(
                self.quality.reason,
                Reason::TimingGap
                    | Reason::NonMonotonic
                    | Reason::Dark
                    | Reason::Clipped
                    | Reason::AmbientClipped
            ) {
                self.quality.reason = Reason::WarmingUp;
            }
            return None;
        }
        if now_ms < self.next_analysis_ms {
            return None;
        }
        self.next_analysis_ms = now_ms + 1000;
        match self.estimate() {
            Some(bpm) => {
                let result = self.confirmation.candidate(bpm, now_ms);
                self.quality.confirmation = self.confirmation.count;
                self.quality.reason = if result.is_some() {
                    Reason::Valid
                } else {
                    Reason::Confirming
                };
                if result.is_some() {
                    self.quality.qualified = self.quality.qualified.saturating_add(1);
                }
                result
            }
            None => {
                if self.quality.reason == Reason::NotPeriodic
                    && self.quality.similarity >= 5000
                    && self.quality.concentration >= MARGINAL_CONCENTRATION
                {
                    self.confirmation.miss(now_ms);
                } else {
                    self.confirmation = RateConfirmation::new();
                }
                self.quality.confirmation = self.confirmation.count;
                None
            }
        }
    }
}
