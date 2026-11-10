//! On-demand pulse state and a conservative, integer PPG baseline estimator.
//! This is not a medically validated algorithm. Hardware waveform validation
//! and comparison with a reference are required before release.
pub const MAX_MEASUREMENT_MS: u64 = 45_000;
pub const SAMPLE_MS: u64 = 40;
pub const FRESH_MS: u64 = 30_000;
// HRS and ALS use the driver's fixed 0x66 (14-bit) configuration.
const ADC_CLIP: u32 = 0x3ff8;

/// Bounded analog calibration with headroom and hysteresis. Do not lower the
/// pulse-quality checks to turn a weak/noisy signal into a plausible number.
pub fn target_gain(current: u8, min: u32, max: u32) -> u8 {
    if min > max || ![1, 2, 4, 8].contains(&current) {
        return current;
    }
    if max >= 12_000 && current > 1 {
        return current / 2;
    }
    let midpoint = min + (max - min) / 2;
    if max < 5500 && current < 8 && (midpoint < 4000 || max - min < 32) {
        return current * 2;
    }
    current
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
    };

    /// Version, phase, live BPM, last error, gain/count LE, PPG/ALS u32 LE,
    /// one-second PPG min/max u16 LE, sample age milliseconds u16 LE.
    pub fn encode(self, state: State, now_ms: u64) -> [u8; 20] {
        let mut bytes = [0; 20];
        bytes[0] = 2;
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
        let age = if self.samples == 0 {
            u16::MAX
        } else {
            now_ms.saturating_sub(self.at_ms).min(u64::from(u16::MAX)) as u16
        };
        bytes[18..20].copy_from_slice(&age.to_le_bytes());
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

pub struct Estimator {
    dc: Option<i32>,
    smooth: [i32; 3],
    cursor: usize,
    envelope: i32,
    previous: [i32; 2],
    previous_ms: Option<u64>,
    started_ms: Option<u64>,
    peak_ms: Option<u64>,
    intervals: [u32; 4],
    interval_count: usize,
    waveform: [i16; 200],
    waveform_cursor: usize,
    waveform_count: usize,
}

impl Estimator {
    pub const fn new() -> Self {
        Self {
            dc: None,
            smooth: [0; 3],
            cursor: 0,
            envelope: 0,
            previous: [0; 2],
            previous_ms: None,
            started_ms: None,
            peak_ms: None,
            intervals: [0; 4],
            interval_count: 0,
            waveform: [0; 200],
            waveform_cursor: 0,
            waveform_count: 0,
        }
    }

    fn periodic(&self, lag: usize) -> bool {
        if lag == 0 || self.waveform_count < lag * 3 {
            return false;
        }
        let count = self.waveform_count;
        let start = (self.waveform_cursor + 200 - count) % 200;
        let (mut dot, mut aa, mut bb, mut a_sum, mut b_sum) = (0i64, 0i64, 0i64, 0i64, 0i64);
        for index in lag..count {
            let a = i64::from(self.waveform[(start + index) % 200]);
            let b = i64::from(self.waveform[(start + index - lag) % 200]);
            dot += a * b;
            aa += a * a;
            bb += b * b;
            a_sum += a;
            b_sum += b;
        }
        let pairs = (count - lag) as i64;
        dot -= a_sum * b_sum / pairs;
        aa -= a_sum * a_sum / pairs;
        bb -= b_sum * b_sum / pairs;
        // Normalized delayed similarity >= 0.65, without floats or sqrt.
        aa > 0 && bb > 0 && dot > 0 && dot * 2000 >= (aa + bb) * 650
    }

    /// Returns a new reading only on a qualifying beat. Timing uses actual
    /// monotonic timestamps; delayed BLE/I2C scheduling cannot fabricate BPM.
    pub fn sample(&mut self, ppg: u32, ambient: u32, now_ms: u64) -> Option<u8> {
        let previous_ms = self.previous_ms;
        if ppg <= 8
            || ppg >= ADC_CLIP
            || ambient >= ADC_CLIP
            || previous_ms.is_some_and(|at| now_ms <= at || now_ms - at > 120)
        {
            *self = Self::new();
            return None;
        }
        self.previous_ms = Some(now_ms);
        let start = *self.started_ms.get_or_insert(now_ms);
        let raw = ppg.min(0x3ffff) as i32;
        let dc = self.dc.get_or_insert(raw << 8);
        *dc += ((raw << 8) - *dc) / 32;
        self.smooth[self.cursor] = raw - (*dc >> 8);
        self.cursor = (self.cursor + 1) % 3;
        let filtered = self.smooth.iter().sum::<i32>() / 3;
        self.waveform[self.waveform_cursor] = filtered.clamp(-32767, 32767) as i16;
        self.waveform_cursor = (self.waveform_cursor + 1) % 200;
        self.waveform_count = (self.waveform_count + 1).min(200);
        self.envelope += (filtered.abs() - self.envelope) / 16;
        let is_peak = self.previous[1] > self.previous[0]
            && self.previous[1] >= filtered
            && self.previous[1] > self.envelope.max(8);
        self.previous = [self.previous[1], filtered];
        if now_ms.saturating_sub(start) < 2000 || !is_peak {
            return None;
        }
        let peak = previous_ms?;
        let previous = match self.peak_ms {
            Some(previous) => previous,
            None => {
                self.peak_ms = Some(peak);
                return None;
            }
        };
        let elapsed = peak.checked_sub(previous)?;
        if elapsed < 240 {
            return None;
        } // Allow 40 ms sampling quantization; final rate is range-checked.
        self.peak_ms = Some(peak);
        if elapsed > 2040 {
            self.interval_count = 0;
            return None;
        }
        self.intervals.rotate_left(1);
        self.intervals[3] = elapsed as u32;
        self.interval_count = (self.interval_count + 1).min(4);
        if self.interval_count < 4 {
            return None;
        }
        let mut sorted = self.intervals;
        sorted.sort_unstable();
        let median = (sorted[1] + sorted[2]) / 2;
        if self
            .intervals
            .iter()
            .any(|v| v.abs_diff(median) > median / 5 + 40)
        {
            return None;
        }
        let bpm = (60_000 + median / 2) / median;
        ((30..=220).contains(&bpm)
            && self.periodic(((u64::from(median) + SAMPLE_MS / 2) / SAMPLE_MS) as usize))
        .then_some(bpm as u8)
    }
}
