//! Recent battery samples kept while the phone is out of range.
//!
//! Timestamps are monotonic uptime seconds. Furu anchors them to phone time
//! when it reconnects, so a missing or stale CTS clock cannot invent dates.

use heapless::Deque;

pub const CAPACITY: usize = 512;
const SAMPLE_INTERVAL_SECS: u32 = 5 * 60;
const PAGE_RECORDS: usize = 18;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sample {
    pub uptime_secs: u32,
    /// UTC seconds, or zero until the phone has supplied an epoch anchor.
    pub epoch_secs: u32,
    pub mv: u16,
    pub percent: u8,
    pub charging: bool,
}

pub struct History {
    samples: Deque<Sample, CAPACITY>,
    next_seq: u32,
    epoch_anchor: Option<(u32, u32)>,
}

impl History {
    pub const fn new() -> Self {
        Self {
            samples: Deque::new(),
            next_seq: 0,
            epoch_anchor: None,
        }
    }

    pub fn record(
        &mut self,
        uptime_secs: u32,
        mv: u16,
        percent: u8,
        charging: bool,
    ) -> Option<Sample> {
        let should_record = self.samples.back().is_none_or(|last| {
            uptime_secs < last.uptime_secs
                || last.charging != charging
                || uptime_secs.saturating_sub(last.uptime_secs) >= SAMPLE_INTERVAL_SECS
        });
        if !should_record {
            return None;
        }
        if self.samples.is_full() {
            self.samples.pop_front();
        }
        let epoch_secs = self
            .epoch_anchor
            .map_or(0, |(anchor_uptime, anchor_epoch)| {
                anchor_epoch.saturating_add(uptime_secs.saturating_sub(anchor_uptime))
            });
        let sample = Sample {
            uptime_secs,
            epoch_secs,
            mv,
            percent,
            charging,
        };
        self.samples.push_back(sample).ok();
        self.next_seq = self.next_seq.wrapping_add(1);
        Some(sample)
    }

    pub fn set_epoch(&mut self, uptime_secs: u32, epoch_secs: u32) {
        self.epoch_anchor = Some((uptime_secs, epoch_secs));
        for sample in self.samples.iter_mut() {
            if sample.epoch_secs == 0 && sample.uptime_secs <= uptime_secs {
                sample.epoch_secs = epoch_secs.saturating_sub(uptime_secs - sample.uptime_secs);
            }
        }
    }

    /// Restore only samples with known UTC time from a compatible filesystem.
    pub fn restore(&mut self, sample: Sample) {
        if sample.epoch_secs == 0 {
            return;
        }
        if self.samples.is_full() {
            self.samples.pop_front();
        }
        self.samples.push_back(sample).ok();
        self.next_seq = self.next_seq.wrapping_add(1);
    }

    pub fn page(&self, cursor: u32, now_uptime_secs: u32) -> heapless::Vec<u8, 240> {
        let oldest = self.next_seq.saturating_sub(self.samples.len() as u32);
        let base = cursor.max(oldest).min(self.next_seq);
        let count = (self.next_seq - base).min(PAGE_RECORDS as u32) as usize;
        let mut bytes = heapless::Vec::<u8, 240>::new();
        bytes.push(1).unwrap();
        bytes.extend_from_slice(&base.to_le_bytes()).unwrap();
        bytes
            .extend_from_slice(&(base + count as u32).to_le_bytes())
            .unwrap();
        bytes.push(count as u8).unwrap();
        bytes
            .extend_from_slice(&now_uptime_secs.to_le_bytes())
            .unwrap();
        let start = (base - oldest) as usize;
        for sample in self.samples.iter().skip(start).take(count) {
            bytes
                .extend_from_slice(&sample.uptime_secs.to_le_bytes())
                .unwrap();
            bytes
                .extend_from_slice(&sample.epoch_secs.to_le_bytes())
                .unwrap();
            bytes.extend_from_slice(&sample.mv.to_le_bytes()).unwrap();
            bytes.push(sample.percent).unwrap();
            bytes.push(u8::from(sample.charging)).unwrap();
        }
        bytes
    }
}
