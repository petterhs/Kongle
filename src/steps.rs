//! RAM-only daily accounting, independent of the Bosch hardware counter.
use chrono::NaiveDate;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct State {
    pub date: Option<NaiveDate>,
    pub daily: Option<u32>,
    pub session: Option<u32>,
    pub unassigned: u32,
}

impl State {
    pub const EMPTY: Self = Self {
        date: None,
        daily: None,
        session: None,
        unassigned: 0,
    };

    /// A real zero is valid; unavailable/undated/stale-date data is not zero.
    pub fn gatt_value(self, current_date: Option<NaiveDate>) -> Option<[u8; 4]> {
        if self.date.is_none() || self.date != current_date {
            return None;
        }
        self.daily.map(u32::to_le_bytes)
    }

    pub fn displayed(self) -> Option<u32> {
        self.daily.or(self.session)
    }
}

#[derive(Clone, Copy)]
struct Bucket {
    date: NaiveDate,
    total: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    MonotonicWentBackwards,
    CounterDiscontinuity { previous: u32, current: u32 },
}

pub struct Counts {
    buckets: [Option<Bucket>; 3],
    next_bucket: usize,
    previous_raw: u32,
    previous_ms: u64,
    previous_date: Option<NaiveDate>,
    session: u32,
    unassigned: u32,
}

impl Counts {
    /// The application has just reset/initialized the sensor, whose count is 0.
    pub const fn new(now_ms: u64) -> Self {
        Self {
            buckets: [None; 3],
            next_bucket: 0,
            previous_raw: 0,
            previous_ms: now_ms,
            previous_date: None,
            session: 0,
            unassigned: 0,
        }
    }

    fn bucket(&mut self, date: NaiveDate) -> usize {
        if let Some(index) = self
            .buckets
            .iter()
            .position(|b| b.is_some_and(|b| b.date == date))
        {
            return index;
        }
        let index = self.next_bucket;
        self.next_bucket = (self.next_bucket + 1) % self.buckets.len();
        self.buckets[index] = Some(Bucket { date, total: 0 });
        index
    }

    pub fn observe(
        &mut self,
        raw: u32,
        now_ms: u64,
        date: Option<NaiveDate>,
    ) -> Result<State, Error> {
        let elapsed = now_ms
            .checked_sub(self.previous_ms)
            .ok_or(Error::MonotonicWentBackwards)?;
        let delta = raw.wrapping_sub(self.previous_raw);
        let previous = self.previous_raw;
        let previous_date = self.previous_date;
        self.previous_raw = raw;
        self.previous_ms = now_ms;
        self.previous_date = date;
        let bucket = date.map(|date| self.bucket(date));
        // Detect reset/corruption, rather than turning an unsigned subtraction
        // into billions of steps. Allow delayed reporting plus ample cadence.
        // This is a sanity bound, not gait filtering or accuracy tuning.
        let limit = 64u64.saturating_add((elapsed / 1000 + 1).saturating_mul(12));
        if u64::from(delta) > limit {
            return Err(Error::CounterDiscontinuity {
                previous,
                current: raw,
            });
        }
        self.session = self.session.saturating_add(delta);
        match bucket {
            Some(index) if previous_date == date => {
                let bucket = self.buckets[index].as_mut().unwrap();
                bucket.total = bucket.total.saturating_add(delta);
            }
            _ => {
                // No per-step timestamps exist. A batch crossing midnight or
                // first clock sync cannot honestly be assigned to either day.
                self.unassigned = self.unassigned.saturating_add(delta);
            }
        }
        let daily = bucket.map(|index| self.buckets[index].unwrap().total);
        Ok(State {
            date,
            daily,
            session: Some(self.session),
            unassigned: self.unassigned,
        })
    }

    /// Retain accounting across transient read errors, but publish no value.
    pub fn unavailable(&self) -> State {
        State {
            date: self.previous_date,
            unassigned: self.unassigned,
            ..State::EMPTY
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn date(day: u32) -> Option<NaiveDate> {
        NaiveDate::from_ymd_opt(2026, 11, day)
    }

    #[test]
    fn unsynced_steps_remain_visible_but_are_not_fabricated_daily_counts() {
        let mut counts = Counts::new(0);
        let state = counts.observe(10, 5000, None).unwrap();
        assert_eq!(state.displayed(), Some(10));
        assert_eq!(state.gatt_value(None), None);
        assert_eq!(state.unassigned, 10);
        let state = counts.observe(15, 10000, date(10)).unwrap();
        assert_eq!(state.daily, Some(0));
        assert_eq!(state.unassigned, 15);
        let state = counts.observe(25, 15000, date(10)).unwrap();
        assert_eq!(state.daily, Some(10));
        assert_eq!(state.gatt_value(date(10)), Some(10u32.to_le_bytes()));
        assert_eq!(state.gatt_value(date(11)), None);
    }

    #[test]
    fn midnight_and_backwards_clock_corrections_never_recount_old_steps() {
        let mut counts = Counts::new(0);
        counts.observe(0, 0, date(10)).unwrap();
        assert_eq!(counts.observe(20, 5000, date(10)).unwrap().daily, Some(20));
        let state = counts.observe(22, 6000, date(11)).unwrap();
        assert_eq!(state.daily, Some(0));
        assert_eq!(state.unassigned, 2);
        assert_eq!(counts.observe(32, 11000, date(11)).unwrap().daily, Some(10));
        let state = counts.observe(34, 12000, date(10)).unwrap();
        assert_eq!(state.daily, Some(20));
        assert_eq!(state.unassigned, 4);
        assert_eq!(counts.observe(39, 14000, date(10)).unwrap().daily, Some(25));
        assert_eq!(counts.observe(39, 15000, date(11)).unwrap().daily, Some(10));
        assert_eq!(counts.observe(39, 16000, date(10)).unwrap().daily, Some(25));
    }

    #[test]
    fn counter_reset_is_not_unsigned_wrap_and_reads_resume_from_new_baseline() {
        let mut counts = Counts::new(0);
        counts.observe(0, 0, date(10)).unwrap();
        counts.observe(40, 5000, date(10)).unwrap();
        assert_eq!(
            counts.observe(1, 6000, date(10)),
            Err(Error::CounterDiscontinuity {
                previous: 40,
                current: 1
            })
        );
        assert_eq!(counts.unavailable().gatt_value(date(10)), None);
        let state = counts.observe(6, 8000, date(10)).unwrap();
        assert_eq!(state.daily, Some(45));
        assert_eq!(state.session, Some(45));
    }

    #[test]
    fn real_wrap_and_missing_reads_preserve_deltas_and_monotonic_errors_do_not_mutate() {
        let mut counts = Counts::new(0);
        counts.observe(0, 0, date(10)).unwrap();
        counts.previous_raw = u32::MAX - 4;
        assert_eq!(counts.observe(5, 5000, date(10)).unwrap().daily, Some(10));
        let _ = counts.unavailable(); // A failed read must not become a zero sample.
        assert_eq!(counts.observe(10, 9000, date(10)).unwrap().daily, Some(15));
        assert_eq!(
            counts.observe(12, 8000, date(10)),
            Err(Error::MonotonicWentBackwards)
        );
        assert_eq!(counts.observe(15, 10000, date(10)).unwrap().daily, Some(20));
    }

    #[test]
    fn missing_read_across_midnight_does_not_assign_the_whole_gap_to_today() {
        let mut counts = Counts::new(0);
        counts.observe(0, 0, date(10)).unwrap();
        counts.observe(10, 5000, date(10)).unwrap();
        assert_eq!(counts.unavailable().daily, None);
        let state = counts.observe(30, 35000, date(11)).unwrap();
        assert_eq!(state.daily, Some(0));
        assert_eq!(state.unassigned, 20);
        let state = counts.observe(40, 40000, date(11)).unwrap();
        assert_eq!(state.daily, Some(10));
        assert_eq!(state.session, Some(40));
        assert_eq!(counts.observe(40, 41000, date(10)).unwrap().daily, Some(10));
    }
}
