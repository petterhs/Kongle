//! Debounced edge interpretation, independent of the input pin and clock.

pub const DOUBLE_PRESS_MS: u64 = 350;
const MAX_DOUBLE_HOLD_MS: u64 = 600;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Click {
    First,
    Double,
}

pub struct Gestures {
    pending: Option<u64>,
    pressed_at: u64,
    second: bool,
    first_short: bool,
}

impl Gestures {
    pub const fn new() -> Self {
        Self {
            pending: None,
            pressed_at: 0,
            second: false,
            first_short: false,
        }
    }

    pub fn press(&mut self, now_ms: u64) {
        let within_window = self
            .pending
            .take()
            .is_some_and(|deadline| now_ms <= deadline);
        self.second = self.first_short && within_window;
        self.pressed_at = now_ms;
    }

    pub fn release(&mut self, now_ms: u64) -> Click {
        if self.second && now_ms.saturating_sub(self.pressed_at) <= MAX_DOUBLE_HOLD_MS {
            self.cancel();
            Click::Double
        } else {
            self.second = false;
            self.first_short = now_ms.saturating_sub(self.pressed_at) <= MAX_DOUBLE_HOLD_MS;
            self.pending = Some(now_ms.saturating_add(DOUBLE_PRESS_MS));
            Click::First
        }
    }

    pub const fn deadline(&self) -> Option<u64> {
        self.pending
    }

    pub fn expire(&mut self, now_ms: u64) -> bool {
        if self.pending.is_some_and(|deadline| now_ms >= deadline) {
            self.pending = None;
            true
        } else {
            false
        }
    }

    pub fn cancel(&mut self) {
        self.pending = None;
        self.second = false;
        self.first_short = false;
    }
}
