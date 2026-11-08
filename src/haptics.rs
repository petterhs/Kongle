//! Bounded motor pulse patterns; each pulse is stopped by TIMER2/PPI.
#[derive(Clone, Copy)]
pub enum Pattern {
    Notification,
    Connected,
    Disconnected,
}

impl Pattern {
    pub const fn pulses_ms(self) -> &'static [u32] {
        match self {
            Self::Notification => &[50],
            Self::Connected => &[180],
            Self::Disconnected => &[50, 50],
        }
    }
}
