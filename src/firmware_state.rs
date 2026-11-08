//! Confirmation policy and touch flow, independent of the flash driver.
use crate::ota;

pub const ERASED: u32 = u32::MAX;
pub const FLAG_SET: u32 = 0xffff_ff01;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootState {
    Standalone,
    NotTrial,
    Trial,
    Confirmed,
    Invalid,
}

pub fn classify(copy_done: u32, image_ok: u32, magic: &[u8; 16]) -> BootState {
    if copy_done == ERASED && image_ok == ERASED && *magic == [0xff; 16] {
        return BootState::NotTrial;
    }
    if *magic != ota::TRAILER_MAGIC {
        return BootState::Invalid;
    }
    match (copy_done, image_ok) {
        (FLAG_SET, ERASED) => BootState::Trial,
        (FLAG_SET | ERASED, FLAG_SET) => BootState::Confirmed,
        _ => BootState::Invalid,
    }
}

/// Only an erased word in a valid completed trial may be programmed. No erase.
pub fn confirmation_word(copy_done: u32, image_ok: u32, magic: &[u8; 16]) -> Option<[u8; 4]> {
    (classify(copy_done, image_ok, magic) == BootState::Trial).then_some(FLAG_SET.to_le_bytes())
}

pub struct Confirmation {
    pub state: BootState,
    pub armed: bool,
    requested: bool,
}

impl Confirmation {
    pub const fn new() -> Self {
        Self {
            state: BootState::Standalone,
            armed: false,
            requested: false,
        }
    }
    pub fn cancel(&mut self) -> bool {
        let changed = self.armed || self.requested;
        self.armed = false;
        self.requested = false;
        changed
    }
    pub fn set_state(&mut self, state: BootState) {
        self.cancel();
        self.state = state;
    }
    pub fn tap(&mut self, y: u8) -> bool {
        if self.state != BootState::Trial {
            return false;
        }
        if self.armed && (118..=160).contains(&y) {
            self.armed = false;
            self.requested = true;
            return true;
        }
        if (183..=230).contains(&y) {
            // The original button becomes Cancel. Repeated reports cannot confirm.
            self.armed = !self.armed;
            return true;
        }
        false
    }
    pub fn take_request(&mut self) -> bool {
        core::mem::take(&mut self.requested)
    }
}
