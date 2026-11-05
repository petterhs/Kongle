//! Bounded, volatile inbox for Furu's Alert Notification Service writes.

use heapless::{String, Vec};

pub const MAX_ALERT_BYTES: usize = 103;
const INBOX_CAPACITY: usize = 4;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Notification {
    pub title: String<32>,
    pub body: String<88>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    InvalidLength,
    InvalidHeader,
    MissingSeparator,
}

fn display_text<const N: usize>(bytes: &[u8]) -> String<N> {
    let valid = core::str::from_utf8(bytes)
        .unwrap_or_else(|err| core::str::from_utf8(&bytes[..err.valid_up_to()]).unwrap_or(""));
    let mut result = String::new();
    for ch in valid.chars() {
        let display = if ch.is_ascii_graphic() || ch == ' ' {
            ch
        } else if ch.is_whitespace() {
            ' '
        } else {
            '?'
        };
        if result.push(display).is_err() {
            break;
        }
    }
    result
}

/// Furu sends `[category, count, reserved, title, 0, body]`, capped at 103 bytes.
/// Only the displayable prefix is kept; malformed writes are rejected by GATT.
pub fn parse_new_alert(data: &[u8]) -> Result<Notification, ParseError> {
    if data.len() < 5 || data.len() > MAX_ALERT_BYTES {
        return Err(ParseError::InvalidLength);
    }
    if data[0] > 9 || data[1] == 0 {
        return Err(ParseError::InvalidHeader);
    }
    let text = &data[3..];
    let Some(split) = text.iter().position(|&byte| byte == 0) else {
        return Err(ParseError::MissingSeparator);
    };
    let title = display_text(&text[..split]);
    let body = display_text(&text[split + 1..]);
    Ok(Notification { title, body })
}

pub struct Inbox {
    items: Vec<Notification, INBOX_CAPACITY>,
    selected: usize,
}

impl Inbox {
    pub const fn new() -> Self {
        Self {
            items: Vec::new(),
            selected: 0,
        }
    }

    pub fn push(&mut self, notification: Notification) {
        if self.items.len() == INBOX_CAPACITY {
            self.items.pop();
        }
        let _ = self.items.insert(0, notification);
        self.selected = 0;
    }

    pub fn selected(&self) -> Option<&Notification> {
        self.items.get(self.selected)
    }

    pub fn count(&self) -> usize {
        self.items.len()
    }

    pub fn position(&self) -> usize {
        self.selected + 1
    }

    pub fn older(&mut self) -> bool {
        if self.selected + 1 >= self.items.len() {
            return false;
        }
        self.selected += 1;
        true
    }

    pub fn newer(&mut self) -> bool {
        if self.selected == 0 {
            return false;
        }
        self.selected -= 1;
        true
    }
}
