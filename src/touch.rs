//! CST816S touch event decoding, independent of the I2C driver.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(target_arch = "arm", derive(defmt::Format))]
pub enum TouchEvent {
    Contact,
    Tap { x: u8, y: u8 },
    DoubleTap { x: u8, y: u8 },
    SwipeUp,
    SwipeDown,
    SwipeLeft,
    SwipeRight,
}

/// Registers 0x01..=0x06. Gestures are reported by the controller; position
/// is only trusted for taps because release/gesture frames can lack a point.
pub fn decode(frame: [u8; 6]) -> Option<TouchEvent> {
    let x = (((frame[2] & 0x0f) as u16) << 8) | frame[3] as u16;
    let y = (((frame[4] & 0x0f) as u16) << 8) | frame[5] as u16;
    match frame[0] {
        0x00 if frame[1] & 0x0f != 0 => Some(TouchEvent::Contact),
        0x01 => Some(TouchEvent::SwipeDown),
        0x02 => Some(TouchEvent::SwipeUp),
        0x03 => Some(TouchEvent::SwipeLeft),
        0x04 => Some(TouchEvent::SwipeRight),
        0x05 if x < 240 && y < 240 => Some(TouchEvent::Tap {
            x: x as u8,
            y: y as u8,
        }),
        0x0b if x < 240 && y < 240 => Some(TouchEvent::DoubleTap {
            x: x as u8,
            y: y as u8,
        }),
        _ => None,
    }
}
