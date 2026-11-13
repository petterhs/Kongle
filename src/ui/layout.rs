//! Shared geometry for rendering and hit testing on full-height pages.
use crate::touch::TouchEvent;
use embedded_graphics::{prelude::*, primitives::Rectangle};

pub const BRIGHTNESS: Rectangle = Rectangle::new(Point::new(16, 26), Size::new(208, 60));
pub const BRIGHTNESS_SELECTION: Rectangle = Rectangle::new(Point::new(16, 50), Size::new(208, 28));
pub const VIBRATION: Rectangle = Rectangle::new(Point::new(16, 92), Size::new(208, 42));
pub const DISPLAY: Rectangle = Rectangle::new(Point::new(16, 140), Size::new(208, 42));
pub const SYSTEM: Rectangle = Rectangle::new(Point::new(16, 188), Size::new(208, 42));
pub const BATTERY_PERCENT: Rectangle = Rectangle::new(Point::new(16, 34), Size::new(208, 50));
pub const LANGUAGE: Rectangle = Rectangle::new(Point::new(16, 94), Size::new(208, 74));
pub const ABOUT: Rectangle = Rectangle::new(Point::new(16, 34), Size::new(208, 50));
pub const FIRMWARE: Rectangle = Rectangle::new(Point::new(16, 86), Size::new(208, 50));
#[cfg(feature = "hr-test-profiles")]
pub const HEART_PROFILE: Rectangle = Rectangle::new(Point::new(16, 202), Size::new(208, 28));
pub const HEART_MEASURE: Rectangle = Rectangle::new(Point::new(16, 148), Size::new(208, 50));

pub fn tapped(event: TouchEvent, rect: Rectangle) -> bool {
    match event {
        TouchEvent::Tap { x, y } => rect.contains(Point::new(x as i32, y as i32)),
        _ => false,
    }
}
