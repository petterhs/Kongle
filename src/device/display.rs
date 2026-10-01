use embedded_graphics::{
    geometry::Point, geometry::Size, mono_font::MonoFont, primitives::Rectangle,
};
use embedded_graphics::{pixelcolor::Rgb565, prelude::RgbColor};
use mipidsi::options::{ColorInversion, ColorOrder, Orientation, Rotation};

pub const DISPLAY_WIDTH: u16 = 240;
pub const DISPLAY_HEIGHT: u16 = 240;
pub const DISPLAY_OFFSET_X: u16 = 0;
pub const DISPLAY_OFFSET_Y: u16 = 0;
pub const DISPLAY_ORIENTATION: Orientation = Orientation::new().rotate(Rotation::Deg0);
pub const DISPLAY_COLOR_ORDER: ColorOrder = ColorOrder::Bgr;
pub const DISPLAY_COLOR_INVERSION: ColorInversion = ColorInversion::Inverted;

pub const BACKGROUND_COLOR: Rgb565 = Rgb565::BLACK;
pub const TEXT_COLOR: Rgb565 = Rgb565::WHITE;

pub const TIME_POS_X: i32 = 6;
pub const TIME_POS_Y: i32 = 100;
pub const TIME_CHARS: u32 = 5;

pub const SECONDS_POS_X: i32 = 178;
pub const SECONDS_POS_Y: i32 = 180;
pub const SECONDS_CHARS: u32 = 2;

pub const BATTERY_POS_X: i32 = 10;
pub const BATTERY_POS_Y: i32 = 18;
pub const BATTERY_CHARS: u32 = 11;

pub const UPDATE_POS_X: i32 = 10;
pub const UPDATE_POS_Y: i32 = 226;
pub const UPDATE_CHARS: u32 = 22;

pub fn text_bounds(font: &MonoFont, chars: u32, pos: Point) -> Rectangle {
    let char_w = font.character_size.width + font.character_spacing;
    let width = char_w
        .checked_mul(chars)
        .unwrap_or(0)
        .saturating_sub(font.character_spacing);
    let height = font.character_size.height;
    Rectangle::new(
        pos - Point::new(0, font.baseline as i32),
        Size::new(width, height),
    )
}
