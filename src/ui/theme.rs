//! Catppuccin Mocha, palette v1.8.0 (MIT, attribution in docs/design/mocha-tui.md).
use embedded_graphics::pixelcolor::Rgb565;

const fn rgb(r: u8, g: u8, b: u8) -> Rgb565 {
    Rgb565::new(r >> 3, g >> 2, b >> 3)
}
// The LCD makes Mocha's base look gray. Keep black and use Mocha for foregrounds.
pub const BACKGROUND: Rgb565 = rgb(0, 0, 0);
pub const TEXT: Rgb565 = rgb(0xcd, 0xd6, 0xf4);
pub const MUTED: Rgb565 = rgb(0xa6, 0xad, 0xc8);
pub const BORDER: Rgb565 = rgb(0x89, 0xb4, 0xfa);
pub const DISABLED: Rgb565 = rgb(0x58, 0x5b, 0x70);
pub const ACCENT: Rgb565 = rgb(0x89, 0xb4, 0xfa);
pub const BLUETOOTH: Rgb565 = rgb(0x89, 0xb4, 0xfa); // Mocha blue
pub const HEADING: Rgb565 = rgb(0xcb, 0xa6, 0xf7); // Mauve
pub const CLOCK: Rgb565 = HEADING;
pub const STEPS: Rgb565 = rgb(0xa6, 0xe3, 0xa1); // Green
pub const ERROR: Rgb565 = rgb(0xf3, 0x8b, 0xa8);
