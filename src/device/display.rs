use display_interface::WriteOnlyDataCommand;
use embedded_graphics::{
    geometry::Point, geometry::Size, mono_font::MonoFont, primitives::Rectangle,
};
use embedded_graphics::{pixelcolor::Rgb565, prelude::RgbColor};
use embedded_hal::{delay::DelayNs, digital::OutputPin};
use mipidsi::options::{ColorInversion, ColorOrder, Orientation, Rotation};
use mipidsi::{
    dcs::{
        BitsPerPixel, Dcs, EnterNormalMode, ExitSleepMode, PixelFormat, SetAddressMode,
        SetDisplayOn, SetInvertMode, SetPixelFormat, SoftReset,
    },
    error::{Error, InitError},
    models::{Model, ST7789},
    options::ModelOptions,
};

/// ST7789 initialization shared by the PineTime's V2 and P3 controllers.
///
/// The upstream mipidsi 0.8 model sends Sleep Out before MADCTL and waits only
/// 10 ms. On ST7789P3 this can leave the image rotated. Program the registers
/// while asleep, then wait 120 ms after Sleep Out before turning the panel on.
pub struct PineTimeSt7789;

impl Model for PineTimeSt7789 {
    type ColorFormat = Rgb565;
    const FRAMEBUFFER_SIZE: (u16, u16) = (240, 320);

    fn init<RST, DELAY, DI>(
        &mut self,
        dcs: &mut Dcs<DI>,
        delay: &mut DELAY,
        options: &ModelOptions,
        rst: &mut Option<RST>,
    ) -> Result<SetAddressMode, InitError<RST::Error>>
    where
        RST: OutputPin,
        DELAY: DelayNs,
        DI: WriteOnlyDataCommand,
    {
        let madctl = SetAddressMode::from(options);
        match rst {
            Some(rst) => self.hard_reset(rst, delay)?,
            None => dcs.write_command(SoftReset)?,
        }
        delay.delay_us(150_000);

        dcs.write_command(madctl)?;
        dcs.write_command(SetInvertMode::new(options.invert_colors))?;
        let pixel_format = PixelFormat::with_all(BitsPerPixel::from_rgb_color::<Rgb565>());
        dcs.write_command(SetPixelFormat::new(pixel_format))?;

        dcs.write_command(ExitSleepMode)?;
        delay.delay_us(120_000);
        dcs.write_command(EnterNormalMode)?;
        delay.delay_us(10_000);
        dcs.write_command(SetDisplayOn)?;
        delay.delay_us(120_000);
        Ok(madctl)
    }

    fn write_pixels<DI, I>(&mut self, dcs: &mut Dcs<DI>, colors: I) -> Result<(), Error>
    where
        DI: WriteOnlyDataCommand,
        I: IntoIterator<Item = Self::ColorFormat>,
    {
        ST7789.write_pixels(dcs, colors)
    }
}

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

pub const BLE_POS_X: i32 = 220;
pub const BLE_POS_Y: i32 = 18;

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
