use defmt::Format;
use embassy_nrf::gpio::Output;

/// Backlight control for PineTime display
///
/// The PineTime has three active-low backlight pins, each connected to a FET:
/// - Low: 2.2 kΩ resistor
/// - Mid: 100 Ω resistor  
/// - High: 30 Ω resistor
///
/// Through combinations, 8 brightness levels (0-7) can be configured.
pub struct Backlight {
    low: Output<'static>,
    mid: Output<'static>,
    high: Output<'static>,
    brightness: u8,
}

impl Backlight {
    /// Initialize backlight with specified brightness (0-7)
    pub fn new(
        low_pin: Output<'static>,
        mid_pin: Output<'static>,
        high_pin: Output<'static>,
        brightness: u8,
    ) -> Self {
        let mut backlight = Self {
            low: low_pin,
            mid: mid_pin,
            high: high_pin,
            brightness: 0,
        };
        backlight.set(brightness);
        backlight
    }

    /// Set brightness level (0-7, where 0 = off, 7 = max brightness)
    pub fn set(&mut self, mut brightness: u8) {
        if brightness > 7 {
            brightness = 7;
        }

        // Active-low logic - set low to turn on
        if brightness & 0x01 != 0 {
            self.low.set_low();
        } else {
            self.low.set_high();
        }

        if brightness & 0x02 != 0 {
            self.mid.set_low();
        } else {
            self.mid.set_high();
        }

        if brightness & 0x04 != 0 {
            self.high.set_low();
        } else {
            self.high.set_high();
        }

        self.brightness = brightness;
    }

    /// Turn off backlight
    pub fn off(&mut self) {
        self.set(0);
    }

    /// Increase brightness by one level (wraps to 0 after 7)
    pub fn brighter(&mut self) {
        self.set((self.brightness + 1) % 8);
    }

    /// Decrease brightness by one level (wraps to 7 after 0)
    pub fn darker(&mut self) {
        self.set(self.brightness.wrapping_sub(1));
    }

    /// Get current brightness level (0-7)
    pub fn get_brightness(&self) -> u8 {
        self.brightness
    }
}

impl Format for Backlight {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Backlight: {}", self.brightness)
    }
}
