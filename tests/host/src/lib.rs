// Compile the hardware-independent firmware modules on the host.
#![allow(dead_code)]
#[path = "../../../src/current_time.rs"]
mod current_time;
#[path = "../../../src/device/display.rs"]
mod display;

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics::{
        mono_font::{ascii::FONT_10X20, MonoTextStyle},
        pixelcolor::Rgb565,
        prelude::*,
        text::Text,
    };

    #[test]
    fn current_time_requires_complete_value() {
        let value = [0xea, 0x07, 10, 1, 12, 34, 56, 4, 0, 0];
        assert_eq!(
            current_time::parse_cts(&value).unwrap().to_string(),
            "2026-10-01 12:34:56"
        );
        for len in 0..10 {
            assert!(current_time::parse_cts(&value[..len]).is_none());
        }
        let mut oversized = value.to_vec();
        oversized.push(0);
        assert!(current_time::parse_cts(&oversized).is_none());
    }

    #[test]
    fn current_time_rejects_invalid_calendar_values() {
        let mut value = [0xea, 0x07, 2, 30, 12, 34, 56, 0, 0, 0];
        assert!(current_time::parse_cts(&value).is_none());
        value[3] = 28;
        value[4] = 24;
        assert!(current_time::parse_cts(&value).is_none());
    }

    #[test]
    fn current_time_enforces_cts_year_and_weekday_ranges() {
        let mut value = [0x2e, 0x06, 1, 1, 0, 0, 0, 0, 0, 0]; // 1582
        assert!(current_time::parse_cts(&value).is_some());

        value[0] = 0;
        value[1] = 0; // 0
        assert!(current_time::parse_cts(&value).is_none());

        value[0] = 0x0f;
        value[1] = 0x27; // 9999
        assert!(current_time::parse_cts(&value).is_some());
        value[0] = 0x10;
        value[1] = 0x27; // 10000
        assert!(current_time::parse_cts(&value).is_none());

        value[0] = 0x2e;
        value[1] = 0x06; // 1582
        value[7] = 7;
        assert!(current_time::parse_cts(&value).is_some());
        value[7] = 8;
        assert!(current_time::parse_cts(&value).is_none());
    }

    #[test]
    fn battery_clear_covers_the_longest_text_at_its_baseline() {
        let pos = Point::new(display::BATTERY_POS_X, display::BATTERY_POS_Y);
        let style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
        let text = Text::new("4.20V+ 100%", pos, style).bounding_box();
        let clear = display::text_bounds(&FONT_10X20, display::BATTERY_CHARS, pos);
        assert!(clear.contains(text.top_left));
        assert!(clear.contains(text.bottom_right().unwrap()));
    }
}
