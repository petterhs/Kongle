// Compile the hardware-independent firmware modules on the host.
#![allow(dead_code)]
#[path = "../../../src/current_time.rs"]
mod current_time;
#[path = "../../../src/device/display.rs"]
mod display;

#[cfg(test)]
mod tests {
    use super::*;
    use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
    use embedded_graphics::{
        mono_font::{ascii::FONT_10X20, MonoTextStyle},
        pixelcolor::Rgb565,
        prelude::*,
        text::Text,
    };
    use embedded_hal::delay::DelayNs;
    use mipidsi::{dcs::Dcs, models::Model, options::ModelOptions, NoResetPin};
    use std::{
        cell::{Cell, RefCell},
        rc::Rc,
    };

    #[test]
    fn pinetime_display_programs_orientation_before_sleep_out() {
        struct TraceDisplay {
            commands: Rc<RefCell<Vec<(u8, u64)>>>,
            elapsed_ns: Rc<Cell<u64>>,
        }

        impl WriteOnlyDataCommand for TraceDisplay {
            fn send_commands(&mut self, data: DataFormat<'_>) -> Result<(), DisplayError> {
                if let DataFormat::U8(bytes) = data {
                    for &byte in bytes {
                        self.commands
                            .borrow_mut()
                            .push((byte, self.elapsed_ns.get()));
                    }
                }
                Ok(())
            }

            fn send_data(&mut self, _data: DataFormat<'_>) -> Result<(), DisplayError> {
                Ok(())
            }
        }

        struct TraceDelay(Rc<Cell<u64>>);
        impl DelayNs for TraceDelay {
            fn delay_ns(&mut self, ns: u32) {
                self.0.set(self.0.get() + u64::from(ns));
            }
        }

        let commands = Rc::new(RefCell::new(Vec::new()));
        let elapsed_ns = Rc::new(Cell::new(0));
        let mut dcs = Dcs::write_only(TraceDisplay {
            commands: commands.clone(),
            elapsed_ns: elapsed_ns.clone(),
        });
        let mut delay = TraceDelay(elapsed_ns);
        let mut reset: Option<NoResetPin> = None;
        display::PineTimeSt7789
            .init(
                &mut dcs,
                &mut delay,
                &ModelOptions::with_all((240, 240), (0, 0)),
                &mut reset,
            )
            .unwrap();

        let commands = commands.borrow();
        let time_of = |instruction| {
            let (index, (_, at)) = commands
                .iter()
                .enumerate()
                .find(|(_, (command, _))| *command == instruction)
                .unwrap();
            (index, *at)
        };
        let (madctl_index, _) = time_of(0x36);
        let (sleep_out_index, sleep_out_at) = time_of(0x11);
        let (display_on_index, display_on_at) = time_of(0x29);
        assert!(madctl_index < sleep_out_index);
        assert!(sleep_out_index < display_on_index);
        assert!(display_on_at - sleep_out_at >= 120_000_000);
    }

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
    fn current_time_clock_saturates_at_maximum_cts_datetime() {
        let max = chrono::NaiveDate::from_ymd_opt(9999, 12, 31)
            .unwrap()
            .and_hms_opt(23, 59, 59)
            .unwrap();
        assert_eq!(current_time::next_cts_second(max), max);
        assert_eq!(
            current_time::next_cts_second(max - chrono::Duration::seconds(1)),
            max
        );
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
