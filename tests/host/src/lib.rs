// Compile the hardware-independent firmware modules on the host.
#![allow(dead_code)]
#[path = "../../../src/current_time.rs"]
mod current_time;
#[path = "../../../src/device/display.rs"]
mod display;
#[path = "../../../src/ota.rs"]
mod ota;

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

    #[test]
    fn ota_progress_is_bounded_and_never_claims_validation() {
        assert_eq!(
            ota::UpdateStatus::Receiving {
                received: 1,
                total: 3
            }
            .percent(),
            Some(33)
        );
        assert_eq!(
            ota::UpdateStatus::Receiving {
                received: 4,
                total: 3
            }
            .percent(),
            Some(100)
        );
        assert_eq!(
            ota::UpdateStatus::Receiving {
                received: 0,
                total: 0
            }
            .percent(),
            None
        );
        assert_eq!(ota::UpdateStatus::Validating.percent(), None);
    }

    #[test]
    fn mcuboot_magic_is_at_the_end_of_the_secondary_slot() {
        assert_eq!(ota::TRAILER_MAGIC_ADDRESS, 0x000b_3ff0);
        assert_eq!(
            ota::TRAILER_MAGIC,
            [
                0x77, 0xc2, 0x95, 0xf3, 0x60, 0xd2, 0xef, 0x7f, 0x35, 0x52, 0x50, 0x0f, 0x2c, 0xb6,
                0x79, 0x80
            ]
        );
    }

    #[test]
    fn activation_requires_full_valid_mcuboot_readback() {
        let mut header = [0; 16];
        header[..4].copy_from_slice(&[0x3d, 0xb8, 0xf3, 0x96]);
        header[8..10].copy_from_slice(&0x200u16.to_le_bytes());
        header[12..16].copy_from_slice(&0x3e00u32.to_le_bytes());
        assert!(
            ota::ActivationPlan::from_readback(0x4000, 0x4000, 0x1234, 0x1234, &header).is_some()
        );
        assert!(
            ota::ActivationPlan::from_readback(0x3fff, 0x4000, 0x1234, 0x1234, &header).is_none()
        );
        assert!(
            ota::ActivationPlan::from_readback(0x4000, 0x4000, 0x1234, 0x1235, &header).is_none()
        );
        assert!(ota::ActivationPlan::from_readback(
            ota::SECONDARY_SLOT_SIZE,
            ota::SECONDARY_SLOT_SIZE,
            0x1234,
            0x1234,
            &header
        )
        .is_none());
        header[0] = 0;
        assert!(
            ota::ActivationPlan::from_readback(0x4000, 0x4000, 0x1234, 0x1234, &header).is_none()
        );
    }

    #[cfg(feature = "ota-activation")]
    #[test]
    fn activation_writes_only_trailer_magic_and_checks_readback() {
        struct Flash {
            written: Option<(u32, Vec<u8>)>,
            corrupt_readback: bool,
        }
        impl ota::SecondarySlotFlash for Flash {
            fn write(&mut self, address: u32, data: &[u8]) -> Result<(), ()> {
                self.written = Some((address, data.to_vec()));
                Ok(())
            }
            fn read(&mut self, _address: u32, data: &mut [u8]) -> Result<(), ()> {
                data.copy_from_slice(&ota::TRAILER_MAGIC);
                if self.corrupt_readback {
                    data[0] ^= 1;
                }
                Ok(())
            }
        }
        let mut header = [0; 16];
        header[..4].copy_from_slice(&[0x3d, 0xb8, 0xf3, 0x96]);
        header[8..10].copy_from_slice(&0x200u16.to_le_bytes());
        header[12..16].copy_from_slice(&0x3e00u32.to_le_bytes());
        let plan = ota::ActivationPlan::from_readback(0x4000, 0x4000, 1, 1, &header).unwrap();
        let mut flash = Flash {
            written: None,
            corrupt_readback: false,
        };
        assert!(plan.mark_pending(&mut flash).is_ok());
        assert_eq!(
            flash.written,
            Some((ota::TRAILER_MAGIC_ADDRESS, ota::TRAILER_MAGIC.to_vec()))
        );
        let plan = ota::ActivationPlan::from_readback(0x4000, 0x4000, 1, 1, &header).unwrap();
        flash.corrupt_readback = true;
        assert!(plan.mark_pending(&mut flash).is_err());
    }
}
