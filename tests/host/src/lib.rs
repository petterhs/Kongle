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
    fn legacy_init_packet_and_crc_match_furu() {
        let payload = b"123456789";
        let crc = ota::crc16_update(0xffff, payload);
        assert_eq!(crc, 0x29b1);
        assert_eq!(
            ota::crc16_update(ota::crc16_update(0xffff, &payload[..4]), &payload[4..]),
            crc
        );
        let mut init = [0; 14];
        init[..2].copy_from_slice(&0x0052u16.to_le_bytes());
        init[8..10].copy_from_slice(&1u16.to_le_bytes());
        init[12..14].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(ota::expected_crc16(&init), Some(crc));
        init[0] = 0;
        assert_eq!(ota::expected_crc16(&init), None);
    }

    #[test]
    fn ota_erase_order_and_write_bounds_protect_other_partitions() {
        assert_eq!(ota::SECONDARY_SECTORS, 116);
        assert_eq!(ota::erase_sector_address(0), Some(0xB3000));
        assert_eq!(ota::erase_sector_address(1), Some(0x40000));
        assert_eq!(ota::erase_sector_address(115), Some(0xB2000));
        assert_eq!(ota::erase_sector_address(116), None);
        assert!(ota::image_range_ok(0x40000, ota::MAX_IMAGE_SIZE as usize));
        assert!(!ota::image_range_ok(0x3ffff, 1));
        assert!(!ota::image_range_ok(ota::TRAILER_MAGIC_ADDRESS, 1));
        assert!(!ota::image_range_ok(u32::MAX, 2));
    }

    #[test]
    fn dfu_rejects_oversize_and_truncated_packets() {
        let mut size = [0; 12];
        size[8..].copy_from_slice(&ota::MAX_IMAGE_SIZE.to_le_bytes());
        assert_eq!(
            ota::parse_application_size(&size),
            Some(ota::MAX_IMAGE_SIZE)
        );
        size[8..].copy_from_slice(&(ota::MAX_IMAGE_SIZE + 1).to_le_bytes());
        assert_eq!(ota::parse_application_size(&size), None);
        size[0] = 1;
        assert_eq!(ota::parse_application_size(&size), None);
        assert!(ota::packet_len_ok(20, 21));
        assert!(ota::packet_len_ok(1, 1));
        assert!(!ota::packet_len_ok(1, 21));
        assert!(!ota::packet_len_ok(21, 21));
        assert!(!ota::packet_len_ok(0, 20));
    }

    #[test]
    fn unconfirmed_trial_cannot_erase_rollback_slot() {
        assert_eq!(ota::PRIMARY_COPY_DONE_ADDRESS, 0x7bfe0);
        assert_eq!(ota::PRIMARY_IMAGE_OK_ADDRESS, 0x7bfe8);
        assert!(ota::primary_allows_staging(0xff, 0xff));
        assert!(ota::primary_allows_staging(1, 1));
        assert!(!ota::primary_allows_staging(1, 0xff));
        assert!(!ota::primary_allows_staging(0, 0xff));
        assert!(!ota::primary_allows_staging(0, 1));
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
        // Values from pinetime-mcuboot-app-dfu-1.16.1.zip: a 32-byte header,
        // 386176-byte payload and 386248-byte transfer with CRC 0xf40b.
        header[8..10].copy_from_slice(&0x20u16.to_le_bytes());
        header[12..16].copy_from_slice(&386176u32.to_le_bytes());
        assert!(
            ota::ActivationPlan::from_readback(386248, 386248, 0xf40b, 0xf40b, &header).is_some()
        );
        assert!(
            ota::ActivationPlan::from_readback(386248, 386248, 0xf40b, 0xf40c, &header).is_none()
        );
        header[8..10].copy_from_slice(&0x200u16.to_le_bytes());
        header[12..16].copy_from_slice(&0x3e00u32.to_le_bytes());
        header[8..10].copy_from_slice(&0x100u16.to_le_bytes());
        assert!(
            ota::ActivationPlan::from_readback(0x4000, 0x4000, 0x1234, 0x1234, &header).is_some()
        );
        for invalid_header_size in [0x10u16, 0x21, 0x4000] {
            header[8..10].copy_from_slice(&invalid_header_size.to_le_bytes());
            assert!(
                ota::ActivationPlan::from_readback(0x4000, 0x4000, 0x1234, 0x1234, &header)
                    .is_none()
            );
        }
        header[8..10].copy_from_slice(&0x200u16.to_le_bytes());
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
    fn verified_image_requests_only_a_trial_marker() {
        let mut header = [0; 16];
        header[..4].copy_from_slice(&[0x3d, 0xb8, 0xf3, 0x96]);
        header[8..10].copy_from_slice(&0x200u16.to_le_bytes());
        header[12..16].copy_from_slice(&0x3e00u32.to_le_bytes());
        let plan = ota::ActivationPlan::from_readback(0x4000, 0x4000, 1, 1, &header).unwrap();
        assert_eq!(
            plan.trial_marker(),
            (ota::TRAILER_MAGIC_ADDRESS, ota::TRAILER_MAGIC)
        );
    }
}
