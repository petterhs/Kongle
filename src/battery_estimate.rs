//! Approximate PineTime state of charge; retain measured mV for calibration.
//!
//! These measured-curve reference points are from InfiniTime 1.16.1's battery
//! controller (see docs/battery-history.md). Interpolation is implemented here
//! independently; this is a provisional curve, not a calibrated fuel gauge.
const DISCHARGE_CURVE: [(u16, u8); 6] = [
    (3500, 0),
    (3616, 3),
    (3723, 22),
    (3776, 48),
    (3979, 79),
    (4180, 100),
];

pub const fn percent_from_mv(mv: u16) -> u8 {
    if mv <= DISCHARGE_CURVE[0].0 {
        return 0;
    }
    let mut i = 1;
    while i < DISCHARGE_CURVE.len() {
        let (high_mv, high_percent) = DISCHARGE_CURVE[i];
        if mv <= high_mv {
            let (low_mv, low_percent) = DISCHARGE_CURVE[i - 1];
            let rise = (mv - low_mv) as u32 * (high_percent - low_percent) as u32;
            return low_percent + (rise / (high_mv - low_mv) as u32) as u8;
        }
        i += 1;
    }
    100
}

/// 12-bit SAADC, internal 600 mV reference, gain 1/6 and a 1:2 divider.
/// Multiply before dividing so the conversion does not discard odd millivolts.
pub const fn millivolts_from_adc(raw: i16) -> u16 {
    let code = if raw < 0 {
        0
    } else if raw > 4095 {
        4095
    } else {
        raw as u32
    };
    (code * 7200 / 4096) as u16
}
