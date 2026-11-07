//! Voltage estimate shared by the display, BLE telemetry and recorded history.
//! This preserves the original 3.2–4.2 V approximation with 1% resolution.
//! It is not a calibrated state-of-charge curve; keep recording millivolts.
pub const fn percent_from_mv(mv: u16) -> u8 {
    if mv >= 4200 {
        100
    } else {
        (mv.saturating_sub(3200) / 10) as u8
    }
}
