use chrono::NaiveDateTime;

/// Parse an exactly 10-byte Current Time characteristic value.
pub fn parse_cts(data: &[u8]) -> Option<NaiveDateTime> {
    if data.len() != 10 {
        return None;
    }
    let year = u16::from_le_bytes([data[0], data[1]]) as i32;
    let date = chrono::NaiveDate::from_ymd_opt(year, data[2] as u32, data[3] as u32)?;
    let time = chrono::NaiveTime::from_hms_opt(data[4] as u32, data[5] as u32, data[6] as u32)?;
    Some(date.and_time(time))
}
