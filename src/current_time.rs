use chrono::NaiveDateTime;

/// Parse an exactly 10-byte Current Time characteristic value.
pub fn parse_cts(data: &[u8]) -> Option<NaiveDateTime> {
    if data.len() != 10 {
        return None;
    }
    let year = u16::from_le_bytes([data[0], data[1]]) as i32;
    // Bluetooth Current Time Service permits years 1582–9999 and weekday 0
    // (unknown) through 7 (Sunday).
    if !(1582..=9999).contains(&year) || data[7] > 7 {
        return None;
    }
    let date = chrono::NaiveDate::from_ymd_opt(year, data[2] as u32, data[3] as u32)?;
    let time = chrono::NaiveTime::from_hms_opt(data[4] as u32, data[5] as u32, data[6] as u32)?;
    Some(date.and_time(time))
}

/// Advance one second without publishing a year outside the CTS range.
pub fn next_cts_second(now: NaiveDateTime) -> NaiveDateTime {
    let max = chrono::NaiveDate::from_ymd_opt(9999, 12, 31)
        .and_then(|date| date.and_hms_opt(23, 59, 59))
        .expect("maximum CTS datetime is valid");
    if now >= max {
        max
    } else {
        now + chrono::Duration::seconds(1)
    }
}
