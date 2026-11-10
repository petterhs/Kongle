use chrono::Datelike;
use core::fmt::Write;
use heapless::String;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Language {
    English,
    Bokmal,
}

impl Language {
    pub fn toggle(&mut self) {
        *self = match self {
            Self::English => Self::Bokmal,
            Self::Bokmal => Self::English,
        };
    }

    pub const fn text(self, english: &'static str, bokmal: &'static str) -> &'static str {
        match self {
            Self::English => english,
            Self::Bokmal => bokmal,
        }
    }

    pub fn date(self, year: i32, month: u32, day: u32) -> String<32> {
        let mut out = String::new();
        let Some(date) = chrono::NaiveDate::from_ymd_opt(year, month, day) else {
            return out;
        };
        let (weekdays, months) = match self {
            Self::English => (
                [
                    "Monday",
                    "Tuesday",
                    "Wednesday",
                    "Thursday",
                    "Friday",
                    "Saturday",
                    "Sunday",
                ],
                [
                    "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov",
                    "Dec",
                ],
            ),
            Self::Bokmal => (
                [
                    "mandag", "tirsdag", "onsdag", "torsdag", "fredag", "lørdag", "søndag",
                ],
                [
                    "jan", "feb", "mar", "apr", "mai", "jun", "jul", "aug", "sep", "okt", "nov",
                    "des",
                ],
            ),
        };
        let weekday = weekdays[date.weekday().num_days_from_monday() as usize];
        let month = months[date.month0() as usize];
        if self == Self::Bokmal {
            let _ = write!(out, "{weekday} {day}. {month}");
        } else {
            let _ = write!(out, "{weekday} {day} {month}");
        }
        out
    }
}
