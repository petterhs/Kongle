//! Clock-screen rendering. The application task owns the display and forwards
//! state changes here; BLE, input, and OTA tasks never draw directly.

use core::fmt::Write;

use chrono::Datelike;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Line, PrimitiveStyle, PrimitiveStyleBuilder},
    text::Text,
};
use heapless::String;

use crate::{
    device::{battery::BatteryState, display as cfg},
    fonts::JETBRAINS_FONT_54_POINT_EXTRA_BOLD,
    ota::UpdateStatus,
    TimeState, FIRMWARE_VERSION,
};

fn update_text(status: UpdateStatus) -> String<22> {
    let mut text = String::new();
    match status {
        UpdateStatus::Idle => {}
        UpdateStatus::Erasing => {
            let _ = text.push_str("Preparing update");
        }
        UpdateStatus::Receiving { .. } => {
            let _ = write!(&mut text, "Update {}%", status.percent().unwrap_or(0));
        }
        UpdateStatus::Validating => {
            let _ = text.push_str("Verifying update");
        }
        UpdateStatus::Staged => {
            let _ = text.push_str("Update staged");
        }
        UpdateStatus::ReadyToRestart => {
            let _ = text.push_str("Update ready");
        }
        UpdateStatus::Trial => {
            let _ = text.push_str("Trial: reset reverts");
        }
        UpdateStatus::Failed => {
            let _ = text.push_str("Update failed");
        }
    }
    text
}

fn date_text(time: TimeState) -> String<16> {
    const WEEKDAYS: [&str; 7] = ["MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"];
    const MONTHS: [&str; 12] = [
        "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC",
    ];
    let mut text = String::new();
    if let Some(date) =
        chrono::NaiveDate::from_ymd_opt(time.year as i32, time.month as u32, time.day as u32)
    {
        let weekday = WEEKDAYS[date.weekday().num_days_from_monday() as usize];
        let month = MONTHS[date.month0() as usize];
        let _ = write!(&mut text, "{} {:02} {}", weekday, time.day, month);
    }
    text
}

pub struct WatchUi {
    last_time: Option<TimeState>,
    last_seconds: Option<u8>,
    last_battery: Option<BatteryState>,
}

impl WatchUi {
    pub const fn new() -> Self {
        Self {
            last_time: None,
            last_seconds: None,
            last_battery: None,
        }
    }

    pub fn draw_static<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        let divider = Line::new(
            Point::new(cfg::DATE_POS_X, cfg::DATE_DIVIDER_Y),
            Point::new(
                cfg::DISPLAY_WIDTH as i32 - cfg::DATE_POS_X,
                cfg::DATE_DIVIDER_Y,
            ),
        )
        .into_styled(PrimitiveStyle::with_stroke(cfg::ACCENT_COLOR, 2));
        let _ = divider.draw(display);
        let version_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(cfg::TEXT_COLOR)
            .background_color(cfg::BACKGROUND_COLOR)
            .build();
        let _ = Text::new(
            FIRMWARE_VERSION,
            Point::new(cfg::VERSION_POS_X, cfg::VERSION_POS_Y),
            version_style,
        )
        .draw(display);
    }

    pub fn draw_time<D: DrawTarget<Color = Rgb565>>(&mut self, display: &mut D, time: TimeState) {
        let clear_style = PrimitiveStyleBuilder::new()
            .fill_color(cfg::BACKGROUND_COLOR)
            .build();
        if self
            .last_time
            .map(|t| (t.year, t.month, t.day) != (time.year, time.month, time.day))
            .unwrap_or(true)
        {
            let pos = Point::new(cfg::DATE_POS_X, cfg::DATE_POS_Y);
            cfg::text_bounds(&FONT_10X20, cfg::DATE_CHARS, pos)
                .into_styled(clear_style)
                .draw(display)
                .ok();
            let style = MonoTextStyleBuilder::new()
                .font(&FONT_10X20)
                .text_color(cfg::ACCENT_COLOR)
                .background_color(cfg::BACKGROUND_COLOR)
                .build();
            let text = date_text(time);
            let _ = Text::new(&text, pos, style).draw(display);
        }
        if self
            .last_time
            .map(|t| t.hours != time.hours || t.minutes != time.minutes)
            .unwrap_or(true)
        {
            let pos = Point::new(cfg::TIME_POS_X, cfg::TIME_POS_Y);
            cfg::text_bounds(&JETBRAINS_FONT_54_POINT_EXTRA_BOLD, cfg::TIME_CHARS, pos)
                .into_styled(clear_style)
                .draw(display)
                .ok();
            let style = MonoTextStyleBuilder::new()
                .font(&JETBRAINS_FONT_54_POINT_EXTRA_BOLD)
                .text_color(cfg::TEXT_COLOR)
                .background_color(cfg::BACKGROUND_COLOR)
                .build();
            let mut text: String<8> = String::new();
            let _ = write!(&mut text, "{:02}:{:02}", time.hours, time.minutes);
            let _ = Text::new(&text, pos, style).draw(display);
        }
        if self.last_seconds != Some(time.seconds) {
            let pos = Point::new(cfg::SECONDS_POS_X, cfg::SECONDS_POS_Y);
            cfg::text_bounds(&FONT_10X20, cfg::SECONDS_CHARS, pos)
                .into_styled(clear_style)
                .draw(display)
                .ok();
            let style = MonoTextStyleBuilder::new()
                .font(&FONT_10X20)
                .text_color(cfg::TEXT_COLOR)
                .background_color(cfg::BACKGROUND_COLOR)
                .build();
            let mut text: String<2> = String::new();
            let _ = write!(&mut text, "{:02}", time.seconds);
            let _ = Text::new(&text, pos, style).draw(display);
            self.last_seconds = Some(time.seconds);
        }
        self.last_time = Some(time);
    }

    pub fn draw_battery<D: DrawTarget<Color = Rgb565>>(
        &mut self,
        display: &mut D,
        state: BatteryState,
    ) {
        if self.last_battery == Some(state) {
            return;
        }
        let pos = Point::new(cfg::BATTERY_POS_X, cfg::BATTERY_POS_Y);
        let clear_style = PrimitiveStyleBuilder::new()
            .fill_color(cfg::BACKGROUND_COLOR)
            .build();
        cfg::text_bounds(&FONT_10X20, cfg::BATTERY_CHARS, pos)
            .into_styled(clear_style)
            .draw(display)
            .ok();
        let mut text: String<16> = String::new();
        let charging = if state.charging { "+" } else { "" };
        let _ = write!(
            &mut text,
            "{}.{:02}V{} {}%",
            state.mv / 1000,
            (state.mv % 1000) / 10,
            charging,
            state.percent
        );
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(cfg::TEXT_COLOR)
            .background_color(cfg::BACKGROUND_COLOR)
            .build();
        let _ = Text::new(&text, pos, style).draw(display);
        self.last_battery = Some(state);
    }

    pub fn draw_connection<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D, connected: bool) {
        let pos = Point::new(cfg::BLE_POS_X, cfg::BLE_POS_Y);
        let clear_style = PrimitiveStyleBuilder::new()
            .fill_color(cfg::BACKGROUND_COLOR)
            .build();
        cfg::text_bounds(&FONT_10X20, 1, pos)
            .into_styled(clear_style)
            .draw(display)
            .ok();
        if connected {
            let style = MonoTextStyleBuilder::new()
                .font(&FONT_10X20)
                .text_color(cfg::TEXT_COLOR)
                .background_color(cfg::BACKGROUND_COLOR)
                .build();
            let _ = Text::new("B", pos, style).draw(display);
        }
    }

    pub fn draw_update<D: DrawTarget<Color = Rgb565>>(
        &self,
        display: &mut D,
        status: UpdateStatus,
    ) {
        let pos = Point::new(cfg::UPDATE_POS_X, cfg::UPDATE_POS_Y);
        let clear_style = PrimitiveStyleBuilder::new()
            .fill_color(cfg::BACKGROUND_COLOR)
            .build();
        cfg::text_bounds(&FONT_10X20, cfg::UPDATE_CHARS, pos)
            .into_styled(clear_style)
            .draw(display)
            .ok();
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(cfg::TEXT_COLOR)
            .background_color(cfg::BACKGROUND_COLOR)
            .build();
        let text = update_text(status);
        let _ = Text::new(&text, pos, style).draw(display);
    }

    pub fn draw_full<D: DrawTarget<Color = Rgb565>>(
        &mut self,
        display: &mut D,
        time: Option<TimeState>,
        battery: Option<BatteryState>,
        status: UpdateStatus,
        connected: bool,
    ) {
        self.last_time = None;
        self.last_seconds = None;
        self.last_battery = None;
        self.draw_static(display);
        self.draw_connection(display, connected);
        if let Some(time) = time {
            self.draw_time(display, time);
        }
        if let Some(battery) = battery {
            self.draw_battery(display, battery);
        }
        self.draw_update(display, status);
    }
}
