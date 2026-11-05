//! Clock-screen rendering. The application task owns the display and forwards
//! state changes here; BLE, input, and OTA tasks never draw directly.

mod navigation;

use core::fmt::Write;

use chrono::Datelike;
use embedded_graphics::{
    image::Image,
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use embedded_iconoir::{
    icons::size24px::{connectivity::Bluetooth, system},
    prelude::IconoirIcon,
    Icon,
};
use heapless::String;

use crate::{
    device::{battery::BatteryState, display as cfg},
    fonts::JETBRAINS_FONT_54_POINT_EXTRA_BOLD,
    ota::UpdateStatus,
    touch::TouchEvent,
    TimeState, FIRMWARE_VERSION,
};
use navigation::{Navigation, Page};

fn draw_icon<D: DrawTarget<Color = Rgb565>, I: IconoirIcon>(
    display: &mut D,
    position: Point,
    color: Rgb565,
) {
    let icon = Icon::<Rgb565, I>::new(color);
    let _ = Image::new(&icon, position).draw(display);
}

fn draw_battery_icon<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    position: Point,
    state: BatteryState,
) {
    let color = if state.charging {
        cfg::ACCENT_COLOR
    } else if state.percent < 15 {
        Rgb565::new(31, 0, 0)
    } else {
        cfg::TEXT_COLOR
    };
    if state.charging {
        draw_icon::<D, system::BatteryCharging>(display, position, color);
    } else if state.percent >= 88 {
        draw_icon::<D, system::BatteryFull>(display, position, color);
    } else if state.percent >= 63 {
        draw_icon::<D, system::BatterySevenFive>(display, position, color);
    } else if state.percent >= 38 {
        draw_icon::<D, system::BatteryFiveZero>(display, position, color);
    } else if state.percent >= 13 {
        draw_icon::<D, system::BatteryTwoFive>(display, position, color);
    } else {
        draw_icon::<D, system::BatteryEmpty>(display, position, color);
    }
}

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
    navigation: Navigation,
    show_battery_percent: bool,
    last_time: Option<TimeState>,
    last_battery: Option<BatteryState>,
    skip_clear: bool,
}

impl WatchUi {
    pub const fn new() -> Self {
        Self {
            navigation: Navigation::new(),
            show_battery_percent: true,
            last_time: None,
            last_battery: None,
            skip_clear: false,
        }
    }

    pub fn handle_touch(&mut self, event: TouchEvent) -> bool {
        if self.navigation.page() == Page::Settings
            && matches!(event, TouchEvent::Tap { y: 70..=130, .. })
        {
            self.show_battery_percent = !self.show_battery_percent;
            return true;
        }
        self.navigation.handle(event)
    }

    /// OTA progress must remain visible even if another page was open.
    pub fn show_clock(&mut self) -> bool {
        self.navigation.clock()
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
    }

    pub fn draw_time<D: DrawTarget<Color = Rgb565>>(&mut self, display: &mut D, time: TimeState) {
        if self.navigation.page() != Page::Clock {
            return;
        }
        let clear_style = PrimitiveStyleBuilder::new()
            .fill_color(cfg::BACKGROUND_COLOR)
            .build();
        if self
            .last_time
            .map(|t| (t.year, t.month, t.day) != (time.year, time.month, time.day))
            .unwrap_or(true)
        {
            let pos = Point::new(cfg::DATE_POS_X, cfg::DATE_POS_Y);
            if !self.skip_clear {
                cfg::text_bounds(&FONT_10X20, cfg::DATE_CHARS, pos)
                    .into_styled(clear_style)
                    .draw(display)
                    .ok();
            }
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
            if !self.skip_clear {
                cfg::text_bounds(&JETBRAINS_FONT_54_POINT_EXTRA_BOLD, cfg::TIME_CHARS, pos)
                    .into_styled(clear_style)
                    .draw(display)
                    .ok();
            }
            let style = MonoTextStyleBuilder::new()
                .font(&JETBRAINS_FONT_54_POINT_EXTRA_BOLD)
                .text_color(cfg::TEXT_COLOR)
                .background_color(cfg::BACKGROUND_COLOR)
                .build();
            let mut text: String<8> = String::new();
            let _ = write!(&mut text, "{:02}:{:02}", time.hours, time.minutes);
            let _ = Text::new(&text, pos, style).draw(display);
        }
        self.last_time = Some(time);
    }

    pub fn draw_battery<D: DrawTarget<Color = Rgb565>>(
        &mut self,
        display: &mut D,
        state: BatteryState,
    ) {
        let page = self.navigation.page();
        if !matches!(page, Page::Clock | Page::About) {
            return;
        }
        if self.last_battery == Some(state) {
            return;
        }
        let clear_style = PrimitiveStyleBuilder::new()
            .fill_color(cfg::BACKGROUND_COLOR)
            .build();
        let charging = if state.charging { "+" } else { "" };
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(cfg::TEXT_COLOR)
            .background_color(cfg::BACKGROUND_COLOR)
            .build();
        if page == Page::Clock {
            let icon_pos = Point::new(cfg::BATTERY_ICON_POS_X, cfg::STATUS_ICON_POS_Y);
            let percent_pos = Point::new(cfg::BATTERY_PERCENT_POS_X, cfg::BATTERY_POS_Y);
            if !self.skip_clear {
                Rectangle::new(
                    icon_pos,
                    Size::new(cfg::STATUS_ICON_SIZE, cfg::STATUS_ICON_SIZE),
                )
                .into_styled(clear_style)
                .draw(display)
                .ok();
                cfg::text_bounds(&FONT_10X20, cfg::BATTERY_CHARS, percent_pos)
                    .into_styled(clear_style)
                    .draw(display)
                    .ok();
            }
            draw_battery_icon(display, icon_pos, state);
            if self.show_battery_percent {
                let mut text: String<5> = String::new();
                let _ = write!(&mut text, "{}%{}", state.percent, charging);
                let _ = Text::new(&text, percent_pos, style).draw(display);
            }
        } else {
            let percent_pos = Point::new(cfg::ABOUT_BATTERY_POS_X, cfg::ABOUT_BATTERY_PERCENT_Y);
            let voltage_pos = Point::new(cfg::ABOUT_BATTERY_POS_X, cfg::ABOUT_BATTERY_VOLTAGE_Y);
            if !self.skip_clear {
                for pos in [percent_pos, voltage_pos] {
                    cfg::text_bounds(&FONT_10X20, cfg::ABOUT_BATTERY_CHARS, pos)
                        .into_styled(clear_style)
                        .draw(display)
                        .ok();
                }
            }
            let mut percent: String<20> = String::new();
            let _ = write!(&mut percent, "Battery: {}%{}", state.percent, charging);
            let _ = Text::new(&percent, percent_pos, style).draw(display);
            let mut voltage: String<20> = String::new();
            let _ = write!(
                &mut voltage,
                "Voltage: {}.{:02}V",
                state.mv / 1000,
                (state.mv % 1000) / 10
            );
            let _ = Text::new(&voltage, voltage_pos, style).draw(display);
        }
        self.last_battery = Some(state);
    }

    pub fn draw_connection<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D, connected: bool) {
        let pos = Point::new(cfg::BLE_ICON_POS_X, cfg::STATUS_ICON_POS_Y);
        let clear_style = PrimitiveStyleBuilder::new()
            .fill_color(cfg::BACKGROUND_COLOR)
            .build();
        if !self.skip_clear {
            Rectangle::new(pos, Size::new(cfg::STATUS_ICON_SIZE, cfg::STATUS_ICON_SIZE))
                .into_styled(clear_style)
                .draw(display)
                .ok();
        }
        let color = if connected {
            cfg::ACCENT_COLOR
        } else {
            Rgb565::new(5, 10, 5)
        };
        draw_icon::<D, Bluetooth>(display, pos, color);
    }

    pub fn draw_update<D: DrawTarget<Color = Rgb565>>(
        &self,
        display: &mut D,
        status: UpdateStatus,
    ) {
        if self.navigation.page() != Page::Clock {
            return;
        }
        let pos = Point::new(cfg::UPDATE_POS_X, cfg::UPDATE_POS_Y);
        let clear_style = PrimitiveStyleBuilder::new()
            .fill_color(cfg::BACKGROUND_COLOR)
            .build();
        if !self.skip_clear {
            cfg::text_bounds(&FONT_10X20, cfg::UPDATE_CHARS, pos)
                .into_styled(clear_style)
                .draw(display)
                .ok();
        }
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
        self.last_battery = None;
        // The caller already cleared the entire panel while the backlight was
        // off. Avoid filling each field a second time during wake.
        self.skip_clear = true;
        if self.navigation.page() != Page::Clock {
            self.draw_page(display);
            self.draw_connection(display, connected);
            if let Some(battery) = battery {
                self.draw_battery(display, battery);
            }
            self.skip_clear = false;
            return;
        }
        self.draw_static(display);
        self.draw_connection(display, connected);
        if let Some(time) = time {
            self.draw_time(display, time);
        }
        if let Some(battery) = battery {
            self.draw_battery(display, battery);
        }
        self.draw_update(display, status);
        self.skip_clear = false;
    }

    fn draw_page<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        let (title, hint) = match self.navigation.page() {
            Page::Clock => return,
            Page::Launcher => ("LAUNCHER", "Swipe down: clock"),
            Page::Notifications => ("NOTIFICATIONS", "Swipe up: clock"),
            Page::Settings => ("SETTINGS", "Swipe down: back"),
            Page::About => ("ABOUT", "Swipe down: back"),
        };
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(cfg::TEXT_COLOR)
            .background_color(cfg::BACKGROUND_COLOR)
            .build();
        let accent = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(cfg::ACCENT_COLOR)
            .background_color(cfg::BACKGROUND_COLOR)
            .build();
        let x = ((cfg::DISPLAY_WIDTH as i32 - title.len() as i32 * 10) / 2).max(0);
        let _ = Text::new(title, Point::new(x, 42), accent).draw(display);
        let _ = Text::new(hint, Point::new(10, 220), style).draw(display);
        match self.navigation.page() {
            Page::Launcher => {
                let _ = Text::new("SETTINGS", Point::new(20, 110), style).draw(display);
                let _ = Text::new("ABOUT", Point::new(20, 165), style).draw(display);
            }
            Page::Notifications => {
                let _ = Text::new("No notifications", Point::new(20, 110), style).draw(display);
            }
            Page::Settings => {
                let battery = if self.show_battery_percent {
                    "Battery %: ON"
                } else {
                    "Battery %: OFF"
                };
                let row = Rectangle::new(Point::new(10, 73), Size::new(220, 52))
                    .into_styled(PrimitiveStyle::with_stroke(cfg::ACCENT_COLOR, 1));
                let _ = row.draw(display);
                let _ = Text::new(battery, Point::new(20, 105), style).draw(display);
                let wake = if cfg!(feature = "touch-wake-debug") {
                    "Touch wake: debug"
                } else {
                    "Touch wake: off"
                };
                let _ = Text::new(wake, Point::new(20, 165), style).draw(display);
            }
            Page::About => {
                let _ = Text::new("Kongle", Point::new(20, 110), style).draw(display);
                let _ = Text::new(FIRMWARE_VERSION, Point::new(20, 145), style).draw(display);
            }
            Page::Clock => {}
        }
    }
}
