//! Clock-screen rendering. The application task owns the display and forwards
//! state changes here; BLE, input, and OTA tasks never draw directly.

#[path = "ui/language.rs"]
mod language;
#[path = "ui/navigation.rs"]
mod navigation;
#[path = "ui/text.rs"]
mod text;

use crate::{theme, ui_layout as layout};
use language::Language;

use core::fmt::Write;

use eg_seven_segment::{SevenSegmentStyle, SevenSegmentStyleBuilder};
use embedded_graphics::{
    image::Image,
    mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle},
    text::{Baseline, Text},
};
use embedded_iconoir::{
    icons::size24px::{
        activities::Walking,
        communication::Bell,
        connectivity::Bluetooth,
        music::{Pause, Play, SkipNext, SkipPrev},
        system,
    },
    prelude::IconoirIcon,
    Icon,
};
use heapless::String;

use crate::{
    device::{battery::BatteryState, display as cfg},
    firmware_state::{BootState, Confirmation},
    notifications::{Inbox, Notification},
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
        theme::ERROR
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

fn draw_text_lines<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    text: &str,
    y: i32,
    max_lines: usize,
    style: embedded_graphics::mono_font::MonoTextStyle<'_, Rgb565>,
) {
    let mut remaining = text;
    for line in 0..max_lines {
        let (part, rest) = self::text::next_line(remaining, 20);
        if part.is_empty() {
            break;
        }
        let _ = Text::new(part, Point::new(20, y + line as i32 * 23), style).draw(display);
        remaining = rest;
    }
}

fn clock_digits() -> SevenSegmentStyle<Rgb565> {
    SevenSegmentStyleBuilder::new()
        .digit_size(Size::new(42, 70))
        .digit_spacing(6)
        .segment_width(7)
        .segment_color(theme::CLOCK)
        .build()
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

fn panel<D: DrawTarget<Color = Rgb565>>(display: &mut D, title: &str, top: i32, height: u32) {
    let _ = RoundedRectangle::with_equal_corners(
        Rectangle::new(Point::new(8, top), Size::new(224, height)),
        Size::new(7, 7),
    )
    .into_styled(PrimitiveStyle::with_stroke(theme::BORDER, 2))
    .draw(display);
    let width = (title.chars().count() as u32 * 10 + 8).min(202);
    let _ = Rectangle::new(Point::new(18, top - 10), Size::new(width, 20))
        .into_styled(PrimitiveStyle::with_fill(theme::BACKGROUND))
        .draw(display);
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(theme::HEADING)
        .background_color(theme::BACKGROUND)
        .build();
    let _ =
        Text::with_baseline(title, Point::new(22, top - 10), style, Baseline::Top).draw(display);
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Brightness {
    Low,
    Medium,
    High,
}

impl Brightness {
    const fn backlight_level(self) -> u8 {
        match self {
            Self::Low => 1,
            Self::Medium => 3,
            Self::High => 7,
        }
    }

    const fn from_touch_x(x: u8) -> Self {
        match x {
            0..=79 => Self::Low,
            80..=159 => Self::Medium,
            _ => Self::High,
        }
    }
}

pub struct WatchUi {
    navigation: Navigation,
    language: Language,
    notifications: Inbox,
    has_unread: bool,
    show_battery_percent: bool,
    vibration_enabled: bool,
    changed_settings: u8,
    firmware: Confirmation,
    confirmation_failed: bool,
    brightness: Brightness,
    last_time: Option<TimeState>,
    last_battery: Option<BatteryState>,
    skip_clear: bool,
    ble_address: Option<[u8; 6]>,
    media: crate::media::State,
    media_request: Option<crate::media::Request>,
}

impl WatchUi {
    pub const fn new() -> Self {
        Self {
            navigation: Navigation::new(),
            language: Language::English,
            notifications: Inbox::new(),
            has_unread: false,
            show_battery_percent: true,
            vibration_enabled: true,
            changed_settings: 0,
            firmware: Confirmation::new(),
            confirmation_failed: false,
            brightness: Brightness::High,
            last_time: None,
            last_battery: None,
            skip_clear: false,
            ble_address: None,
            media: crate::media::State::new(),
            media_request: None,
        }
    }

    fn tr(&self, english: &'static str, bokmal: &'static str) -> &'static str {
        self.language.text(english, bokmal)
    }

    pub fn set_ble_address(&mut self, address: [u8; 6]) {
        self.ble_address = Some(address);
    }

    pub fn is_clock(&self) -> bool {
        self.navigation.page() == Page::Clock
    }

    pub fn open_media(&mut self) {
        if self.navigation.page() == Page::Notifications {
            self.has_unread = false;
        }
        self.firmware.cancel();
        self.navigation.media();
        self.request_media(crate::media::Command::Open);
    }

    fn request_media(&mut self, command: crate::media::Command) {
        if self.media.session != 0 {
            self.media_request = Some(crate::media::Request {
                session: self.media.session,
                command,
            });
        }
    }

    pub fn take_media_request(&mut self) -> Option<crate::media::Request> {
        self.media_request.take()
    }

    pub fn set_media(&mut self, state: crate::media::State) -> bool {
        let changed = self.media != state;
        self.media = state;
        changed
    }

    pub fn handle_touch(&mut self, event: TouchEvent) -> bool {
        if self.navigation.page() == Page::Media && self.media.available() {
            if let TouchEvent::Tap { x, y: 140..=203 } = event {
                let command = match x {
                    16..=79 => Some(crate::media::Command::Previous),
                    88..=151 => Some(if self.media.playing {
                        crate::media::Command::Pause
                    } else {
                        crate::media::Command::Play
                    }),
                    160..=223 => Some(crate::media::Command::Next),
                    _ => None,
                };
                if let Some(command) = command {
                    self.request_media(command);
                }
                return false;
            }
        }

        let mut cancelled_confirmation = false;
        if self.navigation.page() == Page::Firmware {
            if let TouchEvent::Tap { y, .. } = event {
                return self.firmware.tap(y);
            }
            cancelled_confirmation = self.firmware.cancel();
        }
        if self.navigation.page() == Page::Notifications {
            if self.has_unread {
                self.has_unread = false;
                self.changed_settings |= 8;
            }
            match event {
                TouchEvent::SwipeLeft => return self.notifications.older(),
                TouchEvent::SwipeRight if self.notifications.newer() => return true,
                _ => {}
            }
        }
        if self.navigation.page() == Page::Settings {
            if layout::tapped(event, layout::VIBRATION) {
                self.vibration_enabled = !self.vibration_enabled;
                self.changed_settings |= 4;
                return false;
            }
            match event {
                TouchEvent::Tap { x, .. } if layout::tapped(event, layout::BRIGHTNESS) => {
                    let next = Brightness::from_touch_x(x);
                    let changed = self.brightness != next;
                    self.brightness = next;
                    if changed {
                        self.changed_settings |= 2;
                    }
                    return false;
                }
                _ => {}
            }
        }
        if self.navigation.page() == Page::Preferences
            && layout::tapped(event, layout::BATTERY_PERCENT)
        {
            self.show_battery_percent = !self.show_battery_percent;
            self.changed_settings |= 1;
            return false;
        }
        if self.navigation.page() == Page::Preferences && layout::tapped(event, layout::LANGUAGE) {
            self.language.toggle();
            return true;
        }
        let changed = self.navigation.handle(event);
        if changed && self.navigation.page() == Page::Media {
            self.request_media(crate::media::Command::Open);
        }
        if changed && self.navigation.page() == Page::Notifications {
            self.has_unread = false;
        }
        changed || cancelled_confirmation
    }

    pub fn receive_notification(&mut self, notification: Notification, show: bool) {
        self.notifications.push(notification);
        self.has_unread = true;
        if show {
            self.firmware.cancel();
            self.navigation.notifications();
        }
    }

    pub const fn brightness_level(&self) -> u8 {
        self.brightness.backlight_level()
    }

    pub const fn vibration_enabled(&self) -> bool {
        self.vibration_enabled
    }

    pub fn handle_button(&mut self, awake: bool) -> bool {
        if self.navigation.page() == Page::Notifications {
            self.has_unread = false;
        }
        self.firmware.cancel();
        self.navigation.button(awake)
    }

    pub fn set_firmware_state(&mut self, state: BootState, failed: bool) {
        self.firmware.set_state(state);
        self.confirmation_failed = failed;
    }

    pub fn take_confirmation_request(&mut self) -> bool {
        self.firmware.take_request()
    }

    pub fn draw_changed_settings<D: DrawTarget<Color = Rgb565>>(&mut self, display: &mut D) {
        let changed = core::mem::take(&mut self.changed_settings);
        if changed & 8 != 0 {
            self.draw_notification_indicator(display);
        }
        if self.navigation.page() == Page::Preferences && changed & 1 != 0 {
            self.draw_battery_setting(display);
        }
        if self.navigation.page() != Page::Settings {
            return;
        }
        if changed & 2 != 0 {
            self.draw_brightness_selection(display);
        }
        if changed & 4 != 0 {
            self.draw_vibration_setting(display);
        }
    }

    fn draw_setting<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D, text: &str, y: i32) {
        let _ = Rectangle::new(Point::new(16, y - 20), Size::new(208, 27))
            .into_styled(PrimitiveStyle::with_fill(cfg::BACKGROUND_COLOR))
            .draw(display);
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(cfg::TEXT_COLOR)
            .background_color(cfg::BACKGROUND_COLOR)
            .build();
        let _ = Text::new(text, Point::new(20, y), style).draw(display);
    }

    fn draw_battery_setting<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        self.draw_setting(
            display,
            if self.show_battery_percent {
                self.tr("Battery %: ON", "Batteri %: PÅ")
            } else {
                self.tr("Battery %: OFF", "Batteri %: AV")
            },
            60,
        );
    }

    fn draw_vibration_setting<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        self.draw_setting(
            display,
            if self.vibration_enabled {
                self.tr("Vibration: ON", "Vibrasjon: PÅ")
            } else {
                self.tr("Vibration: OFF", "Vibrasjon: AV")
            },
            118,
        );
    }

    /// Refresh only the selected brightness labels while the backlight stays on.
    pub fn draw_brightness_selection<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        if self.navigation.page() != Page::Settings {
            return;
        }
        if !self.skip_clear {
            let clear = PrimitiveStyle::with_fill(cfg::BACKGROUND_COLOR);
            let _ = layout::BRIGHTNESS_SELECTION
                .into_styled(clear)
                .draw(display);
        }
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
        for (label, x, preset) in [
            ("LOW", 20, Brightness::Low),
            ("MED", 91, Brightness::Medium),
            ("HIGH", 161, Brightness::High),
        ] {
            let selected_style = if self.brightness == preset {
                accent
            } else {
                style
            };
            let _ = Text::new(label, Point::new(x, 70), selected_style).draw(display);
        }
    }

    /// OTA progress must remain visible even if another page was open.
    pub fn show_clock(&mut self) -> bool {
        self.firmware.cancel();
        self.navigation.clock()
    }

    pub fn draw_static<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        panel(display, "", 44, 108);
        self.draw_statuses(display);
    }

    fn draw_statuses<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        // The first local field awaits the step source. Remote fields are
        // omitted until an integration explicitly enables them.
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(theme::MUTED)
            .background_color(theme::BACKGROUND)
            .build();
        let _ = Text::with_baseline("--", Point::new(44, 175), style, Baseline::Top).draw(display);
        draw_icon::<D, Walking>(display, Point::new(14, 171), theme::STEPS);
    }

    pub fn draw_notification_indicator<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        if self.navigation.page() != Page::Clock {
            return;
        }
        let area = Rectangle::new(Point::new(110, 0), Size::new(74, 28));
        let _ = area
            .into_styled(PrimitiveStyle::with_fill(theme::BACKGROUND))
            .draw(display);
        if self.has_unread {
            draw_icon::<D, Bell>(display, Point::new(114, 0), theme::MUTED);
        }
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
            let title = self
                .language
                .date(time.year as i32, time.month as u32, time.day as u32);
            let _ = Rectangle::new(Point::new(9, 33), Size::new(222, 22))
                .into_styled(PrimitiveStyle::with_fill(theme::BACKGROUND))
                .draw(display);
            panel(display, &title, 44, 108);
        }
        if self
            .last_time
            .map(|t| t.hours != time.hours || t.minutes != time.minutes)
            .unwrap_or(true)
        {
            let pos = Point::new(cfg::TIME_POS_X, cfg::TIME_POS_Y);
            if !self.skip_clear {
                Text::with_baseline("88:88", pos, clock_digits(), Baseline::Top)
                    .bounding_box()
                    .into_styled(clear_style)
                    .draw(display)
                    .ok();
            }
            let mut text: String<8> = String::new();
            let _ = write!(&mut text, "{:02}:{:02}", time.hours, time.minutes);
            let _ = Text::with_baseline(&text, pos, clock_digits(), Baseline::Top).draw(display);
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
        if self.navigation.page() != Page::Clock {
            return;
        }
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
        if connected {
            draw_icon::<D, Bluetooth>(display, pos, theme::BLUETOOTH);
        }
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
        // The status replaces the lower sensor row during OTA/trial, rather
        // than leaving half of its icons visible behind the message.
        Rectangle::new(Point::new(0, 201), Size::new(240, 39))
            .into_styled(clear_style)
            .draw(display)
            .ok();
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(cfg::TEXT_COLOR)
            .background_color(cfg::BACKGROUND_COLOR)
            .build();
        if status == UpdateStatus::Idle {
            self.draw_statuses(display);
            return;
        }
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
        self.draw_notification_indicator(display);
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

    pub fn draw_media<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        if self.navigation.page() != Page::Media {
            return;
        }
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(theme::TEXT)
            .build();
        let muted = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(theme::MUTED)
            .build();
        let _ = Rectangle::new(Point::new(16, 30), Size::new(208, 94))
            .into_styled(PrimitiveStyle::with_fill(theme::BACKGROUND))
            .draw(display);
        if self.media.available() {
            draw_text_lines(
                display,
                if self.media.title.is_empty() {
                    self.tr("Media", "Avspilling")
                } else {
                    &self.media.title
                },
                50,
                2,
                style,
            );
            draw_text_lines(display, &self.media.artist, 110, 1, muted);
        } else {
            let _ = Text::new(
                self.tr("No player", "Ingen avspiller"),
                Point::new(20, 58),
                style,
            )
            .draw(display);
        }
        let color = if self.media.available() {
            theme::ACCENT
        } else {
            theme::DISABLED
        };
        for x in [16, 88, 160] {
            let _ = RoundedRectangle::with_equal_corners(
                Rectangle::new(Point::new(x, 140), Size::new(64, 64)),
                Size::new(6, 6),
            )
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .fill_color(theme::BACKGROUND)
                    .stroke_color(color)
                    .stroke_width(2)
                    .build(),
            )
            .draw(display);
        }
        draw_icon::<D, SkipPrev>(display, Point::new(36, 160), color);
        if self.media.playing {
            draw_icon::<D, Pause>(display, Point::new(108, 160), color);
        } else {
            draw_icon::<D, Play>(display, Point::new(108, 160), color);
        }
        draw_icon::<D, SkipNext>(display, Point::new(180, 160), color);
    }

    fn draw_page<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        let title = match self.navigation.page() {
            Page::Clock => return,
            Page::System => self.tr("system", "system"),
            Page::Notifications => self.tr("notifications", "varsler"),
            Page::Settings => self.tr("settings", "innstillinger"),
            Page::About => self.tr("about", "om klokka"),
            Page::Firmware => "firmware",
            Page::Home => self.tr("home", "hjemme"),
            Page::Activity => self.tr("activity", "aktivitet"),
            Page::Media => self.tr("media", "medier"),
            Page::Preferences => self.tr("display", "visning"),
        };
        // Only the clock reserves a status row. Other page titles sit at the
        // top edge, with their border running down the full available height.
        panel(display, title, 10, 226);
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
        match self.navigation.page() {
            Page::System => {
                let _ = Text::new(self.tr("About", "Om klokka"), Point::new(20, 60), style)
                    .draw(display);
                let _ = Text::new("Firmware", Point::new(20, 112), style).draw(display);
            }
            Page::Notifications => {
                if let Some(notification) = self.notifications.selected() {
                    draw_text_lines(display, &notification.title, 50, 2, accent);
                    draw_text_lines(display, &notification.body, 104, 4, style);
                    let mut position: String<16> = String::new();
                    let _ = write!(
                        &mut position,
                        "{} / {}  < >",
                        self.notifications.position(),
                        self.notifications.count()
                    );
                    let _ = Text::new(&position, Point::new(20, 218), style).draw(display);
                } else {
                    let _ = Text::new(
                        self.tr("No notifications", "Ingen varsler"),
                        Point::new(20, 58),
                        style,
                    )
                    .draw(display);
                }
            }
            Page::Settings => {
                let _ = Text::new(
                    self.tr("Brightness", "Lysstyrke"),
                    Point::new(20, 42),
                    style,
                )
                .draw(display);
                self.draw_brightness_selection(display);
                self.draw_vibration_setting(display);
                let _ = Text::new(
                    self.tr("Display >", "Visning >"),
                    Point::new(20, 166),
                    style,
                )
                .draw(display);
                let _ = Text::new("System >", Point::new(20, 214), style).draw(display);
            }
            Page::Preferences => {
                self.draw_battery_setting(display);
                let _ = Text::new(self.tr("Language", "Språk"), Point::new(20, 110), style)
                    .draw(display);
                let _ = Text::new(
                    self.tr("English", "Norsk bokmål"),
                    Point::new(20, 144),
                    accent,
                )
                .draw(display);
            }
            Page::Activity => {
                let _ = Text::new("--", Point::new(20, 60), accent).draw(display);
                let _ = Text::new(
                    self.tr("Steps unavailable", "Skritt utilgjengelig"),
                    Point::new(20, 100),
                    style,
                )
                .draw(display);
            }
            Page::Home => {
                for (label, x, y) in [
                    (self.tr("Living", "Stue"), 16, 34),
                    (self.tr("Kitchen", "Kjøkken"), 124, 34),
                    (self.tr("Reading", "Leselys"), 16, 126),
                    (self.tr("Evening", "Kveld"), 124, 126),
                ] {
                    let _ = RoundedRectangle::with_equal_corners(
                        Rectangle::new(Point::new(x, y), Size::new(100, 80)),
                        Size::new(6, 6),
                    )
                    .into_styled(PrimitiveStyle::with_stroke(theme::DISABLED, 2))
                    .draw(display);
                    let _ = Text::new(
                        label,
                        Point::new(x + 8, y + 46),
                        MonoTextStyleBuilder::new()
                            .font(&FONT_10X20)
                            .text_color(theme::MUTED)
                            .build(),
                    )
                    .draw(display);
                }
                let _ = Text::new(
                    self.tr("Not configured", "Ikke konfigurert"),
                    Point::new(20, 230),
                    style,
                )
                .draw(display);
            }
            Page::Media => self.draw_media(display),
            Page::About => {
                let _ = Text::new("Kongle", Point::new(20, 58), style).draw(display);
                let _ = Text::new(FIRMWARE_VERSION, Point::new(20, 100), style).draw(display);
                let _ = Text::new("BLE MAC", Point::new(20, 196), accent).draw(display);
                if let Some(address) = self.ble_address {
                    let address = crate::ble_identity::display_address(address);
                    let _ = Text::new(&address, Point::new(20, 222), style).draw(display);
                } else {
                    let _ = Text::new(
                        self.tr("Unavailable", "Utilgjengelig"),
                        Point::new(20, 222),
                        style,
                    )
                    .draw(display);
                }
            }
            Page::Firmware => {
                if self.firmware.armed {
                    let _ =
                        Text::new("Reset will no longer", Point::new(20, 44), style).draw(display);
                    let _ =
                        Text::new("restore old firmware", Point::new(20, 67), style).draw(display);
                    let _ = Rectangle::new(
                        Point::new(10, crate::firmware_state::CONFIRM_TOP as i32),
                        Size::new(
                            220,
                            (crate::firmware_state::CONFIRM_BOTTOM
                                - crate::firmware_state::CONFIRM_TOP
                                + 1) as u32,
                        ),
                    )
                    .into_styled(PrimitiveStyle::with_stroke(cfg::ACCENT_COLOR, 2))
                    .draw(display);
                    let _ = Text::new("Confirm now", Point::new(20, 116), accent).draw(display);
                    let _ = Rectangle::new(
                        Point::new(10, crate::firmware_state::ARM_TOP as i32),
                        Size::new(
                            220,
                            (crate::firmware_state::ARM_BOTTOM - crate::firmware_state::ARM_TOP + 1)
                                as u32,
                        ),
                    )
                    .into_styled(PrimitiveStyle::with_stroke(cfg::ACCENT_COLOR, 2))
                    .draw(display);
                    let _ = Text::new("Keep trial", Point::new(20, 201), style).draw(display);
                } else {
                    let _ = Text::new(FIRMWARE_VERSION, Point::new(20, 52), style).draw(display);
                    let status = match self.firmware.state {
                        BootState::Standalone => "Standalone firmware",
                        BootState::NotTrial => "Not a trial image",
                        BootState::Trial => "Trial: reset reverts",
                        BootState::Confirmed => "Firmware confirmed",
                        BootState::Invalid => "Unknown boot state",
                    };
                    let _ = Text::new(status, Point::new(20, 86), style).draw(display);
                    if self.confirmation_failed {
                        let _ = Text::new("Confirmation failed", Point::new(20, 122), style)
                            .draw(display);
                    }
                    if self.firmware.state == BootState::Trial {
                        let _ = Rectangle::new(
                            Point::new(10, crate::firmware_state::ARM_TOP as i32),
                            Size::new(
                                220,
                                (crate::firmware_state::ARM_BOTTOM - crate::firmware_state::ARM_TOP
                                    + 1) as u32,
                            ),
                        )
                        .into_styled(PrimitiveStyle::with_stroke(cfg::ACCENT_COLOR, 2))
                        .draw(display);
                        let _ = Text::new("Confirm firmware", Point::new(20, 201), accent)
                            .draw(display);
                    }
                }
            }
            Page::Clock => {}
        }
    }
}
