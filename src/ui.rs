//! Screen layer: owns drawing state so `main` only routes events to it.
//!
//! Only the clock screen exists for now; more screens and navigation hook in
//! through [`Screen`] and [`Ui`].

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use heapless::String;

use crate::device::battery::BatteryState;
use crate::device::display as display_cfg;
use crate::fonts::JETBRAINS_FONT_54_POINT_EXTRA_BOLD;
use crate::ota::UpdateStatus;
use crate::{TimeState, FIRMWARE_VERSION};

/// The screens the watch can show.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Screen {
    Clock,
}

pub struct Ui {
    screen: Screen,
    time_style: MonoTextStyle<'static, Rgb565>,
    small_style: MonoTextStyle<'static, Rgb565>,
    clear_style: PrimitiveStyle<Rgb565>,
    time_pos: Point,
    seconds_pos: Point,
    battery_pos: Point,
    update_pos: Point,
    ble_pos: Point,
    version_pos: Point,
    time_bounds: Rectangle,
    seconds_bounds: Rectangle,
    battery_bounds: Rectangle,
    update_bounds: Rectangle,
    ble_bounds: Rectangle,
    last_time: Option<TimeState>,
    last_seconds: Option<u8>,
    last_battery: Option<BatteryState>,
    ota: UpdateStatus,
    ble_connected: bool,
}

fn time_text(time: &TimeState) -> String<8> {
    let mut text = String::new();
    let _ = write!(&mut text, "{:02}:{:02}", time.hours, time.minutes);
    text
}

fn seconds_text(time: &TimeState) -> String<2> {
    let mut text = String::new();
    let _ = write!(&mut text, "{:02}", time.seconds);
    text
}

fn battery_text(state: &BatteryState) -> String<16> {
    let mut text = String::new();
    let charging = if state.charging { "+" } else { "" };
    let _ = write!(
        &mut text,
        "{}.{:02}V{} {}%",
        state.mv / 1000,
        (state.mv % 1000) / 10,
        charging,
        state.percent
    );
    text
}

impl Ui {
    pub fn new() -> Self {
        let time_font = &JETBRAINS_FONT_54_POINT_EXTRA_BOLD;
        let small_font = &FONT_10X20;
        let style = |font| {
            MonoTextStyleBuilder::new()
                .font(font)
                .text_color(display_cfg::TEXT_COLOR)
                .background_color(display_cfg::BACKGROUND_COLOR)
                .build()
        };
        let time_pos = Point::new(display_cfg::TIME_POS_X, display_cfg::TIME_POS_Y);
        let seconds_pos = Point::new(display_cfg::SECONDS_POS_X, display_cfg::SECONDS_POS_Y);
        let battery_pos = Point::new(display_cfg::BATTERY_POS_X, display_cfg::BATTERY_POS_Y);
        let update_pos = Point::new(display_cfg::UPDATE_POS_X, display_cfg::UPDATE_POS_Y);
        let ble_pos = Point::new(display_cfg::BLE_POS_X, display_cfg::BLE_POS_Y);
        Self {
            screen: Screen::Clock,
            time_style: style(time_font),
            small_style: style(small_font),
            clear_style: PrimitiveStyleBuilder::new()
                .fill_color(display_cfg::BACKGROUND_COLOR)
                .build(),
            time_pos,
            seconds_pos,
            battery_pos,
            update_pos,
            ble_pos,
            version_pos: Point::new(display_cfg::VERSION_POS_X, display_cfg::VERSION_POS_Y),
            time_bounds: display_cfg::text_bounds(time_font, display_cfg::TIME_CHARS, time_pos),
            seconds_bounds: display_cfg::text_bounds(
                small_font,
                display_cfg::SECONDS_CHARS,
                seconds_pos,
            ),
            battery_bounds: display_cfg::text_bounds(
                small_font,
                display_cfg::BATTERY_CHARS,
                battery_pos,
            ),
            update_bounds: display_cfg::text_bounds(
                small_font,
                display_cfg::UPDATE_CHARS,
                update_pos,
            ),
            ble_bounds: display_cfg::text_bounds(small_font, 1, ble_pos),
            last_time: None,
            last_seconds: None,
            last_battery: None,
            ota: UpdateStatus::Idle,
            ble_connected: false,
        }
    }

    #[allow(dead_code)]
    pub fn screen(&self) -> Screen {
        self.screen
    }

    pub fn ota(&self) -> UpdateStatus {
        self.ota
    }

    pub fn set_ota(&mut self, status: UpdateStatus) {
        self.ota = status;
    }

    pub fn set_ble_connected(&mut self, connected: bool) {
        self.ble_connected = connected;
    }

    fn clear<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D, bounds: Rectangle) {
        bounds.into_styled(self.clear_style).draw(display).ok();
    }

    /// Version string only; shown while the rest of the firmware starts up.
    pub fn draw_version<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        let _ = Text::new(FIRMWARE_VERSION, self.version_pos, self.small_style).draw(display);
    }

    /// Redraws the current screen onto an already cleared display.
    /// `time` and `battery` are the latest known values, if any.
    pub fn draw_full<D: DrawTarget<Color = Rgb565>>(
        &mut self,
        display: &mut D,
        time: Option<TimeState>,
        battery: Option<BatteryState>,
    ) {
        match self.screen {
            Screen::Clock => self.draw_clock(display, time, battery),
        }
    }

    fn draw_clock<D: DrawTarget<Color = Rgb565>>(
        &mut self,
        display: &mut D,
        time: Option<TimeState>,
        battery: Option<BatteryState>,
    ) {
        self.draw_version(display);
        self.last_time = None;
        self.last_seconds = None;
        self.last_battery = None;
        if self.ble_connected {
            let _ = Text::new("B", self.ble_pos, self.small_style).draw(display);
        }
        if let Some(time) = time {
            let _ = Text::new(&time_text(&time), self.time_pos, self.time_style).draw(display);
            let _ =
                Text::new(&seconds_text(&time), self.seconds_pos, self.small_style).draw(display);
            self.last_time = Some(time);
            self.last_seconds = Some(time.seconds);
        }
        if let Some(state) = battery {
            let _ =
                Text::new(&battery_text(&state), self.battery_pos, self.small_style).draw(display);
            self.last_battery = Some(state);
        }
        self.draw_update_text(display);
    }

    fn draw_update_text<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        let text = crate::update_text(self.ota);
        let _ = Text::new(&text, self.update_pos, self.small_style).draw(display);
    }

    /// Incremental update after the OTA status changed (see [`Ui::set_ota`]).
    pub fn update_ota<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        self.clear(display, self.update_bounds);
        self.draw_update_text(display);
    }

    pub fn update_time<D: DrawTarget<Color = Rgb565>>(&mut self, display: &mut D, time: TimeState) {
        if self
            .last_time
            .map(|t| t.hours != time.hours || t.minutes != time.minutes)
            .unwrap_or(true)
        {
            self.clear(display, self.time_bounds);
            let _ = Text::new(&time_text(&time), self.time_pos, self.time_style).draw(display);
        }
        if self.last_seconds != Some(time.seconds) {
            self.clear(display, self.seconds_bounds);
            let _ =
                Text::new(&seconds_text(&time), self.seconds_pos, self.small_style).draw(display);
            self.last_seconds = Some(time.seconds);
        }
        self.last_time = Some(time);
    }

    pub fn update_battery<D: DrawTarget<Color = Rgb565>>(
        &mut self,
        display: &mut D,
        state: BatteryState,
    ) {
        if self.last_battery != Some(state) {
            self.clear(display, self.battery_bounds);
            let _ =
                Text::new(&battery_text(&state), self.battery_pos, self.small_style).draw(display);
            self.last_battery = Some(state);
        }
    }

    /// Incremental update after [`Ui::set_ble_connected`].
    pub fn update_ble<D: DrawTarget<Color = Rgb565>>(&self, display: &mut D) {
        self.clear(display, self.ble_bounds);
        if self.ble_connected {
            let _ = Text::new("B", self.ble_pos, self.small_style).draw(display);
        }
    }
}
