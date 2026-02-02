#![cfg_attr(not(test), no_std)]
#![no_main]

mod ble;
mod device;
mod fonts;

use core::fmt::Write;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::saadc;
use embassy_nrf::spim::{Config as SpimConfig, Spim};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch,
};
use embassy_time::{Delay, Duration, Ticker};
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    prelude::*,
    primitives::PrimitiveStyleBuilder,
    text::Text,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use heapless::String;
use mipidsi::{models::ST7789, Builder};
use panic_probe as _;

use device::battery::BatteryState;
use device::display as display_cfg;
use device::input::InputEvent;
use fonts::JETBRAINS_FONT_54_POINT_EXTRA_BOLD;

embassy_nrf::bind_interrupts!(struct Irqs {
    TWISPI0 => embassy_nrf::spim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
    SAADC => embassy_nrf::saadc::InterruptHandler;
});

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct TimeState {
    hours: u8,
    minutes: u8,
    seconds: u8,
}

impl TimeState {
    fn tick(&mut self) {
        self.seconds = self.seconds.wrapping_add(1);
        if self.seconds >= 60 {
            self.seconds = 0;
            self.minutes = self.minutes.wrapping_add(1);
        }
        if self.minutes >= 60 {
            self.minutes = 0;
            self.hours = self.hours.wrapping_add(1);
        }
        if self.hours >= 24 {
            self.hours = 0;
        }
    }
}

static TIME_WATCH: Watch<CriticalSectionRawMutex, TimeState, 2> = Watch::new();
static BATTERY_WATCH: Watch<CriticalSectionRawMutex, BatteryState, 2> = Watch::new();
static BUTTON_CH: Channel<CriticalSectionRawMutex, InputEvent, 4> = Channel::new();

#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault()
}

#[embassy_executor::task]
async fn clock_task(sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, TimeState, 2>) {
    let mut state = TimeState {
        hours: 12,
        minutes: 0,
        seconds: 0,
    };
    sender.send(state);
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;
        state.tick();
        sender.send(state);
    }
}

#[embassy_executor::task]
async fn battery_task(
    mut battery: device::battery::Battery<'static>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, BatteryState, 2>,
) {
    battery.calibrate().await;
    let mut ticker = Ticker::every(Duration::from_secs(10));
    loop {
        ticker.next().await;
        let state = battery.update().await;
        sender.send(state);
    }
}

#[embassy_executor::task]
async fn button_task(
    mut button: device::input::Button<'static>,
    sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, InputEvent, 4>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        if let Some(event) = button.poll().await {
            sender.send(event).await;
        }
        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize Embassy with proper interrupt priorities
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    // Button on P0.15 with pullup
    let button = device::input::Button::new(Input::new(p.P0_15, Pull::Up));

    // PineTime backlight pins (active-low): P0.14, P0.22, P0.23
    let backlight_low = Output::new(p.P0_14, Level::High, OutputDrive::Standard);
    let backlight_mid = Output::new(p.P0_22, Level::High, OutputDrive::Standard);
    let backlight_high = Output::new(p.P0_23, Level::High, OutputDrive::Standard);
    let mut backlight = device::Backlight::new(backlight_low, backlight_mid, backlight_high, 7);
    backlight.set(7);

    // PineTime display pins (adjust if your wiring differs)
    let dc = Output::new(p.P0_18, Level::High, OutputDrive::Standard);
    let cs = Output::new(p.P0_25, Level::High, OutputDrive::Standard);
    let rst = Output::new(p.P0_26, Level::High, OutputDrive::Standard);

    let spim = Spim::new_txonly(
        p.TWISPI0,
        Irqs,
        p.P0_02, // SCK
        p.P0_03, // MOSI
        SpimConfig::default(),
    );
    let spi_dev = ExclusiveDevice::new_no_delay(spim, cs).unwrap();
    let di = display_interface_spi::SPIInterface::new(spi_dev, dc);
    let mut delay = Delay;
    let mut display = Builder::new(ST7789, di)
        .display_size(display_cfg::DISPLAY_WIDTH, display_cfg::DISPLAY_HEIGHT)
        .display_offset(display_cfg::DISPLAY_OFFSET_X, display_cfg::DISPLAY_OFFSET_Y)
        .orientation(display_cfg::DISPLAY_ORIENTATION)
        .color_order(display_cfg::DISPLAY_COLOR_ORDER)
        .invert_colors(display_cfg::DISPLAY_COLOR_INVERSION)
        .reset_pin(rst)
        .init(&mut delay)
        .unwrap();

    display.clear(display_cfg::BACKGROUND_COLOR).unwrap();

    let time_font = &JETBRAINS_FONT_54_POINT_EXTRA_BOLD;
    let time_style = MonoTextStyleBuilder::new()
        .font(time_font)
        .text_color(display_cfg::TEXT_COLOR)
        .background_color(display_cfg::BACKGROUND_COLOR)
        .build();
    let seconds_font = &FONT_10X20;
    let seconds_style = MonoTextStyleBuilder::new()
        .font(seconds_font)
        .text_color(display_cfg::TEXT_COLOR)
        .background_color(display_cfg::BACKGROUND_COLOR)
        .build();
    let battery_style = MonoTextStyleBuilder::new()
        .font(seconds_font)
        .text_color(display_cfg::TEXT_COLOR)
        .background_color(display_cfg::BACKGROUND_COLOR)
        .build();

    let time_pos = Point::new(display_cfg::TIME_POS_X, display_cfg::TIME_POS_Y);
    let seconds_pos = Point::new(display_cfg::SECONDS_POS_X, display_cfg::SECONDS_POS_Y);
    let battery_pos = Point::new(display_cfg::BATTERY_POS_X, display_cfg::BATTERY_POS_Y);
    let time_bounds = display_cfg::text_bounds(time_font, display_cfg::TIME_CHARS, time_pos);
    let seconds_bounds =
        display_cfg::text_bounds(seconds_font, display_cfg::SECONDS_CHARS, seconds_pos);
    let battery_bounds =
        display_cfg::text_bounds(seconds_font, display_cfg::BATTERY_CHARS, battery_pos);
    let clear_style = PrimitiveStyleBuilder::new()
        .fill_color(display_cfg::BACKGROUND_COLOR)
        .build();

    let charge_pin = Input::new(p.P0_12, Pull::Up);
    let channel = saadc::ChannelConfig::single_ended(p.P0_31);
    let saadc = saadc::Saadc::new(p.SAADC, Irqs, saadc::Config::default(), [channel]);
    let battery = device::battery::Battery::new(saadc, charge_pin);

    spawner.spawn(clock_task(TIME_WATCH.sender())).unwrap();
    spawner
        .spawn(battery_task(battery, BATTERY_WATCH.sender()))
        .unwrap();
    spawner
        .spawn(button_task(button, BUTTON_CH.sender()))
        .unwrap();

    defmt::info!("Kongle started");

    let mut time_rx = TIME_WATCH.receiver().unwrap();
    let mut battery_rx = BATTERY_WATCH.receiver().unwrap();
    let button_rx = BUTTON_CH.receiver();

    let mut last_time: Option<TimeState> = None;
    let mut last_seconds: Option<u8> = None;
    let mut last_battery: Option<BatteryState> = None;
    let mut brightness: u8 = 7;
    loop {
        let time = time_rx.changed().await;
        if last_time.map(|t| t.hours != time.hours || t.minutes != time.minutes).unwrap_or(true) {
            time_bounds
                .into_styled(clear_style)
                .draw(&mut display)
                .ok();
            let mut time_text: String<8> = String::new();
            let _ = write!(&mut time_text, "{:02}:{:02}", time.hours, time.minutes);
            let _ = Text::new(&time_text, time_pos, time_style).draw(&mut display);
        }
        if last_seconds != Some(time.seconds) {
            seconds_bounds
                .into_styled(clear_style)
                .draw(&mut display)
                .ok();
            let mut seconds_text: String<2> = String::new();
            let _ = write!(&mut seconds_text, "{:02}", time.seconds);
            let _ = Text::new(&seconds_text, seconds_pos, seconds_style).draw(&mut display);
            last_seconds = Some(time.seconds);
        }
        last_time = Some(time);

        if let Some(battery_state) = battery_rx.try_changed() {
            if last_battery != Some(battery_state) {
                battery_bounds
                    .into_styled(clear_style)
                    .draw(&mut display)
                    .ok();
                let mut battery_text: String<16> = String::new();
                let volts = battery_state.mv / 1000;
                let frac = (battery_state.mv % 1000) / 10;
                let charging = if battery_state.charging { "+" } else { "" };
                let _ = write!(
                    &mut battery_text,
                    "{}.{}V{} {}%",
                    volts,
                    frac,
                    charging,
                    battery_state.percent
                );
                let _ = Text::new(&battery_text, battery_pos, battery_style).draw(&mut display);
                last_battery = Some(battery_state);
            }
        }

        while let Ok(event) = button_rx.try_receive() {
            if matches!(event, InputEvent::ButtonPressed) {
                brightness = if brightness >= 7 { 1 } else { brightness + 1 };
                backlight.set(brightness);
            }
        }

        defmt::info!("heartbeat");
    }
}
