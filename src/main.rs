#![cfg_attr(not(test), no_std)]
#![no_main]

mod ble;
mod current_time;
mod device;
mod fonts;

use core::fmt::Write;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::saadc;
use embassy_nrf::spim::{Config as SpimConfig, Frequency, Spim, MODE_3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch};
use embassy_time::{Duration, Ticker, Timer};
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

use chrono::{Datelike, NaiveDateTime, Timelike};
use device::battery::BatteryState;
use device::display as display_cfg;
use device::input::InputEvent;
use fonts::JETBRAINS_FONT_54_POINT_EXTRA_BOLD;

embassy_nrf::bind_interrupts!(struct Irqs {
    TWISPI0 => embassy_nrf::spim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
    SAADC => embassy_nrf::saadc::InterruptHandler;
});

/// Full date+time for display and BLE CTS (Current Time Service).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TimeState {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hours: u8,
    pub minutes: u8,
    pub seconds: u8,
}

impl TimeState {
    pub fn from_naive(dt: NaiveDateTime) -> Self {
        Self {
            year: dt.year().max(0).min(u16::MAX as i32) as u16,
            month: dt.month() as u8,
            day: dt.day() as u8,
            hours: dt.hour() as u8,
            minutes: dt.minute() as u8,
            seconds: dt.second() as u8,
        }
    }

    pub fn to_naive(&self) -> Option<NaiveDateTime> {
        chrono::NaiveDate::from_ymd_opt(self.year as i32, self.month as u32, self.day as u32)
            .and_then(|d| {
                chrono::NaiveTime::from_hms_opt(
                    self.hours as u32,
                    self.minutes as u32,
                    self.seconds as u32,
                )
                .map(|t| d.and_time(t))
            })
    }
}

static TIME_WATCH: Watch<CriticalSectionRawMutex, TimeState, 2> = Watch::new();
static BATTERY_WATCH: Watch<CriticalSectionRawMutex, BatteryState, 2> = Watch::new();
static BUTTON_CH: Channel<CriticalSectionRawMutex, InputEvent, 4> = Channel::new();
/// Channel for BLE to send new date/time; clock_task applies it.
pub(crate) static SET_TIME_CH: Channel<CriticalSectionRawMutex, NaiveDateTime, 1> = Channel::new();

#[defmt::panic_handler]
fn panic() -> ! {
    panic_probe::hard_fault()
}

#[embassy_executor::task]
async fn clock_task(
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, TimeState, 2>,
    set_time_rx: embassy_sync::channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        NaiveDateTime,
        1,
    >,
) {
    let mut dt = NaiveDateTime::new(
        chrono::NaiveDate::from_ymd_opt(2026, 1, 1).unwrap(),
        chrono::NaiveTime::from_hms_opt(11, 0, 0).unwrap(),
    );
    sender.send(TimeState::from_naive(dt));
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;
        if let Ok(new_dt) = set_time_rx.try_receive() {
            dt = new_dt;
        } else {
            dt += chrono::Duration::seconds(1);
        }
        sender.send(TimeState::from_naive(dt));
    }
}

#[embassy_executor::task]
async fn battery_task(
    mut battery: device::battery::Battery<'static>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, BatteryState, 2>,
) {
    battery.calibrate().await;
    loop {
        Timer::after(Duration::from_secs(10)).await;
        let state = battery.update().await;
        sender.send(state);
    }
}

#[embassy_executor::task]
async fn button_task(
    mut button: device::input::Button<'static>,
    sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, InputEvent, 4>,
) {
    loop {
        if let Some(event) = button.poll() {
            sender.send(event).await;
        }
        // Six samples at 10 ms intervals provide debounce without a busy loop.
        Timer::after(Duration::from_millis(10)).await;
    }
}

/// Feeds the MCUBoot bootloader's watchdog when it has already been started (~7 s timeout).
/// A missed feed resets the MCU; an unconfirmed trial image can then be reverted.
#[embassy_executor::task]
async fn wdt_task_1(mut h0: embassy_nrf::wdt::WatchdogHandle) {
    let mut ticker = Ticker::every(Duration::from_secs(3));
    loop {
        ticker.next().await;
        h0.pet();
    }
}

#[embassy_executor::task]
async fn run_controller(controller_task: apache_nimble::controller::NimbleControllerTask) {
    controller_task.run().await
}

#[embassy_executor::task]
async fn ble_peripheral_task(
    controller: apache_nimble::controller::NimbleController,
    address: [u8; 6],
    time_rx: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, TimeState, 2>,
    set_time_tx: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, NaiveDateTime, 1>,
) {
    ble::run(controller, address, time_rx, set_time_tx).await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    // When running under the MCUBoot bootloader, it starts the WDT before jumping here.
    // Adopt it before BLE initialization and its startup delay. This does not confirm a trial image.
    if let Some(wdt_config) = embassy_nrf::wdt::Config::try_new(&p.WDT) {
        defmt::info!(
            "WDT is running (bootloader started it); timeout_ticks={}",
            wdt_config.timeout_ticks
        );
        if let Ok((_, [h0])) = embassy_nrf::wdt::Watchdog::try_new::<_, 1>(p.WDT, wdt_config) {
            defmt::info!("WDT acquired with 1 handle; spawning wdt_task");
            spawner.spawn(wdt_task_1(h0)).unwrap();
        } else {
            defmt::warn!("WDT config/handle count mismatch; watchdog may timeout (bootloader may use >1 handle)");
        }
    } else {
        defmt::info!("WDT not running (standalone mode)");
    }

    apache_nimble::initialize_nimble();
    let controller = apache_nimble::controller::NimbleController::new();
    spawner
        .spawn(run_controller(controller.create_task()))
        .unwrap();
    // Wait for RNG to calm down before starting BLE peripheral
    Timer::after(Duration::from_secs(1)).await;

    // Active-high button input, with its supply driven by P0.15.
    let button = device::input::Button::new(
        Input::new(p.P0_13, Pull::Down),
        Output::new(p.P0_15, Level::High, OutputDrive::Standard),
    );

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

    let mut spim_config = SpimConfig::default();
    spim_config.mode = MODE_3;
    spim_config.frequency = Frequency::M8;
    let spim = Spim::new_txonly(
        p.TWISPI0,
        Irqs,
        p.P0_02, // SCK
        p.P0_03, // MOSI
        spim_config,
    );
    let spi_dev = ExclusiveDevice::new_no_delay(spim, cs).unwrap();
    let di = display_interface_spi::SPIInterface::new(spi_dev, dc);

    // Create a simple blocking delay instead of embassy_time::Delay
    struct BlockingDelay;
    impl embedded_hal::delay::DelayNs for BlockingDelay {
        fn delay_ns(&mut self, ns: u32) {
            // 64 MHz CPU, 64 cycles per microsecond, so ~64/1000 cycles per nanosecond
            // Be conservative and use 1 cycle per 10ns
            let cycles = ns / 10;
            cortex_m::asm::delay(cycles);
        }
    }
    let mut delay = BlockingDelay;

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

    spawner
        .spawn(clock_task(TIME_WATCH.sender(), SET_TIME_CH.receiver()))
        .unwrap();
    spawner
        .spawn(battery_task(battery, BATTERY_WATCH.sender()))
        .unwrap();
    spawner
        .spawn(button_task(button, BUTTON_CH.sender()))
        .unwrap();

    let time_rx_ble = TIME_WATCH.receiver().unwrap();
    // Factory-programmed device address, in Bluetooth little-endian byte order.
    let low = embassy_nrf::pac::FICR.deviceaddr(0).read().to_le_bytes();
    let high = embassy_nrf::pac::FICR.deviceaddr(1).read().to_le_bytes();
    let address = [low[0], low[1], low[2], low[3], high[0], high[1] | 0xc0];
    spawner
        .spawn(ble_peripheral_task(
            controller,
            address,
            time_rx_ble,
            SET_TIME_CH.sender(),
        ))
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
        if last_time
            .map(|t| t.hours != time.hours || t.minutes != time.minutes)
            .unwrap_or(true)
        {
            time_bounds.into_styled(clear_style).draw(&mut display).ok();
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
                    "{}.{:02}V{} {}%",
                    volts, frac, charging, battery_state.percent
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
