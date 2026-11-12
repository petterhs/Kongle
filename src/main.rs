#![cfg_attr(not(test), no_std)]
#![no_main]

mod battery_estimate;
mod battery_history;
#[cfg(feature = "battery-persist-experimental")]
mod battery_persist;
#[cfg(feature = "battery-persist-experimental")]
mod battery_record;
mod ble;
mod ble_identity;
mod button_gestures;
mod current_time;
mod device;
mod dfu;
#[cfg(feature = "mcuboot")]
mod firmware_confirm;
mod firmware_state;
mod flash;
mod haptics;
mod heart_rate;
mod hrs3300;
mod media;
mod motion;
mod notifications;
mod ota;
mod steps;
#[path = "ui/theme.rs"]
mod theme;
mod touch;
mod ui;
#[path = "ui/layout.rs"]
mod ui_layout;

use core::cell::RefCell;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice as SharedSpiDevice;
use embassy_executor::Spawner;
use embassy_nrf::interrupt::Priority;
use embassy_nrf::saadc;
use embassy_nrf::spim::{Config as SpimConfig, Frequency, Spim, MODE_3};
use embassy_nrf::{
    gpio::{Input, Level, Output, OutputDrive, Pull},
    gpiote::{OutputChannel as GpioteOutput, OutputChannelPolarity},
    ppi::Ppi,
    timer::{Frequency as TimerFrequency, Timer as HardwareTimer},
};
use embassy_sync::blocking_mutex::{raw::NoopRawMutex, Mutex as BlockingMutex};
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_sync::signal::Signal;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_graphics::prelude::*;
use mipidsi::Builder;
use panic_probe as _;
use static_cell::StaticCell;

use chrono::{Datelike, NaiveDateTime, Timelike};
use device::battery::BatteryState;
use device::display as display_cfg;
use device::input::InputEvent;
use ota::UpdateStatus;

pub(crate) const FIRMWARE_VERSION: &str =
    concat!(env!("CARGO_PKG_VERSION"), "+", env!("KONGLE_BUILD_ID"));

embassy_nrf::bind_interrupts!(struct Irqs {
    TWISPI0 => embassy_nrf::spim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
    TWISPI1 => embassy_nrf::twim::InterruptHandler<embassy_nrf::peripherals::TWISPI1>;
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
static BATTERY_HISTORY: BlockingMutex<CriticalSectionRawMutex, RefCell<battery_history::History>> =
    BlockingMutex::new(RefCell::new(battery_history::History::new()));
static BLE_CONNECTED_WATCH: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();
static INPUT_CH: Channel<CriticalSectionRawMutex, InputEvent, 8> = Channel::new();
static NOTIFICATION_CH: Channel<CriticalSectionRawMutex, notifications::Notification, 2> =
    Channel::new();
static MEDIA_WATCH: Watch<CriticalSectionRawMutex, media::State, 2> = Watch::new();
static MEDIA_COMMANDS: Channel<CriticalSectionRawMutex, media::Request, 4> = Channel::new();
static SCREEN_AWAKE_WATCH: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();
static MOTION_WATCH: Watch<CriticalSectionRawMutex, steps::State, 1> = Watch::new();
static STEP_DATE: Watch<CriticalSectionRawMutex, Option<chrono::NaiveDate>, 1> = Watch::new();
static HEART_STATE: Watch<CriticalSectionRawMutex, heart_rate::State, 1> = Watch::new();
static HEART_DIAGNOSTICS: Watch<CriticalSectionRawMutex, heart_rate::Diagnostics, 1> = Watch::new();
static HEART_REQUEST: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static VIBRATIONS: Channel<CriticalSectionRawMutex, haptics::Pattern, 4> = Channel::new();
static SPI_BUS: StaticCell<BlockingMutex<NoopRawMutex, RefCell<Spim<'static>>>> = StaticCell::new();
static FLASH_CELL: StaticCell<AsyncMutex<CriticalSectionRawMutex, flash::Flash<FlashSpi>>> =
    StaticCell::new();
static SENSOR_TX_BUFFER: StaticCell<[u8; 32]> = StaticCell::new();
static SENSOR_BUS: StaticCell<device::i2c::Bus> = StaticCell::new();
type FlashSpi = SharedSpiDevice<'static, NoopRawMutex, Spim<'static>, Output<'static>>;
/// The future BLE receiver publishes progress here; the display owns the view.
pub(crate) static OTA_WATCH: Watch<CriticalSectionRawMutex, UpdateStatus, 2> = Watch::new();
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
    let mut synchronized = false;
    let mut step_date = None;
    STEP_DATE.sender().send(None);
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;
        if let Ok(new_dt) = set_time_rx.try_receive() {
            dt = new_dt;
            synchronized = true;
        } else {
            dt = current_time::next_cts_second(dt);
        }
        let date = synchronized.then_some(dt.date());
        if date != step_date {
            STEP_DATE.sender().send(date);
            step_date = date;
        }
        sender.send(TimeState::from_naive(dt));
    }
}

#[embassy_executor::task]
async fn battery_task(
    mut battery: device::battery::Battery<'static>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, BatteryState, 2>,
    #[cfg(feature = "battery-persist-experimental")] flash: &'static AsyncMutex<
        CriticalSectionRawMutex,
        flash::Flash<FlashSpi>,
    >,
) {
    battery.calibrate().await;
    loop {
        let state = battery.update().await;
        let sample = BATTERY_HISTORY.lock(|history| {
            history.borrow_mut().record(
                (Instant::now().as_millis() / 1000) as u32,
                state.mv,
                state.percent,
                state.charging,
            )
        });
        sender.send(state);
        #[cfg(feature = "battery-persist-experimental")]
        if let Some(sample) = sample {
            if let Err(error) = battery_persist::append(flash, sample).await {
                defmt::warn!("Battery LittleFS append failed: {}", error.code());
            }
        }
        #[cfg(not(feature = "battery-persist-experimental"))]
        let _ = sample;
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn button_task(
    mut button: device::input::Button<'static>,
    sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, InputEvent, 8>,
) {
    let mut pressed = button.is_pressed();
    let mut pressed_since = None;
    let mut held_sent = false;
    // A stuck or deliberately held button at startup must not reset the app
    // again before the bootloader's button menu can be used. Arm only after a
    // release, then require a fresh press for a long hold.
    let mut armed = !pressed;
    let mut clicks = button_gestures::Gestures::new();
    loop {
        if pressed {
            if armed && !held_sent {
                let deadline = pressed_since.unwrap() + Duration::from_secs(3);
                match embassy_futures::select::select(
                    button.wait_for_release(),
                    Timer::at(deadline),
                )
                .await
                {
                    embassy_futures::select::Either::First(()) => {}
                    embassy_futures::select::Either::Second(()) => {
                        held_sent = true;
                        clicks.cancel();
                        sender.send(InputEvent::ButtonHeld).await;
                        continue;
                    }
                }
            } else {
                button.wait_for_release().await;
            }
        } else {
            if let Some(deadline) = clicks.deadline() {
                match embassy_futures::select::select(
                    button.wait_for_press(),
                    Timer::at(Instant::from_millis(deadline)),
                )
                .await
                {
                    embassy_futures::select::Either::First(()) => {}
                    embassy_futures::select::Either::Second(()) => {
                        if clicks.expire(Instant::now().as_millis()) {
                            sender.send(InputEvent::ButtonSingleConfirmed).await;
                        }
                        continue;
                    }
                }
            } else {
                button.wait_for_press().await;
            }
        }

        // Sample only after an edge. Six stable readings reject button bounce
        // without a 10 ms timer running for the entire lifetime of the watch.
        let next_pressed = !pressed;
        let mut stable = true;
        for _ in 0..6 {
            Timer::after(Duration::from_millis(10)).await;
            if button.is_pressed() != next_pressed {
                stable = false;
                break;
            }
        }
        if !stable {
            continue;
        }
        pressed = next_pressed;
        if !armed {
            if !pressed {
                armed = true;
            }
            continue;
        }
        if pressed {
            pressed_since = Some(Instant::now());
            clicks.press(Instant::now().as_millis());
            held_sent = false;
            sender.send(InputEvent::ButtonPressed).await;
        } else {
            pressed_since = None;
            // A release following the recovery hold is not a tap.
            if held_sent {
                held_sent = false;
            } else {
                let event = match clicks.release(Instant::now().as_millis()) {
                    button_gestures::Click::First => InputEvent::ButtonReleased,
                    button_gestures::Click::Double => InputEvent::ButtonDoublePressed,
                };
                sender.send(event).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn touch_task(
    mut touch: device::input::TouchScreen<'static>,
    mut awake_rx: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, bool, 2>,
    sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, InputEvent, 8>,
) {
    let mut active = false;
    let mut awake = true;
    let mut suppress_until = Instant::now();
    let mut gesture_debouncer = touch::GestureDebouncer::new();
    loop {
        if !active {
            if !awake && !cfg!(feature = "touch-wake-debug") {
                awake = awake_rx.changed().await;
                continue;
            }
            match touch.init().await {
                Ok(chip_id) => {
                    defmt::info!("Touch CST816S initialized; chip ID {:02x}", chip_id);
                    active = true;
                }
                Err(error) => {
                    defmt::warn!("Touch initialization failed: {:?}", error);
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }
            }
        }
        match embassy_futures::select::select(touch.next(), awake_rx.changed()).await {
            embassy_futures::select::Either::First(result) => match result {
                Ok(Some(gesture)) if !awake && cfg!(feature = "touch-wake-debug") => {
                    defmt::info!("Touch wake: {:?}", gesture);
                    sender.send(InputEvent::TouchWake).await;
                    suppress_until = Instant::now() + Duration::from_millis(500);
                }
                Ok(Some(touch::TouchEvent::Contact)) | Ok(None) => {}
                Ok(Some(gesture))
                    if awake
                        && Instant::now() >= suppress_until
                        && gesture_debouncer.accept(gesture, Instant::now().as_millis()) =>
                {
                    defmt::info!("Touch gesture: {:?}", gesture);
                    sender.send(InputEvent::Touch(gesture)).await;
                }
                Ok(Some(_)) => {}
                Err(error) => {
                    defmt::warn!("Touch read failed: {:?}; resetting controller", error);
                    active = false;
                }
            },
            embassy_futures::select::Either::Second(new_awake) => {
                if awake == new_awake {
                    continue;
                }
                awake = new_awake;
                if cfg!(feature = "touch-wake-debug") {
                    continue;
                }
                if !awake {
                    if let Err(error) = touch.sleep().await {
                        defmt::warn!("Touch sleep failed: {:?}", error);
                    }
                    active = false;
                }
            }
        }
    }
}

/// Feeds the MCUBoot bootloader's watchdog when it has already been started (7 s timeout).
/// A missed feed resets the MCU; an unconfirmed trial image can then be reverted.
#[embassy_executor::task]
async fn wdt_task_1(mut h0: embassy_nrf::wdt::WatchdogHandle) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;
        h0.pet();
    }
}

#[embassy_executor::task]
async fn motion_task(
    bus: device::i2c::Device<'static>,
    mut awake_rx: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, bool, 2>,
) {
    let mut sensor = motion::Motion::new(bus, embassy_time::Delay);
    let sender = MOTION_WATCH.sender();
    sender.send(steps::State::EMPTY);
    defmt::info!("[motion] initializing step engine");
    match embassy_time::with_timeout(Duration::from_secs(8), sensor.initialize()).await {
        Ok(Ok(config)) => {
            let name = match config.variant {
                motion::Variant::Bma421 => "BMA421",
                motion::Variant::Bma425 => "BMA425",
            };
            defmt::info!(
                "[motion] {} ready; config ID {:04x}, accel {:02x}, range {:02x}",
                name,
                config.id,
                config.accel,
                config.range
            );
        }
        result => {
            defmt::warn!(
                "[motion] initialization failed: {:?}",
                defmt::Debug2Format(&result)
            );
            if !matches!(
                embassy_time::with_timeout(Duration::from_millis(500), sensor.stop()).await,
                Ok(Ok(()))
            ) {
                defmt::warn!("[motion] shutdown failed after initialization error");
            }
            return;
        }
    }
    let mut previous = None;
    let mut failures = 0;
    let mut counts = steps::Counts::new(Instant::now().as_millis());
    let mut date_rx = STEP_DATE.receiver().unwrap();
    loop {
        match embassy_time::with_timeout(Duration::from_millis(500), sensor.counter()).await {
            Ok(Ok(raw)) => {
                failures = 0;
                let date = date_rx.try_get().flatten();
                match counts.observe(raw, Instant::now().as_millis(), date) {
                    Ok(state) => sender.send(state),
                    Err(error) => {
                        sender.send(counts.unavailable());
                        defmt::warn!(
                            "[motion] count discontinuity: {:?}",
                            defmt::Debug2Format(&error)
                        );
                    }
                }
                if previous != Some(raw) {
                    defmt::info!("[motion] raw steps: {}", raw);
                    previous = Some(raw);
                }
            }
            result => {
                sender.send(counts.unavailable());
                failures += 1;
                defmt::warn!(
                    "[motion] counter read failed: {:?}",
                    defmt::Debug2Format(&result)
                );
                if failures >= 3 {
                    let _ =
                        embassy_time::with_timeout(Duration::from_millis(500), sensor.stop()).await;
                    defmt::warn!("[motion] stopped after repeated read errors; no automatic reset");
                    return;
                }
            }
        }
        let awake = awake_rx.try_get().unwrap_or(true);
        // The hardware counts independently of the CPU and display. Refresh
        // promptly on wake; background reads are bounded rather than per-step.
        let interval = if awake { 5 } else { 30 };
        let _ = awake_rx.try_changed();
        let _ = date_rx.try_changed();
        embassy_futures::select::select(
            Timer::after(Duration::from_secs(interval)),
            embassy_futures::select::select(awake_rx.changed(), date_rx.changed()),
        )
        .await;
    }
}

fn heart_error(error: &hrs3300::Error<embassy_nrf::twim::Error>) -> u8 {
    match error {
        hrs3300::Error::Bus(_) => 1,
        hrs3300::Error::WrongId(_) => 2,
        hrs3300::Error::NotRunning => 3,
        hrs3300::Error::InvalidGain(_) => 4,
        hrs3300::Error::Readback { register, .. } => 0x80 | register,
    }
}

async fn stop_heart(
    sensor: &mut hrs3300::Hrs3300<device::i2c::BoundedDevice<'_>>,
    diagnostic: &mut heart_rate::Diagnostics,
) -> bool {
    for attempt in 0..3 {
        match sensor.stop().await {
            Ok(()) => return true,
            Err(error) => {
                diagnostic.error = heart_error(&error);
                defmt::warn!(
                    "[hr] shutdown attempt {}: {:?}",
                    attempt + 1,
                    defmt::Debug2Format(&error)
                );
                Timer::after(Duration::from_millis(50)).await;
            }
        }
    }
    false
}

#[embassy_executor::task]
async fn heart_task(bus: &'static device::i2c::Bus) {
    use heart_rate::{Estimator, Phase, Reading, State, SAMPLE_MS};
    let mut sensor = hrs3300::Hrs3300::new(device::i2c::BoundedDevice::new(bus));
    let sender = HEART_STATE.sender();
    let mut state = State::EMPTY;
    let mut diagnostic = heart_rate::Diagnostics::EMPTY;
    match sensor.probe().await {
        Ok(()) => defmt::info!("[hr] HRS3300 detected and powered down"),
        Err(error) => {
            defmt::warn!("[hr] sensor unavailable: {:?}", defmt::Debug2Format(&error));
            state.phase = Phase::Missing;
            diagnostic.error = heart_error(&error);
            if !stop_heart(&mut sensor, &mut diagnostic).await {
                state.phase = Phase::Fault;
            }
        }
    }
    sender.send(state);
    HEART_DIAGNOSTICS.sender().send(diagnostic);
    loop {
        if !HEART_REQUEST.wait().await {
            continue;
        }
        diagnostic = heart_rate::Diagnostics::EMPTY;
        if let Err(error) = sensor.start().await {
            defmt::warn!("[hr] start failed: {:?}", defmt::Debug2Format(&error));
            state.phase = Phase::Fault;
            diagnostic.error = heart_error(&error);
            let _ = stop_heart(&mut sensor, &mut diagnostic).await;
            sender.send(state);
            HEART_DIAGNOSTICS.sender().send(diagnostic);
            continue;
        }
        state.phase = Phase::Measuring;
        state.started_ms = Instant::now().as_millis();
        diagnostic.gain = hrs3300::MEASUREMENT_GAIN;
        sender.send(state);
        defmt::info!(
            "[hr] measuring; 15-bit, 10 Hz, gain x{}, 12.5 mA drive; 45 second limit",
            hrs3300::MEASUREMENT_GAIN
        );
        let mut estimator = Estimator::new();
        HEART_DIAGNOSTICS.sender().send(diagnostic);
        let mut window_min = u32::MAX;
        let mut window_max = 0u32;
        let settle_until = state.started_ms + 200;
        let mut next_sample = Instant::now() + Duration::from_millis(SAMPLE_MS);
        let mut publish_at = Instant::now();
        let mut timed_out = false;
        let mut failed = false;
        loop {
            if HEART_REQUEST.try_take() == Some(false) {
                break;
            }
            match embassy_futures::select::select(HEART_REQUEST.wait(), Timer::at(next_sample))
                .await
            {
                embassy_futures::select::Either::First(false) => break,
                embassy_futures::select::Either::First(true) => continue,
                embassy_futures::select::Either::Second(_) => {}
            }
            let now = Instant::now();
            if state.expired(now.as_millis()) {
                timed_out = true;
                break;
            }
            match sensor.sample().await {
                Ok(sample) => {
                    // Timestamp the completed read, including any shared-bus
                    // wait, rather than pretending it happened at timer wakeup.
                    let now = Instant::now();
                    if state.expired(now.as_millis()) {
                        timed_out = true;
                        break;
                    }
                    diagnostic.ppg = sample.ppg;
                    diagnostic.ambient = sample.ambient;
                    diagnostic.samples = diagnostic.samples.saturating_add(1);
                    diagnostic.at_ms = now.as_millis();
                    if now.as_millis() >= settle_until {
                        window_min = window_min.min(sample.ppg);
                        window_max = window_max.max(sample.ppg);
                        diagnostic.min = window_min.min(u32::from(u16::MAX)) as u16;
                        diagnostic.max = window_max.min(u32::from(u16::MAX)) as u16;
                        let analysis_at = Instant::now();
                        let reading = estimator.sample(sample.ppg, sample.ambient, now.as_millis());
                        diagnostic.quality = estimator.quality();
                        let analysis_ms = Instant::now().duration_since(analysis_at).as_millis();
                        if analysis_ms >= 10 {
                            defmt::debug!("[hr] analysis took {} ms", analysis_ms);
                        }
                        if let Some(bpm) = reading {
                            state.last = Some(Reading {
                                bpm,
                                at_ms: now.as_millis(),
                            });
                            defmt::info!("[hr] pulse candidate: {} BPM", bpm);
                            // Publish a newly qualified reading immediately;
                            // the independent housekeeping phase can lag it.
                            sender.send(state);
                            publish_at = Instant::now() + Duration::from_secs(1);
                        }
                    }
                    if now >= publish_at {
                        sender.send(state);
                        window_min = u32::MAX;
                        window_max = 0;
                        defmt::debug!("[hr] PPG {}, ambient {}", sample.ppg, sample.ambient);
                        publish_at = now + Duration::from_secs(1);
                    }
                    // Latest-value debug stream; no queue or raw sample history.
                    // Only an explicitly subscribed diagnostic client transmits it.
                    HEART_DIAGNOSTICS.sender().send(diagnostic);
                }
                Err(error) => {
                    diagnostic.error = heart_error(&error);
                    defmt::warn!("[hr] sample failed: {:?}", defmt::Debug2Format(&error));
                    failed = true;
                    break;
                }
            }
            // No catch-up burst: skipped samples reset estimator timing.
            next_sample += Duration::from_millis(SAMPLE_MS);
            if next_sample <= Instant::now() {
                next_sample = Instant::now() + Duration::from_millis(SAMPLE_MS);
            }
        }
        let had_signal = state.bpm(Instant::now().as_millis()).is_some();
        state.phase = if failed {
            Phase::Fault
        } else if timed_out && !had_signal {
            Phase::NoSignal
        } else {
            Phase::Idle
        };
        // Every exit path attempts LED+engine shutdown before publishing idle.
        if stop_heart(&mut sensor, &mut diagnostic).await {
            defmt::info!("[hr] sensor and LED drive off");
        } else {
            state.phase = Phase::Fault;
        }
        sender.send(state);
        HEART_DIAGNOSTICS.sender().send(diagnostic);
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
    notification_tx: embassy_sync::channel::Sender<
        'static,
        CriticalSectionRawMutex,
        notifications::Notification,
        2,
    >,
    flash: &'static AsyncMutex<CriticalSectionRawMutex, flash::Flash<FlashSpi>>,
    battery_rx: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, BatteryState, 2>,
) {
    ble::run(
        controller,
        address,
        time_rx,
        set_time_tx,
        BLE_CONNECTED_WATCH.sender(),
        notification_tx,
        flash,
        battery_rx,
        &BATTERY_HISTORY,
    )
    .await
}

/// PineTime's active-low motor output, matching InfiniTime's MotorController.
/// TIMER2 and PPI switch it off independently of cooperative executor scheduling.
#[embassy_executor::task]
async fn vibration_task(
    motor: GpioteOutput<'static>,
    timer: HardwareTimer<'static>,
    _shutdown: Ppi<'static, embassy_nrf::peripherals::PPI_CH0, 1, 1>,
) {
    loop {
        let pattern = VIBRATIONS.receive().await;
        for (index, pulse_ms) in pattern.pulses_ms().iter().enumerate() {
            if index > 0 {
                Timer::after(Duration::from_millis(100)).await;
            }
            timer.clear();
            timer.cc(0).clear_events();
            timer.cc(0).write(*pulse_ms * 1_000);
            // PPI switches the motor off even if the executor is busy.
            motor.out();
            timer.start();
            Timer::after(Duration::from_millis(u64::from(*pulse_ms))).await;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;
    // PineTime has the external DCC inductor; InfiniTime enables this too.
    // Allow the regulator to choose the more efficient DC/DC mode under load.
    config.dcdc.reg1 = true;
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);
    #[cfg(feature = "mcuboot")]
    let mut nvmc = embassy_nrf::nvmc::Nvmc::new(p.NVMC);
    let motor = GpioteOutput::new(
        p.GPIOTE_CH0,
        p.P0_16,
        Level::High,
        OutputDrive::Standard,
        OutputChannelPolarity::Toggle,
    );
    let timer = HardwareTimer::new(p.TIMER2);
    timer.set_frequency(TimerFrequency::F1MHz);
    timer.cc(0).write(50_000);
    timer.cc(0).short_compare_stop();
    let mut shutdown =
        Ppi::new_one_to_one(p.PPI_CH0, timer.cc(0).event_compare(), motor.task_out());
    shutdown.enable();
    spawner
        .spawn(vibration_task(motor, timer, shutdown))
        .unwrap();

    // Adopt an already-running watchdog before BLE initialization and its
    // startup delay. This does not confirm a trial image.
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
        defmt::info!("WDT not running; no watchdog to adopt");
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
    let (twim, motion_known) =
        device::i2c::prepare(p.TWISPI1, p.P0_06, p.P0_07, SENSOR_TX_BUFFER.init([0; 32])).await;
    let sensor_bus = SENSOR_BUS.init(AsyncMutex::new(twim));
    spawner.spawn(heart_task(sensor_bus)).unwrap();
    let touch = device::input::TouchScreen::new(
        device::i2c::Device::new(sensor_bus),
        Input::new(p.P0_28, Pull::Up),
        Output::new(p.P0_10, Level::High, OutputDrive::Standard),
    );
    if motion_known {
        spawner
            .spawn(motion_task(
                device::i2c::Device::new(sensor_bus),
                SCREEN_AWAKE_WATCH.receiver().unwrap(),
            ))
            .unwrap();
    }

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
    let spim = Spim::new(
        p.TWISPI0,
        Irqs,
        p.P0_02, // SCK
        p.P0_04, // MISO from external flash
        p.P0_03, // MOSI
        spim_config,
    );
    let spi_bus = SPI_BUS.init(BlockingMutex::new(RefCell::new(spim)));
    let spi_dev = SharedSpiDevice::new(spi_bus, cs);
    let flash_cs = Output::new(p.P0_05, Level::High, OutputDrive::Standard);
    let flash = FLASH_CELL.init(AsyncMutex::new(flash::Flash::new(SharedSpiDevice::new(
        spi_bus, flash_cs,
    ))));
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

    let mut display = Builder::new(display_cfg::PineTimeSt7789, di)
        .display_size(display_cfg::DISPLAY_WIDTH, display_cfg::DISPLAY_HEIGHT)
        .display_offset(display_cfg::DISPLAY_OFFSET_X, display_cfg::DISPLAY_OFFSET_Y)
        .orientation(display_cfg::DISPLAY_ORIENTATION)
        .color_order(display_cfg::DISPLAY_COLOR_ORDER)
        .invert_colors(display_cfg::DISPLAY_COLOR_INVERSION)
        .reset_pin(rst)
        .init(&mut delay)
        .unwrap();

    display.clear(display_cfg::BACKGROUND_COLOR).unwrap();
    let mut watch_ui = ui::WatchUi::new();
    watch_ui.draw_static(&mut display);

    let charge_pin = Input::new(p.P0_12, Pull::Up);
    let channel = saadc::ChannelConfig::single_ended(p.P0_31);
    let saadc = saadc::Saadc::new(p.SAADC, Irqs, saadc::Config::default(), [channel]);
    let battery = device::battery::Battery::new(saadc, charge_pin);

    #[cfg(feature = "battery-persist-experimental")]
    if let Err(error) = battery_persist::load(flash, &BATTERY_HISTORY).await {
        defmt::warn!(
            "Battery LittleFS unavailable; keeping history in RAM: {}",
            error.code()
        );
    }

    spawner
        .spawn(clock_task(TIME_WATCH.sender(), SET_TIME_CH.receiver()))
        .unwrap();
    spawner
        .spawn(battery_task(
            battery,
            BATTERY_WATCH.sender(),
            #[cfg(feature = "battery-persist-experimental")]
            flash,
        ))
        .unwrap();
    spawner
        .spawn(button_task(button, INPUT_CH.sender()))
        .unwrap();
    SCREEN_AWAKE_WATCH.sender().send(true);
    spawner
        .spawn(touch_task(
            touch,
            SCREEN_AWAKE_WATCH.receiver().unwrap(),
            INPUT_CH.sender(),
        ))
        .unwrap();

    let time_rx_ble = TIME_WATCH.receiver().unwrap();
    // Use the factory static-random address, matching InfiniTime's nRF52
    // ble_hw_get_static_addr. Firmware changes must not change device identity.
    let low = embassy_nrf::pac::FICR.deviceaddr(0).read().to_le_bytes();
    let high = embassy_nrf::pac::FICR.deviceaddr(1).read().to_le_bytes();
    let address = [low[0], low[1], low[2], low[3], high[0], high[1] | 0xc0];
    watch_ui.set_ble_address(address);
    let copy_done =
        unsafe { core::ptr::read_volatile(ota::PRIMARY_COPY_DONE_ADDRESS as *const u8) };
    let image_ok = unsafe { core::ptr::read_volatile(ota::PRIMARY_IMAGE_OK_ADDRESS as *const u8) };
    #[cfg(feature = "mcuboot")]
    watch_ui.set_firmware_state(firmware_confirm::state(), false);
    if copy_done == 1 && image_ok == 0xff {
        defmt::warn!("Running unconfirmed trial firmware; next reset reverts");
        OTA_WATCH.sender().send(UpdateStatus::Trial);
    } else {
        OTA_WATCH.sender().send(UpdateStatus::Idle);
    }
    spawner
        .spawn(ble_peripheral_task(
            controller,
            address,
            time_rx_ble,
            SET_TIME_CH.sender(),
            NOTIFICATION_CH.sender(),
            flash,
            BATTERY_WATCH.receiver().unwrap(),
        ))
        .unwrap();

    defmt::info!("Kongle started");

    let mut time_rx = TIME_WATCH.receiver().unwrap();
    let mut battery_rx = BATTERY_WATCH.receiver().unwrap();
    let mut ota_rx = OTA_WATCH.receiver().unwrap();
    let mut ble_rx = BLE_CONNECTED_WATCH.receiver().unwrap();
    let input_rx = INPUT_CH.receiver();
    let notification_rx = NOTIFICATION_CH.receiver();
    let mut media_rx = MEDIA_WATCH.receiver().unwrap();
    let mut motion_rx = MOTION_WATCH.receiver().unwrap();

    let mut last_ota = UpdateStatus::Idle;
    let mut ble_connected = false;
    let mut awake = true;
    let mut sleep_at = Instant::now() + Duration::from_secs(30);
    let mut redraw = false;
    let mut pending_button_sleep = false;
    let mut heart_page_was_open = false;
    let mut heart_was_measuring = false;
    loop {
        match embassy_futures::select::select(
            ota_rx.changed(),
            embassy_futures::select::select(
                time_rx.changed(),
                embassy_futures::select::select(
                    battery_rx.changed(),
                    embassy_futures::select::select(
                        input_rx.receive(),
                        embassy_futures::select::select(
                            ble_rx.changed(),
                            embassy_futures::select::select(
                                notification_rx.receive(),
                                media_rx.changed(),
                            ),
                        ),
                    ),
                ),
            ),
        )
        .await
        {
            embassy_futures::select::Either::First(status) => {
                pending_button_sleep = false;
                last_ota = status;
                sleep_at = Instant::now() + Duration::from_secs(30);
                if matches!(
                    status,
                    UpdateStatus::Erasing
                        | UpdateStatus::Receiving { .. }
                        | UpdateStatus::Validating
                ) && watch_ui.show_clock()
                {
                    redraw = true;
                }
                if matches!(
                    status,
                    UpdateStatus::Erasing
                        | UpdateStatus::Receiving { .. }
                        | UpdateStatus::Validating
                ) && !awake
                {
                    awake = true;
                    redraw = true;
                }
                if !awake {
                    continue;
                }
                if !redraw {
                    watch_ui.draw_update(&mut display, status);
                }
            }
            embassy_futures::select::Either::Second(embassy_futures::select::Either::First(
                time,
            )) => {
                if !awake {
                    continue;
                }
                watch_ui.draw_time(&mut display, time);
            }
            embassy_futures::select::Either::Second(embassy_futures::select::Either::Second(
                embassy_futures::select::Either::First(battery_state),
            )) => {
                if !awake {
                    continue;
                }
                watch_ui.draw_battery(&mut display, battery_state);
            }
            embassy_futures::select::Either::Second(embassy_futures::select::Either::Second(
                embassy_futures::select::Either::Second(embassy_futures::select::Either::First(
                    event,
                )),
            )) => {
                if matches!(
                    event,
                    InputEvent::ButtonPressed | InputEvent::Touch(_) | InputEvent::TouchWake
                ) {
                    pending_button_sleep = false;
                }
                if matches!(event, InputEvent::ButtonReleased) {
                    pending_button_sleep = awake && watch_ui.is_clock();
                    if !pending_button_sleep {
                        awake = watch_ui.handle_button(awake);
                        redraw = true;
                        sleep_at = Instant::now() + Duration::from_secs(30);
                    }
                }
                if matches!(event, InputEvent::ButtonSingleConfirmed) && pending_button_sleep {
                    pending_button_sleep = false;
                    awake = false;
                }
                if matches!(event, InputEvent::ButtonDoublePressed) {
                    pending_button_sleep = false;
                    watch_ui.open_media();
                    awake = true;
                    redraw = true;
                    sleep_at = Instant::now() + Duration::from_secs(30);
                    defmt::info!("Button double press: media");
                }
                if matches!(event, InputEvent::TouchWake)
                    && cfg!(feature = "touch-wake-debug")
                    && !awake
                {
                    awake = true;
                    redraw = true;
                    sleep_at = Instant::now() + Duration::from_secs(30);
                }
                if let InputEvent::Touch(gesture) = event {
                    if awake {
                        sleep_at = Instant::now() + Duration::from_secs(30);
                        let previous_brightness = watch_ui.brightness_level();
                        let needs_redraw = watch_ui.handle_touch(gesture);
                        if watch_ui.brightness_level() != previous_brightness {
                            backlight.set(watch_ui.brightness_level());
                        }
                        if watch_ui.take_confirmation_request() {
                            #[cfg(feature = "mcuboot")]
                            {
                                let busy = matches!(
                                    last_ota,
                                    UpdateStatus::Erasing
                                        | UpdateStatus::Receiving { .. }
                                        | UpdateStatus::Validating
                                        | UpdateStatus::Staged
                                        | UpdateStatus::ReadyToRestart
                                );
                                let result = if busy {
                                    Err(())
                                } else {
                                    firmware_confirm::confirm(&mut nvmc)
                                };
                                match result {
                                    Ok(state) => {
                                        defmt::info!(
                                            "Firmware manually confirmed; reset no longer reverts"
                                        );
                                        watch_ui.set_firmware_state(state, false);
                                        last_ota = UpdateStatus::Idle;
                                        OTA_WATCH.sender().send(last_ota);
                                    }
                                    Err(()) => {
                                        defmt::warn!(
                                            "Firmware confirmation refused or readback failed"
                                        );
                                        watch_ui
                                            .set_firmware_state(firmware_confirm::state(), true);
                                    }
                                }
                            }
                            redraw = true;
                        } else if needs_redraw {
                            redraw = true;
                        } else {
                            watch_ui.draw_changed_settings(&mut display);
                        }
                    }
                }
                #[cfg(feature = "mcuboot")]
                if matches!(event, InputEvent::ButtonHeld) {
                    defmt::info!("Long hold: restarting into MCUBoot button menu");
                    cortex_m::peripheral::SCB::sys_reset();
                }
            }
            embassy_futures::select::Either::Second(embassy_futures::select::Either::Second(
                embassy_futures::select::Either::Second(embassy_futures::select::Either::Second(
                    embassy_futures::select::Either::First(connected),
                )),
            )) => {
                if ble_connected != connected && watch_ui.vibration_enabled() {
                    let pattern = if connected {
                        haptics::Pattern::Connected
                    } else {
                        haptics::Pattern::Disconnected
                    };
                    if VIBRATIONS.try_send(pattern).is_err() {
                        defmt::warn!("Vibration queue full; dropped connection feedback");
                    }
                }
                ble_connected = connected;
                if !awake {
                    continue;
                }
                watch_ui.draw_connection(&mut display, connected);
            }
            embassy_futures::select::Either::Second(embassy_futures::select::Either::Second(
                embassy_futures::select::Either::Second(embassy_futures::select::Either::Second(
                    embassy_futures::select::Either::Second(
                        embassy_futures::select::Either::First(notification),
                    ),
                )),
            )) => {
                let show = !matches!(
                    last_ota,
                    UpdateStatus::Erasing
                        | UpdateStatus::Receiving { .. }
                        | UpdateStatus::Validating
                );
                watch_ui.receive_notification(notification, show);
                if awake {
                    watch_ui.draw_notification_indicator(&mut display);
                }
                if watch_ui.vibration_enabled() {
                    let _ = VIBRATIONS.try_send(haptics::Pattern::Notification);
                }
                defmt::info!("[ans] notification stored; preview={}", show);
                if show {
                    pending_button_sleep = false;
                    awake = true;
                    redraw = true;
                    sleep_at = Instant::now() + Duration::from_secs(30);
                }
            }
            embassy_futures::select::Either::Second(embassy_futures::select::Either::Second(
                embassy_futures::select::Either::Second(embassy_futures::select::Either::Second(
                    embassy_futures::select::Either::Second(
                        embassy_futures::select::Either::Second(state),
                    ),
                )),
            )) => {
                if watch_ui.set_media(state) && awake {
                    watch_ui.draw_media(&mut display);
                }
            }
        }
        if let Some(raw_steps) = motion_rx.try_changed() {
            if watch_ui.set_steps(raw_steps) && awake && !redraw {
                watch_ui.draw_steps(&mut display);
            }
        }
        let heart_state = HEART_STATE.try_get().unwrap_or(heart_rate::State::EMPTY);
        let heart_measuring = heart_state.phase == heart_rate::Phase::Measuring;
        if awake && watch_ui.heart_visible() {
            if heart_measuring && !heart_state.expired(Instant::now().as_millis()) {
                sleep_at = sleep_at.max(Instant::now() + Duration::from_secs(2));
            } else if heart_was_measuring && !heart_measuring {
                sleep_at = Instant::now() + Duration::from_secs(30);
            }
        }
        heart_was_measuring = heart_measuring;
        let heart_changed = watch_ui.set_heart(heart_state, Instant::now().as_millis());
        let quality_changed = watch_ui.set_heart_quality(
            HEART_DIAGNOSTICS
                .try_get()
                .unwrap_or(heart_rate::Diagnostics::EMPTY)
                .quality,
        );
        if (heart_changed || quality_changed) && awake && !redraw {
            watch_ui.draw_heart(&mut display);
        }
        if let Some(start) = watch_ui.take_heart_request() {
            if !start
                || (watch_ui.heart_visible()
                    && !matches!(
                        last_ota,
                        UpdateStatus::Erasing
                            | UpdateStatus::Receiving { .. }
                            | UpdateStatus::Validating
                    ))
            {
                HEART_REQUEST.signal(start);
            }
        }
        if heart_page_was_open && !watch_ui.heart_visible() {
            HEART_REQUEST.signal(false);
        }
        heart_page_was_open = watch_ui.heart_visible();
        if let Some(request) = watch_ui.take_media_request() {
            if MEDIA_COMMANDS.try_send(request).is_err() {
                defmt::warn!("[media] command queue full");
            }
        }
        if awake
            && Instant::now() >= sleep_at
            && !matches!(
                last_ota,
                UpdateStatus::Erasing | UpdateStatus::Receiving { .. } | UpdateStatus::Validating
            )
        {
            awake = false;
        }
        if !awake {
            if !display.is_sleeping() {
                HEART_REQUEST.signal(false);
                backlight.off();
                display.sleep(&mut DeferredDelay).unwrap();
                SCREEN_AWAKE_WATCH.sender().send(false);
                Timer::after(Duration::from_millis(120)).await;
                defmt::info!("Display asleep; BLE remains active");
            }
            continue;
        }
        let waking = display.is_sleeping();
        if waking {
            display.wake(&mut DeferredDelay).unwrap();
            Timer::after(Duration::from_millis(120)).await;
            redraw = true;
        }
        if redraw {
            // Keep the backlight dark until the full wake redraw has finished.
            // Otherwise the clear and each text draw become visible in sequence.
            if !waking {
                backlight.off();
            }
            display.clear(display_cfg::BACKGROUND_COLOR).unwrap();
            watch_ui.draw_full(
                &mut display,
                time_rx.try_get(),
                battery_rx.try_get(),
                last_ota,
                ble_connected,
            );
            redraw = false;
            SCREEN_AWAKE_WATCH.sender().send(true);
            backlight.set(watch_ui.brightness_level());
            if waking {
                defmt::info!("Display awake");
            }
        }
    }
}

/// The display library sends the sleep command synchronously. Its required
/// 120 ms settling time is awaited by the caller so BLE is not starved.
struct DeferredDelay;
impl embedded_hal::delay::DelayNs for DeferredDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}
