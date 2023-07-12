// #![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]
// #![cfg_attr(not(test))]
// #![warn(rust_2018_idioms)]
// Panic handler
#[cfg(not(test))]
use panic_rtt_target as _;

use crate::monotonic_nrf52::MonoTimer;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::{
    geometry::Point,
    geometry::Size,
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::rectangle::Rectangle,
    primitives::PrimitiveStyleBuilder,
    text::Text,
};
use nrf52832_hal as hal;
use nrf52832_hal::gpio::{p0, Floating, Input, Level, Output, Pin, PushPull};
use nrf52832_hal::prelude::*;
use numtoa::NumToA;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

use rubble::{
    config::Config,
    gatt::BatteryServiceAttrs,
    l2cap::{BleChannelMap, L2CAPState},
    link::{
        ad_structure::AdStructure,
        queue::{PacketQueue, SimpleQueue},
        LinkLayer, Responder, MIN_PDU_BUF,
    },
    security::NoSecurity,
    time::{Duration as RubbleDuration, Timer},
};
use rubble_nrf5x::{
    radio::{BleRadio, PacketBuffer},
    timer::BleTimer,
    utils::get_device_address,
};

use chrono::{NaiveDateTime, Timelike};
use st7789::{self, Orientation};

use core::fmt::Write;
use heapless::String;

mod backlight;
mod battery;
mod delay;
mod monotonic_nrf52;

use debouncr::{debounce_6, Debouncer, Edge, Repeat6};
use monotonic_nrf52::ExtU32;

const LCD_W: u16 = 240;
const LCD_H: u16 = 240;

const MARGIN: u16 = 10;

const BACKGROUND_COLOR: Rgb565 = Rgb565::new(0, 0b000111, 0);

pub struct AppConfig {}

impl Config for AppConfig {
    type Timer = BleTimer<hal::pac::TIMER2>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<BatteryServiceAttrs, NoSecurity>;
    type PacketQueue = &'static mut SimpleQueue;
}

#[app(device = crate::hal::pac, peripherals = true, dispatchers = [SWI0_EGU0,SWI1_EGU1,SWI2_EGU2,SWI3_EGU3,SWI4_EGU4,SWI5_EGU5])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        lcd: st7789::ST7789<
            display_interface_spi::SPIInterfaceNoCS<
                hal::spim::Spim<hal::pac::SPIM1>,
                p0::P0_18<Output<PushPull>>,
            >,
            p0::P0_26<Output<PushPull>>,
            p0::P0_22<Output<PushPull>>,
        >,

        // Battery
        battery: battery::BatteryStatus,

        // Styles
        // text_style: MonoTextStyleBuilder<'static,Rgb565>,
        // text_style: MonoTextStyleBuilder<Rgb565>,

        // BLE
        radio: BleRadio,
        ble_ll: LinkLayer<AppConfig>,
        ble_r: Responder<AppConfig>,
    }

    #[local]
    struct Local {
        backlight: backlight::Backlight,

        // ts resources
        ts: i64,

        // Button
        button: Pin<Input<Floating>>,
        button_debouncer: Debouncer<u8, Repeat6>,
    }

    #[monotonic(binds = TIMER1, default = true)]
    type Tonic = MonoTimer<hal::pac::TIMER1>;

    // let buffer: &'static mut [u8; 1024] = cx.local.buffer;
    #[init(local = [ble_tx_buf: PacketBuffer = [0; MIN_PDU_BUF],
        ble_rx_buf: PacketBuffer = [0; MIN_PDU_BUF],
        tx_queue: SimpleQueue = SimpleQueue::new(),
        rx_queue: SimpleQueue = SimpleQueue::new()
        ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Destructure device peripherals
        let hal::pac::Peripherals {
            CLOCK,
            FICR,
            P0,
            RADIO,
            SAADC,
            SPIM1,
            TIMER0,
            TIMER1,
            TIMER2,
            ..
        } = cx.device;

        // Init RTT
        rtt_init_print!();
        rprintln!("Initializing…");

        // Set up clocks. On reset, the high frequency clock is already used,
        // but we also need to switch to the external HF oscillator. This is
        // needed for Bluetooth to work.
        let _clocks = hal::clocks::Clocks::new(CLOCK).enable_ext_hfosc();

        // Set up delay provider on TIMER0
        let mut lcd_delay = delay::TimerDelay::new(TIMER0);

        // Initialize monotonic timer on TIMER1 (for RTIC)
        // monotonic_nrf52::Tim1::initialize(TIMER1);
        let mono = MonoTimer::new(TIMER1);

        // Initialize BLE timer on TIMER2
        let ble_timer = BleTimer::init(TIMER2);

        // Set up GPIO peripheral
        let gpio = hal::gpio::p0::Parts::new(P0);

        // Enable backlight
        let backlight = backlight::Backlight::init(
            gpio.p0_14.into_push_pull_output(Level::High).degrade(),
            gpio.p0_22.into_push_pull_output(Level::High).degrade(),
            gpio.p0_23.into_push_pull_output(Level::High).degrade(),
            1,
        );

        // Battery status
        let battery = battery::BatteryStatus::init(
            gpio.p0_12.into_floating_input(),
            gpio.p0_31.into_floating_input(),
            SAADC,
        );

        // Enable button
        gpio.p0_15.into_push_pull_output(Level::High);
        let button = gpio.p0_13.into_floating_input().degrade();

        // Get bluetooth device address
        let device_address = get_device_address();
        rprintln!("Bluetooth device address: {:?}", device_address);

        // Initialize radio
        let mut radio = BleRadio::new(RADIO, &FICR, cx.local.ble_tx_buf, cx.local.ble_rx_buf);

        // Create bluetooth TX/RX queues
        let (tx, tx_cons) = cx.local.tx_queue.split();
        let (rx_prod, rx) = cx.local.rx_queue.split();

        // Create the actual BLE stack objects
        let mut ble_ll = LinkLayer::<AppConfig>::new(device_address, ble_timer);
        let ble_r = Responder::<AppConfig>::new(
            tx,
            rx,
            L2CAPState::new(BleChannelMap::with_attributes(BatteryServiceAttrs::new())),
        );

        // Send advertisement and set up regular interrupt
        let next_update = ble_ll
            .start_advertise(
                RubbleDuration::from_millis(200),
                &[AdStructure::CompleteLocalName("Kongle")],
                &mut radio,
                tx_cons,
                rx_prod,
            )
            .unwrap();
        ble_ll.timer().configure_interrupt(next_update);

        // Set up SPI pins
        let spi_clk = gpio.p0_02.into_push_pull_output(Level::Low).degrade();
        let spi_mosi = gpio.p0_03.into_push_pull_output(Level::Low).degrade();
        let spi_miso = gpio.p0_04.into_floating_input().degrade();
        let spi_pins = hal::spim::Pins {
            sck: Some(spi_clk),
            miso: Some(spi_miso),
            mosi: Some(spi_mosi),
        };

        // Set up LCD pins
        // LCD_CS (P0.25): Chip select
        let mut lcd_cs = gpio.p0_25.into_push_pull_output(Level::Low);
        // LCD_RS (P0.18): Data/clock pin
        let lcd_dc = gpio.p0_18.into_push_pull_output(Level::Low);
        // LCD_RESET (P0.26): Display reset
        let lcd_rst = gpio.p0_26.into_push_pull_output(Level::Low);

        // Initialize SPI
        let spi = hal::Spim::new(
            SPIM1,
            spi_pins,
            // Use SPI at 8MHz (the fastest clock available on the nRF52832)
            // because otherwise refreshing will be super slow.
            hal::spim::Frequency::M8,
            // SPI must be used in mode 3. Mode 0 (the default) won't work.
            hal::spim::MODE_3,
            0,
        );

        // Chip select must be held low while driving the display. It must be high
        // when using other SPI devices on the same bus (such as external flash
        // storage) so that the display controller won't respond to the wrong
        // commands.
        lcd_cs.set_low().unwrap();

        // display interface abstraction from SPI and DC
        let di = SPIInterfaceNoCS::new(spi, lcd_dc);

        // Initialize LCD
        let mut lcd = st7789::ST7789::new(di, Some(lcd_rst), None, LCD_W, LCD_H);
        lcd.init(&mut lcd_delay).unwrap();
        lcd.set_orientation(Orientation::Portrait).unwrap();

        // Draw something onto the LCD
        let backdrop_style = PrimitiveStyleBuilder::new()
            .fill_color(BACKGROUND_COLOR)
            .build();
        Rectangle::new(Point::new(0, 0), Size::new(LCD_W as u32, LCD_H as u32))
            .into_styled(backdrop_style)
            .draw(&mut lcd)
            .unwrap();

        // Choose text style
        // let text_style = TextStyleBuilder::new()
        //     .font(&FONT_10X20)
        //     .text_color(Rgb565::WHITE);
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::WHITE)
            .build();

        // Draw text
        Text::new("Kongle PineTime", Point::new(10, 20), text_style)
            .draw(&mut lcd)
            .unwrap();

        // Schedule tasks immediately
        write_ts::spawn().unwrap();
        poll_button::spawn().unwrap();
        show_battery_status::spawn().unwrap();
        update_battery_status::spawn().unwrap();

        (
            Shared {
                lcd,
                battery,
                // text_style,
                radio,
                ble_ll,
                ble_r,
            },
            Local {
                backlight,
                ts: 0,
                button,
                button_debouncer: debounce_6(false),
            },
            init::Monotonics(mono),
        )
    }

    /// Hook up the RADIO interrupt to the Rubble BLE stack.
    #[task(binds = RADIO, shared = [radio, ble_ll], priority = 3)]
    fn radio(mut cx: radio::Context) {
        cx.shared.ble_ll.lock(|ble_ll| {
            cx.shared.radio.lock(|radio| {
                if let Some(cmd) = radio.recv_interrupt(ble_ll.timer().now(), ble_ll) {
                    radio.configure_receiver(cmd.radio);
                    ble_ll.timer().configure_interrupt(cmd.next_update);

                    if cmd.queued_work {
                        // If there's any lower-priority work to be done, ensure that happens.
                        // If we fail to spawn the task, it's already scheduled.
                        ble_worker::spawn().ok();
                    }
                }
            });
        });
    }

    /// Hook up the TIMER2 interrupt to the Rubble BLE stack.
    #[task(binds = TIMER2, shared = [radio, ble_ll], priority = 3)]
    fn timer2(mut cx: timer2::Context) {
        cx.shared.ble_ll.lock(|ble_ll| {
            cx.shared.radio.lock(|radio| {
                let timer = ble_ll.timer();
                if !timer.is_interrupt_pending() {
                    return;
                }
                timer.clear_interrupt();

                let cmd = ble_ll.update_timer(&mut *radio);
                radio.configure_receiver(cmd.radio);

                ble_ll.timer().configure_interrupt(cmd.next_update);

                if cmd.queued_work {
                    // If there's any lower-priority work to be done, ensure that happens.
                    // If we fail to spawn the task, it's already scheduled.
                    ble_worker::spawn().ok();
                }
            });
        });
    }

    /// Lower-priority task spawned from RADIO and TIMER2 interrupts.
    #[task(shared = [ble_r], priority = 2)]
    fn ble_worker(mut cx: ble_worker::Context) {
        // Fully drain the packet queue
        cx.shared.ble_r.lock(|ble_r| {
            while ble_r.has_work() {
                ble_r.process_one().unwrap();
            }
        })
    }

    #[task(shared = [lcd], local = [ts])]
    fn write_ts(mut cx: write_ts::Context) {
        rprintln!("ts is {}", cx.local.ts);

        // Write time to the display
        let dt = NaiveDateTime::from_timestamp_opt(*cx.local.ts, 0);
        let time = dt.unwrap().time();
        let mut text: heapless::String<6> = String::new();
        write!(&mut text, "{:02}:{:02}", time.hour(), time.minute()).unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::WHITE)
            .background_color(BACKGROUND_COLOR)
            .build();

        let text = Text::new(
            &text,
            Point::new(10, LCD_H as i32 - 10 - MARGIN as i32),
            text_style,
        );

        cx.shared.lcd.lock(|lcd| {
            text.draw(lcd).unwrap();
        });

        // Increment ts
        *cx.local.ts += 1;

        // Re-schedule the timer interrupt
        write_ts::spawn_after(1000.millis()).unwrap();
    }

    #[task(local = [button, button_debouncer])]
    fn poll_button(cx: poll_button::Context) {
        // Poll button
        let pressed = cx.local.button.is_high().unwrap();
        let edge = cx.local.button_debouncer.update(pressed);

        // Dispatch event
        if edge == Some(Edge::Rising) {
            button_pressed::spawn().unwrap();
        }

        // Re-schedule the timer interrupt in 2ms
        poll_button::spawn_after(2.millis()).unwrap();
    }

    /// Called when button is pressed without bouncing for 12 (6 * 2) ms.
    #[task(local = [backlight])]
    fn button_pressed(cx: button_pressed::Context) {
        if cx.local.backlight.get_brightness() < 7 {
            cx.local.backlight.brighter();
        } else {
            cx.local.backlight.off();
        }
    }

    /// Fetch the battery status from the hardware. Update the text if
    /// something changed.
    #[task(shared = [battery])]
    fn update_battery_status(mut cx: update_battery_status::Context) {
        rprintln!("Update battery status");

        let changed = cx.shared.battery.lock(|battery| battery.update());
        if changed {
            rprintln!("Battery status changed");
            show_battery_status::spawn().unwrap();
        }

        // Re-schedule the timer interrupt in 1s
        update_battery_status::spawn_after(1000.millis()).unwrap();
    }

    /// Show the battery status on the LCD.
    #[task(shared = [battery, lcd])]
    fn show_battery_status(mut cx: show_battery_status::Context) {
        let mut voltage = 0;
        let mut charging = false;

        cx.shared.battery.lock(|battery| {
            voltage = battery.voltage();
            charging = battery.is_charging();
        });

        rprintln!(
            "Battery status: {} ({})",
            voltage,
            if charging { "charging" } else { "discharging" },
        );

        // Show battery status in top right corner
        let mut buf = [0u8; 6];
        (voltage / 10).numtoa(10, &mut buf[0..1]);
        buf[1] = b'.';
        (voltage % 10).numtoa(10, &mut buf[2..3]);
        buf[3] = b'V';
        buf[4] = b'/';
        buf[5] = if charging { b'C' } else { b'D' };
        let status = core::str::from_utf8(&buf).unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::WHITE)
            .background_color(BACKGROUND_COLOR)
            .build();

        let text = Text::new(
            status,
            Point::new(
                LCD_W as i32 - 60_i32 - MARGIN as i32,
                LCD_H as i32 - 10 - MARGIN as i32,
            ),
            text_style,
        );

        cx.shared.lcd.lock(|lcd| {
            text.draw(lcd).unwrap();
        });
    }
}
