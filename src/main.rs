// #![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]
// #![cfg_attr(not(test))]
// #![warn(rust_2018_idioms)]
// Panic handler
#[cfg(not(test))]
use panic_rtt_target as _;

use crate::monotonic_nrf52::{ExtU32, MonoTimer, Instant};

use hal::{
    gpio::{p0, Floating, Input, Level, Output, Pin, PushPull, PullUp},
    prelude::*,
    twim,
};
use nrf52832_hal as hal;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

use core::time::Duration;

// use cortex_m_semihosting::rprintln;

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

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::{
    geometry::Point,
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use st7789::{self, Orientation};
use cst816s::CST816S;

use lvgl::{
    self, Align, Color, Part, State, Widget, UI, Obj,
    style::Style,
    widgets::{
        Btn,
        Label
    },
    input_device::{
        InputData,
        Pointer,
    },
};

// use fugit::{Duration, Instant};
use cstr_core::CStr;
use debouncr::{debounce_6, Debouncer, Edge, Repeat6};
use numtoa::NumToA;

mod backlight;
mod battery;
mod delay;
mod monotonic_nrf52;

const LCD_W: u16 = 240;
const LCD_H: u16 = 240;
// const MARGIN: u16 = 10;
// const BACKGROUND_COLOR: Rgb565 = Rgb565::new(0, 0b000111, 0);

pub struct AppConfig {}

impl Config for AppConfig {
    type Timer = BleTimer<hal::pac::TIMER2>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<BatteryServiceAttrs, NoSecurity>;
    type PacketQueue = &'static mut SimpleQueue;
}

#[app(device = crate::hal::pac, peripherals = true, dispatchers = [SWI0_EGU0,SWI1_EGU1,SWI2_EGU2,SWI3_EGU3,SWI4_EGU4,SWI5_EGU5])]
mod app {

    use delay::TimerDelay;
    use st7789::ST7789;

    use super::*;

    #[shared]
    struct Shared {


        // ui: UI<ST7789<
        //     SPIInterfaceNoCS<
        //         hal::spim::Spim<hal::pac::SPIM1>,
        //         p0::P0_18<Output<PushPull>>,
        //     >,
        //     p0::P0_26<Output<PushPull>>,
        //     p0::P0_22<Output<PushPull>>>,
        //     Rgb565,
        // >,


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
        ui: UI<ST7789<
            SPIInterfaceNoCS<
                hal::spim::Spim<hal::pac::SPIM1>,
                p0::P0_18<Output<PushPull>>,
            >,
            p0::P0_26<Output<PushPull>>,
            p0::P0_22<Output<PushPull>>>,
            Rgb565,
        >,

        lcd_delay: TimerDelay,



        backlight: backlight::Backlight,

        touchpad: CST816S<twim::Twim<hal::pac::TWIM1>, Pin<Input<PullUp>>, Pin<Output<PushPull>>>,

        // lcd: st7789::ST7789<
        //     display_interface_spi::SPIInterfaceNoCS<
        //         hal::spim::Spim<hal::pac::SPIM1>,
        //         p0::P0_18<Output<PushPull>>,
        //     >,
        //     p0::P0_26<Output<PushPull>>,
        //     p0::P0_22<Output<PushPull>>,
        // >,

        // Button
        button: Pin<Input<Floating>>,
        button_debouncer: Debouncer<u8, Repeat6>,
    }

    #[monotonic(binds = TIMER1, default = true)]
    type Tonic = MonoTimer<hal::pac::TIMER1>;

    #[init(local = [
        ble_tx_buf: PacketBuffer = [0; MIN_PDU_BUF], 
        ble_rx_buf: PacketBuffer = [0; MIN_PDU_BUF],
        tx_queue: SimpleQueue = SimpleQueue::new(),
        rx_queue: SimpleQueue = SimpleQueue::new(),
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
            TWIM1,
            
            ..
        } = cx.device;

        // Desctructure local resources
        let init::LocalResources {
            ble_tx_buf,
            ble_rx_buf,
            tx_queue,
            rx_queue,
        } = cx.local;

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
        let mut radio = BleRadio::new(RADIO, &FICR, ble_tx_buf, ble_rx_buf);

        // Create bluetooth TX/RX queues
        let (tx, tx_cons) = tx_queue.split();
        let (rx_prod, rx) = rx_queue.split();

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

        lcd_delay.delay_us(1000u32);
        // internal i2c0 bus devices: BMA421 (accel), HRS3300 (hrs), CST816S (TouchPad)
        // BMA421-INT:  P0.08
        // TP-INT: P0.28
        let i2c0_pins = twim::Pins {
            scl: gpio.p0_07.into_floating_input().degrade(),
            sda: gpio.p0_06.into_floating_input().degrade(),
        };
        let i2c_port = twim::Twim::new(TWIM1, i2c0_pins, twim::Frequency::K400);
    
        // setup touchpad external interrupt pin: P0.28/AIN4 (TP_INT)
        let touch_int = gpio.p0_28.into_pullup_input().degrade();
        // setup touchpad reset pin: P0.10/NFC2 (TP_RESET)
        let touch_rst = gpio.p0_10.into_push_pull_output(Level::High).degrade();
    
        let mut touchpad = CST816S::new(i2c_port, touch_int, touch_rst);
        touchpad.setup(&mut lcd_delay).unwrap();

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


        // Initialize LVGL
        let mut ui = UI::init().unwrap();
        ui.disp_drv_register(lcd).unwrap();


        // Schedule tasks immediately
        display::spawn().unwrap();
        poll_button::spawn().unwrap();
        show_battery_status::spawn().unwrap();
        update_battery_status::spawn().unwrap();

        (
            Shared {
                // gpio,

                battery,
                // text_style,
                radio,
                ble_ll,
                ble_r,
            },
            Local {
                ui,
                lcd_delay,
                touchpad,

                backlight,

                button,
                button_debouncer: debounce_6(false),
            },
            init::Monotonics(mono),
        )
    }


    #[idle()]
    fn idle(cx: idle::Context) -> ! {

        loop {
            // Now Wait For Interrupt is used instead of a busy-wait loop
            // to allow MCU to sleep between interrupts
            // https://developer.arm.com/documentation/ddi0406/c/Application-Level-Architecture/Instruction-Details/Alphabetical-list-of-instructions/WFI
            rtic::export::wfi()
        }
    }

    /// Hook up the RADIO interrupt to the Rubble BLE stack.
    #[task(binds = RADIO, shared = [radio, ble_ll], priority = 3)]
    fn radio(cx: radio::Context) {
        //Destructure the shared resources
        let radio::SharedResources {
            mut radio,
            mut ble_ll,
        } = cx.shared;

        ble_ll.lock(|ble_ll| {
            radio.lock(|radio| {
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
    fn timer2(cx: timer2::Context) {
        //Destructure the shared resources
        let timer2::SharedResources {
            mut radio,
            mut ble_ll,
        } = cx.shared;

        ble_ll.lock(|ble_ll| {
            radio.lock(|radio| {
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
    fn ble_worker(cx: ble_worker::Context) {
        //Destructure the shared resources
        let ble_worker::SharedResources { mut ble_r } = cx.shared;

        // Fully drain the packet queue
        ble_r.lock(|ble_r| {
            while ble_r.has_work() {
                ble_r.process_one().unwrap();
            }
        })
    }
    
    #[task(shared = [],
        local = [ui, touchpad, lcd_delay], priority = 3)]
    fn display(cx: display::Context) {
        //Destructure the shared resources
        // let display::SharedResources {} = cx.shared;

        //Desctructure the local resources
        let display::LocalResources {
            ui,
            touchpad,
            lcd_delay,
        } = cx.local;


        // Define the initial state of the input
        let mut latest_touch_point = Point::new(0, 0);
        let mut latest_touch_status = InputData::Touch(latest_touch_point.clone())
            .released()
            .once();

        // Register a new input device that's capable of reading the current state of the input
        let mut touch_device = Pointer::new(|| latest_touch_status);
        ui.indev_drv_register(&mut touch_device).unwrap();

        // Create screen and widgets
        let mut screen = ui.scr_act().unwrap();

        // Draw a black background to the screen
        let mut screen_style = Style::default();
        screen_style.set_bg_color(State::DEFAULT, Color::from_rgb((0, 0, 0)));
        screen.add_style(Part::Main, screen_style).unwrap();

        //Create the button
        let text_click_me = CStr::from_bytes_with_nul("Click!\0".as_bytes()).unwrap();
        let mut touch_button = Btn::new(&mut screen).unwrap();
        touch_button
            .set_align(&mut screen, Align::InLeftMid, 30, 0)
            .unwrap();
        touch_button.set_size(180, 80).unwrap();
        let mut btn_lbl = Label::new(&mut touch_button).unwrap();
        btn_lbl.set_text(text_click_me).unwrap();

        // touch_button.on_event(|mut btn, event| {
        //     if let lvgl::Event::Clicked = event {
        //         btn.toggle().unwrap();
        //     }
        // })
        // .unwrap();

        let mut btn_state = false;

        touch_button.on_event(|mut btn, event| {
            if let lvgl::Event::Clicked = event {
                if btn_state {
                    let nt = CStr::from_bytes_with_nul("Click me!\0".as_bytes()).unwrap();
                    btn_lbl.set_text(nt).unwrap();
                } else {
                    let nt = CStr::from_bytes_with_nul("Click!\0".as_bytes()).unwrap();
                    btn_lbl.set_text(nt).unwrap();
                }
                btn_state = !btn_state;
                rprintln!("Clicked! Inner..");
                btn.toggle().unwrap();
            }
        }).unwrap();

        let mut time_style = Style::default();
        time_style.set_text_color(State::DEFAULT, Color::from_rgb((255, 255, 255)));


        let mut time_lbl = Label::new(&mut screen).unwrap();
        time_lbl
            .set_align(&mut screen, Align::OutTopMid, 0, -50)
            .unwrap();
        let time_text = CStr::from_bytes_with_nul("TIME\0".as_bytes()).unwrap();
        time_lbl.set_text(time_text).unwrap();
        time_lbl.add_style(Part::Main, time_style).unwrap();

        let mut loop_time = Instant::now();
        let mut total_time = Duration::from_secs(0);
        let mut time_text_buf = [0u8; 20];
    
        let mut last_update_time_secs = 0;
        let mut last_inc_time_ms = 0;

        loop {
            ui.task_handler();

            if let Some(evt) = touchpad.read_one_touch_event(true) {
                latest_touch_point = Point::new(evt.x, evt.y);
                // Pressed
                latest_touch_status = InputData::Touch(latest_touch_point.clone())
                    .pressed()
                    .once();
            } else {
                // Released
                latest_touch_status = InputData::Touch(latest_touch_point.clone())
                    .released()
                    .once();

                lcd_delay.delay_us(1u32);
            }

            total_time += Duration::from_micros(loop_time.elapsed().as_cycles() as u64);

            if total_time.as_secs() > last_update_time_secs {
                last_update_time_secs = total_time.as_secs();
    
                let text = (total_time.as_secs() as u32).numtoa(10, &mut time_text_buf);
    
                let time_text = unsafe { CStr::from_bytes_with_nul_unchecked(&text) };
                time_lbl.set_text(time_text).unwrap();
                time_lbl
                    .set_align(&mut touch_button, Align::OutTopMid, 0, -50)
                    .unwrap();
    
                // Reset buffer
                for p in time_text_buf.iter_mut() {
                    *p = '\0' as u8;
                }
            }
    
            if total_time.as_millis() > last_inc_time_ms {
                //let diff_ms = total_time.as_millis() - last_inc_time_ms;
                last_inc_time_ms = total_time.as_millis();
            }
            ui.tick_inc(Duration::from_millis(40));
    
    
            loop_time = Instant::now();
        }

        // Re-schedule the timer interrupt
        // display::spawn_after(40.millis()).unwrap();
    }

    #[task(local = [button, button_debouncer])]
    fn poll_button(cx: poll_button::Context) {
        //Destructure the local resources
        let poll_button::LocalResources {
            button,
            button_debouncer,
        } = cx.local;

        // Poll button
        let pressed = button.is_high().unwrap();
        let edge = button_debouncer.update(pressed);

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
        //Destructure the local resources
        let button_pressed::LocalResources { backlight } = cx.local;

        if backlight.get_brightness() < 7 {
            backlight.brighter();
        } else {
            backlight.off();
        }
    }

    /// Fetch the battery status from the hardware. Update the text if
    /// something changed.
    #[task(shared = [battery])]
    fn update_battery_status(cx: update_battery_status::Context) {
        //Destructure the shared resources
        let update_battery_status::SharedResources { mut battery } = cx.shared;

        rprintln!("Update battery status");

        let changed = battery.lock(|battery| battery.update());
        if changed {
            rprintln!("Battery status changed");
            show_battery_status::spawn().unwrap();
        }

        // Re-schedule the timer interrupt in 1s
        update_battery_status::spawn_after(1000.millis()).unwrap();
    }

    /// Show the battery status on the LCD.
    #[task(shared = [battery])]
    fn show_battery_status(cx: show_battery_status::Context) {
        // //Destructure the shared resources
        // let show_battery_status::SharedResources {
        //     mut battery,
        //     mut ui,
        // } = cx.shared;

        // let mut voltage = 0;
        // let mut charging = false;

        // battery.lock(|battery| {
        //     voltage = battery.voltage();
        //     charging = battery.is_charging();
        // });

        // rprintln!(
        //     "Battery status: {} ({})",
        //     voltage,
        //     if charging { "charging" } else { "discharging" },
        // );

        // // Show battery status in top right corner
        // let mut buf = [0u8; 6];
        // (voltage / 10).numtoa(10, &mut buf[0..1]);
        // buf[1] = b'.';
        // (voltage % 10).numtoa(10, &mut buf[2..3]);
        // buf[3] = b'V';
        // buf[4] = b'/';
        // buf[5] = if charging { b'C' } else { b'D' };
        // let status = core::str::from_utf8(&buf).unwrap();

        // let text_style = MonoTextStyleBuilder::new()
        //     .font(&FONT_10X20)
        //     .text_color(Rgb565::WHITE)
        //     .background_color(BACKGROUND_COLOR)
        //     .build();

        // let text = Text::new(
        //     status,
        //     Point::new(
        //         LCD_W as i32 - 60 as i32 - MARGIN as i32,
        //         LCD_H as i32 - 10 - MARGIN as i32,
        //     ),
        //     text_style,
        // );

        // lcd.lock(|lcd| {
        //     text.draw(lcd).unwrap();
        // });
    }

}
