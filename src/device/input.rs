use debouncr::{debounce_6, Debouncer, Edge, Repeat6};
use defmt::Format;
use embassy_nrf::{
    gpio::{Input, Output},
    gpiote::InputChannel,
    twim::{self, Twim},
};
use embassy_time::{Duration, Timer};

use crate::touch::{self, TouchEvent};

/// Button input events
#[derive(Clone, Copy, Debug, PartialEq, Format)]
pub enum InputEvent {
    ButtonPressed,
    ButtonReleased,
    ButtonHeld,
    TouchWake,
    Touch(TouchEvent),
}

/// Button handler for PineTime (simplified)
pub struct Button<'d> {
    button_pin: Input<'d>,
    _enable: Output<'d>,
    debouncer: Debouncer<u8, Repeat6>,
}

impl<'d> Button<'d> {
    /// Create new button handler
    pub fn new(button_pin: Input<'d>, enable: Output<'d>) -> Self {
        Self {
            button_pin,
            _enable: enable,
            debouncer: debounce_6(false),
        }
    }

    /// Poll button and return events
    pub fn poll(&mut self) -> Option<InputEvent> {
        // P0.15 supplies the button; P0.13 is active-high with a pull-down.
        let pressed = self.is_pressed();
        let edge = self.debouncer.update(pressed);

        match edge {
            Some(Edge::Rising) => Some(InputEvent::ButtonPressed),
            Some(Edge::Falling) => Some(InputEvent::ButtonReleased),
            None => None,
        }
    }

    /// Check if button is currently pressed
    pub fn is_pressed(&self) -> bool {
        self.button_pin.is_high()
    }
}

/// PineTime CST816S on TWIM1. IRQ is active low; register 0x01 reports the
/// gesture and registers 0x03..0x06 report its position.
pub struct TouchScreen<'d> {
    twim: Twim<'d>,
    irq: InputChannel<'d>,
    reset: Output<'d>,
}

impl<'d> TouchScreen<'d> {
    const ADDRESS: u8 = 0x15;

    pub fn new(twim: Twim<'d>, irq: InputChannel<'d>, reset: Output<'d>) -> Self {
        Self { twim, irq, reset }
    }

    pub async fn init(&mut self) -> Result<u8, twim::Error> {
        self.reset.set_low();
        Timer::after(Duration::from_millis(5)).await;
        self.reset.set_high();
        Timer::after(Duration::from_millis(50)).await;

        // InfiniTime issues a dummy read before the chip ID read. Some
        // controllers report an unreliable ID even though touch works.
        let mut dummy = [0];
        let _ = self
            .twim
            .write_read(Self::ADDRESS, &[0x15], &mut dummy)
            .await;
        Timer::after(Duration::from_millis(5)).await;
        let mut chip_id = [0xff];
        let _ = self
            .twim
            .write_read(Self::ADDRESS, &[0xa7], &mut chip_id)
            .await;
        Timer::after(Duration::from_millis(5)).await;
        // Follow InfiniTime's continuous horizontal-motion and double-tap
        // setup. The controller still reports all four swipe codes.
        self.twim.write(Self::ADDRESS, &[0xec, 0x05]).await?;
        self.twim.write(Self::ADDRESS, &[0xfa, 0x70]).await?;
        self.twim.write(Self::ADDRESS, &[0xfb, 0x00]).await?;
        #[cfg(feature = "touch-wake-debug")]
        self.twim.write(Self::ADDRESS, &[0xfe, 0x01]).await?;
        Ok(chip_id[0])
    }

    /// Stop touch scanning while the display sleeps in normal builds.
    pub async fn sleep(&mut self) -> Result<(), twim::Error> {
        self.twim.write(Self::ADDRESS, &[0xa5, 0x03]).await
    }

    pub async fn next(&mut self) -> Result<Option<TouchEvent>, twim::Error> {
        self.irq.wait().await;
        let register = [0x01];
        let mut frame = [0; 6];
        self.twim
            .write_read(Self::ADDRESS, &register, &mut frame)
            .await?;
        Ok(touch::decode(frame))
    }
}

impl Format for Button<'_> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Button: {}", self.is_pressed())
    }
}
