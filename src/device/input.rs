use super::i2c::{Device, Error};
use defmt::Format;
use embassy_nrf::gpio::{Input, Output};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

use crate::touch::{self, TouchEvent};

/// Button input events
#[derive(Clone, Copy, Debug, PartialEq, Format)]
pub enum InputEvent {
    ButtonPressed,
    ButtonReleased,
    ButtonSingleConfirmed,
    ButtonDoublePressed,
    ButtonHeld,
    TouchWake,
    Touch(TouchEvent),
}

/// Button input for PineTime.
pub struct Button<'d> {
    button_pin: Input<'d>,
    _enable: Output<'d>,
}

impl<'d> Button<'d> {
    /// Create new button handler
    pub fn new(button_pin: Input<'d>, enable: Output<'d>) -> Self {
        Self {
            button_pin,
            _enable: enable,
        }
    }

    /// Wait for the active-high button rather than waking the CPU to poll it.
    pub async fn wait_for_press(&mut self) {
        self.button_pin.wait_for_high().await;
    }

    pub async fn wait_for_release(&mut self) {
        self.button_pin.wait_for_low().await;
    }

    /// Check if button is currently pressed
    pub fn is_pressed(&self) -> bool {
        // P0.15 supplies the button; P0.13 is active-high with a pull-down.
        self.button_pin.is_high()
    }
}

/// PineTime CST816S on TWIM1. IRQ is active low; register 0x01 reports the
/// gesture and registers 0x03..0x06 report its position.
pub struct TouchScreen<'d> {
    twim: Device<'d>,
    irq: Input<'d>,
    reset: Output<'d>,
}

impl<'d> TouchScreen<'d> {
    const ADDRESS: u8 = 0x15;

    pub fn new(twim: Device<'d>, irq: Input<'d>, reset: Output<'d>) -> Self {
        Self { twim, irq, reset }
    }

    pub async fn init(&mut self) -> Result<u8, Error> {
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
    pub async fn sleep(&mut self) -> Result<(), Error> {
        self.twim.write(Self::ADDRESS, &[0xa5, 0x03]).await
    }

    pub async fn next(&mut self) -> Result<Option<TouchEvent>, Error> {
        // Use the pin's level-sensitive PORT event. The dedicated GPIOTE
        // channel wait path panicked after a gesture on the development watch.
        // The CST816S can hold IRQ low after a read. Require it to return high
        // before accepting another low; otherwise one swipe is read repeatedly.
        self.irq.wait_for_high().await;
        self.irq.wait_for_low().await;
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
