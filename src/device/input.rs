use debouncr::{debounce_6, Debouncer, Edge, Repeat6};
use defmt::Format;
use embassy_nrf::gpio::{Input, Output};
use embassy_time::{Duration, Timer};

/// Button input events
#[derive(Clone, Copy, Debug, PartialEq, Format)]
pub enum InputEvent {
    ButtonPressed,
    ButtonReleased,
    TouchDetected,
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

/// Touch screen handler placeholder for future implementation
pub struct TouchScreen {
    // Future implementation with CST816S driver
}

impl TouchScreen {
    pub fn new() -> Self {
        Self {}
    }

    /// Initialize touch screen (placeholder)
    pub async fn init(&mut self) {
        // Future: Initialize CST816S touch controller
    }

    /// Check for touch events (placeholder)
    pub async fn poll(&mut self) -> Option<InputEvent> {
        // Future: Implement touch polling
        Timer::after(Duration::from_millis(10)).await;
        None
    }
}

impl Format for Button<'_> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Button: {}", self.is_pressed())
    }
}
