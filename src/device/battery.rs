use crate::battery_estimate::percent_from_mv;
use defmt::Format;
use embassy_nrf::gpio::Input;
use embassy_nrf::saadc::Saadc;

/// Battery monitoring for PineTime (simplified)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BatteryState {
    pub mv: u16,
    pub percent: u8,
    pub charging: bool,
}

pub struct Battery<'d> {
    saadc: Saadc<'d, 1>,
    /// Charging detection pin (low = charging, high = discharging)
    charge_pin: Input<'d>,
    state: BatteryState,
}

impl<'d> Battery<'d> {
    /// Initialize battery monitoring
    pub fn new(saadc: Saadc<'d, 1>, charge_pin: Input<'d>) -> Self {
        Self {
            saadc,
            charge_pin,
            state: BatteryState {
                mv: 4200,
                percent: 100,
                charging: false,
            },
        }
    }

    /// Read voltage and charging state, then estimate the remaining charge.
    pub async fn update(&mut self) -> BatteryState {
        let mut buf = [0i16; 1];
        self.saadc.sample(&mut buf).await;
        let mv = adc_to_mv(buf[0]);
        let charging = self.charge_pin.is_low();
        let percent = percent_from_mv(mv);

        self.state = BatteryState {
            mv,
            percent,
            charging,
        };
        self.state
    }

    pub async fn calibrate(&mut self) {
        self.saadc.calibrate().await;
    }
}

impl Format for Battery<'_> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Battery: {}mV ({}%)", self.state.mv, self.state.percent)
    }
}

fn adc_to_mv(raw: i16) -> u16 {
    let raw = raw.max(0) as i32;
    // SAADC defaults to 12-bit resolution (0..=4095). The input divider halves
    // the battery voltage, so double the measured pin voltage afterwards.
    let adc_mv = raw * 3600 / 4096;
    let battery_mv = adc_mv * 2;
    battery_mv as u16
}
