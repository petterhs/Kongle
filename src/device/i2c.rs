//! Shared asynchronous bus for touch, motion and pulse sensors.
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::{
    peripherals,
    twim::{Config, Twim},
    Peri,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

pub type Bus = Mutex<CriticalSectionRawMutex, Twim<'static>>;
pub type Device<'a> = I2cDevice<'a, CriticalSectionRawMutex, Twim<'static>>;
pub type Error = embassy_embedded_hal::shared_bus::I2cDeviceError<embassy_nrf::twim::Error>;

/// HR uses finite transfers rather than cancelling an in-flight EasyDMA
/// future on timeout. This holds the shared lock only for one transaction;
/// the normal eight-byte sample takes about a millisecond at 100 kHz.
pub struct BoundedDevice<'a> {
    bus: &'a Bus,
}

impl<'a> BoundedDevice<'a> {
    pub fn new(bus: &'a Bus) -> Self {
        Self { bus }
    }
}

impl embedded_hal::i2c::ErrorType for BoundedDevice<'_> {
    type Error = embassy_nrf::twim::Error;
}

impl embedded_hal_async::i2c::I2c for BoundedDevice<'_> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut bus = self.bus.lock().await;
        let result =
            bus.blocking_transaction_timeout(address, operations, Duration::from_millis(10));
        if result == Err(embassy_nrf::twim::Error::Timeout) {
            // We exclusively own TWIM1 through the mutex. Disable DMA before
            // the borrowed sample buffers can leave scope, then recover the
            // controller. Never steal peripherals or reset the touch chip.
            let regs = embassy_nrf::pac::TWIM1;
            use embassy_nrf::pac::twim::vals::Enable;
            regs.enable().write(|w| w.set_enable(Enable::DISABLED));
            cortex_m::asm::dsb();
            regs.intenclr().write(|w| w.0 = 0xffff_ffff);
            regs.events_stopped().write_value(0);
            regs.events_suspended().write_value(0);
            regs.events_error().write_value(0);
            regs.enable().write(|w| w.set_enable(Enable::ENABLED));
        }
        result
    }
}

fn config() -> Config {
    let mut config = Config::default();
    config.sda_pullup = true;
    config.scl_pullup = true;
    config
}

/// Reset motion before publishing the shared bus to touch/other tasks.
/// Reborrowing and dropping the temporary TWIM avoids stealing peripherals or
/// changing live HAL registers behind another client's back.
pub async fn prepare(
    mut peripheral: Peri<'static, peripherals::TWISPI1>,
    mut sda: Peri<'static, peripherals::P0_06>,
    mut scl: Peri<'static, peripherals::P0_07>,
    buffer: &'static mut [u8],
) -> (Twim<'static>, bool) {
    // Keep DMA buffers alive until the temporary driver has been disabled,
    // including when the reset command makes the peripheral time out.
    let mut chip_id = [0u8];
    let reset_command = [0x7e, 0xb6];
    let known;
    {
        let mut temporary = Twim::new(
            peripheral.reborrow(),
            crate::Irqs,
            sda.reborrow(),
            scl.reborrow(),
            config(),
            &mut *buffer,
        );
        known = match temporary.blocking_write_read_timeout(
            0x18,
            &[0x00],
            &mut chip_id,
            Duration::from_millis(20),
        ) {
            Ok(()) if crate::motion::Variant::from_id(chip_id[0]).is_some() => {
                defmt::info!(
                    "[motion] chip ID {:02x}; resetting before configuration",
                    chip_id[0]
                );
                if let Err(error) = temporary.blocking_write_timeout(
                    0x18,
                    &reset_command,
                    Duration::from_millis(20),
                ) {
                    // A reset can interrupt the transaction. Recreate TWIM and
                    // let Motion::initialize verify that ASIC state is fresh.
                    defmt::warn!("[motion] reset transaction: {:?}; recovering TWIM", error);
                }
                true
            }
            Ok(()) => {
                defmt::warn!(
                    "[motion] unknown chip ID {:02x}; no reset or configuration",
                    chip_id[0]
                );
                false
            }
            Err(error) => {
                defmt::warn!(
                    "[motion] startup probe failed: {:?}; skipping motion",
                    error
                );
                false
            }
        };
        drop(temporary);
    }
    if known {
        Timer::after(Duration::from_millis(200)).await;
    }
    (
        Twim::new(peripheral, crate::Irqs, sda, scl, config(), buffer),
        known,
    )
}
