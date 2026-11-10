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
