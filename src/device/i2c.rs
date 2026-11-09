//! Shared asynchronous bus for touch, motion and pulse sensors.
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::twim::Twim;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

pub type Bus = Mutex<CriticalSectionRawMutex, Twim<'static>>;
pub type Device<'a> = I2cDevice<'a, CriticalSectionRawMutex, Twim<'static>>;
pub type Error = embassy_embedded_hal::shared_bus::I2cDeviceError<embassy_nrf::twim::Error>;
