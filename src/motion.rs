//! Minimal asynchronous Bosch step-engine transport.
//!
//! Register protocol follows Bosch's BSD-3-Clause BMA4/BMA423 API; see
//! device/bosch/README.md and LICENSE for source revision and blob provenance.
//! Initialization is not yet enabled in the firmware: first capture the chip ID
//! with the read-only probe and validate the shared bus on hardware.
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

const ADDRESS: u8 = 0x18;
const CHUNK: usize = 16;
const FEATURE_LEN: usize = 70;
const CONFIG_421: &[u8; 6144] = include_bytes!("device/bosch/bma423.bin");
const CONFIG_425: &[u8; 6144] = include_bytes!("device/bosch/bma425.bin");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Variant {
    Bma421,
    Bma425,
}

impl Variant {
    pub fn from_id(id: u8) -> Option<Self> {
        match id {
            0x11 => Some(Self::Bma421),
            0x13 => Some(Self::Bma425),
            _ => None,
        }
    }

    fn config(self) -> &'static [u8; 6144] {
        match self {
            Self::Bma421 => CONFIG_421,
            Self::Bma425 => CONFIG_425,
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    UnknownChip(u8),
    InitializationStatus(u8),
    Readback,
    NotInitialized,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Configuration {
    pub variant: Variant,
    pub id: u16,
    pub accel: u8,
    pub range: u8,
}

pub struct Motion<I, D> {
    bus: I,
    delay: D,
    ready: bool,
    identified: bool,
}

impl<I: I2c, D: DelayNs> Motion<I, D> {
    pub fn new(bus: I, delay: D) -> Self {
        Self {
            bus,
            delay,
            ready: false,
            identified: false,
        }
    }

    async fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), Error<I::Error>> {
        self.bus
            .write_read(ADDRESS, &[register], data)
            .await
            .map_err(Error::Bus)
    }

    async fn byte(&mut self, register: u8) -> Result<u8, Error<I::Error>> {
        let mut data = [0];
        self.read(register, &mut data).await?;
        Ok(data[0])
    }

    async fn write(&mut self, register: u8, data: &[u8]) -> Result<(), Error<I::Error>> {
        // All callers use <= CHUNK bytes; stack data also avoids EasyDMA flash
        // pointers. The shared TWIM transmit buffer must hold CHUNK + 1 bytes.
        let mut packet = [0; CHUNK + 1];
        packet[0] = register;
        packet[1..1 + data.len()].copy_from_slice(data);
        self.bus
            .write(ADDRESS, &packet[..1 + data.len()])
            .await
            .map_err(Error::Bus)?;
        // Bosch specifies 450 us after writes in suspend/low-power mode.
        self.delay.delay_us(450).await;
        Ok(())
    }

    async fn pointer(&mut self, words: u16) -> Result<(), Error<I::Error>> {
        self.write(0x5b, &[(words & 0x0f) as u8]).await?;
        self.write(0x5c, &[(words >> 4) as u8]).await
    }

    /// Baseline: 100 Hz, ±2g, average 4, CIC mode; not a tuned accuracy claim.
    /// Caller must bound the whole operation with a timeout and keep sensor
    /// resets separate from touch. No soft-reset command is sent here.
    pub async fn initialize(&mut self) -> Result<Configuration, Error<I::Error>> {
        self.ready = false;
        self.identified = false;
        let id = self.byte(0x00).await?;
        let variant = Variant::from_id(id).ok_or(Error::UnknownChip(id))?;
        self.identified = true;
        // Preserve unrelated power bits and disable acceleration during upload.
        let power = self.byte(0x7d).await? & !0x04;
        self.write(0x7d, &[power]).await?;
        let power_conf = self.byte(0x7c).await?;
        self.write(0x7c, &[power_conf & !0x01]).await?;
        self.write(0x59, &[0]).await?;
        for (index, chunk) in variant.config().chunks_exact(CHUNK).enumerate() {
            self.pointer((index * CHUNK / 2) as u16).await?;
            self.write(0x5e, chunk).await?;
        }
        self.write(0x59, &[1]).await?;
        self.delay.delay_ms(150).await;
        let status = self.byte(0x2a).await? & 0x0f;
        if status != 1 {
            return Err(Error::InitializationStatus(status));
        }
        // ASIC publishes its feature start pointer after configuration load.
        let low = self.byte(0x5b).await? & 0x0f;
        let high = self.byte(0x5c).await?;
        let start = u16::from(low) | (u16::from(high) << 4);
        if start + (FEATURE_LEN / 2) as u16 > 0x1000 {
            return Err(Error::Readback);
        }
        let mut features = [0; FEATURE_LEN];
        for (index, chunk) in features.chunks_mut(CHUNK).enumerate() {
            self.pointer(start + (index * CHUNK / 2) as u16).await?;
            self.read(0x5e, chunk).await?;
        }
        let config_id = u16::from_le_bytes([features[0x42], features[0x43]]);
        // Enable cumulative counting, disable per-step detection and activity
        // classification. Retain the firmware's opaque tuning parameters.
        features[0x3b] = (features[0x3b] | 0x10) & !(0x08 | 0x20);
        for (index, chunk) in features.chunks(CHUNK).enumerate() {
            self.pointer(start + (index * CHUNK / 2) as u16).await?;
            self.write(0x5e, chunk).await?;
        }
        self.pointer(start + 0x3a / 2).await?;
        let mut enabled = [0; 2];
        self.read(0x5e, &mut enabled).await?;
        if enabled != features[0x3a..0x3c] {
            return Err(Error::Readback);
        }
        self.pointer(start).await?;
        self.write(0x40, &[0x28]).await?;
        self.write(0x41, &[0]).await?;
        self.write(0x7c, &[power_conf | 0x01]).await?;
        self.write(0x7d, &[power | 0x04]).await?;
        if self.byte(0x40).await? != 0x28
            || self.byte(0x41).await? & 3 != 0
            || self.byte(0x7d).await? & 4 == 0
            || self.byte(0x7c).await? & 1 == 0
        {
            return Err(Error::Readback);
        }
        self.ready = true;
        Ok(Configuration {
            variant,
            id: config_id,
            accel: 0x28,
            range: 0,
        })
    }

    pub async fn counter(&mut self) -> Result<u32, Error<I::Error>> {
        if !self.ready {
            return Err(Error::NotInitialized);
        }
        let mut data = [0; 4];
        self.read(0x1e, &mut data).await?;
        Ok(u32::from_le_bytes(data))
    }

    /// Best-effort shutdown after init/read failure; errors remain visible.
    pub async fn stop(&mut self) -> Result<(), Error<I::Error>> {
        self.ready = false;
        if !self.identified {
            return Ok(());
        }
        let power = self.byte(0x7d).await?;
        self.write(0x7d, &[power & !0x04]).await
    }
}
