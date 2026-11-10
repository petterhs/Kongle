//! Asynchronous HRS3300 register transport. Register fields follow the TYHX
//! datasheet on files.pine64.org. No InfiniTime/wasp-os DSP code is incorporated.
use embedded_hal_async::i2c::I2c;

const ADDRESS: u8 = 0x44;
const ENABLE_OFF: u8 = 0x50; // 50 ms wait, HEN=0, 12.5 mA drive selection.
const DRIVER_ON: u8 = 0x2f; // Oscillator on; PineTime's steady-drive low nibble.

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    WrongId(u8),
    Readback {
        register: u8,
        expected: u8,
        actual: u8,
    },
    NotRunning,
    InvalidGain(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sample {
    pub ppg: u32,
    pub ambient: u32,
}

pub struct Hrs3300<I> {
    bus: I,
    recognized: bool,
    running: bool,
}

impl<I: I2c> Hrs3300<I> {
    pub fn new(bus: I) -> Self {
        Self {
            bus,
            recognized: false,
            running: false,
        }
    }

    async fn byte(&mut self, register: u8) -> Result<u8, Error<I::Error>> {
        let mut value = [0];
        self.bus
            .write_read(ADDRESS, &[register], &mut value)
            .await
            .map_err(Error::Bus)?;
        Ok(value[0])
    }

    async fn write(&mut self, register: u8, value: u8) -> Result<(), Error<I::Error>> {
        self.bus
            .write(ADDRESS, &[register, value])
            .await
            .map_err(Error::Bus)
    }

    async fn check(&mut self, register: u8, mask: u8, expected: u8) -> Result<(), Error<I::Error>> {
        let actual = self.byte(register).await?;
        if actual & mask != expected {
            return Err(Error::Readback {
                register,
                expected,
                actual,
            });
        }
        Ok(())
    }

    pub async fn probe(&mut self) -> Result<(), Error<I::Error>> {
        self.recognized = false;
        self.running = false;
        let id = self.byte(0).await?;
        if id != 0x21 {
            return Err(Error::WrongId(id));
        }
        self.recognized = true;
        self.stop().await
    }

    pub async fn start(&mut self) -> Result<(), Error<I::Error>> {
        if !self.recognized {
            self.probe().await?;
        }
        self.running = false;
        self.write(0x01, ENABLE_OFF).await?;
        self.write(0x16, 0x77).await?; // PineTime reference: 15-bit conversion.
        self.set_gain(1).await?; // Fixed reference gain; no automatic escalation.
        self.write(0x0c, DRIVER_ON).await?;
        self.write(0x01, ENABLE_OFF | 0x80).await?;
        self.check(0x01, 0xf8, ENABLE_OFF | 0x80).await?;
        self.check(0x0c, 0x60, 0x20).await?;
        self.check(0x16, 0x0f, 0x07).await?; // Do not require reserved bits to read back.
        self.check(0x17, 0x1c, 0).await?;
        self.running = true;
        Ok(())
    }

    /// Only the modest documented gains are used; x64 is deliberately absent.
    /// This changes analog gain, not LED current or ADC resolution.
    pub async fn set_gain(&mut self, multiplier: u8) -> Result<(), Error<I::Error>> {
        let bits = match multiplier {
            1 => 0,
            2 => 4,
            4 => 8,
            8 => 12,
            _ => return Err(Error::InvalidGain(multiplier)),
        };
        if !self.recognized {
            return Err(Error::NotRunning);
        }
        self.write(0x17, bits).await?;
        self.check(0x17, 0x1c, bits).await
    }

    pub async fn sample(&mut self) -> Result<Sample, Error<I::Error>> {
        if !self.running {
            return Err(Error::NotRunning);
        }
        let mut frame = [0; 8];
        self.bus
            .write_read(ADDRESS, &[0x08], &mut frame)
            .await
            .map_err(Error::Bus)?;
        Ok(decode(frame))
    }

    pub async fn stop(&mut self) -> Result<(), Error<I::Error>> {
        self.running = false;
        if !self.recognized {
            return Ok(());
        }
        // Attempt both even if the first fails. Never make shutdown depend on
        // a preceding read; PDRIVER=0 also shuts down the oscillator/LED drive.
        let driver = self.write(0x0c, 0).await;
        let engine = self.write(0x01, ENABLE_OFF).await;
        driver.and(engine)?;
        self.check(0x0c, 0x60, 0).await?;
        self.check(0x01, 0x80, 0).await
    }
}

pub fn decode(frame: [u8; 8]) -> Sample {
    Sample {
        ppg: (u32::from(frame[1]) << 8)
            | (u32::from(frame[2] & 15) << 4)
            | u32::from(frame[7] & 15)
            | (u32::from(frame[7] & 0x30) << 12),
        ambient: (u32::from(frame[5] & 0x7f) << 11)
            | (u32::from(frame[0]) << 3)
            | u32::from(frame[6] & 7),
    }
}
