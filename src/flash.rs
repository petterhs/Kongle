//! PineTime external SPI NOR flash access, restricted to MCUBoot's secondary slot.

use embassy_time::{Duration, Instant, Timer};
use embedded_hal::spi::{Operation, SpiDevice};

use crate::ota::{
    erase_sector_address, image_range_ok, SECONDARY_SECTORS, SECONDARY_SLOT_SIZE,
    SECONDARY_SLOT_START, TRAILER_MAGIC,
};

const PAGE_SIZE: u32 = 256;
const SLOT_END: u32 = SECONDARY_SLOT_START + SECONDARY_SLOT_SIZE;
const IMAGE_END: u32 = SLOT_END - TRAILER_MAGIC.len() as u32;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum FlashError {
    Bus,
    UnsupportedChip,
    OutOfBounds,
    TimedOut,
    WriteNotEnabled,
    Verification,
}

pub struct Flash<SPI> {
    spi: SPI,
}

impl<SPI: SpiDevice<u8>> Flash<SPI> {
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    fn command(&mut self, bytes: &[u8]) -> Result<(), FlashError> {
        self.spi.write(bytes).map_err(|_| FlashError::Bus)
    }

    fn command_read(&mut self, command: &[u8], data: &mut [u8]) -> Result<(), FlashError> {
        self.spi
            .transaction(&mut [Operation::Write(command), Operation::Read(data)])
            .map_err(|_| FlashError::Bus)
    }

    fn address_command(opcode: u8, address: u32) -> [u8; 4] {
        [
            opcode,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
        ]
    }

    fn check_range(address: u32, len: usize, allow_trailer: bool) -> Result<(), FlashError> {
        let end = address
            .checked_add(u32::try_from(len).map_err(|_| FlashError::OutOfBounds)?)
            .ok_or(FlashError::OutOfBounds)?;
        if address < SECONDARY_SLOT_START || end > if allow_trailer { SLOT_END } else { IMAGE_END }
        {
            return Err(FlashError::OutOfBounds);
        }
        Ok(())
    }

    pub async fn initialize(&mut self) -> Result<[u8; 3], FlashError> {
        self.command(&[0xab])?; // release from deep power-down
        Timer::after(Duration::from_millis(1)).await;
        let mut id = [0; 3];
        self.command_read(&[0x9f], &mut id)?;
        // Accept reported JEDEC IDs for PineTime's 4 MiB flash variants;
        // check the specific board against its bootloader before activation.
        if !matches!(id[0], 0x0b | 0x16 | 0x68) || id[1] != 0x40 || id[2] != 0x16 {
            return Err(FlashError::UnsupportedChip);
        }
        Ok(id)
    }

    fn status(&mut self) -> Result<u8, FlashError> {
        let mut status = [0];
        self.command_read(&[0x05], &mut status)?;
        Ok(status[0])
    }

    async fn wait_ready(&mut self, timeout: Duration) -> Result<(), FlashError> {
        let started = Instant::now();
        loop {
            if self.status()? & 1 == 0 {
                return Ok(());
            }
            if started.elapsed() >= timeout {
                return Err(FlashError::TimedOut);
            }
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    fn write_enable(&mut self) -> Result<(), FlashError> {
        self.command(&[0x06])?;
        if self.status()? & 2 == 0 {
            return Err(FlashError::WriteNotEnabled);
        }
        Ok(())
    }

    pub async fn erase_secondary(&mut self) -> Result<(), FlashError> {
        // An old pending-image marker may still be present. Remove its sector
        // first so interruption cannot leave magic pointing to a partial image.
        let trailer_sector = SLOT_END - 4096;
        for index in 0..SECONDARY_SECTORS {
            let address = erase_sector_address(index).ok_or(FlashError::OutOfBounds)?;
            self.write_enable()?;
            self.command(&Self::address_command(0x20, address))?;
            self.wait_ready(Duration::from_secs(3)).await?;
            if address == trailer_sector {
                let mut magic = [0; 16];
                self.read(SLOT_END - 16, &mut magic)?;
                if magic != [0xff; 16] {
                    return Err(FlashError::Verification);
                }
            }
        }
        Ok(())
    }

    pub fn read(&mut self, address: u32, data: &mut [u8]) -> Result<(), FlashError> {
        Self::check_range(address, data.len(), true)?;
        self.command_read(&Self::address_command(0x03, address), data)
    }

    pub async fn write_image(&mut self, address: u32, data: &[u8]) -> Result<(), FlashError> {
        if !image_range_ok(address, data.len()) {
            return Err(FlashError::OutOfBounds);
        }
        let mut offset = 0;
        while offset < data.len() {
            let at = address + offset as u32;
            let page_remaining = (PAGE_SIZE - at % PAGE_SIZE) as usize;
            let size = page_remaining.min(data.len() - offset);
            self.write_enable()?;
            self.spi
                .transaction(&mut [
                    Operation::Write(&Self::address_command(0x02, at)),
                    Operation::Write(&data[offset..offset + size]),
                ])
                .map_err(|_| FlashError::Bus)?;
            self.wait_ready(Duration::from_millis(100)).await?;
            let mut readback = [0; PAGE_SIZE as usize];
            self.read(at, &mut readback[..size])?;
            if readback[..size] != data[offset..offset + size] {
                return Err(FlashError::Verification);
            }
            offset += size;
        }
        Ok(())
    }
}
