//! Opt-in battery log inside InfiniTime's existing LittleFS partition.
//! Never formats or erases an unmountable filesystem.

use core::cell::RefCell;

use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex as BlockingMutex},
    mutex::Mutex as AsyncMutex,
};
use embedded_hal::spi::SpiDevice;
use littlefs2::{
    consts::{U16, U2},
    driver::Storage,
    fs::Filesystem,
    io::{Error, SeekFrom, Write},
};

use crate::{
    battery_history::{History, Sample},
    battery_record::{decode, encode, RECORD_SIZE},
    flash::{Flash, FS_SIZE, FS_START},
};

const PATH: &littlefs2::path::Path = littlefs2::path!("kongle-battery.bin");
const TEMP_PATH: &littlefs2::path::Path = littlefs2::path!("kongle-battery.tmp");
const MAX_FILE_SIZE: usize = 64 * 1024;
const KEEP_BYTES: usize = RECORD_SIZE * crate::battery_history::CAPACITY;

struct FsStorage<'a, S: SpiDevice<u8>> {
    flash: &'a mut Flash<S>,
}

impl<S: SpiDevice<u8>> Storage for FsStorage<'_, S> {
    const READ_SIZE: usize = 16;
    const WRITE_SIZE: usize = 8;
    const BLOCK_SIZE: usize = 4096;
    const BLOCK_COUNT: usize = FS_SIZE as usize / 4096;
    const BLOCK_CYCLES: isize = 1000;
    type CACHE_SIZE = U16;
    type LOOKAHEAD_SIZE = U2;

    fn read(&mut self, off: usize, buf: &mut [u8]) -> Result<usize, Error> {
        let address = FS_START.checked_add(off as u32).ok_or(Error::IO)?;
        self.flash.fs_read(address, buf).map_err(|_| Error::IO)?;
        Ok(buf.len())
    }

    fn write(&mut self, off: usize, data: &[u8]) -> Result<usize, Error> {
        let address = FS_START.checked_add(off as u32).ok_or(Error::IO)?;
        self.flash
            .fs_program(address, data)
            .map_err(|_| Error::IO)?;
        Ok(data.len())
    }

    fn erase(&mut self, off: usize, len: usize) -> Result<usize, Error> {
        if len != 4096 {
            return Err(Error::INVALID);
        }
        let address = FS_START.checked_add(off as u32).ok_or(Error::IO)?;
        self.flash.fs_erase(address).map_err(|_| Error::IO)?;
        Ok(len)
    }
}

pub async fn load<S: SpiDevice<u8>>(
    flash: &AsyncMutex<CriticalSectionRawMutex, Flash<S>>,
    history: &BlockingMutex<CriticalSectionRawMutex, RefCell<History>>,
) -> Result<(), Error> {
    let mut flash = flash.lock().await;
    flash.initialize().await.map_err(|_| Error::IO)?;
    let mut storage = FsStorage { flash: &mut flash };
    Filesystem::mount_and_then(&mut storage, |fs| {
        match fs.open_file_and_then(PATH, |file| {
            let len = file.len()?;
            let start = len.saturating_sub(KEEP_BYTES);
            let start = start + (RECORD_SIZE - start % RECORD_SIZE) % RECORD_SIZE;
            file.seek(SeekFrom::Start(start as u32))?;
            let mut bytes = [0u8; RECORD_SIZE];
            for _ in 0..crate::battery_history::CAPACITY {
                match file.read(&mut bytes)? {
                    RECORD_SIZE => {
                        if let Some(sample) = decode(&bytes) {
                            history.lock(|h| h.borrow_mut().restore(sample));
                        }
                    }
                    _ => break,
                }
            }
            Ok(())
        }) {
            Ok(()) | Err(Error::NO_SUCH_ENTRY) => Ok(()),
            Err(error) => Err(error),
        }
    })
}

pub async fn append<S: SpiDevice<u8>>(
    flash: &AsyncMutex<CriticalSectionRawMutex, Flash<S>>,
    sample: Sample,
) -> Result<(), Error> {
    if sample.epoch_secs == 0 {
        return Ok(());
    }
    let mut flash = flash.lock().await;
    let mut storage = FsStorage { flash: &mut flash };
    Filesystem::mount_and_then(&mut storage, |fs| {
        fs.open_file_with_options_and_then(
            |o| o.write(true).create(true).append(true),
            PATH,
            |file| file.write_all(&encode(sample)),
        )?;
        let len = fs.open_file_and_then(PATH, |file| file.len())?;
        if len > MAX_FILE_SIZE {
            let (tail, _) = fs.read_chunk::<KEEP_BYTES>(
                PATH,
                littlefs2::io::OpenSeekFrom::End(-(KEEP_BYTES as i32)),
            )?;
            fs.write(TEMP_PATH, &tail)?;
            fs.rename(TEMP_PATH, PATH)?;
        }
        Ok(())
    })
}
