//! NVMC access is confined to the primary image_ok word of the pinned bootloader.
use crate::{
    firmware_state::{self, BootState},
    ota,
};
use embassy_nrf::nvmc::Nvmc;
use embedded_storage::nor_flash::NorFlash;

fn snapshot() -> (u32, u32, [u8; 16]) {
    // Fixed, aligned internal-flash addresses. OTA writes only external flash.
    unsafe {
        let copy_done = core::ptr::read_volatile(ota::PRIMARY_COPY_DONE_ADDRESS as *const u32);
        let image_ok = core::ptr::read_volatile(ota::PRIMARY_IMAGE_OK_ADDRESS as *const u32);
        let magic = core::array::from_fn(|i| {
            core::ptr::read_volatile((ota::PRIMARY_MAGIC_ADDRESS + i) as *const u8)
        });
        (copy_done, image_ok, magic)
    }
}

pub fn state() -> BootState {
    let (copy_done, image_ok, magic) = snapshot();
    firmware_state::classify(copy_done, image_ok, &magic)
}

pub fn confirm(nvmc: &mut Nvmc<'_>) -> Result<BootState, ()> {
    let (copy_done, image_ok, magic) = snapshot();
    if firmware_state::classify(copy_done, image_ok, &magic) == BootState::Confirmed {
        return Ok(BootState::Confirmed);
    }
    let word = firmware_state::confirmation_word(copy_done, image_ok, &magic).ok_or(())?;
    nvmc.write(ota::PRIMARY_IMAGE_OK_ADDRESS as u32, &word)
        .map_err(|_| ())?;
    let readback = state();
    if readback == BootState::Confirmed {
        Ok(readback)
    } else {
        Err(())
    }
}
