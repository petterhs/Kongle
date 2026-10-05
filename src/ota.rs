//! Hardware-independent OTA status and MCUBoot secondary-slot layout.
//!
//! The receiver and external-flash driver stage images for a trial boot.

/// External SPI flash region used by the PineTime bootloader for the secondary image.
pub const SECONDARY_SLOT_START: u32 = 0x0004_0000;
pub const SECONDARY_SLOT_SIZE: u32 = 0x0007_4000;
pub const TRAILER_MAGIC_ADDRESS: u32 = SECONDARY_SLOT_START + SECONDARY_SLOT_SIZE - 16;

/// MCUBoot image magic in the byte order written to SPI flash.
/// Write this only after verifying the complete staged image.
pub const TRAILER_MAGIC: [u8; 16] = [
    0x77, 0xc2, 0x95, 0xf3, 0x60, 0xd2, 0xef, 0x7f, 0x35, 0x52, 0x50, 0x0f, 0x2c, 0xb6, 0x79, 0x80,
];

// Keep the entire last 4 KiB sector for MCUBoot's swap-status trailer. Its
// metadata occupies more than just the final 16-byte magic.
pub const MAX_IMAGE_SIZE: u32 = SECONDARY_SLOT_SIZE - 4096;
pub const SECONDARY_SECTORS: u32 = SECONDARY_SLOT_SIZE / 4096;

/// Primary-slot trailer flags for the pinned PineTime MCUBoot layout
/// (8-byte trailer alignment; magic begins at 0x7bff0).
pub const PRIMARY_COPY_DONE_ADDRESS: usize = 0x0007_bfe0;
pub const PRIMARY_IMAGE_OK_ADDRESS: usize = 0x0007_bfe8;

/// A freshly debugger-flashed primary has erased trailer words; a swapped
/// trial has copy_done=1 but image_ok erased. Do not erase its rollback copy.
pub fn primary_allows_staging(copy_done: u8, image_ok: u8) -> bool {
    (copy_done == 0xff && image_ok == 0xff)
        || ((copy_done == 1 || copy_done == 0xff) && image_ok == 1)
}

/// Erase the trailer sector first so an interrupted update cannot leave a
/// pending marker alongside a partially replaced image.
pub fn erase_sector_address(index: u32) -> Option<u32> {
    match index {
        0 => Some(SECONDARY_SLOT_START + SECONDARY_SLOT_SIZE - 4096),
        1..SECONDARY_SECTORS => Some(SECONDARY_SLOT_START + (index - 1) * 4096),
        _ => None,
    }
}

pub fn image_range_ok(address: u32, len: usize) -> bool {
    let Ok(len) = u32::try_from(len) else {
        return false;
    };
    let Some(end) = address.checked_add(len) else {
        return false;
    };
    address >= SECONDARY_SLOT_START && end <= SECONDARY_SLOT_START + MAX_IMAGE_SIZE
}

pub fn parse_application_size(data: &[u8]) -> Option<u32> {
    if data.len() != 12 || data[..8] != [0; 8] {
        return None;
    }
    let size = u32::from_le_bytes(data[8..12].try_into().ok()?);
    (0x200..=MAX_IMAGE_SIZE).contains(&size).then_some(size)
}

pub fn packet_len_ok(len: usize, remaining: u32) -> bool {
    len > 0 && len <= 20 && len as u32 <= remaining && (len == 20 || len as u32 == remaining)
}

/// Nordic legacy init packet for PineTime application-only images.
pub fn expected_crc16(init: &[u8]) -> Option<u16> {
    if !(14..=20).contains(&init.len()) || u16::from_le_bytes([init[0], init[1]]) != 0x0052 {
        return None;
    }
    let sd_count = u16::from_le_bytes([init[8], init[9]]) as usize;
    if sd_count == 0 || init.len() != 12 + 2 * sd_count {
        return None;
    }
    Some(u16::from_le_bytes([
        init[init.len() - 2],
        init[init.len() - 1],
    ]))
}

/// CRC-16/CCITT-FALSE used by the legacy init packet (initial value 0xffff).
pub fn crc16_update(mut crc: u16, data: &[u8]) -> u16 {
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Evidence the receiver must collect from a full readback of the staged
/// image before it can ask MCUBoot to try the secondary slot.
pub struct ActivationPlan {
    _verified: (),
}

impl ActivationPlan {
    pub fn from_readback(
        received: u32,
        declared_size: u32,
        expected_crc16: u16,
        readback_crc16: u16,
        header: &[u8; 16],
    ) -> Option<Self> {
        let image_limit = MAX_IMAGE_SIZE;
        let header_is_mcuboot = header[..4] == [0x3d, 0xb8, 0xf3, 0x96];
        let header_size = u16::from_le_bytes([header[8], header[9]]);
        let payload_size = u32::from_le_bytes([header[12], header[13], header[14], header[15]]);
        if received != declared_size
            || !(0x200..=image_limit).contains(&received)
            || expected_crc16 != readback_crc16
            || !header_is_mcuboot
            // InfiniTime uses the standard 32-byte MCUBoot header, while
            // Kongle pads its own header to 512 bytes for the vector offset.
            || !matches!(header_size, 0x20 | 0x200)
            || payload_size == 0
            || payload_size > received - header_size as u32
        {
            return None;
        }
        Some(Self { _verified: () })
    }

    #[cfg(feature = "ota-activation")]
    pub fn trial_marker(self) -> (u32, [u8; 16]) {
        (TRAILER_MAGIC_ADDRESS, TRAILER_MAGIC)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UpdateStatus {
    Idle,
    Erasing,
    Receiving { received: u32, total: u32 },
    Validating,
    Staged,
    ReadyToRestart,
    Trial,
    Failed,
}

impl UpdateStatus {
    /// Returns a bounded percentage for display; 100% means all bytes arrived,
    /// not that validation or activation succeeded.
    pub fn percent(self) -> Option<u8> {
        match self {
            Self::Receiving { received, total } if total > 0 => {
                Some(((received.min(total) as u64 * 100) / total as u64) as u8)
            }
            _ => None,
        }
    }
}
