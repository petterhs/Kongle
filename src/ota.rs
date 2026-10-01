//! Hardware-independent OTA status and MCUBoot secondary-slot layout.
//!
//! The receiver and external-flash driver are still to be implemented. Nothing
//! in this module writes flash or requests a swap.

/// External SPI flash region used by the PineTime bootloader for the secondary image.
pub const SECONDARY_SLOT_START: u32 = 0x0004_0000;
pub const SECONDARY_SLOT_SIZE: u32 = 0x0007_4000;
pub const TRAILER_MAGIC_ADDRESS: u32 = SECONDARY_SLOT_START + SECONDARY_SLOT_SIZE - 16;

/// MCUBoot image magic in the byte order written to SPI flash.
/// A future receiver must write this only after verifying the complete staged image.
pub const TRAILER_MAGIC: [u8; 16] = [
    0x77, 0xc2, 0x95, 0xf3, 0x60, 0xd2, 0xef, 0x7f, 0x35, 0x52, 0x50, 0x0f, 0x2c, 0xb6, 0x79, 0x80,
];

/// Evidence a future receiver must collect from a full readback of the staged
/// image before it can ask MCUBoot to try the secondary slot.
pub struct ActivationPlan;

impl ActivationPlan {
    pub fn from_readback(
        received: u32,
        declared_size: u32,
        expected_crc16: u16,
        readback_crc16: u16,
        header: &[u8; 16],
    ) -> Option<Self> {
        let image_limit = SECONDARY_SLOT_SIZE - TRAILER_MAGIC.len() as u32;
        let header_is_mcuboot = header[..4] == [0x3d, 0xb8, 0xf3, 0x96];
        let header_size = u16::from_le_bytes([header[8], header[9]]);
        let payload_size = u32::from_le_bytes([header[12], header[13], header[14], header[15]]);
        if received != declared_size
            || !(0x200..=image_limit).contains(&received)
            || expected_crc16 != readback_crc16
            || !header_is_mcuboot
            || header_size != 0x200
            || payload_size == 0
            || payload_size > received - header_size as u32
        {
            return None;
        }
        Some(Self)
    }

    /// Deliberately has no call site or SPI implementation yet. This is the
    /// only operation that would make a staged image bootable.
    #[cfg(feature = "ota-activation")]
    pub fn mark_pending(self, flash: &mut impl SecondarySlotFlash) -> Result<(), ()> {
        flash.write(TRAILER_MAGIC_ADDRESS, &TRAILER_MAGIC)?;
        let mut readback = [0; 16];
        flash.read(TRAILER_MAGIC_ADDRESS, &mut readback)?;
        if readback != TRAILER_MAGIC {
            return Err(());
        }
        Ok(())
    }
}

#[cfg(feature = "ota-activation")]
pub trait SecondarySlotFlash {
    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), ()>;
    fn read(&mut self, address: u32, data: &mut [u8]) -> Result<(), ()>;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UpdateStatus {
    Idle,
    Receiving { received: u32, total: u32 },
    Validating,
    ReadyToRestart,
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
