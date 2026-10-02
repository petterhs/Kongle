//! Nordic legacy application DFU receiver used by Furu and InfiniTime.
//! Transfer and readback are supported; installing the staged image is disabled.

use embassy_time::{Duration, Timer};
use embedded_hal::spi::SpiDevice;
use heapless::Vec;

use crate::{
    flash::{Flash, FlashError},
    ota::{
        crc16_update, expected_crc16, packet_len_ok, parse_application_size,
        primary_allows_staging, ActivationPlan, UpdateStatus, PRIMARY_COPY_DONE_ADDRESS,
        PRIMARY_IMAGE_OK_ADDRESS, SECONDARY_SLOT_START,
    },
    OTA_WATCH,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Idle,
    Sizes,
    InitStart,
    InitData,
    InitDone,
    InitComplete,
    Receiving,
    Received,
    Staged,
    Failed,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum DfuError {
    Protocol,
    Flash(FlashError),
    InvalidImage,
    ActivationDisabled,
    UnconfirmedTrial,
}

impl From<FlashError> for DfuError {
    fn from(error: FlashError) -> Self {
        Self::Flash(error)
    }
}

fn notification(data: &[u8]) -> Vec<u8, 5> {
    Vec::from_slice(data).unwrap()
}

pub fn failure_response(opcode: u8) -> Vec<u8, 5> {
    notification(&[0x10, opcode, 0x05])
}

pub struct Receiver<'a, S> {
    flash: &'a mut Flash<S>,
    phase: Phase,
    total: u32,
    received: u32,
    expected_crc: u16,
    packet_count: u32,
    prn: u16,
    last_percent: Option<u8>,
}

impl<'a, S: SpiDevice<u8>> Receiver<'a, S> {
    pub fn new(flash: &'a mut Flash<S>) -> Self {
        Self {
            flash,
            phase: Phase::Idle,
            total: 0,
            received: 0,
            expected_crc: 0,
            packet_count: 0,
            prn: 0,
            last_percent: None,
        }
    }

    pub fn packet_failure_opcode(&self) -> u8 {
        match self.phase {
            Phase::Sizes => 0x01,
            Phase::InitData => 0x02,
            _ => 0x03,
        }
    }

    pub fn fail(&mut self) {
        self.phase = Phase::Failed;
        OTA_WATCH.sender().send(UpdateStatus::Failed);
    }

    pub fn on_disconnect(&mut self) {
        match self.phase {
            Phase::Idle | Phase::Failed | Phase::Staged => {}
            _ => self.fail(),
        }
    }

    pub async fn control(&mut self, data: &[u8]) -> Result<Option<Vec<u8, 5>>, DfuError> {
        match data {
            [0x01, 0x04] if matches!(self.phase, Phase::Idle | Phase::Failed | Phase::Staged) => {
                if !cfg!(feature = "ota-staging") {
                    defmt::warn!("DFU staging is disabled in this build");
                    return Err(DfuError::ActivationDisabled);
                }
                // The secondary slot is also MCUBoot's rollback copy after a
                // trial swap. Do not erase it until that image is confirmed.
                let copy_done =
                    unsafe { core::ptr::read_volatile(PRIMARY_COPY_DONE_ADDRESS as *const u32) };
                let image_ok =
                    unsafe { core::ptr::read_volatile(PRIMARY_IMAGE_OK_ADDRESS as *const u32) };
                if !primary_allows_staging(copy_done, image_ok) {
                    defmt::warn!(
                        "DFU primary image is an unconfirmed trial: copy_done={:08x} image_ok={:08x}",
                        copy_done,
                        image_ok
                    );
                    return Err(DfuError::UnconfirmedTrial);
                }
                self.flash.initialize().await?;
                self.phase = Phase::Sizes;
                self.total = 0;
                self.received = 0;
                self.packet_count = 0;
                self.prn = 0;
                self.last_percent = None;
                Ok(None)
            }
            [0x02, 0x00] if self.phase == Phase::InitStart => {
                self.phase = Phase::InitData;
                Ok(None)
            }
            [0x02, 0x01] if self.phase == Phase::InitDone => {
                self.phase = Phase::InitComplete;
                // The sender requests the packet receipt interval next.
                Ok(Some(notification(&[0x10, 0x02, 0x01])))
            }
            [0x08, low, high] if self.phase == Phase::InitComplete => {
                self.prn = u16::from_le_bytes([*low, *high]);
                if self.prn == 0 {
                    return Err(DfuError::Protocol);
                }
                Ok(None)
            }
            [0x03] if self.phase == Phase::InitComplete && self.prn > 0 => {
                self.phase = Phase::Receiving;
                self.last_percent = Some(0);
                OTA_WATCH.sender().send(UpdateStatus::Receiving {
                    received: 0,
                    total: self.total,
                });
                Ok(None)
            }
            [0x04] if self.phase == Phase::Received => {
                OTA_WATCH.sender().send(UpdateStatus::Validating);
                let mut header = [0; 16];
                self.flash.read(SECONDARY_SLOT_START, &mut header)?;
                let mut crc = 0xffff;
                let mut offset = 0;
                let mut buffer = [0; 200];
                while offset < self.total {
                    let size = (self.total - offset).min(buffer.len() as u32) as usize;
                    self.flash
                        .read(SECONDARY_SLOT_START + offset, &mut buffer[..size])?;
                    crc = crc16_update(crc, &buffer[..size]);
                    offset += size as u32;
                    Timer::after(Duration::from_millis(1)).await;
                }
                if ActivationPlan::from_readback(
                    self.received,
                    self.total,
                    self.expected_crc,
                    crc,
                    &header,
                )
                .is_none()
                {
                    self.fail();
                    return Ok(Some(failure_response(0x04)));
                }
                self.phase = Phase::Staged;
                OTA_WATCH.sender().send(UpdateStatus::Staged);
                // A successful validation response would make Furu send opcode 5
                // and report success. Do not claim success until activation has
                // been wired and tested on the development board.
                Ok(Some(failure_response(0x04)))
            }
            [0x05] if self.phase == Phase::Staged => Err(DfuError::ActivationDisabled),
            _ => Err(DfuError::Protocol),
        }
    }

    pub async fn packet(&mut self, data: &[u8]) -> Result<Option<Vec<u8, 5>>, DfuError> {
        match self.phase {
            Phase::Sizes => {
                let total = parse_application_size(data).ok_or(DfuError::InvalidImage)?;
                self.total = total;
                OTA_WATCH.sender().send(UpdateStatus::Erasing);
                self.flash.erase_secondary().await?;
                self.phase = Phase::InitStart;
                Ok(Some(notification(&[0x10, 0x01, 0x01])))
            }
            Phase::InitData => {
                self.expected_crc = expected_crc16(data).ok_or(DfuError::Protocol)?;
                self.phase = Phase::InitDone;
                Ok(None)
            }
            Phase::Receiving => {
                let remaining = self.total - self.received;
                if !packet_len_ok(data.len(), remaining) {
                    return Err(DfuError::Protocol);
                }
                self.flash
                    .write_image(SECONDARY_SLOT_START + self.received, data)
                    .await?;
                self.received += data.len() as u32;
                self.packet_count += 1;
                let status = UpdateStatus::Receiving {
                    received: self.received,
                    total: self.total,
                };
                if status.percent() != self.last_percent {
                    self.last_percent = status.percent();
                    OTA_WATCH.sender().send(status);
                }
                if self.received == self.total {
                    self.phase = Phase::Received;
                    Ok(Some(notification(&[0x10, 0x03, 0x01])))
                } else if self.packet_count % self.prn as u32 == 0 {
                    let mut response = notification(&[0x11]);
                    response
                        .extend_from_slice(&self.received.to_le_bytes())
                        .unwrap();
                    Ok(Some(response))
                } else {
                    Ok(None)
                }
            }
            _ => Err(DfuError::Protocol),
        }
    }
}
