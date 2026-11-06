//! On-disk record format for reboot-persistent battery samples.

use crate::battery_history::Sample;

pub const RECORD_SIZE: usize = 14;

fn checksum(bytes: &[u8]) -> u16 {
    bytes.iter().fold(0xffffu16, |crc, byte| {
        let mut crc = crc ^ ((*byte as u16) << 8);
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
        crc
    })
}

pub fn encode(sample: Sample) -> [u8; RECORD_SIZE] {
    let mut data = [0u8; RECORD_SIZE];
    data[0..4].copy_from_slice(&sample.epoch_secs.to_le_bytes());
    data[4..8].copy_from_slice(&sample.uptime_secs.to_le_bytes());
    data[8..10].copy_from_slice(&sample.mv.to_le_bytes());
    data[10] = sample.percent;
    data[11] = u8::from(sample.charging);
    let crc = checksum(&data[..12]);
    data[12..14].copy_from_slice(&crc.to_le_bytes());
    data
}

pub fn decode(data: &[u8; RECORD_SIZE]) -> Option<Sample> {
    let epoch_secs = u32::from_le_bytes(data[0..4].try_into().ok()?);
    let uptime_secs = u32::from_le_bytes(data[4..8].try_into().ok()?);
    let mv = u16::from_le_bytes(data[8..10].try_into().ok()?);
    if epoch_secs < 1_700_000_000
        || data[10] > 100
        || data[11] > 1
        || u16::from_le_bytes(data[12..14].try_into().ok()?) != checksum(&data[..12])
    {
        return None;
    }
    Some(Sample {
        uptime_secs,
        epoch_secs,
        mv,
        percent: data[10],
        charging: data[11] == 1,
    })
}
