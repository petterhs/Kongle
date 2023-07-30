//! Defines a custom `AttributeValueProvider`.

use chrono::{NaiveDate, NaiveDateTime, NaiveTime};
use core::cmp;
use rtt_target::rprintln;
use rubble::{
    att::{AttUuid, Attribute, AttributeAccessPermissions, AttributeProvider, Handle, HandleRange},
    uuid::Uuid16,
    Error,
};

pub struct KongleAttrs {
    // Attributes exposed to clients that don't change.
    // This includes the "primary service" and "characteristic" attributes.
    // Some attributes are copied from the declaration of `BatteryServiceAttrs` in the gatt module.
    static_attributes: [Attribute<&'static [u8]>; 3],
    // State and resources to be modified/queried when packets are received.
    // The `AttributeValueProvider` interface allows attributes to be generated lazily; those
    // attributes should use these fields.
    // current_time_buf: [u8; 10],
}

const PRIMARY_SERVICE_UUID16: Uuid16 = Uuid16(0x2800);
const CHARACTERISTIC_UUID16: Uuid16 = Uuid16(0x2803);
// const BATTERY_LEVEL_UUID16: Uuid16 = Uuid16(0x2A19);
// const DATE_TIME_UUID16: Uuid16 = Uuid16(0x2A08);
const CURRENT_TIME_UUID16: Uuid16 = Uuid16(0x2A2B);


const CURRENT_TIME_CHAR_DECL_VALUE: [u8; 19] = [
    0x02 | 0x08, // 0x02 = read, 0x08 = write with response
    // 2 byte handle pointing to characteristic value
    0x03,
    0x00,
    // 128-bit UUID of characteristic value (copied from above constant)
    0xFB,
    0x34,
    0x9B,
    0x5F,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0x2B,
    0x2A,
    0x00,
    0x00,
];

impl KongleAttrs {
    pub fn new() -> Self {
        Self {
            static_attributes: [
                Attribute::new(
                    PRIMARY_SERVICE_UUID16.into(),
                    Handle::from_raw(0x0001),
                    &[0x05, 0x18], // "Current Time Service" = 0x1805
                ),
                Attribute::new(
                    CHARACTERISTIC_UUID16.into(),
                    Handle::from_raw(0x0002),
                    &CURRENT_TIME_CHAR_DECL_VALUE,
                ),
                Attribute::new(CURRENT_TIME_UUID16.into(), Handle::from_raw(0x0003), &[]),
            ],
            // current_time_buf: [0u8; 10],
        }
    }
}

impl AttributeProvider for KongleAttrs {
    /// Retrieves the permissions for attribute with the given handle.
    fn attr_access_permissions(&self, handle: Handle) -> AttributeAccessPermissions {
        match handle.as_u16() {
            // 0x0002 => AttributeAccessPermissions::ReadableAndWriteable,
            0x0003 => AttributeAccessPermissions::ReadableAndWriteable,
            _ => AttributeAccessPermissions::Readable,
        }
    }

    /// Attempts to write data to the attribute with the given handle.
    /// If any of your attributes are writeable, this function must be implemented.
    fn write_attr(&mut self, handle: Handle, data: &[u8]) -> Result<(), Error> {
        match handle.as_u16() {
            0x0003 => {
                if data.is_empty() {
                    return Err(Error::InvalidLength);
                }
                rprintln!("data: {:?}", data);
                if data.len() == 10 {
                    let date = NaiveDate::from_ymd_opt(
                        i32::from(data[1]) << 8 | i32::from(data[0]),
                        u32::from(data[2]),
                        u32::from(data[3]),
                    );

                    let time = NaiveTime::from_hms_opt(
                        u32::from(data[4]),
                        u32::from(data[5]),
                        u32::from(data[6]),
                    );

                    if let (Some(date), Some(time)) = (date, time) {
                        rprintln!("date: {:?}, time: {:?}", date, time);
                        let dt = NaiveDateTime::new(date, time);
                        rprintln!("dt: {:?}", dt);
                        crate::app::set_date_time::spawn(dt).unwrap();
                    }
                }

                Ok(())
            }
            _ => panic!("Attempted to write an unwriteable attribute"),
        }
    }

    fn is_grouping_attr(&self, uuid: AttUuid) -> bool {
        uuid == PRIMARY_SERVICE_UUID16 || uuid == CHARACTERISTIC_UUID16
    }

    fn group_end(&self, handle: Handle) -> Option<&Attribute<dyn AsRef<[u8]>>> {
        match handle.as_u16() {
            0x0001 | 0x0002 => Some(&self.static_attributes[2]),
            _ => None,
        }
    }

    /// Applies a function to all attributes with handles within the specified range
    fn for_attrs_in_range(
        &mut self,
        range: HandleRange,
        mut f: impl FnMut(&Self, &Attribute<dyn AsRef<[u8]>>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let count = self.static_attributes.len();
        let start = usize::from(range.start().as_u16() - 1); // handles start at 1, not 0
        let end = usize::from(range.end().as_u16() - 1);

        let attrs = if start >= count {
            &[]
        } else {
            let end = cmp::min(count - 1, end);
            &self.static_attributes[start..=end]
        };

        for attr in attrs {
            f(self, attr)?;
        }
        Ok(())
    }
}
