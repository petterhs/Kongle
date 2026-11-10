//! Human-readable address formatting for the exact bytes passed to the BLE stack.
use core::fmt::Write;
use heapless::String;

pub fn display_address(address: [u8; 6]) -> String<17> {
    let mut text = String::new();
    // HCI addresses are little-endian; phone apps display the most significant
    // octet first. Never generate a second identity just for the About page.
    let _ = write!(
        text,
        "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        address[5], address[4], address[3], address[2], address[1], address[0]
    );
    text
}
