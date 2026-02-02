#![allow(dead_code)]

use defmt::warn;

/// Placeholder for future BLE time sync implementation.
pub async fn run_time_sync() {
    warn!("BLE time sync not implemented yet");
    futures::future::pending::<()>().await;
}
