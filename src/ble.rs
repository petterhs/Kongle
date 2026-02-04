//! BLE peripheral with Current Time Service (CTS) for time sync over BLE.
//! Uses trouble-host with Apache NimBLE controller (no nRF SoftDevice).

use chrono::NaiveDateTime;
use embassy_futures::join::join;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender, watch::Receiver,
};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use trouble_host::prelude::DefaultPacketPool;
use trouble_host::prelude::*;

use crate::TimeState;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels (signal + att)
const L2CAP_CHANNELS_MAX: usize = 2;

/// Current Time Service (0x1805) - Bluetooth GATT standard; literal required by macro
const CTS_SERVICE_UUID: &str = "00001805-0000-1000-8000-00805f9b34fb";
/// Current Time characteristic (0x2A2B); literal required by macro
const CTS_CHAR_UUID: &str = "00002a2b-0000-1000-8000-00805f9b34fb";

/// GATT Server with Current Time Service only (macros from prelude must be in scope)
#[gatt_server]
struct Server {
    current_time_service: CurrentTimeService,
}

/// Current Time Service: single characteristic (Current Time, 10 bytes)
#[gatt_service(uuid = "00001805-0000-1000-8000-00805f9b34fb")]
struct CurrentTimeService {
    /// Current Time: year(2), month, day, hour, min, sec, day_of_week, fractions256, reason
    #[characteristic(uuid = "00002a2b-0000-1000-8000-00805f9b34fb", read, write)]
    current_time: [u8; 10],
}

// Global GATT server. Safe because configuration uses only 'static data.
static GATT_SERVER: StaticCell<Server<'static>> = StaticCell::new();

/// Encode TimeState to 10-byte CTS format (Bluetooth spec)
fn encode_cts(t: &TimeState) -> [u8; 10] {
    let year_bytes = t.year.to_le_bytes();
    [
        year_bytes[0],
        year_bytes[1],
        t.month,
        t.day,
        t.hours,
        t.minutes,
        t.seconds,
        0, // day of week (1-7, 0 = unknown)
        0, // fractions of second (1/256)
        0, // adjust reason
    ]
}

/// Parse 10-byte CTS write into NaiveDateTime (same as master branch)
fn parse_cts(data: &[u8]) -> Option<NaiveDateTime> {
    if data.len() < 7 {
        return None;
    }
    let year = u16::from_le_bytes([data[0], data[1]]) as i32;
    let month = data[2];
    let day = data[3];
    let hour = data[4];
    let minute = data[5];
    let second = data[6];
    let date = chrono::NaiveDate::from_ymd_opt(year, month as u32, day as u32)?;
    let time = chrono::NaiveTime::from_hms_opt(hour as u32, minute as u32, second as u32)?;
    Some(date.and_time(time))
}

/// Run the BLE stack: advertise "Kongle", expose CTS, handle read/write and set-time channel.
pub async fn run<C>(
    controller: C,
    time_rx: Receiver<'static, CriticalSectionRawMutex, TimeState, 2>,
    set_time_tx: Sender<'static, CriticalSectionRawMutex, NaiveDateTime, 1>,
) where
    C: Controller + 'static,
{
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    defmt::info!("BLE address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    defmt::info!("Starting BLE advertising and CTS");
    let server: &Server<'static> = {
        let gap = GapConfig::Peripheral(PeripheralConfig {
            name: "Kongle",
            appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
        });
        GATT_SERVER.init(Server::new_with_config(gap).unwrap())
    };

    let mut time_rx = time_rx;
    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Kongle", &mut peripheral).await {
                Ok(advertiser) => {
                    defmt::info!("[adv] advertising");
                    if let Ok(conn) = advertiser
                        .accept()
                        .await
                        .and_then(|c| c.with_attribute_server(server))
                    {
                        defmt::info!("[adv] connection established");
                        gatt_events_and_time_sync_task(server, &conn, &mut time_rx, &set_time_tx)
                            .await;
                    } else {
                        defmt::warn!("[adv] error while accepting/attaching GATT server");
                    }
                }
                Err(_e) => {
                    defmt::warn!("[adv] error while starting advertising");
                }
            }
        }
    })
    .await;
}

/// Background task required by trouble-host: runs the BLE host stack.
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if runner.run().await.is_err() {
            defmt::warn!("[ble_task] host error");
        }
    }
}

/// Handle GATT events and keep CTS characteristic updated from TIME_WATCH.
async fn gatt_events_and_time_sync_task<P: PacketPool>(
    server: &Server<'static>,
    conn: &GattConnection<'_, '_, P>,
    time_rx: &mut Receiver<'static, CriticalSectionRawMutex, TimeState, 2>,
    set_time_tx: &Sender<'static, CriticalSectionRawMutex, NaiveDateTime, 1>,
) {
    loop {
        let disconnected = match embassy_futures::select::select(
            async {
                match conn.next().await {
                    GattConnectionEvent::Disconnected { reason } => {
                        defmt::info!("[gatt] disconnected: {:?}", reason);
                        true
                    }
                    GattConnectionEvent::Gatt { event } => {
                        match &event {
                            GattEvent::Read(_) => {}
                            GattEvent::Write(event) => {
                                let data = event.data();
                                if data.len() >= 7 {
                                    if let Some(dt) = parse_cts(data) {
                                        defmt::info!("[gatt] Write Current Time received");
                                        let _ = set_time_tx.try_send(dt);
                                    }
                                }
                            }
                            _ => {}
                        }
                        match event.accept() {
                            Ok(reply) => reply.send().await,
                            Err(e) => defmt::warn!("[gatt] error sending response: {:?}", e),
                        }
                        false
                    }
                    _ => false,
                }
            },
            async {
                Timer::after(Duration::from_millis(100)).await;
                if let Some(t) = time_rx.try_changed() {
                    let buf = encode_cts(&t);
                    let _ = server.set(&server.current_time_service.current_time, &buf);
                }
                false
            },
        )
        .await
        {
            embassy_futures::select::Either::First(b)
            | embassy_futures::select::Either::Second(b) => b,
        };
        if disconnected {
            break;
        }
    }
}

/// Advertise and wait for a connection, returning an `Advertiser` handle.
async fn advertise<'values, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    // GATT server is attached later by the caller
) -> Result<Advertiser<'values, C, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x05, 0x18]]), // Current Time Service 0x1805
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser: Advertiser<'values, C, DefaultPacketPool> = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    Ok(advertiser)
}
