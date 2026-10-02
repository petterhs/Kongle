//! BLE peripheral with Current Time Service and Nordic legacy application DFU.
//! Uses trouble-host with Apache NimBLE controller (no nRF SoftDevice).

use chrono::NaiveDateTime;
use embassy_futures::join::join;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender, watch::Receiver,
};
use static_cell::StaticCell;
use trouble_host::prelude::DefaultPacketPool;
use trouble_host::prelude::*;

use crate::{current_time::parse_cts, dfu, flash::Flash, TimeState};
use embedded_hal::spi::SpiDevice;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels (signal + att)
const L2CAP_CHANNELS_MAX: usize = 2;

/// Current Time Service (0x1805) - Bluetooth GATT standard; literal required by macro
const CTS_SERVICE_UUID: &str = "00001805-0000-1000-8000-00805f9b34fb";
/// Current Time characteristic (0x2A2B); literal required by macro
const CTS_CHAR_UUID: &str = "00002a2b-0000-1000-8000-00805f9b34fb";

/// GATT server with CTS and opt-in Nordic legacy DFU staging.
#[gatt_server]
struct Server {
    current_time_service: CurrentTimeService,
    dfu_service: DfuService,
}

#[gatt_service(uuid = "00001530-1212-efde-1523-785feabcd123")]
struct DfuService {
    #[characteristic(
        uuid = "00001531-1212-efde-1523-785feabcd123",
        write,
        write_without_response,
        notify
    )]
    control_point: heapless09::Vec<u8, 20>,
    #[characteristic(uuid = "00001532-1212-efde-1523-785feabcd123", write_without_response)]
    packet: heapless09::Vec<u8, 20>,
    #[characteristic(uuid = "00001534-1212-efde-1523-785feabcd123", read, value = 8)]
    revision: u16,
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

/// Run the BLE stack: advertise "Kongle", expose CTS, handle read/write and set-time channel.
pub async fn run<C, S>(
    controller: C,
    address: [u8; 6],
    time_rx: Receiver<'static, CriticalSectionRawMutex, TimeState, 2>,
    set_time_tx: Sender<'static, CriticalSectionRawMutex, NaiveDateTime, 1>,
    mut flash: Flash<S>,
) where
    C: Controller + 'static,
    S: SpiDevice<u8>,
{
    let address = Address::random(address);
    defmt::info!("BLE address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    defmt::info!("Starting BLE advertising, CTS and DFU staging");
    let server: &Server<'static> = {
        let gap = GapConfig::Peripheral(PeripheralConfig {
            name: "Kongle",
            appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
        });
        GATT_SERVER.init(Server::new_with_config(gap).unwrap())
    };

    let mut time_rx = time_rx;
    if let Some(t) = time_rx.try_changed() {
        let _ = server.set(&server.current_time_service.current_time, &encode_cts(&t));
    }
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
                        gatt_events_and_time_sync_task(
                            server,
                            &conn,
                            &mut time_rx,
                            &set_time_tx,
                            &mut flash,
                        )
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
async fn gatt_events_and_time_sync_task<P: PacketPool, S: SpiDevice<u8>>(
    server: &Server<'static>,
    conn: &GattConnection<'_, '_, P>,
    time_rx: &mut Receiver<'static, CriticalSectionRawMutex, TimeState, 2>,
    set_time_tx: &Sender<'static, CriticalSectionRawMutex, NaiveDateTime, 1>,
    flash: &mut Flash<S>,
) {
    let mut dfu = dfu::Receiver::new(flash);
    // Time updates are not consumed while advertising. Publish the latest one
    // before the newly connected client can read the shared GATT attribute.
    if let Some(t) = time_rx.try_changed() {
        let _ = server.set(&server.current_time_service.current_time, &encode_cts(&t));
    }

    // Keep the GATT listener alive for the entire connection. Cancelling even
    // conn.next() on each clock tick can lose an unacknowledged DFU packet;
    // Furu then waits forever for a receipt at the requested packet count.
    let _ = embassy_futures::select::select(
        async {
            loop {
                match conn.next().await {
                    GattConnectionEvent::Disconnected { reason } => {
                        defmt::info!("[gatt] disconnected: {:?}", reason);
                        break;
                    }
                    GattConnectionEvent::Gatt { event } => {
                        let mut dfu_reply = None;
                        let error = match &event {
                            GattEvent::Write(write)
                                if write.handle()
                                    == server.current_time_service.current_time.handle =>
                            {
                                let data = write.data();
                                if data.len() != 10 {
                                    Some(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH)
                                } else if let Some(dt) = parse_cts(data) {
                                    // Acknowledged writes must actually reach the clock task.
                                    set_time_tx
                                        .try_send(dt)
                                        .err()
                                        .map(|_| AttErrorCode::INSUFFICIENT_RESOURCES)
                                } else {
                                    Some(AttErrorCode::VALUE_NOT_ALLOWED)
                                }
                            }
                            GattEvent::Write(write)
                                if write.handle() == server.dfu_service.control_point.handle =>
                            {
                                let opcode = write.data().first().copied().unwrap_or(0);
                                match dfu.control(write.data()).await {
                                    Ok(reply) => {
                                        dfu_reply = reply;
                                    }
                                    Err(e) => {
                                        defmt::warn!("[dfu] control error: {:?}", e);
                                        dfu.fail();
                                        dfu_reply = Some(dfu::failure_response(opcode));
                                    }
                                }
                                None
                            }
                            GattEvent::Write(write)
                                if write.handle() == server.dfu_service.packet.handle =>
                            {
                                match dfu.packet(write.data()).await {
                                    Ok(reply) => {
                                        dfu_reply = reply;
                                    }
                                    Err(e) => {
                                        defmt::warn!("[dfu] packet error: {:?}", e);
                                        let opcode = dfu.packet_failure_opcode();
                                        dfu.fail();
                                        dfu_reply = Some(dfu::failure_response(opcode));
                                    }
                                }
                                None
                            }
                            _ => None,
                        };
                        let response = match error {
                            Some(error) => event.reject(error),
                            None => event.accept(),
                        };
                        match response {
                            Ok(reply) => reply.send().await,
                            Err(e) => defmt::warn!("[gatt] error sending response: {:?}", e),
                        }
                        if let Some(reply) = dfu_reply {
                            if let Ok(payload) = heapless09::Vec::<u8, 20>::from_slice(&reply) {
                                if server
                                    .dfu_service
                                    .control_point
                                    .notify(conn, &payload)
                                    .await
                                    .is_err()
                                {
                                    defmt::warn!("[dfu] failed to notify control point");
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
        },
        async {
            loop {
                let t = time_rx.changed().await;
                let _ = server.set(&server.current_time_service.current_time, &encode_cts(&t));
            }
        },
    )
    .await;
    dfu.on_disconnect();
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
