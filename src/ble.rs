//! BLE peripheral with Current Time Service and Nordic legacy application DFU.
//! Uses trouble-host with Apache NimBLE controller (no nRF SoftDevice).

use chrono::NaiveDateTime;
use core::cell::RefCell;
use embassy_futures::join::join;
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex as BlockingMutex},
    channel::Sender,
    mutex::Mutex as AsyncMutex,
    watch::{Receiver, Sender as WatchSender},
};
use embassy_time::Instant;
#[cfg(feature = "ota-activation")]
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use trouble_host::prelude::DefaultPacketPool;
use trouble_host::prelude::*;

use crate::{
    battery_history::History, current_time::parse_cts, device::battery::BatteryState, dfu,
    flash::Flash, notifications, notifications::Notification, TimeState, FIRMWARE_VERSION,
};
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
    // Append new services so existing CTS/DFU handles remain stable for
    // Android's cached GATT database at Kongle's persistent BLE address.
    device_information: DeviceInformationService,
    alert_notification: AlertNotificationService,
    battery_service: BatteryService,
    battery_history: BatteryHistoryService,
}

#[gatt_service(uuid = "0000180f-0000-1000-8000-00805f9b34fb")]
struct BatteryService {
    #[characteristic(uuid = "00002a19-0000-1000-8000-00805f9b34fb", read)]
    battery_level: u8,
}

#[gatt_service(uuid = "6b6f0000-1886-4b40-ae7d-00a11ce50001")]
struct BatteryHistoryService {
    /// millivolts LE, percent, charging (0 or 1)
    #[characteristic(uuid = "6b6f0001-1886-4b40-ae7d-00a11ce50001", read)]
    current: [u8; 4],
    /// Read the page selected by a cursor write.
    #[characteristic(uuid = "6b6f0002-1886-4b40-ae7d-00a11ce50001", read)]
    page: heapless09::Vec<u8, 240>,
    #[characteristic(uuid = "6b6f0003-1886-4b40-ae7d-00a11ce50001", write)]
    cursor: [u8; 4],
    #[characteristic(uuid = "6b6f0004-1886-4b40-ae7d-00a11ce50001", write)]
    epoch: [u8; 4],
}

/// Furu writes InfiniTime-compatible New Alert payloads here.
#[gatt_service(uuid = "00001811-0000-1000-8000-00805f9b34fb")]
struct AlertNotificationService {
    #[characteristic(uuid = "00002a46-0000-1000-8000-00805f9b34fb", write)]
    new_alert: heapless09::Vec<u8, 103>,
}

/// Standard Device Information Service. Furu reads Firmware Revision from it.
#[gatt_service(uuid = "0000180a-0000-1000-8000-00805f9b34fb")]
struct DeviceInformationService {
    #[characteristic(uuid = "00002a26-0000-1000-8000-00805f9b34fb", read)]
    firmware_revision: heapless09::Vec<u8, 32>,
    #[characteristic(uuid = "00002a29-0000-1000-8000-00805f9b34fb", read)]
    manufacturer_name: heapless09::Vec<u8, 32>,
    #[characteristic(uuid = "00002a24-0000-1000-8000-00805f9b34fb", read)]
    model_number: heapless09::Vec<u8, 32>,
    #[characteristic(uuid = "00002a27-0000-1000-8000-00805f9b34fb", read)]
    hardware_revision: heapless09::Vec<u8, 32>,
    #[characteristic(uuid = "00002a28-0000-1000-8000-00805f9b34fb", read)]
    software_revision: heapless09::Vec<u8, 32>,
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
    connected_tx: WatchSender<'static, CriticalSectionRawMutex, bool, 2>,
    notification_tx: Sender<'static, CriticalSectionRawMutex, Notification, 2>,
    flash: &'static AsyncMutex<CriticalSectionRawMutex, Flash<S>>,
    mut battery_rx: Receiver<'static, CriticalSectionRawMutex, BatteryState, 2>,
    history: &'static BlockingMutex<CriticalSectionRawMutex, RefCell<History>>,
) where
    C: Controller + 'static,
    S: SpiDevice<u8> + 'static,
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
    let firmware_revision = heapless09::Vec::<u8, 32>::from_slice(FIRMWARE_VERSION.as_bytes())
        .expect("firmware version fits the GATT characteristic");
    server
        .set(
            &server.device_information.firmware_revision,
            &firmware_revision,
        )
        .unwrap();

    for (characteristic, value) in [
        (&server.device_information.manufacturer_name, "PINE64"),
        (&server.device_information.model_number, "PineTime"),
        // The PCB revision cannot be identified reliably at runtime.
        (
            &server.device_information.hardware_revision,
            "PineTime / nRF52832",
        ),
        (
            &server.device_information.software_revision,
            concat!("Kongle ", env!("CARGO_PKG_VERSION")),
        ),
    ] {
        let value = heapless09::Vec::<u8, 32>::from_slice(value.as_bytes())
            .expect("device information fits the GATT characteristic");
        server.set(characteristic, &value).unwrap();
    }

    let mut time_rx = time_rx;
    connected_tx.send(false);
    if let Some(t) = time_rx.try_changed() {
        let _ = server.set(&server.current_time_service.current_time, &encode_cts(&t));
    }
    if let Some(battery) = battery_rx.try_get() {
        publish_battery(server, battery);
    }
    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Kongle", &mut peripheral).await {
                Ok(advertiser) => {
                    defmt::info!("[adv] advertising");
                    match advertiser.accept().await {
                        Ok(raw_conn) => match raw_conn.with_attribute_server(server) {
                            Ok(conn) => {
                                defmt::info!("[adv] GATT connection established");
                                connected_tx.send(true);
                                gatt_events_and_time_sync_task(
                                    server,
                                    &conn,
                                    &mut time_rx,
                                    &set_time_tx,
                                    &notification_tx,
                                    flash,
                                    &mut battery_rx,
                                    history,
                                )
                                .await;
                                connected_tx.send(false);
                                defmt::info!("[adv] GATT connection closed; advertising again");
                            }
                            Err(e) => defmt::warn!("[adv] GATT attach failed: {:?}", e),
                        },
                        Err(e) => defmt::warn!("[adv] accept failed: {:?}", e),
                    }
                }
                Err(BleHostError::BleHost(e)) => {
                    defmt::warn!("[adv] host advertising error: {:?}", e)
                }
                Err(BleHostError::Controller(_)) => {
                    defmt::warn!("[adv] controller advertising error")
                }
            }
        }
    })
    .await;
}

/// Background task required by trouble-host: runs the BLE host stack.
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            match e {
                BleHostError::BleHost(error) => defmt::warn!("[ble_task] host error: {:?}", error),
                BleHostError::Controller(_) => defmt::warn!("[ble_task] controller error"),
            }
        }
    }
}

/// Handle GATT events and keep CTS characteristic updated from TIME_WATCH.
async fn gatt_events_and_time_sync_task<P: PacketPool, S: SpiDevice<u8>>(
    server: &Server<'static>,
    conn: &GattConnection<'_, '_, P>,
    time_rx: &mut Receiver<'static, CriticalSectionRawMutex, TimeState, 2>,
    set_time_tx: &Sender<'static, CriticalSectionRawMutex, NaiveDateTime, 1>,
    notification_tx: &Sender<'static, CriticalSectionRawMutex, Notification, 2>,
    flash: &AsyncMutex<CriticalSectionRawMutex, Flash<S>>,
    battery_rx: &mut Receiver<'static, CriticalSectionRawMutex, BatteryState, 2>,
    history: &'static BlockingMutex<CriticalSectionRawMutex, RefCell<History>>,
) {
    let mut dfu = dfu::Receiver::new();
    // Time updates are not consumed while advertising. Publish the latest one
    // before the newly connected client can read the shared GATT attribute.
    if let Some(t) = time_rx.try_changed() {
        let _ = server.set(&server.current_time_service.current_time, &encode_cts(&t));
    }
    if let Some(battery) = battery_rx.try_changed() {
        publish_battery(server, battery);
    }

    // Keep the GATT listener alive for the entire connection. Cancelling even
    // conn.next() on each clock tick can lose an unacknowledged DFU packet;
    // Furu then waits forever for a receipt at the requested packet count.
    let gatt_and_time = embassy_futures::select::select(
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
                                if write.handle() == server.battery_history.epoch.handle =>
                            {
                                if let Ok(epoch) = <[u8; 4]>::try_from(write.data()) {
                                    let epoch = u32::from_le_bytes(epoch);
                                    if epoch < 1_700_000_000 {
                                        Some(AttErrorCode::VALUE_NOT_ALLOWED)
                                    } else {
                                        history.lock(|h| {
                                            h.borrow_mut().set_epoch(
                                                (Instant::now().as_millis() / 1000) as u32,
                                                epoch,
                                            )
                                        });
                                        None
                                    }
                                } else {
                                    Some(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH)
                                }
                            }
                            GattEvent::Write(write)
                                if write.handle() == server.battery_history.cursor.handle =>
                            {
                                if let Ok(cursor) = <[u8; 4]>::try_from(write.data()) {
                                    let page = history.lock(|h| {
                                        h.borrow().page(
                                            u32::from_le_bytes(cursor),
                                            (Instant::now().as_millis() / 1000) as u32,
                                        )
                                    });
                                    let value = heapless09::Vec::<u8, 240>::from_slice(&page)
                                        .expect("history page fits GATT value");
                                    let _ = server.set(&server.battery_history.page, &value);
                                    None
                                } else {
                                    Some(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH)
                                }
                            }
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
                                if write.handle() == server.alert_notification.new_alert.handle =>
                            {
                                match notifications::parse_new_alert(write.data()) {
                                    Ok(notification) => {
                                        defmt::info!("[ans] new alert received");
                                        notification_tx
                                            .try_send(notification)
                                            .err()
                                            .map(|_| AttErrorCode::INSUFFICIENT_RESOURCES)
                                    }
                                    Err(_) => {
                                        defmt::warn!("[ans] invalid alert");
                                        Some(AttErrorCode::VALUE_NOT_ALLOWED)
                                    }
                                }
                            }
                            GattEvent::Write(write)
                                if write.handle() == server.dfu_service.control_point.handle =>
                            {
                                let opcode = write.data().first().copied().unwrap_or(0);
                                let mut flash = flash.lock().await;
                                match dfu.control(&mut flash, write.data()).await {
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
                                let mut flash = flash.lock().await;
                                match dfu.packet(&mut flash, write.data()).await {
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
                                let trace_receipt = reply.len() == 5
                                    && reply[0] == 0x11
                                    && u32::from_le_bytes([reply[1], reply[2], reply[3], reply[4]])
                                        <= 2_000;
                                if trace_receipt {
                                    defmt::info!("DFU queuing receipt notification");
                                }
                                if server
                                    .dfu_service
                                    .control_point
                                    .notify(conn, &payload)
                                    .await
                                    .is_err()
                                {
                                    defmt::warn!("[dfu] failed to notify control point");
                                } else if trace_receipt {
                                    defmt::info!("DFU receipt notification queued");
                                }
                            }
                        }
                        #[cfg(feature = "ota-activation")]
                        if dfu.activation_requested() {
                            // Opcode 5 has no DFU notification. Allow the ATT
                            // write and Android's callback to settle first.
                            Timer::after(Duration::from_millis(500)).await;
                            cortex_m::peripheral::SCB::sys_reset();
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
    );
    let _ = embassy_futures::select::select(gatt_and_time, async {
        loop {
            publish_battery(server, battery_rx.changed().await);
        }
    })
    .await;
    dfu.on_disconnect();
}

fn publish_battery(server: &Server<'static>, battery: BatteryState) {
    let _ = server.set(&server.battery_service.battery_level, &battery.percent);
    let [lo, hi] = battery.mv.to_le_bytes();
    let _ = server.set(
        &server.battery_history.current,
        &[lo, hi, battery.percent, u8::from(battery.charging)],
    );
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
