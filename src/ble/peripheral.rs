use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::SyncSender,
    },
    time::{Duration, Instant},
};

use esp_idf_hal::{sys::BLE_ATT_MTU_MAX, usb_serial::UsbSerialDriver};
use esp32_nimble::{
    BLEAdvertisementData, BLECharacteristic, BLEDevice, BLEServer, NimbleProperties,
    utilities::{BleUuid, mutex::Mutex},
    uuid128,
};
use eyre::{Context, OptionExt};
use log::{error, info};
use oneshot::Receiver;

use crate::{
    DeviceResponse, LedMode, ble::RemovableSender, blocking_write_all, serialize_response,
    wait_for_ack,
};

const STATE_ID: BleUuid = uuid128!("00000005-A123-48CE-896B-4C76973373E6");
const CLIENT2SERVER_ID: BleUuid = uuid128!("00000006-A123-48CE-896B-4C76973373E6");
const SERVER2CLIENT_ID: BleUuid = uuid128!("00000007-A123-48CE-896B-4C76973373E6");
const IDENT_ID: BleUuid = uuid128!("00000008-A123-48CE-896B-4C76973373E6");

pub fn serve_peripheral(
    usb_serial: &mut UsbSerialDriver<'_>,
    led_tx: &SyncSender<(LedMode, Option<Instant>)>,
    service_uuid_bytes: [u8; 16],
    ble_ident: [u8; 16],
    payload: Vec<u8>,
) -> eyre::Result<()> {
    let started_at = Instant::now();
    let service_uuid = BleUuid::from_uuid128(service_uuid_bytes);
    info!("starting peripheral data exchange: {service_uuid}");

    // Set when any error occurs, needs to be checked before any blocking op
    let cancel = Arc::new(AtomicBool::new(false));

    // Data exchange between bluetooth callbacks and main thread code
    let (connected_tx, connected_rx) = RemovableSender::<u16>::new();
    let (ready_tx, ready_rx) = RemovableSender::<bool>::new();
    // let (data_tx, data_rx) = mpsc::sync_channel::<Vec<u8>>(4);
    let (data_done_tx, data_done_rx) = RemovableSender::<()>::new();

    let rx_data_buf = Arc::new(Mutex::new(Vec::new()));

    led_tx.send((LedMode::BleSearching, None))?;

    // We must init the device because we deinit it when we're done
    BLEDevice::init();
    let ble_device = BLEDevice::take();
    ble_device.set_preferred_mtu(u16::try_from(BLE_ATT_MTU_MAX).unwrap())?;

    let ble_advertising = ble_device.get_advertising();
    let server = ble_device.get_server();

    server.advertise_on_disconnect(false);

    server.on_connect({
        let cancel = cancel.clone();

        move |server, desc| {
            info!("on_connect: {desc:?}");
            server
                .update_conn_params(desc.conn_handle(), 24, 48, 0, 60)
                .unwrap();
            if let Err(err) = connected_tx.send(desc.conn_handle()) {
                error!("failed to send connect: {err}");
                cancel.store(true, Ordering::Relaxed);
            }
        }
    });

    server.on_disconnect({
        let cancel = cancel.clone();

        move |_, reason| {
            info!("on_disconnect: {reason:?}");
            cancel.store(true, Ordering::Relaxed);
        }
    });

    let service = server.create_service(service_uuid);

    let state = service.lock().create_characteristic(
        STATE_ID,
        NimbleProperties::NOTIFY | NimbleProperties::WRITE_NO_RSP,
    );
    state.lock().on_write({
        let cancel = cancel.clone();

        move |args| {
            let recv_data = args.recv_data();
            info!("got state write: {}", hex::encode(recv_data));

            if recv_data.len() != 1 {
                error!("incorrect state length for recv data: {recv_data:?}");
                cancel.store(true, Ordering::Relaxed);
                args.reject();
                return;
            }

            if let Err(err) = ready_tx.send(recv_data[0] == 0x01) {
                error!("failed to send ready: {err}");
                cancel.store(true, Ordering::Relaxed);
            }
        }
    });

    let client_server = service
        .lock()
        .create_characteristic(CLIENT2SERVER_ID, NimbleProperties::WRITE_NO_RSP);
    client_server.lock().on_write({
        let cancel = cancel.clone();
        let rx_data_buf = rx_data_buf.clone();

        move |args| {
            info!(
                "got client to server write: {}",
                hex::encode(args.recv_data())
            );

            rx_data_buf.lock().push(args.recv_data().to_vec());

            if args.recv_data()[0] == 0x00 {
                if let Err(err) = data_done_tx.send(()) {
                    error!("could not send data done: {err}");
                    cancel.store(true, Ordering::Relaxed);
                }
            }
        }
    });

    let server_client = service
        .lock()
        .create_characteristic(SERVER2CLIENT_ID, NimbleProperties::NOTIFY);

    let ident = service
        .lock()
        .create_characteristic(IDENT_ID, NimbleProperties::READ);
    ident.lock().set_value(&ble_ident);

    ble_advertising.lock().set_data(
        BLEAdvertisementData::new()
            .name("mdoc reader")
            .add_service_uuid(service_uuid),
    )?;
    ble_advertising.lock().start()?;

    if let Err(err) = exchange_data(
        usb_serial,
        led_tx,
        server,
        server_client,
        service_uuid_bytes,
        &payload,
        cancel,
        connected_rx,
        ready_rx,
        rx_data_buf,
        data_done_rx,
    ) {
        error!("could not exchange data: {err}");

        let mut buf = [0u8; 64];
        let resp = DeviceResponse::BleEngagementFailed {
            service_uuid: service_uuid_bytes,
        };
        let resp = serialize_response(&mut buf, &resp)?;
        blocking_write_all(usb_serial, resp)?;
        wait_for_ack(usb_serial)?;

        led_tx.send((
            LedMode::BleFailed,
            Some(Instant::now() + Duration::from_secs(5)),
        ))?;
    } else {
        led_tx.send((
            LedMode::BleCompleted,
            Some(Instant::now() + Duration::from_secs(2)),
        ))?;
    }

    state.lock().set_value(&[0x02]).notify();

    info!("shutting down bluetooth stack");
    BLEDevice::deinit_full()?;

    info!(
        "finished peripheral data exchange in {} seconds",
        started_at.elapsed().as_secs()
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn exchange_data(
    usb_serial: &mut UsbSerialDriver<'_>,
    led_tx: &SyncSender<(LedMode, Option<Instant>)>,
    server: &mut BLEServer,
    server_client: Arc<Mutex<BLECharacteristic>>,
    service_uuid: [u8; 16],
    payload: &[u8],
    cancel: Arc<AtomicBool>,
    connected_rx: Receiver<u16>,
    ready_rx: Receiver<bool>,
    recv_data_buf: Arc<Mutex<Vec<Vec<u8>>>>,
    data_done_rx: Receiver<()>,
) -> eyre::Result<()> {
    led_tx.send((LedMode::BleWaiting, None))?;

    let conn_handle = connected_rx
        .recv_timeout(Duration::from_secs(30))
        .wrap_err("connected_rx recv_timeout")?;
    info!("connected conn_handle={conn_handle}");

    let ready = ready_rx
        .recv_timeout(Duration::from_secs(10))
        .wrap_err("ready_rx recv_timeout")?;
    info!("ready {ready}");
    if !ready {
        eyre::bail!("device did not indicate ready");
    }

    let mtu = server
        .connections()
        .find(|conn| conn.conn_handle() == conn_handle)
        .ok_or_eyre("missing conn_handle connection")?
        .mtu();
    let mut buf = Vec::with_capacity(usize::from(mtu - 3));
    info!("alloc buffer for mtu={mtu}");

    led_tx.send((LedMode::BleExchanging, None))?;

    // mtu minus 4 for the additional byte indicating if there are more messages
    let mut chunks = payload.chunks(usize::from(mtu - 4)).peekable();
    while let Some(chunk) = chunks.next() {
        if cancel.load(Ordering::Relaxed) {
            eyre::bail!("cancelled while notifying");
        }

        buf.clear();
        buf.push(if chunks.peek().is_some() { 0x01 } else { 0x00 });
        buf.extend_from_slice(chunk);

        info!("notifying chunk len={}, more={:02x}", buf.len(), buf[0]);
        server_client.lock().set_value(&buf).notify();
        esp_idf_hal::task::do_yield();
    }

    info!("finished sending payload, waiting for data");
    let mut attempts = 0;
    loop {
        if cancel.load(Ordering::Relaxed) {
            eyre::bail!("cancelled while waiting");
        }

        if data_done_rx.recv_timeout(Duration::from_secs(1)).is_ok() {
            break;
        }

        if attempts > 60 {
            eyre::bail!("too many attempts");
        }

        attempts += 1;
    }

    let mut buf = vec![0u8; usize::from(mtu) * 2];
    for chunk in recv_data_buf.lock().iter() {
        let complete = chunk[0] == 0x00;

        let resp = crate::DeviceResponse::BleEngagementData {
            service_uuid,
            payload: &chunk[1..],
            complete,
        };
        info!("writing chunk, len {}, complete {complete}", chunk.len());
        let data = crate::serialize_response(&mut buf, &resp)
            .wrap_err("error serializing ble engagement data")?;
        blocking_write_all(usb_serial, data).wrap_err("error writing ble engagement data")?;
        wait_for_ack(usb_serial).wrap_err("error waiting for ble engagement data ack")?;

        if complete {
            break;
        }
    }

    info!("completed exchange!");

    Ok(())
}
