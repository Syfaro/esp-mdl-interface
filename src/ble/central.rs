use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::SyncSender,
    },
    time::{Duration, Instant},
};

use esp_idf_hal::{sys::BLE_ATT_MTU_MAX, task::block_on, usb_serial::UsbSerialDriver};
use esp32_nimble::{
    BLEDevice, BLEScan,
    utilities::{BleUuid, mutex::Mutex},
    uuid128,
};
use eyre::OptionExt;
use log::{error, info};

use crate::{
    DeviceResponse, LedMode, ble::RemovableSender, blocking_write_all, serialize_response,
    wait_for_ack,
};

const STATE_ID: BleUuid = uuid128!("00000001-A123-48CE-896B-4C76973373E6");
const CLIENT2SERVER_ID: BleUuid = uuid128!("00000002-A123-48CE-896B-4C76973373E6");
const SERVER2CLIENT_ID: BleUuid = uuid128!("00000003-A123-48CE-896B-4C76973373E6");

pub fn connect_client(
    usb_serial: &mut UsbSerialDriver<'_>,
    led_tx: &SyncSender<(LedMode, Option<Instant>)>,
    service_uuid_bytes: [u8; 16],
    payload: Vec<u8>,
) -> eyre::Result<()> {
    let started_at = Instant::now();
    let service_uuid = BleUuid::from_uuid128(service_uuid_bytes);
    info!("starting central data exchange: {service_uuid}");

    // Set when any error occurs, needs to be checked before any blocking op
    let cancel = Arc::new(AtomicBool::new(false));

    // // Data exchange between bluetooth callbacks and main thread code
    // let (connected_tx, connected_rx) = RemovableSender::<u16>::new();
    // let (ready_tx, ready_rx) = RemovableSender::<bool>::new();
    // // let (data_tx, data_rx) = mpsc::sync_channel::<Vec<u8>>(4);

    let res = block_on(async {
        BLEDevice::init();

        let (data_done_tx, data_done_rx) = RemovableSender::<()>::new();
        let rx_data_buf = Arc::new(Mutex::new(Vec::<Vec<u8>>::new()));

        led_tx.send((LedMode::BleSearching, None))?;

        let ble_device = BLEDevice::take();
        ble_device.set_preferred_mtu(u16::try_from(BLE_ATT_MTU_MAX).unwrap())?;

        let mut ble_scan = BLEScan::new();
        let device = ble_scan
            .active_scan(true)
            .interval(100)
            .window(99)
            .start(ble_device, 30_000, |device, data| {
                data.service_uuids()
                    .any(|found_service_uuid| found_service_uuid == service_uuid)
                    .then_some(*device)
            })
            .await?
            .ok_or_eyre("could not find device with service")?;
        info!("found device {}", device.addr());

        let mut client = ble_device.new_client();

        led_tx.send((LedMode::BleWaiting, None))?;

        client.on_connect(|client| client.update_conn_params(120, 120, 0, 60).unwrap());
        client.on_disconnect({
            let cancel = cancel.clone();
            move |_| {
                cancel.store(true, Ordering::Relaxed);
            }
        });

        client.connect(&device.addr()).await?;

        let mtu = usize::from(client.desc()?.mtu());
        info!("got mtu: {mtu}");

        let service = client.get_service(service_uuid).await?;

        let state = service.get_characteristic(STATE_ID).await?;
        state
            .on_notify(|data| {
                info!("got state notify: {}", hex::encode(data));
            })
            .subscribe_notify(true)
            .await?;

        state.write_value(&[0x01], false).await?;

        let server_client = service.get_characteristic(SERVER2CLIENT_ID).await?;
        server_client
            .on_notify({
                let rx_data_buf = rx_data_buf.clone();

                move |data| {
                    info!("got server_client notify: {}", hex::encode(data));
                    rx_data_buf.lock().push(data.to_vec());

                    if data[0] == 0x00 {
                        if let Err(err) = data_done_tx.send(()) {
                            error!("could not sent data done tx: {err}");
                        }
                    }
                }
            })
            .subscribe_notify(true)
            .await?;

        led_tx.send((LedMode::BleExchanging, None))?;

        let mut buf: Vec<u8> = Vec::with_capacity(mtu - 3);
        let mut chunks = payload.chunks(mtu - 1).peekable();

        let client_server = service.get_characteristic(CLIENT2SERVER_ID).await?;
        while let Some(chunk) = chunks.next() {
            if cancel.load(Ordering::Relaxed) {
                eyre::bail!("cancelled while writing");
            }

            buf.clear();

            buf.push(if chunks.peek().is_none() { 0x00 } else { 0x01 });
            buf.extend_from_slice(chunk);

            client_server.write_value(&buf, false).await?;
            esp_idf_hal::task::do_yield();
        }

        let mut attempts = 0;
        loop {
            if attempts > 60 {
                eyre::bail!("ran out of time waiting for data to be ready");
            }

            if cancel.load(Ordering::Relaxed) {
                eyre::bail!("cancelled while waiting for recv");
            }

            if data_done_rx.recv_timeout(Duration::from_secs(1)).is_ok() {
                break;
            }

            attempts += 1;
        }

        let mut buf = vec![0u8; mtu * 2];
        for chunk in rx_data_buf.lock().iter() {
            let complete = chunk[0] == 0x00;

            let resp = crate::DeviceResponse::BleEngagementData {
                service_uuid: service_uuid_bytes,
                payload: &chunk[1..],
                complete,
            };
            info!("writing chunk, len {}, complete {complete}", chunk.len());
            let data = crate::serialize_response(&mut buf, &resp)?;
            blocking_write_all(usb_serial, data)?;
            wait_for_ack(usb_serial)?;

            if complete {
                break;
            }
        }

        client.disconnect()?;

        Ok::<_, eyre::Report>(())
    });

    info!("shutting down bluetooth stack");
    BLEDevice::deinit_full()?;

    match res {
        Ok(_) => {
            info!(
                "finished peripheral data exchange in {} seconds",
                started_at.elapsed().as_secs()
            );

            led_tx.send((
                LedMode::BleCompleted,
                Some(Instant::now() + Duration::from_secs(2)),
            ))?;
        }
        Err(err) => {
            error!("could not exchange data: {err:?}");

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
        }
    }

    Ok(())
}
