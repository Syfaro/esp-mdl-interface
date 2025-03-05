use std::{
    convert::Infallible,
    sync::{
        LazyLock, Mutex,
        mpsc::{self, SyncSender},
    },
    time::{Duration, Instant},
};

use embedded_io::Write;
use esp_idf_hal::{
    delay::TickType,
    gpio::{Input, PinDriver},
    prelude::*,
    spi::{self, Operation, SpiDeviceDriver, SpiDriver, config::DriverConfig},
    task::thread::ThreadSpawnConfiguration,
    usb_serial::{self, UsbSerialDriver},
};
use eyre::Context;
use log::{debug, error, info, warn};
use nfc::ndef::BleRole;
use palette::{Srgb, rgb::Rgb};
use pn532::nb;
use postcard::accumulator::{CobsAccumulator, FeedResult};
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use serde::{Deserialize, Serialize};
use smart_led_effects::strip::EffectIterator;
use ws2812_esp32_rmt_driver::Ws2812Esp32Rmt;

mod ble;
mod nfc;

const USB_SERIAL_BUFFER_SIZE: usize = 2048;

#[derive(Debug, Deserialize)]
pub enum HostRequest {
    Ping,
    Ack,
    StartBluetoothPeripheral {
        service_uuid: [u8; 16],
        ble_ident: [u8; 16],
        payload: Vec<u8>,
    },
    StartBluetoothCentral {
        service_uuid: [u8; 16],
        payload: Vec<u8>,
    },
}

#[derive(Debug, Serialize)]
pub enum DeviceResponse<'a> {
    Ack,
    NfcEngagementStarted {
        uid: [u8; 4],
    },
    NfcEngagementFailed {
        uid: [u8; 4],
    },
    NfcEngagementData {
        uid: [u8; 4],
        use_central: bool,
        service_uuid: [u8; 16],
        device_engagement: &'a [u8],
        handover_select: &'a [u8],
        handover_request: Option<&'a [u8]>,
    },
    BleEngagementFailed {
        service_uuid: [u8; 16],
    },
    BleEngagementData {
        service_uuid: [u8; 16],
        payload: &'a [u8],
        complete: bool,
    },
}

enum LedMode {
    Initializing,
    Waiting,
    FatalError,

    NfcReading,
    NfcCompleted,
    NfcFailed,

    BleSearching,
    BleWaiting,
    BleExchanging,
    BleCompleted,
    BleFailed,
}

struct LedController<'a> {
    ws2812: Ws2812Esp32Rmt<'a>,
    reset_at: Option<Instant>,
    solid_color: Option<Rgb<Srgb, u8>>,
    effect: Box<dyn EffectIterator + Send + Sync>,
}

impl<'a> LedController<'a> {
    fn new(ws2812: Ws2812Esp32Rmt<'a>) -> Self {
        Self {
            ws2812,
            reset_at: None,
            solid_color: None,
            effect: Box::new(smart_led_effects::strip::Strobe::new(
                LED_COUNT,
                Some(Rgb::new(255, 0, 0)),
                Duration::from_secs(1),
                None,
            )),
        }
    }

    fn set_mode(&mut self, mode: LedMode) {
        use smart_led_effects::strip::*;

        self.solid_color = None;

        self.effect = match mode {
            LedMode::Initializing => Box::new(Strobe::new(
                LED_COUNT,
                Some(Rgb::new(191, 0, 255)),
                Duration::from_secs(1),
                None,
            )),
            LedMode::Waiting => {
                self.solid_color = Some(Rgb::new(0, 0, 0));
                return;
            }
            LedMode::FatalError => Box::new(Strobe::new(
                LED_COUNT,
                Some(Rgb::new(255, 0, 0)),
                Duration::from_secs(1),
                Some(0.1),
            )),
            LedMode::NfcReading => Box::new(Strobe::new(
                LED_COUNT,
                Some(Rgb::new(0, 0, 128)),
                Duration::from_secs(1),
                None,
            )),
            LedMode::NfcCompleted => {
                self.solid_color = Some(Rgb::new(0, 128, 0));
                return;
            }
            LedMode::NfcFailed | LedMode::BleFailed => Box::new(Strobe::new(
                LED_COUNT,
                Some(Rgb::new(255, 0, 0)),
                Duration::from_secs(1),
                Some(0.05),
            )),
            LedMode::BleSearching => {
                Box::new(Breathe::new(LED_COUNT, Some(Rgb::new(0.0, 0.0, 0.5)), None))
            }
            LedMode::BleWaiting => {
                Box::new(Cylon::new(LED_COUNT, Rgb::new(0, 0, 128), Some(3), None))
            }
            LedMode::BleExchanging => {
                self.solid_color = Some(Rgb::new(0, 0, 128));
                return;
            }
            LedMode::BleCompleted => Box::new(Strobe::new(
                LED_COUNT,
                Some(Rgb::new(0, 255, 0)),
                Duration::from_secs(1),
                None,
            )),
        }
    }

    fn next(&mut self) {
        if matches!(self.reset_at, Some(reset_at) if reset_at < Instant::now()) {
            self.set_mode(LedMode::Waiting);
        }

        if let Some(color) = self.solid_color {
            self.ws2812
                .write_nocopy(core::iter::repeat_n(color.into_components(), LED_COUNT))
                .unwrap();
            return;
        }

        let led_values = self.effect.next().unwrap();

        self.ws2812
            .write_nocopy(led_values.iter().map(|value| value.into_components()))
            .unwrap();
    }
}

static SERIAL_BUF: LazyLock<Mutex<ConstGenericRingBuffer<u8, 256>>> =
    LazyLock::new(|| Mutex::new(ConstGenericRingBuffer::new()));

const LED_COUNT: usize = 12;

fn main() -> eyre::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    // Used for debugging, to set a trigger on device boot.
    let mut boot_pin = PinDriver::output(peripherals.pins.gpio0)?;
    boot_pin.set_low()?;
    esp_idf_hal::delay::Ets::delay_ms(1);
    boot_pin.set_high()?;
    esp_idf_hal::delay::Ets::delay_ms(1);
    boot_pin.set_low()?;
    drop(boot_pin);

    info!("hello, world!");

    let led_pin = peripherals.pins.gpio1;
    let rmt_channel = peripherals.rmt.channel0;

    let (led_tx, led_rx) = mpsc::sync_channel::<(LedMode, Option<Instant>)>(1);
    led_tx.send((LedMode::Initializing, None))?;

    let ws2812 = Ws2812Esp32Rmt::new(rmt_channel, led_pin)?;
    let mut led_controller = LedController::new(ws2812);

    ThreadSpawnConfiguration {
        name: Some(b"LED\0"),
        priority: 1,
        ..Default::default()
    }
    .set()?;

    std::thread::spawn(move || {
        loop {
            if let Ok((mode, reset_at)) = led_rx.try_recv() {
                led_controller.set_mode(mode);
                led_controller.reset_at = reset_at;
            }

            led_controller.next();

            esp_idf_hal::delay::FreeRtos::delay_ms(1000 / 60);
        }
    });

    let mut usb_serial = UsbSerialDriver::new(
        peripherals.usb_serial,
        peripherals.pins.gpio12,
        peripherals.pins.gpio13,
        &usb_serial::UsbSerialConfig::new()
            .rx_buffer_size(USB_SERIAL_BUFFER_SIZE)
            .tx_buffer_size(USB_SERIAL_BUFFER_SIZE),
    )?;

    let spi_driver = SpiDriver::new(
        peripherals.spi2,
        peripherals.pins.gpio19,
        peripherals.pins.gpio18,
        Some(peripherals.pins.gpio20),
        &DriverConfig::new(),
    )?;

    let mut spi_pn532 = SpiDeviceDriver::new(
        &spi_driver,
        Some(peripherals.pins.gpio21),
        &spi::config::Config::new()
            .baudrate(MegaHertz(2).into())
            .bit_order(spi::config::BitOrder::LsbFirst)
            .data_mode(spi::config::MODE_0)
            .allow_pre_post_delays(true)
            .cs_pre_delay_us(16),
    )?;

    let irq_pin = PinDriver::input(peripherals.pins.gpio2)?;

    // In order to wake the PN532 from sleep over SPI, we must hold the CS line
    // low for around 2ms then execute a command. Any command works.
    spi_pn532.transaction(&mut [
        Operation::Write(&[0x55]),
        Operation::DelayNs(2_000_000),
        Operation::Write(&[
            0x01, 0x00, 0x00, 0xFF, 0x05, 0xFB, 0xD4, 0x14, 0x01, 0x14, 0x01, 0x02, 0x00,
        ]),
    ])?;

    // After it wakes up, it'll hold IRQ low until the next command.
    while irq_pin.is_high() {
        esp_idf_hal::task::do_yield();
    }

    let irq_input = InfallibleInput(irq_pin);
    let interface = pn532::spi::SPIInterfaceWithIrq {
        spi: spi_pn532,
        irq: irq_input,
    };
    let mut pn532 = pn532::Pn532::<_, _, 273>::new(interface, SysTimer::new());

    // Sometimes the first request fails, so just keep trying until it works.
    loop {
        match pn532.process(
            &pn532::Request::new(pn532::requests::Command::SAMConfiguration, [
                0x01, 0x14, 0x01,
            ]),
            0,
            Duration::from_secs(2),
        ) {
            Ok(_) => {
                info!("configured sam");
                break;
            }
            Err(err) => error!("error configuring sam: {err:?}"),
        }

        esp_idf_hal::delay::FreeRtos::delay_ms(10);
    }

    if let Err(err) = process(&mut usb_serial, pn532, led_tx.clone()) {
        error!("main task exited: {err:?}");
        led_tx.send((LedMode::FatalError, None))?;
    }

    Ok(())
}

fn process(
    usb_serial: &mut UsbSerialDriver<'_>,
    pn532: nfc::Pn532,
    led_tx: SyncSender<(LedMode, Option<Instant>)>,
) -> eyre::Result<()> {
    let mut engagement_handler = nfc::EngagementHandler::new(pn532)?;

    let mut buf = [0u8; 2048];
    let mut acc: CobsAccumulator<2084> = CobsAccumulator::new();

    let mut next_nfc_read = Instant::now();

    led_tx.send((LedMode::Waiting, None))?;

    loop {
        let mut serial_buf = SERIAL_BUF.lock().unwrap();
        if !serial_buf.is_empty() {
            for byte in serial_buf.drain() {
                info!("reading extra byte off serial buf: {byte:02x}");

                match acc.feed::<HostRequest>(&[byte]) {
                    FeedResult::Consumed | FeedResult::OverFull(_) | FeedResult::DeserError(_) => {
                        continue;
                    }
                    FeedResult::Success { data, .. } => {
                        match handle_request(usb_serial, &led_tx, data) {
                            Ok(_) => info!("handled request!"),
                            Err(err) => error!("failed to handle request: {err:?}"),
                        }
                    }
                }
            }
        }
        drop(serial_buf);

        while let Ok(len) = usb_serial.read(&mut buf, TickType::new_millis(100).into()) {
            if len == 0 {
                break;
            }

            let buf = &buf[..len];
            let mut window = buf;
            info!("read more data: {}", hex::encode(buf));

            'cobs: while !window.is_empty() {
                window = match acc.feed::<HostRequest>(window) {
                    FeedResult::Consumed => break 'cobs,
                    FeedResult::OverFull(new_window) => new_window,
                    FeedResult::DeserError(new_window) => new_window,
                    FeedResult::Success { data, remaining } => {
                        match handle_request(usb_serial, &led_tx, data) {
                            Ok(_) => info!("handled request!"),
                            Err(err) => error!("failed to handle request: {err:?}"),
                        }
                        remaining
                    }
                }
            }
        }

        if next_nfc_read > Instant::now() {
            info!("last nfc event was too recent, skipping check");
            continue;
        }

        match engagement_handler.poll_14a() {
            Ok(Some(uid)) => {
                info!("got tag uid: {}", hex::encode(uid));

                led_tx.send((LedMode::NfcReading, None))?;

                let resp = DeviceResponse::NfcEngagementStarted { uid };
                let resp = serialize_response(&mut buf, &resp)?;
                blocking_write_all(usb_serial, resp)?;
                wait_for_ack(usb_serial)?;

                match engagement_handler.get_engagement() {
                    Ok(conn) => {
                        info!("got connection data: {conn:?}");

                        let resp = DeviceResponse::NfcEngagementData {
                            uid,
                            use_central: matches!(
                                conn.ble_role,
                                BleRole::Central
                                    | BleRole::PeripheralCentral
                                    | BleRole::CentralPeripheral
                            ),
                            service_uuid: conn.service_uuid,
                            device_engagement: &conn.device_engagement,
                            handover_select: &conn.handover_select,
                            handover_request: conn.handover_request.as_deref(),
                        };
                        let resp = serialize_response(&mut buf, &resp)?;
                        blocking_write_all(usb_serial, resp)?;
                        wait_for_ack(usb_serial)?;
                        next_nfc_read = Instant::now() + Duration::from_secs(15);

                        led_tx.send((
                            LedMode::NfcCompleted,
                            Some(Instant::now() + Duration::from_secs(3)),
                        ))?;

                        info!("sent engagement data");
                    }
                    Err(err) => {
                        warn!("failed to get engagement data: {err:?}");

                        let resp = DeviceResponse::NfcEngagementFailed { uid };
                        let resp = serialize_response(&mut buf, &resp)?;
                        blocking_write_all(usb_serial, resp)?;
                        wait_for_ack(usb_serial)?;
                        next_nfc_read = Instant::now() + Duration::from_millis(250);

                        // It's easy to get a communication error because the
                        // device isn't close enough yet, so don't show the
                        // user this.
                        if !matches!(err, nfc::EngagementError::Pn532Communication(_)) {
                            led_tx.send((
                                LedMode::NfcFailed,
                                Some(Instant::now() + Duration::from_secs(5)),
                            ))?;
                        }

                        info!("sent engagement failure");
                    }
                }
            }
            Ok(None) => {
                debug!("no tag detected");
            }
            Err(nfc::EngagementError::Pn532(nfc::Pn532Error(pn532::Error::TimeoutResponse))) => {
                debug!("timeout polling pn532");
            }
            Err(err) => {
                error!("could not poll pn532: {err:?}");
            }
        }
    }
}

fn wait_for_ack(usb_serial: &mut UsbSerialDriver<'_>) -> eyre::Result<()> {
    let start = Instant::now();

    let mut buf = [0u8; 16];
    let mut acc: CobsAccumulator<16> = CobsAccumulator::new();

    info!("waiting for serial ack");

    loop {
        if start.elapsed() > Duration::from_millis(250) {
            eyre::bail!("timeout waiting for host ack");
        }

        while let Ok(len) = usb_serial.read(&mut buf, TickType::new_millis(50).into()) {
            if len == 0 {
                break;
            }

            let buf = &buf[..len];
            let mut window = buf;
            info!("read more data: {}", hex::encode(buf));

            'cobs: while !window.is_empty() {
                window = match acc.feed::<HostRequest>(window) {
                    FeedResult::Consumed => break 'cobs,
                    FeedResult::OverFull(new_window) => new_window,
                    FeedResult::DeserError(new_window) => new_window,
                    FeedResult::Success { data, remaining } => {
                        SERIAL_BUF.lock().unwrap().extend(remaining.iter().copied());

                        if matches!(data, HostRequest::Ack) {
                            return Ok(());
                        } else {
                            eyre::bail!("got unexpected host request");
                        }
                    }
                }
            }
        }
    }
}

fn blocking_write_all(usb_serial: &mut UsbSerialDriver<'_>, mut data: &[u8]) -> eyre::Result<()> {
    info!("attempting write of data: {}", hex::encode(data));

    while !data.is_empty() {
        // Never try to write more than half the buffer size. If the data is
        // greater than the buffer size it will never write, always returning
        // that it wrote 0 bytes.
        let len = usb_serial
            .write(
                &data[..std::cmp::min(data.len(), USB_SERIAL_BUFFER_SIZE / 2)],
                TickType::new_millis(1_000).into(),
            )
            .wrap_err("error writing data to usb serial")?;
        info!("wrote {len} bytes, remaining: {}", hex::encode(data));
        data = &data[len..];
        esp_idf_hal::task::do_yield();
    }

    Ok(())
}

fn serialize_response<'a>(
    buf: &'a mut [u8],
    response: &'a DeviceResponse,
) -> eyre::Result<&'a [u8]> {
    let data = postcard::serialize_with_flavor::<_, _, _>(
        response,
        postcard::ser_flavors::Cobs::try_new(postcard::ser_flavors::Slice::new(buf))
            .wrap_err_with(|| eyre::eyre!("failed to serialize response: {response:?}"))?,
    )?;

    Ok(data)
}

fn handle_request(
    usb_serial: &mut UsbSerialDriver<'_>,
    led_tx: &SyncSender<(LedMode, Option<Instant>)>,
    request: HostRequest,
) -> eyre::Result<()> {
    match request {
        HostRequest::Ping => {
            info!("got ping");
            let mut resp = [0u8; 16];
            serialize_response(&mut resp, &DeviceResponse::Ack)?;
            usb_serial.write_all(&resp)?;
        }
        HostRequest::Ack => {
            warn!("got unexpected ack");
        }
        HostRequest::StartBluetoothPeripheral {
            service_uuid,
            ble_ident,
            payload,
        } => {
            ble::serve_peripheral(usb_serial, led_tx, service_uuid, ble_ident, payload)?;
        }
        HostRequest::StartBluetoothCentral {
            service_uuid,
            payload,
        } => {
            ble::connect_client(usb_serial, led_tx, service_uuid, payload)?;
        }
    }

    Ok(())
}

pub struct InfallibleInput<'a, T>(PinDriver<'a, T, Input>)
where
    T: esp_idf_hal::gpio::Pin;

impl<T> embedded_hal::digital::ErrorType for InfallibleInput<'_, T>
where
    T: esp_idf_hal::gpio::Pin,
{
    type Error = Infallible;
}

impl<T> embedded_hal::digital::InputPin for InfallibleInput<'_, T>
where
    T: esp_idf_hal::gpio::Pin,
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_low())
    }
}

pub struct SysTimer {
    start: Instant,
    duration: Duration,
}

impl SysTimer {
    pub fn new() -> SysTimer {
        SysTimer {
            start: Instant::now(),
            duration: Duration::from_millis(0),
        }
    }
}

impl Default for SysTimer {
    fn default() -> SysTimer {
        SysTimer::new()
    }
}

impl pn532::CountDown for SysTimer {
    type Time = Duration;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        self.start = Instant::now();
        self.duration = count.into();
    }

    fn wait(&mut self) -> nb::Result<(), Infallible> {
        if self.start.elapsed() >= self.duration {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
