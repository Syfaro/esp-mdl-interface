use std::time::Duration;

use esp_idf_hal::spi::{SpiDeviceDriver, SpiDriver, SpiError};
use log::{error, info};
use ndef::NdefWrite;
use packed_struct::PrimitiveEnum;
use pn532::requests::{BorrowedRequest, Command};
use serde::Serialize;
use thiserror::Error;

pub mod ndef;

#[derive(Debug, Error)]
pub enum EngagementError {
    #[error(transparent)]
    Pn532(#[from] Pn532Error),
    #[error("pn532 communication error {0}")]
    Pn532Communication(u8),
    #[error("got unexpected bytes")]
    UnexpectedBytes { expected: Vec<u8>, got: Vec<u8> },
    #[error("wrong data length")]
    WrongDataLength { expected: usize, got: usize },
    #[error("missing required data {0}")]
    MissingRequiredData(&'static str),
    #[error(transparent)]
    Packing(#[from] packed_struct::PackingError),
}

pub type Pn532<'a> = pn532::Pn532<
    pn532::spi::SPIInterfaceWithIrq<
        SpiDeviceDriver<'a, &'a SpiDriver<'a>>,
        InfallibleInput<'a, esp_idf_hal::gpio::Gpio2>,
    >,
    crate::SysTimer,
    273,
>;
type EngagementResult<T> = Result<T, EngagementError>;

macro_rules! ensure_length {
    ($expected:expr, $got:expr) => {
        if $expected != $got {
            return Err(crate::nfc::EngagementError::WrongDataLength {
                expected: $expected,
                got: $got,
            });
        }
    };
}

macro_rules! get_byte {
    ($iter:expr, $idx:expr, $name:expr) => {
        match $iter.get($idx) {
            Some(byte) => *byte,
            None => return Err(crate::nfc::EngagementError::MissingRequiredData($name)),
        }
    };
}

macro_rules! get_bytes {
    ($bytes:expr, $count:expr) => {{
        let bytes: [u8; $count] = match $bytes.try_into() {
            Ok(bytes) => bytes,
            Err(_) => {
                return Err(crate::nfc::EngagementError::WrongDataLength {
                    expected: $count,
                    got: $bytes.len(),
                });
            }
        };
        bytes
    }};
}

macro_rules! ensure_bytes {
    ($expected:expr, $got:expr) => {
        if $expected != $got {
            return Err(crate::nfc::EngagementError::UnexpectedBytes {
                expected: $expected.to_vec(),
                got: $got.to_vec(),
            });
        }
    };
}

use crate::InfallibleInput;

pub(crate) use {ensure_bytes, get_byte, get_bytes};

pub struct EngagementHandler<'a> {
    pn532: Pn532<'a>,
}

#[derive(Debug, Serialize)]
pub struct NegotiatedConnection {
    pub ble_role: ndef::BleRole,
    pub service_uuid: [u8; 16],
    pub device_engagement: Vec<u8>,
    pub handover_select: Vec<u8>,
    pub handover_request: Option<Vec<u8>>,
}

impl<'a> EngagementHandler<'a> {
    const DEFAULT_TIMEOUT: Duration = Duration::from_millis(500);

    const NDEF_AID: [u8; 7] = [0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01];

    pub fn new(pn532: Pn532<'a>) -> EngagementResult<Self> {
        let mut handler = Self { pn532 };

        handler
            .pn532
            .process(
                &pn532::Request::sam_configuration(pn532::requests::SAMMode::Normal, true),
                0,
                Self::DEFAULT_TIMEOUT,
            )
            .map_err(Pn532Error::from)?;

        handler.reset_settings()?;

        Ok(handler)
    }

    fn reset_settings(&mut self) -> EngagementResult<()> {
        // reset ending transmission bits
        self.write_register(0x633D, 0x07, 0x00)?;

        // handle crc tx
        self.write_register(0x6302, 0x80, 0x80)?;
        // handle crc rx
        self.write_register(0x6303, 0x80, 0x80)?;

        // handle parity
        self.write_register(0x630D, 0x10, 0x00)?;

        // deactivate crypto1
        self.write_register(0x6338, 0x08, 0x00)?;

        Ok(())
    }

    fn read_register(&mut self, address: u16) -> EngagementResult<u8> {
        info!(
            "read_register address={}",
            hex::encode(address.to_be_bytes())
        );

        let request = pn532::Request::new(Command::ReadRegister, address.to_be_bytes());
        let response = self
            .pn532
            .process(&request, 1, Self::DEFAULT_TIMEOUT)
            .map_err(Pn532Error::from)?;
        info!("read_register response={}", hex::encode(response));

        Ok(response[0])
    }

    fn write_register(&mut self, address: u16, mask: u8, value: u8) -> EngagementResult<()> {
        let value = if mask != 0xFF {
            let current_value = self.read_register(address)?;
            let new_value = (value & mask) | (current_value & !mask);
            if new_value == current_value {
                return Ok(());
            }
            new_value
        } else {
            value
        };

        let mut data = Vec::with_capacity(3);
        data.extend_from_slice(&address.to_be_bytes());
        data.push(value);
        info!(
            "write_register address={}, value={value:02x}, data={}",
            hex::encode(address.to_be_bytes()),
            hex::encode(&data)
        );

        let request = BorrowedRequest::new(Command::WriteRegister, &data);
        self.pn532
            .process_no_response(request, Self::DEFAULT_TIMEOUT)
            .map_err(Pn532Error::from)?;
        info!("write_register ok");

        Ok(())
    }

    /// Process a request, automatically removing the status byte and returning
    /// if there were any communication errors.
    fn process_rf<'p>(
        &mut self,
        request: impl Into<BorrowedRequest<'p>>,
        response_len: usize,
        timeout: Duration,
    ) -> EngagementResult<&[u8]> {
        // automatically increase response length to account for the status byte
        let response = self
            .pn532
            .process(request, response_len + 1, timeout)
            .map_err(Pn532Error::from)?;
        let status = response[0] & 0b00111111;
        if status != 0x00 {
            return Err(EngagementError::Pn532Communication(status));
        }

        Ok(&response[1..])
    }

    pub fn poll_14a(&mut self) -> EngagementResult<Option<[u8; 4]>> {
        let resp = self
            .pn532
            .process(
                &pn532::Request::INLIST_ONE_ISO_A_TARGET,
                17,
                Duration::from_millis(100),
            )
            .map_err(Pn532Error::from)?;
        info!("got resp: {}", hex::encode(resp));

        if resp[0] == 0x00 {
            return Ok(None);
        }

        if resp[1] != 0x01 {
            info!("not 14a tag");
            return Ok(None);
        }

        if resp[5] != 0x04 {
            info!("uid was not 4 bytes");
            return Ok(None);
        }

        let uid_bytes: [u8; 4] = resp[6..10].try_into().unwrap();
        info!("got uid uid_bytes={}", hex::encode(uid_bytes));
        Ok(Some(uid_bytes))
    }

    pub fn get_engagement(&mut self) -> EngagementResult<NegotiatedConnection> {
        self.select_app(&Self::NDEF_AID)?;
        self.select_file(0xE103)?;

        let cc_data = self.read_file(0, 15)?;
        ensure_length!(cc_data.len(), 15);

        let ndef_file_id = Self::get_ndef_file_id_from_cc(cc_data)?;
        info!("ndef_file_id={ndef_file_id}");

        self.select_file(ndef_file_id)?;
        let mut ndef_file_data = self.read_file_ndef()?;
        info!("ndef_file_data={}", hex::encode(&ndef_file_data));

        let records =
            ndef::NdefRecordIterator::new(&ndef_file_data).collect::<Result<Vec<_>, _>>()?;

        let (records, handover_request) = if let Some(record) = records.iter().find(|record| {
            record.flags.type_name_format == ndef::TypeNameFormat::WellKnown
                && record.type_data == b"Tp"
        }) {
            let (_, service_parameter) = ndef::TnepServiceParameter::decode(record.payload_data)?;
            info!("got tnep service parameter: {service_parameter:?}");

            let wait_time = service_parameter.wait_time_ms();

            esp_idf_hal::delay::Delay::new_default().delay_ms(wait_time);

            let service_parameter_version = service_parameter.version;
            let mut service_select_data = Vec::new();
            ndef::TnepServiceSelect {
                service_name: service_parameter.service_name,
            }
            .ndef_write_data(&mut service_select_data, true, true)
            .unwrap();
            self.update_file_ndef(&service_select_data)?;

            esp_idf_hal::delay::Delay::new_default().delay_ms(wait_time);

            ndef_file_data = self.read_file_ndef()?;
            let (_, record) = ndef::RawNdefRecord::decode_single(&ndef_file_data)?;
            info!("got record: {record:?}");
            if record.flags.type_name_format != ndef::TypeNameFormat::WellKnown
                || record.type_data != b"Te"
            {
                return Err(EngagementError::MissingRequiredData("tnep status"));
            }
            match ndef::TnepStatus::decode(record.payload_data) {
                Ok((_, ndef::TnepStatus::Success)) => info!("tnep status success"),
                Ok((_, ndef::TnepStatus::Unknown(other))) => {
                    error!("unexpected tnep status code: {other:02x}");
                    return Err(EngagementError::MissingRequiredData("tnep success"));
                }
                Err(_) => return Err(EngagementError::MissingRequiredData("tnep status")),
            }

            let service_uuid = uuid::Uuid::new_v4();
            info!("service_uuid={service_uuid}");

            let mut handover_message_data = Vec::new();

            ndef::HandoverRequest {
                version: service_parameter_version,
                collision_resolution: ndef::CollisionResolution::new_rand(),
                alternative_carriers: vec![ndef::AlternativeCarrier {
                    carrier_power_state: ndef::CarrierPowerState::Active,
                    carrier_data_reference: b"0",
                    auxiliary_data_references: vec![b"mdocreader"],
                }],
            }
            .ndef_write_data(&mut handover_message_data, true, false)
            .unwrap();

            esp_idf_hal::delay::Delay::new_default().delay_ms(wait_time);

            let ble_role = [ndef::BleRole::PeripheralCentral.to_primitive()];
            let fields: Vec<(u8, &[u8])> = vec![(0x1C, &ble_role), (0x07, service_uuid.as_bytes())];
            ndef::CarrierConfigurationBle { fields }
                .ndef_write_data(&mut handover_message_data, false, false)
                .unwrap();

            ndef::ReaderEngagement { version: "1.0" }
                .ndef_write_data(&mut handover_message_data, false, true)
                .unwrap();

            self.update_file_ndef(&handover_message_data)?;

            esp_idf_hal::delay::Delay::new_default().delay_ms(wait_time);

            ndef_file_data = self.read_file_ndef()?;
            let records =
                ndef::NdefRecordIterator::new(&ndef_file_data).collect::<Result<Vec<_>, _>>()?;

            (records, Some(handover_message_data))
        } else {
            (records, None)
        };

        if let Some(record) = records.iter().find(|record| {
            record.flags.type_name_format == ndef::TypeNameFormat::WellKnown
                && record.type_data == b"Hs"
        }) {
            let (_, handover_select) = ndef::HandoverSelect::decode(record.payload_data)?;
            info!("got handover select record: {handover_select:?}");

            let Some(carrier_ble) = records
                .iter()
                .filter(|record| {
                    record.flags.type_name_format == ndef::TypeNameFormat::MediaType
                        && record.id_data
                            == Some(handover_select.alternative_carrier.carrier_data_reference)
                        && record.type_data == b"application/vnd.bluetooth.le.oob"
                })
                .find_map(|record| ndef::CarrierConfigurationBle::decode(record.payload_data).ok())
            else {
                return Err(EngagementError::MissingRequiredData(
                    "carrier configuration ble",
                ));
            };
            info!("got ble carrier confg: {carrier_ble:?}");

            let Some(device_engagement) = records.iter().find(|record| {
                record.flags.type_name_format == ndef::TypeNameFormat::External
                    && record.type_data == b"iso.org:18013:deviceengagement"
                    && handover_select
                        .alternative_carrier
                        .auxiliary_data_references
                        .iter()
                        .any(|id_data| Some(*id_data) == record.id_data)
            }) else {
                return Err(EngagementError::MissingRequiredData("device engagement"));
            };
            info!("got device engagement: {device_engagement:?}");

            return Ok(NegotiatedConnection {
                ble_role: carrier_ble
                    .role()
                    .ok_or(EngagementError::MissingRequiredData("ble role"))?,
                service_uuid: carrier_ble
                    .service_uuid()
                    .ok_or(EngagementError::MissingRequiredData("service uuid"))?,
                device_engagement: device_engagement.payload_data.to_vec(),
                handover_select: ndef_file_data,
                handover_request,
            });
        }

        Err(EngagementError::MissingRequiredData("no known records"))
    }

    fn check_status(data: &[u8]) -> EngagementResult<()> {
        if data.len() < 2 {
            return Err(EngagementError::MissingRequiredData(
                "status should have at least 2 bytes",
            ));
        }

        let status_bytes = get_bytes!(data[data.len() - 2..], 2);
        ensure_bytes!([0x90, 0x00], status_bytes);
        Ok(())
    }

    fn get_ndef_file_id_from_cc(cc_data: &[u8]) -> EngagementResult<u16> {
        let ndef_file_control_tlv = &cc_data[7..15];
        info!(
            "ndef_file_control_tlv={}",
            hex::encode(ndef_file_control_tlv)
        );
        ensure_bytes!([0x04, 0x06], ndef_file_control_tlv[..2]);

        let ndef_file_id_bytes = get_bytes!(ndef_file_control_tlv[2..4], 2);
        let ndef_file_id = u16::from_be_bytes(ndef_file_id_bytes);
        info!("ndef_file_id={}", hex::encode(ndef_file_id.to_be_bytes()));

        Ok(ndef_file_id)
    }

    fn select_app(&mut self, aid: &[u8]) -> EngagementResult<()> {
        let mut select_app_data = vec![
            0x01,
            0x00,
            0xA4,
            0x04,
            0x00,
            u8::try_from(aid.len()).unwrap(),
        ];
        select_app_data.extend_from_slice(aid);
        select_app_data.push(0x00);
        info!("select_app_data={}", hex::encode(&select_app_data));

        let select_app_request = BorrowedRequest::new(Command::InDataExchange, &select_app_data);
        let select_app_response = self.process_rf(select_app_request, 2, Self::DEFAULT_TIMEOUT)?;
        info!("select_app_response={}", hex::encode(select_app_response));

        Self::check_status(select_app_response)?;

        Ok(())
    }

    fn select_file(&mut self, file_id: u16) -> EngagementResult<()> {
        let mut select_file_data = vec![0x01, 0x00, 0xA4, 0x00, 0x0C, 0x02];
        select_file_data.extend_from_slice(&file_id.to_be_bytes());
        info!("select_file_data={}", hex::encode(&select_file_data));

        let select_file_request = BorrowedRequest::new(Command::InDataExchange, &select_file_data);
        let select_file_response =
            self.process_rf(select_file_request, 2, Self::DEFAULT_TIMEOUT)?;
        info!("select_file_response={}", hex::encode(select_file_response));

        Self::check_status(select_file_response)?;

        Ok(())
    }

    fn read_file(&mut self, offset: u16, length: u8) -> EngagementResult<&[u8]> {
        let mut read_file_data = vec![0x01, 0x00, 0xB0];
        read_file_data.extend_from_slice(&offset.to_be_bytes());
        read_file_data.push(length);
        info!("read_file_data={}", hex::encode(&read_file_data));

        let read_file_request = BorrowedRequest::new(Command::InDataExchange, &read_file_data);
        let read_file_response = self.process_rf(
            read_file_request,
            usize::from(length) + 2,
            Self::DEFAULT_TIMEOUT,
        )?;
        info!("read_file_response={}", hex::encode(read_file_response));

        Self::check_status(read_file_response)?;
        Ok(&read_file_response[..read_file_response.len() - 2])
    }

    fn read_file_ndef(&mut self) -> EngagementResult<Vec<u8>> {
        let ndef_file_len_bytes = self.read_file(0, 2)?;
        let ndef_file_len_bytes = get_bytes!(ndef_file_len_bytes, 2);
        let ndef_file_len = u16::from_be_bytes(ndef_file_len_bytes);
        info!("ndef_file_len={ndef_file_len}");

        let mut ndef_file_data = Vec::with_capacity(usize::from(ndef_file_len));
        let mut offset = 2;

        while ndef_file_data.len() < usize::from(ndef_file_len) {
            let remaining = usize::from(ndef_file_len) - ndef_file_data.len();
            let chunk_len = u8::try_from(remaining).unwrap_or(u8::MAX);
            info!("remaining={remaining} chunk_len={chunk_len}");

            let ndef_chunk = self.read_file(offset, chunk_len)?;
            info!("ndef_chunk={}", hex::encode(ndef_chunk));
            ndef_file_data.extend_from_slice(ndef_chunk);

            offset += u16::from(chunk_len);
        }
        ensure_length!(ndef_file_data.len(), usize::from(ndef_file_len));

        Ok(ndef_file_data)
    }

    fn update_file(&mut self, offset: u16, data: &[u8]) -> EngagementResult<()> {
        let mut update_file_data = vec![0x01, 0x00, 0xD6];
        update_file_data.extend_from_slice(&offset.to_be_bytes());
        update_file_data.push(u8::try_from(data.len()).unwrap());
        update_file_data.extend_from_slice(data);
        info!("update_file_data={}", hex::encode(&update_file_data));

        let update_file_request = BorrowedRequest::new(Command::InDataExchange, &update_file_data);
        let update_file_response =
            self.process_rf(update_file_request, 2, Self::DEFAULT_TIMEOUT)?;
        info!("update_file_response={}", hex::encode(update_file_response));

        Self::check_status(update_file_response)?;
        Ok(())
    }

    fn update_file_ndef(&mut self, file_data: &[u8]) -> EngagementResult<()> {
        self.update_file(0x00, &0u16.to_be_bytes())?;

        let mut offset = 2;
        for chunk in file_data.chunks(0xFF) {
            self.update_file(offset, chunk)?;
            offset += u16::try_from(chunk.len()).unwrap();
        }

        let ndef_file_data_len = u16::try_from(file_data.len()).unwrap();
        self.update_file(0x00, &ndef_file_data_len.to_be_bytes())?;

        Ok(())
    }
}

pub struct Pn532Error(pub pn532::Error<SpiError>);

impl std::fmt::Display for Pn532Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.0 {
            pn532::Error::BadAck => write!(f, "bad ack"),
            pn532::Error::BadResponseFrame => write!(f, "bad response frame"),
            pn532::Error::Syntax => write!(f, "syntax"),
            pn532::Error::CrcError => write!(f, "crc error"),
            pn532::Error::BufTooSmall => write!(f, "buf too small"),
            pn532::Error::TimeoutAck => write!(f, "timeout ack"),
            pn532::Error::TimeoutResponse => write!(f, "timeout response"),
            pn532::Error::InterfaceError(err) => err.fmt(f),
        }
    }
}

impl std::fmt::Debug for Pn532Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

impl From<pn532::Error<SpiError>> for Pn532Error {
    fn from(value: pn532::Error<SpiError>) -> Self {
        Self(value)
    }
}

impl std::error::Error for Pn532Error {}
