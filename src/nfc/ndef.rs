use log::trace;
use packed_struct::prelude::*;
use serde::Serialize;

use crate::nfc::{EngagementResult, get_byte, get_bytes};

use super::EngagementError;

pub trait NdefWrite<W>
where
    W: embedded_io::Write,
{
    fn ndef_write_data(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
    ) -> Result<(), W::Error>;

    #[allow(clippy::too_many_arguments)]
    fn ndef_write_headers(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
        type_name_format: TypeNameFormat,
        payload_len: usize,
        type_data: &[u8],
        id_data: Option<&[u8]>,
    ) -> Result<(), W::Error> {
        let flags = NdefFlags {
            message_begin,
            message_end,
            chunk: false,
            short_record: payload_len < usize::from(u8::MAX),
            id_length_present: id_data.is_some(),
            type_name_format,
        };

        wtr.write_all(&flags.pack().unwrap())?;

        wtr.write_all(&u8::try_from(type_data.len()).unwrap().to_be_bytes())?;
        if flags.short_record {
            wtr.write_all(&u8::try_from(payload_len).unwrap().to_be_bytes())?;
        } else {
            wtr.write_all(&u32::try_from(payload_len).unwrap().to_be_bytes())?;
        }
        if let Some(id_data) = id_data {
            wtr.write_all(&u8::try_from(id_data.len()).unwrap().to_be_bytes())?;
        }

        wtr.write_all(type_data)?;
        if let Some(id_data) = id_data {
            wtr.write_all(id_data)?;
        }

        Ok(())
    }
}

#[derive(Debug)]
pub struct RawNdefRecord<'a> {
    pub flags: NdefFlags,
    pub type_data: &'a [u8],
    pub id_data: Option<&'a [u8]>,
    pub payload_data: &'a [u8],
}

#[derive(Debug, PackedStruct)]
#[packed_struct(bit_numbering = "msb0")]
pub struct NdefFlags {
    #[packed_field(bits = "0")]
    pub message_begin: bool,
    #[packed_field(bits = "1")]
    pub message_end: bool,
    #[packed_field(bits = "2")]
    pub chunk: bool,
    #[packed_field(bits = "3")]
    pub short_record: bool,
    #[packed_field(bits = "4")]
    pub id_length_present: bool,
    #[packed_field(bits = "5..=7", ty = "enum")]
    pub type_name_format: TypeNameFormat,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PrimitiveEnum_u8)]
pub enum TypeNameFormat {
    Empty = 0x00,
    WellKnown = 0x01,
    MediaType = 0x02,
    AbsoluteUri = 0x03,
    External = 0x04,
    Unknown = 0x05,
    Unchanged = 0x06,
    Reserved = 0x07,
}

pub struct NdefRecordIterator<'a> {
    data: &'a [u8],
    offset: usize,
}

impl<'a> NdefRecordIterator<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, offset: 0 }
    }
}

impl<'a> Iterator for NdefRecordIterator<'a> {
    type Item = EngagementResult<RawNdefRecord<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.data.len() <= self.offset {
            return None;
        }

        let (offset, record) = match RawNdefRecord::decode_single(&self.data[self.offset..]) {
            Ok(data) => data,
            Err(err) => return Some(Err(err)),
        };

        self.offset += offset;

        Some(Ok(record))
    }
}

impl<'a> RawNdefRecord<'a> {
    pub fn decode_single(data: &'a [u8]) -> EngagementResult<(usize, RawNdefRecord<'a>)> {
        let mut offset = 0;

        let flags = NdefFlags::unpack(&[data[offset]])?;
        offset += 1;
        trace!("flags={flags:?}");

        let type_length = get_byte!(data, offset, "type_length");
        offset += 1;

        let payload_length = if flags.short_record {
            let len = u32::from(get_byte!(data, offset, "payload_length"));
            offset += 1;
            len
        } else {
            let len = u32::from_be_bytes(get_bytes!(data[offset..offset + 4], 4));
            offset += 4;
            len
        };

        let id_length = if flags.id_length_present {
            let len = Some(get_byte!(data, offset, "id_length"));
            offset += 1;
            len
        } else {
            None
        };

        trace!(
            "type_length={type_length}, payload_length={payload_length}, id_length={id_length:?}"
        );

        let type_data = &data[offset..offset + usize::from(type_length)];
        trace!("type_data={}", hex::encode(type_data));
        offset += usize::from(type_length);

        let id_data = if let Some(id_length) = id_length {
            let data = &data[offset..offset + usize::from(id_length)];
            trace!("id_data={}", hex::encode(data));
            offset += usize::from(id_length);
            Some(data)
        } else {
            None
        };

        let payload_data = &data[offset..offset + usize::try_from(payload_length).unwrap()];
        offset += usize::try_from(payload_length).unwrap();
        trace!("payload_data={}", hex::encode(payload_data));

        Ok((offset, Self {
            flags,
            type_data,
            id_data,
            payload_data,
        }))
    }
}

impl<W> NdefWrite<W> for RawNdefRecord<'_>
where
    W: embedded_io::Write,
{
    fn ndef_write_data(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
    ) -> Result<(), W::Error> {
        self.ndef_write_headers(
            wtr,
            message_begin,
            message_end,
            self.flags.type_name_format,
            self.payload_data.len(),
            self.type_data,
            self.id_data,
        )?;
        wtr.write_all(self.payload_data)?;

        Ok(())
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct TnepServiceParameter<'a> {
    pub version: u8,
    pub service_name: &'a [u8],
    pub tnep_communication_mode: u8,
    pub minimum_waiting_time: u8,
    pub maximum_waiting_time_extensions: u8,
    pub maximum_message_size: u16,
}

impl<'a> TnepServiceParameter<'a> {
    pub fn decode(data: &'a [u8]) -> EngagementResult<(usize, TnepServiceParameter<'a>)> {
        let mut offset = 0;

        let version = get_byte!(data, offset, "version");
        offset += 1;

        let service_name_len = get_byte!(data, offset, "service_name_len");
        offset += 1;

        let service_name = &data[offset..offset + usize::from(service_name_len)];
        offset += usize::from(service_name_len);

        let tnep_communication_mode = get_byte!(data, offset, "tnep_communication_mode");
        offset += 1;

        let minimum_waiting_time = get_byte!(data, offset, "minimum_waiting_time");
        offset += 1;

        let maximum_waiting_time_extensions =
            get_byte!(data, offset, "maximum_waiting_time_extensions");
        offset += 1;

        let maximum_message_size = u16::from_be_bytes(get_bytes!(data[offset..offset + 2], 2));
        offset += 2;

        Ok((offset, TnepServiceParameter {
            version,
            service_name,
            tnep_communication_mode,
            minimum_waiting_time,
            maximum_waiting_time_extensions,
            maximum_message_size,
        }))
    }

    pub fn wait_time_ms(&self) -> u32 {
        2u32.pow((self.minimum_waiting_time / 4 - 1).into())
    }
}

pub struct TnepServiceSelect<'a> {
    pub service_name: &'a [u8],
}

impl<W> NdefWrite<W> for TnepServiceSelect<'_>
where
    W: embedded_io::Write,
{
    fn ndef_write_data(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
    ) -> Result<(), W::Error> {
        let mut payload = Vec::with_capacity(1 + self.service_name.len());
        payload.push(u8::try_from(self.service_name.len()).unwrap());
        payload.extend_from_slice(self.service_name);

        self.ndef_write_headers(
            wtr,
            message_begin,
            message_end,
            TypeNameFormat::WellKnown,
            payload.len(),
            b"Ts",
            None,
        )?;
        wtr.write_all(&payload)?;

        Ok(())
    }
}

pub enum TnepStatus {
    Success,
    Unknown(u8),
}

impl TnepStatus {
    pub fn decode(data: &[u8]) -> EngagementResult<(usize, Self)> {
        let status = get_byte!(data, 0, "status");
        let status = match status {
            0 => Self::Success,
            other => Self::Unknown(other),
        };
        Ok((1, status))
    }
}

#[derive(Debug)]
pub struct HandoverRequest<'a> {
    pub version: u8,
    pub collision_resolution: CollisionResolution,
    pub alternative_carriers: Vec<AlternativeCarrier<'a>>,
}

impl<W> NdefWrite<W> for HandoverRequest<'_>
where
    W: embedded_io::Write,
{
    fn ndef_write_data(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
    ) -> Result<(), W::Error> {
        let mut payload = vec![self.version];

        self.collision_resolution
            .ndef_write_data(&mut payload, true, false)
            .unwrap();

        let mut alt_carriers = self.alternative_carriers.iter().peekable();

        while let Some(alt_carrier) = alt_carriers.next() {
            alt_carrier
                .ndef_write_data(&mut payload, false, alt_carriers.peek().is_none())
                .unwrap();
        }

        self.ndef_write_headers(
            wtr,
            message_begin,
            message_end,
            TypeNameFormat::WellKnown,
            payload.len(),
            b"Hr",
            None,
        )?;
        wtr.write_all(&payload)?;

        Ok(())
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct HandoverSelect<'a> {
    pub version: u8,
    pub alternative_carrier: AlternativeCarrier<'a>,
}

impl<'a> HandoverSelect<'a> {
    pub fn decode(data: &'a [u8]) -> EngagementResult<(usize, HandoverSelect<'a>)> {
        let mut offset = 0;

        let version = get_byte!(data, offset, "version");
        offset += 1;

        let (record_len, record) = RawNdefRecord::decode_single(&data[offset..])?;
        offset += record_len;

        if record.flags.type_name_format != TypeNameFormat::WellKnown || record.type_data != b"ac" {
            return Err(EngagementError::MissingRequiredData("ac record"));
        }

        // ignore offset because this was inside of another record
        let (_, alternative_carrier) = AlternativeCarrier::decode(record.payload_data)?;

        Ok((offset, HandoverSelect {
            version,
            alternative_carrier,
        }))
    }
}

impl<W> NdefWrite<W> for HandoverSelect<'_>
where
    W: embedded_io::Write,
{
    fn ndef_write_data(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
    ) -> Result<(), W::Error> {
        let mut inner_records = Vec::new();
        self.alternative_carrier
            .ndef_write_data(&mut inner_records, true, true)
            .unwrap();

        self.ndef_write_headers(
            wtr,
            message_begin,
            message_end,
            TypeNameFormat::WellKnown,
            inner_records.len(),
            b"Hs",
            None,
        )?;
        wtr.write_all(&inner_records)?;

        Ok(())
    }
}

#[derive(Debug)]
pub struct CollisionResolution {
    pub value: u16,
}

impl CollisionResolution {
    /// Generate a pseudorandom collision resolution record.
    pub fn new_rand() -> Self {
        let rand =
            esp_idf_svc::systime::EspSystemTime::now(&esp_idf_svc::systime::EspSystemTime {})
                .subsec_nanos()
                / 65537;
        let [_, _, high, low] = rand.to_ne_bytes();

        Self {
            value: u16::from_ne_bytes([low, high]),
        }
    }
}

impl<W> NdefWrite<W> for CollisionResolution
where
    W: embedded_io::Write,
{
    fn ndef_write_data(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
    ) -> Result<(), W::Error> {
        self.ndef_write_headers(
            wtr,
            message_begin,
            message_end,
            TypeNameFormat::WellKnown,
            2,
            b"cr",
            None,
        )?;
        wtr.write_all(&self.value.to_be_bytes())?;
        Ok(())
    }
}

#[derive(Debug)]
pub struct AlternativeCarrier<'a> {
    pub carrier_power_state: CarrierPowerState,
    pub carrier_data_reference: &'a [u8],
    pub auxiliary_data_references: Vec<&'a [u8]>,
}

impl<'a> AlternativeCarrier<'a> {
    pub fn decode(data: &'a [u8]) -> EngagementResult<(usize, AlternativeCarrier<'a>)> {
        let mut offset = 0;

        let carrier_power_state_byte = get_byte!(data, offset, "carrier_power_state");
        offset += 1;

        let carrier_power_state =
            CarrierPowerState::from_primitive(carrier_power_state_byte & 0b00000011)
                .ok_or(EngagementError::MissingRequiredData("carrier_power_state"))?;

        let carrier_data_ref_len = usize::from(get_byte!(data, offset, "carrier_data_ref"));
        offset += 1;

        let carrier_data_reference = &data[offset..offset + carrier_data_ref_len];
        offset += carrier_data_ref_len;

        let aux_data_ref_count = get_byte!(data, offset, "aux_data_ref_count");
        offset += 1;

        let mut auxiliary_data_references = Vec::with_capacity(usize::from(aux_data_ref_count));

        for _ in 0..aux_data_ref_count {
            let aux_data_ref_len = usize::from(get_byte!(data, offset, "aux_data_ref_len"));
            offset += 1;

            auxiliary_data_references.push(&data[offset..offset + aux_data_ref_len]);
            offset += aux_data_ref_len;
        }

        Ok((offset, AlternativeCarrier {
            carrier_power_state,
            carrier_data_reference,
            auxiliary_data_references,
        }))
    }
}

impl<W> NdefWrite<W> for AlternativeCarrier<'_>
where
    W: embedded_io::Write,
{
    fn ndef_write_data(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
    ) -> Result<(), W::Error> {
        let payload_len = 1 + // carrier power state
            1 + // carrier data reference length
            self.carrier_data_reference.len() +
            1 + // auxiliary data references length
            self.auxiliary_data_references.iter().map(|aux_ref| 1 + aux_ref.len()).sum::<usize>();

        self.ndef_write_headers(
            wtr,
            message_begin,
            message_end,
            TypeNameFormat::WellKnown,
            payload_len,
            b"ac",
            None,
        )?;

        wtr.write_all(&self.carrier_power_state.to_primitive().to_be_bytes())?;

        wtr.write_all(
            &u8::try_from(self.carrier_data_reference.len())
                .unwrap()
                .to_be_bytes(),
        )?;
        wtr.write_all(self.carrier_data_reference)?;

        wtr.write_all(
            &u8::try_from(self.auxiliary_data_references.len())
                .unwrap()
                .to_be_bytes(),
        )?;
        for aux_ref in self.auxiliary_data_references.iter() {
            wtr.write_all(&u8::try_from(aux_ref.len()).unwrap().to_be_bytes())?;
            wtr.write_all(aux_ref)?;
        }

        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, PrimitiveEnum_u8)]
pub enum CarrierPowerState {
    Inactive = 0x00,
    Active = 0x01,
    Activating = 0x02,
    Unknown = 0x03,
}

#[derive(Debug)]
pub struct CarrierConfigurationBle<'a> {
    pub fields: Vec<(u8, &'a [u8])>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, PrimitiveEnum_u8, Serialize)]
pub enum BleRole {
    Peripheral = 0x00,
    Central = 0x01,
    PeripheralCentral = 0x02,
    CentralPeripheral = 0x03,
}

impl<'a> CarrierConfigurationBle<'a> {
    pub fn decode(data: &'a [u8]) -> EngagementResult<CarrierConfigurationBle<'a>> {
        let mut fields = Vec::with_capacity(2);

        let mut offset = 0;
        while offset < data.len() {
            let data_length = usize::from(get_byte!(data, offset, "data_length"));
            offset += 1;

            let data = &data[offset..offset + data_length];
            offset += data_length;

            let (field_id, data_value) = (data[0], &data[1..]);

            fields.push((field_id, data_value));
        }

        Ok(CarrierConfigurationBle { fields })
    }

    pub fn get_field_value(&self, wanted_field_id: u8) -> Option<&'a [u8]> {
        self.fields.iter().find_map(|(field_id, data_value)| {
            (*field_id == wanted_field_id).then_some(*data_value)
        })
    }

    pub fn role(&self) -> Option<BleRole> {
        self.get_field_value(0x1C)
            .and_then(|data| data.first().copied())
            .and_then(BleRole::from_primitive)
    }

    pub fn service_uuid(&self) -> Option<[u8; 16]> {
        self.get_field_value(0x07)
            .and_then(|data| <[u8; 16]>::try_from(data).ok())
            .map(|mut bytes| {
                bytes.reverse();
                bytes
            })
    }
}

impl<W> NdefWrite<W> for CarrierConfigurationBle<'_>
where
    W: embedded_io::Write,
{
    fn ndef_write_data(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
    ) -> Result<(), W::Error> {
        let payload_len = self
            .fields
            .iter()
            .map(|(_, data_value)| data_value.len() + 2)
            .sum();

        self.ndef_write_headers(
            wtr,
            message_begin,
            message_end,
            TypeNameFormat::MediaType,
            payload_len,
            b"application/vnd.bluetooth.le.oob",
            Some(b"0"),
        )?;

        for (field_id, data_value) in self.fields.iter() {
            wtr.write_all(&[u8::try_from(data_value.len() + 1).unwrap(), *field_id])?;
            wtr.write_all(data_value)?;
        }

        Ok(())
    }
}

pub struct ReaderEngagement<'a> {
    pub version: &'a str,
}

impl<W> NdefWrite<W> for ReaderEngagement<'_>
where
    W: embedded_io::Write,
{
    fn ndef_write_data(
        &self,
        wtr: &mut W,
        message_begin: bool,
        message_end: bool,
    ) -> Result<(), W::Error> {
        let mut buf = Vec::new();
        let mut encoder = minicbor::Encoder::new(&mut buf);

        encoder
            .begin_map()
            .unwrap()
            .u8(0)
            .unwrap()
            .str(self.version)
            .unwrap()
            .end()
            .unwrap();

        self.ndef_write_headers(
            wtr,
            message_begin,
            message_end,
            TypeNameFormat::External,
            buf.len(),
            b"iso.org:18013:readerengagement",
            Some(b"mdocreader"),
        )?;
        wtr.write_all(&buf)?;

        Ok(())
    }
}
