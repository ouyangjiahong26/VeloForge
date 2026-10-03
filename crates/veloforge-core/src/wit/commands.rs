//! Wit BWT901BLE5.0 command bytes.

use thiserror::Error;

/// Hint for the active frame length; not part of the public API.
#[doc(hidden)]
pub const FRAME_LEN_HINT: u16 = 20;

/// Vendor commands used by the SDK. Only the commands required by the
/// VeloForge use cases are exposed; the firmware accepts more.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WitCommand {
    UnlockRegister,
    SaveRegister,
    AppliedCalibration,
    StartFieldCalibration,
    EndFieldCalibration,
    SetReturnRate(ReturnRate),
    SetBandwidth(Bandwidth),
    ReadRegister(u8),
    WriteRegister { reg: u8, value: u16 },
}

impl WitCommand {
    /// Encode the command into the 5-byte vendor frame.
    pub fn encode(self) -> [u8; 5] {
        match self {
            WitCommand::UnlockRegister => [0xff, 0xaa, 0x69, 0x88, 0xb5],
            WitCommand::SaveRegister => [0xff, 0xaa, 0x00, 0x00, 0x00],
            WitCommand::AppliedCalibration => [0xff, 0xaa, 0x01, 0x01, 0x00],
            WitCommand::StartFieldCalibration => [0xff, 0xaa, 0x01, 0x07, 0x00],
            WitCommand::EndFieldCalibration => [0xff, 0xaa, 0x01, 0x00, 0x00],
            WitCommand::SetReturnRate(rate) => [0xff, 0xaa, 0x03, rate as u8, 0x00],
            WitCommand::SetBandwidth(band) => [0xff, 0xaa, 0x1f, band as u8, 0x00],
            WitCommand::ReadRegister(reg) => [0xff, 0xaa, 0x27, reg, 0x00],
            WitCommand::WriteRegister { reg, value } => [
                0xff,
                0xaa,
                reg,
                (value & 0xff) as u8,
                ((value >> 8) & 0xff) as u8,
            ],
        }
    }
}

/// Return rate code accepted by the firmware. The 200 Hz code is the
/// `0x0B` value used in the vendor Android reference; the previous
/// `0x0A` value seen in some C# code is firmware-dependent and must be
/// validated on the actual hardware. Both are exposed.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReturnRate {
    Hz1 = 0x03,
    Hz5 = 0x05,
    Hz10 = 0x06,
    Hz50 = 0x08,
    Hz100 = 0x09,
    Hz200Legacy = 0x0a,
    Hz200 = 0x0b,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Bandwidth {
    Hz5 = 0x07,
    Hz10 = 0x06,
    Hz20 = 0x05,
    Hz42 = 0x04,
    Hz98 = 0x03,
    Hz188 = 0x02,
    Hz256 = 0x00,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum WitProtocolError {
    #[error("register number out of range: {0}")]
    RegisterOutOfRange(u8),
    #[error("register value out of range: {0}")]
    ValueOutOfRange(u16),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unlocks_register() {
        assert_eq!(
            WitCommand::UnlockRegister.encode(),
            [0xff, 0xaa, 0x69, 0x88, 0xb5]
        );
    }

    #[test]
    fn encodes_calibration_commands() {
        assert_eq!(
            WitCommand::AppliedCalibration.encode(),
            [0xff, 0xaa, 0x01, 0x01, 0x00]
        );
        assert_eq!(
            WitCommand::StartFieldCalibration.encode(),
            [0xff, 0xaa, 0x01, 0x07, 0x00]
        );
        assert_eq!(
            WitCommand::EndFieldCalibration.encode(),
            [0xff, 0xaa, 0x01, 0x00, 0x00]
        );
    }

    #[test]
    fn encodes_200hz_return_rate() {
        assert_eq!(
            WitCommand::SetReturnRate(ReturnRate::Hz200).encode(),
            [0xff, 0xaa, 0x03, 0x0b, 0x00]
        );
    }

    #[test]
    fn encodes_register_write_with_little_endian_value() {
        assert_eq!(
            WitCommand::WriteRegister {
                reg: 0x1f,
                value: 0x1234,
            }
            .encode(),
            [0xff, 0xaa, 0x1f, 0x34, 0x12]
        );
    }
}
