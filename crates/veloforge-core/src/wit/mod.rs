//! Wit BWT901BLE5.0 protocol primitives: stream decoder and command bytes.

use thiserror::Error;

pub mod commands;

pub use commands::{WitCommand, WitProtocolError};

const FRAME_LEN: usize = 20;
const HEADER_0: u8 = 0x55;
const HEADER_1: u8 = 0x61;

/// Incremental decoder for the active 0x55 0x61 frame stream.
#[derive(Debug, Default)]
pub struct WitStreamDecoder {
    buffer: Vec<u8>,
    total_frames: u64,
    bad_prefixes_skipped: u64,
}

impl WitStreamDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Push a chunk of bytes received from the BLE notification channel.
    /// Returns a vector of decoded `DecodedFrame` values (currently zero or
    /// more; the 0x71 register frame is left for future expansion).
    pub fn feed(&mut self, chunk: &[u8]) -> Vec<DecodedFrame> {
        self.buffer.extend_from_slice(chunk);
        let mut out = Vec::new();
        loop {
            match self.next_frame() {
                Ok(Some(frame)) => {
                    self.total_frames += 1;
                    out.push(frame);
                }
                Ok(None) => break,
                Err(_) => {
                    // Should be unreachable because next_frame only returns
                    // Ok values; keep loop safe.
                    break;
                }
            }
        }
        out
    }

    fn next_frame(&mut self) -> Result<Option<DecodedFrame>, DecodeError> {
        // Drop leading bytes until we see the frame header.
        let mut header_pos = None;
        for i in 0..self.buffer.len().saturating_sub(1) {
            if self.buffer[i] == HEADER_0 && self.buffer[i + 1] == HEADER_1 {
                header_pos = Some(i);
                break;
            }
        }
        let pos = match header_pos {
            Some(p) => p,
            None => {
                // Keep the last byte in case the next chunk completes a header.
                if self.buffer.len() > 1 {
                    let drop = self.buffer.len() - 1;
                    self.bad_prefixes_skipped += drop as u64;
                    self.buffer.drain(..drop);
                }
                return Ok(None);
            }
        };
        if pos > 0 {
            self.bad_prefixes_skipped += pos as u64;
            self.buffer.drain(..pos);
        }
        if self.buffer.len() < FRAME_LEN {
            return Ok(None);
        }
        let frame: [u8; FRAME_LEN] = self.buffer[..FRAME_LEN]
            .try_into()
            .expect("FRAME_LEN matches slice length");
        self.buffer.drain(..FRAME_LEN);
        Ok(Some(decode_frame(&frame)))
    }

    pub fn total_frames(&self) -> u64 {
        self.total_frames
    }

    pub fn bad_prefixes_skipped(&self) -> u64 {
        self.bad_prefixes_skipped
    }
}

/// A decoded active frame. Sensor units are kept in their vendor form.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodedFrame {
    /// Acceleration in g, body frame.
    pub accel_g: [f64; 3],
    /// Angular velocity in deg/s, body frame.
    pub gyro_dps: [f64; 3],
    /// Euler angles in degrees.
    pub euler_deg: [f64; 3],
}

fn decode_frame(frame: &[u8; FRAME_LEN]) -> DecodedFrame {
    let raw = |off: usize| i16::from_le_bytes([frame[off], frame[off + 1]]);
    let accel = [
        raw(2) as f64 / 32768.0 * 16.0,
        raw(4) as f64 / 32768.0 * 16.0,
        raw(6) as f64 / 32768.0 * 16.0,
    ];
    let gyro = [
        raw(8) as f64 / 32768.0 * 2000.0,
        raw(10) as f64 / 32768.0 * 2000.0,
        raw(12) as f64 / 32768.0 * 2000.0,
    ];
    let euler = [
        raw(14) as f64 / 32768.0 * 180.0,
        raw(16) as f64 / 32768.0 * 180.0,
        raw(18) as f64 / 32768.0 * 180.0,
    ];
    DecodedFrame {
        accel_g: accel,
        gyro_dps: gyro,
        euler_deg: euler,
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum DecodeError {
    #[error("frame too short: {0} bytes")]
    FrameTooShort(usize),
    #[error("invalid header bytes: {0:#x} {1:#x}")]
    InvalidHeader(u8, u8),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_frame(accel: [i16; 3], gyro: [i16; 3], euler: [i16; 3]) -> [u8; FRAME_LEN] {
        let mut buf = [0u8; FRAME_LEN];
        buf[0] = HEADER_0;
        buf[1] = HEADER_1;
        for (i, v) in accel.iter().enumerate() {
            let bytes = v.to_le_bytes();
            buf[2 + i * 2] = bytes[0];
            buf[3 + i * 2] = bytes[1];
        }
        for (i, v) in gyro.iter().enumerate() {
            let bytes = v.to_le_bytes();
            buf[8 + i * 2] = bytes[0];
            buf[9 + i * 2] = bytes[1];
        }
        for (i, v) in euler.iter().enumerate() {
            let bytes = v.to_le_bytes();
            buf[14 + i * 2] = bytes[0];
            buf[15 + i * 2] = bytes[1];
        }
        buf
    }

    #[test]
    fn decodes_full_frame_with_known_values() {
        // raw 16384 -> +0.5 of full scale
        // accel full scale ±16 g, gyro ±2000°/s, euler ±180°
        let frame = make_frame([16384, -16384, 0], [4096, 0, -4096], [0, 16384, -16384]);
        let mut dec = WitStreamDecoder::new();
        let out = dec.feed(&frame);
        assert_eq!(out.len(), 1);
        let f = &out[0];
        assert!(
            (f.accel_g[0] - 8.0).abs() < 1e-6,
            "accel x = {}",
            f.accel_g[0]
        );
        assert!((f.accel_g[1] - (-8.0)).abs() < 1e-6);
        assert!(f.accel_g[2].abs() < 1e-6);
        assert!((f.gyro_dps[0] - 250.0).abs() < 1e-3);
        assert!(f.gyro_dps[1].abs() < 1e-6);
        assert!((f.gyro_dps[2] - (-250.0)).abs() < 1e-3);
        assert!(f.euler_deg[0].abs() < 1e-6);
        assert!((f.euler_deg[1] - 90.0).abs() < 1e-6);
        assert!((f.euler_deg[2] - (-90.0)).abs() < 1e-6);
    }

    #[test]
    fn skips_garbage_prefix() {
        let mut dec = WitStreamDecoder::new();
        let prefix = [0x00u8, 0x01, 0x02, 0x03];
        let frame = make_frame([0, 0, 16384], [0, 0, 0], [0, 0, 0]);
        let mut data = prefix.to_vec();
        data.extend_from_slice(&frame);
        let out = dec.feed(&data);
        assert_eq!(out.len(), 1);
        assert!(dec.bad_prefixes_skipped() >= 4);
    }

    #[test]
    fn re_synchronizes_after_split_chunk() {
        let mut dec = WitStreamDecoder::new();
        let frame = make_frame([0, 0, 16384], [0, 0, 0], [0, 0, 0]);
        // Split a single frame across two chunks.
        let (a, b) = frame.split_at(7);
        assert_eq!(dec.feed(a).len(), 0);
        assert_eq!(dec.feed(b).len(), 1);
        assert_eq!(dec.total_frames(), 1);
    }

    #[test]
    fn decodes_multiple_frames_in_one_chunk() {
        let mut dec = WitStreamDecoder::new();
        let mut data = Vec::new();
        data.extend_from_slice(&make_frame([0, 0, 0], [0, 0, 0], [0, 0, 0]));
        data.extend_from_slice(&make_frame([16384, 0, 0], [0, 0, 0], [0, 0, 0]));
        let out = dec.feed(&data);
        assert_eq!(out.len(), 2);
        assert_eq!(dec.total_frames(), 2);
    }
}
