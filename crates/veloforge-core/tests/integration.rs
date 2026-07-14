//! Integration tests for the protocol decoder using hand-built frames.

use veloforge_core::WitStreamDecoder;
use veloforge_core::wit::commands::WitCommand;

const FRAME_LEN: usize = 20;

fn active_frame(
    accel: [i16; 3],
    gyro: [i16; 3],
    euler: [i16; 3],
) -> [u8; FRAME_LEN] {
    let mut buf = [0u8; FRAME_LEN];
    buf[0] = 0x55;
    buf[1] = 0x61;
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
fn round_trip_frame_via_decoder() {
    let frame = active_frame([16384, 0, 0], [0, 0, 0], [0, 0, 0]);
    let mut dec = WitStreamDecoder::new();
    let out = dec.feed(&frame);
    assert_eq!(out.len(), 1);
    assert!((out[0].accel_g[0] - 8.0).abs() < 1e-6);
}

#[test]
fn round_trip_unlock_command() {
    let cmd = WitCommand::UnlockRegister.encode();
    assert_eq!(cmd, [0xff, 0xaa, 0x69, 0x88, 0xb5]);
}
