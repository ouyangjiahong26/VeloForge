//! VeloForge cross-platform BLE device driver.

#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]

// Implementation is staged; this stub keeps the workspace valid until the
// BLE driver lands. It re-exports the core types so downstream crates can
// already depend on them.
pub use veloforge_core as core;

#[derive(Debug, thiserror::Error)]
pub enum DeviceError {
    #[error("BLE manager not initialised")]
    NotInitialised,
    #[error("device not found: {0}")]
    DeviceNotFound(String),
    #[error("service/characteristic missing")]
    GattMissing,
    #[error("transport error: {0}")]
    Transport(String),
    #[error("queue closed")]
    QueueClosed,
}
