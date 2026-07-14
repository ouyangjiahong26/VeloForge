//! VeloForge core: protocol and motion processing.

#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]

pub mod math;
pub mod pipeline;
pub mod types;
pub mod wit;

pub use pipeline::MotionPipeline;
pub use types::{
    ImuSample, MotionPhase, ProcessedSample, ProcessingConfig, QualityFlags, RepetitionResult,
};
pub use wit::{WitCommand, WitStreamDecoder};
