//! PyO3 adapter exposing the protocol decoder and motion pipeline to Python.

#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]

use std::sync::Mutex;

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use veloforge_core::wit::commands::{Bandwidth, ReturnRate, WitCommand};
use veloforge_core::{ImuSample, MotionPipeline, WitStreamDecoder};

/// Simple non-BLE driver that decodes and processes bytes for tests and
/// the offline replay UI. Mirrors the shape of the future BLE engine so
/// the Python side can swap implementations.
#[pyclass(name = "SensorEngine")]
pub struct PySensorEngine {
    inner: Mutex<EngineInner>,
}

struct EngineInner {
    decoder: WitStreamDecoder,
    pipeline: MotionPipeline,
    frames_decoded: u64,
}

impl EngineInner {
    fn new() -> Self {
        Self {
            decoder: WitStreamDecoder::new(),
            pipeline: MotionPipeline::default(),
            frames_decoded: 0,
        }
    }
}

#[pyclass(name = "DeviceInfo")]
#[derive(Debug, Clone)]
pub struct PyDeviceInfo {
    #[pyo3(get)]
    pub id: String,
    #[pyo3(get)]
    pub name: String,
}

#[pymethods]
impl PyDeviceInfo {
    fn __repr__(&self) -> String {
        format!("DeviceInfo(id={:?}, name={:?})", self.id, self.name)
    }
}

#[pyclass(name = "Sample")]
#[derive(Debug, Clone)]
pub struct PySample {
    #[pyo3(get)]
    pub monotonic_ns: i64,
    #[pyo3(get)]
    pub linear_accel: [f64; 3],
    #[pyo3(get)]
    pub velocity: [f64; 3],
    #[pyo3(get)]
    pub bar_velocity_mps: f64,
    #[pyo3(get)]
    pub stationary: bool,
    #[pyo3(get)]
    pub phase: String,
    #[pyo3(get)]
    pub quality: u32,
}

#[pymethods]
impl PySample {
    fn __repr__(&self) -> String {
        format!(
            "Sample(ns={}, v={:.3} m/s, static={}, phase={}, quality=0x{:x})",
            self.monotonic_ns, self.bar_velocity_mps, self.stationary, self.phase, self.quality
        )
    }
}

#[pyclass(name = "StateView")]
#[derive(Debug, Clone)]
pub struct PyStateView {
    #[pyo3(get)]
    pub frames_decoded: u64,
    #[pyo3(get)]
    pub bad_prefixes: u64,
    #[pyo3(get)]
    pub velocity: [f64; 3],
    #[pyo3(get)]
    pub bar_velocity_mps: f64,
    #[pyo3(get)]
    pub stationary: bool,
    #[pyo3(get)]
    pub phase: String,
    #[pyo3(get)]
    pub quality: u32,
}

#[pymethods]
impl PySensorEngine {
    #[new]
    fn new() -> PyResult<Self> {
        Ok(Self {
            inner: Mutex::new(EngineInner::new()),
        })
    }

    /// Feed raw BLE bytes from the transport. Returns decoded `Sample`
    /// objects in arrival order.
    fn feed(&self, chunk: &[u8]) -> PyResult<Vec<PySample>> {
        let mut inner = self.inner.lock().map_err(poisoned_to_pyerr)?;
        let decoded = inner.decoder.feed(chunk);
        inner.frames_decoded += decoded.len() as u64;
        let mut out = Vec::with_capacity(decoded.len());
        for frame in decoded {
            let monotonic_ns = current_monotonic_ns();
            let sample = ImuSample {
                monotonic_ns,
                accel_g: frame.accel_g,
                gyro_dps: frame.gyro_dps,
                euler_deg: Some(frame.euler_deg),
                quaternion_wxyz: None,
            };
            let processed = inner
                .pipeline
                .push(sample)
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            out.push(sample_to_py(processed));
        }
        Ok(out)
    }

    fn state(&self) -> PyResult<PyStateView> {
        let inner = self.inner.lock().map_err(poisoned_to_pyerr)?;
        let velocity = inner.pipeline.velocity();
        let bad_prefixes = inner.decoder.bad_prefixes_skipped();
        let frames = inner.frames_decoded;
        Ok(PyStateView {
            frames_decoded: frames,
            bad_prefixes,
            velocity,
            bar_velocity_mps: 0.0,
            stationary: true,
            phase: "idle".to_string(),
            quality: 0,
        })
    }

    fn reset(&self) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(poisoned_to_pyerr)?;
        inner.decoder = WitStreamDecoder::new();
        inner.pipeline.reset();
        inner.frames_decoded = 0;
        Ok(())
    }
}

fn poisoned_to_pyerr<T>(_: std::sync::PoisonError<T>) -> PyErr {
    PyValueError::new_err("engine mutex poisoned").into()
}

fn sample_to_py(p: veloforge_core::ProcessedSample) -> PySample {
    let phase = match p.phase {
        veloforge_core::MotionPhase::Idle => "idle",
        veloforge_core::MotionPhase::Concentric => "concentric",
        veloforge_core::MotionPhase::Eccentric => "eccentric",
    };
    PySample {
        monotonic_ns: p.monotonic_ns,
        linear_accel: p.linear_accel_nav_mps2,
        velocity: p.velocity_nav_mps,
        bar_velocity_mps: p.bar_velocity_mps,
        stationary: p.stationary,
        phase: phase.to_string(),
        quality: p.quality.bits(),
    }
}

fn current_monotonic_ns() -> i64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0)
}

/// Encode a Wit command to its 5-byte form for direct BLE writes.
#[pyfunction]
fn encode_command(name: &str) -> PyResult<[u8; 5]> {
    let cmd = match name {
        "unlock" => WitCommand::UnlockRegister,
        "save" => WitCommand::SaveRegister,
        "applied_calibration" => WitCommand::AppliedCalibration,
        "start_field_calibration" => WitCommand::StartFieldCalibration,
        "end_field_calibration" => WitCommand::EndFieldCalibration,
        "return_rate_200" => WitCommand::SetReturnRate(ReturnRate::Hz200),
        "return_rate_100" => WitCommand::SetReturnRate(ReturnRate::Hz100),
        "return_rate_50" => WitCommand::SetReturnRate(ReturnRate::Hz50),
        "bandwidth_256" => WitCommand::SetBandwidth(Bandwidth::Hz256),
        "bandwidth_188" => WitCommand::SetBandwidth(Bandwidth::Hz188),
        other => return Err(PyValueError::new_err(format!("unknown command: {other}"))),
    };
    Ok(cmd.encode())
}

#[pymodule]
fn _core(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PySensorEngine>()?;
    m.add_class::<PyDeviceInfo>()?;
    m.add_class::<PySample>()?;
    m.add_class::<PyStateView>()?;
    m.add_function(wrap_pyfunction!(encode_command, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
