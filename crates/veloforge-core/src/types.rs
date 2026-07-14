//! Core data types. Units and ordering are part of the public contract.

use bitflags::bitflags;

bitflags! {
    /// Quality flags reported with each processed sample.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct QualityFlags: u32 {
        /// Frame decoded successfully.
        const OK              = 0b0000_0001;
        /// Quaternion was not provided, fell back to Euler angles.
        const EULER_FALLBACK  = 0b0000_0010;
        /// Sample arrived after a backward time stamp; pipeline did not advance.
        const TIME_GOES_BACK  = 0b0000_0100;
        /// dt was outside `[0.5, 2.0]` of the nominal period.
        const DT_OUTLIER      = 0b0000_1000;
        /// Magnitude of linear acceleration exceeded sanity bound.
        const ACCEL_SATURATED = 0b0001_0000;
        /// Quaternion norm deviated from unit length.
        const QUAT_NON_UNIT   = 0b0010_0000;
        /// Sample was interpolated because the BLE buffer under-ran.
        const INTERPOLATED    = 0b0100_0000;
    }
}

/// Single raw sample derived from a Wit BWT901BLE 0x55 0x61 frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Host-side monotonic timestamp in nanoseconds.
    pub monotonic_ns: i64,
    /// Acceleration in g, body frame.
    pub accel_g: [f64; 3],
    /// Angular velocity in deg/s, body frame.
    pub gyro_dps: [f64; 3],
    /// Euler angles in degrees, optional (firmware may omit).
    pub euler_deg: Option<[f64; 3]>,
    /// Quaternion in `wxyz` order, optional.
    pub quaternion_wxyz: Option<[f64; 4]>,
}

impl ImuSample {
    /// Construct a sample where only raw measurements are known.
    pub fn from_raw(
        monotonic_ns: i64,
        accel_g: [f64; 3],
        gyro_dps: [f64; 3],
    ) -> Self {
        Self {
            monotonic_ns,
            accel_g,
            gyro_dps,
            euler_deg: None,
            quaternion_wxyz: None,
        }
    }
}

/// Result of pushing a sample into the motion pipeline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProcessedSample {
    pub monotonic_ns: i64,
    /// Linear acceleration after gravity removal, in navigation frame, m/s².
    pub linear_accel_nav_mps2: [f64; 3],
    /// Integrated velocity, m/s.
    pub velocity_nav_mps: [f64; 3],
    /// Projected bar velocity, m/s.
    pub bar_velocity_mps: f64,
    /// Whether the pipeline considers the device stationary at this sample.
    pub stationary: bool,
    pub phase: MotionPhase,
    pub quality: QualityFlags,
}

impl Default for ProcessedSample {
    fn default() -> Self {
        Self {
            monotonic_ns: 0,
            linear_accel_nav_mps2: [0.0; 3],
            velocity_nav_mps: [0.0; 3],
            bar_velocity_mps: 0.0,
            stationary: true,
            phase: MotionPhase::Idle,
            quality: QualityFlags::OK,
        }
    }
}

/// High level motion phase tag for UI bookkeeping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotionPhase {
    Idle,
    Concentric,
    Eccentric,
}

/// Pipeline configuration. Tune thresholds in field, never in UI code.
#[derive(Debug, Clone)]
pub struct ProcessingConfig {
    /// Nominal sample rate in Hz; used for time-outlier detection.
    pub nominal_rate_hz: f64,
    /// Local gravity in m/s².
    pub gravity_mps2: f64,
    /// Body→bar axis projection (unit vector in navigation frame).
    pub body_to_bar_axis: [f64; 3],
    /// Stationary threshold on the linear acceleration vector norm, m/s².
    pub stationary_accel_threshold_mps2: f64,
    /// Stationary threshold on the angular velocity vector norm, rad/s.
    pub stationary_gyro_threshold_rads: f64,
    /// Time the device must remain still to enter the stationary state, ms.
    pub stationary_enter_ms: u32,
    /// Time the device must remain moving to leave the stationary state, ms.
    pub stationary_exit_ms: u32,
    /// Bound beyond which a sample is considered saturated, m/s².
    pub accel_saturation_mps2: f64,
    /// Expected samples per nominal period for clamping; ratio.
    pub max_dt_ratio: f64,
    pub min_dt_ratio: f64,
}

impl Default for ProcessingConfig {
    fn default() -> Self {
        Self {
            nominal_rate_hz: 200.0,
            gravity_mps2: 9.7915,
            body_to_bar_axis: [0.0, 0.0, 1.0],
            stationary_accel_threshold_mps2: 0.35,
            stationary_gyro_threshold_rads: 0.0026, // 0.15 deg/s
            stationary_enter_ms: 50,
            stationary_exit_ms: 60,
            accel_saturation_mps2: 80.0,
            max_dt_ratio: 2.0,
            min_dt_ratio: 0.5,
        }
    }
}

/// End-of-rep summary emitted by the pipeline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RepetitionResult {
    pub start_ns: i64,
    pub end_ns: i64,
    pub mean_concentric_velocity_mps: f64,
    pub peak_velocity_mps: f64,
    pub duration_s: f64,
    pub quality: QualityFlags,
}
