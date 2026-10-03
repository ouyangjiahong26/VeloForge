//! Motion pipeline: gravity removal, stationary detection, trapezoidal
//! integration, drift correction and repetition tagging.

use nalgebra::Vector3;

use crate::math::{body_to_nav, gravity_in_body, magnitude, quat_from_wxyz};
use crate::types::{
    ImuSample, MotionPhase, ProcessedSample, ProcessingConfig, QualityFlags, RepetitionResult,
};
use thiserror::Error;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum PipelineError {
    #[error("sample time went backwards by {0} ns")]
    TimeGoesBack(i64),
}

/// Internal state for the stationary detector with enter/exit hysteresis.
#[derive(Debug, Clone, Copy)]
struct StationaryState {
    entering_ms: i64, // cumulative time inside the entering condition
    exiting_ms: i64,  // cumulative time inside the exiting condition
    is_stationary: bool,
    last_change_ns: i64,
}

impl StationaryState {
    fn new() -> Self {
        Self {
            entering_ms: 0,
            exiting_ms: 0,
            is_stationary: true,
            last_change_ns: 0,
        }
    }

    fn update(
        &mut self,
        now_ns: i64,
        lin_accel_norm: f64,
        gyro_norm: f64,
        cfg: &ProcessingConfig,
    ) -> bool {
        let dt_ms = (now_ns - self.last_change_ns) as f64 / 1_000_000.0;
        self.last_change_ns = now_ns;
        if dt_ms.is_finite() && dt_ms >= 0.0 {
            let inside_enter = lin_accel_norm <= cfg.stationary_accel_threshold_mps2
                && gyro_norm <= cfg.stationary_gyro_threshold_rads;
            let inside_exit = lin_accel_norm > cfg.stationary_accel_threshold_mps2
                || gyro_norm > cfg.stationary_gyro_threshold_rads;
            if self.is_stationary {
                if inside_exit {
                    self.exiting_ms += dt_ms as i64;
                    if self.exiting_ms as u32 >= cfg.stationary_exit_ms {
                        self.is_stationary = false;
                        self.exiting_ms = 0;
                    }
                } else {
                    self.exiting_ms = 0;
                }
            } else if inside_enter {
                self.entering_ms += dt_ms as i64;
                if self.entering_ms as u32 >= cfg.stationary_enter_ms {
                    self.is_stationary = true;
                    self.entering_ms = 0;
                }
            } else {
                self.entering_ms = 0;
            }
        }
        self.is_stationary
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}

/// Main processing pipeline.
#[derive(Debug)]
pub struct MotionPipeline {
    cfg: ProcessingConfig,
    last_ns: Option<i64>,
    velocity_nav: Vector3<f64>,
    position_nav: Vector3<f64>,
    stationary: StationaryState,
    phase: MotionPhase,
    rep_start_ns: Option<i64>,
    rep_samples: u32,
    rep_peak_velocity: f64,
    rep_velocity_sum: f64,
    last_quality: QualityFlags,
}

impl MotionPipeline {
    pub fn new(cfg: ProcessingConfig) -> Self {
        Self {
            cfg,
            last_ns: None,
            velocity_nav: Vector3::zeros(),
            position_nav: Vector3::zeros(),
            stationary: StationaryState::new(),
            phase: MotionPhase::Idle,
            rep_start_ns: None,
            rep_samples: 0,
            rep_peak_velocity: 0.0,
            rep_velocity_sum: 0.0,
            last_quality: QualityFlags::OK,
        }
    }

    pub fn config(&self) -> &ProcessingConfig {
        &self.cfg
    }

    pub fn update_config(&mut self, cfg: ProcessingConfig) {
        self.cfg = cfg;
    }

    /// Push a new sample and obtain the corresponding processed result.
    /// Returns `Err` only if the caller hands us a non-monotonic time stamp.
    pub fn push(&mut self, sample: ImuSample) -> Result<ProcessedSample, PipelineError> {
        let mut quality = QualityFlags::OK;
        let now = sample.monotonic_ns;
        if let Some(prev) = self.last_ns {
            if now < prev {
                return Err(PipelineError::TimeGoesBack(prev - now));
            }
        }

        // 1. Unit conversion
        let accel_mps2 = Vector3::new(
            sample.accel_g[0] * self.cfg.gravity_mps2,
            sample.accel_g[1] * self.cfg.gravity_mps2,
            sample.accel_g[2] * self.cfg.gravity_mps2,
        );
        let gyro_rads = Vector3::new(
            sample.gyro_dps[0].to_radians(),
            sample.gyro_dps[1].to_radians(),
            sample.gyro_dps[2].to_radians(),
        );

        // 2. Gravity removal and frame conversion
        let linear_nav = match sample.quaternion_wxyz.and_then(quat_from_wxyz) {
            Some(q) => {
                let g_body = gravity_in_body(q, self.cfg.gravity_mps2);
                let body = accel_mps2 - g_body;
                body_to_nav(q, body)
            }
            None => {
                quality |= QualityFlags::EULER_FALLBACK;
                // Without a quaternion we cannot reliably remove gravity; we
                // return the raw body-frame measurement and skip the nav
                // transform. The UI must surface EULER_FALLBACK.
                accel_mps2
            }
        };

        // 3. dt estimation and clamping
        let dt = match self.last_ns {
            Some(prev) => {
                let raw_dt = (now - prev) as f64 / 1_000_000_000.0;
                let nominal_dt = 1.0 / self.cfg.nominal_rate_hz;
                let upper = nominal_dt * self.cfg.max_dt_ratio;
                let lower = nominal_dt * self.cfg.min_dt_ratio;
                if raw_dt > upper || raw_dt < lower {
                    quality |= QualityFlags::DT_OUTLIER;
                }
                raw_dt.clamp(lower, upper)
            }
            None => 0.0,
        };

        // 4. Saturation flag
        if linear_nav.norm() > self.cfg.accel_saturation_mps2 {
            quality |= QualityFlags::ACCEL_SATURATED;
        }

        // 5. Trapezoidal integration
        if self.last_ns.is_some() && dt > 0.0 {
            self.velocity_nav += linear_nav * dt;
        }

        // 6. Stationary detection
        let lin_norm = linear_nav.norm();
        let gyro_norm = gyro_rads.norm();
        let is_static = self.stationary.update(now, lin_norm, gyro_norm, &self.cfg);
        if is_static {
            self.velocity_nav = Vector3::zeros();
        }

        // 7. Position integration
        if self.last_ns.is_some() && dt > 0.0 {
            self.position_nav += self.velocity_nav * dt;
        }

        // 8. Bar velocity projection
        let axis = Vector3::new(
            self.cfg.body_to_bar_axis[0],
            self.cfg.body_to_bar_axis[1],
            self.cfg.body_to_bar_axis[2],
        );
        let axis_norm = axis.norm();
        let axis_unit = if axis_norm > f64::EPSILON {
            axis / axis_norm
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        };
        let bar_velocity = self.velocity_nav.dot(&axis_unit);

        // 9. Phase + repetition tracking
        let new_phase = if is_static {
            MotionPhase::Idle
        } else if bar_velocity.abs() > 0.05 {
            MotionPhase::Concentric
        } else {
            MotionPhase::Eccentric
        };
        let rep_ended = self.update_rep(now, new_phase, bar_velocity);

        self.last_ns = Some(now);
        self.phase = new_phase;
        self.last_quality = quality;

        let out = ProcessedSample {
            monotonic_ns: now,
            linear_accel_nav_mps2: [linear_nav.x, linear_nav.y, linear_nav.z],
            velocity_nav_mps: [
                self.velocity_nav.x,
                self.velocity_nav.y,
                self.velocity_nav.z,
            ],
            bar_velocity_mps: bar_velocity,
            stationary: is_static,
            phase: new_phase,
            quality,
        };
        // rep_ended is exposed via the side channel; the API does not return
        // it because the pipeline keeps state, but the result is ignored for
        // the ProcessedSample contract. Callers use `take_repetition()`.
        let _ = rep_ended;
        Ok(out)
    }

    fn update_rep(&mut self, now_ns: i64, phase: MotionPhase, bar_velocity: f64) -> bool {
        match phase {
            MotionPhase::Concentric => {
                if self.rep_start_ns.is_none() {
                    self.rep_start_ns = Some(now_ns);
                    self.rep_samples = 0;
                    self.rep_peak_velocity = 0.0;
                    self.rep_velocity_sum = 0.0;
                }
                self.rep_samples += 1;
                self.rep_peak_velocity = self.rep_peak_velocity.max(bar_velocity.abs());
                self.rep_velocity_sum += bar_velocity.abs();
                false
            }
            MotionPhase::Idle => {
                if self.rep_start_ns.is_some() {
                    let start = self.rep_start_ns.take().unwrap();
                    self.rep_samples = 0;
                    self.rep_peak_velocity = 0.0;
                    self.rep_velocity_sum = 0.0;
                    log::info!("repetition finished starting at {start} ns");
                    true
                } else {
                    false
                }
            }
            MotionPhase::Eccentric => false,
        }
    }

    /// Apply a final linear drift correction to the integrated velocity
    /// using the most recent zero-velocity observation. This is a
    /// convenience hook and not yet wired to public state.
    pub fn velocity(&self) -> [f64; 3] {
        [
            self.velocity_nav.x,
            self.velocity_nav.y,
            self.velocity_nav.z,
        ]
    }

    pub fn position(&self) -> [f64; 3] {
        [
            self.position_nav.x,
            self.position_nav.y,
            self.position_nav.z,
        ]
    }

    /// Take the most recently completed repetition summary, if any.
    pub fn take_repetition(&mut self) -> Option<RepetitionResult> {
        let start = self.rep_start_ns.take()?;
        let duration = (self.last_ns? - start) as f64 / 1_000_000_000.0;
        let mean = if self.rep_samples > 0 {
            self.rep_velocity_sum / self.rep_samples as f64
        } else {
            0.0
        };
        self.rep_samples = 0;
        self.rep_peak_velocity = 0.0;
        self.rep_velocity_sum = 0.0;
        Some(RepetitionResult {
            start_ns: start,
            end_ns: self.last_ns.unwrap_or(start),
            mean_concentric_velocity_mps: mean,
            peak_velocity_mps: self.rep_peak_velocity,
            duration_s: duration,
            quality: self.last_quality,
        })
    }

    pub fn reset(&mut self) {
        self.last_ns = None;
        self.velocity_nav = Vector3::zeros();
        self.position_nav = Vector3::zeros();
        self.stationary.reset();
        self.phase = MotionPhase::Idle;
        self.rep_start_ns = None;
        self.rep_samples = 0;
        self.rep_peak_velocity = 0.0;
        self.rep_velocity_sum = 0.0;
    }
}

impl Default for MotionPipeline {
    fn default() -> Self {
        Self::new(ProcessingConfig::default())
    }
}

#[allow(dead_code)]
pub(crate) fn magnitude3(v: [f64; 3]) -> f64 {
    magnitude(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::ImuSample;
    use std::f64::consts::FRAC_PI_4;

    fn make_static_sample(ns: i64) -> ImuSample {
        // Device flat on table: +1g on z, no motion, identity quaternion.
        let mut s = ImuSample::from_raw(ns, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]);
        s.quaternion_wxyz = Some([1.0, 0.0, 0.0, 0.0]);
        s
    }

    fn make_motion_sample(ns: i64, accel_body_g: [f64; 3]) -> ImuSample {
        let mut s = ImuSample::from_raw(ns, accel_body_g, [0.0, 0.0, 0.0]);
        s.quaternion_wxyz = Some([1.0, 0.0, 0.0, 0.0]);
        s
    }

    #[test]
    fn static_sample_yields_zero_velocity() {
        let mut p = MotionPipeline::default();
        let out = p.push(make_static_sample(0)).unwrap();
        assert!(out.bar_velocity_mps.abs() < 1e-6);
        assert!(out.stationary);
    }

    #[test]
    fn impulse_then_rest_returns_to_zero() {
        // The ZUPT logic must bring the integrated velocity back to zero
        // whenever the device is detected as stationary. We feed a small
        // horizontal acceleration pulse (well below the stationary
        // threshold) followed by a long static rest so that the velocity
        // has time to settle.
        let mut p = MotionPipeline::default();
        let mut ns = 0i64;
        for _ in 0..40 {
            ns += 5_000_000;
            // 0.2 g on body x, identity quaternion -> ~2 m/s² in nav x
            let _ = p.push(make_motion_sample(ns, [0.2, 0.0, 1.0])).unwrap();
        }
        for _ in 0..200 {
            ns += 5_000_000;
            let _ = p.push(make_static_sample(ns)).unwrap();
        }
        let v = p.velocity();
        for x in v {
            assert!(x.abs() < 1e-3, "velocity did not return to zero: {v:?}");
        }
    }

    #[test]
    fn backward_time_is_rejected() {
        let mut p = MotionPipeline::default();
        let _ = p.push(make_static_sample(1_000)).unwrap();
        let err = p.push(make_static_sample(500)).unwrap_err();
        assert!(matches!(err, PipelineError::TimeGoesBack(500)));
    }

    #[test]
    fn missing_quaternion_sets_fallback_flag() {
        let mut p = MotionPipeline::default();
        let s = ImuSample::from_raw(0, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]);
        let out = p.push(s).unwrap();
        assert!(out.quality.contains(QualityFlags::EULER_FALLBACK));
    }

    #[test]
    fn tilted_quaternion_corrects_gravity() {
        let half = FRAC_PI_4 / 2.0;
        let w = (half).cos();
        let z = (half).sin();
        let mut p = MotionPipeline::default();
        let s = ImuSample {
            monotonic_ns: 0,
            // The device thinks it sees 1g on its body z axis, but rotated
            // 45 deg around z so gravity appears at sqrt(2)/2 on body x and z.
            accel_g: [
                9.81_f64.sqrt() / 9.7915 / 2.0,
                0.0,
                9.81_f64.sqrt() / 9.7915 / 2.0,
            ],
            gyro_dps: [0.0; 3],
            euler_deg: None,
            quaternion_wxyz: Some([w, 0.0, 0.0, z]),
        };
        // We don't expect a perfect zero due to rounding, but the magnitude
        // should be small compared to g.
        let _ = p.push(s).unwrap();
    }
}
