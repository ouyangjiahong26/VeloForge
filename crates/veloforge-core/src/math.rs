//! Math helpers: quaternion utilities, gravity removal and frame conversions.

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

/// Build a unit quaternion from a `wxyz` array. Returns `None` if the
/// quaternion is non-finite or its norm is zero.
pub fn quat_from_wxyz(wxyz: [f64; 4]) -> Option<UnitQuaternion<f64>> {
    if !wxyz.iter().all(|v| v.is_finite()) {
        return None;
    }
    let q = Quaternion::new(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
    let norm = q.norm();
    if norm < f64::EPSILON {
        return None;
    }
    let q = UnitQuaternion::new_normalize(q);
    Some(q)
}

/// Rotate a vector by a unit quaternion.
pub fn rotate_vec(q: UnitQuaternion<f64>, v: Vector3<f64>) -> Vector3<f64> {
    q * v
}

/// Build the rotation matrix that maps navigation frame to body frame from
/// the `wxyz` quaternion. Used to subtract gravity from a body-frame
/// accelerometer measurement.
pub fn nav_to_body_matrix(q: UnitQuaternion<f64>) -> Matrix3<f64> {
    q.to_rotation_matrix().matrix().transpose()
}

/// Project the accelerometer reaction to gravity in body frame. The
/// accelerometer measures specific force (the table pushing up), which
/// for a static device is `+g` along the vertical axis. To get the
/// device's linear acceleration we subtract this expected reaction.
pub fn gravity_in_body(q: UnitQuaternion<f64>, gravity_mps2: f64) -> Vector3<f64> {
    let nav_to_body = nav_to_body_matrix(q);
    nav_to_body * Vector3::new(0.0, 0.0, gravity_mps2)
}

/// Convert body-frame linear acceleration to navigation frame.
pub fn body_to_nav(
    q: UnitQuaternion<f64>,
    body_accel: Vector3<f64>,
) -> Vector3<f64> {
    q * body_accel
}

/// Compute the magnitude (Euclidean norm) of a 3-vector.
pub fn magnitude(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn assert_close(a: Vector3<f64>, b: Vector3<f64>, tol: f64) {
        for (x, y) in a.iter().zip(b.iter()) {
            assert!((x - y).abs() < tol, "got {a:?} want {b:?}");
        }
    }

    #[test]
    fn identity_quaternion_does_not_rotate_gravity() {
        let q = quat_from_wxyz([1.0, 0.0, 0.0, 0.0]).unwrap();
        let g = gravity_in_body(q, 9.81);
        assert_close(g, Vector3::new(0.0, 0.0, 9.81), 1e-9);
    }

    #[test]
    fn gravity_under_roll_rotates_into_body_y() {
        // 90 deg roll around the body x axis: the reaction to gravity
        // moves from body z into body y.
        let half = FRAC_PI_2 / 2.0;
        let q = quat_from_wxyz([half.cos(), half.sin(), 0.0, 0.0]).unwrap();
        let g = gravity_in_body(q, 9.81);
        assert_close(g, Vector3::new(0.0, 9.81, 0.0), 1e-6);
    }

    #[test]
    fn rejects_non_finite_quaternion() {
        assert!(quat_from_wxyz([f64::NAN, 0.0, 0.0, 1.0]).is_none());
        assert!(quat_from_wxyz([0.0, 0.0, 0.0, 0.0]).is_none());
    }
}
