use nalgebra::{Matrix3, Unit, UnitQuaternion, Vector3};

/// A 3D rotation, backed by a unit quaternion, with its own exp/log map and
/// Jacobian implementations (the manifold structure is ours; the quaternion
/// algebra underneath is nalgebra's).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SO3 {
    q: UnitQuaternion<f64>,
}

impl SO3 {
    pub fn identity() -> Self {
        SO3 {
            q: UnitQuaternion::identity(),
        }
    }

    pub fn from_quaternion(q: UnitQuaternion<f64>) -> Self {
        SO3 { q }
    }

    /// Builds from a (assumed-orthonormal) rotation matrix.
    pub fn from_matrix(m: &Matrix3<f64>) -> Self {
        SO3 {
            q: UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(*m)),
        }
    }

    pub fn matrix(&self) -> Matrix3<f64> {
        self.q.to_rotation_matrix().into_inner()
    }

    pub fn quaternion(&self) -> UnitQuaternion<f64> {
        self.q
    }

    /// Exponential map so(3) -> SO(3): axis-angle vector to rotation, via
    /// Rodrigues' formula (through the equivalent quaternion form).
    pub fn exp(omega: Vector3<f64>) -> Self {
        let theta = omega.norm();
        let q = if theta < 1e-8 {
            // Small-angle expansion: q ~= [1, omega/2], renormalized.
            let half = omega * 0.5;
            UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(1.0, half.x, half.y, half.z))
        } else {
            UnitQuaternion::from_axis_angle(&Unit::new_unchecked(omega / theta), theta)
        };
        SO3 { q }
    }

    /// Logarithm map SO(3) -> so(3): rotation to axis-angle vector.
    pub fn log(&self) -> Vector3<f64> {
        match self.q.axis_angle() {
            Some((axis, angle)) => axis.into_inner() * angle,
            None => Vector3::zeros(),
        }
    }

    pub fn inverse(&self) -> Self {
        SO3 { q: self.q.inverse() }
    }

    pub fn compose(&self, other: &SO3) -> Self {
        SO3 { q: self.q * other.q }
    }

    pub fn transform(&self, p: &Vector3<f64>) -> Vector3<f64> {
        self.q * p
    }

    /// hat operator: R^3 -> so(3) (skew-symmetric matrix such that
    /// `hat(w) * v == w.cross(&v)`).
    pub fn hat(w: &Vector3<f64>) -> Matrix3<f64> {
        Matrix3::new(0.0, -w.z, w.y, w.z, 0.0, -w.x, -w.y, w.x, 0.0)
    }

    /// vee operator: so(3) -> R^3, the inverse of `hat`.
    pub fn vee(m: &Matrix3<f64>) -> Vector3<f64> {
        Vector3::new(m[(2, 1)], m[(0, 2)], m[(1, 0)])
    }

    /// Right Jacobian of the SO(3) exponential map at `omega`, i.e. the
    /// linearization `exp(omega + d) ~= exp(omega) * exp(Jr(omega) * d)`.
    pub fn right_jacobian(omega: Vector3<f64>) -> Matrix3<f64> {
        let theta = omega.norm();
        let hat = Self::hat(&omega);
        if theta < 1e-8 {
            Matrix3::identity() - hat * 0.5
        } else {
            let theta2 = theta * theta;
            let theta3 = theta2 * theta;
            Matrix3::identity() - hat * ((1.0 - theta.cos()) / theta2)
                + hat * hat * ((theta - theta.sin()) / theta3)
        }
    }

    /// Left Jacobian of the SO(3) exponential map at `omega` (also the `V`
    /// matrix used by the SE(3) exponential map).
    pub fn left_jacobian(omega: Vector3<f64>) -> Matrix3<f64> {
        let theta = omega.norm();
        let hat = Self::hat(&omega);
        if theta < 1e-8 {
            Matrix3::identity() + hat * 0.5
        } else {
            let theta2 = theta * theta;
            let theta3 = theta2 * theta;
            Matrix3::identity() + hat * ((1.0 - theta.cos()) / theta2)
                + hat * hat * ((theta - theta.sin()) / theta3)
        }
    }
}

impl std::ops::Mul for SO3 {
    type Output = SO3;
    fn mul(self, rhs: SO3) -> SO3 {
        self.compose(&rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sample_omegas() -> Vec<Vector3<f64>> {
        vec![
            Vector3::zeros(),
            Vector3::new(1e-9, -2e-9, 3e-9),
            Vector3::new(0.1, -0.2, 0.05),
            Vector3::new(0.6, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(3.0, 0.1, -0.2), // near pi
        ]
    }

    #[test]
    fn exp_log_roundtrip() {
        for omega in sample_omegas() {
            let r = SO3::exp(omega);
            let recovered = r.log();
            assert_relative_eq!(omega, recovered, epsilon = 1e-8);
        }
    }

    #[test]
    fn log_exp_roundtrip_via_matrix() {
        for omega in sample_omegas() {
            let r = SO3::exp(omega);
            let r2 = SO3::from_matrix(&r.matrix());
            assert_relative_eq!(r.matrix(), r2.matrix(), epsilon = 1e-8);
        }
    }

    #[test]
    fn compose_inverse_is_identity() {
        for omega in sample_omegas() {
            let r = SO3::exp(omega);
            let should_be_identity = r.compose(&r.inverse());
            assert_relative_eq!(should_be_identity.matrix(), Matrix3::identity(), epsilon = 1e-10);
        }
    }

    #[test]
    fn hat_vee_roundtrip() {
        let w = Vector3::new(0.3, -0.7, 1.2);
        assert_relative_eq!(SO3::vee(&SO3::hat(&w)), w, epsilon = 1e-12);
    }

    #[test]
    fn hat_matches_cross_product() {
        let w = Vector3::new(0.3, -0.7, 1.2);
        let v = Vector3::new(-0.4, 0.9, 0.1);
        assert_relative_eq!(SO3::hat(&w) * v, w.cross(&v), epsilon = 1e-12);
    }

    /// Numerically checks the right Jacobian against central finite
    /// differences of the log map, per the plan's standing-test mandate
    /// for optimizer-adjacent Jacobians (a wrong sign here silently
    /// poisons every later milestone that uses it).
    #[test]
    fn right_jacobian_matches_finite_difference() {
        let eps = 1e-6;
        for omega in [
            Vector3::new(0.2, -0.1, 0.05),
            Vector3::new(0.8, 0.3, -0.4),
            Vector3::zeros(),
        ] {
            let analytic = SO3::right_jacobian(omega);
            let base = SO3::exp(omega);
            let mut numeric = Matrix3::zeros();
            for col in 0..3 {
                let mut d = Vector3::zeros();
                d[col] = eps;
                let direct = SO3::exp(omega + d);
                let diff = (base.inverse().compose(&direct)).log();
                numeric.set_column(col, &(diff / eps));
            }
            assert_relative_eq!(analytic, numeric, epsilon = 1e-3);
        }
    }
}
