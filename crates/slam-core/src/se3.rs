use nalgebra::{Matrix4, Vector3, Vector6};

use crate::so3::SO3;

/// A rigid-body transform (rotation + translation), with its own exp/log map
/// built on top of `SO3`.
#[derive(Debug, Clone, Copy)]
pub struct SE3 {
    pub rotation: SO3,
    pub translation: Vector3<f64>,
}

impl SE3 {
    pub fn identity() -> Self {
        SE3 {
            rotation: SO3::identity(),
            translation: Vector3::zeros(),
        }
    }

    pub fn new(rotation: SO3, translation: Vector3<f64>) -> Self {
        SE3 {
            rotation,
            translation,
        }
    }

    pub fn from_matrix(m: &Matrix4<f64>) -> Self {
        let r = m.fixed_view::<3, 3>(0, 0).into_owned();
        let t = m.fixed_view::<3, 1>(0, 3).into_owned();
        SE3 {
            rotation: SO3::from_matrix(&r),
            translation: Vector3::new(t[0], t[1], t[2]),
        }
    }

    pub fn matrix(&self) -> Matrix4<f64> {
        let mut m = Matrix4::identity();
        m.fixed_view_mut::<3, 3>(0, 0).copy_from(&self.rotation.matrix());
        m.fixed_view_mut::<3, 1>(0, 3).copy_from(&self.translation);
        m
    }

    pub fn inverse(&self) -> Self {
        let r_inv = self.rotation.inverse();
        SE3 {
            rotation: r_inv,
            translation: -r_inv.transform(&self.translation),
        }
    }

    pub fn compose(&self, other: &SE3) -> Self {
        SE3 {
            rotation: self.rotation.compose(&other.rotation),
            translation: self.translation + self.rotation.transform(&other.translation),
        }
    }

    pub fn transform(&self, p: &Vector3<f64>) -> Vector3<f64> {
        self.rotation.transform(p) + self.translation
    }

    /// Exponential map se(3) -> SE(3), `xi = [rho; phi]` (translation part
    /// first, rotation part second).
    pub fn exp(xi: Vector6<f64>) -> Self {
        let rho = xi.fixed_rows::<3>(0).into_owned();
        let phi = xi.fixed_rows::<3>(3).into_owned();
        let rotation = SO3::exp(phi);
        let v = SO3::left_jacobian(phi);
        SE3 {
            rotation,
            translation: v * rho,
        }
    }

    /// Logarithm map SE(3) -> se(3).
    pub fn log(&self) -> Vector6<f64> {
        let phi = self.rotation.log();
        let v = SO3::left_jacobian(phi);
        let v_inv = v
            .try_inverse()
            .expect("SE3 left-Jacobian V is singular (should never happen away from theta=2*pi*n)");
        let rho = v_inv * self.translation;
        let mut xi = Vector6::zeros();
        xi.fixed_rows_mut::<3>(0).copy_from(&rho);
        xi.fixed_rows_mut::<3>(3).copy_from(&phi);
        xi
    }
}

impl std::ops::Mul for SE3 {
    type Output = SE3;
    fn mul(self, rhs: SE3) -> SE3 {
        self.compose(&rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sample_xis() -> Vec<Vector6<f64>> {
        vec![
            Vector6::zeros(),
            Vector6::new(1.0, 2.0, 3.0, 0.1, -0.2, 0.05),
            Vector6::new(-0.5, 0.2, 0.1, 0.6, 0.0, 0.0),
            Vector6::new(0.01, 0.02, -0.03, 1.0, 1.0, 1.0),
        ]
    }

    #[test]
    fn exp_log_roundtrip() {
        for xi in sample_xis() {
            let t = SE3::exp(xi);
            assert_relative_eq!(xi, t.log(), epsilon = 1e-7);
        }
    }

    #[test]
    fn compose_inverse_is_identity() {
        for xi in sample_xis() {
            let t = SE3::exp(xi);
            let identity = t.compose(&t.inverse());
            assert_relative_eq!(identity.matrix(), Matrix4::identity(), epsilon = 1e-9);
        }
    }

    #[test]
    fn transform_matches_matrix_multiplication() {
        let t = SE3::exp(Vector6::new(1.0, -2.0, 0.5, 0.3, -0.1, 0.2));
        let p = Vector3::new(0.7, -1.3, 2.1);
        let via_struct = t.transform(&p);
        let homogeneous = t.matrix() * nalgebra::Vector4::new(p.x, p.y, p.z, 1.0);
        assert_relative_eq!(via_struct, homogeneous.fixed_rows::<3>(0).into_owned(), epsilon = 1e-10);
    }
}
