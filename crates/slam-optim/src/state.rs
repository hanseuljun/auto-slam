use nalgebra::{SVector, Vector3};
use slam_core::SE3;

/// The full IMU state carried at one keyframe: pose, velocity, and the two
/// (slowly-varying) IMU biases. 15 tangent-space DoF total: 6 (pose) + 3
/// (velocity) + 3 (gyro bias) + 3 (accel bias), in that order — every
/// Jacobian in this crate follows this same column ordering.
#[derive(Debug, Clone, Copy)]
pub struct KeyframeState {
    pub pose: SE3,
    pub velocity: Vector3<f64>,
    pub bias_gyro: Vector3<f64>,
    pub bias_accel: Vector3<f64>,
}

pub const STATE_DIM: usize = 15;

impl KeyframeState {
    pub fn new(pose: SE3, velocity: Vector3<f64>, bias_gyro: Vector3<f64>, bias_accel: Vector3<f64>) -> Self {
        KeyframeState {
            pose,
            velocity,
            bias_gyro,
            bias_accel,
        }
    }

    /// Applies a tangent-space update: pose via a left-multiplicative SE3
    /// retraction (same convention as `slam_geometry::refine_pose_gauss_newton`),
    /// velocity/biases via plain vector addition.
    pub fn retract(&self, delta: &SVector<f64, STATE_DIM>) -> KeyframeState {
        let pose_delta = delta.fixed_rows::<6>(0).into_owned();
        KeyframeState {
            pose: SE3::exp(pose_delta) * self.pose,
            velocity: self.velocity + delta.fixed_rows::<3>(6),
            bias_gyro: self.bias_gyro + delta.fixed_rows::<3>(9),
            bias_accel: self.bias_accel + delta.fixed_rows::<3>(12),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slam_core::SO3;

    #[test]
    fn zero_delta_is_identity_retraction() {
        let state = KeyframeState::new(
            SE3::new(SO3::exp(Vector3::new(0.1, -0.2, 0.05)), Vector3::new(1.0, 2.0, 3.0)),
            Vector3::new(0.5, 0.1, -0.2),
            Vector3::new(0.01, 0.02, -0.01),
            Vector3::new(0.05, -0.03, 0.02),
        );
        let retracted = state.retract(&SVector::<f64, STATE_DIM>::zeros());
        assert_relative_eq!(retracted.pose.matrix(), state.pose.matrix(), epsilon = 1e-12);
        assert_relative_eq!(retracted.velocity, state.velocity, epsilon = 1e-12);
        assert_relative_eq!(retracted.bias_gyro, state.bias_gyro, epsilon = 1e-12);
        assert_relative_eq!(retracted.bias_accel, state.bias_accel, epsilon = 1e-12);
    }

    #[test]
    fn retract_moves_each_block_independently() {
        let state = KeyframeState::new(SE3::identity(), Vector3::zeros(), Vector3::zeros(), Vector3::zeros());
        let mut delta = SVector::<f64, STATE_DIM>::zeros();
        delta[6] = 0.5; // velocity x
        delta[9] = 0.1; // gyro bias x
        delta[12] = -0.2; // accel bias x
        let retracted = state.retract(&delta);
        assert_relative_eq!(retracted.pose.matrix(), SE3::identity().matrix(), epsilon = 1e-12);
        assert_relative_eq!(retracted.velocity, Vector3::new(0.5, 0.0, 0.0), epsilon = 1e-12);
        assert_relative_eq!(retracted.bias_gyro, Vector3::new(0.1, 0.0, 0.0), epsilon = 1e-12);
        assert_relative_eq!(retracted.bias_accel, Vector3::new(-0.2, 0.0, 0.0), epsilon = 1e-12);
    }
}
