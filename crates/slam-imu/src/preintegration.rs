use nalgebra::{Matrix3, Vector3};
use slam_core::SO3;

/// On-manifold IMU preintegration between two keyframes (Forster et al.,
/// "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry").
/// Accumulates the relative rotation/velocity/position increment implied by
/// a stream of raw IMU measurements, plus first-order Jacobians of that
/// increment with respect to the bias linearization point — so a later
/// bias update (e.g. from the backend optimizer) can cheaply correct the
/// preintegrated values without re-integrating from scratch.
///
/// Covariance propagation (needed to weight IMU factors in the backend) is
/// deferred to M5, where it has an actual consumer — see `memory/decisions`.
#[derive(Debug, Clone)]
pub struct Preintegration {
    bias_gyro_lin: Vector3<f64>,
    bias_accel_lin: Vector3<f64>,

    delta_rotation: SO3,
    delta_velocity: Vector3<f64>,
    delta_position: Vector3<f64>,
    elapsed: f64,

    d_rotation_d_bias_gyro: Matrix3<f64>,
    d_velocity_d_bias_gyro: Matrix3<f64>,
    d_velocity_d_bias_accel: Matrix3<f64>,
    d_position_d_bias_gyro: Matrix3<f64>,
    d_position_d_bias_accel: Matrix3<f64>,
}

impl Preintegration {
    pub fn new(bias_gyro_lin: Vector3<f64>, bias_accel_lin: Vector3<f64>) -> Self {
        Preintegration {
            bias_gyro_lin,
            bias_accel_lin,
            delta_rotation: SO3::identity(),
            delta_velocity: Vector3::zeros(),
            delta_position: Vector3::zeros(),
            elapsed: 0.0,
            d_rotation_d_bias_gyro: Matrix3::zeros(),
            d_velocity_d_bias_gyro: Matrix3::zeros(),
            d_velocity_d_bias_accel: Matrix3::zeros(),
            d_position_d_bias_gyro: Matrix3::zeros(),
            d_position_d_bias_accel: Matrix3::zeros(),
        }
    }

    /// Integrates one raw (un-bias-corrected) IMU sample over `dt` seconds.
    pub fn integrate_measurement(&mut self, raw_gyro: Vector3<f64>, raw_accel: Vector3<f64>, dt: f64) {
        let w = raw_gyro - self.bias_gyro_lin;
        let a = raw_accel - self.bias_accel_lin;

        let r_old = self.delta_rotation.matrix();

        self.delta_position += self.delta_velocity * dt + 0.5 * r_old * a * dt * dt;
        self.delta_velocity += r_old * a * dt;

        let a_hat = SO3::hat(&a);
        self.d_position_d_bias_accel +=
            self.d_velocity_d_bias_accel * dt - 0.5 * r_old * dt * dt;
        self.d_position_d_bias_gyro +=
            self.d_velocity_d_bias_gyro * dt - 0.5 * r_old * a_hat * self.d_rotation_d_bias_gyro * dt * dt;
        self.d_velocity_d_bias_accel += -r_old * dt;
        self.d_velocity_d_bias_gyro += -r_old * a_hat * self.d_rotation_d_bias_gyro * dt;

        let w_dt = w * dt;
        let delta_r_step = SO3::exp(w_dt);
        let jr = SO3::right_jacobian(w_dt);
        self.d_rotation_d_bias_gyro = delta_r_step.matrix().transpose() * self.d_rotation_d_bias_gyro - jr * dt;
        self.delta_rotation = self.delta_rotation.compose(&delta_r_step);

        self.elapsed += dt;
    }

    pub fn delta_rotation(&self) -> SO3 {
        self.delta_rotation
    }
    pub fn delta_velocity(&self) -> Vector3<f64> {
        self.delta_velocity
    }
    pub fn delta_position(&self) -> Vector3<f64> {
        self.delta_position
    }
    pub fn elapsed(&self) -> f64 {
        self.elapsed
    }
    pub fn bias_gyro_lin(&self) -> Vector3<f64> {
        self.bias_gyro_lin
    }
    pub fn bias_accel_lin(&self) -> Vector3<f64> {
        self.bias_accel_lin
    }
    pub fn d_rotation_d_bias_gyro(&self) -> Matrix3<f64> {
        self.d_rotation_d_bias_gyro
    }
    pub fn d_velocity_d_bias_gyro(&self) -> Matrix3<f64> {
        self.d_velocity_d_bias_gyro
    }
    pub fn d_velocity_d_bias_accel(&self) -> Matrix3<f64> {
        self.d_velocity_d_bias_accel
    }
    pub fn d_position_d_bias_gyro(&self) -> Matrix3<f64> {
        self.d_position_d_bias_gyro
    }
    pub fn d_position_d_bias_accel(&self) -> Matrix3<f64> {
        self.d_position_d_bias_accel
    }

    /// First-order-corrected preintegrated rotation/velocity/position for a
    /// bias estimate near (not necessarily equal to) the linearization
    /// point this preintegration was integrated with.
    pub fn corrected(&self, bias_gyro: Vector3<f64>, bias_accel: Vector3<f64>) -> (SO3, Vector3<f64>, Vector3<f64>) {
        let d_bg = bias_gyro - self.bias_gyro_lin;
        let d_ba = bias_accel - self.bias_accel_lin;

        let corrected_rotation = self
            .delta_rotation
            .compose(&SO3::exp(self.d_rotation_d_bias_gyro * d_bg));
        let corrected_velocity =
            self.delta_velocity + self.d_velocity_d_bias_gyro * d_bg + self.d_velocity_d_bias_accel * d_ba;
        let corrected_position =
            self.delta_position + self.d_position_d_bias_gyro * d_bg + self.d_position_d_bias_accel * d_ba;

        (corrected_rotation, corrected_velocity, corrected_position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Constant angular velocity, zero acceleration: ΔR should match the
    /// closed-form `Exp(w * total_time)` regardless of step count.
    #[test]
    fn constant_rotation_matches_closed_form() {
        let w = Vector3::new(0.1, -0.2, 0.05);
        let dt = 0.005;
        let steps = 400; // 2 seconds at 200Hz, matching EuRoC's imu0 rate.

        let mut pre = Preintegration::new(Vector3::zeros(), Vector3::zeros());
        for _ in 0..steps {
            pre.integrate_measurement(w, Vector3::zeros(), dt);
        }

        let expected = SO3::exp(w * (steps as f64 * dt));
        assert_relative_eq!(pre.delta_rotation().matrix(), expected.matrix(), epsilon = 1e-6);
    }

    /// Zero rotation, constant acceleration: Δv, Δp should match the
    /// closed-form constant-acceleration kinematics.
    #[test]
    fn constant_acceleration_matches_closed_form() {
        let a = Vector3::new(1.0, 0.0, -0.5);
        let dt = 0.005;
        let steps = 400;
        let t = steps as f64 * dt;

        let mut pre = Preintegration::new(Vector3::zeros(), Vector3::zeros());
        for _ in 0..steps {
            pre.integrate_measurement(Vector3::zeros(), a, dt);
        }

        // Discrete forward-Euler integration accumulates O(dt) truncation
        // error relative to the continuous closed form; loose tolerance
        // reflects that, not a bug.
        assert_relative_eq!(pre.delta_velocity(), a * t, epsilon = 1e-3);
        assert_relative_eq!(pre.delta_position(), 0.5 * a * t * t, epsilon = 1e-3);
    }

    fn random_measurements(seed: u64, n: usize) -> Vec<(Vector3<f64>, Vector3<f64>)> {
        let mut state = seed;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        (0..n)
            .map(|_| {
                (
                    Vector3::new(next(), next(), next()) * 0.5,
                    Vector3::new(next(), next(), next()) * 8.0 + Vector3::new(0.0, 0.0, 9.81),
                )
            })
            .collect()
    }

    fn integrate_with_bias(
        measurements: &[(Vector3<f64>, Vector3<f64>)],
        dt: f64,
        bias_gyro: Vector3<f64>,
        bias_accel: Vector3<f64>,
    ) -> Preintegration {
        let mut pre = Preintegration::new(bias_gyro, bias_accel);
        for &(w, a) in measurements {
            pre.integrate_measurement(w, a, dt);
        }
        pre
    }

    /// The whole point of the bias Jacobians: re-integrating from scratch
    /// with a perturbed bias should match the first-order `corrected()`
    /// prediction from the *unperturbed* linearization, to first order.
    /// Same finite-difference-vs-analytic discipline as SO3's right
    /// Jacobian test in slam-core, for the same reason (a silent sign bug
    /// here poisons every later milestone that consumes it).
    #[test]
    fn bias_jacobians_match_finite_difference_reintegration() {
        let measurements = random_measurements(42, 60);
        let dt = 0.005;
        let bg0 = Vector3::new(0.01, -0.02, 0.005);
        let ba0 = Vector3::new(0.05, -0.03, 0.02);

        let base = integrate_with_bias(&measurements, dt, bg0, ba0);

        let eps = 1e-6;
        for axis in 0..3 {
            let mut d_bg = Vector3::zeros();
            d_bg[axis] = eps;
            let perturbed = integrate_with_bias(&measurements, dt, bg0 + d_bg, ba0);

            let (pred_r, pred_v, pred_p) = base.corrected(bg0 + d_bg, ba0);
            assert_relative_eq!(pred_r.matrix(), perturbed.delta_rotation().matrix(), epsilon = 1e-5);
            assert_relative_eq!(pred_v, perturbed.delta_velocity(), epsilon = 1e-4);
            assert_relative_eq!(pred_p, perturbed.delta_position(), epsilon = 1e-4);
        }

        for axis in 0..3 {
            let mut d_ba = Vector3::zeros();
            d_ba[axis] = eps;
            let perturbed = integrate_with_bias(&measurements, dt, bg0, ba0 + d_ba);

            let (_, pred_v, pred_p) = base.corrected(bg0, ba0 + d_ba);
            assert_relative_eq!(pred_v, perturbed.delta_velocity(), epsilon = 1e-4);
            assert_relative_eq!(pred_p, perturbed.delta_position(), epsilon = 1e-4);
        }
    }
}
