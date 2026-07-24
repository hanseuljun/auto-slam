use nalgebra::{Matrix3, Matrix6, SMatrix, Vector3};
use slam_core::SO3;

/// 9x9 covariance for the `[rotation; velocity; position]` error state, same
/// ordering `slam_optim::imu_residual`'s 9-dim residual uses.
pub type Covariance9 = SMatrix<f64, 9, 9>;

/// On-manifold IMU preintegration between two keyframes (Forster et al.,
/// "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry").
/// Accumulates the relative rotation/velocity/position increment implied by
/// a stream of raw IMU measurements, plus first-order Jacobians of that
/// increment with respect to the bias linearization point — so a later
/// bias update (e.g. from the backend optimizer) can cheaply correct the
/// preintegrated values without re-integrating from scratch.
///
/// Also propagates the 9x9 covariance of the `[rotation; velocity;
/// position]` error state (`plan/STAGE6.md` M2, `memory/decisions/0022` —
/// closing the gap Stage 1 M5's own deferral and Stage 2 M6's
/// `decisions/0016` both left open: weighting IMU factors with real
/// propagated uncertainty instead of ad hoc constants). Uses the same
/// per-step state-transition recursion Forster et al. themselves publish,
/// re-derived here against this struct's own exact discretization (not
/// copied from a generic table) the same way `slam_optim::imu_factor`'s
/// analytic residual Jacobian was — see that module's own doc comment for
/// why re-deriving against the specific convention in use matters.
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

    /// Continuous-time raw gyro/accel measurement noise density
    /// (`sensor.yaml`'s own `gyroscope_noise_density`/
    /// `accelerometer_noise_density` units: rad/s/sqrt(Hz),
    /// m/s^2/sqrt(Hz)) — the per-step discrete noise covariance used by
    /// covariance propagation is `density^2 / dt` (standard continuous-
    /// to-discrete white noise conversion), *not* `density^2 * dt` (that's
    /// the *integrated*-noise/random-walk scaling `decisions/0016`'s own
    /// `bias_gyro_rw_weight` derivation already uses correctly for a
    /// different quantity — easy to mix the two up, so named explicitly
    /// here rather than left as a bare `f64` parameter at each call site).
    gyro_noise_density: f64,
    accel_noise_density: f64,

    covariance: Covariance9,
}

impl Preintegration {
    pub fn new(bias_gyro_lin: Vector3<f64>, bias_accel_lin: Vector3<f64>, gyro_noise_density: f64, accel_noise_density: f64) -> Self {
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
            gyro_noise_density,
            accel_noise_density,
            covariance: Covariance9::zeros(),
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

        // Covariance propagation: Sigma_{k+1} = A*Sigma_k*A^T + B*Sigma_eta*B^T.
        // `A`/`B` derived by perturbing this exact update (see the module
        // doc comment); computed *before* `d_rotation_d_bias_gyro` is
        // overwritten below, since `A`/`B` only need `r_old`/`a`/`w_dt`,
        // not the bias Jacobians themselves.
        let mut a_mat = SMatrix::<f64, 9, 9>::zeros();
        let exp_neg_wdt = SO3::exp(-w_dt).matrix();
        let neg_r_old_ahat_dt = -r_old * a_hat * dt;
        a_mat.fixed_view_mut::<3, 3>(0, 0).copy_from(&exp_neg_wdt);
        a_mat.fixed_view_mut::<3, 3>(3, 0).copy_from(&neg_r_old_ahat_dt);
        a_mat.fixed_view_mut::<3, 3>(3, 3).copy_from(&Matrix3::identity());
        a_mat.fixed_view_mut::<3, 3>(6, 0).copy_from(&(0.5 * dt * neg_r_old_ahat_dt));
        a_mat.fixed_view_mut::<3, 3>(6, 3).copy_from(&(Matrix3::identity() * dt));
        a_mat.fixed_view_mut::<3, 3>(6, 6).copy_from(&Matrix3::identity());

        let mut b_mat = SMatrix::<f64, 9, 6>::zeros();
        b_mat.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-jr * dt));
        b_mat.fixed_view_mut::<3, 3>(3, 3).copy_from(&(-r_old * dt));
        b_mat.fixed_view_mut::<3, 3>(6, 3).copy_from(&(-0.5 * r_old * dt * dt));

        let mut sigma_eta = Matrix6::<f64>::zeros();
        let gyro_var = self.gyro_noise_density * self.gyro_noise_density / dt;
        let accel_var = self.accel_noise_density * self.accel_noise_density / dt;
        for k in 0..3 {
            sigma_eta[(k, k)] = gyro_var;
            sigma_eta[(3 + k, 3 + k)] = accel_var;
        }

        self.covariance = a_mat * self.covariance * a_mat.transpose() + b_mat * sigma_eta * b_mat.transpose();

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

    /// The 9x9 covariance of the `[rotation; velocity; position]` error
    /// state, from propagating raw gyro/accel measurement noise alone
    /// (`gyro_noise_density`/`accel_noise_density`, assuming perfectly
    /// known bias) — see `total_covariance` for the version that also
    /// includes bias *uncertainty*'s own contribution, the specific gap
    /// `decisions/0016` found missing from the simpler formula it tried
    /// and reverted.
    pub fn covariance(&self) -> Covariance9 {
        self.covariance
    }

    /// `covariance()` plus bias uncertainty's own contribution, propagated
    /// through the same bias Jacobians `corrected()` already uses —
    /// `bias_gyro_rw_density`/`bias_accel_rw_density` are `sensor.yaml`'s
    /// own random-walk densities (already used by `SolverConfig`'s
    /// `bias_gyro_rw_weight`/`bias_accel_rw_weight`, same units); bias
    /// variance accumulated since the linearization point is modeled as
    /// `density^2 * elapsed` (standard random-walk variance growth, the
    /// same scaling `solver_config_from_sensor_noise` already uses for
    /// the bias-random-walk *factor*'s own weight — reused here for a
    /// different purpose: how much that same growing uncertainty should
    /// discount trust in *this* preintegration's own rotation/velocity/
    /// position prediction).
    pub fn total_covariance(&self, bias_gyro_rw_density: f64, bias_accel_rw_density: f64) -> Covariance9 {
        let bg_var = bias_gyro_rw_density * bias_gyro_rw_density * self.elapsed;
        let ba_var = bias_accel_rw_density * bias_accel_rw_density * self.elapsed;

        let mut j_bg = SMatrix::<f64, 9, 3>::zeros();
        j_bg.fixed_view_mut::<3, 3>(0, 0).copy_from(&self.d_rotation_d_bias_gyro);
        j_bg.fixed_view_mut::<3, 3>(3, 0).copy_from(&self.d_velocity_d_bias_gyro);
        j_bg.fixed_view_mut::<3, 3>(6, 0).copy_from(&self.d_position_d_bias_gyro);

        let mut j_ba = SMatrix::<f64, 9, 3>::zeros();
        // Rotation doesn't depend on accel bias (rows 0..3 stay zero).
        j_ba.fixed_view_mut::<3, 3>(3, 0).copy_from(&self.d_velocity_d_bias_accel);
        j_ba.fixed_view_mut::<3, 3>(6, 0).copy_from(&self.d_position_d_bias_accel);

        self.covariance + j_bg * (bg_var * Matrix3::identity()) * j_bg.transpose() + j_ba * (ba_var * Matrix3::identity()) * j_ba.transpose()
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

        let mut pre = Preintegration::new(Vector3::zeros(), Vector3::zeros(), 0.0, 0.0);
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

        let mut pre = Preintegration::new(Vector3::zeros(), Vector3::zeros(), 0.0, 0.0);
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
        let mut pre = Preintegration::new(bias_gyro, bias_accel, 0.0, 0.0);
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

    /// Covariance propagation's own correctness check: Monte Carlo. Draw
    /// many independent noisy measurement streams (same nominal gyro/accel,
    /// independent per-step noise at the configured density), preintegrate
    /// each, and confirm the *sample* covariance of the resulting [rotation
    /// (as a small-angle vector relative to the noise-free mean);
    /// velocity; position] matches the *analytically propagated* one —
    /// the same "don't trust a hand derivation blindly" discipline
    /// `slam_optim::imu_factor`'s analytic Jacobian used, adapted to a
    /// covariance (not a point Jacobian): there's no single "finite
    /// difference" oracle for covariance, so Monte Carlo sampling is the
    /// equivalent cross-check.
    #[test]
    fn covariance_matches_monte_carlo_sampling() {
        let gyro_density = 0.01; // rad/s/sqrt(Hz), same order as EuRoC's ADIS16448.
        let accel_density = 0.02; // m/s^2/sqrt(Hz).
        let dt = 0.005;
        let steps = 40; // 0.2s — short enough for many trials to run fast.
        let w_nominal = Vector3::new(0.2, -0.1, 0.15);
        let a_nominal = Vector3::new(0.3, -0.2, 9.7);

        let mut noise_free = Preintegration::new(Vector3::zeros(), Vector3::zeros(), gyro_density, accel_density);
        for _ in 0..steps {
            noise_free.integrate_measurement(w_nominal, a_nominal, dt);
        }
        let analytic_cov = noise_free.covariance();

        let trials = 4000;
        let mut seed = 999u64;
        let mut next_gaussian = || -> f64 {
            // Box-Muller, using the same LCG pattern already used
            // throughout this crate's own randomized tests.
            let mut u1 = 0.0;
            while u1 <= 1e-12 {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                u1 = (seed >> 33) as f64 / (1u64 << 31) as f64;
            }
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u2 = (seed >> 33) as f64 / (1u64 << 31) as f64;
            (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
        };

        let gyro_sigma = gyro_density / dt.sqrt();
        let accel_sigma = accel_density / dt.sqrt();

        let mut sum = SMatrix::<f64, 9, 1>::zeros();
        let mut sum_outer = SMatrix::<f64, 9, 9>::zeros();
        for _ in 0..trials {
            let mut pre = Preintegration::new(Vector3::zeros(), Vector3::zeros(), gyro_density, accel_density);
            for _ in 0..steps {
                let gyro_noise = Vector3::new(next_gaussian(), next_gaussian(), next_gaussian()) * gyro_sigma;
                let accel_noise = Vector3::new(next_gaussian(), next_gaussian(), next_gaussian()) * accel_sigma;
                pre.integrate_measurement(w_nominal + gyro_noise, a_nominal + accel_noise, dt);
            }
            let drot = (noise_free.delta_rotation().inverse().compose(&pre.delta_rotation())).log();
            let dvel = pre.delta_velocity() - noise_free.delta_velocity();
            let dpos = pre.delta_position() - noise_free.delta_position();
            let mut sample = SMatrix::<f64, 9, 1>::zeros();
            sample.fixed_view_mut::<3, 1>(0, 0).copy_from(&drot);
            sample.fixed_view_mut::<3, 1>(3, 0).copy_from(&dvel);
            sample.fixed_view_mut::<3, 1>(6, 0).copy_from(&dpos);
            sum += sample;
            sum_outer += sample * sample.transpose();
        }
        let mean = sum / (trials as f64);
        let sample_cov = sum_outer / (trials as f64) - mean * mean.transpose();

        // Loose tolerance: Monte Carlo with 4000 trials has real sampling
        // noise (relative error ~ 1/sqrt(2*trials) ~ 1.8% per entry for a
        // chi-square-ish quantity, more for smaller/near-zero entries) —
        // this test is checking the propagation is in the right ballpark
        // and has the right *shape* (diagonal terms in particular), not
        // exact agreement to many digits.
        for k in 0..9 {
            let a = analytic_cov[(k, k)];
            let s = sample_cov[(k, k)];
            assert!(a > 0.0, "diagonal covariance entry {k} should be positive, got {a}");
            let rel_err = (a - s).abs() / a;
            assert!(rel_err < 0.35, "diagonal entry {k}: analytic={a:.6e} monte_carlo={s:.6e} rel_err={rel_err:.3}");
        }
    }
}
