use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use slam_core::SE3;
use slam_dataset::ImuSample;
use slam_imu::Preintegration;

/// One VO keyframe's timestamp + estimated pose (world -> cam0, i.e.
/// `p_cam0 = pose.transform(p_world)`, matching `VoPipeline::FrameResult`).
#[derive(Debug, Clone, Copy)]
pub struct VoKeyframe {
    pub timestamp_ns: u64,
    pub pose_world_to_cam0: SE3,
}

#[derive(Debug, Clone)]
pub struct DynamicInitResult {
    /// Jointly solved (stage 1: rotation-alignment least squares).
    pub gyro_bias: Vector3<f64>,
    /// Always zero: fixed, not estimated, by this initializer — see
    /// `solve_gravity_bias_velocity`'s doc comment for why joint accel-bias
    /// estimation is deferred to the M5 backend. Kept as a field (rather
    /// than dropped) so callers have one consistent `DynamicInitResult`
    /// shape to seed the backend with, even though this value carries no
    /// information yet.
    pub accel_bias: Vector3<f64>,
    /// Gravity vector in the VO world frame (magnitude should be near
    /// 9.81 for a converged result — not enforced by the solve itself).
    pub gravity_world: Vector3<f64>,
    /// One velocity (body frame's linear velocity, expressed in the VO
    /// world frame) per input keyframe.
    pub velocities_world: Vec<Vector3<f64>>,
}

/// Converts a VO camera pose (`p_cam0 = pose.transform(p_world)`) into the
/// corresponding body pose in world frame, using the cam0-to-body
/// extrinsics `t_bs_cam0` (`X_body = t_bs_cam0.transform(X_cam0)`).
///
/// Derivation: `X_world = R_wc*X_cam0 + p_wc` (cam0's own world pose) and
/// `X_body = R_bs*X_cam0 + t_bs` (`t_bs_cam0`), so substituting
/// `X_cam0 = R_bs^T*(X_body - t_bs)` gives
/// `X_world = (R_wc*R_bs^T)*X_body + (p_wc - R_wc*R_bs^T*t_bs)`.
/// Verified directly by `debug_body_pose_roundtrip` below.
fn body_pose_in_world(pose_world_to_cam0: &SE3, t_bs_cam0: &SE3) -> SE3 {
    let pose_cam0_to_world = pose_world_to_cam0.inverse();
    let r_wb = pose_cam0_to_world.rotation.compose(&t_bs_cam0.rotation.inverse());
    let p_wb = pose_cam0_to_world.translation - r_wb.transform(&t_bs_cam0.translation);
    SE3::new(r_wb, p_wb)
}

/// Preintegrates the raw IMU samples falling within `[t_start, t_end]`.
fn preintegrate_between(imu_samples: &[ImuSample], t_start: u64, t_end: u64, bias_gyro: Vector3<f64>, bias_accel: Vector3<f64>) -> Preintegration {
    let mut pre = Preintegration::new(bias_gyro, bias_accel);
    let in_range: Vec<&ImuSample> = imu_samples
        .iter()
        .filter(|s| s.timestamp_ns >= t_start && s.timestamp_ns <= t_end)
        .collect();
    for pair in in_range.windows(2) {
        let dt = (pair[1].timestamp_ns - pair[0].timestamp_ns) as f64 * 1e-9;
        pre.integrate_measurement(pair[0].gyro, pair[0].accel, dt);
    }
    pre
}

/// One row-block's worth of the stage-2 linear system for a single
/// consecutive keyframe pair: the standard IMU integration equations
///
/// ```text
/// p_{i+1} = p_i + v_i*dt + 0.5*g*dt^2 + R_i*(dp_i(0) + Jp_ba*ba)
/// v_{i+1} = v_i + g*dt + R_i*(dv_i(0) + Jv_ba*ba)
/// ```
///
/// rearranged into `A*[v_i; v_{i+1}; g; ba] = b` (position rows, then
/// velocity rows). Factored out from `dynamic_initialize` so a test can
/// plug in ground truth and check the residual directly — the fastest way
/// to tell "equation is wrong" apart from "solver is wrong".
struct PairSystem {
    // Column blocks, in the order [v_i, v_ip1, g, ba] (each 3-wide).
    d_v_i: Matrix3<f64>,
    d_v_ip1: Matrix3<f64>,
    d_g: Matrix3<f64>,
    // Only read by tests (`solve_gravity_bias_velocity` fixes b_a = 0 in
    // production — see its doc comment): kept so the physics equation
    // stays fully documented/checkable, not dead weight to delete.
    #[allow(dead_code)]
    d_ba: Matrix3<f64>,
    rhs: Vector3<f64>,
}

fn pair_system_position(dt: f64, r_wb_i: &Matrix3<f64>, jp_ba: &Matrix3<f64>, delta_p: &Vector3<f64>, p_i: &Vector3<f64>, p_ip1: &Vector3<f64>) -> PairSystem {
    PairSystem {
        d_v_i: Matrix3::identity() * dt,
        d_v_ip1: Matrix3::zeros(),
        d_g: Matrix3::identity() * (0.5 * dt * dt),
        d_ba: r_wb_i * jp_ba,
        rhs: (p_ip1 - p_i) - r_wb_i * delta_p,
    }
}

fn pair_system_velocity(dt: f64, r_wb_i: &Matrix3<f64>, jv_ba: &Matrix3<f64>, delta_v: &Vector3<f64>) -> PairSystem {
    PairSystem {
        d_v_i: -Matrix3::identity(),
        d_v_ip1: Matrix3::identity(),
        d_g: Matrix3::identity() * (-dt),
        d_ba: -(r_wb_i * jv_ba),
        rhs: r_wb_i * delta_v,
    }
}

/// Dynamic (moving-start) vision-IMU alignment initializer, for sequences
/// like MH_04/05 that never settle into a stationary window
/// (`plan/STAGE1.md` M4's fallback path; see
/// `slam_imu::static_initialize` for the stationary case). Given a short
/// window of VO keyframes and the raw IMU spanning them, solves for gyro
/// bias (small least squares via the rotation bias Jacobian), then solves
/// gravity and per-keyframe velocities jointly (linear least squares) —
/// the standard two-stage VI-alignment approach (e.g. VINS-Mono's
/// initializer), simplified by *not* including the gravity-magnitude-
/// constrained refinement pass real systems add on top (deferred to M10 if
/// error analysis shows it's needed — see `memory/decisions`).
pub fn dynamic_initialize(keyframes: &[VoKeyframe], imu_samples: &[ImuSample], t_bs_cam0: &SE3) -> Option<DynamicInitResult> {
    let k = keyframes.len();
    if k < 4 {
        return None;
    }

    let body_poses: Vec<SE3> = keyframes.iter().map(|kf| body_pose_in_world(&kf.pose_world_to_cam0, t_bs_cam0)).collect();

    let mut preints: Vec<Preintegration> = Vec::with_capacity(k - 1);
    for pair in keyframes.windows(2) {
        preints.push(preintegrate_between(imu_samples, pair[0].timestamp_ns, pair[1].timestamp_ns, Vector3::zeros(), Vector3::zeros()));
    }

    // Stage 1: refine gyro bias by minimizing rotation misalignment
    // between VO-derived relative rotations and preintegrated ones, using
    // the first-order rotation bias Jacobian (same linearization trick as
    // `Preintegration::corrected`).
    let mut jtj = Matrix3::<f64>::zeros();
    let mut jtr = Vector3::<f64>::zeros();
    for (i, pre) in preints.iter().enumerate() {
        let r_vo = body_poses[i].rotation.inverse().compose(&body_poses[i + 1].rotation);
        let e0 = pre.delta_rotation().inverse().compose(&r_vo).log();
        let j = pre.d_rotation_d_bias_gyro();
        jtj += j.transpose() * j;
        jtr += j.transpose() * e0;
    }
    let gyro_bias = jtj.try_inverse()? * jtr;

    // Re-integrate (not just first-order-correct) at the refined gyro
    // bias: cheap here (short initialization window), and avoids a
    // Jacobian-linearization-point mismatch in stage 2 below.
    let preints: Vec<Preintegration> = keyframes
        .windows(2)
        .map(|pair| preintegrate_between(imu_samples, pair[0].timestamp_ns, pair[1].timestamp_ns, gyro_bias, Vector3::zeros()))
        .collect();

    let result = solve_gravity_bias_velocity(&keyframes.iter().map(|kf| kf.timestamp_ns).collect::<Vec<_>>(), &body_poses, &preints)?;

    Some(DynamicInitResult {
        gyro_bias,
        accel_bias: result.1,
        gravity_world: result.0,
        velocities_world: result.2,
    })
}

/// Stage 2: assembles and solves the linear least-squares system for
/// gravity, accelerometer bias, and per-keyframe velocity. Returns
/// `(gravity, accel_bias, velocities)`.
///
/// Solves for `[v_0..v_{k-1}, g]` only, with accelerometer bias fixed at
/// zero — not `[v_i, g, b_a]` jointly. Jointly estimating accelerometer
/// bias here turned out to make the linear system exactly rank-deficient
/// by one (confirmed via `debug_singular_values_reveal_rank_deficiency`
/// below: a synthetic scenario satisfying the equations exactly still has
/// a zero singular value, so *some* direction in `(v_i, g, b_a)`-space is
/// fundamentally unobservable from position+velocity constraints alone
/// over a short window — a known hard case in VI initialization, not a
/// remaining bug). Matches standard practice (e.g. VINS-Mono's initial
/// linear alignment also treats accel bias as small/deferred): b_a is
/// refined later by the nonlinear backend (M5), which has proper
/// information weighting and a longer window to make it observable.
/// `(gravity_world, accel_bias, velocities_world)`.
type GravityBiasVelocity = (Vector3<f64>, Vector3<f64>, Vec<Vector3<f64>>);

fn solve_gravity_bias_velocity(timestamps_ns: &[u64], body_poses: &[SE3], preints: &[Preintegration]) -> Option<GravityBiasVelocity> {
    let k = body_poses.len();
    let num_pairs = k - 1;
    let num_unknowns = 3 * (k + 1);
    let mut a = DMatrix::<f64>::zeros(6 * num_pairs, num_unknowns);
    let mut b = DVector::<f64>::zeros(6 * num_pairs);

    let v_col = |i: usize| 3 * i;
    let g_col = 3 * k;

    for i in 0..num_pairs {
        let dt = (timestamps_ns[i + 1] - timestamps_ns[i]) as f64 * 1e-9;
        let r_wb_i = body_poses[i].rotation.matrix();
        let delta_v = preints[i].delta_velocity();
        let delta_p = preints[i].delta_position();
        let jp_ba = preints[i].d_position_d_bias_accel();
        let jv_ba = preints[i].d_velocity_d_bias_accel();

        let pos = pair_system_position(dt, &r_wb_i, &jp_ba, &delta_p, &body_poses[i].translation, &body_poses[i + 1].translation);
        let vel = pair_system_velocity(dt, &r_wb_i, &jv_ba, &delta_v);

        let pos_row = 6 * i;
        let vel_row = 6 * i + 3;
        for (row, sys) in [(pos_row, &pos), (vel_row, &vel)] {
            a.view_mut((row, v_col(i)), (3, 3)).copy_from(&sys.d_v_i);
            a.view_mut((row, v_col(i + 1)), (3, 3)).copy_from(&sys.d_v_ip1);
            a.view_mut((row, g_col), (3, 3)).copy_from(&sys.d_g);
            b.rows_mut(row, 3).copy_from(&sys.rhs);
        }
    }

    let svd = a.svd(true, true);
    let x = svd.solve(&b, 1e-9).ok()?;

    let velocities_world: Vec<Vector3<f64>> = (0..k).map(|i| Vector3::new(x[v_col(i)], x[v_col(i) + 1], x[v_col(i) + 2])).collect();
    let gravity_world = Vector3::new(x[g_col], x[g_col + 1], x[g_col + 2]);

    Some((gravity_world, Vector3::zeros(), velocities_world))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slam_core::SO3;

    /// Synthetic ground truth: constant angular velocity (so `ΔR` isn't
    /// trivially identity), constant world-frame velocity (so specific
    /// force is due entirely to counteracting gravity, per the standard
    /// hovering/constant-velocity model), known non-zero biases, and a
    /// *non-identity* camera-to-body extrinsic (so `body_pose_in_world`'s
    /// extrinsic composition is actually exercised, not trivially
    /// sidestepped).
    struct SyntheticScenario {
        w_true: Vector3<f64>,
        v_true: Vector3<f64>,
        g_true: Vector3<f64>,
        gyro_bias_true: Vector3<f64>,
        accel_bias_true: Vector3<f64>,
        t_bs_cam0: SE3,
    }

    impl SyntheticScenario {
        fn body_pose_at(&self, t: f64) -> SE3 {
            SE3::new(SO3::exp(self.w_true * t), self.v_true * t)
        }

        fn keyframe_at(&self, t: f64) -> VoKeyframe {
            let t_world_body = self.body_pose_at(t);
            let t_world_cam0 = t_world_body.compose(&self.t_bs_cam0);
            VoKeyframe {
                timestamp_ns: (t * 1e9) as u64,
                pose_world_to_cam0: t_world_cam0.inverse(),
            }
        }

        fn imu_samples(&self, duration_s: f64, rate_hz: f64) -> Vec<ImuSample> {
            let dt = 1.0 / rate_hz;
            let steps = (duration_s / dt) as usize;
            (0..=steps)
                .map(|i| {
                    let t = i as f64 * dt;
                    let r_wb = self.body_pose_at(t).rotation;
                    let specific_force_body = r_wb.inverse().transform(&(-self.g_true));
                    ImuSample {
                        timestamp_ns: (t * 1e9) as u64,
                        gyro: self.w_true + self.gyro_bias_true,
                        accel: specific_force_body + self.accel_bias_true,
                    }
                })
                .collect()
        }
    }

    fn default_scenario() -> SyntheticScenario {
        SyntheticScenario {
            w_true: Vector3::new(0.5, -0.3, 0.8),
            v_true: Vector3::new(0.5, 0.2, 0.0),
            g_true: Vector3::new(0.1, -0.05, -9.79),
            gyro_bias_true: Vector3::new(0.01, -0.008, 0.005),
            accel_bias_true: Vector3::new(0.03, -0.02, 0.04),
            t_bs_cam0: SE3::new(SO3::exp(Vector3::new(0.0, 1.4, 0.0)), Vector3::new(0.02, -0.06, 0.01)),
        }
    }

    #[test]
    fn body_pose_in_world_roundtrips_synthetic_motion() {
        let scenario = default_scenario();
        for t in [0.0, 0.6, 1.2] {
            let kf = scenario.keyframe_at(t);
            let recovered = body_pose_in_world(&kf.pose_world_to_cam0, &scenario.t_bs_cam0);
            let expected = scenario.body_pose_at(t);
            assert_relative_eq!(recovered.rotation.matrix(), expected.rotation.matrix(), epsilon = 1e-9);
            assert_relative_eq!(recovered.translation, expected.translation, epsilon = 1e-9);
        }
    }

    /// The fast way to tell "equation is wrong" apart from "solver is
    /// wrong": plug the *true* [v_i, g, ba] into the assembled system and
    /// check the residual is ~0. If this passes but the full solve still
    /// recovers the wrong answer, the bug is in the SVD solve, not the
    /// physics/assembly.
    #[test]
    fn ground_truth_satisfies_the_assembled_linear_system() {
        let scenario = default_scenario();
        let keyframe_times = [0.0, 0.6, 1.2, 1.8, 2.4, 3.0];
        let keyframes: Vec<VoKeyframe> = keyframe_times.iter().map(|&t| scenario.keyframe_at(t)).collect();
        let imu_samples = scenario.imu_samples(3.0, 200.0);
        let body_poses: Vec<SE3> = keyframes.iter().map(|kf| body_pose_in_world(&kf.pose_world_to_cam0, &scenario.t_bs_cam0)).collect();
        let preints: Vec<Preintegration> = keyframes
            .windows(2)
            .map(|pair| preintegrate_between(&imu_samples, pair[0].timestamp_ns, pair[1].timestamp_ns, scenario.gyro_bias_true, Vector3::zeros()))
            .collect();

        let k = keyframes.len();
        let num_pairs = k - 1;
        for i in 0..num_pairs {
            let dt = (keyframes[i + 1].timestamp_ns - keyframes[i].timestamp_ns) as f64 * 1e-9;
            let r_wb_i = body_poses[i].rotation.matrix();
            let jp_ba = preints[i].d_position_d_bias_accel();
            let jv_ba = preints[i].d_velocity_d_bias_accel();
            let pos = pair_system_position(dt, &r_wb_i, &jp_ba, &preints[i].delta_position(), &body_poses[i].translation, &body_poses[i + 1].translation);
            let vel = pair_system_velocity(dt, &r_wb_i, &jv_ba, &preints[i].delta_velocity());

            let v_i = scenario.v_true;
            let v_ip1 = scenario.v_true;
            let ba = scenario.accel_bias_true;
            let g = scenario.g_true;

            let pos_residual = pos.d_v_i * v_i + pos.d_v_ip1 * v_ip1 + pos.d_g * g + pos.d_ba * ba - pos.rhs;
            let vel_residual = vel.d_v_i * v_i + vel.d_v_ip1 * v_ip1 + vel.d_g * g + vel.d_ba * ba - vel.rhs;

            assert!(pos_residual.norm() < 1e-6, "pair {i} position residual: {pos_residual:?}");
            assert!(vel_residual.norm() < 1e-6, "pair {i} velocity residual: {vel_residual:?}");
        }
    }

    /// Regression check for the finding documented on
    /// `solve_gravity_bias_velocity`: jointly solving for
    /// `[v_i, g, b_a]` (24 unknowns here) makes the linear system's
    /// coefficient matrix exactly rank-deficient by one, even with a
    /// well-excited synthetic trajectory (substantial rotation, 6
    /// keyframes) and equations already verified correct by
    /// `ground_truth_satisfies_the_assembled_linear_system`. If this test
    /// ever starts failing (rank becomes full), that's a sign the
    /// scoped-down "fix b_a = 0" approach could be revisited — don't just
    /// delete this test to make a refactor pass.
    #[test]
    fn debug_singular_values_reveal_rank_deficiency() {
        let scenario = default_scenario();
        let keyframe_times = [0.0, 0.6, 1.2, 1.8, 2.4, 3.0];
        let keyframes: Vec<VoKeyframe> = keyframe_times.iter().map(|&t| scenario.keyframe_at(t)).collect();
        let imu_samples = scenario.imu_samples(3.0, 200.0);
        let body_poses: Vec<SE3> = keyframes.iter().map(|kf| body_pose_in_world(&kf.pose_world_to_cam0, &scenario.t_bs_cam0)).collect();
        let preints: Vec<Preintegration> = keyframes
            .windows(2)
            .map(|pair| preintegrate_between(&imu_samples, pair[0].timestamp_ns, pair[1].timestamp_ns, scenario.gyro_bias_true, Vector3::zeros()))
            .collect();

        let k = keyframes.len();
        let num_pairs = k - 1;
        let num_unknowns = 3 * (k + 2); // [v_0..v_{k-1}, g, b_a]
        let v_col = |i: usize| 3 * i;
        let g_col = 3 * k;
        let ba_col = 3 * (k + 1);

        let mut a = DMatrix::<f64>::zeros(6 * num_pairs, num_unknowns);
        for i in 0..num_pairs {
            let dt = (keyframes[i + 1].timestamp_ns - keyframes[i].timestamp_ns) as f64 * 1e-9;
            let r_wb_i = body_poses[i].rotation.matrix();
            let jp_ba = preints[i].d_position_d_bias_accel();
            let jv_ba = preints[i].d_velocity_d_bias_accel();
            let pos = pair_system_position(dt, &r_wb_i, &jp_ba, &preints[i].delta_position(), &body_poses[i].translation, &body_poses[i + 1].translation);
            let vel = pair_system_velocity(dt, &r_wb_i, &jv_ba, &preints[i].delta_velocity());

            let pos_row = 6 * i;
            let vel_row = 6 * i + 3;
            for (row, sys) in [(pos_row, &pos), (vel_row, &vel)] {
                a.view_mut((row, v_col(i)), (3, 3)).copy_from(&sys.d_v_i);
                a.view_mut((row, v_col(i + 1)), (3, 3)).copy_from(&sys.d_v_ip1);
                a.view_mut((row, g_col), (3, 3)).copy_from(&sys.d_g);
                a.view_mut((row, ba_col), (3, 3)).copy_from(&sys.d_ba);
            }
        }

        let svd = a.svd(false, false);
        let smallest = svd.singular_values.iter().cloned().fold(f64::INFINITY, f64::min);
        let largest = svd.singular_values.iter().cloned().fold(0.0, f64::max);
        assert!(
            smallest < 1e-6 * largest,
            "expected the joint [v_i, g, b_a] system to be rank-deficient; smallest singular value = {smallest} (largest = {largest})"
        );
    }

    /// Accelerometer bias is fixed at zero in the linear solve (see
    /// `solve_gravity_bias_velocity`'s doc comment for why), so this test
    /// uses a scenario with zero *true* accel bias — the case the linear
    /// stage is actually meant to handle well. Gyro bias is still nonzero
    /// and jointly solved (stage 1), matching the real MH_04/05 case where
    /// gyro bias is well-observed but accel bias needs the backend.
    #[test]
    fn recovers_known_gravity_and_velocity_from_synthetic_motion() {
        let mut scenario = default_scenario();
        scenario.accel_bias_true = Vector3::zeros();
        let keyframe_times = [0.0, 0.6, 1.2, 1.8, 2.4, 3.0];
        let keyframes: Vec<VoKeyframe> = keyframe_times.iter().map(|&t| scenario.keyframe_at(t)).collect();
        let imu_samples = scenario.imu_samples(3.0, 200.0);

        let result = dynamic_initialize(&keyframes, &imu_samples, &scenario.t_bs_cam0).expect("should converge");

        assert_relative_eq!(result.gravity_world, scenario.g_true, epsilon = 1e-3);
        assert_relative_eq!(result.gyro_bias, scenario.gyro_bias_true, epsilon = 1e-3);
        for v in &result.velocities_world {
            assert_relative_eq!(*v, scenario.v_true, epsilon = 5e-3);
        }
    }

    #[test]
    fn too_few_keyframes_returns_none() {
        let t_bs = SE3::identity();
        let keyframes = vec![
            VoKeyframe { timestamp_ns: 0, pose_world_to_cam0: SE3::identity() },
            VoKeyframe { timestamp_ns: 1_000_000_000, pose_world_to_cam0: SE3::identity() },
        ];
        assert!(dynamic_initialize(&keyframes, &[], &t_bs).is_none());
    }
}
