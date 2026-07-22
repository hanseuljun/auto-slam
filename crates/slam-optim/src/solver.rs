use std::collections::HashMap;

use nalgebra::{DMatrix, DVector, Dim, Matrix3, SMatrix, SVector, Storage, Vector2, Vector3};
use slam_core::SE3;
use slam_imu::Preintegration;

use crate::bias_random_walk::bias_random_walk_residual_jacobian;
use crate::huber::huber_weight;
use crate::imu_factor::imu_residual_jacobian;
use crate::reprojection::reprojection_residual_jacobian;
use crate::state::{KeyframeState, STATE_DIM};

pub struct ReprojectionObservation {
    pub keyframe_idx: usize,
    pub landmark_idx: usize,
    pub t_bs_cam: SE3,
    pub observed_normalized: Vector2<f64>,
}

pub struct ImuFactorSpec {
    pub i: usize,
    pub j: usize,
    pub preint: Preintegration,
    pub dt: f64,
}

pub struct BiasRwFactorSpec {
    pub i: usize,
    pub j: usize,
}

/// Ad hoc (not covariance-propagated) isotropic information weights for
/// each residual type, plus the Huber threshold for reprojection
/// residuals. `plan/STAGE1.md` explicitly earmarks replacing these with
/// weights derived from `sensor.yaml`'s real noise densities (and, for
/// the IMU factor, real preintegration covariance propagation — deferred
/// since M4, see `memory/decisions`) for M10's "accuracy closing pass";
/// this scope is intentional, not an oversight.
#[derive(Debug, Clone, Copy)]
pub struct SolverConfig {
    pub max_iterations: usize,
    pub initial_lambda: f64,
    /// 1/sigma^2 in normalized-image-coordinate units.
    pub reprojection_weight: f64,
    /// Huber threshold, in *weighted*
    /// (`sqrt(reprojection_weight)`-scaled) residual-norm units.
    pub huber_delta: f64,
    pub imu_rotation_weight: f64,
    pub imu_velocity_weight: f64,
    pub imu_position_weight: f64,
    pub bias_rw_weight: f64,
}

impl Default for SolverConfig {
    fn default() -> Self {
        SolverConfig {
            max_iterations: 10,
            initial_lambda: 1e-3,
            reprojection_weight: 1.0 / (0.002 * 0.002), // ~2px at f~460 -> sigma ~2e-3 normalized
            huber_delta: 3.0,
            imu_rotation_weight: 1.0 / (0.02 * 0.02),
            imu_velocity_weight: 1.0 / (0.05 * 0.05),
            imu_position_weight: 1.0 / (0.02 * 0.02),
            bias_rw_weight: 1.0 / (0.01 * 0.01),
        }
    }
}

/// A sliding-window bundle-adjustment-style problem: `keyframes[0]` is
/// held fixed (the gauge anchor — see `memory/decisions` for why a single
/// fixed pose is used instead of a full null-space projection);
/// `keyframes[1..]` and all `landmarks` are optimized.
pub struct Problem {
    pub keyframes: Vec<KeyframeState>,
    pub landmarks: Vec<Vector3<f64>>,
    pub reprojection_obs: Vec<ReprojectionObservation>,
    pub imu_factors: Vec<ImuFactorSpec>,
    pub bias_rw_factors: Vec<BiasRwFactorSpec>,
    pub gravity_world: Vector3<f64>,
}

fn free_index(keyframe_idx: usize) -> Option<usize> {
    keyframe_idx.checked_sub(1)
}

/// Adds `block` into `h`'s `(row, col)` sub-block, for any statically- or
/// dynamically-sized nalgebra matrix — avoids needing ad hoc view-type
/// conversions at each call site.
fn add_block<R: Dim, C: Dim, S: Storage<f64, R, C>>(h: &mut DMatrix<f64>, row: usize, col: usize, block: &nalgebra::Matrix<f64, R, C, S>) {
    let (nrows, ncols) = block.shape();
    let mut view = h.view_mut((row, col), (nrows, ncols));
    for r in 0..nrows {
        for c in 0..ncols {
            view[(r, c)] += block[(r, c)];
        }
    }
}

fn sub_rows<R: Dim, S: Storage<f64, R>>(v: &mut DVector<f64>, row: usize, block: &nalgebra::Matrix<f64, R, nalgebra::U1, S>) {
    for r in 0..block.nrows() {
        v[row + r] -= block[r];
    }
}

struct LandmarkSchur {
    h_ll_inv: Matrix3<f64>,
    b_l: Vector3<f64>,
    h_lp: HashMap<usize, SMatrix<f64, 3, STATE_DIM>>,
}

struct NormalEquations {
    h_pp: DMatrix<f64>,
    b_p: DVector<f64>,
    landmark_schur: HashMap<usize, LandmarkSchur>,
}

/// Sum of weighted (and, for reprojection, Huber-downweighted) squared
/// residuals at the current state — no Jacobians, used for LM's
/// accept/reject cost comparison.
fn compute_cost(problem: &Problem, config: &SolverConfig) -> f64 {
    let mut cost = 0.0;
    for f in &problem.imu_factors {
        let r = crate::imu_factor::imu_residual(&problem.keyframes[f.i], &problem.keyframes[f.j], &f.preint, problem.gravity_world, f.dt);
        for k in 0..3 {
            cost += config.imu_rotation_weight * r[k] * r[k];
            cost += config.imu_velocity_weight * r[3 + k] * r[3 + k];
            cost += config.imu_position_weight * r[6 + k] * r[6 + k];
        }
    }
    for f in &problem.bias_rw_factors {
        let (r, _, _) = bias_random_walk_residual_jacobian(&problem.keyframes[f.i], &problem.keyframes[f.j]);
        cost += config.bias_rw_weight * r.norm_squared();
    }
    let sqrt_reproj_w = config.reprojection_weight.sqrt();
    for obs in &problem.reprojection_obs {
        let Some((r, _, _)) = reprojection_residual_jacobian(&problem.keyframes[obs.keyframe_idx], &obs.t_bs_cam, problem.landmarks[obs.landmark_idx], obs.observed_normalized) else {
            continue;
        };
        let weighted_norm = (r * sqrt_reproj_w).norm();
        let w = sqrt_reproj_w * huber_weight(weighted_norm, config.huber_delta);
        cost += (r * w).norm_squared();
    }
    cost
}

/// Builds the reduced (landmarks Schur-eliminated) normal equations at the
/// current state, plus enough per-landmark data
/// (`h_ll_inv`, `b_l`, `h_lp`) to back-substitute landmark updates once
/// the reduced system is solved for a pose delta.
fn build_normal_equations(problem: &Problem, config: &SolverConfig) -> NormalEquations {
    let num_free = problem.keyframes.len() - 1;
    let dim = num_free * STATE_DIM;
    let mut h_pp = DMatrix::<f64>::zeros(dim, dim);
    let mut b_p = DVector::<f64>::zeros(dim);

    for f in &problem.imu_factors {
        let (r, jac_i, jac_j) = imu_residual_jacobian(&problem.keyframes[f.i], &problem.keyframes[f.j], &f.preint, problem.gravity_world, f.dt);
        let mut sqrt_w = SVector::<f64, 9>::zeros();
        for k in 0..3 {
            sqrt_w[k] = config.imu_rotation_weight.sqrt();
            sqrt_w[3 + k] = config.imu_velocity_weight.sqrt();
            sqrt_w[6 + k] = config.imu_position_weight.sqrt();
        }
        let wr = r.component_mul(&sqrt_w);
        let wji = SMatrix::<f64, 9, STATE_DIM>::from_fn(|r, c| jac_i[(r, c)] * sqrt_w[r]);
        let wjj = SMatrix::<f64, 9, STATE_DIM>::from_fn(|r, c| jac_j[(r, c)] * sqrt_w[r]);
        accumulate_pair(&mut h_pp, &mut b_p, free_index(f.i), free_index(f.j), &wji, &wjj, &wr);
    }

    for f in &problem.bias_rw_factors {
        let (r, jac_i, jac_j) = bias_random_walk_residual_jacobian(&problem.keyframes[f.i], &problem.keyframes[f.j]);
        let w = config.bias_rw_weight.sqrt();
        let wr = r * w;
        let wji = jac_i * w;
        let wjj = jac_j * w;
        accumulate_pair(&mut h_pp, &mut b_p, free_index(f.i), free_index(f.j), &wji, &wjj, &wr);
    }

    let mut by_landmark: HashMap<usize, Vec<&ReprojectionObservation>> = HashMap::new();
    for obs in &problem.reprojection_obs {
        by_landmark.entry(obs.landmark_idx).or_default().push(obs);
    }

    let mut landmark_schur = HashMap::new();
    let sqrt_reproj_w = config.reprojection_weight.sqrt();
    for (&landmark_idx, obs_list) in &by_landmark {
        let landmark = problem.landmarks[landmark_idx];
        let mut h_ll = Matrix3::<f64>::zeros();
        let mut b_l = Vector3::<f64>::zeros();
        let mut h_lp: HashMap<usize, SMatrix<f64, 3, STATE_DIM>> = HashMap::new();

        for obs in obs_list {
            let Some((r, jac_pose, jac_landmark)) = reprojection_residual_jacobian(&problem.keyframes[obs.keyframe_idx], &obs.t_bs_cam, landmark, obs.observed_normalized) else {
                continue;
            };
            let weighted_norm = (r * sqrt_reproj_w).norm();
            let w = sqrt_reproj_w * huber_weight(weighted_norm, config.huber_delta);

            let wr = r * w;
            let wjp = jac_pose * w;
            let wjl = jac_landmark * w;

            h_ll += wjl.transpose() * wjl;
            b_l += -(wjl.transpose() * wr);

            if let Some(fk) = free_index(obs.keyframe_idx) {
                *h_lp.entry(fk).or_insert_with(SMatrix::<f64, 3, STATE_DIM>::zeros) += wjl.transpose() * wjp;

                let rk = fk * STATE_DIM;
                add_block(&mut h_pp, rk, rk, &(wjp.transpose() * wjp));
                sub_rows(&mut b_p, rk, &(wjp.transpose() * wr));
            }
        }

        let h_ll_reg = h_ll + Matrix3::identity() * 1e-6;
        let Some(h_ll_inv) = h_ll_reg.try_inverse() else {
            continue;
        };

        for (&fk1, hlp1) in &h_lp {
            let r1 = fk1 * STATE_DIM;
            sub_rows(&mut b_p, r1, &(hlp1.transpose() * h_ll_inv * b_l));
            for (&fk2, hlp2) in &h_lp {
                let r2 = fk2 * STATE_DIM;
                let contribution = -(hlp1.transpose() * h_ll_inv * hlp2);
                add_block(&mut h_pp, r1, r2, &contribution);
            }
        }

        landmark_schur.insert(landmark_idx, LandmarkSchur { h_ll_inv, b_l, h_lp });
    }

    NormalEquations { h_pp, b_p, landmark_schur }
}

fn accumulate_pair<const N: usize>(h_pp: &mut DMatrix<f64>, b_p: &mut DVector<f64>, fi: Option<usize>, fj: Option<usize>, wji: &SMatrix<f64, N, STATE_DIM>, wjj: &SMatrix<f64, N, STATE_DIM>, wr: &SVector<f64, N>) {
    if let Some(fi) = fi {
        let ri = fi * STATE_DIM;
        add_block(h_pp, ri, ri, &(wji.transpose() * wji));
        sub_rows(b_p, ri, &(wji.transpose() * wr));
    }
    if let Some(fj) = fj {
        let rj = fj * STATE_DIM;
        add_block(h_pp, rj, rj, &(wjj.transpose() * wjj));
        sub_rows(b_p, rj, &(wjj.transpose() * wr));
    }
    if let (Some(fi), Some(fj)) = (fi, fj) {
        let (ri, rj) = (fi * STATE_DIM, fj * STATE_DIM);
        add_block(h_pp, ri, rj, &(wji.transpose() * wjj));
        add_block(h_pp, rj, ri, &(wjj.transpose() * wji));
    }
}

/// Runs Levenberg-Marquardt (with per-iteration Schur-complement landmark
/// elimination) on `problem` in place. `problem.keyframes[0]` is never
/// modified. Returns the final cost.
pub fn optimize(problem: &mut Problem, config: &SolverConfig) -> f64 {
    let mut lambda = config.initial_lambda;
    let mut current_cost = compute_cost(problem, config);

    for _ in 0..config.max_iterations {
        let ne = build_normal_equations(problem, config);
        let dim = ne.h_pp.nrows();
        if dim == 0 {
            break;
        }

        let mut damped = ne.h_pp.clone();
        for i in 0..dim {
            damped[(i, i)] += lambda * ne.h_pp[(i, i)].max(1e-12);
        }

        let Some(delta) = damped.lu().solve(&ne.b_p) else {
            lambda *= 4.0;
            continue;
        };

        let backup_keyframes = problem.keyframes.clone();
        let backup_landmarks = problem.landmarks.clone();

        for k in 1..problem.keyframes.len() {
            let d = SVector::<f64, STATE_DIM>::from_column_slice(delta.rows((k - 1) * STATE_DIM, STATE_DIM).as_slice());
            problem.keyframes[k] = problem.keyframes[k].retract(&d);
        }
        for (&landmark_idx, schur) in &ne.landmark_schur {
            let mut rhs = schur.b_l;
            for (&fk, hlp) in &schur.h_lp {
                let d = SVector::<f64, STATE_DIM>::from_column_slice(delta.rows(fk * STATE_DIM, STATE_DIM).as_slice());
                rhs -= hlp * d;
            }
            problem.landmarks[landmark_idx] += schur.h_ll_inv * rhs;
        }

        let trial_cost = compute_cost(problem, config);
        if trial_cost < current_cost {
            current_cost = trial_cost;
            lambda = (lambda / 3.0).max(1e-10);
        } else {
            problem.keyframes = backup_keyframes;
            problem.landmarks = backup_landmarks;
            lambda *= 4.0;
        }
    }

    current_cost
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slam_core::SO3;

    /// End-to-end toy problem: 4 keyframes with known ground-truth motion
    /// (constant angular + linear velocity, matching the same synthetic
    /// model used throughout `slam-imu`/`vi_init`'s tests), 20 landmarks
    /// observed from every keyframe, noise-free. Starts from a perturbed
    /// initial guess (everything except the anchor keyframe) and checks
    /// `optimize` converges back to ground truth — the first real test of
    /// the whole LM + Schur-complement pipeline together, not just
    /// individual factor Jacobians.
    #[test]
    fn converges_to_ground_truth_on_a_noise_free_toy_problem() {
        let w_true = Vector3::new(0.1, -0.05, 0.15);
        let v_true = Vector3::new(0.3, 0.1, -0.05);
        let g_true = Vector3::new(0.0, 0.0, -9.81);
        let dt_step = 1.0 / 200.0;
        let dt_keyframe = 0.2;

        let body_pose_at = |t: f64| SE3::new(SO3::exp(w_true * t), v_true * t);
        let true_state_at = |t: f64| KeyframeState::new(body_pose_at(t).inverse(), v_true, Vector3::zeros(), Vector3::zeros());

        let num_keyframes = 4;
        let true_states: Vec<KeyframeState> = (0..num_keyframes).map(|k| true_state_at(k as f64 * dt_keyframe)).collect();

        let mut imu_factors = Vec::new();
        for k in 0..num_keyframes - 1 {
            let mut pre = Preintegration::new(Vector3::zeros(), Vector3::zeros());
            let steps = (dt_keyframe / dt_step) as usize;
            for s in 0..steps {
                let t = k as f64 * dt_keyframe + s as f64 * dt_step;
                let r_wb = body_pose_at(t).rotation;
                let specific_force = r_wb.inverse().transform(&(-g_true));
                pre.integrate_measurement(w_true, specific_force, dt_step);
            }
            imu_factors.push(ImuFactorSpec { i: k, j: k + 1, preint: pre, dt: dt_keyframe });
        }
        let bias_rw_factors: Vec<BiasRwFactorSpec> = (0..num_keyframes - 1).map(|k| BiasRwFactorSpec { i: k, j: k + 1 }).collect();

        // 20 landmarks in front of the trajectory, observed from every keyframe.
        let true_landmarks: Vec<Vector3<f64>> = (0..20)
            .map(|i| {
                let a = i as f64;
                Vector3::new((a * 0.37).sin() * 1.5, (a * 0.53).cos() * 1.5, 3.0 + (a * 0.19).sin())
            })
            .collect();

        let mut reprojection_obs = Vec::new();
        for (k, state) in true_states.iter().enumerate() {
            for (l, landmark) in true_landmarks.iter().enumerate() {
                let p_body = state.pose.transform(landmark);
                if p_body.z <= 0.05 {
                    continue;
                }
                let obs = Vector2::new(p_body.x / p_body.z, p_body.y / p_body.z);
                reprojection_obs.push(ReprojectionObservation {
                    keyframe_idx: k,
                    landmark_idx: l,
                    t_bs_cam: SE3::identity(),
                    observed_normalized: obs,
                });
            }
        }
        assert!(reprojection_obs.len() > num_keyframes * 15, "expected ample observations");

        // Perturbed initial guess (keyframe 0 stays exact: it's the anchor).
        let mut keyframes = true_states.clone();
        for kf in keyframes.iter_mut().skip(1) {
            let perturb = SVector::<f64, STATE_DIM>::from_fn(|i, _| if i < 6 { 0.02 } else { 0.01 });
            *kf = kf.retract(&perturb);
        }
        let landmarks: Vec<Vector3<f64>> = true_landmarks.iter().map(|p| p + Vector3::new(0.03, -0.02, 0.04)).collect();

        let mut problem = Problem {
            keyframes,
            landmarks,
            reprojection_obs,
            imu_factors,
            bias_rw_factors,
            gravity_world: g_true,
        };

        let config = SolverConfig {
            max_iterations: 20,
            ..SolverConfig::default()
        };
        optimize(&mut problem, &config);

        for (estimated, expected) in problem.keyframes.iter().zip(true_states.iter()).skip(1) {
            assert_relative_eq!(estimated.pose.matrix(), expected.pose.matrix(), epsilon = 1e-3);
            assert_relative_eq!(estimated.velocity, expected.velocity, epsilon = 1e-2);
        }
        for (estimated, true_landmark) in problem.landmarks.iter().zip(true_landmarks.iter()) {
            assert_relative_eq!(estimated, true_landmark, epsilon = 1e-2);
        }
    }
}
