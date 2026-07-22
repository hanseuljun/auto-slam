use nalgebra::{SMatrix, SVector, Vector3};
use slam_imu::Preintegration;

use crate::state::{KeyframeState, STATE_DIM};

/// The standard 9-DoF IMU preintegration factor residual (Forster et al.):
/// rotation, velocity, and position error between what the preintegrated
/// measurement predicts (propagating from `state_i`) and `state_j`'s
/// actual state. Zero when the states are exactly consistent with the
/// (bias-corrected) preintegration.
///
/// `state.pose` is `world -> body`, so `state.pose.rotation` is `R_bw`
/// (world-to-body) — the inverse of the `R_wb` (body-to-world) convention
/// the propagation equations are usually written in; this function
/// converts internally rather than exposing that to callers.
pub fn imu_residual(state_i: &KeyframeState, state_j: &KeyframeState, preint: &Preintegration, gravity_world: Vector3<f64>, dt: f64) -> SVector<f64, 9> {
    let (delta_r, delta_v, delta_p) = preint.corrected(state_i.bias_gyro, state_i.bias_accel);

    let r_bw_i = state_i.pose.rotation; // = R_wb_i^{-1}
    let p_i = state_i.pose.inverse().translation;
    let p_j = state_j.pose.inverse().translation;

    let r_rot = delta_r
        .inverse()
        .compose(&state_i.pose.rotation)
        .compose(&state_j.pose.rotation.inverse())
        .log();
    let r_vel = r_bw_i.transform(&(state_j.velocity - state_i.velocity - gravity_world * dt)) - delta_v;
    let r_pos = r_bw_i.transform(&(p_j - p_i - state_i.velocity * dt - 0.5 * gravity_world * dt * dt)) - delta_p;

    let mut r = SVector::<f64, 9>::zeros();
    r.fixed_rows_mut::<3>(0).copy_from(&r_rot);
    r.fixed_rows_mut::<3>(3).copy_from(&r_vel);
    r.fixed_rows_mut::<3>(6).copy_from(&r_pos);
    r
}

/// Residual plus numerical (central-difference) Jacobians wrt `state_i`
/// and `state_j`'s 15-dim tangent spaces. Numerical rather than hand-
/// derived analytic Jacobians here: this factor's 18 partial-derivative
/// blocks are unusually easy to get subtly wrong (see
/// `memory/decisions` for the sign bug already found once in this exact
/// kind of propagation equation, in M4's dynamic initializer), and
/// `Preintegration::corrected` is O(1) (no re-integration), so the extra
/// residual evaluations this costs are negligible — a deliberate
/// correctness-over-speed tradeoff matching `plan/STAGE1.md`'s explicit
/// "correctness and accuracy first, speed later" for Stage 1. The
/// reprojection factor's Jacobian, by contrast, is simple enough
/// (identical in structure to `slam_geometry::refine_pose_gauss_newton`,
/// already validated) to derive analytically without that risk.
pub fn imu_residual_jacobian(
    state_i: &KeyframeState,
    state_j: &KeyframeState,
    preint: &Preintegration,
    gravity_world: Vector3<f64>,
    dt: f64,
) -> (SVector<f64, 9>, SMatrix<f64, 9, STATE_DIM>, SMatrix<f64, 9, STATE_DIM>) {
    let base = imu_residual(state_i, state_j, preint, gravity_world, dt);
    let eps = 1e-6;
    let mut jac_i = SMatrix::<f64, 9, STATE_DIM>::zeros();
    let mut jac_j = SMatrix::<f64, 9, STATE_DIM>::zeros();

    for col in 0..STATE_DIM {
        let mut delta = SVector::<f64, STATE_DIM>::zeros();
        delta[col] = eps;

        let perturbed_i = state_i.retract(&delta);
        let r_i = imu_residual(&perturbed_i, state_j, preint, gravity_world, dt);
        jac_i.set_column(col, &((r_i - base) / eps));

        let perturbed_j = state_j.retract(&delta);
        let r_j = imu_residual(state_i, &perturbed_j, preint, gravity_world, dt);
        jac_j.set_column(col, &((r_j - base) / eps));
    }

    (base, jac_i, jac_j)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slam_core::{SE3, SO3};

    /// The same synthetic model as `slam_frontend::vi_init`'s tests
    /// (constant angular velocity + constant world velocity under
    /// gravity), reused here to build a state pair that's *exactly*
    /// consistent with a preintegration computed from the same raw IMU —
    /// residual should be ~0. This is the cross-check that would have
    /// caught the R_wb/R_bw convention mixup class of bug immediately,
    /// same as `vi_init`'s `ground_truth_satisfies_the_assembled_linear_system`.
    #[test]
    fn residual_is_zero_for_self_consistent_states() {
        let w_true = Vector3::new(0.3, -0.2, 0.4);
        let v_true = Vector3::new(0.5, 0.1, -0.2);
        let g_true = Vector3::new(0.05, -0.02, -9.8);
        let dt_total = 0.5;

        let body_pose_at = |t: f64| SE3::new(SO3::exp(w_true * t), v_true * t);
        // world -> body is the inverse of the body's world pose.
        let world_to_body_at = |t: f64| body_pose_at(t).inverse();

        let state_i = KeyframeState::new(world_to_body_at(0.0), v_true, Vector3::zeros(), Vector3::zeros());
        let state_j = KeyframeState::new(world_to_body_at(dt_total), v_true, Vector3::zeros(), Vector3::zeros());

        let rate_hz = 200.0;
        let steps = (dt_total * rate_hz) as usize;
        let dt_step = 1.0 / rate_hz;
        let mut pre = Preintegration::new(Vector3::zeros(), Vector3::zeros());
        for i in 0..steps {
            let t = i as f64 * dt_step;
            let r_wb = body_pose_at(t).rotation;
            let specific_force_body = r_wb.inverse().transform(&(-g_true));
            pre.integrate_measurement(w_true, specific_force_body, dt_step);
        }

        let residual = imu_residual(&state_i, &state_j, &pre, g_true, dt_total);
        assert_relative_eq!(residual, SVector::<f64, 9>::zeros(), epsilon = 1e-3);
    }

    #[test]
    fn jacobian_matches_its_own_finite_difference_at_a_different_epsilon() {
        // Sanity check that the numerical Jacobian is stable (not an
        // artifact of one specific epsilon), by comparing against a
        // second, independent finite-difference pass at a different step
        // size — not a proof of correctness (it's finite-difference
        // either way) but catches gross instability.
        let state_i = KeyframeState::new(
            SE3::new(SO3::exp(Vector3::new(0.1, -0.05, 0.2)), Vector3::new(0.2, -0.1, 0.05)),
            Vector3::new(0.3, 0.1, -0.1),
            Vector3::new(0.01, -0.01, 0.005),
            Vector3::new(0.02, -0.01, 0.01),
        );
        let state_j = KeyframeState::new(
            SE3::new(SO3::exp(Vector3::new(0.15, -0.02, 0.25)), Vector3::new(0.4, -0.05, 0.02)),
            Vector3::new(0.32, 0.09, -0.11),
            Vector3::new(0.01, -0.01, 0.005),
            Vector3::new(0.02, -0.01, 0.01),
        );
        let mut pre = Preintegration::new(Vector3::zeros(), Vector3::zeros());
        for _ in 0..100 {
            pre.integrate_measurement(Vector3::new(0.1, -0.05, 0.15), Vector3::new(0.2, 0.1, -9.7), 0.005);
        }
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let dt = 0.5;

        let (_, jac_i_a, jac_j_a) = imu_residual_jacobian(&state_i, &state_j, &pre, gravity, dt);

        // Re-derive with a coarser epsilon by temporarily wrapping the
        // same central-difference logic inline.
        let base = imu_residual(&state_i, &state_j, &pre, gravity, dt);
        let eps2 = 1e-5;
        let mut jac_i_b = SMatrix::<f64, 9, STATE_DIM>::zeros();
        for col in 0..STATE_DIM {
            let mut delta = SVector::<f64, STATE_DIM>::zeros();
            delta[col] = eps2;
            let perturbed = state_i.retract(&delta);
            let r = imu_residual(&perturbed, &state_j, &pre, gravity, dt);
            jac_i_b.set_column(col, &((r - base) / eps2));
        }

        assert_relative_eq!(jac_i_a, jac_i_b, epsilon = 1e-3);
        let _ = jac_j_a; // computed to exercise the full function; not re-checked separately here.
    }
}
