use nalgebra::{SMatrix, SVector};

use crate::state::{KeyframeState, STATE_DIM};

/// A prior that penalizes gyro/accel bias changing faster than the
/// sensor's random-walk noise density allows between two keyframes:
/// residual = `[bias_gyro_j - bias_gyro_i; bias_accel_j - bias_accel_i]`,
/// zero when biases don't drift at all (the un-weighted residual; the
/// solver applies the random-walk noise density as an information weight
/// when accumulating, not this function). Linear in the state, so the
/// Jacobian is exact (not an approximation) and trivial to write down
/// directly — no finite-difference risk here.
pub fn bias_random_walk_residual_jacobian(state_i: &KeyframeState, state_j: &KeyframeState) -> (SVector<f64, 6>, SMatrix<f64, 6, STATE_DIM>, SMatrix<f64, 6, STATE_DIM>) {
    let mut residual = SVector::<f64, 6>::zeros();
    residual.fixed_rows_mut::<3>(0).copy_from(&(state_j.bias_gyro - state_i.bias_gyro));
    residual.fixed_rows_mut::<3>(3).copy_from(&(state_j.bias_accel - state_i.bias_accel));

    let mut jac_i = SMatrix::<f64, 6, STATE_DIM>::zeros();
    let mut jac_j = SMatrix::<f64, 6, STATE_DIM>::zeros();
    // Columns 9..12 = gyro bias, 12..15 = accel bias (see state.rs's
    // documented STATE_DIM ordering).
    jac_i.fixed_view_mut::<3, 3>(0, 9).copy_from(&(-nalgebra::Matrix3::identity()));
    jac_i.fixed_view_mut::<3, 3>(3, 12).copy_from(&(-nalgebra::Matrix3::identity()));
    jac_j.fixed_view_mut::<3, 3>(0, 9).copy_from(&nalgebra::Matrix3::identity());
    jac_j.fixed_view_mut::<3, 3>(3, 12).copy_from(&nalgebra::Matrix3::identity());

    (residual, jac_i, jac_j)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Vector3;
    use slam_core::SE3;

    #[test]
    fn jacobians_match_finite_difference() {
        let state_i = KeyframeState::new(SE3::identity(), Vector3::zeros(), Vector3::new(0.01, -0.02, 0.005), Vector3::new(0.03, -0.01, 0.02));
        let state_j = KeyframeState::new(SE3::identity(), Vector3::zeros(), Vector3::new(0.02, -0.01, 0.01), Vector3::new(0.02, -0.02, 0.03));

        let (base, jac_i, jac_j) = bias_random_walk_residual_jacobian(&state_i, &state_j);

        let eps = 1e-6;
        for col in 0..STATE_DIM {
            let mut delta = SVector::<f64, STATE_DIM>::zeros();
            delta[col] = eps;
            let (r_i, _, _) = bias_random_walk_residual_jacobian(&state_i.retract(&delta), &state_j);
            let (r_j, _, _) = bias_random_walk_residual_jacobian(&state_i, &state_j.retract(&delta));
            assert_relative_eq!(jac_i.column(col).into_owned(), (r_i - base) / eps, epsilon = 1e-9);
            assert_relative_eq!(jac_j.column(col).into_owned(), (r_j - base) / eps, epsilon = 1e-9);
        }
    }
}
