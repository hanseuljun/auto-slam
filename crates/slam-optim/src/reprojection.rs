use nalgebra::{Matrix2x3, SMatrix, Vector2, Vector3};
use slam_core::{SE3, SO3};

use crate::state::{KeyframeState, STATE_DIM};

/// A single camera observation of a landmark from one keyframe: reprojects
/// the landmark through the keyframe's pose and a fixed body-to-camera
/// extrinsic, residual = predicted - observed normalized coordinates
/// (matches `slam_geometry`'s convention throughout this codebase).
///
/// `state.pose` is `world -> body` (`p_body = pose.transform(p_world)`,
/// the IMU/body convention `KeyframeState` uses); `t_bs_cam` is the usual
/// EuRoC `cam -> body` extrinsic (`X_body = t_bs_cam.transform(X_cam)`).
pub fn reprojection_residual_jacobian(
    state: &KeyframeState,
    t_bs_cam: &SE3,
    landmark: Vector3<f64>,
    observed_normalized: Vector2<f64>,
) -> Option<(Vector2<f64>, SMatrix<f64, 2, STATE_DIM>, nalgebra::Matrix2x3<f64>)> {
    let t_cam_bs = t_bs_cam.inverse(); // body -> cam
    let p_body = state.pose.transform(&landmark);
    let p_cam = t_cam_bs.transform(&p_body);
    if p_cam.z <= 1e-6 {
        return None;
    }

    let inv_z = 1.0 / p_cam.z;
    let predicted = Vector2::new(p_cam.x * inv_z, p_cam.y * inv_z);
    let residual = predicted - observed_normalized;

    let jac_proj: Matrix2x3<f64> = Matrix2x3::new(
        inv_z, 0.0, -p_cam.x * inv_z * inv_z,
        0.0, inv_z, -p_cam.y * inv_z * inv_z,
    );

    let r_cam_body = t_cam_bs.rotation.matrix();
    let d_pcam_d_deltapose = r_cam_body * {
        let mut m = nalgebra::Matrix3x6::<f64>::zeros();
        m.fixed_view_mut::<3, 3>(0, 0).copy_from(&nalgebra::Matrix3::identity());
        m.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-SO3::hat(&p_body)));
        m
    };
    let jac_pose_6 = jac_proj * d_pcam_d_deltapose;

    let mut jac_state = SMatrix::<f64, 2, STATE_DIM>::zeros();
    jac_state.fixed_view_mut::<2, 6>(0, 0).copy_from(&jac_pose_6);

    let jac_landmark = jac_proj * r_cam_body * state.pose.rotation.matrix();

    Some((residual, jac_state, jac_landmark))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn finite_diff_pose_jacobian(state: &KeyframeState, t_bs_cam: &SE3, landmark: Vector3<f64>, obs: Vector2<f64>) -> SMatrix<f64, 2, STATE_DIM> {
        let eps = 1e-6;
        let mut jac = SMatrix::<f64, 2, STATE_DIM>::zeros();
        let (base_residual, _, _) = reprojection_residual_jacobian(state, t_bs_cam, landmark, obs).unwrap();
        for col in 0..6 {
            let mut delta = nalgebra::SVector::<f64, STATE_DIM>::zeros();
            delta[col] = eps;
            let perturbed_state = state.retract(&delta);
            let (perturbed_residual, _, _) = reprojection_residual_jacobian(&perturbed_state, t_bs_cam, landmark, obs).unwrap();
            jac.set_column(col, &((perturbed_residual - base_residual) / eps));
        }
        jac
    }

    fn finite_diff_landmark_jacobian(state: &KeyframeState, t_bs_cam: &SE3, landmark: Vector3<f64>, obs: Vector2<f64>) -> Matrix2x3<f64> {
        let eps = 1e-6;
        let mut jac = Matrix2x3::zeros();
        let (base_residual, _, _) = reprojection_residual_jacobian(state, t_bs_cam, landmark, obs).unwrap();
        for col in 0..3 {
            let mut d = Vector3::zeros();
            d[col] = eps;
            let (perturbed_residual, _, _) = reprojection_residual_jacobian(state, t_bs_cam, landmark + d, obs).unwrap();
            jac.set_column(col, &((perturbed_residual - base_residual) / eps));
        }
        jac
    }

    #[test]
    fn zero_residual_when_landmark_projects_exactly() {
        let state = KeyframeState::new(SE3::identity(), Vector3::zeros(), Vector3::zeros(), Vector3::zeros());
        let t_bs_cam = SE3::identity();
        let landmark = Vector3::new(0.2, -0.1, 3.0);
        let obs = Vector2::new(landmark.x / landmark.z, landmark.y / landmark.z);
        let (residual, _, _) = reprojection_residual_jacobian(&state, &t_bs_cam, landmark, obs).unwrap();
        assert_relative_eq!(residual, Vector2::zeros(), epsilon = 1e-12);
    }

    #[test]
    fn jacobians_match_finite_difference() {
        let state = KeyframeState::new(
            SE3::new(SO3::exp(Vector3::new(0.1, -0.2, 0.05)), Vector3::new(0.5, -0.3, 0.2)),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let t_bs_cam = SE3::new(SO3::exp(Vector3::new(0.0, 1.4, 0.0)), Vector3::new(0.02, -0.06, 0.01));
        let landmark = Vector3::new(1.0, 0.5, 4.0);
        let obs = Vector2::new(0.1, -0.05); // arbitrary, doesn't affect Jacobian

        let (_, jac_state, jac_landmark) = reprojection_residual_jacobian(&state, &t_bs_cam, landmark, obs).unwrap();
        let numeric_pose = finite_diff_pose_jacobian(&state, &t_bs_cam, landmark, obs);
        let numeric_landmark = finite_diff_landmark_jacobian(&state, &t_bs_cam, landmark, obs);

        assert_relative_eq!(jac_state, numeric_pose, epsilon = 1e-4);
        assert_relative_eq!(jac_landmark, numeric_landmark, epsilon = 1e-4);
    }

    #[test]
    fn behind_camera_returns_none() {
        let state = KeyframeState::new(SE3::identity(), Vector3::zeros(), Vector3::zeros(), Vector3::zeros());
        let t_bs_cam = SE3::identity();
        let landmark = Vector3::new(0.1, 0.1, -1.0);
        assert!(reprojection_residual_jacobian(&state, &t_bs_cam, landmark, Vector2::zeros()).is_none());
    }
}
