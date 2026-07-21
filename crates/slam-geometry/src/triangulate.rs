use nalgebra::{DMatrix, Matrix2x3, Vector2, Vector3};
use slam_core::SE3;

/// Linear (DLT) triangulation from N calibrated (normalized-coordinate)
/// observations of the same 3D point. `observations[i] = (T_i, x_i)` where
/// `T_i` maps a world-frame point into view `i`'s camera frame
/// (`p_cam = T_i.transform(p_world)`) and `x_i` is the observed normalized
/// image coordinate. Requires at least 2 observations.
pub fn triangulate_linear(observations: &[(SE3, Vector2<f64>)]) -> Option<Vector3<f64>> {
    if observations.len() < 2 {
        return None;
    }
    let n = observations.len();
    let mut a = DMatrix::<f64>::zeros(2 * n, 4);
    for (i, (t, x)) in observations.iter().enumerate() {
        let r = t.rotation.matrix();
        let tr = t.translation;
        let p_row = |row: usize| -> [f64; 4] { [r[(row, 0)], r[(row, 1)], r[(row, 2)], tr[row]] };
        let p1 = p_row(0);
        let p2 = p_row(1);
        let p3 = p_row(2);
        for j in 0..4 {
            a[(2 * i, j)] = x.x * p3[j] - p1[j];
            a[(2 * i + 1, j)] = x.y * p3[j] - p2[j];
        }
    }

    let svd = a.svd(false, true);
    let v_t = svd.v_t?;
    let min_idx = svd
        .singular_values
        .iter()
        .enumerate()
        .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .map(|(i, _)| i)?;
    let row = v_t.row(min_idx);
    if row[3].abs() < 1e-12 {
        return None;
    }
    Some(Vector3::new(row[0] / row[3], row[1] / row[3], row[2] / row[3]))
}

/// Refines a triangulated point by minimizing reprojection error (Gauss-
/// Newton, unweighted, no robust kernel — the full robust/weighted version
/// lives in the backend once `slam-optim` exists in M5).
pub fn triangulate_refine(
    observations: &[(SE3, Vector2<f64>)],
    initial: Vector3<f64>,
    iterations: usize,
) -> Vector3<f64> {
    let mut x = initial;
    for _ in 0..iterations {
        let mut jtj = nalgebra::Matrix3::zeros();
        let mut jtr = Vector3::zeros();
        for (t, obs) in observations {
            let p = t.transform(&x);
            if p.z <= 1e-9 {
                continue;
            }
            let inv_z = 1.0 / p.z;
            let predicted = Vector2::new(p.x * inv_z, p.y * inv_z);
            let residual = predicted - obs;

            let jac_proj = Matrix2x3::new(
                inv_z, 0.0, -p.x * inv_z * inv_z,
                0.0, inv_z, -p.y * inv_z * inv_z,
            );
            let j = jac_proj * t.rotation.matrix();

            jtj += j.transpose() * j;
            jtr += j.transpose() * residual;
        }
        if let Some(inv) = jtj.try_inverse() {
            x -= inv * jtr;
        } else {
            break;
        }
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slam_core::SO3;

    fn mh01_stereo_relative_pose() -> SE3 {
        // Same as StereoRig::relative_pose_cam1_from_cam0 for MH_01's rig,
        // precomputed here to keep this module's tests independent of
        // `rectify`'s.
        let t_bs_cam0 = SE3::from_matrix(&nalgebra::Matrix4::from_row_slice(&[
            0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            0.0, 0.0, 0.0, 1.0,
        ]));
        let t_bs_cam1 = SE3::from_matrix(&nalgebra::Matrix4::from_row_slice(&[
            0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
            0.0, 0.0, 0.0, 1.0,
        ]));
        t_bs_cam1.inverse().compose(&t_bs_cam0)
    }

    #[test]
    fn triangulates_stereo_point_to_submillimeter_accuracy() {
        let t10 = mh01_stereo_relative_pose();
        let true_point_cam0 = Vector3::new(0.3, -0.2, 4.0);
        let true_point_cam1 = t10.transform(&true_point_cam0);

        let x0 = Vector2::new(true_point_cam0.x / true_point_cam0.z, true_point_cam0.y / true_point_cam0.z);
        let x1 = Vector2::new(true_point_cam1.x / true_point_cam1.z, true_point_cam1.y / true_point_cam1.z);

        let observations = [(SE3::identity(), x0), (t10, x1)];
        let linear = triangulate_linear(&observations).expect("triangulation should succeed");
        assert_relative_eq!(linear, true_point_cam0, epsilon = 1e-9);

        let refined = triangulate_refine(&observations, linear, 5);
        assert_relative_eq!(refined, true_point_cam0, epsilon = 1e-9);
    }

    #[test]
    fn refine_pulls_noisy_linear_estimate_back_toward_ground_truth() {
        let t10 = mh01_stereo_relative_pose();
        let true_point_cam0 = Vector3::new(-0.1, 0.4, 3.0);
        let true_point_cam1 = t10.transform(&true_point_cam0);
        let x0 = Vector2::new(true_point_cam0.x / true_point_cam0.z, true_point_cam0.y / true_point_cam0.z);
        let x1 = Vector2::new(true_point_cam1.x / true_point_cam1.z, true_point_cam1.y / true_point_cam1.z);
        let observations = [(SE3::identity(), x0), (t10, x1)];

        let noisy_initial = true_point_cam0 + Vector3::new(0.05, -0.03, 0.1);
        let refined = triangulate_refine(&observations, noisy_initial, 20);
        let error_before = (noisy_initial - true_point_cam0).norm();
        let error_after = (refined - true_point_cam0).norm();
        assert!(error_after < error_before * 1e-3);
    }

    #[test]
    fn three_view_triangulation_with_general_poses() {
        let true_point = Vector3::new(1.5, -0.5, 6.0);
        let poses = [
            SE3::identity(),
            SE3::new(SO3::exp(Vector3::new(0.02, -0.01, 0.05)), Vector3::new(0.3, 0.0, -0.05)),
            SE3::new(SO3::exp(Vector3::new(-0.03, 0.04, -0.02)), Vector3::new(-0.2, 0.15, 0.1)),
        ];
        let observations: Vec<(SE3, Vector2<f64>)> = poses
            .iter()
            .map(|t| {
                let p = t.transform(&true_point);
                (*t, Vector2::new(p.x / p.z, p.y / p.z))
            })
            .collect();

        let linear = triangulate_linear(&observations).expect("triangulation should succeed");
        assert_relative_eq!(linear, true_point, epsilon = 1e-8);
    }
}
