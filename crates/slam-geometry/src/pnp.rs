use nalgebra::{DMatrix, Matrix2x3, Matrix3, Matrix6, SMatrix, Vector2, Vector3, Vector6};
use slam_core::{SE3, SO3};

/// Direct Linear Transform camera resectioning: recovers the pose mapping
/// world-frame points into the camera frame (`p_cam = T.transform(p_world)`)
/// from at least 6 3D-2D correspondences in normalized (calibration-free)
/// image coordinates. This is the linear initializer that
/// `refine_pose_gauss_newton` then polishes; a minimal-point solver (P3P,
/// for RANSAC-based robust initialization) is deferred to M3/M4 where a
/// RANSAC loop actually consumes it — see `memory/decisions`.
pub fn estimate_pose_dlt(points_world: &[Vector3<f64>], observations: &[Vector2<f64>]) -> Option<SE3> {
    let n = points_world.len();
    if n < 6 || observations.len() != n {
        return None;
    }
    let mut a = DMatrix::<f64>::zeros(2 * n, 12);
    for i in 0..n {
        let x = points_world[i];
        let (u, v) = (observations[i].x, observations[i].y);
        let row_u = [-x.x, -x.y, -x.z, -1.0, 0.0, 0.0, 0.0, 0.0, u * x.x, u * x.y, u * x.z, u];
        let row_v = [0.0, 0.0, 0.0, 0.0, -x.x, -x.y, -x.z, -1.0, v * x.x, v * x.y, v * x.z, v];
        for j in 0..12 {
            a[(2 * i, j)] = row_u[j];
            a[(2 * i + 1, j)] = row_v[j];
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
    let p = v_t.row(min_idx).clone_owned();

    // Columns 0..4 are P's row 1, 4..8 are row 2, 8..12 are row 3 (see the
    // row_u/row_v layout above).
    let mut m = Matrix3::new(
        p[(0, 0)], p[(0, 1)], p[(0, 2)],
        p[(0, 4)], p[(0, 5)], p[(0, 6)],
        p[(0, 8)], p[(0, 9)], p[(0, 10)],
    );
    let mut t_raw = Vector3::new(p[(0, 3)], p[(0, 7)], p[(0, 11)]);

    // The DLT nullspace vector's sign is arbitrary; fixing det(M) > 0 here
    // simultaneously (a) selects the sign that puts points in front of the
    // camera and (b) guarantees the SVD-based nearest rotation below has
    // det = +1, since det(M) = det(U)*det(V)*prod(singular_values) with all
    // singular values >= 0.
    if m.determinant() < 0.0 {
        m = -m;
        t_raw = -t_raw;
    }

    let svd_m = m.svd(true, true);
    let u = svd_m.u?;
    let v_t_m = svd_m.v_t?;
    let scale = svd_m.singular_values.mean();
    if scale < 1e-12 {
        return None;
    }
    let rotation = u * v_t_m;
    let translation = t_raw / scale;

    Some(SE3::new(SO3::from_matrix(&rotation), translation))
}

/// Polishes a pose estimate by minimizing reprojection error over the SE(3)
/// manifold (Gauss-Newton with a left-multiplicative update, using
/// `slam_core::SO3::hat` for the pose Jacobian). Unweighted, no robust
/// kernel or damping — the full LM/Schur-complement solver lives in
/// `slam-optim` (M5); this is enough to validate the camera model and DLT
/// initializer now.
pub fn refine_pose_gauss_newton(
    points_world: &[Vector3<f64>],
    observations: &[Vector2<f64>],
    initial: SE3,
    iterations: usize,
) -> SE3 {
    let mut t = initial;
    for _ in 0..iterations {
        let mut jtj = Matrix6::zeros();
        let mut jtr = Vector6::zeros();
        for (x, obs) in points_world.iter().zip(observations) {
            let p = t.transform(x);
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
            // d(p_cam)/d(delta) for delta = [rho; phi], left-multiplicative:
            // exp(delta)*T transforms X to ~= p + rho + phi x p.
            let mut jac_pose = SMatrix::<f64, 3, 6>::zeros();
            jac_pose.fixed_view_mut::<3, 3>(0, 0).copy_from(&Matrix3::identity());
            jac_pose.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-SO3::hat(&p)));

            let j = jac_proj * jac_pose;
            jtj += j.transpose() * j;
            jtr += j.transpose() * residual;
        }
        match jtj.try_inverse() {
            Some(inv) => {
                let delta: Vector6<f64> = -(inv * jtr);
                t = SE3::exp(delta) * t;
            }
            None => break,
        }
    }
    t
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// A non-coplanar point cloud: camera resectioning via DLT is degenerate
    /// for coplanar points (the 3x4 projective camera has an extra
    /// ambiguity), so `z` must not be an affine function of `x, y`.
    fn scene_points() -> Vec<Vector3<f64>> {
        let mut points = Vec::new();
        for x in [-1.5f64, -0.5, 0.5, 1.5] {
            for y in [-1.0f64, 0.0, 1.0] {
                let z = 5.0 + 0.3 * x - 0.2 * y + 0.4 * (x * y).sin() + 0.6 * x * x * y;
                points.push(Vector3::new(x, y, z));
            }
        }
        points
    }

    fn project_all(t: &SE3, points: &[Vector3<f64>]) -> Vec<Vector2<f64>> {
        points
            .iter()
            .map(|x| {
                let p = t.transform(x);
                Vector2::new(p.x / p.z, p.y / p.z)
            })
            .collect()
    }

    #[test]
    fn dlt_plus_refine_recovers_pose_to_high_precision() {
        let true_pose = SE3::new(
            SO3::exp(Vector3::new(0.15, -0.08, 0.2)),
            Vector3::new(0.3, -0.4, 1.2),
        );
        let points = scene_points();
        let observations = project_all(&true_pose, &points);

        let dlt = estimate_pose_dlt(&points, &observations).expect("DLT should succeed");
        // DLT alone should already be close in this noise-free, well-conditioned case.
        assert_relative_eq!(dlt.rotation.matrix(), true_pose.rotation.matrix(), epsilon = 1e-4);
        assert_relative_eq!(dlt.translation, true_pose.translation, epsilon = 1e-4);

        let refined = refine_pose_gauss_newton(&points, &observations, dlt, 10);
        assert_relative_eq!(refined.rotation.matrix(), true_pose.rotation.matrix(), epsilon = 1e-9);
        assert_relative_eq!(refined.translation, true_pose.translation, epsilon = 1e-9);
    }

    #[test]
    fn refine_converges_from_a_perturbed_initial_guess() {
        let true_pose = SE3::new(
            SO3::exp(Vector3::new(-0.1, 0.2, 0.05)),
            Vector3::new(-0.2, 0.1, 2.0),
        );
        let points = scene_points();
        let observations = project_all(&true_pose, &points);

        let perturbed = SE3::exp(Vector6::new(0.05, -0.03, 0.02, 0.02, -0.01, 0.03)) * true_pose;
        let refined = refine_pose_gauss_newton(&points, &observations, perturbed, 15);

        assert_relative_eq!(refined.rotation.matrix(), true_pose.rotation.matrix(), epsilon = 1e-8);
        assert_relative_eq!(refined.translation, true_pose.translation, epsilon = 1e-8);
    }

    #[test]
    fn dlt_returns_none_with_too_few_points() {
        let points = vec![Vector3::new(0.0, 0.0, 1.0); 5];
        let obs = vec![Vector2::new(0.0, 0.0); 5];
        assert!(estimate_pose_dlt(&points, &obs).is_none());
    }
}
