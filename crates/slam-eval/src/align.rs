use nalgebra::{Matrix3, Vector3};

/// A similarity transform `p -> scale * (rotation * p) + translation`.
#[derive(Debug, Clone, Copy)]
pub struct Sim3Alignment {
    pub scale: f64,
    pub rotation: Matrix3<f64>,
    pub translation: Vector3<f64>,
}

impl Sim3Alignment {
    pub fn apply(&self, p: &Vector3<f64>) -> Vector3<f64> {
        self.scale * (self.rotation * p) + self.translation
    }
}

/// Umeyama's method: the least-squares similarity transform mapping `source`
/// points onto `target` points. Used to align an estimated SLAM trajectory
/// (arbitrary world frame, and for VO-only, arbitrary scale) onto ground
/// truth before computing ATE — comparing raw coordinates would be
/// meaningless (see `memory/notes/dataset-quirks.md`).
pub fn umeyama_alignment(source: &[Vector3<f64>], target: &[Vector3<f64>]) -> Option<Sim3Alignment> {
    let n = source.len();
    if n < 3 || target.len() != n {
        return None;
    }
    let n_f = n as f64;

    let mu_source: Vector3<f64> = source.iter().sum::<Vector3<f64>>() / n_f;
    let mu_target: Vector3<f64> = target.iter().sum::<Vector3<f64>>() / n_f;

    let mut sigma = Matrix3::<f64>::zeros();
    let mut source_variance = 0.0;
    for i in 0..n {
        let sc = source[i] - mu_source;
        let tc = target[i] - mu_target;
        sigma += tc * sc.transpose();
        source_variance += sc.norm_squared();
    }
    sigma /= n_f;
    source_variance /= n_f;
    if source_variance < 1e-12 {
        return None;
    }

    let svd = sigma.svd(true, true);
    let u = svd.u?;
    let v_t = svd.v_t?;
    let d = svd.singular_values;

    let mut s = Matrix3::identity();
    if (u.determinant() * v_t.determinant()) < 0.0 {
        s[(2, 2)] = -1.0;
    }

    let rotation = u * s * v_t;
    let scale = (d[0] * s[(0, 0)] + d[1] * s[(1, 1)] + d[2] * s[(2, 2)]) / source_variance;
    let translation = mu_target - scale * (rotation * mu_source);

    Some(Sim3Alignment {
        scale,
        rotation,
        translation,
    })
}

/// Absolute Trajectory Error summary (RMSE/mean/median/std of per-point
/// Euclidean error), after alignment.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct AteStats {
    pub rmse: f64,
    pub mean: f64,
    pub median: f64,
    pub std: f64,
    pub max: f64,
    pub num_points: usize,
}

/// Aligns `estimated` onto `groundtruth` via Umeyama and reports ATE stats.
pub fn compute_ate(estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>]) -> Option<AteStats> {
    let alignment = umeyama_alignment(estimated, groundtruth)?;
    let mut errors: Vec<f64> = estimated
        .iter()
        .zip(groundtruth.iter())
        .map(|(e, g)| (alignment.apply(e) - g).norm())
        .collect();
    errors.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let n = errors.len() as f64;
    let mean = errors.iter().sum::<f64>() / n;
    let rmse = (errors.iter().map(|e| e * e).sum::<f64>() / n).sqrt();
    let variance = errors.iter().map(|e| (e - mean).powi(2)).sum::<f64>() / n;
    let median = errors[errors.len() / 2];
    let max = *errors.last().unwrap();

    Some(AteStats {
        rmse,
        mean,
        median,
        std: variance.sqrt(),
        max,
        num_points: errors.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slam_core::SO3;

    #[test]
    fn recovers_a_known_similarity_transform() {
        let true_scale = 2.5;
        let true_rotation = SO3::exp(Vector3::new(0.2, -0.1, 0.4)).matrix();
        let true_translation = Vector3::new(1.0, -2.0, 0.5);

        let source: Vec<Vector3<f64>> = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(1.0, 2.0, -1.0),
            Vector3::new(-2.0, 1.0, 3.0),
        ];
        let target: Vec<Vector3<f64>> = source
            .iter()
            .map(|p| true_scale * (true_rotation * p) + true_translation)
            .collect();

        let alignment = umeyama_alignment(&source, &target).expect("alignment should succeed");
        assert_relative_eq!(alignment.scale, true_scale, epsilon = 1e-8);
        assert_relative_eq!(alignment.rotation, true_rotation, epsilon = 1e-8);
        assert_relative_eq!(alignment.translation, true_translation, epsilon = 1e-8);
    }

    #[test]
    fn identical_point_sets_give_zero_ate() {
        let points: Vec<Vector3<f64>> = vec![
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(-1.0, 0.5, 2.0),
            Vector3::new(4.0, -3.0, 1.0),
            Vector3::new(0.0, 0.0, 0.0),
        ];
        let stats = compute_ate(&points, &points).expect("ATE should succeed");
        assert!(stats.rmse < 1e-9);
        assert!(stats.max < 1e-9);
    }

    #[test]
    fn ate_reports_known_offset() {
        let source: Vec<Vector3<f64>> = (0..10)
            .map(|i| Vector3::new(i as f64, 0.0, 0.0))
            .collect();
        // A pure per-point offset noise (not a global similarity transform)
        // should survive alignment and show up in ATE.
        let target: Vec<Vector3<f64>> = source
            .iter()
            .enumerate()
            .map(|(i, p)| p + Vector3::new(0.0, if i % 2 == 0 { 0.1 } else { -0.1 }, 0.0))
            .collect();
        let stats = compute_ate(&source, &target).expect("ATE should succeed");
        assert!(stats.rmse > 0.05 && stats.rmse < 0.15);
    }
}
