use nalgebra::Vector3;

use crate::align::umeyama_alignment;

/// Relative Pose Error summary at a fixed frame/keyframe delta. This is a
/// translational simplification of the standard (TUM RGB-D benchmark)
/// RPE: it compares relative *translation* over `delta` steps, not the
/// full relative SE3 (which would need per-point orientations threaded
/// through every VO/VIO/loop-closure call site — `compute_ate`'s own
/// interface is position-only for the same reason). It still measures
/// what ATE alone can hide: local drift rate, rather than a single
/// worst-point divergence.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct RpeStats {
    pub delta: usize,
    pub rmse: f64,
    pub mean: f64,
    pub median: f64,
    pub std: f64,
    pub max: f64,
    pub num_pairs: usize,
}

/// Aligns `estimated` onto `groundtruth` via Umeyama (same alignment
/// `compute_ate` uses), then compares relative-translation segments
/// `[i, i+delta]` between the aligned estimate and groundtruth.
pub fn compute_rpe(estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>], delta: usize) -> Option<RpeStats> {
    if delta == 0 || estimated.len() != groundtruth.len() || estimated.len() <= delta {
        return None;
    }
    let alignment = umeyama_alignment(estimated, groundtruth)?;
    let aligned: Vec<Vector3<f64>> = estimated.iter().map(|p| alignment.apply(p)).collect();

    let mut errors: Vec<f64> = (0..aligned.len() - delta)
        .map(|i| {
            let est_rel = aligned[i + delta] - aligned[i];
            let gt_rel = groundtruth[i + delta] - groundtruth[i];
            (est_rel - gt_rel).norm()
        })
        .collect();
    errors.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let n = errors.len() as f64;
    let mean = errors.iter().sum::<f64>() / n;
    let rmse = (errors.iter().map(|e| e * e).sum::<f64>() / n).sqrt();
    let variance = errors.iter().map(|e| (e - mean).powi(2)).sum::<f64>() / n;
    let median = errors[errors.len() / 2];
    let max = *errors.last().unwrap();

    Some(RpeStats {
        delta,
        rmse,
        mean,
        median,
        std: variance.sqrt(),
        max,
        num_pairs: errors.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identical_trajectories_give_zero_rpe() {
        let points: Vec<Vector3<f64>> = (0..20).map(|i| Vector3::new(i as f64 * 0.5, (i as f64 * 0.3).sin(), 0.0)).collect();
        let stats = compute_rpe(&points, &points, 5).expect("RPE should compute");
        assert!(stats.rmse < 1e-9);
        assert_eq!(stats.num_pairs, 15);
        assert_eq!(stats.delta, 5);
    }

    #[test]
    fn a_global_similarity_transform_does_not_show_up_as_drift() {
        // RPE should be invariant to a pure Sim3 misalignment between
        // estimate and groundtruth, same as ATE after alignment.
        let source: Vec<Vector3<f64>> = (0..15).map(|i| Vector3::new(i as f64, (i as f64 * 0.4).cos(), 0.1 * i as f64)).collect();
        let target: Vec<Vector3<f64>> = source.iter().map(|p| 3.0 * p + Vector3::new(5.0, -2.0, 1.0)).collect();
        let stats = compute_rpe(&source, &target, 3).expect("RPE should compute");
        assert!(stats.rmse < 1e-6, "expected ~0 RPE under a pure similarity transform, got {}", stats.rmse);
    }

    #[test]
    fn per_step_drift_accumulates_into_growing_rpe_error() {
        // A trajectory that drifts by a constant offset per step relative
        // to groundtruth (not a global similarity transform, since it
        // grows with index) should show up as nonzero RPE.
        let n = 20;
        let groundtruth: Vec<Vector3<f64>> = (0..n).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        let estimated: Vec<Vector3<f64>> = (0..n).map(|i| Vector3::new(i as f64, 0.02 * (i as f64) * (i as f64), 0.0)).collect();
        let stats = compute_rpe(&estimated, &groundtruth, 1).expect("RPE should compute");
        assert!(stats.rmse > 0.01, "expected drift to show up in per-step RPE, got {}", stats.rmse);
    }

    #[test]
    fn mismatched_lengths_or_too_small_delta_window_returns_none() {
        let points: Vec<Vector3<f64>> = (0..5).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        assert!(compute_rpe(&points, &points, 0).is_none());
        assert!(compute_rpe(&points, &points, 5).is_none());
        assert!(compute_rpe(&points[..4], &points, 1).is_none());
    }
}
