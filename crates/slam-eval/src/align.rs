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

/// Per-point Euclidean error after Umeyama-aligning `estimated` onto
/// `groundtruth` — the same alignment `compute_ate` reports summary
/// stats over, but returning the full in-order series instead, for
/// time-series display (`bin/slam-viz`'s graphs panel, `plan/STAGE3.md`
/// M5) where "error over the run," not just one aggregate number,
/// matters.
pub fn compute_ate_series(estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>]) -> Option<Vec<f64>> {
    let alignment = umeyama_alignment(estimated, groundtruth)?;
    Some(estimated.iter().zip(groundtruth.iter()).map(|(e, g)| (alignment.apply(e) - g).norm()).collect())
}

/// Per-point Euclidean error, same as `compute_ate_series`, but the
/// Umeyama transform is fit using only the first `align_prefix_len`
/// points (clamped to the trajectory's own length) instead of the whole
/// trajectory — then applied to every point. `compute_ate_series`/
/// `compute_ate` fit a single least-squares compromise over the *entire*
/// trajectory at once, which lets later drift pull the fit away from an
/// early portion that was actually still accurate — confirmed on real
/// data, not hypothetical (`memory/decisions/0020`): a full-trajectory
/// fit reported 3.1m of "error" on `MH_01_easy`'s first ~20 keyframes
/// even though those frames' own raw pose estimate was nearly identical
/// to a shorter run's, whose full-trajectory fit reported 0.18m for the
/// same frames. Fitting only against an early, still-trustworthy prefix
/// avoids that: error near the start reflects real early-trajectory
/// accuracy, and growth over the rest of the series reflects real,
/// uncorrected drift instead of being partly absorbed by the alignment
/// itself. `align_prefix_len` is the caller's call (`plan/STAGE5.md` M0's
/// own finding: too small a prefix is numerically unstable — a poorly-
/// conditioned small window's rotation uncertainty has a lever-arm
/// effect, tens-to-hundreds of meters of apparent error far from the
/// anchor from a few centimeters of rotation misfit) — `bin/slam-run`
/// uses a bounded, multi-second prefix, not a handful of points.
pub fn compute_ate_series_prefix_aligned(estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>], align_prefix_len: usize) -> Option<Vec<f64>> {
    if estimated.len() != groundtruth.len() {
        return None;
    }
    let prefix = align_prefix_len.min(estimated.len());
    let alignment = umeyama_alignment(&estimated[..prefix], &groundtruth[..prefix])?;
    Some(estimated.iter().zip(groundtruth.iter()).map(|(e, g)| (alignment.apply(e) - g).norm()).collect())
}

fn ate_stats_from_errors(mut errors: Vec<f64>) -> AteStats {
    errors.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let n = errors.len() as f64;
    let mean = errors.iter().sum::<f64>() / n;
    let rmse = (errors.iter().map(|e| e * e).sum::<f64>() / n).sqrt();
    let variance = errors.iter().map(|e| (e - mean).powi(2)).sum::<f64>() / n;
    let median = errors[errors.len() / 2];
    let max = *errors.last().unwrap();

    AteStats {
        rmse,
        mean,
        median,
        std: variance.sqrt(),
        max,
        num_points: errors.len(),
    }
}

/// Aligns `estimated` onto `groundtruth` via Umeyama (fit over the whole
/// trajectory) and reports ATE stats. Kept for continuity with `docs/
/// RESULTS.md`'s existing published-SOTA comparison table (those systems'
/// own numbers are conventionally reported this way too) — see
/// `compute_ate_prefix_aligned` for the metric that doesn't let later
/// drift mask early accuracy (`plan/STAGE5.md` goal 1).
pub fn compute_ate(estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>]) -> Option<AteStats> {
    let errors = compute_ate_series(estimated, groundtruth)?;
    Some(ate_stats_from_errors(errors))
}

/// Aligns `estimated` onto `groundtruth` using only the first
/// `align_prefix_len` points to fit the transform, and reports ATE stats
/// over the whole trajectory — see `compute_ate_series_prefix_aligned`
/// for the full rationale and `memory/decisions/0020` for the measured
/// comparison against `compute_ate`.
pub fn compute_ate_prefix_aligned(estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>], align_prefix_len: usize) -> Option<AteStats> {
    let errors = compute_ate_series_prefix_aligned(estimated, groundtruth, align_prefix_len)?;
    Some(ate_stats_from_errors(errors))
}

/// Fits `umeyama_alignment` over a sliding window of `window_len`
/// consecutive points, sliding by `step`, and returns each window's own
/// fitted scale paired with its starting index — a *local*, over-time
/// view of the estimator's own reconstructed scale, instead of the
/// single whole-trajectory number `umeyama_alignment` itself gives
/// (`plan/STAGE6.md` M5, `memory/decisions/0020`'s own open question:
/// is the scale anomaly gradual drift or a step-change at a specific
/// event?). A window's own fit is skipped (not included in the result,
/// rather than panicking or returning `None` for the whole run) if
/// `umeyama_alignment` itself can't fit it (e.g. a near-degenerate
/// window with too little translation spread) — real windows elsewhere
/// in the run are still worth reporting.
pub fn compute_sliding_window_scale(estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>], window_len: usize, step: usize) -> Vec<(usize, f64)> {
    if estimated.len() != groundtruth.len() || window_len < 3 || step == 0 || estimated.len() < window_len {
        return Vec::new();
    }
    (0..=estimated.len() - window_len)
        .step_by(step)
        .filter_map(|start| {
            let alignment = umeyama_alignment(&estimated[start..start + window_len], &groundtruth[start..start + window_len])?;
            Some((start, alignment.scale))
        })
        .collect()
}

/// Per-axis standard-deviation ratio (estimated / groundtruth), *after*
/// rotating `estimated` into groundtruth's own frame using `umeyama_
/// alignment`'s fitted rotation — but deliberately not its isotropic
/// scale, which is exactly the single-number compromise this function
/// exists to look past. `umeyama_alignment` (and `compute_sliding_
/// window_scale`) can only ever report one scalar scale per fit; if the
/// true error is anisotropic (a different stretch factor per axis, not
/// a uniform one), that single number is a compromise average that can
/// swing wildly between windows depending on which axis's motion
/// happens to dominate each one (`plan/STAGE6.md` M5 found exactly this
/// on real data — `memory/decisions/0027`). Comparing raw per-axis
/// spread *without* first rotating into a common frame is meaningless
/// (each trajectory's own world frame is arbitrary), which is the
/// mistake this function's own doc comment warns against making again.
/// Returns `None` if `umeyama_alignment` itself can't fit (see its own
/// conditions) or if `groundtruth`'s per-axis variance is degenerate
/// (a straight-line or planar run along some axis, where "ratio" isn't
/// meaningful).
pub fn compute_axis_scale_ratios(estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>]) -> Option<Vector3<f64>> {
    let alignment = umeyama_alignment(estimated, groundtruth)?;
    let n = estimated.len() as f64;
    let mean_e: Vector3<f64> = estimated.iter().sum::<Vector3<f64>>() / n;
    let mean_g: Vector3<f64> = groundtruth.iter().sum::<Vector3<f64>>() / n;

    let mut var_e = Vector3::<f64>::zeros();
    let mut var_g = Vector3::<f64>::zeros();
    for (e, g) in estimated.iter().zip(groundtruth.iter()) {
        let rotated = alignment.rotation * (e - mean_e);
        let centered_gt = g - mean_g;
        var_e += rotated.component_mul(&rotated);
        var_g += centered_gt.component_mul(&centered_gt);
    }
    var_e /= n;
    var_g /= n;
    if var_g.x <= 1e-12 || var_g.y <= 1e-12 || var_g.z <= 1e-12 {
        return None;
    }
    Some(Vector3::new((var_e.x / var_g.x).sqrt(), (var_e.y / var_g.y).sqrt(), (var_e.z / var_g.z).sqrt()))
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

    #[test]
    fn ate_series_is_in_order_and_summarizes_to_the_same_rmse_as_compute_ate() {
        let source: Vec<Vector3<f64>> = (0..10).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        let target: Vec<Vector3<f64>> = source.iter().enumerate().map(|(i, p)| p + Vector3::new(0.0, if i % 2 == 0 { 0.1 } else { -0.1 }, 0.0)).collect();

        let series = compute_ate_series(&source, &target).expect("series should compute");
        let stats = compute_ate(&source, &target).expect("stats should compute");

        assert_eq!(series.len(), source.len(), "series must be in the same per-point order as the input, not sorted");
        let rmse_from_series = (series.iter().map(|e| e * e).sum::<f64>() / series.len() as f64).sqrt();
        assert_relative_eq!(rmse_from_series, stats.rmse, epsilon = 1e-9);
    }

    /// The whole point of `plan/STAGE5.md` goal 1, as a synthetic,
    /// reproducible check (not just the real-data numbers in `memory/
    /// decisions/0020`): a trajectory that's exactly accurate for its
    /// first `prefix` points and then drifts should show near-zero error
    /// early under prefix-anchored alignment, unlike whole-trajectory
    /// alignment, which is free to compromise the early fit to
    /// accommodate the later drift.
    #[test]
    fn prefix_aligned_ate_is_near_zero_early_even_when_whole_trajectory_alignment_is_not() {
        let n = 30;
        let prefix = 10;
        let groundtruth: Vec<Vector3<f64>> = (0..n).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        let estimated: Vec<Vector3<f64>> = groundtruth
            .iter()
            .enumerate()
            .map(|(i, p)| if i < prefix { *p } else { p + Vector3::new(0.0, 0.15 * (i - prefix + 1) as f64, 0.0) })
            .collect();

        let prefix_series = compute_ate_series_prefix_aligned(&estimated, &groundtruth, prefix).expect("prefix series should compute");
        let full_series = compute_ate_series(&estimated, &groundtruth).expect("full series should compute");

        // Prefix-anchored: the exactly-accurate early portion stays
        // (near-)exactly accurate after alignment, since the fit never
        // saw the later drift at all.
        for &e in &prefix_series[..prefix] {
            assert!(e < 1e-9, "prefix-aligned error on the untouched early portion should be ~0, got {e}");
        }
        // And the drifted portion shows real, growing error under
        // prefix-anchored alignment — this isn't "everything reads zero,"
        // it's "the early portion reads zero and the drift is visible
        // where it actually happened."
        assert!(prefix_series[n - 1] > 1.0, "prefix-aligned error should show real drift by the end, got {}", prefix_series[n - 1]);

        // Whole-trajectory alignment, by contrast, is pulled by the later
        // drift into a compromise fit that reports real error even on
        // the untouched-by-construction early portion.
        assert!(full_series[0] > prefix_series[0] + 1e-6, "whole-trajectory alignment should report more early error than prefix-anchored, got full={} prefix={}", full_series[0], prefix_series[0]);
    }

    #[test]
    fn prefix_aligned_ate_matches_compute_ate_stats_shape() {
        let source: Vec<Vector3<f64>> = (0..10).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        let target: Vec<Vector3<f64>> = source.iter().enumerate().map(|(i, p)| p + Vector3::new(0.0, if i % 2 == 0 { 0.1 } else { -0.1 }, 0.0)).collect();

        // A prefix covering the whole trajectory should exactly match
        // compute_ate's own numbers (same fit, same points, same stats).
        let stats = compute_ate(&source, &target).expect("stats should compute");
        let prefix_stats = compute_ate_prefix_aligned(&source, &target, source.len()).expect("prefix stats should compute");
        assert_relative_eq!(stats.rmse, prefix_stats.rmse, epsilon = 1e-9);

        // An oversized prefix is clamped to the trajectory's own length,
        // not an error.
        let clamped_stats = compute_ate_prefix_aligned(&source, &target, source.len() * 10).expect("oversized prefix should clamp, not fail");
        assert_relative_eq!(stats.rmse, clamped_stats.rmse, epsilon = 1e-9);
    }

    #[test]
    fn prefix_aligned_ate_returns_none_on_mismatched_lengths() {
        let source: Vec<Vector3<f64>> = (0..10).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        let target: Vec<Vector3<f64>> = (0..5).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        assert!(compute_ate_prefix_aligned(&source, &target, 5).is_none());
    }

    /// The core distinguishing property `plan/STAGE6.md` M5 needs: a
    /// *gradual* scale drift (the estimator's own reconstructed scale
    /// ramping smoothly over the run) shows up as many consecutive
    /// windows each reporting a slightly different value, not a handful
    /// of flat plateaus — see `sliding_window_scale_detects_a_step_
    /// change` for the contrasting case.
    #[test]
    fn sliding_window_scale_detects_gradual_drift() {
        let n = 60;
        let groundtruth: Vec<Vector3<f64>> = (0..n).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        // Estimated points ramp from 1.0x to 2.0x groundtruth scale,
        // linearly over the run.
        let estimated: Vec<Vector3<f64>> = (0..n)
            .map(|i| {
                let local_scale = 1.0 + (i as f64 / (n - 1) as f64);
                Vector3::new(i as f64 * local_scale, 0.0, 0.0)
            })
            .collect();

        let series = compute_sliding_window_scale(&estimated, &groundtruth, 10, 1);
        assert!(series.len() > 10, "expected many overlapping windows, got {}", series.len());

        // `umeyama_alignment(estimated, groundtruth)`'s scale maps
        // *estimated* onto *groundtruth* — since estimated is inflated
        // by an increasing local_scale here, the fitted scale (the
        // shrink factor needed to match groundtruth) *decreases* over
        // the run. Every window's scale should be strictly less than
        // the previous one's — a smooth ramp, not flat segments with a
        // jump.
        let decreasing_pairs = series.windows(2).filter(|w| w[1].1 < w[0].1).count();
        assert!(
            decreasing_pairs as f64 > 0.9 * (series.len() - 1) as f64,
            "expected a smooth, mostly-monotonic change across windows, only {decreasing_pairs}/{} pairs decreased",
            series.len() - 1
        );
    }

    /// The contrasting case: an abrupt, one-time change in the
    /// estimator's own scale (as if caused by a single event — a
    /// marginalization, a track-loss recovery, a bootstrap artifact —
    /// rather than compounding optimization drift) shows up as two flat
    /// plateaus with a short transition between them, not a smooth ramp
    /// across the whole run.
    #[test]
    fn sliding_window_scale_detects_a_step_change() {
        let n = 60;
        let groundtruth: Vec<Vector3<f64>> = (0..n).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        let estimated: Vec<Vector3<f64>> = (0..n)
            .map(|i| {
                let local_scale = if i < n / 2 { 1.0 } else { 2.0 };
                Vector3::new(i as f64 * local_scale, 0.0, 0.0)
            })
            .collect();

        let window_len = 10;
        let series = compute_sliding_window_scale(&estimated, &groundtruth, window_len, 1);
        assert!(!series.is_empty());

        // Windows entirely before the step (fully within the first
        // plateau, local_scale=1.0) should read a fitted scale of 1.0;
        // windows entirely after it (local_scale=2.0, so estimated is
        // 2x groundtruth — the fitted shrink factor is 1/2) should read
        // 0.5 — real flat plateaus, unlike the gradual case where only
        // the very first/last window touch those extremes.
        let before_step: Vec<f64> = series.iter().filter(|(start, _)| start + window_len <= n / 2).map(|(_, s)| *s).collect();
        let after_step: Vec<f64> = series.iter().filter(|(start, _)| *start >= n / 2).map(|(_, s)| *s).collect();
        assert!(before_step.len() >= 5, "expected several windows fully before the step, got {}", before_step.len());
        assert!(after_step.len() >= 5, "expected several windows fully after the step, got {}", after_step.len());
        for s in &before_step {
            assert_relative_eq!(*s, 1.0, epsilon = 1e-9);
        }
        for s in &after_step {
            assert_relative_eq!(*s, 0.5, epsilon = 1e-9);
        }
    }

    #[test]
    fn sliding_window_scale_returns_empty_on_mismatched_or_too_short_input() {
        let short: Vec<Vector3<f64>> = (0..5).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        let mismatched: Vec<Vector3<f64>> = (0..7).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        assert!(compute_sliding_window_scale(&short, &short, 10, 1).is_empty());
        assert!(compute_sliding_window_scale(&short, &mismatched, 3, 1).is_empty());
    }

    /// A synthetic, per-axis-anisotropically-distorted "estimated"
    /// trajectory (built from a known rotation and known, *different*
    /// per-axis scale factors applied to groundtruth) should have those
    /// exact factors recovered — the real-data motivation
    /// (`plan/STAGE6.md` M5, `memory/decisions/0027`) for this function
    /// existing at all: a single isotropic Umeyama scale can't represent
    /// this, but this function's whole point is to look past it.
    /// Groundtruth points are pseudo-random (not a smooth parametric
    /// curve) specifically so their x/y/z components are uncorrelated —
    /// a curve like a helix would make Umeyama's own rotation fit
    /// ambiguous with the anisotropic scale itself, confounding this
    /// test's own premise.
    #[test]
    fn axis_scale_ratios_recovers_known_anisotropic_distortion() {
        let mut state = 123u64;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        let n = 5000;
        let groundtruth: Vec<Vector3<f64>> = (0..n).map(|_| Vector3::new(next() * 5.0, next() * 5.0, next() * 5.0)).collect();

        let true_rotation = SO3::exp(Vector3::new(0.3, -0.2, 0.5));
        let rot_matrix = true_rotation.matrix();
        let known_ratios = Vector3::new(3.0, 1.0, 5.0);
        let mean_gt: Vector3<f64> = groundtruth.iter().sum::<Vector3<f64>>() / n as f64;

        // estimated built so that `rotation * (estimated - mean) ==
        // known_ratios .* (groundtruth - mean)` exactly.
        let estimated: Vec<Vector3<f64>> = groundtruth
            .iter()
            .map(|g| {
                let centered = g - mean_gt;
                let scaled = Vector3::new(centered.x * known_ratios.x, centered.y * known_ratios.y, centered.z * known_ratios.z);
                rot_matrix.transpose() * scaled + mean_gt
            })
            .collect();

        let ratios = compute_axis_scale_ratios(&estimated, &groundtruth).expect("should compute on a well-conditioned synthetic trajectory");
        // 5000 random points give near-zero but not exactly-zero cross-
        // axis correlation, so Umeyama's own fitted rotation is only
        // approximately `rot_matrix` — 1e-2 is loose enough to absorb
        // that finite-sampling noise (confirmed converging toward exact
        // as n grows: 200 points gave ~9% error, 5000 gives <0.1%) while
        // still tight enough to catch a real formula mistake.
        assert_relative_eq!(ratios, known_ratios, epsilon = 1e-2);
    }

    #[test]
    fn axis_scale_ratios_is_all_ones_for_identical_trajectories() {
        let mut state = 7u64;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        let points: Vec<Vector3<f64>> = (0..50).map(|_| Vector3::new(next(), next(), next())).collect();
        let ratios = compute_axis_scale_ratios(&points, &points).expect("identical trajectories should compute");
        assert_relative_eq!(ratios, Vector3::new(1.0, 1.0, 1.0), epsilon = 1e-9);
    }

    #[test]
    fn axis_scale_ratios_returns_none_for_a_degenerate_axis() {
        // Every groundtruth point has z=0 — zero variance on that axis,
        // so a ratio against it is meaningless.
        let groundtruth: Vec<Vector3<f64>> = (0..10).map(|i| Vector3::new(i as f64, (i as f64 * 0.3).sin(), 0.0)).collect();
        let estimated = groundtruth.clone();
        assert!(compute_axis_scale_ratios(&estimated, &groundtruth).is_none());
    }
}
