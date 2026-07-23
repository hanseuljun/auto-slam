use std::path::Path;

use nalgebra::Vector3;

use crate::align::{compute_ate, AteStats};
use crate::rpe::{compute_rpe, RpeStats};
use crate::timing::TimingBreakdown;

/// A full report for one estimated trajectory against groundtruth: ATE,
/// RPE at each requested delta, and (optionally) the wall-clock timing
/// breakdown `plan/STAGE2.md`'s real-time bar is measured against.
/// Produced by `bin/slam-run` per sequence, and aggregated into
/// `docs/RESULTS.md`'s tables.
#[derive(Debug, Clone)]
pub struct TrajectoryReport {
    pub sequence_name: String,
    pub ate: AteStats,
    pub rpe: Vec<RpeStats>,
    pub timing: Option<TimingBreakdown>,
}

/// Builds a `TrajectoryReport` from timestamp-matched estimated/groundtruth
/// position pairs (already the intersection actually covered by
/// groundtruth — see callers' `gt.interpolate` filtering).
pub fn build_report(sequence_name: &str, estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>], rpe_deltas: &[usize], timing: Option<TimingBreakdown>) -> Option<TrajectoryReport> {
    let ate = compute_ate(estimated, groundtruth)?;
    let rpe = rpe_deltas.iter().filter_map(|&delta| compute_rpe(estimated, groundtruth, delta)).collect();
    Some(TrajectoryReport {
        sequence_name: sequence_name.to_string(),
        ate,
        rpe,
        timing,
    })
}

/// Writes a per-timestamp CSV of estimated vs. groundtruth position, for
/// external plotting (`CLAUDE.md`'s "plain text/CSV" verification
/// requirement) — one row per timestamp actually covered by groundtruth.
pub fn write_trajectory_csv(path: impl AsRef<Path>, timestamps: &[u64], estimated: &[Vector3<f64>], groundtruth: &[Vector3<f64>]) -> anyhow::Result<()> {
    anyhow::ensure!(
        timestamps.len() == estimated.len() && estimated.len() == groundtruth.len(),
        "timestamps/estimated/groundtruth length mismatch: {} vs {} vs {}",
        timestamps.len(),
        estimated.len(),
        groundtruth.len()
    );
    if let Some(parent) = path.as_ref().parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut writer = csv::Writer::from_path(path)?;
    writer.write_record(["timestamp_ns", "est_x", "est_y", "est_z", "gt_x", "gt_y", "gt_z"])?;
    for ((t, e), g) in timestamps.iter().zip(estimated.iter()).zip(groundtruth.iter()) {
        writer.write_record([t.to_string(), e.x.to_string(), e.y.to_string(), e.z.to_string(), g.x.to_string(), g.y.to_string(), g.z.to_string()])?;
    }
    writer.flush()?;
    Ok(())
}

/// The parsed contents of a `trajectory.csv` written by
/// `write_trajectory_csv` — what `bin/slam-viz`'s 3D panel (`plan/
/// STAGE3.md` M2) reads a run's trajectory back through, instead of a
/// second, parallel CSV parser living in `slam-render`.
#[derive(Debug, Clone, Default)]
pub struct TrajectoryPoints {
    pub timestamps: Vec<u64>,
    pub estimated: Vec<Vector3<f64>>,
    pub groundtruth: Vec<Vector3<f64>>,
}

/// Reads back a `trajectory.csv` written by `write_trajectory_csv` — the
/// exact inverse, field for field.
pub fn read_trajectory_csv(path: impl AsRef<Path>) -> anyhow::Result<TrajectoryPoints> {
    let mut reader = csv::ReaderBuilder::new().has_headers(true).from_path(path)?;
    let mut points = TrajectoryPoints::default();
    for record in reader.records() {
        let record = record?;
        let field = |i: usize| -> anyhow::Result<f64> { Ok(record.get(i).unwrap().trim().parse()?) };
        points.timestamps.push(record.get(0).unwrap().trim().parse::<u64>()?);
        points.estimated.push(Vector3::new(field(1)?, field(2)?, field(3)?));
        points.groundtruth.push(Vector3::new(field(4)?, field(5)?, field(6)?));
    }
    Ok(points)
}

/// Writes one summary row per sequence's `TrajectoryReport` (ATE, every
/// RPE delta present in the *first* report — assumed consistent across all
/// reports passed in — and timing/real-time-factor columns when present)
/// — the "aggregate report" M9/Stage 2's M0 asks for.
pub fn write_summary_csv(path: impl AsRef<Path>, reports: &[TrajectoryReport]) -> anyhow::Result<()> {
    if let Some(parent) = path.as_ref().parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut writer = csv::Writer::from_path(path)?;
    let deltas: Vec<usize> = reports.first().map(|r| r.rpe.iter().map(|s| s.delta).collect()).unwrap_or_default();

    let mut header = vec![
        "sequence".to_string(),
        "ate_rmse".to_string(),
        "ate_mean".to_string(),
        "ate_median".to_string(),
        "ate_std".to_string(),
        "ate_max".to_string(),
        "num_points".to_string(),
    ];
    for d in &deltas {
        header.push(format!("rpe_rmse_d{d}"));
    }
    header.extend(["vision_seconds".to_string(), "optimization_seconds".to_string(), "global_ba_seconds".to_string(), "loop_closure_seconds".to_string(), "data_seconds".to_string(), "real_time_factor".to_string()]);
    writer.write_record(&header)?;

    for r in reports {
        let mut row = vec![
            r.sequence_name.clone(),
            format!("{:.6}", r.ate.rmse),
            format!("{:.6}", r.ate.mean),
            format!("{:.6}", r.ate.median),
            format!("{:.6}", r.ate.std),
            format!("{:.6}", r.ate.max),
            r.ate.num_points.to_string(),
        ];
        for d in &deltas {
            let rmse = r.rpe.iter().find(|s| s.delta == *d).map(|s| s.rmse);
            row.push(rmse.map(|v| format!("{v:.6}")).unwrap_or_default());
        }
        match r.timing {
            Some(t) => row.extend([format!("{:.3}", t.vision_seconds), format!("{:.3}", t.optimization_seconds), format!("{:.3}", t.global_ba_seconds), format!("{:.3}", t.loop_closure_seconds), format!("{:.3}", t.data_seconds), format!("{:.4}", t.real_time_factor())]),
            None => row.extend([String::new(), String::new(), String::new(), String::new(), String::new(), String::new()]),
        }
        writer.write_record(&row)?;
    }
    writer.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_report_includes_ate_and_every_requested_rpe_delta() {
        let n = 20;
        let groundtruth: Vec<Vector3<f64>> = (0..n).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        let estimated: Vec<Vector3<f64>> = groundtruth.iter().map(|p| p + Vector3::new(0.05, 0.0, 0.0)).collect();
        let report = build_report("TEST_SEQ", &estimated, &groundtruth, &[1, 5], None).expect("report should build");
        assert_eq!(report.sequence_name, "TEST_SEQ");
        assert_eq!(report.rpe.len(), 2);
        assert!(report.rpe.iter().any(|s| s.delta == 1));
        assert!(report.rpe.iter().any(|s| s.delta == 5));
        assert!(report.timing.is_none());
    }

    #[test]
    fn writes_trajectory_csv_with_expected_header_and_row_count() {
        let dir = std::env::temp_dir().join(format!("slam-eval-test-{}", std::process::id()));
        let path = dir.join("trajectory.csv");
        let timestamps = vec![100u64, 200, 300];
        let estimated = vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), Vector3::new(2.0, 0.0, 0.0)];
        let groundtruth = vec![Vector3::new(0.1, 0.0, 0.0), Vector3::new(1.1, 0.0, 0.0), Vector3::new(2.1, 0.0, 0.0)];
        write_trajectory_csv(&path, &timestamps, &estimated, &groundtruth).expect("write should succeed");

        let contents = std::fs::read_to_string(&path).expect("read back");
        assert_eq!(contents.lines().count(), 4); // header + 3 rows
        assert!(contents.contains("timestamp_ns,est_x"));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn read_trajectory_csv_round_trips_write_trajectory_csv() {
        let dir = std::env::temp_dir().join(format!("slam-eval-test-roundtrip-{}", std::process::id()));
        let path = dir.join("trajectory.csv");
        let timestamps = vec![100u64, 200, 300];
        let estimated = vec![Vector3::new(0.0, 0.1, -0.2), Vector3::new(1.5, 0.0, 3.3), Vector3::new(2.0, -1.0, 0.0)];
        let groundtruth = vec![Vector3::new(0.1, 0.0, 0.0), Vector3::new(1.1, 0.0, 0.0), Vector3::new(2.1, 0.0, 0.0)];
        write_trajectory_csv(&path, &timestamps, &estimated, &groundtruth).expect("write should succeed");

        let points = read_trajectory_csv(&path).expect("read should succeed");
        assert_eq!(points.timestamps, timestamps);
        for (a, b) in points.estimated.iter().zip(estimated.iter()) {
            assert!((a - b).norm() < 1e-9);
        }
        for (a, b) in points.groundtruth.iter().zip(groundtruth.iter()) {
            assert!((a - b).norm() < 1e-9);
        }
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn writes_summary_csv_with_timing_columns_when_present() {
        let dir = std::env::temp_dir().join(format!("slam-eval-test-summary-{}", std::process::id()));
        let path = dir.join("summary.csv");
        let n = 10;
        let groundtruth: Vec<Vector3<f64>> = (0..n).map(|i| Vector3::new(i as f64, 0.0, 0.0)).collect();
        let estimated = groundtruth.clone();
        let timing = TimingBreakdown { vision_seconds: 1.0, optimization_seconds: 2.0, global_ba_seconds: 3.0, loop_closure_seconds: 0.0, data_seconds: 5.0 };
        let report_a = build_report("SEQ_A", &estimated, &groundtruth, &[1], Some(timing)).unwrap();
        let report_b = build_report("SEQ_B", &estimated, &groundtruth, &[1], None).unwrap();
        write_summary_csv(&path, &[report_a, report_b]).expect("write should succeed");

        let contents = std::fs::read_to_string(&path).expect("read back");
        assert_eq!(contents.lines().count(), 3); // header + 2 rows
        assert!(contents.contains("real_time_factor"));
        assert!(contents.contains("0.6000")); // (1.0+2.0)/5.0 for SEQ_A
        std::fs::remove_dir_all(&dir).ok();
    }
}
