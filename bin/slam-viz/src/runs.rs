use std::path::{Path, PathBuf};

use slam_eval::RunMeta;

/// One discovered `runs/<sequence>/<run_id>/` entry — the run picker's
/// data source (`plan/STAGE3.md` M3/M7, goal 3: per-run browsing).
#[derive(Debug, Clone)]
pub struct DiscoveredRun {
    pub dir: PathBuf,
    pub meta: RunMeta,
}

/// Scans `runs_dir` for `runs/<sequence>/<run_id>/meta.json` files
/// (Stage 3 M0's non-clobbering per-run history layout) and returns
/// every one that parses, most recent `run_id` first. Directories
/// without a `meta.json` (e.g. `runs/<sequence>/trajectory.csv`'s own
/// latest-snapshot path, which lives one level up from the per-run
/// entries) are silently skipped, not an error — `runs/` mixes both
/// layouts on purpose (`plan/STAGE3.md` M0 kept the old path additive).
pub fn discover_runs(runs_dir: &Path) -> Vec<DiscoveredRun> {
    let mut found = Vec::new();
    let Ok(sequence_entries) = std::fs::read_dir(runs_dir) else {
        return found;
    };
    for seq_entry in sequence_entries.flatten() {
        let seq_path = seq_entry.path();
        if !seq_path.is_dir() {
            continue;
        }
        let Ok(run_entries) = std::fs::read_dir(&seq_path) else {
            continue;
        };
        for run_entry in run_entries.flatten() {
            let run_dir = run_entry.path();
            if !run_dir.is_dir() {
                continue;
            }
            if let Ok(meta) = slam_eval::read_run_meta(run_dir.join("meta.json")) {
                found.push(DiscoveredRun { dir: run_dir, meta });
            }
        }
    }
    found.sort_by(|a, b| b.meta.run_id.cmp(&a.meta.run_id));
    found
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use slam_eval::{AteStats, RunConfig};

    fn fake_meta(sequence_name: &str, run_id: &str) -> RunMeta {
        RunMeta {
            sequence_name: sequence_name.to_string(),
            run_id: run_id.to_string(),
            timestamp_utc: "2026-07-23T00:00:00Z".to_string(),
            git_commit: None,
            num_frames: 10,
            config: RunConfig { window_size: 8, keyframe_stride: 10, huber_delta: 3.0, solver_max_iterations: 6, full_sequence: false, frame_cap: 600 },
            ate: AteStats { rmse: 0.1, mean: 0.1, median: 0.1, std: 0.01, max: 0.2, num_points: 10 },
            rpe: vec![],
            timing: None,
        }
    }

    fn write_fixture_run(runs_dir: &Path, sequence: &str, run_id: &str, estimated: &[Vector3<f64>]) {
        let run_dir = runs_dir.join(sequence).join(run_id);
        let timestamps: Vec<u64> = (0..estimated.len() as u64).collect();
        let groundtruth = estimated.to_vec();
        slam_eval::write_trajectory_csv(run_dir.join("trajectory.csv"), &timestamps, estimated, &groundtruth).unwrap();
        slam_eval::write_run_meta(run_dir.join("meta.json"), &fake_meta(sequence, run_id)).unwrap();
    }

    #[test]
    fn discovers_runs_across_sequences_sorted_most_recent_first() {
        let dir = std::env::temp_dir().join(format!("slam-viz-test-runs-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        write_fixture_run(&dir, "MH_01_easy", "20260101-000000-000", &[Vector3::zeros()]);
        write_fixture_run(&dir, "MH_01_easy", "20260102-000000-000", &[Vector3::zeros()]);
        write_fixture_run(&dir, "MH_02_easy", "20260101-120000-000", &[Vector3::zeros()]);

        let runs = discover_runs(&dir);
        assert_eq!(runs.len(), 3);
        assert_eq!(runs[0].meta.run_id, "20260102-000000-000");
        assert_eq!(runs[1].meta.run_id, "20260101-120000-000");
        assert_eq!(runs[2].meta.run_id, "20260101-000000-000");

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn missing_runs_dir_returns_empty_not_a_panic() {
        let missing = PathBuf::from("/nonexistent/slam-viz-runs-dir-that-does-not-exist");
        assert_eq!(discover_runs(&missing).len(), 0);
    }

    #[test]
    fn a_sequence_directory_without_any_valid_meta_json_is_skipped() {
        let dir = std::env::temp_dir().join(format!("slam-viz-test-runs-skip-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        // The old, still-supported "latest snapshot" layout: a bare
        // trajectory.csv directly under the sequence dir, no meta.json,
        // no run_id subdirectory - must not be mistaken for a run entry.
        std::fs::create_dir_all(dir.join("MH_01_easy")).unwrap();
        std::fs::write(dir.join("MH_01_easy").join("trajectory.csv"), "timestamp_ns,est_x\n").unwrap();

        assert_eq!(discover_runs(&dir).len(), 0);
        std::fs::remove_dir_all(&dir).ok();
    }
}
