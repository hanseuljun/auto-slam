use std::path::Path;

use chrono::Utc;
use serde::{Deserialize, Serialize};

use crate::align::AteStats;
use crate::rpe::RpeStats;
use crate::timing::TimingBreakdown;

/// The pipeline knobs that actually affect a run's accuracy/timing
/// numbers, captured alongside the numbers themselves — the exact kind
/// of value `memory/decisions/0017`'s tuning sweeps varied run to run,
/// with no record beyond a commit message of which run used which
/// config. Flat and `Copy` on purpose: this is a snapshot for display in
/// `bin/slam-viz`'s run picker (`plan/STAGE3.md` M7), not a live handle
/// on the pipeline.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RunConfig {
    pub window_size: usize,
    pub keyframe_stride: usize,
    pub huber_delta: f64,
    pub solver_max_iterations: usize,
    pub full_sequence: bool,
    pub frame_cap: usize,
}

/// Everything about one sequence's run worth showing in a run picker
/// without re-parsing `trajectory.csv`: when it ran, against what code,
/// with what config, and the resulting ATE/RPE/timing — the `plan/
/// STAGE3.md` M0 prerequisite for goal 3 (per-run browsing). Written as
/// `runs/<sequence>/<run_id>/meta.json` alongside that run's
/// `trajectory.csv`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunMeta {
    pub sequence_name: String,
    pub run_id: String,
    /// RFC3339 timestamp, same instant `run_id` is derived from — kept
    /// separately since `run_id` is filesystem-safe (no `:`) and this
    /// isn't, and callers displaying a run want the readable form.
    pub timestamp_utc: String,
    /// `git rev-parse --short HEAD` at run time, best-effort: `None` if
    /// git isn't available or the command fails, not a hard error — a
    /// missing commit hash shouldn't block writing the rest of a run's
    /// results.
    pub git_commit: Option<String>,
    pub num_frames: usize,
    pub config: RunConfig,
    pub ate: AteStats,
    pub rpe: Vec<RpeStats>,
    pub timing: Option<TimingBreakdown>,
}

/// A sortable, filesystem-safe run identifier derived from the current
/// UTC instant: `YYYYMMDD-HHMMSS-mmm`. Millisecond resolution so two
/// runs of the *same* sequence started less than a second apart (e.g.
/// re-running `slam-run` quickly while tuning) still get distinct,
/// non-clobbering directories.
pub fn generate_run_id() -> String {
    Utc::now().format("%Y%m%d-%H%M%S-%3f").to_string()
}

/// Best-effort `git rev-parse --short HEAD`; `None` (not an error) if
/// git isn't on `PATH`, this isn't a git checkout, or the command
/// otherwise fails — a run's numbers are still worth recording even
/// without a commit hash attached.
pub fn current_git_commit() -> Option<String> {
    let output = std::process::Command::new("git").args(["rev-parse", "--short", "HEAD"]).output().ok()?;
    if !output.status.success() {
        return None;
    }
    let commit = String::from_utf8(output.stdout).ok()?;
    let commit = commit.trim();
    if commit.is_empty() {
        None
    } else {
        Some(commit.to_string())
    }
}

/// Writes `meta.json` for one run, creating parent directories as
/// needed (mirrors `write_trajectory_csv`'s own behavior).
pub fn write_run_meta(path: impl AsRef<Path>, meta: &RunMeta) -> anyhow::Result<()> {
    if let Some(parent) = path.as_ref().parent() {
        std::fs::create_dir_all(parent)?;
    }
    let json = serde_json::to_string_pretty(meta)?;
    std::fs::write(path, json)?;
    Ok(())
}

/// Reads back a `meta.json` written by `write_run_meta` — used by
/// `bin/slam-viz`'s run picker (`plan/STAGE3.md` M7), not by this
/// crate's own tests only.
pub fn read_run_meta(path: impl AsRef<Path>) -> anyhow::Result<RunMeta> {
    let contents = std::fs::read_to_string(path)?;
    Ok(serde_json::from_str(&contents)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn run_id_is_filesystem_safe_and_monotonic_across_calls() {
        let a = generate_run_id();
        std::thread::sleep(std::time::Duration::from_millis(5));
        let b = generate_run_id();
        assert!(!a.contains(':'), "run_id must not contain filesystem-unsafe characters: {a}");
        assert!(a < b, "run_id must sort chronologically: {a} vs {b}");
    }

    #[test]
    fn writes_and_reads_back_run_meta() {
        let dir = std::env::temp_dir().join(format!("slam-eval-test-run-meta-{}", std::process::id()));
        let path = dir.join("meta.json");
        let meta = RunMeta {
            sequence_name: "MH_01_easy".to_string(),
            run_id: "20260722-120000-000".to_string(),
            timestamp_utc: "2026-07-22T12:00:00Z".to_string(),
            git_commit: Some("abc1234".to_string()),
            num_frames: 600,
            config: RunConfig { window_size: 8, keyframe_stride: 10, huber_delta: 3.0, solver_max_iterations: 6, full_sequence: false, frame_cap: 600 },
            ate: AteStats { rmse: 0.151, mean: 0.122, median: 0.126, std: 0.089, max: 0.350, num_points: 101 },
            rpe: vec![RpeStats { delta: 1, rmse: 0.105, mean: 0.08, median: 0.09, std: 0.03, max: 0.21, num_pairs: 97 }],
            timing: Some(TimingBreakdown { vision_seconds: 13.7, optimization_seconds: 2.6, global_ba_seconds: 2.8, loop_closure_seconds: 0.0, data_seconds: 30.0 }),
        };

        write_run_meta(&path, &meta).expect("write should succeed");
        let read_back = read_run_meta(&path).expect("read should succeed");

        assert_eq!(read_back.sequence_name, meta.sequence_name);
        assert_eq!(read_back.run_id, meta.run_id);
        assert_eq!(read_back.git_commit, meta.git_commit);
        assert_eq!(read_back.config.window_size, meta.config.window_size);
        assert!((read_back.ate.rmse - meta.ate.rmse).abs() < 1e-12);
        assert_eq!(read_back.rpe.len(), 1);
        assert!(read_back.timing.is_some());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn missing_git_repo_returns_none_not_a_panic() {
        // Best-effort: whatever the sandbox's git state is, this must not
        // panic or return an error - either a commit hash or None.
        let _ = current_git_commit();
    }
}
