//! `slam-run`: Stage 2's M0, finishing Stage 1's M9 — "one command runs a
//! sequence and prints the report," extended with the wall-clock timing
//! `plan/STAGE2.md`'s real-time bar (goal 1) is measured against. Runs the
//! full stereo-inertial VIO pipeline (Stage 1 M5, track-loss recovery M6)
//! plus one global bundle-adjustment pass (M8), computes ATE + RPE against
//! groundtruth, and reports a real-time factor: wall-clock spent in the
//! continuous per-frame loop (frontend tracking + windowed backend
//! optimization) divided by the amount of sensor data it processed.
//!
//! Defaults to a **bounded** run (`--frames`, default below), not a full
//! sequence: an earlier, since-rolled-back attempt at this exact milestone
//! ran the un-truncated pipeline over one full sequence and took 30+
//! minutes wall-clock (see `plan/STAGE2.md`'s "What we already know" for
//! the root cause — M8's global BA is a dense O(n^3) solve that scales
//! with total keyframe count, since M5's window doesn't marginalize yet).
//! Pass `--full` for a genuine full-sequence run once that's actually been
//! fixed (Stage 2's M1-M3) or when the wait is acceptable.
//!
//! Loop closure (M7) is deliberately not chained in here: it currently
//! operates on `VoPipeline`, not `VioPipeline`, and only `MH_05_difficult`
//! has a real loop to close — `bin/slam-inspect` already demonstrates that
//! path on its own.

use std::path::{Path, PathBuf};
use std::time::Instant;

use clap::Parser;
use nalgebra::Vector3;

#[derive(Parser)]
#[command(name = "slam-run", about = "Run stereo-inertial VIO + global BA over one or more EuRoC sequences and report ATE/RPE/real-time factor")]
struct Cli {
    /// Sequence directories to run (each containing an `mav0/`
    /// subdirectory). Defaults to every sequence under `data/machine_hall`.
    #[arg(value_name = "SEQUENCE_DIR")]
    sequences: Vec<PathBuf>,

    /// Directory to write per-sequence trajectory CSVs and the aggregate
    /// summary CSV under.
    #[arg(long, default_value = "runs")]
    out_dir: PathBuf,

    /// Run every frame of the sequence instead of the bounded default.
    /// Slow (see module doc comment) until Stage 2's M1-M3 land.
    #[arg(long)]
    full: bool,

    /// Frame cap for the default (non-`--full`) run. ~600 frames is ~30s
    /// of data at EuRoC's 20Hz cam0 rate — enough keyframes for a
    /// meaningful real-time-factor measurement without the tool itself
    /// becoming impractical to iterate with.
    #[arg(long, default_value_t = 600)]
    frames: usize,
}

fn default_sequences() -> Vec<PathBuf> {
    let root = PathBuf::from("data/machine_hall");
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(&root)
        .map(|entries| entries.filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.is_dir()).collect())
        .unwrap_or_default();
    dirs.sort();
    dirs
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let sequences = if cli.sequences.is_empty() { default_sequences() } else { cli.sequences };
    if sequences.is_empty() {
        anyhow::bail!("no sequences found under data/machine_hall and none given on the command line");
    }

    // One run_id per invocation (not per sequence): all sequences in a
    // single `slam-run` call share it, so `bin/slam-viz`'s run picker
    // (`plan/STAGE3.md` M7) can group "this set of sequences was run
    // together, under this config" the same way `runs/summary.csv`
    // already aggregates one invocation.
    let run_id = slam_eval::generate_run_id();
    let git_commit = slam_eval::current_git_commit();

    let mut reports = Vec::new();
    for seq_dir in &sequences {
        if let Some(report) = run_sequence(seq_dir, &cli.out_dir, cli.full, cli.frames, &run_id, git_commit.as_deref())? {
            reports.push(report);
        }
    }

    if reports.is_empty() {
        anyhow::bail!("no sequence produced a report (see per-sequence messages above)");
    }

    print_summary_table(&reports);
    let summary_path = cli.out_dir.join("summary.csv");
    slam_eval::write_summary_csv(&summary_path, &reports)?;
    println!("\nwrote aggregate summary to {}", summary_path.display());

    Ok(())
}

/// RPE deltas reported for every sequence, in units of *keyframes* (not
/// raw frames): 1 (immediate drift rate) and 10 (~10s at this pipeline's
/// default keyframe stride of 10 frames / ~20Hz cam0), giving both a
/// local and a medium-horizon drift picture.
const RPE_DELTAS: &[usize] = &[1, 10];

fn run_sequence(seq_dir: &Path, out_dir: &Path, full: bool, frame_cap: usize, run_id: &str, git_commit: Option<&str>) -> anyhow::Result<Option<slam_eval::TrajectoryReport>> {
    let name = seq_dir.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| seq_dir.display().to_string());
    println!("==== {name} ====");

    let mav0 = seq_dir.join("mav0");
    let seq = slam_dataset::EuRocSequence::load(&mav0)?;
    let gt_path = mav0.join("state_groundtruth_estimate0/data.csv");
    if !gt_path.exists() {
        println!("no groundtruth found at {}, skipping", gt_path.display());
        return Ok(None);
    }
    let gt = slam_eval::GroundTruthTrajectory::load(&gt_path)?;

    let all_gyro: Vec<Vector3<f64>> = seq.imu_samples.iter().map(|s| s.gyro).collect();
    let Some(start) = slam_imu::find_stationary_window(&all_gyro, 200, 0.10) else {
        println!("no stationary window to bootstrap from, skipping");
        return Ok(None);
    };
    let accel: Vec<Vector3<f64>> = seq.imu_samples[start..start + 200].iter().map(|s| s.accel).collect();
    let Some(static_init) = slam_imu::static_initialize(&all_gyro[start..start + 200], &accel) else {
        println!("static init failed, skipping");
        return Ok(None);
    };

    let cal = &seq.calibration;
    let rig = slam_geometry::StereoRig {
        t_bs_cam0: slam_core::SE3::from_matrix(&cal.cam0.t_bs),
        t_bs_cam1: slam_core::SE3::from_matrix(&cal.cam1.t_bs),
        cam0: slam_geometry::PinholeCamera::new(cal.cam0.intrinsics, cal.cam0.distortion_coefficients),
        cam1: slam_geometry::PinholeCamera::new(cal.cam1.intrinsics, cal.cam1.distortion_coefficients),
    };
    let initial_state = slam_optim::KeyframeState::new(slam_core::SE3::identity(), Vector3::zeros(), static_init.gyro_bias, Vector3::zeros());
    let gravity_world = static_init.gravity_direction_body * static_init.gravity_magnitude;

    let full_len = seq.cam0_frames.len().min(seq.cam1_frames.len());
    let num_frames = if full { full_len } else { frame_cap.min(full_len) };
    if !full && num_frames < full_len {
        println!("bounded run: {num_frames}/{full_len} frames (pass --full for the complete sequence)");
    }

    let left0 = seq.load_cam0_image(0)?;
    let right0 = seq.load_cam1_image(0)?;
    // Not `solver_config_from_sensor_noise`: tried, measured on real data,
    // reverted — see `decisions/0016`. The tuned ad hoc weights
    // (`SolverConfig::default`) outperform the sensor.yaml-derived ones
    // on 4 of 5 real sequences, even in the narrowest form tried.
    let params = slam_backend::VioParams {
        solver: slam_optim::SolverConfig { max_iterations: 6, ..slam_optim::SolverConfig::default() },
        ..slam_backend::VioParams::default()
    };
    let mut vio = slam_backend::VioPipeline::new(rig, initial_state, seq.cam0_frames[0].timestamp_ns, gravity_world, params);
    vio.init_map(&left0, &right0);

    let mut prev_timestamp = seq.cam0_frames[0].timestamp_ns;
    let mut lost_frames = 0usize;
    for i in 1..num_frames {
        let left = seq.load_cam0_image(i)?;
        let right = seq.load_cam1_image(i)?;
        let timestamp = seq.cam0_frames[i].timestamp_ns;
        let imu_since_last: Vec<_> = seq.imu_samples.iter().filter(|s| s.timestamp_ns > prev_timestamp && s.timestamp_ns <= timestamp).cloned().collect();
        prev_timestamp = timestamp;
        if vio.process_frame(&left, &right, timestamp, &imu_since_last).is_none() {
            // M6's recovery only fails to produce a result on a single
            // genuinely-untrackable frame; the next real frame recovers
            // via IMU propagation. Count and keep going rather than
            // aborting the whole sequence over it.
            lost_frames += 1;
        }
    }
    let data_seconds = (prev_timestamp - seq.cam0_frames[0].timestamp_ns) as f64 * 1e-9;

    let before_ba = vio.all_keyframe_poses().len();
    let ba_wall_start = Instant::now();
    vio.global_bundle_adjustment();
    let ba_wall_elapsed = ba_wall_start.elapsed();
    let trajectory = vio.all_keyframe_poses();

    let mut timestamps = Vec::new();
    let mut estimated = Vec::new();
    let mut groundtruth = Vec::new();
    for (t, p) in &trajectory {
        if let Some(pose) = gt.interpolate(*t) {
            timestamps.push(*t);
            estimated.push(p.inverse().translation);
            groundtruth.push(pose.position);
        }
    }

    let vio_timing = vio.timing();
    let timing = slam_eval::TimingBreakdown {
        vision_seconds: vio_timing.vision_seconds,
        optimization_seconds: vio_timing.optimization_seconds,
        // `vio.timing()`'s own global_ba_seconds is the same number as
        // `ba_wall_elapsed` (both measure the one `global_bundle_adjustment`
        // call above) — using the directly-measured wall-clock here avoids
        // relying on the pipeline's internal bookkeeping matching exactly.
        global_ba_seconds: ba_wall_elapsed.as_secs_f64(),
        loop_closure_seconds: 0.0,
        data_seconds,
    };

    let Some(report) = slam_eval::build_report(&name, &estimated, &groundtruth, RPE_DELTAS, Some(timing)) else {
        println!("not enough groundtruth-covered keyframes to compute a report ({} keyframes, {} with groundtruth), skipping", before_ba, estimated.len());
        return Ok(None);
    };

    // Latest-snapshot path, kept for backward compatibility with
    // `docs/RESULTS.md`'s existing reproduction instructions and anything
    // else that reads "the current trajectory" for this sequence
    // directly, additive to the per-run history written below (`plan/
    // STAGE3.md` M0 — this stage doesn't own `bin/slam-run`'s existing
    // consumers, so it doesn't get to break them).
    let csv_path = out_dir.join(&name).join("trajectory.csv");
    slam_eval::write_trajectory_csv(&csv_path, &timestamps, &estimated, &groundtruth)?;

    let run_dir = out_dir.join(&name).join(run_id);
    let run_csv_path = run_dir.join("trajectory.csv");
    slam_eval::write_trajectory_csv(&run_csv_path, &timestamps, &estimated, &groundtruth)?;

    let run_meta = slam_eval::RunMeta {
        sequence_name: name.clone(),
        run_id: run_id.to_string(),
        timestamp_utc: chrono::Utc::now().to_rfc3339(),
        git_commit: git_commit.map(str::to_string),
        num_frames,
        config: slam_eval::RunConfig {
            window_size: params.window_size,
            keyframe_stride: params.keyframe_stride,
            huber_delta: params.solver.huber_delta,
            solver_max_iterations: params.solver.max_iterations,
            full_sequence: full,
            frame_cap,
        },
        ate: report.ate,
        rpe: report.rpe.clone(),
        timing: report.timing,
    };
    slam_eval::write_run_meta(run_dir.join("meta.json"), &run_meta)?;

    println!(
        "{num_frames} frames ({data_seconds:.1}s of data), {before_ba} keyframes ({lost_frames} unrecoverable single frames), ATE rmse={:.3}m mean={:.3}m median={:.3}m std={:.3}m max={:.3}m",
        report.ate.rmse, report.ate.mean, report.ate.median, report.ate.std, report.ate.max
    );
    for rpe in &report.rpe {
        println!("  RPE (delta={} keyframes): rmse={:.3}m mean={:.3}m max={:.3}m over {} pairs", rpe.delta, rpe.rmse, rpe.mean, rpe.max, rpe.num_pairs);
    }
    println!(
        "  timing: vision={:.2}s optimization={:.2}s global_ba={:.2}s (data={:.1}s) -> real-time factor={:.3}",
        timing.vision_seconds,
        timing.optimization_seconds,
        timing.global_ba_seconds,
        timing.data_seconds,
        timing.real_time_factor()
    );
    println!("wrote trajectory CSV to {}", csv_path.display());
    println!("wrote this run's history entry to {} (trajectory.csv + meta.json)", run_dir.display());
    println!();

    Ok(Some(report))
}

fn print_summary_table(reports: &[slam_eval::TrajectoryReport]) {
    println!("==== summary ====");
    println!("{:<20} {:>10} {:>10} {:>10}", "sequence", "ATE rmse", "ATE mean", "RT factor");
    for r in reports {
        let rtf = r.timing.map(|t| t.real_time_factor());
        println!("{:<20} {:>9.3}m {:>9.3}m {:>10}", r.sequence_name, r.ate.rmse, r.ate.mean, rtf.map(|v| format!("{v:.3}")).unwrap_or_else(|| "n/a".to_string()));
    }
}
