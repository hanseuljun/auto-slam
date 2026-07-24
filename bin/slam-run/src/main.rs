//! `slam-run`: Stage 2's M0, finishing Stage 1's M9 — "one command runs a
//! sequence and prints the report," extended with the wall-clock timing
//! `plan/STAGE2.md`'s real-time bar (goal 1) is measured against. Runs the
//! full stereo-inertial VIO pipeline (Stage 1 M5, track-loss recovery M6)
//! plus one global bundle-adjustment pass (M8), computes ATE + RPE against
//! groundtruth, and reports a real-time factor: wall-clock spent in the
//! continuous per-frame loop (frontend tracking + windowed backend
//! optimization) divided by the amount of sensor data it processed.
//!
//! Defaults to the **full, un-truncated sequence** (`plan/STAGE4.md` M3).
//! Stage 2 through Stage 4 M0/M1 defaulted to a 600-frame bounded clip
//! instead, because an earlier, since-rolled-back attempt at Stage 2's own
//! milestone ran the un-truncated pipeline and took 30+ minutes wall-clock
//! (`global_bundle_adjustment`'s dense O(n^3) solve scaling with total
//! keyframe count, unbounded). Stage 4 M1 fixed that (`VioParams::
//! max_global_ba_keyframes` bounds global BA's own scope), and M2 confirmed
//! the resulting full-sequence accuracy isn't a regression relative to the
//! bounded clip (cross-validated against Stage 1 M6's independent VO-only
//! baseline) — see `docs/RESULTS.md`'s "Full-sequence results" section.
//! Pass `--frames N` for the old bounded/fast-iteration mode (e.g. `--frames
//! 600` for ~30s of data), still useful for quick tuning turnaround
//! (`decisions/0016`-`0017`'s sweeps used exactly this).
//!
//! Loop closure (`plan/STAGE1.md` M7) is chained in as of `plan/STAGE5.md`
//! M2: every `machine_hall` sequence returns near its own start position
//! (confirmed via groundtruth, `plan/STAGE5.md`'s Finding 2), so this
//! isn't a one-sequence special case. Detection/verification/pose-graph
//! optimization run as a post-processing pass over the trajectory
//! `global_bundle_adjustment` produces (mirroring global BA's own one-shot-
//! pass shape), and the correction is only actually applied if it verifiably
//! brings the trajectory's own start and end closer together — a real,
//! cheap geometric gate, not just "a loop was detected" (`memory/
//! decisions/0021`: applying a *detected* loop's correction blindly
//! regressed `MH_01_easy` during this milestone's own baseline check).
//! `bin/slam-inspect`'s own `VoPipeline`-only demo predates this and is
//! independent of it.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use clap::Parser;
use nalgebra::Vector3;
use slam_core::SE3;

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

    /// Cap the run at this many frames instead of the full-sequence
    /// default — the old bounded/fast-iteration mode (e.g. 600 is ~30s of
    /// data at EuRoC's 20Hz cam0 rate). Omit for the full, un-truncated
    /// sequence.
    #[arg(long)]
    frames: Option<usize>,
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
        if let Some(report) = run_sequence(seq_dir, &cli.out_dir, cli.frames, &run_id, git_commit.as_deref())? {
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

/// How much of a run's leading data the honest, prefix-anchored ATE
/// metric fits its alignment against (`plan/STAGE5.md` goal 1,
/// `memory/decisions/0020`) — matches the duration this pipeline's own
/// bounded/fast-iteration mode (`--frames 600`) already uses, reusing an
/// existing, already-understood magnitude instead of a new one. Measured
/// (not guessed) to be large enough to avoid the small-window lever-arm
/// instability a handful of points has, and small enough that later
/// drift doesn't get the chance to pull the fit.
const ALIGN_PREFIX_SECONDS: f64 = 30.0;

/// Capture a loop-closure keyframe every `LOOP_CLOSURE_CAPTURE_STRIDE`-th
/// VIO keyframe. `plan/STAGE5.md` M2 originally set this to 4 because the
/// dense pose graph's own O(n^3) solve (`optimize_pose_graph`, pre-
/// `plan/STAGE6.md` M3) made capturing at full density (up to 741 nodes
/// on `MH_01_easy`'s full run) cost 590s+ on one sequence alone.
/// `plan/STAGE6.md` M3 replaced that solve with a sparse one (~97ms on a
/// 741-node graph, measured — `memory/decisions/0025`), so M4 re-measured
/// this constant directly rather than assuming 1 (every VIO keyframe, no
/// downsampling) now costs nothing: stride 1 measurably *breaks* the
/// real-time bar on `MH_01_easy` (whole-run factor 1.082) — the pose-
/// graph solve is cheap now, but BoW vocabulary training and place-
/// recognition queries still scale with keyframe count, and at stride 1
/// that's ~4x the descriptors/queries stride 4 had. Stride 2 holds the
/// bar on all 5 sequences (0.822-0.925) with real, substantial
/// gap-closure improvements over stride 4 on most sequences — the
/// measured, not guessed, choice (`memory/decisions/0026`).
const LOOP_CLOSURE_CAPTURE_STRIDE: usize = 2;

fn run_sequence(seq_dir: &Path, out_dir: &Path, frame_cap: Option<usize>, run_id: &str, git_commit: Option<&str>) -> anyhow::Result<Option<slam_eval::TrajectoryReport>> {
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
    let num_frames = frame_cap.map(|n| n.min(full_len)).unwrap_or(full_len);
    if let Some(n) = frame_cap {
        if n < full_len {
            println!("bounded run: {num_frames}/{full_len} frames (omit --frames for the complete sequence)");
        }
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
    // `VioPipeline::new` takes `rig` by value; loop-closure keyframe
    // capture (below) needs its own copy plus the rectification `rig`
    // alone doesn't carry.
    let lc_rig = rig.clone();
    let lc_rect = lc_rig.rectify();
    let mut vio = slam_backend::VioPipeline::new(rig, initial_state, seq.cam0_frames[0].timestamp_ns, gravity_world, params);
    vio.init_map(&left0, &right0);

    let mut loop_closure_time = Duration::ZERO;
    let capture_start = Instant::now();
    let mut loop_keyframes = vec![slam_loopclosure::capture_loop_keyframe(
        &left0,
        &right0,
        slam_loopclosure::KeyframeMeta { keyframe_id: 0, timestamp_ns: seq.cam0_frames[0].timestamp_ns, pose_world_to_cam0: world_to_cam0(lc_rig.t_bs_cam0, SE3::identity()) },
        &lc_rig,
        &lc_rect,
        &slam_loopclosure::CaptureParams::default(),
    )];
    loop_closure_time += capture_start.elapsed();

    let mut prev_timestamp = seq.cam0_frames[0].timestamp_ns;
    let mut lost_frames = 0usize;
    let mut recovered_frames = 0usize;
    let mut vio_keyframe_count = 0usize;
    for i in 1..num_frames {
        let left = seq.load_cam0_image(i)?;
        let right = seq.load_cam1_image(i)?;
        let timestamp = seq.cam0_frames[i].timestamp_ns;
        let imu_since_last: Vec<_> = seq.imu_samples.iter().filter(|s| s.timestamp_ns > prev_timestamp && s.timestamp_ns <= timestamp).cloned().collect();
        prev_timestamp = timestamp;
        match vio.process_frame(&left, &right, timestamp, &imu_since_last) {
            None => {
                // M6's recovery only fails to produce a result on a single
                // genuinely-untrackable frame; the next real frame recovers
                // via IMU propagation. Count and keep going rather than
                // aborting the whole sequence over it.
                lost_frames += 1;
            }
            Some(r) => {
                if r.recovered {
                    recovered_frames += 1;
                }
                if r.is_keyframe {
                    vio_keyframe_count += 1;
                    // Every VIO keyframe (up to 741 on MH_01_easy's full
                    // run, track-loss recovery included) is far denser than
                    // loop-closure keyframe capture needs to be — capturing
                    // at that density measurably broke the real-time bar
                    // this milestone must not regress (`memory/
                    // decisions/0021`: 590s of loop-closure cost alone on
                    // one sequence, both from the O(n) capture cost itself
                    // and the O(n^2) BoW query loop below scaling with it).
                    // `LOOP_CLOSURE_CAPTURE_STRIDE` decouples capture
                    // density from VioPipeline's own keyframe cadence,
                    // landing in the same few-hundred-keyframes-total range
                    // `bin/slam-inspect`'s own demo (raw-frame stride 20)
                    // already proved fast enough.
                    if vio_keyframe_count.is_multiple_of(LOOP_CLOSURE_CAPTURE_STRIDE) {
                        // Reuses the (left, right) pair already loaded for
                        // `vio.process_frame` above — no extra image I/O,
                        // just the extra detection/stereo-matching/
                        // descriptor work itself (measured in
                        // `loop_closure_time`). `keyframe_id` is this
                        // capture's own position within `loop_keyframes`
                        // (not `vio`'s own keyframe index, since the stride
                        // above means they're no longer 1:1) — matched back
                        // up to `vio.all_keyframe_poses()`'s final poses by
                        // *timestamp* after the main loop, not by index.
                        let capture_start = Instant::now();
                        let keyframe_id = loop_keyframes.len();
                        loop_keyframes.push(slam_loopclosure::capture_loop_keyframe(
                            &left,
                            &right,
                            slam_loopclosure::KeyframeMeta { keyframe_id, timestamp_ns: timestamp, pose_world_to_cam0: world_to_cam0(lc_rig.t_bs_cam0, r.pose_world_to_body) },
                            &lc_rig,
                            &lc_rect,
                            &slam_loopclosure::CaptureParams::default(),
                        ));
                        loop_closure_time += capture_start.elapsed();
                    }
                }
            }
        }
    }
    let data_seconds = (prev_timestamp - seq.cam0_frames[0].timestamp_ns) as f64 * 1e-9;

    let before_ba = vio.all_keyframe_poses().len();
    let ba_wall_start = Instant::now();
    vio.global_bundle_adjustment();
    let ba_wall_elapsed = ba_wall_start.elapsed();
    let trajectory = vio.all_keyframe_poses();

    // Loop closure (`plan/STAGE5.md` M2), as a post-processing pass over
    // the same final trajectory `global_bundle_adjustment` just produced —
    // never mutates `vio` itself, only builds a corrected copy of the pose
    // list used for reporting/CSV output below. `loop_keyframes` is sparser
    // than `trajectory` (`LOOP_CLOSURE_CAPTURE_STRIDE`), so each capture's
    // final pose is looked up by *timestamp*, not index.
    let lc_start = Instant::now();
    let trajectory_by_ts: std::collections::HashMap<u64, SE3> = trajectory.iter().copied().collect();
    for kf in &mut loop_keyframes {
        if let Some(&final_pose_world_to_body) = trajectory_by_ts.get(&kf.timestamp_ns) {
            rebase_loop_keyframe_landmarks(kf, world_to_cam0(lc_rig.t_bs_cam0, final_pose_world_to_body));
        }
    }
    let (final_poses, loop_closure_outcome) = match find_and_apply_loop_closure(&loop_keyframes, &trajectory, lc_rig.t_bs_cam0) {
        Some((poses, outcome)) => (poses, Some(outcome)),
        None => (trajectory.iter().map(|(_, p)| *p).collect(), None),
    };
    loop_closure_time += lc_start.elapsed();

    let mut timestamps = Vec::new();
    let mut estimated = Vec::new();
    let mut groundtruth = Vec::new();
    for ((t, _), corrected_pose) in trajectory.iter().zip(final_poses.iter()) {
        if let Some(pose) = gt.interpolate(*t) {
            timestamps.push(*t);
            estimated.push(corrected_pose.inverse().translation);
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
        loop_closure_seconds: loop_closure_time.as_secs_f64(),
        data_seconds,
    };

    // How many leading keyframes fall within the first ALIGN_PREFIX_SECONDS
    // of data — the alignment window `ate_prefix_aligned` fits against
    // (`plan/STAGE5.md` M0/M1, `memory/decisions/0020`). Time-based, not a
    // fixed keyframe count: track-loss recovery (`plan/STAGE4.md` M2)
    // forces off-stride keyframes, so keyframe count per second of data
    // isn't the same across sequences or runs.
    let align_prefix_len = match timestamps.first() {
        Some(&t0) => timestamps.iter().take_while(|&&t| (t - t0) as f64 * 1e-9 <= ALIGN_PREFIX_SECONDS).count(),
        None => 0,
    };

    let Some(report) = slam_eval::build_report(&name, &estimated, &groundtruth, RPE_DELTAS, Some(timing), Some(align_prefix_len)) else {
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
            full_sequence: frame_cap.is_none(),
            frame_cap: frame_cap.unwrap_or(full_len),
        },
        ate: report.ate,
        ate_prefix_aligned: report.ate_prefix_aligned,
        rpe: report.rpe.clone(),
        timing: report.timing,
    };
    slam_eval::write_run_meta(run_dir.join("meta.json"), &run_meta)?;

    println!(
        "{num_frames} frames ({data_seconds:.1}s of data), {before_ba} keyframes ({lost_frames} unrecoverable single frames, {recovered_frames} track-loss recoveries), ATE rmse={:.3}m mean={:.3}m median={:.3}m std={:.3}m max={:.3}m",
        report.ate.rmse, report.ate.mean, report.ate.median, report.ate.std, report.ate.max
    );
    if let Some(p) = report.ate_prefix_aligned {
        println!(
            "  ATE (prefix-aligned, first {ALIGN_PREFIX_SECONDS:.0}s anchors the fit, `plan/STAGE5.md` goal 1): rmse={:.3}m mean={:.3}m median={:.3}m max={:.3}m",
            p.rmse, p.mean, p.median, p.max
        );
    }
    for rpe in &report.rpe {
        println!("  RPE (delta={} keyframes): rmse={:.3}m mean={:.3}m max={:.3}m over {} pairs", rpe.delta, rpe.rmse, rpe.mean, rpe.max, rpe.num_pairs);
    }
    match &loop_closure_outcome {
        Some(o) if o.applied => println!(
            "  loop closure: keyframe {} <-> {} ({} inliers) applied — start/end gap {:.3}m -> {:.3}m",
            o.current_id, o.candidate_id, o.num_inliers, o.gap_before, o.gap_after
        ),
        Some(o) => println!(
            "  loop closure: keyframe {} <-> {} ({} inliers) detected but NOT applied — didn't verifiably close the loop (start/end gap {:.3}m -> {:.3}m)",
            o.current_id, o.candidate_id, o.num_inliers, o.gap_before, o.gap_after
        ),
        None => println!("  loop closure: no verified loop detected"),
    }
    println!(
        "  timing: vision={:.2}s optimization={:.2}s global_ba={:.2}s loop_closure={:.2}s (data={:.1}s) -> VIO-loop factor={:.3} whole-run factor={:.3}",
        timing.vision_seconds,
        timing.optimization_seconds,
        timing.global_ba_seconds,
        timing.loop_closure_seconds,
        timing.data_seconds,
        timing.real_time_factor(),
        timing.whole_run_factor()
    );
    println!("wrote trajectory CSV to {}", csv_path.display());
    println!("wrote this run's history entry to {} (trajectory.csv + meta.json)", run_dir.display());
    println!();

    Ok(Some(report))
}

/// `pose_world_to_body.compose`'d with the body<->cam0 extrinsic to get
/// the `pose_world_to_cam0` convention `slam_loopclosure`'s whole API
/// (designed against `VoPipeline`, which operates directly in cam0's
/// frame) expects. `t_bs_cam0` is EuRoC's own `T_BS`: cam0 -> body, so
/// `t_bs_cam0.inverse()` is body -> cam0; composed after
/// `pose_world_to_body` (world -> body) gives world -> cam0.
fn world_to_cam0(t_bs_cam0: SE3, pose_world_to_body: SE3) -> SE3 {
    t_bs_cam0.inverse().compose(&pose_world_to_body)
}

/// Refreshes a captured `LoopKeyframe`'s `landmarks_world` (and its own
/// `pose_world_to_cam0`) to match a pose that may have moved since capture
/// — needed because `global_bundle_adjustment` (`plan/STAGE4.md` M1) can
/// still adjust a keyframe's pose *after* its loop-closure keyframe was
/// captured earlier in the same run, for any keyframe within its own
/// `max_global_ba_keyframes` scope. Landmark positions are recovered by
/// re-projecting through the *old* pose back into cam0's own frame (camera-
/// frame-invariant, unaffected by which world-frame pose is current) and
/// then through the *new* pose — a rigid re-transform, not a re-detection.
fn rebase_loop_keyframe_landmarks(kf: &mut slam_loopclosure::LoopKeyframe, new_pose_world_to_cam0: SE3) {
    let old_pose_world_to_cam0 = kf.pose_world_to_cam0;
    let new_pose_cam0_to_world = new_pose_world_to_cam0.inverse();
    for landmark in &mut kf.landmarks_world {
        let point_cam0 = old_pose_world_to_cam0.transform(landmark);
        *landmark = new_pose_cam0_to_world.transform(&point_cam0);
    }
    kf.pose_world_to_cam0 = new_pose_world_to_cam0;
}

/// What `find_and_apply_loop_closure` found and did, reported so a human
/// reading `bin/slam-run`'s output can tell "was there a loop to close"
/// apart from "did closing it verifiably help" — two different questions,
/// `plan/STAGE5.md` M2's own baseline check found (a detected, verified,
/// even high-inlier-count loop still regressed `MH_01_easy` when applied
/// unconditionally — see `memory/decisions/0021`).
struct LoopClosureOutcome {
    /// `loop_keyframes`' own (sparse, `LOOP_CLOSURE_CAPTURE_STRIDE`-
    /// downsampled) indices, not raw frame or dense-trajectory indices.
    current_id: usize,
    candidate_id: usize,
    num_inliers: usize,
    /// Distance between the trajectory's own first and last pose, in this
    /// pipeline's own (ungrounded) world frame — not aligned to
    /// groundtruth. The direct, human-checkable geometric claim `plan/
    /// STAGE5.md` M3 asks for: a sequence whose *groundtruth* start and
    /// end are close (`plan/STAGE5.md`'s Finding 2, true of every
    /// `machine_hall` sequence) should, after a *correctly* closed loop,
    /// also have its own *estimated* start and end close — independent of
    /// whatever `ate`/`ate_prefix_aligned` numbers say.
    gap_before: f64,
    gap_after: f64,
    /// `true` only if `gap_after < gap_before` — the correction is
    /// discarded (not just flagged) when it isn't, so `bin/slam-run`
    /// never reports a "loop-closed" trajectory whose own start and end
    /// are verifiably farther apart than before the "fix."
    applied: bool,
}

/// Detects the single best loop-closure candidate across `loop_keyframes`
/// (preferring the largest keyframe-id gap — the most direct proxy for
/// "connects this sequence's own start back to its own end," `plan/
/// STAGE5.md` goal 2 — over the previous naive "most descriptor-match
/// inliers, wherever it happens to be" choice `bin/slam-inspect`'s older
/// demo used), runs pose-graph optimization with it, and returns the
/// corrected `pose_world_to_body` trajectory *only if* it verifiably
/// brings the trajectory's own start and end closer together. Returns
/// `None` if no loop clears geometric verification at all (nothing to
/// report); otherwise `Some((poses, outcome))` where `poses` is the
/// corrected trajectory when `outcome.applied`, or an unchanged copy of
/// `trajectory`'s own poses otherwise.
///
/// The pose graph itself is built over `loop_keyframes` (the sparse,
/// `LOOP_CLOSURE_CAPTURE_STRIDE`-downsampled set), not every dense
/// `trajectory` keyframe — `optimize_pose_graph`'s own dense O(n^3) LU
/// solve (`crates/slam-loopclosure/src/pose_graph.rs`, one 6-DoF-per-node
/// block per pose, no sparsity exploited) makes that the exact scaling
/// mistake `plan/STAGE4.md` M1 already fixed for `global_bundle_
/// adjustment` — confirmed the hard way this milestone (`memory/
/// decisions/0021`): running it over all 741 of `MH_01_easy`'s dense
/// keyframes didn't finish in 10+ minutes. The sparse graph's correction
/// is then propagated onto the dense trajectory by nearest-timestamp
/// rigid delta — simple, not perfectly smooth at segment boundaries, but
/// `LOOP_CLOSURE_CAPTURE_STRIDE`'s own density keeps "nearest" close in
/// both time and space.
fn find_and_apply_loop_closure(loop_keyframes: &[slam_loopclosure::LoopKeyframe], trajectory: &[(u64, SE3)], t_bs_cam0: SE3) -> Option<(Vec<SE3>, LoopClosureOutcome)> {
    let all_descriptors: Vec<_> = loop_keyframes.iter().flat_map(|k| k.descriptors.iter().copied()).collect();
    if all_descriptors.len() < 500 {
        return None;
    }
    let vocab = slam_loopclosure::Vocabulary::train(&all_descriptors, 300, 6, 17);

    let mut db = slam_loopclosure::KeyframeDatabase::new();
    for kf in loop_keyframes {
        db.insert(kf.keyframe_id, vocab.compute_bow(&kf.descriptors));
    }

    let min_id_gap = 30;
    let mut best: Option<(usize, usize, slam_loopclosure::VerifiedLoop)> = None;
    for kf in loop_keyframes {
        let query_bow = vocab.compute_bow(&kf.descriptors);
        let Some((candidate_id, _)) = db.query(kf.keyframe_id, &query_bow, min_id_gap, 0.3) else {
            continue;
        };
        let candidate = &loop_keyframes[candidate_id];
        let Some(verified) = slam_loopclosure::verify_loop_candidate(&kf.normalized, &kf.descriptors, &candidate.descriptors, &candidate.landmarks_world, &slam_loopclosure::GeometricVerificationParams::default()) else {
            continue;
        };
        let gap = kf.keyframe_id.abs_diff(candidate_id);
        let is_better = match &best {
            None => true,
            Some((prev_current, prev_candidate, _)) => gap > prev_current.abs_diff(*prev_candidate),
        };
        if is_better {
            best = Some((kf.keyframe_id, candidate_id, verified));
        }
    }
    let (current_id, candidate_id, verified) = best?;

    let sparse_poses: Vec<SE3> = loop_keyframes.iter().map(|k| k.pose_world_to_cam0).collect();
    let mut edges = Vec::with_capacity(sparse_poses.len());
    for i in 0..sparse_poses.len() - 1 {
        edges.push(slam_loopclosure::PoseGraphEdge { i, j: i + 1, relative_pose: sparse_poses[i + 1].compose(&sparse_poses[i].inverse()), weight: 1.0 });
    }
    let loop_relative_pose = verified.relative_pose.compose(&sparse_poses[candidate_id].inverse());
    edges.push(slam_loopclosure::PoseGraphEdge { i: candidate_id, j: current_id, relative_pose: loop_relative_pose, weight: 5000.0 });

    let mut sparse_corrected = sparse_poses.clone();
    slam_loopclosure::optimize_pose_graph(&mut sparse_corrected, &edges, 0, 50);

    // Per-sparse-node correction delta: `delta.compose(&original) ==
    // corrected`, so `delta = corrected.compose(&original.inverse())`.
    // Propagated onto the dense trajectory by *smoothly interpolating*
    // between the two bracketing sparse deltas (SE3 log-space lerp, same
    // primitive `optimize_pose_graph`'s own `edge_residual` uses) — a
    // discrete nearest-sparse-node assignment was tried first and measurably
    // injected a real discontinuity at every stride boundary (RPE delta=1
    // rmse jumped ~7x, 0.162m -> 1.104m, on `MH_01_easy` — a correction
    // shouldn't visibly degrade local frame-to-frame consistency to fix
    // global drift; `memory/decisions/0021`).
    let sparse_deltas: Vec<SE3> = sparse_poses.iter().zip(sparse_corrected.iter()).map(|(orig, corr)| corr.compose(&orig.inverse())).collect();
    let sparse_timestamps: Vec<u64> = loop_keyframes.iter().map(|k| k.timestamp_ns).collect();
    let interpolated_delta = |ts: u64| -> SE3 {
        let pos = sparse_timestamps.partition_point(|&t| t < ts);
        if pos == 0 {
            return sparse_deltas[0];
        }
        if pos == sparse_timestamps.len() {
            return sparse_deltas[sparse_deltas.len() - 1];
        }
        if sparse_timestamps[pos] == ts {
            return sparse_deltas[pos];
        }
        let (lo, hi) = (pos - 1, pos);
        let span = (sparse_timestamps[hi] - sparse_timestamps[lo]) as f64;
        let alpha = if span > 0.0 { (ts - sparse_timestamps[lo]) as f64 / span } else { 0.0 };
        let relative = sparse_deltas[lo].inverse().compose(&sparse_deltas[hi]);
        sparse_deltas[lo].compose(&SE3::exp(relative.log() * alpha))
    };

    let dense_poses: Vec<SE3> = trajectory.iter().map(|(_, p)| world_to_cam0(t_bs_cam0, *p)).collect();
    let dense_corrected: Vec<SE3> = trajectory.iter().zip(dense_poses.iter()).map(|((t, _), p)| interpolated_delta(*t).compose(p)).collect();

    let start_end_gap = |ps: &[SE3]| -> f64 { (ps[0].inverse().translation - ps[ps.len() - 1].inverse().translation).norm() };
    let gap_before = start_end_gap(&dense_poses);
    let gap_after = start_end_gap(&dense_corrected);
    let applied = gap_after < gap_before;

    let final_poses_cam0 = if applied { &dense_corrected } else { &dense_poses };
    let final_poses_body: Vec<SE3> = final_poses_cam0.iter().map(|p| t_bs_cam0.compose(p)).collect();

    Some((final_poses_body, LoopClosureOutcome { current_id, candidate_id, num_inliers: verified.num_inliers, gap_before, gap_after, applied }))
}

fn print_summary_table(reports: &[slam_eval::TrajectoryReport]) {
    println!("==== summary ====");
    println!("{:<20} {:>10} {:>10} {:>12} {:>12}", "sequence", "ATE rmse", "ATE mean", "VIO-loop RT", "whole-run RT");
    for r in reports {
        let vio_rtf = r.timing.map(|t| t.real_time_factor());
        let whole_rtf = r.timing.map(|t| t.whole_run_factor());
        println!(
            "{:<20} {:>9.3}m {:>9.3}m {:>12} {:>12}",
            r.sequence_name,
            r.ate.rmse,
            r.ate.mean,
            vio_rtf.map(|v| format!("{v:.3}")).unwrap_or_else(|| "n/a".to_string()),
            whole_rtf.map(|v| format!("{v:.3}")).unwrap_or_else(|| "n/a".to_string())
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slam_core::SO3;

    fn dummy_descriptor() -> slam_vision::Descriptor {
        slam_vision::Descriptor([0, 0, 0, 0])
    }

    #[test]
    fn world_to_cam0_matches_manual_composition() {
        let t_bs_cam0 = SE3::new(SO3::exp(Vector3::new(0.1, -0.05, 0.02)), Vector3::new(0.01, -0.03, 0.02));
        let pose_world_to_body = SE3::new(SO3::exp(Vector3::new(0.0, 0.3, 0.0)), Vector3::new(1.0, 2.0, 3.0));

        let result = world_to_cam0(t_bs_cam0, pose_world_to_body);

        // A point known in cam0's own frame should round-trip through
        // world coordinates and back to the same point.
        let p_cam0 = Vector3::new(0.5, -0.2, 4.0);
        let p_body = t_bs_cam0.transform(&p_cam0);
        let p_world = pose_world_to_body.inverse().transform(&p_body);
        assert_relative_eq!(result.transform(&p_world), p_cam0, epsilon = 1e-9);
    }

    #[test]
    fn world_to_cam0_of_identity_body_pose_is_just_the_inverse_extrinsic() {
        let t_bs_cam0 = SE3::new(SO3::exp(Vector3::new(0.0, 0.0, 0.2)), Vector3::new(0.05, 0.0, 0.0));
        let result = world_to_cam0(t_bs_cam0, SE3::identity());
        assert_relative_eq!(result.matrix(), t_bs_cam0.inverse().matrix(), epsilon = 1e-9);
    }

    fn make_loop_keyframe(pose_world_to_cam0: SE3, landmarks_world: Vec<Vector3<f64>>) -> slam_loopclosure::LoopKeyframe {
        let n = landmarks_world.len();
        slam_loopclosure::LoopKeyframe {
            keyframe_id: 0,
            timestamp_ns: 0,
            pose_world_to_cam0,
            landmarks_world,
            normalized: vec![nalgebra::Vector2::zeros(); n],
            descriptors: vec![dummy_descriptor(); n],
        }
    }

    #[test]
    fn rebase_to_the_same_pose_is_a_no_op() {
        let pose = SE3::new(SO3::exp(Vector3::new(0.1, 0.0, 0.0)), Vector3::new(1.0, 0.0, 0.0));
        let landmarks = vec![Vector3::new(1.0, 2.0, 5.0), Vector3::new(-1.0, 0.5, 3.0)];
        let mut kf = make_loop_keyframe(pose, landmarks.clone());

        rebase_loop_keyframe_landmarks(&mut kf, pose);

        for (a, b) in kf.landmarks_world.iter().zip(landmarks.iter()) {
            assert_relative_eq!(a, b, epsilon = 1e-9);
        }
    }

    #[test]
    fn rebase_then_rebase_back_recovers_original_landmarks() {
        let old_pose = SE3::new(SO3::exp(Vector3::new(0.1, -0.2, 0.05)), Vector3::new(1.0, 2.0, 0.0));
        let new_pose = SE3::new(SO3::exp(Vector3::new(-0.3, 0.1, 0.4)), Vector3::new(-5.0, 1.0, 3.0));
        let landmarks = vec![Vector3::new(1.0, 2.0, 5.0), Vector3::new(-1.0, 0.5, 3.0), Vector3::new(0.2, -0.4, 6.0)];
        let mut kf = make_loop_keyframe(old_pose, landmarks.clone());

        rebase_loop_keyframe_landmarks(&mut kf, new_pose);
        assert_relative_eq!(kf.pose_world_to_cam0.matrix(), new_pose.matrix(), epsilon = 1e-9);
        // Landmarks must have actually moved (not a no-op) — otherwise
        // the next assertion (round-tripping back to the originals)
        // would pass trivially even if rebasing did nothing.
        assert!((kf.landmarks_world[0] - landmarks[0]).norm() > 1e-6);

        rebase_loop_keyframe_landmarks(&mut kf, old_pose);
        for (a, b) in kf.landmarks_world.iter().zip(landmarks.iter()) {
            assert_relative_eq!(a, b, epsilon = 1e-9);
        }
    }
}
