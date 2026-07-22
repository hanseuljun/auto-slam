//! `slam-inspect`: the running, human-readable test app for this pipeline.
//! Each milestone in `plan/STAGE1.md` extends this app's output rather than
//! spawning a separate demo. M0: dataset load stats + calibration dump +
//! raw groundtruth trajectory sanity check.

use std::path::{Path, PathBuf};

use clap::Parser;

#[derive(Parser)]
#[command(
    name = "slam-inspect",
    about = "Inspect EuRoC sequences and this pipeline's intermediate state"
)]
struct Cli {
    /// Sequence directories to inspect (each containing an `mav0/`
    /// subdirectory). Defaults to every sequence under `data/machine_hall`.
    #[arg(value_name = "SEQUENCE_DIR")]
    sequences: Vec<PathBuf>,
}

fn default_sequences() -> Vec<PathBuf> {
    let root = PathBuf::from("data/machine_hall");
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(&root)
        .map(|entries| {
            entries
                .filter_map(|e| e.ok())
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .collect()
        })
        .unwrap_or_default();
    dirs.sort();
    dirs
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let sequences = if cli.sequences.is_empty() {
        default_sequences()
    } else {
        cli.sequences
    };

    if sequences.is_empty() {
        anyhow::bail!(
            "no sequences found under data/machine_hall and none given on the command line"
        );
    }

    for seq_dir in sequences {
        inspect_sequence(&seq_dir)?;
    }
    Ok(())
}

fn inspect_sequence(seq_dir: &Path) -> anyhow::Result<()> {
    let name = seq_dir
        .file_name()
        .map(|n| n.to_string_lossy().to_string())
        .unwrap_or_else(|| seq_dir.display().to_string());
    println!("==== {name} ====");

    let mav0 = seq_dir.join("mav0");
    let seq = slam_dataset::EuRocSequence::load(&mav0)?;
    print_calibration(&seq.calibration);
    print_stereo_geometry(&seq.calibration);

    println!(
        "cam0 frames: {}    cam1 frames: {}    imu samples: {}",
        seq.cam0_frames.len(),
        seq.cam1_frames.len(),
        seq.imu_samples.len()
    );

    if let (Some(first), Some(last)) = (seq.imu_samples.first(), seq.imu_samples.last()) {
        println!(
            "imu span: {:.2}s",
            (last.timestamp_ns - first.timestamp_ns) as f64 * 1e-9
        );
    }

    let events: Vec<(u64, slam_dataset::Event)> = seq.events().collect();
    let time_ordered = events.windows(2).all(|w| w[0].0 <= w[1].0);
    println!(
        "merged event stream: {} events, time-ordered = {time_ordered}",
        events.len()
    );

    print_vision_frontend(&seq);

    let gt_path = mav0.join("state_groundtruth_estimate0/data.csv");
    let groundtruth = if gt_path.exists() {
        let traj = slam_eval::GroundTruthTrajectory::load(&gt_path)?;
        print_groundtruth_summary(&traj);
        Some(traj)
    } else {
        println!("no groundtruth found at {}", gt_path.display());
        None
    };

    let vo_keyframes = print_stereo_vo(&seq, groundtruth.as_ref());
    print_imu_initialization(&seq, &vo_keyframes);
    print_stereo_inertial_vio(&seq, groundtruth.as_ref());
    if name == "MH_05_difficult" {
        // The only MH_* sequence with a real, documented loop (it
        // revisits its own start position at the very end) — see
        // memory/notes/dataset-quirks.md. A full-sequence VO run +
        // vocabulary training is real work, so this isn't run for every
        // sequence.
        print_loop_closure(&seq, groundtruth.as_ref());
    }

    println!();
    Ok(())
}

fn print_calibration(cal: &slam_dataset::Calibration) {
    println!(
        "cam0: intrinsics(fu,fv,cu,cv)={:?} dist={:?} res={:?}",
        cal.cam0.intrinsics, cal.cam0.distortion_coefficients, cal.cam0.resolution
    );
    println!(
        "cam1: intrinsics(fu,fv,cu,cv)={:?} dist={:?} res={:?}",
        cal.cam1.intrinsics, cal.cam1.distortion_coefficients, cal.cam1.resolution
    );
    println!(
        "imu0: rate={}Hz gyro_noise={:.4e} gyro_rw={:.4e} accel_noise={:.4e} accel_rw={:.4e}",
        cal.imu0.rate_hz,
        cal.imu0.gyroscope_noise_density,
        cal.imu0.gyroscope_random_walk,
        cal.imu0.accelerometer_noise_density,
        cal.imu0.accelerometer_random_walk
    );
}

/// Demonstrates M1 (`slam-geometry`): stereo rectification stats plus a
/// synthetic point projected through the real calibration and recovered via
/// triangulation, as a live round-trip sanity check.
fn print_stereo_geometry(cal: &slam_dataset::Calibration) {
    let rig = slam_geometry::StereoRig {
        t_bs_cam0: slam_core::SE3::from_matrix(&cal.cam0.t_bs),
        t_bs_cam1: slam_core::SE3::from_matrix(&cal.cam1.t_bs),
        cam0: slam_geometry::PinholeCamera::new(cal.cam0.intrinsics, cal.cam0.distortion_coefficients),
        cam1: slam_geometry::PinholeCamera::new(cal.cam1.intrinsics, cal.cam1.distortion_coefficients),
    };
    let rect = rig.rectify();
    println!(
        "stereo rectification: baseline={:.4}m rectified_intrinsics(fu,fv,cu,cv)={:?}",
        rect.baseline, rect.rectified_intrinsics
    );

    let true_point_cam0 = nalgebra::Vector3::new(0.1, -0.05, 3.0);
    let t10 = rig.relative_pose_cam1_from_cam0();
    let true_point_cam1 = t10.transform(&true_point_cam0);
    let n0 = rig.cam0.unproject_to_normalized(rig.cam0.project(true_point_cam0));
    let n1 = rig.cam1.unproject_to_normalized(rig.cam1.project(true_point_cam1));
    let observations = [(slam_core::SE3::identity(), n0), (t10, n1)];

    if let Some(linear) = slam_geometry::triangulate_linear(&observations) {
        let refined = slam_geometry::triangulate_refine(&observations, linear, 5);
        let error_mm = (refined - true_point_cam0).norm() * 1000.0;
        println!(
            "triangulation round-trip check: synthetic point at {:.2}m depth, recovered error = {:.6}mm",
            true_point_cam0.z, error_mm
        );
    }
}

/// Demonstrates M2 (`slam-vision`): grid-distributed FAST detection on the
/// first frame, then pyramidal LK tracking survival over a handful of
/// consecutive real frames.
fn print_vision_frontend(seq: &slam_dataset::EuRocSequence) {
    const NUM_FRAMES: usize = 5;
    let num_frames = NUM_FRAMES.min(seq.cam0_frames.len());
    if num_frames < 2 {
        println!("vision frontend: not enough frames to demonstrate tracking");
        return;
    }

    let frames: Vec<image::GrayImage> = (0..num_frames)
        .map(|i| seq.load_cam0_image(i))
        .collect::<anyhow::Result<_>>()
        .expect("decode frames for vision frontend demo");

    let keypoints = slam_vision::detect_grid(&frames[0], 20, 40, 3);
    let pyramids: Vec<slam_vision::ImagePyramid> =
        frames.iter().map(|f| slam_vision::ImagePyramid::build(f, 4)).collect();
    let params = slam_vision::LkParams::default();

    let initial_count = keypoints.len();
    let mut positions: Vec<(f32, f32)> = keypoints.iter().map(|k| (k.x, k.y)).collect();
    for i in 1..num_frames {
        let results = slam_vision::track_pyramid(&pyramids[i - 1], &pyramids[i], &positions, &params);
        positions = results
            .into_iter()
            .filter(|r| r.found && r.x >= 0.0 && r.y >= 0.0 && r.x < frames[i].width() as f32 && r.y < frames[i].height() as f32)
            .map(|r| (r.x, r.y))
            .collect();
    }

    println!(
        "vision frontend: {initial_count} grid-FAST keypoints on frame 0, {} survived LK tracking across {num_frames} frames ({:.0}%)",
        positions.len(),
        100.0 * positions.len() as f64 / initial_count.max(1) as f64
    );
}

/// Demonstrates M3 (`slam-frontend`): stereo-only (no IMU, no backend) VO
/// over a real clip, aligned onto ground truth via Umeyama, reported as
/// ATE. This is the first end-to-end accuracy checkpoint from
/// `plan/STAGE1.md` — not the SOTA VIO bar (that needs M4/M5 IMU fusion
/// and backend optimization), just proof the frontend is geometrically
/// sane.
fn print_stereo_vo(seq: &slam_dataset::EuRocSequence, groundtruth: Option<&slam_eval::GroundTruthTrajectory>) -> Vec<slam_frontend::VoKeyframe> {
    const NUM_FRAMES: usize = 150;
    let num_frames = NUM_FRAMES.min(seq.cam0_frames.len());
    if num_frames < 2 {
        println!("stereo VO: not enough frames to demonstrate");
        return Vec::new();
    }

    let cal = &seq.calibration;
    let rig = slam_geometry::StereoRig {
        t_bs_cam0: slam_core::SE3::from_matrix(&cal.cam0.t_bs),
        t_bs_cam1: slam_core::SE3::from_matrix(&cal.cam1.t_bs),
        cam0: slam_geometry::PinholeCamera::new(cal.cam0.intrinsics, cal.cam0.distortion_coefficients),
        cam1: slam_geometry::PinholeCamera::new(cal.cam1.intrinsics, cal.cam1.distortion_coefficients),
    };
    let mut vo = slam_frontend::VoPipeline::new(rig, slam_frontend::VoParams::default());

    let left0 = seq.load_cam0_image(0).expect("decode frame 0");
    let right0 = seq.load_cam1_image(0).expect("decode frame 0");
    vo.init(&left0, &right0);

    let mut keyframes = vec![slam_frontend::VoKeyframe {
        timestamp_ns: seq.cam0_frames[0].timestamp_ns,
        pose_world_to_cam0: slam_core::SE3::identity(),
    }];
    let mut lost_at = None;
    for i in 1..num_frames {
        let left = seq.load_cam0_image(i).expect("decode frame");
        let right = seq.load_cam1_image(i).expect("decode frame");
        match vo.process_frame(&left, &right) {
            Some(result) => keyframes.push(slam_frontend::VoKeyframe {
                timestamp_ns: seq.cam0_frames[i].timestamp_ns,
                pose_world_to_cam0: result.pose_world_to_cam0,
            }),
            None => {
                lost_at = Some(i);
                break;
            }
        }
    }

    print!(
        "stereo VO: {} landmarks initialized, tracked {}/{num_frames} frames{}",
        vo.num_landmarks(),
        keyframes.len(),
        lost_at.map(|i| format!(" (lost at frame {i})")).unwrap_or_default()
    );

    match groundtruth {
        Some(gt) => {
            let mut aligned_estimate = Vec::new();
            let mut aligned_groundtruth = Vec::new();
            for kf in &keyframes {
                if let Some(pose) = gt.interpolate(kf.timestamp_ns) {
                    aligned_estimate.push(kf.pose_world_to_cam0.inverse().translation);
                    aligned_groundtruth.push(pose.position);
                }
            }
            match slam_eval::compute_ate(&aligned_estimate, &aligned_groundtruth) {
                Some(stats) => println!(
                    ", ATE (Sim3-aligned, VO-only, no IMU/backend/loop-closure) over {} poses: rmse={:.3}m mean={:.3}m median={:.3}m max={:.3}m",
                    stats.num_points, stats.rmse, stats.mean, stats.median, stats.max
                ),
                None => println!(", not enough groundtruth-covered poses to compute ATE"),
            }
        }
        None => println!(", no groundtruth available for ATE"),
    }

    keyframes
}

/// Demonstrates M4 (`slam-imu` + `slam-frontend::vi_init`): static
/// gravity/gyro-bias initialization from a stationary IMU window if one
/// exists, and the dynamic (moving-start) vision-IMU alignment initializer
/// otherwise/always, reusing the VO keyframes from `print_stereo_vo`.
fn print_imu_initialization(seq: &slam_dataset::EuRocSequence, vo_keyframes: &[slam_frontend::VoKeyframe]) {
    let all_gyro: Vec<nalgebra::Vector3<f64>> = seq.imu_samples.iter().map(|s| s.gyro).collect();
    match slam_imu::find_stationary_window(&all_gyro, 200, 0.10) {
        Some(start) => {
            let gyro = &all_gyro[start..start + 200];
            let accel: Vec<nalgebra::Vector3<f64>> = seq.imu_samples[start..start + 200].iter().map(|s| s.accel).collect();
            match slam_imu::static_initialize(gyro, &accel) {
                Some(r) => println!(
                    "static IMU init: stationary window at sample {start} ({:.1}s in), gravity_magnitude={:.3} gyro_bias_norm={:.4}",
                    start as f64 / 200.0,
                    r.gravity_magnitude,
                    r.gyro_bias.norm()
                ),
                None => println!("static IMU init: stationary window found but initialization failed"),
            }
        }
        None => println!("static IMU init: no stationary window found in this sequence (expected for MH_04/05)"),
    }

    let stride = 10; // ~0.5s at 20Hz
    let dynamic_keyframes: Vec<slam_frontend::VoKeyframe> = vo_keyframes.iter().step_by(stride).copied().collect();
    if dynamic_keyframes.len() < 4 {
        println!("dynamic VI init: not enough keyframes to demonstrate");
        return;
    }
    let t_bs_cam0 = slam_core::SE3::from_matrix(&seq.calibration.cam0.t_bs);
    match slam_frontend::dynamic_initialize(&dynamic_keyframes, &seq.imu_samples, &t_bs_cam0) {
        Some(r) => println!(
            "dynamic VI init: {} keyframes ({:.1}s), gravity_magnitude={:.3} gyro_bias_norm={:.4}",
            dynamic_keyframes.len(),
            (dynamic_keyframes.last().unwrap().timestamp_ns - dynamic_keyframes[0].timestamp_ns) as f64 * 1e-9,
            r.gravity_world.norm(),
            r.gyro_bias.norm()
        ),
        None => println!("dynamic VI init: did not converge"),
    }
}

/// Demonstrates M5 (`slam-optim` + `slam-backend`): the sliding-window
/// stereo-inertial VIO pipeline (reprojection + IMU factors, LM with
/// Schur-complement landmark elimination) over a real clip, reported the
/// same way as the M3 stereo-VO-only section for direct comparison.
fn print_stereo_inertial_vio(seq: &slam_dataset::EuRocSequence, groundtruth: Option<&slam_eval::GroundTruthTrajectory>) {
    // Long enough that several keyframes slide past the default 8-keyframe
    // window (decisions/0007) into retained history, so the M8 global-BA
    // pass below actually has more than the window's own keyframes to work
    // with (see slam-backend's global_bundle_adjustment_does_not_worsen_ate
    // checkpoint test for the same reasoning).
    const NUM_FRAMES: usize = 150;
    let num_frames = NUM_FRAMES.min(seq.cam0_frames.len());
    if num_frames < 2 {
        println!("stereo-inertial VIO: not enough frames to demonstrate");
        return;
    }

    let all_gyro: Vec<nalgebra::Vector3<f64>> = seq.imu_samples.iter().map(|s| s.gyro).collect();
    let Some(start) = slam_imu::find_stationary_window(&all_gyro, 200, 0.10) else {
        println!("stereo-inertial VIO: no stationary window to bootstrap from, skipping");
        return;
    };
    let accel: Vec<nalgebra::Vector3<f64>> = seq.imu_samples[start..start + 200].iter().map(|s| s.accel).collect();
    let Some(static_init) = slam_imu::static_initialize(&all_gyro[start..start + 200], &accel) else {
        println!("stereo-inertial VIO: static init failed, skipping");
        return;
    };

    let cal = &seq.calibration;
    let rig = slam_geometry::StereoRig {
        t_bs_cam0: slam_core::SE3::from_matrix(&cal.cam0.t_bs),
        t_bs_cam1: slam_core::SE3::from_matrix(&cal.cam1.t_bs),
        cam0: slam_geometry::PinholeCamera::new(cal.cam0.intrinsics, cal.cam0.distortion_coefficients),
        cam1: slam_geometry::PinholeCamera::new(cal.cam1.intrinsics, cal.cam1.distortion_coefficients),
    };
    let initial_state = slam_optim::KeyframeState::new(
        slam_core::SE3::identity(),
        nalgebra::Vector3::zeros(),
        static_init.gyro_bias,
        nalgebra::Vector3::zeros(),
    );
    let gravity_world = static_init.gravity_direction_body * static_init.gravity_magnitude;

    let left0 = seq.load_cam0_image(0).expect("decode frame 0");
    let right0 = seq.load_cam1_image(0).expect("decode frame 0");
    let params = slam_backend::VioParams {
        solver: slam_optim::SolverConfig { max_iterations: 6, ..slam_optim::SolverConfig::default() },
        ..slam_backend::VioParams::default()
    };
    let mut vio = slam_backend::VioPipeline::new(rig, initial_state, seq.cam0_frames[0].timestamp_ns, gravity_world, params);
    vio.init_map(&left0, &right0);

    let mut trajectory = vec![(seq.cam0_frames[0].timestamp_ns, nalgebra::Vector3::zeros())];
    let mut prev_timestamp = seq.cam0_frames[0].timestamp_ns;
    let mut lost_at = None;
    for i in 1..num_frames {
        let left = seq.load_cam0_image(i).expect("decode frame");
        let right = seq.load_cam1_image(i).expect("decode frame");
        let timestamp = seq.cam0_frames[i].timestamp_ns;
        let imu_since_last: Vec<_> = seq.imu_samples.iter().filter(|s| s.timestamp_ns > prev_timestamp && s.timestamp_ns <= timestamp).cloned().collect();
        prev_timestamp = timestamp;

        match vio.process_frame(&left, &right, timestamp, &imu_since_last) {
            Some(result) if result.is_keyframe => {
                trajectory.push((timestamp, result.pose_world_to_body.inverse().translation));
            }
            Some(_) => {}
            None => {
                lost_at = Some(i);
                break;
            }
        }
    }

    print!(
        "stereo-inertial VIO: {} landmarks, {} keyframes over {num_frames} frames{}",
        vio.num_landmarks(),
        trajectory.len(),
        lost_at.map(|i| format!(" (lost at frame {i})")).unwrap_or_default()
    );

    let ate_of = |gt: &slam_eval::GroundTruthTrajectory, poses: &[(u64, nalgebra::Vector3<f64>)]| -> Option<slam_eval::AteStats> {
        let mut aligned_estimate = Vec::new();
        let mut aligned_groundtruth = Vec::new();
        for (t, p) in poses {
            if let Some(pose) = gt.interpolate(*t) {
                aligned_estimate.push(*p);
                aligned_groundtruth.push(pose.position);
            }
        }
        slam_eval::compute_ate(&aligned_estimate, &aligned_groundtruth)
    };

    match groundtruth {
        Some(gt) => match ate_of(gt, &trajectory) {
            Some(stats) => println!(
                ", ATE (Sim3-aligned, stereo+IMU, marginalized sliding window) over {} keyframes: rmse={:.3}m mean={:.3}m median={:.3}m max={:.3}m",
                stats.num_points, stats.rmse, stats.mean, stats.median, stats.max
            ),
            None => println!(", not enough groundtruth-covered keyframes to compute ATE"),
        },
        None => println!(", no groundtruth available for ATE"),
    }

    // M8: one global bundle-adjustment pass over every keyframe ever
    // created (history + current window), reusing the same solver as the
    // per-window optimization above — reports before/after ATE so the
    // (typically small, per plan/STAGE1.md "improves or holds") effect is
    // directly visible rather than just claimed.
    let before = vio.all_keyframe_poses();
    if before.len() > params.window_size {
        let n = vio.global_bundle_adjustment();
        let after = vio.all_keyframe_poses();
        let before_world: Vec<_> = before.iter().map(|(t, p)| (*t, p.inverse().translation)).collect();
        let after_world: Vec<_> = after.iter().map(|(t, p)| (*t, p.inverse().translation)).collect();
        match groundtruth {
            Some(gt) => match (ate_of(gt, &before_world), ate_of(gt, &after_world)) {
                (Some(before_stats), Some(after_stats)) => println!(
                    "global bundle adjustment: {n} keyframes, ATE rmse before={:.3}m after={:.3}m",
                    before_stats.rmse, after_stats.rmse
                ),
                _ => println!("global bundle adjustment: {n} keyframes, not enough groundtruth coverage for before/after ATE"),
            },
            None => println!("global bundle adjustment: {n} keyframes, no groundtruth available for ATE"),
        }
    } else {
        println!("global bundle adjustment: skipped, not enough keyframes evicted past the window yet");
    }
}

/// Demonstrates M7 (`slam-loopclosure`): runs stereo VO over the full
/// sequence, captures loop-closure-ready keyframes (stereo-matched
/// landmarks + BRIEF descriptors) at a fixed stride, trains a BoW
/// vocabulary, detects + geometrically verifies a revisit, and reports
/// ATE with vs. without the resulting pose-graph optimization — directly
/// comparable numbers, so the improvement (or lack of one) is visible.
fn print_loop_closure(seq: &slam_dataset::EuRocSequence, groundtruth: Option<&slam_eval::GroundTruthTrajectory>) {
    let Some(gt) = groundtruth else {
        println!("loop closure: no groundtruth available, skipping");
        return;
    };

    let cal = &seq.calibration;
    let rig = slam_geometry::StereoRig {
        t_bs_cam0: slam_core::SE3::from_matrix(&cal.cam0.t_bs),
        t_bs_cam1: slam_core::SE3::from_matrix(&cal.cam1.t_bs),
        cam0: slam_geometry::PinholeCamera::new(cal.cam0.intrinsics, cal.cam0.distortion_coefficients),
        cam1: slam_geometry::PinholeCamera::new(cal.cam1.intrinsics, cal.cam1.distortion_coefficients),
    };
    let rect = rig.rectify();
    let mut vo = slam_frontend::VoPipeline::new(rig.clone(), slam_frontend::VoParams::default());

    let num_frames = seq.cam0_frames.len().min(seq.cam1_frames.len());
    let stride = 20usize;

    let left0 = seq.load_cam0_image(0).expect("decode frame 0");
    let right0 = seq.load_cam1_image(0).expect("decode frame 0");
    vo.init(&left0, &right0);

    let mut node_timestamps = vec![seq.cam0_frames[0].timestamp_ns];
    let mut vo_poses = vec![slam_core::SE3::identity()];
    let meta0 = slam_loopclosure::KeyframeMeta { keyframe_id: 0, timestamp_ns: seq.cam0_frames[0].timestamp_ns, pose_world_to_cam0: slam_core::SE3::identity() };
    let mut keyframes = vec![slam_loopclosure::capture_loop_keyframe(&left0, &right0, meta0, &rig, &rect, &slam_loopclosure::CaptureParams::default())];

    for i in 1..num_frames {
        let left = seq.load_cam0_image(i).expect("decode frame");
        let right = seq.load_cam1_image(i).expect("decode frame");
        let Some(result) = vo.process_frame(&left, &right) else {
            println!("loop closure: VO lost tracking at frame {i}, skipping");
            return;
        };
        if i % stride == 0 {
            let node_id = node_timestamps.len();
            node_timestamps.push(seq.cam0_frames[i].timestamp_ns);
            vo_poses.push(result.pose_world_to_cam0);
            let meta = slam_loopclosure::KeyframeMeta { keyframe_id: node_id, timestamp_ns: seq.cam0_frames[i].timestamp_ns, pose_world_to_cam0: result.pose_world_to_cam0 };
            keyframes.push(slam_loopclosure::capture_loop_keyframe(&left, &right, meta, &rig, &rect, &slam_loopclosure::CaptureParams::default()));
        }
    }

    let all_descriptors: Vec<_> = keyframes.iter().flat_map(|k| k.descriptors.iter().copied()).collect();
    if all_descriptors.len() < 500 {
        println!("loop closure: too few descriptors gathered, skipping");
        return;
    }
    let vocab = slam_loopclosure::Vocabulary::train(&all_descriptors, 300, 6, 17);

    let mut db = slam_loopclosure::KeyframeDatabase::new();
    for kf in &keyframes {
        db.insert(kf.keyframe_id, vocab.compute_bow(&kf.descriptors));
    }

    let min_id_gap = 30;
    let mut best_loop: Option<(usize, usize, slam_loopclosure::VerifiedLoop)> = None;
    for kf in &keyframes {
        let query_bow = vocab.compute_bow(&kf.descriptors);
        let Some((candidate_id, _)) = db.query(kf.keyframe_id, &query_bow, min_id_gap, 0.3) else {
            continue;
        };
        let candidate = &keyframes[candidate_id];
        let Some(verified) = slam_loopclosure::verify_loop_candidate(&kf.normalized, &kf.descriptors, &candidate.descriptors, &candidate.landmarks_world, &slam_loopclosure::GeometricVerificationParams::default()) else {
            continue;
        };
        if best_loop.as_ref().map(|(_, _, v)| verified.num_inliers > v.num_inliers).unwrap_or(true) {
            best_loop = Some((kf.keyframe_id, candidate_id, verified));
        }
    }
    let Some((current_id, candidate_id, verified)) = best_loop else {
        println!("loop closure: no loop detected/verified");
        return;
    };

    let mut edges = Vec::new();
    for i in 0..vo_poses.len() - 1 {
        let relative = vo_poses[i + 1].compose(&vo_poses[i].inverse());
        edges.push(slam_loopclosure::PoseGraphEdge { i, j: i + 1, relative_pose: relative, weight: 1.0 });
    }
    let relative_pose = verified.relative_pose.compose(&vo_poses[candidate_id].inverse());
    edges.push(slam_loopclosure::PoseGraphEdge { i: candidate_id, j: current_id, relative_pose, weight: 5000.0 });

    let ate_of = |poses: &[slam_core::SE3]| -> Option<f64> {
        let mut est = Vec::new();
        let mut truth = Vec::new();
        for (t, p) in node_timestamps.iter().zip(poses.iter()) {
            if let Some(pose) = gt.interpolate(*t) {
                est.push(p.inverse().translation);
                truth.push(pose.position);
            }
        }
        slam_eval::compute_ate(&est, &truth).map(|s| s.rmse)
    };
    let ate_without = ate_of(&vo_poses).unwrap_or(f64::NAN);
    let mut optimized_poses = vo_poses.clone();
    slam_loopclosure::optimize_pose_graph(&mut optimized_poses, &edges, 0, 50);
    let ate_with = ate_of(&optimized_poses).unwrap_or(f64::NAN);

    println!(
        "loop closure: keyframe {current_id} <-> {candidate_id} ({} inliers), ATE without={ate_without:.3}m with={ate_with:.3}m",
        verified.num_inliers
    );
}

fn print_groundtruth_summary(traj: &slam_eval::GroundTruthTrajectory) {
    let states = traj.states();
    let Some(first) = states.first() else {
        println!("groundtruth: 0 states");
        return;
    };
    let last = states.last().unwrap();
    let duration_s = (last.timestamp_ns - first.timestamp_ns) as f64 * 1e-9;

    let mut min = first.position;
    let mut max = first.position;
    for s in states {
        min.x = min.x.min(s.position.x);
        min.y = min.y.min(s.position.y);
        min.z = min.z.min(s.position.z);
        max.x = max.x.max(s.position.x);
        max.y = max.y.max(s.position.y);
        max.z = max.z.max(s.position.z);
    }

    println!(
        "groundtruth: {} states, {:.2}s span, start pos=({:.2},{:.2},{:.2}) end pos=({:.2},{:.2},{:.2})",
        states.len(),
        duration_s,
        first.position.x,
        first.position.y,
        first.position.z,
        last.position.x,
        last.position.y,
        last.position.z,
    );
    println!(
        "groundtruth bbox: x=[{:.2},{:.2}] y=[{:.2},{:.2}] z=[{:.2},{:.2}]",
        min.x, max.x, min.y, max.y, min.z, max.z
    );
}
