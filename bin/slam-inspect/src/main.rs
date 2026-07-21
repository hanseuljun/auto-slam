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
    if gt_path.exists() {
        let traj = slam_eval::GroundTruthTrajectory::load(&gt_path)?;
        print_groundtruth_summary(&traj);
    } else {
        println!("no groundtruth found at {}", gt_path.display());
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
