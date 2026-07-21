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
