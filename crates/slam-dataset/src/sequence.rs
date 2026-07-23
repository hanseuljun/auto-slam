use std::path::{Path, PathBuf};

use nalgebra::Vector3;

use crate::calibration::Calibration;
use crate::events::EventStream;

/// One row of `cam{0,1}/data.csv`: a timestamp and the PNG filename it maps to.
#[derive(Debug, Clone)]
pub struct CameraFrame {
    pub timestamp_ns: u64,
    pub filename: String,
}

/// One row of `imu0/data.csv`.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub timestamp_ns: u64,
    pub gyro: Vector3<f64>,
    pub accel: Vector3<f64>,
}

fn read_camera_csv(path: &Path) -> anyhow::Result<Vec<CameraFrame>> {
    let mut reader = csv::ReaderBuilder::new()
        .has_headers(true)
        .from_path(path)?;
    let mut frames = Vec::new();
    for record in reader.records() {
        let record = record?;
        let timestamp_ns: u64 = record.get(0).unwrap().trim().parse()?;
        let filename = record.get(1).unwrap().trim().to_string();
        frames.push(CameraFrame {
            timestamp_ns,
            filename,
        });
    }
    Ok(frames)
}

fn read_imu_csv(path: &Path) -> anyhow::Result<Vec<ImuSample>> {
    let mut reader = csv::ReaderBuilder::new()
        .has_headers(true)
        .from_path(path)?;
    let mut samples = Vec::new();
    for record in reader.records() {
        let record = record?;
        let field = |i: usize| -> anyhow::Result<f64> { Ok(record.get(i).unwrap().trim().parse()?) };
        let timestamp_ns = field(0)? as u64;
        let gyro = Vector3::new(field(1)?, field(2)?, field(3)?);
        let accel = Vector3::new(field(4)?, field(5)?, field(6)?);
        samples.push(ImuSample {
            timestamp_ns,
            gyro,
            accel,
        });
    }
    Ok(samples)
}

/// A loaded EuRoC `mav0/` sequence: calibration plus the three raw streams
/// (`cam0`, `cam1`, `imu0`). Image data is decoded lazily on request, not
/// held in memory.
pub struct EuRocSequence {
    root: PathBuf,
    pub calibration: Calibration,
    pub cam0_frames: Vec<CameraFrame>,
    pub cam1_frames: Vec<CameraFrame>,
    pub imu_samples: Vec<ImuSample>,
}

impl EuRocSequence {
    /// Load a sequence from its `mav0/` directory (e.g.
    /// `data/machine_hall/MH_01_easy/mav0`).
    pub fn load(mav0_root: impl AsRef<Path>) -> anyhow::Result<Self> {
        let root = mav0_root.as_ref().to_path_buf();
        let calibration = Calibration::load(&root)?;
        let cam0_frames = read_camera_csv(&root.join("cam0/data.csv"))?;
        let cam1_frames = read_camera_csv(&root.join("cam1/data.csv"))?;
        let imu_samples = read_imu_csv(&root.join("imu0/data.csv"))?;

        anyhow::ensure!(
            is_sorted_by_timestamp(&cam0_frames),
            "cam0/data.csv is not sorted by timestamp"
        );
        anyhow::ensure!(
            is_sorted_by_timestamp(&cam1_frames),
            "cam1/data.csv is not sorted by timestamp"
        );
        anyhow::ensure!(
            imu_samples.windows(2).all(|w| w[0].timestamp_ns <= w[1].timestamp_ns),
            "imu0/data.csv is not sorted by timestamp"
        );

        Ok(EuRocSequence {
            root,
            calibration,
            cam0_frames,
            cam1_frames,
            imu_samples,
        })
    }

    pub fn load_cam0_image(&self, index: usize) -> anyhow::Result<image::GrayImage> {
        self.load_image("cam0", &self.cam0_frames[index].filename)
    }

    pub fn load_cam1_image(&self, index: usize) -> anyhow::Result<image::GrayImage> {
        self.load_image("cam1", &self.cam1_frames[index].filename)
    }

    fn load_image(&self, cam_dir: &str, filename: &str) -> anyhow::Result<image::GrayImage> {
        let path = self.root.join(cam_dir).join("data").join(filename);
        Ok(image::open(&path)
            .map_err(|e| anyhow::anyhow!("failed to decode {}: {e}", path.display()))?
            .to_luma8())
    }

    /// A lazily-merged, time-ordered stream over all cam0/cam1/imu0 events.
    pub fn events(&self) -> EventStream<'_> {
        EventStream::new(&self.imu_samples, &self.cam0_frames, &self.cam1_frames)
    }

    /// The `cam0_frames` index whose timestamp is closest to `timestamp_ns`
    /// — the "playback time -> frame index" sync `bin/slam-viz`'s video
    /// panel needs (`plan/STAGE3.md` M4), kept here rather than
    /// reimplemented per-consumer since `cam0_frames` being sorted by
    /// timestamp (enforced in `load`) is exactly what makes a binary
    /// search correct.
    pub fn nearest_cam0_frame_index(&self, timestamp_ns: u64) -> usize {
        nearest_frame_index(&self.cam0_frames, timestamp_ns)
    }
}

fn is_sorted_by_timestamp(frames: &[CameraFrame]) -> bool {
    frames.windows(2).all(|w| w[0].timestamp_ns <= w[1].timestamp_ns)
}

/// Index of the frame in `frames` (assumed sorted by `timestamp_ns`,
/// ascending) whose timestamp is closest to `timestamp_ns`. `frames` must
/// be non-empty — callers already only reach this with a loaded
/// sequence's `cam0_frames`, which `EuRocSequence::load` guarantees is
/// both sorted and (per every real EuRoC sequence) non-empty.
fn nearest_frame_index(frames: &[CameraFrame], timestamp_ns: u64) -> usize {
    match frames.binary_search_by_key(&timestamp_ns, |f| f.timestamp_ns) {
        Ok(exact) => exact,
        Err(insert_at) => {
            if insert_at == 0 {
                0
            } else if insert_at >= frames.len() {
                frames.len() - 1
            } else {
                let before = insert_at - 1;
                let after = insert_at;
                let dist_before = timestamp_ns.abs_diff(frames[before].timestamp_ns);
                let dist_after = frames[after].timestamp_ns.abs_diff(timestamp_ns);
                if dist_before <= dist_after {
                    before
                } else {
                    after
                }
            }
        }
    }
}

#[cfg(test)]
mod nearest_frame_tests {
    use super::*;

    fn frames(timestamps: &[u64]) -> Vec<CameraFrame> {
        timestamps.iter().map(|&t| CameraFrame { timestamp_ns: t, filename: format!("{t}.png") }).collect()
    }

    #[test]
    fn exact_timestamp_match_returns_that_index() {
        let f = frames(&[100, 200, 300]);
        assert_eq!(nearest_frame_index(&f, 200), 1);
    }

    #[test]
    fn between_two_frames_picks_the_closer_one() {
        let f = frames(&[100, 200, 300]);
        assert_eq!(nearest_frame_index(&f, 140), 0);
        assert_eq!(nearest_frame_index(&f, 160), 1);
    }

    #[test]
    fn before_first_or_after_last_clamps_to_the_nearest_end() {
        let f = frames(&[100, 200, 300]);
        assert_eq!(nearest_frame_index(&f, 0), 0);
        assert_eq!(nearest_frame_index(&f, 10_000), 2);
    }

    #[test]
    fn single_frame_always_returns_index_zero() {
        let f = frames(&[500]);
        assert_eq!(nearest_frame_index(&f, 0), 0);
        assert_eq!(nearest_frame_index(&f, 999_999), 0);
    }
}
