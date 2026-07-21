//! Ground-truth trajectory I/O, timestamp-interpolated pose lookup, and
//! Sim3/ATE alignment, used to evaluate estimated trajectories against
//! `state_groundtruth_estimate0/data.csv` (M0: I/O; M3: brought Umeyama+ATE
//! forward from M9 since M3's own checkpoint needs it — see
//! `memory/decisions`. The fuller RPE/per-sequence-report harness is still
//! M9's job).

mod align;

use std::path::Path;

use nalgebra::{UnitQuaternion, Vector3};

pub use align::{compute_ate, umeyama_alignment, AteStats, Sim3Alignment};

/// One row of `state_groundtruth_estimate0/data.csv`.
#[derive(Debug, Clone, Copy)]
pub struct GroundTruthState {
    pub timestamp_ns: u64,
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub velocity: Vector3<f64>,
    pub gyro_bias: Vector3<f64>,
    pub accel_bias: Vector3<f64>,
}

/// A linearly-interpolated pose (position: lerp, orientation: slerp) at a
/// query timestamp between two bracketing ground-truth states.
#[derive(Debug, Clone, Copy)]
pub struct InterpolatedPose {
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
}

/// A ground-truth trajectory, sorted by timestamp, with interpolated lookup.
pub struct GroundTruthTrajectory {
    states: Vec<GroundTruthState>,
}

impl GroundTruthTrajectory {
    pub fn load(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let path = path.as_ref();
        let mut reader = csv::ReaderBuilder::new()
            .has_headers(true)
            .from_path(path)?;
        let mut states = Vec::new();
        for record in reader.records() {
            let record = record?;
            let field = |i: usize| -> anyhow::Result<f64> {
                Ok(record.get(i).unwrap().trim().parse()?)
            };
            let timestamp_ns = field(0)? as u64;
            let position = Vector3::new(field(1)?, field(2)?, field(3)?);
            let orientation = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                field(4)?, // w
                field(5)?, // x
                field(6)?, // y
                field(7)?, // z
            ));
            let velocity = Vector3::new(field(8)?, field(9)?, field(10)?);
            let gyro_bias = Vector3::new(field(11)?, field(12)?, field(13)?);
            let accel_bias = Vector3::new(field(14)?, field(15)?, field(16)?);
            states.push(GroundTruthState {
                timestamp_ns,
                position,
                orientation,
                velocity,
                gyro_bias,
                accel_bias,
            });
        }
        anyhow::ensure!(
            states.windows(2).all(|w| w[0].timestamp_ns < w[1].timestamp_ns),
            "{} is not strictly sorted by timestamp",
            path.display()
        );
        Ok(GroundTruthTrajectory { states })
    }

    pub fn states(&self) -> &[GroundTruthState] {
        &self.states
    }

    /// Interpolated pose at `timestamp_ns`, or `None` if it falls outside
    /// the trajectory's time range.
    pub fn interpolate(&self, timestamp_ns: u64) -> Option<InterpolatedPose> {
        let states = &self.states;
        if states.is_empty()
            || timestamp_ns < states.first().unwrap().timestamp_ns
            || timestamp_ns > states.last().unwrap().timestamp_ns
        {
            return None;
        }
        let idx = match states.binary_search_by_key(&timestamp_ns, |s| s.timestamp_ns) {
            Ok(i) => {
                return Some(InterpolatedPose {
                    position: states[i].position,
                    orientation: states[i].orientation,
                })
            }
            Err(i) => i,
        };
        // `idx` is the insertion point, so states[idx-1].t < timestamp_ns < states[idx].t.
        let a = &states[idx - 1];
        let b = &states[idx];
        let span = (b.timestamp_ns - a.timestamp_ns) as f64;
        let t = (timestamp_ns - a.timestamp_ns) as f64 / span;
        Some(InterpolatedPose {
            position: a.position.lerp(&b.position, t),
            orientation: a.orientation.slerp(&b.orientation, t),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn mh01_groundtruth_csv() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(
            "../../data/machine_hall/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv",
        )
    }

    #[test]
    fn loads_mh01_groundtruth_and_first_row_matches_csv() {
        let traj = GroundTruthTrajectory::load(mh01_groundtruth_csv()).expect("load groundtruth");
        // Row count confirmed via `wc -l` (36383) minus the header row.
        assert_eq!(traj.states().len(), 36382);

        let first = &traj.states()[0];
        assert_eq!(first.timestamp_ns, 1403636580838555648);
        assert!((first.position.x - 4.688319).abs() < 1e-6);
        assert!((first.position.y - (-1.786938)).abs() < 1e-6);
        assert!((first.position.z - 0.783338).abs() < 1e-6);
        assert!((first.orientation.quaternion().w - 0.534108).abs() < 1e-6);
    }

    #[test]
    fn interpolate_at_exact_timestamp_matches_state() {
        let traj = GroundTruthTrajectory::load(mh01_groundtruth_csv()).expect("load groundtruth");
        let target = traj.states()[10];
        let pose = traj.interpolate(target.timestamp_ns).expect("in range");
        assert!((pose.position - target.position).norm() < 1e-9);
    }

    #[test]
    fn interpolate_midpoint_is_between_bracketing_states() {
        let traj = GroundTruthTrajectory::load(mh01_groundtruth_csv()).expect("load groundtruth");
        let a = traj.states()[10];
        let b = traj.states()[11];
        let mid_ts = (a.timestamp_ns + b.timestamp_ns) / 2;
        let pose = traj.interpolate(mid_ts).expect("in range");
        // Should be close to the midpoint of the two bracketing positions.
        let expected_mid = (a.position + b.position) / 2.0;
        assert!((pose.position - expected_mid).norm() < 1e-6);
    }

    #[test]
    fn interpolate_out_of_range_returns_none() {
        let traj = GroundTruthTrajectory::load(mh01_groundtruth_csv()).expect("load groundtruth");
        let first_ts = traj.states().first().unwrap().timestamp_ns;
        assert!(traj.interpolate(first_ts - 1).is_none());
        let last_ts = traj.states().last().unwrap().timestamp_ns;
        assert!(traj.interpolate(last_ts + 1).is_none());
    }
}
