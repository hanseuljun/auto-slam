//! EuRoC dataset I/O: `sensor.yaml` calibration parsing, `data.csv` stream
//! parsing, lazy PNG decoding, and a merged time-ordered event stream over
//! cam0/cam1/imu0 (Stage 1 milestone M0).

mod calibration;
mod events;
mod sequence;

pub use calibration::{Calibration, CameraCalibration, ImuCalibration};
pub use events::{Event, EventStream};
pub use sequence::{CameraFrame, EuRocSequence, ImuSample};

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn mh01_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/machine_hall/MH_01_easy/mav0")
    }

    #[test]
    fn loads_mh01_calibration_matching_yaml() {
        let seq = EuRocSequence::load(mh01_root()).expect("load MH_01_easy");

        // cam0 intrinsics/distortion from cam0/sensor.yaml.
        assert_eq!(seq.calibration.cam0.resolution, [752, 480]);
        assert_eq!(seq.calibration.cam0.rate_hz, 20.0);
        assert_eq!(
            seq.calibration.cam0.intrinsics,
            [458.654, 457.296, 367.215, 248.375]
        );
        assert_eq!(
            seq.calibration.cam0.distortion_coefficients,
            [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
        );

        // cam1 intrinsics from cam1/sensor.yaml.
        assert_eq!(
            seq.calibration.cam1.intrinsics,
            [457.587, 456.134, 379.999, 255.238]
        );

        // imu0 is at 200 Hz and defines the body frame (T_BS = identity).
        assert_eq!(seq.calibration.imu0.rate_hz, 200.0);
        assert!(seq.calibration.imu0.t_bs.is_identity(1e-12));
        assert!((seq.calibration.imu0.gyroscope_noise_density - 1.6968e-04).abs() < 1e-12);
        assert!((seq.calibration.imu0.accelerometer_random_walk - 3.0000e-3).abs() < 1e-12);
    }

    #[test]
    fn loads_mh01_frame_and_imu_counts() {
        let seq = EuRocSequence::load(mh01_root()).expect("load MH_01_easy");

        // Confirmed via `wc -l` on the raw csv files (minus header row).
        assert_eq!(seq.cam0_frames.len(), 3682);
        assert_eq!(seq.cam1_frames.len(), 3682);
        assert_eq!(seq.imu_samples.len(), 36820);

        // cam0 and cam1 are triggered together in EuRoC: same count and
        // identical timestamps.
        for (f0, f1) in seq.cam0_frames.iter().zip(seq.cam1_frames.iter()) {
            assert_eq!(f0.timestamp_ns, f1.timestamp_ns);
        }

        // IMU runs at ~200 Hz => ~5ms between consecutive samples (real
        // sensor timestamps jitter slightly around the nominal rate).
        let dt_ns = seq.imu_samples[1].timestamp_ns - seq.imu_samples[0].timestamp_ns;
        assert!(
            (dt_ns as i64 - 5_000_000i64).abs() < 100_000,
            "unexpected imu dt: {dt_ns}ns"
        );
    }

    #[test]
    fn decodes_first_stereo_pair_with_expected_resolution() {
        let seq = EuRocSequence::load(mh01_root()).expect("load MH_01_easy");
        let left = seq.load_cam0_image(0).expect("decode cam0 frame 0");
        let right = seq.load_cam1_image(0).expect("decode cam1 frame 0");
        assert_eq!(left.dimensions(), (752, 480));
        assert_eq!(right.dimensions(), (752, 480));
    }

    #[test]
    fn event_stream_is_time_ordered_and_covers_every_sample() {
        let seq = EuRocSequence::load(mh01_root()).expect("load MH_01_easy");
        let events: Vec<(u64, Event)> = seq.events().collect();

        assert_eq!(
            events.len(),
            seq.imu_samples.len() + seq.cam0_frames.len() + seq.cam1_frames.len()
        );
        assert!(events.windows(2).all(|w| w[0].0 <= w[1].0));

        let imu_count = events
            .iter()
            .filter(|(_, e)| matches!(e, Event::Imu(_)))
            .count();
        let cam0_count = events
            .iter()
            .filter(|(_, e)| matches!(e, Event::Cam0(_)))
            .count();
        let cam1_count = events
            .iter()
            .filter(|(_, e)| matches!(e, Event::Cam1(_)))
            .count();
        assert_eq!(imu_count, seq.imu_samples.len());
        assert_eq!(cam0_count, seq.cam0_frames.len());
        assert_eq!(cam1_count, seq.cam1_frames.len());
    }
}
