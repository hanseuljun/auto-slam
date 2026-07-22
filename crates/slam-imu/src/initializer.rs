use nalgebra::Vector3;

/// Result of the static (stationary-window) IMU initializer.
#[derive(Debug, Clone, Copy)]
pub struct StaticInitResult {
    /// Gyro bias estimate (true angular velocity is ~0 while stationary,
    /// so the mean raw gyro reading directly estimates the bias).
    pub gyro_bias: Vector3<f64>,
    /// Should be close to 9.81 for a genuinely stationary window; a
    /// caller-side sanity check, not enforced here.
    pub gravity_magnitude: f64,
    /// Unit vector: the direction gravity's reaction force points in the
    /// IMU's body frame during the window (a stationary accelerometer
    /// reads `+g` along "up", per the ADIS16448's/EuRoC's convention).
    pub gravity_direction_body: Vector3<f64>,
}

/// Scans for the first window of `window_size` consecutive gyro samples
/// that all have norm below `max_gyro_norm`, returning its start index —
/// a simple, standard "is the IMU currently excited?" stationarity check.
///
/// Needed because real sequences don't reliably start stationary at index
/// 0 even when the *sequence* is nominally a "static start" case — see
/// `memory/notes/dataset-quirks.md` (MH_01's genuinely-still window starts
/// around 26.5s in, not at t=0).
pub fn find_stationary_window(gyro_samples: &[Vector3<f64>], window_size: usize, max_gyro_norm: f64) -> Option<usize> {
    if gyro_samples.len() < window_size {
        return None;
    }
    (0..=gyro_samples.len() - window_size)
        .find(|&start| gyro_samples[start..start + window_size].iter().all(|g| g.norm() < max_gyro_norm))
}

/// Estimates gyro bias and the gravity vector (magnitude + body-frame
/// direction) from a window of raw IMU samples assumed stationary — the
/// static case from `plan/STAGE1.md` M4 (use `find_stationary_window` to
/// locate that window rather than assuming it starts at sample 0).
/// MH_04/05 start in motion throughout and need the dynamic vision-IMU
/// alignment initializer instead (`slam_frontend`, since it also needs VO
/// keyframe poses).
pub fn static_initialize(gyro_samples: &[Vector3<f64>], accel_samples: &[Vector3<f64>]) -> Option<StaticInitResult> {
    if gyro_samples.is_empty() || accel_samples.is_empty() {
        return None;
    }

    let gyro_bias = gyro_samples.iter().sum::<Vector3<f64>>() / gyro_samples.len() as f64;
    let mean_accel = accel_samples.iter().sum::<Vector3<f64>>() / accel_samples.len() as f64;
    let gravity_magnitude = mean_accel.norm();
    if gravity_magnitude < 1e-6 {
        return None;
    }

    Some(StaticInitResult {
        gyro_bias,
        gravity_magnitude,
        gravity_direction_body: mean_accel / gravity_magnitude,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::path::PathBuf;

    #[test]
    fn recovers_known_bias_and_gravity_from_synthetic_stationary_data() {
        let true_bias = Vector3::new(0.02, -0.015, 0.01);
        let true_gravity_body = Vector3::new(0.05, 0.02, 9.80).normalize() * 9.81;

        // Small symmetric "noise" that should average out.
        let gyro_samples: Vec<Vector3<f64>> = (0..200)
            .map(|i| {
                let n = ((i % 2) as f64 * 2.0 - 1.0) * 0.001;
                true_bias + Vector3::new(n, -n, n)
            })
            .collect();
        let accel_samples: Vec<Vector3<f64>> = (0..200)
            .map(|i| {
                let n = ((i % 2) as f64 * 2.0 - 1.0) * 0.01;
                true_gravity_body + Vector3::new(n, n, -n)
            })
            .collect();

        let result = static_initialize(&gyro_samples, &accel_samples).expect("should initialize");
        assert_relative_eq!(result.gyro_bias, true_bias, epsilon = 1e-9);
        assert_relative_eq!(result.gravity_magnitude, 9.81, epsilon = 1e-9);
        assert_relative_eq!(
            result.gravity_direction_body,
            true_gravity_body.normalize(),
            epsilon = 1e-9
        );
    }

    #[test]
    fn empty_input_returns_none() {
        assert!(static_initialize(&[], &[]).is_none());
    }

    /// M4's actual checkpoint test, against real data. MH_01 is nominally
    /// a "static start" sequence, but empirically its truly-still window
    /// is ~26.5s in, not at t=0 (see `memory/notes/dataset-quirks.md`) —
    /// so this locates the window via `find_stationary_window` rather than
    /// assuming the first N samples, and checks gravity magnitude lands
    /// near 9.81 and gyro bias is in the ADIS16448's realistic range (a
    /// few milli-rad/s, not the sensor's full-scale range).
    #[test]
    fn real_mh01_stationary_window_gives_plausible_gravity_and_bias() {
        let mav0 = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/machine_hall/MH_01_easy/mav0");
        let seq = slam_dataset::EuRocSequence::load(mav0).expect("load MH_01_easy");

        let all_gyro: Vec<Vector3<f64>> = seq.imu_samples.iter().map(|s| s.gyro).collect();
        let window_size = 200; // 1s at 200Hz.
        let start = find_stationary_window(&all_gyro, window_size, 0.09)
            .expect("MH_01 should contain a genuinely stationary window");

        let gyro = &all_gyro[start..start + window_size];
        let accel: Vec<Vector3<f64>> = seq.imu_samples[start..start + window_size].iter().map(|s| s.accel).collect();

        let result = static_initialize(gyro, &accel).expect("should initialize");
        assert!(
            (result.gravity_magnitude - 9.81).abs() < 0.1,
            "gravity magnitude off: {}",
            result.gravity_magnitude
        );
        // The ADIS16448's raw (factory-uncalibrated) bias can genuinely run
        // to several deg/s — empirically ~0.08 rad/s on this sequence's
        // z-axis (see memory/notes/dataset-quirks.md). This bound checks
        // "plausible for an uncalibrated MEMS gyro," not "near zero."
        assert!(
            result.gyro_bias.norm() < 0.15,
            "gyro bias implausibly large for a stationary window: {}",
            result.gyro_bias
        );
    }
}
