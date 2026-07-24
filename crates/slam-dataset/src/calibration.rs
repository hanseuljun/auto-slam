use std::path::Path;

use nalgebra::Matrix4;
use serde::Deserialize;

/// A 4x4 matrix as it appears in EuRoC's `sensor.yaml` (`T_BS`), row-major.
#[derive(Debug, Clone, Deserialize)]
struct RawMatrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl RawMatrix {
    fn to_matrix4(&self) -> anyhow::Result<Matrix4<f64>> {
        anyhow::ensure!(
            self.rows == 4 && self.cols == 4 && self.data.len() == 16,
            "expected a 4x4 matrix, got {}x{} with {} entries",
            self.rows,
            self.cols,
            self.data.len()
        );
        // `data` is row-major; nalgebra's `from_row_slice` matches that layout.
        Ok(Matrix4::from_row_slice(&self.data))
    }
}

#[derive(Debug, Deserialize)]
struct RawCameraYaml {
    #[allow(dead_code)]
    sensor_type: String,
    #[serde(rename = "T_BS")]
    t_bs: RawMatrix,
    rate_hz: f64,
    resolution: [u32; 2],
    #[allow(dead_code)]
    camera_model: String,
    intrinsics: [f64; 4],
    #[allow(dead_code)]
    distortion_model: String,
    distortion_coefficients: [f64; 4],
}

#[derive(Debug, Deserialize)]
struct RawImuYaml {
    #[allow(dead_code)]
    sensor_type: String,
    #[serde(rename = "T_BS")]
    t_bs: RawMatrix,
    rate_hz: f64,
    gyroscope_noise_density: f64,
    gyroscope_random_walk: f64,
    accelerometer_noise_density: f64,
    accelerometer_random_walk: f64,
}

/// Camera calibration parsed from a `cam{0,1}/sensor.yaml` file.
#[derive(Debug, Clone)]
pub struct CameraCalibration {
    /// Extrinsics transforming points from the sensor frame to the body
    /// frame (EuRoC's own `T_BS` convention: `X_body = T_BS * X_sensor`
    /// — see `slam_geometry::rectify`'s own doc comment for how this is
    /// actually used).
    pub t_bs: Matrix4<f64>,
    pub rate_hz: f64,
    pub resolution: [u32; 2],
    /// `[fu, fv, cu, cv]`.
    pub intrinsics: [f64; 4],
    /// Radial-tangential distortion `[k1, k2, p1, p2]` (no k3).
    pub distortion_coefficients: [f64; 4],
}

impl CameraCalibration {
    pub fn load(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let path = path.as_ref();
        let raw: RawCameraYaml = serde_yaml::from_reader(std::fs::File::open(path)?)
            .map_err(|e| anyhow::anyhow!("failed to parse {}: {e}", path.display()))?;
        Ok(CameraCalibration {
            t_bs: raw.t_bs.to_matrix4()?,
            rate_hz: raw.rate_hz,
            resolution: raw.resolution,
            intrinsics: raw.intrinsics,
            distortion_coefficients: raw.distortion_coefficients,
        })
    }
}

/// IMU calibration parsed from `imu0/sensor.yaml`.
#[derive(Debug, Clone)]
pub struct ImuCalibration {
    /// Extrinsics transforming points from the sensor frame to the body
    /// frame (identity for EuRoC, since the IMU defines the body frame —
    /// see `lib.rs`'s own `assert!(... t_bs.is_identity(...))` check).
    pub t_bs: Matrix4<f64>,
    pub rate_hz: f64,
    pub gyroscope_noise_density: f64,
    pub gyroscope_random_walk: f64,
    pub accelerometer_noise_density: f64,
    pub accelerometer_random_walk: f64,
}

impl ImuCalibration {
    pub fn load(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let path = path.as_ref();
        let raw: RawImuYaml = serde_yaml::from_reader(std::fs::File::open(path)?)
            .map_err(|e| anyhow::anyhow!("failed to parse {}: {e}", path.display()))?;
        Ok(ImuCalibration {
            t_bs: raw.t_bs.to_matrix4()?,
            rate_hz: raw.rate_hz,
            gyroscope_noise_density: raw.gyroscope_noise_density,
            gyroscope_random_walk: raw.gyroscope_random_walk,
            accelerometer_noise_density: raw.accelerometer_noise_density,
            accelerometer_random_walk: raw.accelerometer_random_walk,
        })
    }
}

/// The full calibration for one EuRoC `mav0/` sequence: stereo cameras + IMU.
#[derive(Debug, Clone)]
pub struct Calibration {
    pub cam0: CameraCalibration,
    pub cam1: CameraCalibration,
    pub imu0: ImuCalibration,
}

impl Calibration {
    pub fn load(mav0_root: impl AsRef<Path>) -> anyhow::Result<Self> {
        let root = mav0_root.as_ref();
        Ok(Calibration {
            cam0: CameraCalibration::load(root.join("cam0/sensor.yaml"))?,
            cam1: CameraCalibration::load(root.join("cam1/sensor.yaml"))?,
            imu0: ImuCalibration::load(root.join("imu0/sensor.yaml"))?,
        })
    }
}
