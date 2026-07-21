use nalgebra::Vector2;

/// A pinhole camera with radial-tangential distortion (EuRoC's model: 4
/// coefficients `[k1, k2, p1, p2]`, no k3).
#[derive(Debug, Clone, Copy)]
pub struct PinholeCamera {
    /// `[fu, fv, cu, cv]`.
    pub intrinsics: [f64; 4],
    /// `[k1, k2, p1, p2]`.
    pub distortion: [f64; 4],
}

impl PinholeCamera {
    pub fn new(intrinsics: [f64; 4], distortion: [f64; 4]) -> Self {
        PinholeCamera {
            intrinsics,
            distortion,
        }
    }

    fn fu(&self) -> f64 {
        self.intrinsics[0]
    }
    fn fv(&self) -> f64 {
        self.intrinsics[1]
    }
    fn cu(&self) -> f64 {
        self.intrinsics[2]
    }
    fn cv(&self) -> f64 {
        self.intrinsics[3]
    }

    /// Applies radial-tangential distortion to ideal (undistorted)
    /// normalized coordinates.
    pub fn distort(&self, xy: Vector2<f64>) -> Vector2<f64> {
        let [k1, k2, p1, p2] = self.distortion;
        let (x, y) = (xy.x, xy.y);
        let r2 = x * x + y * y;
        let radial = 1.0 + k1 * r2 + k2 * r2 * r2;
        let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        Vector2::new(x * radial + dx, y * radial + dy)
    }

    /// Inverts `distort` via fixed-point iteration (the standard approach
    /// for radial-tangential distortion, which has no closed-form inverse).
    pub fn undistort(&self, xy_distorted: Vector2<f64>) -> Vector2<f64> {
        let [k1, k2, p1, p2] = self.distortion;
        let mut xy = xy_distorted;
        for _ in 0..20 {
            let (x, y) = (xy.x, xy.y);
            let r2 = x * x + y * y;
            let radial = 1.0 + k1 * r2 + k2 * r2 * r2;
            let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            xy = Vector2::new((xy_distorted.x - dx) / radial, (xy_distorted.y - dy) / radial);
        }
        xy
    }

    /// Full projection: 3D point in the camera frame -> distorted pixel
    /// coordinates. `p.z` must be positive (point in front of the camera).
    pub fn project(&self, p: nalgebra::Vector3<f64>) -> Vector2<f64> {
        let normalized = Vector2::new(p.x / p.z, p.y / p.z);
        let distorted = self.distort(normalized);
        Vector2::new(
            self.fu() * distorted.x + self.cu(),
            self.fv() * distorted.y + self.cv(),
        )
    }

    /// Inverse of `project`'s 2D part: pixel coordinates -> undistorted
    /// normalized coordinates (a ray direction `[x, y, 1]` up to scale).
    pub fn unproject_to_normalized(&self, pixel: Vector2<f64>) -> Vector2<f64> {
        let distorted_normalized = Vector2::new(
            (pixel.x - self.cu()) / self.fu(),
            (pixel.y - self.cv()) / self.fv(),
        );
        self.undistort(distorted_normalized)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn mh01_cam0() -> PinholeCamera {
        // From data/machine_hall/MH_01_easy/mav0/cam0/sensor.yaml.
        PinholeCamera::new(
            [458.654, 457.296, 367.215, 248.375],
            [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05],
        )
    }

    #[test]
    fn distort_undistort_roundtrip() {
        let cam = mh01_cam0();
        for (x, y) in [
            (0.0, 0.0),
            (0.1, 0.05),
            (-0.2, 0.15),
            (0.3, -0.3),
            (-0.25, -0.2),
        ] {
            let xy = Vector2::new(x, y);
            let distorted = cam.distort(xy);
            let recovered = cam.undistort(distorted);
            assert_relative_eq!(xy, recovered, epsilon = 1e-9);
        }
    }

    #[test]
    fn project_unproject_roundtrip_within_image() {
        let cam = mh01_cam0();
        // A grid of points spanning most of the 752x480 sensor.
        for u in [50.0, 200.0, 376.0, 550.0, 700.0] {
            for v in [30.0, 150.0, 240.0, 350.0, 450.0] {
                let pixel = Vector2::new(u, v);
                let normalized = cam.unproject_to_normalized(pixel);
                let p3 = nalgebra::Vector3::new(normalized.x, normalized.y, 1.0);
                let reprojected = cam.project(p3);
                assert_relative_eq!(pixel, reprojected, epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn zero_distortion_pinhole_is_identity_in_normalized_coords() {
        let cam = PinholeCamera::new([500.0, 500.0, 320.0, 240.0], [0.0, 0.0, 0.0, 0.0]);
        let xy = Vector2::new(0.4, -0.3);
        assert_relative_eq!(cam.distort(xy), xy, epsilon = 1e-12);
        assert_relative_eq!(cam.undistort(xy), xy, epsilon = 1e-12);
    }
}
