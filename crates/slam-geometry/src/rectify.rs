use nalgebra::{Matrix3, Vector2, Vector3};
use slam_core::SE3;

use crate::pinhole::PinholeCamera;

/// A calibrated stereo rig: two cameras with extrinsics expressed relative
/// to a common body frame (EuRoC's `T_BS` convention: `X_body = T_BS *
/// X_sensor`, i.e. `t_bs.translation` is the camera center in the body
/// frame).
#[derive(Debug, Clone)]
pub struct StereoRig {
    pub t_bs_cam0: SE3,
    pub t_bs_cam1: SE3,
    pub cam0: PinholeCamera,
    pub cam1: PinholeCamera,
}

impl StereoRig {
    /// The relative pose mapping a point in cam0's frame to cam1's frame.
    pub fn relative_pose_cam1_from_cam0(&self) -> SE3 {
        self.t_bs_cam1.inverse().compose(&self.t_bs_cam0)
    }

    /// Computes the rectifying rotations, shared rectified intrinsics, and
    /// baseline for this rig, following the standard (Bouguet) stereo
    /// rectification construction.
    pub fn rectify(&self) -> StereoRectification {
        let r0 = self.t_bs_cam0.rotation.matrix();
        let r1 = self.t_bs_cam1.rotation.matrix();
        let c0 = self.t_bs_cam0.translation;
        let c1 = self.t_bs_cam1.translation;

        // Rotation mapping a vector expressed in cam0's frame to cam1's frame.
        let r10 = r1.transpose() * r0;

        // Baseline vector (cam0 -> cam1) expressed in cam0's own frame.
        let baseline_vec = r0.transpose() * (c1 - c0);
        let baseline = baseline_vec.norm();
        let e1 = baseline_vec / baseline;
        let z_axis = Vector3::new(0.0, 0.0, 1.0);
        let e2 = z_axis.cross(&e1).normalize();
        let e3 = e1.cross(&e2);

        let r_rect0 = Matrix3::from_rows(&[e1.transpose(), e2.transpose(), e3.transpose()]);
        let r_rect1 = r_rect0 * r10.transpose();

        let rectified_intrinsics = [
            0.5 * (self.cam0.intrinsics[0] + self.cam1.intrinsics[0]),
            0.5 * (self.cam0.intrinsics[1] + self.cam1.intrinsics[1]),
            0.5 * (self.cam0.intrinsics[2] + self.cam1.intrinsics[2]),
            0.5 * (self.cam0.intrinsics[3] + self.cam1.intrinsics[3]),
        ];

        StereoRectification {
            r_rect0,
            r_rect1,
            rectified_intrinsics,
            baseline,
        }
    }
}

/// The result of stereo rectification: rotations that align both cameras to
/// a common frame with parallel optical axes and a purely horizontal
/// baseline, plus the shared intrinsics used to project into that frame.
#[derive(Debug, Clone)]
pub struct StereoRectification {
    /// Rotates a point expressed in cam0's raw frame into the rectified frame.
    pub r_rect0: Matrix3<f64>,
    /// Rotates a point expressed in cam1's raw frame into the rectified frame.
    pub r_rect1: Matrix3<f64>,
    /// `[fu, fv, cu, cv]`, shared by both rectified virtual cameras.
    pub rectified_intrinsics: [f64; 4],
    /// Distance between the two camera centers, in meters.
    pub baseline: f64,
}

impl StereoRectification {
    fn pinhole_project(&self, p: Vector3<f64>) -> Vector2<f64> {
        let [fu, fv, cu, cv] = self.rectified_intrinsics;
        Vector2::new(fu * p.x / p.z + cu, fv * p.y / p.z + cv)
    }

    /// Projects a point given in cam0's raw (unrectified) frame into the
    /// left rectified image.
    pub fn project_left(&self, p_cam0: Vector3<f64>) -> Vector2<f64> {
        self.pinhole_project(self.r_rect0 * p_cam0)
    }

    /// Projects a point given in cam1's raw (unrectified) frame into the
    /// right rectified image.
    pub fn project_right(&self, p_cam1: Vector3<f64>) -> Vector2<f64> {
        self.pinhole_project(self.r_rect1 * p_cam1)
    }

    /// The depth (rectified z) and disparity `u_left - u_right` a point at
    /// `p_cam0` (cam0 raw frame) would produce; `disparity = fu * baseline
    /// / depth` by construction of the rectified frame.
    pub fn depth_and_disparity(&self, p_cam0: Vector3<f64>) -> (f64, f64) {
        let p_rect = self.r_rect0 * p_cam0;
        let depth = p_rect.z;
        let disparity = self.rectified_intrinsics[0] * self.baseline / depth;
        (depth, disparity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Matrix4;
    use slam_core::SE3;

    fn mh01_rig() -> StereoRig {
        // T_BS matrices and intrinsics from
        // data/machine_hall/MH_01_easy/mav0/cam{0,1}/sensor.yaml.
        let t_bs_cam0 = SE3::from_matrix(&Matrix4::from_row_slice(&[
            0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            0.0, 0.0, 0.0, 1.0,
        ]));
        let t_bs_cam1 = SE3::from_matrix(&Matrix4::from_row_slice(&[
            0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
            0.0, 0.0, 0.0, 1.0,
        ]));
        StereoRig {
            t_bs_cam0,
            t_bs_cam1,
            cam0: PinholeCamera::new(
                [458.654, 457.296, 367.215, 248.375],
                [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05],
            ),
            cam1: PinholeCamera::new(
                [457.587, 456.134, 379.999, 255.238],
                [-0.28368365, 0.07451284, -0.00010473, -3.556e-05],
            ),
        }
    }

    #[test]
    fn baseline_matches_known_euroc_value() {
        let rect = mh01_rig().rectify();
        // The MH sensor's stereo baseline is well known to be ~11cm.
        assert!(
            (rect.baseline - 0.110).abs() < 0.005,
            "unexpected baseline: {}",
            rect.baseline
        );
    }

    #[test]
    fn rectifying_rotations_are_orthonormal() {
        let rect = mh01_rig().rectify();
        for r in [rect.r_rect0, rect.r_rect1] {
            assert_relative_eq!(r * r.transpose(), Matrix3::identity(), epsilon = 1e-10);
            assert_relative_eq!(r.determinant(), 1.0, epsilon = 1e-10);
        }
    }

    /// The whole point of rectification: any 3D point, projected into both
    /// rectified cameras, must land on the same row (zero vertical
    /// disparity) with horizontal disparity `fu * baseline / depth`. This
    /// is a mathematical property of a correct rectification (see
    /// `memory/decisions` for why this replaces a real-feature-match check
    /// at M1 — that needs M2/M3's tracker first).
    #[test]
    fn synthetic_points_have_zero_vertical_disparity_and_correct_horizontal_disparity() {
        let rig = mh01_rig();
        let rect = rig.rectify();
        let t_cam1_from_cam0 = rig.relative_pose_cam1_from_cam0();

        for p_cam0 in [
            Vector3::new(0.0, 0.0, 2.0),
            Vector3::new(0.5, -0.3, 3.0),
            Vector3::new(-1.0, 0.2, 5.0),
            Vector3::new(0.1, 0.4, 1.0),
        ] {
            let p_cam1 = t_cam1_from_cam0.transform(&p_cam0);
            let left = rect.project_left(p_cam0);
            let right = rect.project_right(p_cam1);

            assert_relative_eq!(left.y, right.y, epsilon = 1e-9);

            let (depth, expected_disparity) = rect.depth_and_disparity(p_cam0);
            assert!(depth > 0.0);
            assert_relative_eq!(left.x - right.x, expected_disparity, epsilon = 1e-9);
        }
    }
}
