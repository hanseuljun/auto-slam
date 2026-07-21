//! Pinhole + radial-tangential camera model, stereo rectification,
//! triangulation, and pose-from-points estimation (Stage 1 milestone M1).
//!
//! Minimal-point robust solvers (P3P, 8-point/5-point relative pose +
//! RANSAC) are deferred to M3/M4, where a RANSAC-based initializer is the
//! actual consumer — see `memory/decisions` for why building them now,
//! without that consumer, was judged premature.

mod pinhole;
mod pnp;
mod rectify;
mod triangulate;

pub use pinhole::PinholeCamera;
pub use pnp::{estimate_pose_dlt, refine_pose_gauss_newton};
pub use rectify::{StereoRectification, StereoRig};
pub use triangulate::{triangulate_linear, triangulate_refine};

#[cfg(test)]
mod integration_tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Vector3;
    use slam_core::SE3;
    use std::path::PathBuf;

    fn mh01_calibration() -> slam_dataset::Calibration {
        let mav0 = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/machine_hall/MH_01_easy/mav0");
        slam_dataset::Calibration::load(mav0).expect("load MH_01_easy calibration")
    }

    /// End-to-end M1 checkpoint: synthetic 3D points, projected through the
    /// *real* MH_01 calibration (distortion included), round-tripped
    /// through undistortion + stereo triangulation, recovered to
    /// sub-millimeter accuracy.
    #[test]
    fn synthetic_points_survive_real_calibration_round_trip() {
        let cal = mh01_calibration();
        let cam0 = PinholeCamera::new(cal.cam0.intrinsics, cal.cam0.distortion_coefficients);
        let cam1 = PinholeCamera::new(cal.cam1.intrinsics, cal.cam1.distortion_coefficients);

        let t_bs_cam0 = SE3::from_matrix(&cal.cam0.t_bs);
        let t_bs_cam1 = SE3::from_matrix(&cal.cam1.t_bs);
        let t10 = t_bs_cam1.inverse().compose(&t_bs_cam0);

        for true_point_cam0 in [
            Vector3::new(0.2, -0.1, 3.0),
            Vector3::new(-0.4, 0.3, 5.0),
            Vector3::new(0.0, 0.0, 2.0),
            Vector3::new(0.6, 0.5, 4.0),
        ] {
            let true_point_cam1 = t10.transform(&true_point_cam0);

            let pixel0 = cam0.project(true_point_cam0);
            let pixel1 = cam1.project(true_point_cam1);

            let n0 = cam0.unproject_to_normalized(pixel0);
            let n1 = cam1.unproject_to_normalized(pixel1);

            let observations = [
                (SE3::identity(), n0),
                (t10, n1),
            ];
            let linear = triangulate_linear(&observations).expect("triangulation should succeed");
            let refined = triangulate_refine(&observations, linear, 5);

            assert_relative_eq!(refined, true_point_cam0, epsilon = 1e-6);
        }
    }

    #[test]
    fn real_rig_rectification_has_near_zero_vertical_disparity() {
        let cal = mh01_calibration();
        let rig = StereoRig {
            t_bs_cam0: SE3::from_matrix(&cal.cam0.t_bs),
            t_bs_cam1: SE3::from_matrix(&cal.cam1.t_bs),
            cam0: PinholeCamera::new(cal.cam0.intrinsics, cal.cam0.distortion_coefficients),
            cam1: PinholeCamera::new(cal.cam1.intrinsics, cal.cam1.distortion_coefficients),
        };
        let rect = rig.rectify();
        let t10 = rig.relative_pose_cam1_from_cam0();

        for p_cam0 in [
            Vector3::new(0.0, 0.0, 3.0),
            Vector3::new(1.0, -0.5, 6.0),
            Vector3::new(-0.8, 0.6, 4.0),
        ] {
            let p_cam1 = t10.transform(&p_cam0);
            let left = rect.project_left(p_cam0);
            let right = rect.project_right(p_cam1);
            assert!((left.y - right.y).abs() < 1e-9);
        }
    }
}
