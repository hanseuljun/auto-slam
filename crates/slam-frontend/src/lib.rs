//! Stereo visual tracking, keyframe selection, and static/dynamic VI
//! initialization (Stage 1 milestones M3-M4). M3: stereo matching
//! (`stereo`) + a stereo-only (no IMU) VO pipeline (`vo`) — the first
//! end-to-end accuracy checkpoint. M4's IMU-based initializer lands later,
//! once `slam-imu` exists.

mod stereo;
mod vo;

pub use stereo::{match_stereo_keypoints, StereoMatch, StereoMatchParams};
pub use vo::{FrameResult, VoParams, VoPipeline};

#[cfg(test)]
mod integration_tests {
    use super::*;
    use nalgebra::Vector3;
    use slam_core::SE3;
    use slam_geometry::{PinholeCamera, StereoRig};
    use std::path::PathBuf;

    fn mh01_sequence() -> slam_dataset::EuRocSequence {
        let mav0 = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/machine_hall/MH_01_easy/mav0");
        slam_dataset::EuRocSequence::load(mav0).expect("load MH_01_easy")
    }

    fn stereo_rig(cal: &slam_dataset::Calibration) -> StereoRig {
        StereoRig {
            t_bs_cam0: SE3::from_matrix(&cal.cam0.t_bs),
            t_bs_cam1: SE3::from_matrix(&cal.cam1.t_bs),
            cam0: PinholeCamera::new(cal.cam0.intrinsics, cal.cam0.distortion_coefficients),
            cam1: PinholeCamera::new(cal.cam1.intrinsics, cal.cam1.distortion_coefficients),
        }
    }

    /// M3's checkpoint: run stereo-only VO on a real clip of MH_01_easy,
    /// align the estimated trajectory onto ground truth via Umeyama, and
    /// check ATE lands in a plausible range for a VO-only (no IMU, no
    /// backend optimization, no loop closure) system. This is explicitly
    /// *not* the SOTA VIO accuracy bar from `plan/STAGE1.md` — that's M5+
    /// once IMU fusion and the backend exist; this checkpoint exists to
    /// prove the frontend produces a geometrically sane trajectory at all.
    #[test]
    fn stereo_vo_tracks_a_plausible_trajectory_on_mh01() {
        let seq = mh01_sequence();
        let rig = stereo_rig(&seq.calibration);
        let mut vo = VoPipeline::new(rig, VoParams::default());

        let num_frames = 150usize.min(seq.cam0_frames.len());
        let left0 = seq.load_cam0_image(0).unwrap();
        let right0 = seq.load_cam1_image(0).unwrap();
        vo.init(&left0, &right0);
        assert!(vo.num_landmarks() > 50, "expected a real initial map, got {}", vo.num_landmarks());

        let mut estimated_positions = Vec::new();
        let mut timestamps = Vec::new();
        estimated_positions.push(Vector3::zeros());
        timestamps.push(seq.cam0_frames[0].timestamp_ns);

        let mut lost_at = None;
        for i in 1..num_frames {
            let left = seq.load_cam0_image(i).unwrap();
            let right = seq.load_cam1_image(i).unwrap();
            match vo.process_frame(&left, &right) {
                Some(result) => {
                    // pose maps world -> cam0_i; the camera's own position
                    // in world is the inverse transform's translation.
                    let cam_position_world = result.pose_world_to_cam0.inverse().translation;
                    estimated_positions.push(cam_position_world);
                    timestamps.push(seq.cam0_frames[i].timestamp_ns);
                }
                None => {
                    lost_at = Some(i);
                    break;
                }
            }
        }
        assert!(lost_at.is_none(), "tracking lost at frame {:?}", lost_at);
        assert_eq!(estimated_positions.len(), num_frames);

        let gt_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(
            "../../data/machine_hall/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv",
        );
        let gt = slam_eval::GroundTruthTrajectory::load(gt_path).expect("load groundtruth");

        let mut aligned_estimate = Vec::new();
        let mut aligned_groundtruth = Vec::new();
        for (t, p) in timestamps.iter().zip(estimated_positions.iter()) {
            if let Some(pose) = gt.interpolate(*t) {
                aligned_estimate.push(*p);
                aligned_groundtruth.push(pose.position);
            }
        }
        assert!(aligned_estimate.len() > num_frames / 2, "too few timestamps had groundtruth coverage");

        let stats = slam_eval::compute_ate(&aligned_estimate, &aligned_groundtruth).expect("ATE should compute");
        println!(
            "stereo VO-only ATE over {} frames: rmse={:.3}m mean={:.3}m median={:.3}m max={:.3}m",
            aligned_estimate.len(),
            stats.rmse,
            stats.mean,
            stats.median,
            stats.max
        );
        // Generous bound for a VO-only, backend-free, IMU-free frontend
        // over a short clip: proves the pipeline is geometrically sane
        // (not diverging/garbage), not that it's SOTA — see doc comment.
        assert!(stats.rmse < 1.0, "VO-only ATE RMSE unexpectedly large: {}", stats.rmse);
    }
}
