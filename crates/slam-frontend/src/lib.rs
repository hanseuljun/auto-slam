//! Stereo visual tracking, keyframe selection, and static/dynamic VI
//! initialization (Stage 1 milestones M3-M4). M3: stereo matching
//! (`stereo`) + a stereo-only (no IMU) VO pipeline (`vo`) — the first
//! end-to-end accuracy checkpoint. M4: the dynamic (moving-start)
//! vision-IMU alignment initializer (`vi_init`), for MH_04/05.

mod stereo;
mod vi_init;
mod vo;

pub use stereo::{match_stereo_keypoints, StereoMatch, StereoMatchParams};
pub use vi_init::{dynamic_initialize, DynamicInitResult, VoKeyframe};
pub use vo::{FrameResult, VoParams, VoPipeline};

#[cfg(test)]
mod integration_tests {
    use super::*;
    use nalgebra::Vector3;
    use slam_core::SE3;
    use slam_geometry::{PinholeCamera, StereoRig};
    use std::path::PathBuf;

    fn mh01_sequence() -> slam_dataset::EuRocSequence {
        load_sequence("MH_01_easy")
    }

    fn load_sequence(name: &str) -> slam_dataset::EuRocSequence {
        let mav0 = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join(format!("../../data/machine_hall/{name}/mav0"));
        slam_dataset::EuRocSequence::load(mav0).unwrap_or_else(|e| panic!("load {name}: {e}"))
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

    /// M4's dynamic-initializer checkpoint: MH_04_difficult never settles
    /// into a stationary window (see `memory/notes/dataset-quirks.md`), so
    /// it needs `dynamic_initialize` rather than
    /// `slam_imu::static_initialize`. Runs stereo VO over the first few
    /// real seconds to get keyframe poses, feeds those + the raw IMU into
    /// the initializer, and checks it converges to a plausible gravity
    /// vector within that short window — "converges within the first few
    /// seconds" per `plan/STAGE1.md` M4, not a tight accuracy bound (this
    /// is real, noisy VO + IMU, and accel bias is fixed at zero by design
    /// — see `vi_init::solve_gravity_bias_velocity`'s doc comment).
    #[test]
    fn dynamic_initializer_converges_on_mh04_moving_start() {
        let seq = load_sequence("MH_04_difficult");
        let rig = stereo_rig(&seq.calibration);
        let t_bs_cam0 = rig.t_bs_cam0;
        let mut vo = VoPipeline::new(rig, VoParams::default());

        let num_frames = 100usize.min(seq.cam0_frames.len()); // ~5s at 20Hz
        let left0 = seq.load_cam0_image(0).unwrap();
        let right0 = seq.load_cam1_image(0).unwrap();
        vo.init(&left0, &right0);

        let mut keyframes = vec![VoKeyframe {
            timestamp_ns: seq.cam0_frames[0].timestamp_ns,
            pose_world_to_cam0: SE3::identity(),
        }];
        // Sample VO poses at a regular interval as "keyframes" for the
        // initializer — it just needs reasonably spaced poses + their
        // timestamps, not the VoPipeline's own (denser) keyframe selection.
        let keyframe_stride = 10; // 0.5s at 20Hz
        for i in 1..num_frames {
            let left = seq.load_cam0_image(i).unwrap();
            let right = seq.load_cam1_image(i).unwrap();
            let Some(result) = vo.process_frame(&left, &right) else {
                break;
            };
            if i % keyframe_stride == 0 {
                keyframes.push(VoKeyframe {
                    timestamp_ns: seq.cam0_frames[i].timestamp_ns,
                    pose_world_to_cam0: result.pose_world_to_cam0,
                });
            }
        }
        assert!(keyframes.len() >= 4, "expected enough keyframes to initialize, got {}", keyframes.len());

        let result = dynamic_initialize(&keyframes, &seq.imu_samples, &t_bs_cam0).expect("dynamic initializer should converge");
        println!(
            "MH_04 dynamic init over {} keyframes ({:.1}s): gravity={:?} |g|={:.3} gyro_bias={:?}",
            keyframes.len(),
            (keyframes.last().unwrap().timestamp_ns - keyframes[0].timestamp_ns) as f64 * 1e-9,
            result.gravity_world,
            result.gravity_world.norm(),
            result.gyro_bias
        );

        assert!(
            (result.gravity_world.norm() - 9.81).abs() < 2.0,
            "gravity magnitude implausible for a converged initializer: {}",
            result.gravity_world.norm()
        );
        assert!(
            result.gyro_bias.norm() < 1.0,
            "gyro bias implausibly large: {}",
            result.gyro_bias
        );
    }
}
