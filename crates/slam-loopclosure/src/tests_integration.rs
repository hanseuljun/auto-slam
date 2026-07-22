#[cfg(test)]
mod tests {
    use slam_core::SE3;
    use slam_dataset::EuRocSequence;
    use slam_frontend::{VoParams, VoPipeline};
    use slam_geometry::{PinholeCamera, StereoRig};
    use std::path::PathBuf;

    use crate::{capture_loop_keyframe, optimize_pose_graph, verify_loop_candidate, CaptureParams, GeometricVerificationParams, KeyframeDatabase, KeyframeMeta, PoseGraphEdge, Vocabulary};

    fn load_sequence(name: &str) -> EuRocSequence {
        let mav0 = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(format!("../../data/machine_hall/{name}/mav0"));
        EuRocSequence::load(mav0).unwrap_or_else(|e| panic!("load {name}: {e}"))
    }

    fn stereo_rig(cal: &slam_dataset::Calibration) -> StereoRig {
        StereoRig {
            t_bs_cam0: SE3::from_matrix(&cal.cam0.t_bs),
            t_bs_cam1: SE3::from_matrix(&cal.cam1.t_bs),
            cam0: PinholeCamera::new(cal.cam0.intrinsics, cal.cam0.distortion_coefficients),
            cam1: PinholeCamera::new(cal.cam1.intrinsics, cal.cam1.distortion_coefficients),
        }
    }

    /// M7's checkpoint (`plan/STAGE1.md`): "MH_05 (has a loop) shows
    /// measurable ATE improvement with loop closure enabled vs.
    /// disabled." Runs stereo VO over the full real MH_05 sequence
    /// (confirmed via groundtruth: it revisits its start position, within
    /// 0.15m, at t=111s after ~98m of travel — the loop is at the very
    /// end, not reachable from a short clip), independently captures
    /// loop-closure-ready keyframes (stereo-matched landmarks + BRIEF
    /// descriptors) at a fixed stride, trains a vocabulary, queries for
    /// the revisit, geometrically verifies it, and compares ATE with vs.
    /// without the resulting pose-graph optimization.
    ///
    /// `#[ignore]`d like M6's full-sequence test (`slam-frontend`'s
    /// `full_sequence_runs_survive_...`): a full 2273-frame VO run plus
    /// vocabulary training is genuinely expensive (~40s release, ~15min
    /// debug — confirmed, not a guess). Run explicitly with
    /// `cargo test -p slam-loopclosure --release -- --ignored --nocapture`
    /// after any change to VO, descriptor matching, or pose-graph code;
    /// `bin/slam-inspect` also exercises this same path (MH_05 only) on
    /// every normal run, so day-to-day regressions surface there too.
    #[test]
    #[ignore = "expensive: full 2273-frame VO run + vocabulary training, ~40s release/~15min debug"]
    fn loop_closure_measurably_improves_ate_on_mh05() {
        let seq = load_sequence("MH_05_difficult");
        let rig = stereo_rig(&seq.calibration);
        let mut vo = VoPipeline::new(rig.clone(), VoParams::default());
        let rect = rig.rectify();

        let num_frames = seq.cam0_frames.len().min(seq.cam1_frames.len());
        let stride = 20usize; // ~1s at 20Hz between pose-graph nodes

        let left0 = seq.load_cam0_image(0).unwrap();
        let right0 = seq.load_cam1_image(0).unwrap();
        vo.init(&left0, &right0);

        // Pose-graph nodes: VO's own pose at each stride point.
        let mut node_timestamps = vec![seq.cam0_frames[0].timestamp_ns];
        let mut vo_poses = vec![SE3::identity()];
        let meta0 = KeyframeMeta { keyframe_id: 0, timestamp_ns: seq.cam0_frames[0].timestamp_ns, pose_world_to_cam0: SE3::identity() };
        let mut keyframes = vec![capture_loop_keyframe(&left0, &right0, meta0, &rig, &rect, &CaptureParams::default())];

        let mut lost_at = None;
        for i in 1..num_frames {
            let left = seq.load_cam0_image(i).unwrap();
            let right = seq.load_cam1_image(i).unwrap();
            match vo.process_frame(&left, &right) {
                Some(result) => {
                    if i % stride == 0 {
                        let node_id = node_timestamps.len();
                        node_timestamps.push(seq.cam0_frames[i].timestamp_ns);
                        vo_poses.push(result.pose_world_to_cam0);
                        let meta = KeyframeMeta { keyframe_id: node_id, timestamp_ns: seq.cam0_frames[i].timestamp_ns, pose_world_to_cam0: result.pose_world_to_cam0 };
                        keyframes.push(capture_loop_keyframe(&left, &right, meta, &rig, &rect, &CaptureParams::default()));
                    }
                }
                None => {
                    lost_at = Some(i);
                    break;
                }
            }
        }
        assert!(lost_at.is_none(), "VO tracking lost at frame {:?}", lost_at);
        assert!(keyframes.len() >= 40, "expected enough pose-graph nodes, got {}", keyframes.len());

        // Train a vocabulary on every descriptor gathered.
        let all_descriptors: Vec<_> = keyframes.iter().flat_map(|k| k.descriptors.iter().copied()).collect();
        assert!(all_descriptors.len() > 500, "expected plenty of descriptors, got {}", all_descriptors.len());
        let vocab = Vocabulary::train(&all_descriptors, 300, 6, 17);

        let mut db = KeyframeDatabase::new();
        for kf in &keyframes {
            db.insert(kf.keyframe_id, vocab.compute_bow(&kf.descriptors));
        }

        // Query every node for a loop candidate far enough back in the
        // trajectory, geometrically verify the best one found.
        let min_id_gap = 30;
        let mut best_loop: Option<(usize, usize, crate::VerifiedLoop)> = None;
        for kf in &keyframes {
            let query_bow = vocab.compute_bow(&kf.descriptors);
            let Some((candidate_id, _similarity)) = db.query(kf.keyframe_id, &query_bow, min_id_gap, 0.3) else {
                continue;
            };
            let candidate = &keyframes[candidate_id];
            let Some(verified) = verify_loop_candidate(&kf.normalized, &kf.descriptors, &candidate.descriptors, &candidate.landmarks_world, &GeometricVerificationParams::default()) else {
                continue;
            };
            let better = best_loop.as_ref().map(|(_, _, v)| verified.num_inliers > v.num_inliers).unwrap_or(true);
            if better {
                best_loop = Some((kf.keyframe_id, candidate_id, verified));
            }
        }
        let (current_id, candidate_id, verified) = best_loop.expect("expected MH_05's loop to be detected and verified");
        println!("loop closure found: keyframe {current_id} <-> keyframe {candidate_id}, {} inliers", verified.num_inliers);

        // Build the pose graph: odometry edges between consecutive nodes
        // (trusting VO's own relative poses) plus the verified loop edge.
        let mut edges = Vec::new();
        for i in 0..vo_poses.len() - 1 {
            let relative = vo_poses[i + 1].compose(&vo_poses[i].inverse());
            edges.push(PoseGraphEdge { i, j: i + 1, relative_pose: relative, weight: 1.0 });
        }
        // verify_loop_candidate's `relative_pose` is a PnP result against
        // candidate_landmarks_world, which already live in the *global*
        // rolling VO world frame — i.e. it's an independent *absolute*
        // pose estimate for the current keyframe, not a relative
        // transform between the two nodes. Converting it into a proper
        // relative edge uses the candidate's own (pre-optimization) VO
        // pose as the reference frame.
        let relative_pose = verified.relative_pose.compose(&vo_poses[candidate_id].inverse());
        edges.push(PoseGraphEdge {
            i: candidate_id,
            j: current_id,
            relative_pose,
            // Needs to be large relative to the *sum* of odometry weights
            // (~110 edges at weight 1.0 each here), not just any single
            // edge, to meaningfully redistribute correction across the
            // whole chain.
            weight: 5000.0,
        });

        let gt_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../data/machine_hall/MH_05_difficult/mav0/state_groundtruth_estimate0/data.csv");
        let gt = slam_eval::GroundTruthTrajectory::load(gt_path).expect("load groundtruth");
        let ate_of = |poses: &[SE3]| -> Option<f64> {
            let mut est = Vec::new();
            let mut truth = Vec::new();
            for (t, p) in node_timestamps.iter().zip(poses.iter()) {
                if let Some(pose) = gt.interpolate(*t) {
                    est.push(p.inverse().translation);
                    truth.push(pose.position);
                }
            }
            slam_eval::compute_ate(&est, &truth).map(|s| s.rmse)
        };

        let ate_without_loop_closure = ate_of(&vo_poses).expect("ATE should compute without loop closure");

        let mut optimized_poses = vo_poses.clone();
        optimize_pose_graph(&mut optimized_poses, &edges, 0, 50);
        let ate_with_loop_closure = ate_of(&optimized_poses).expect("ATE should compute with loop closure");

        println!("ATE without loop closure: {ate_without_loop_closure:.3}m, with: {ate_with_loop_closure:.3}m");
        assert!(
            ate_with_loop_closure < ate_without_loop_closure * 0.9,
            "expected loop closure to measurably improve ATE: without={ate_without_loop_closure:.3}m with={ate_with_loop_closure:.3}m"
        );
    }
}
