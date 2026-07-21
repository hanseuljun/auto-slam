//! Vision frontend primitives (Stage 1 milestone M2): image pyramids,
//! grid-distributed FAST corner detection, and pyramidal Lucas-Kanade
//! optical flow. LK is the primary temporal tracker; a descriptor for
//! loop closure/relocalization comes later (M7), per
//! `plan/STAGE1.md`'s M2 recommendation.

mod fast;
mod lk;
mod pyramid;

pub use fast::{detect_fast, detect_grid, Keypoint};
pub use lk::{track_pyramid, LkParams, TrackResult};
pub use pyramid::ImagePyramid;

#[cfg(test)]
mod integration_tests {
    use super::*;
    use std::path::PathBuf;

    fn mh01_sequence() -> slam_dataset::EuRocSequence {
        let mav0 = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/machine_hall/MH_01_easy/mav0");
        slam_dataset::EuRocSequence::load(mav0).expect("load MH_01_easy")
    }

    /// M2's checkpoint test: detect real features on a real MH_01 frame,
    /// verify they're spread across the image (not clustered), then track
    /// a handful of them across several consecutive real frames and check
    /// the tracks survive and stay spatially coherent.
    #[test]
    fn tracks_real_features_across_consecutive_mh01_frames() {
        let seq = mh01_sequence();
        let num_frames = 4;
        let frames: Vec<image::GrayImage> = (0..num_frames)
            .map(|i| seq.load_cam0_image(i).expect("decode frame"))
            .collect();

        let keypoints = detect_grid(&frames[0], 20, 40, 3);
        assert!(
            keypoints.len() > 50,
            "expected plenty of features on a real frame, got {}",
            keypoints.len()
        );
        assert_no_clustering(&keypoints, frames[0].width(), frames[0].height());

        let pyramids: Vec<ImagePyramid> = frames.iter().map(|f| ImagePyramid::build(f, 4)).collect();
        let params = LkParams::default();

        let mut positions: Vec<(f32, f32)> = keypoints.iter().take(30).map(|k| (k.x, k.y)).collect();
        let mut survived = positions.len();

        for i in 1..num_frames {
            let results = track_pyramid(&pyramids[i - 1], &pyramids[i], &positions, &params);
            let mut next_positions = Vec::new();
            for r in &results {
                if r.found && r.x >= 0.0 && r.y >= 0.0 && r.x < frames[i].width() as f32 && r.y < frames[i].height() as f32 {
                    next_positions.push((r.x, r.y));
                }
            }
            survived = next_positions.len();
            positions = next_positions;
        }

        // MH_01 is a slow, deliberate indoor flight at 20Hz — most tracks
        // over a handful of frame-to-frame steps should survive with this
        // basic (no forward-backward check, no outlier gating yet) KLT
        // tracker. Robustifying track survival further is M6's job
        // ("track loss recovery", "outlier rejection"), not M2's.
        assert!(
            survived * 2 >= keypoints.len().min(30),
            "too few tracks survived: {survived}/{} over {num_frames} frames",
            keypoints.len().min(30)
        );
    }

    /// Sanity check that grid-based NMS actually distributes keypoints: no
    /// single cell should hold a wildly disproportionate share.
    fn assert_no_clustering(keypoints: &[Keypoint], width: u32, height: u32) {
        let cell = 80u32;
        let cols = width.div_ceil(cell) as usize;
        let rows = height.div_ceil(cell) as usize;
        let mut counts = vec![0usize; cols * rows];
        for kp in keypoints {
            let col = (kp.x as u32 / cell) as usize;
            let row = (kp.y as u32 / cell) as usize;
            counts[row * cols + col.min(cols - 1)] += 1;
        }
        let max_count = *counts.iter().max().unwrap();
        let occupied_cells = counts.iter().filter(|&&c| c > 0).count();
        assert!(
            occupied_cells > counts.len() / 2,
            "keypoints concentrated in too few cells: {occupied_cells}/{}",
            counts.len()
        );
        assert!(
            (max_count as f32) < keypoints.len() as f32 * 0.3,
            "one cell holds a disproportionate share: {max_count}/{}",
            keypoints.len()
        );
    }
}
