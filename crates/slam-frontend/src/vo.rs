use image::GrayImage;
use nalgebra::{Vector2, Vector3};
use slam_core::SE3;
use slam_geometry::{estimate_pose_dlt, refine_pose_gauss_newton, StereoRectification, StereoRig};
use slam_vision::{detect_grid, track_pyramid, ImagePyramid, LkParams};

use crate::stereo::{match_stereo_keypoints, StereoMatchParams};

#[derive(Debug, Clone, Copy)]
pub struct VoParams {
    pub fast_threshold: u8,
    pub grid_cell_size: u32,
    pub max_keypoints_per_cell: usize,
    /// A new stereo keypoint is skipped if it lands within this many pixels
    /// of an already-tracked point, to avoid piling up near-duplicate
    /// landmarks every keyframe.
    pub min_new_landmark_pixel_distance: f32,
    /// Trigger a new keyframe (fresh stereo matching) once the live track
    /// count drops below this.
    pub min_tracks_before_keyframe: usize,
    pub min_tracks_for_pose: usize,
    /// Reject a PnP pose outright (treat as track loss, triggering
    /// recovery) if it implies a per-frame translation jump larger than
    /// this. DLT+refine has no RANSAC/outlier rejection
    /// (`decisions/0003`) and, on a long real run, occasionally produces
    /// a catastrophically wrong pose (observed: translations in the
    /// billions of meters) from a degenerate point configuration while
    /// still numerically "succeeding" — this catches that before it
    /// corrupts the trajectory and every landmark triangulated
    /// afterward. 2m/frame is already far beyond any plausible MAV motion
    /// at these datasets' ~20Hz frame rate.
    pub max_pose_jump_meters: f64,
    pub lk: LkParams,
    pub stereo: StereoMatchParams,
}

impl Default for VoParams {
    fn default() -> Self {
        VoParams {
            fast_threshold: 20,
            grid_cell_size: 40,
            max_keypoints_per_cell: 3,
            min_new_landmark_pixel_distance: 12.0,
            min_tracks_before_keyframe: 120,
            min_tracks_for_pose: 6,
            max_pose_jump_meters: 2.0,
            lk: LkParams::default(),
            stereo: StereoMatchParams::default(),
        }
    }
}

struct Track {
    pixel: (f32, f32),
    landmark_id: usize,
}

/// A minimal stereo-only (no IMU) visual odometry pipeline: stereo-matches
/// and triangulates a landmark map, tracks it frame-to-frame with pyramidal
/// LK, and estimates each frame's pose via PnP (DLT + Gauss-Newton refine)
/// against the known landmarks. This is M3's checkpoint system — a full
/// sliding-window VIO backend with marginalization comes in M5.
pub struct VoPipeline {
    rig: StereoRig,
    rect: StereoRectification,
    params: VoParams,
    landmarks: Vec<Vector3<f64>>,
    tracks: Vec<Track>,
    prev_pyramid: Option<ImagePyramid>,
    /// The last successfully (or recovered-to) estimated pose — the
    /// anchor `process_frame` resets the local map around on track loss.
    last_pose: SE3,
}

#[derive(Debug, Clone, Copy)]
pub struct FrameResult {
    /// Maps a world-frame point to this frame's cam0 frame:
    /// `p_cam0 = pose.transform(p_world)`. World frame is the first
    /// processed frame's cam0 frame.
    pub pose_world_to_cam0: SE3,
    pub num_tracked: usize,
    pub is_keyframe: bool,
    /// `true` if this frame's pose is a track-loss recovery (map reset
    /// anchored at the last known pose, not a freshly-estimated one) —
    /// see `process_frame`'s doc comment.
    pub recovered: bool,
}

impl VoPipeline {
    pub fn new(rig: StereoRig, params: VoParams) -> Self {
        let rect = rig.rectify();
        VoPipeline {
            rig,
            rect,
            params,
            landmarks: Vec::new(),
            tracks: Vec::new(),
            prev_pyramid: None,
            last_pose: SE3::identity(),
        }
    }

    pub fn num_landmarks(&self) -> usize {
        self.landmarks.len()
    }

    /// Initializes the map from the first stereo pair; the world frame is
    /// defined as this frame's cam0 frame (identity pose).
    pub fn init(&mut self, left: &GrayImage, right: &GrayImage) -> FrameResult {
        self.add_new_landmarks(left, right, &SE3::identity());
        self.prev_pyramid = Some(ImagePyramid::build(left, 4));
        FrameResult {
            pose_world_to_cam0: SE3::identity(),
            num_tracked: self.tracks.len(),
            is_keyframe: true,
            recovered: false,
        }
    }

    /// Tracks into the next stereo pair and estimates its pose. If too few
    /// tracks survive or PnP fails (degenerate point configuration), this
    /// is track loss — rather than failing permanently, the local map is
    /// reset (fresh stereo-matched landmarks) anchored at the last known
    /// pose (`FrameResult::recovered = true`), so a temporary bad frame
    /// (motion blur, a textureless view) doesn't end the run. Returns
    /// `None` only if recovery itself finds nothing to anchor to (e.g. a
    /// genuinely blank/unmatchable frame) — callers should keep calling
    /// `process_frame` on subsequent frames rather than treating that as
    /// fatal, since a later frame may recover.
    pub fn process_frame(&mut self, left: &GrayImage, right: &GrayImage) -> Option<FrameResult> {
        let prev_pyramid = self.prev_pyramid.as_ref().expect("init() must be called first");
        let curr_pyramid = ImagePyramid::build(left, 4);

        let prev_positions: Vec<(f32, f32)> = self.tracks.iter().map(|t| t.pixel).collect();
        let results = track_pyramid(prev_pyramid, &curr_pyramid, &prev_positions, &self.params.lk);

        let (w, h) = (left.width() as f32, left.height() as f32);
        let mut surviving = Vec::with_capacity(self.tracks.len());
        for (track, r) in self.tracks.iter().zip(results.iter()) {
            if r.found && r.x >= 0.0 && r.y >= 0.0 && r.x < w && r.y < h {
                surviving.push(Track {
                    pixel: (r.x, r.y),
                    landmark_id: track.landmark_id,
                });
            }
        }
        self.prev_pyramid = Some(curr_pyramid);

        let estimated_pose = if surviving.len() >= self.params.min_tracks_for_pose {
            let points_world: Vec<Vector3<f64>> = surviving.iter().map(|t| self.landmarks[t.landmark_id]).collect();
            let observations: Vec<Vector2<f64>> = surviving
                .iter()
                .map(|t| self.rig.cam0.unproject_to_normalized(Vector2::new(t.pixel.0 as f64, t.pixel.1 as f64)))
                .collect();
            estimate_pose_dlt(&points_world, &observations)
                .map(|initial| refine_pose_gauss_newton(&points_world, &observations, initial, 10))
                .filter(|pose| {
                    let jump = (pose.inverse().translation - self.last_pose.inverse().translation).norm();
                    jump.is_finite() && jump < self.params.max_pose_jump_meters
                })
        } else {
            None
        };

        if let Some(pose) = estimated_pose {
            self.tracks = surviving;
            self.last_pose = pose;

            let is_keyframe = self.tracks.len() < self.params.min_tracks_before_keyframe;
            if is_keyframe {
                self.add_new_landmarks(left, right, &pose);
            }

            return Some(FrameResult {
                pose_world_to_cam0: pose,
                num_tracked: self.tracks.len(),
                is_keyframe,
                recovered: false,
            });
        }

        // Track loss: reset the local map anchored at the last known pose.
        self.tracks.clear();
        let anchor = self.last_pose;
        self.add_new_landmarks(left, right, &anchor);
        if self.tracks.is_empty() {
            return None;
        }
        Some(FrameResult {
            pose_world_to_cam0: self.last_pose,
            num_tracked: self.tracks.len(),
            is_keyframe: true,
            recovered: true,
        })
    }

    fn add_new_landmarks(&mut self, left: &GrayImage, right: &GrayImage, pose_world_to_cam0: &SE3) {
        let keypoints = detect_grid(
            left,
            self.params.fast_threshold,
            self.params.grid_cell_size,
            self.params.max_keypoints_per_cell,
        );
        let min_dist2 = self.params.min_new_landmark_pixel_distance * self.params.min_new_landmark_pixel_distance;
        let filtered: Vec<_> = keypoints
            .into_iter()
            .filter(|kp| {
                self.tracks.iter().all(|t| {
                    let dx = t.pixel.0 - kp.x;
                    let dy = t.pixel.1 - kp.y;
                    dx * dx + dy * dy > min_dist2
                })
            })
            .collect();

        let matches = match_stereo_keypoints(left, right, &filtered, &self.rig, &self.rect, &self.params.stereo);
        let pose_cam0_to_world = pose_world_to_cam0.inverse();
        for m in matches {
            let point_world = pose_cam0_to_world.transform(&m.point_cam0);
            let landmark_id = self.landmarks.len();
            self.landmarks.push(point_world);
            self.tracks.push(Track {
                pixel: m.left_pixel,
                landmark_id,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use slam_geometry::PinholeCamera;
    use std::path::PathBuf;

    fn mh01_sequence() -> slam_dataset::EuRocSequence {
        let mav0 = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../data/machine_hall/MH_01_easy/mav0");
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

    /// M6's track-loss-recovery checkpoint: a real MH_01 clip, with one
    /// frame forcibly replaced by a blank (textureless) image partway
    /// through — no LK track survives a blank frame, and a blank frame
    /// has no FAST corners to re-anchor to either, so that specific frame
    /// is genuinely unrecoverable (`process_frame` returns `None`, as
    /// documented). The real check is what happens *next*: the following
    /// real frame should trigger recovery (`FrameResult::recovered ==
    /// true`) using the last known good pose as an anchor, and tracking
    /// should continue normally afterward — not stay permanently lost.
    #[test]
    fn recovers_from_a_forced_blank_frame_instead_of_failing_permanently() {
        let seq = mh01_sequence();
        let rig = stereo_rig(&seq.calibration);
        let mut vo = VoPipeline::new(rig, VoParams::default());

        let left0 = seq.load_cam0_image(0).unwrap();
        let right0 = seq.load_cam1_image(0).unwrap();
        vo.init(&left0, &right0);

        // A few normal frames first, to build up real tracking state.
        for i in 1..5 {
            let left = seq.load_cam0_image(i).unwrap();
            let right = seq.load_cam1_image(i).unwrap();
            let result = vo.process_frame(&left, &right).expect("normal frames should track fine");
            assert!(!result.recovered);
        }
        let pose_before_loss = vo.last_pose;

        // Force track loss: independent random noise for left/right. A
        // uniform/blank frame turned out *not* to reliably do this on
        // real images — some real image patches are naturally near-black
        // (shadows, dark machinery), so a patch's residual against a
        // blank frame can coincidentally fall under the rejection
        // threshold even though nothing meaningful matched. Independent
        // random noise avoids that: no real image patch resembles noise,
        // and stereo-matching noise against *independently generated*
        // noise (not the same buffer for both eyes, which would trivially
        // "match" itself at zero disparity) has no genuine correlation to
        // exploit either, so recovery's stereo match should also fail to
        // find anything to re-anchor to.
        let random_noise_image = |seed: u32| -> GrayImage {
            let mut img = GrayImage::new(752, 480);
            let mut state = seed;
            for p in img.pixels_mut() {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                *p = image::Luma([(state >> 24) as u8]);
            }
            img
        };
        let noise_left = random_noise_image(1);
        let noise_right = random_noise_image(2);
        let blank_result = vo.process_frame(&noise_left, &noise_right);
        assert!(blank_result.is_none(), "independent random noise should be genuinely unrecoverable on its own");
        assert!(vo.tracks.is_empty());

        // The next real frame should trigger recovery, not stay lost.
        let left_after = seq.load_cam0_image(5).unwrap();
        let right_after = seq.load_cam1_image(5).unwrap();
        let recovered = vo.process_frame(&left_after, &right_after).expect("should recover on the next real frame");
        assert!(recovered.recovered);
        assert!(recovered.num_tracked > 0);
        assert_relative_eq_se3(&recovered.pose_world_to_cam0, &pose_before_loss);

        // Tracking should continue normally afterward.
        for i in 6..10 {
            let left = seq.load_cam0_image(i).unwrap();
            let right = seq.load_cam1_image(i).unwrap();
            let result = vo.process_frame(&left, &right).expect("should keep tracking after recovery");
            assert!(!result.recovered);
        }
    }

    fn assert_relative_eq_se3(a: &SE3, b: &SE3) {
        approx::assert_relative_eq!(a.matrix(), b.matrix(), epsilon = 1e-9);
    }
}
