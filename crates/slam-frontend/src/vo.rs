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
}

#[derive(Debug, Clone, Copy)]
pub struct FrameResult {
    /// Maps a world-frame point to this frame's cam0 frame:
    /// `p_cam0 = pose.transform(p_world)`. World frame is the first
    /// processed frame's cam0 frame.
    pub pose_world_to_cam0: SE3,
    pub num_tracked: usize,
    pub is_keyframe: bool,
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
        }
    }

    /// Tracks into the next stereo pair, estimates its pose, and — if the
    /// live track count has dropped too far — stereo-matches fresh
    /// keypoints as a new keyframe. Returns `None` if too few points
    /// survived to estimate a pose (track loss; recovery is M6's job).
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

        if surviving.len() < self.params.min_tracks_for_pose {
            return None;
        }

        let points_world: Vec<Vector3<f64>> = surviving.iter().map(|t| self.landmarks[t.landmark_id]).collect();
        let observations: Vec<Vector2<f64>> = surviving
            .iter()
            .map(|t| self.rig.cam0.unproject_to_normalized(Vector2::new(t.pixel.0 as f64, t.pixel.1 as f64)))
            .collect();

        let initial_pose = estimate_pose_dlt(&points_world, &observations)?;
        let pose = refine_pose_gauss_newton(&points_world, &observations, initial_pose, 10);

        self.tracks = surviving;
        self.prev_pyramid = Some(curr_pyramid);

        let is_keyframe = self.tracks.len() < self.params.min_tracks_before_keyframe;
        if is_keyframe {
            self.add_new_landmarks(left, right, &pose);
        }

        Some(FrameResult {
            pose_world_to_cam0: pose,
            num_tracked: self.tracks.len(),
            is_keyframe,
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
