use image::GrayImage;
use nalgebra::{Vector2, Vector3};
use slam_core::SE3;
use slam_frontend::{match_stereo_keypoints, StereoMatchParams};
use slam_geometry::{StereoRectification, StereoRig};
use slam_vision::{compute_descriptor, detect_grid, Descriptor};

/// A loop-closure-ready keyframe: stereo-matched (real-depth, real-scale)
/// landmarks with a BRIEF descriptor for each, plus the pose that
/// triangulated them. Parallel arrays (`landmarks_world[i]` /
/// `normalized[i]` / `descriptors[i]` all describe the same point).
pub struct LoopKeyframe {
    pub keyframe_id: usize,
    pub timestamp_ns: u64,
    pub pose_world_to_cam0: SE3,
    pub landmarks_world: Vec<Vector3<f64>>,
    pub normalized: Vec<Vector2<f64>>,
    pub descriptors: Vec<Descriptor>,
}

#[derive(Debug, Clone, Copy)]
pub struct CaptureParams {
    pub fast_threshold: u8,
    pub grid_cell_size: u32,
    pub max_keypoints_per_cell: usize,
    pub stereo: StereoMatchParams,
}

impl Default for CaptureParams {
    fn default() -> Self {
        CaptureParams {
            fast_threshold: 20,
            grid_cell_size: 40,
            max_keypoints_per_cell: 3,
            stereo: StereoMatchParams::default(),
        }
    }
}

/// The bookkeeping (as opposed to image/calibration) inputs to
/// `capture_loop_keyframe` — grouped so the function doesn't take an
/// unwieldy number of positional arguments.
#[derive(Debug, Clone, Copy)]
pub struct KeyframeMeta {
    pub keyframe_id: usize,
    pub timestamp_ns: u64,
    pub pose_world_to_cam0: SE3,
}

/// Detects, stereo-matches, and describes a fresh set of landmarks at
/// `(left, right)` — independent of whatever tracking/landmark state a
/// VO/VIO pipeline is separately maintaining. Used to build the keyframe
/// database loop closure queries against.
pub fn capture_loop_keyframe(left: &GrayImage, right: &GrayImage, meta: KeyframeMeta, rig: &StereoRig, rect: &StereoRectification, params: &CaptureParams) -> LoopKeyframe {
    let KeyframeMeta { keyframe_id, timestamp_ns, pose_world_to_cam0 } = meta;
    let keypoints = detect_grid(left, params.fast_threshold, params.grid_cell_size, params.max_keypoints_per_cell);
    let matches = match_stereo_keypoints(left, right, &keypoints, rig, rect, &params.stereo);
    let pose_cam0_to_world = pose_world_to_cam0.inverse();

    let mut landmarks_world = Vec::new();
    let mut normalized = Vec::new();
    let mut descriptors = Vec::new();
    for m in matches {
        if let Some(d) = compute_descriptor(left, m.left_pixel.0, m.left_pixel.1) {
            landmarks_world.push(pose_cam0_to_world.transform(&m.point_cam0));
            normalized.push(rig.cam0.unproject_to_normalized(Vector2::new(m.left_pixel.0 as f64, m.left_pixel.1 as f64)));
            descriptors.push(d);
        }
    }

    LoopKeyframe {
        keyframe_id,
        timestamp_ns,
        pose_world_to_cam0,
        landmarks_world,
        normalized,
        descriptors,
    }
}
