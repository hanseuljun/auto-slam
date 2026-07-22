use std::collections::{HashMap, VecDeque};

use image::GrayImage;
use nalgebra::{Vector2, Vector3};
use slam_core::SE3;
use slam_dataset::ImuSample;
use slam_frontend::{match_stereo_keypoints, StereoMatchParams};
use slam_geometry::{estimate_pose_dlt, refine_pose_gauss_newton, StereoRectification, StereoRig};
use slam_imu::Preintegration;
use slam_optim::{optimize, BiasRwFactorSpec, ImuFactorSpec, KeyframeState, Problem, ReprojectionObservation, SolverConfig};
use slam_vision::{detect_grid, track_pyramid, ImagePyramid, LkParams};

#[derive(Debug, Clone, Copy)]
pub struct VioParams {
    pub keyframe_stride: usize,
    pub window_size: usize,
    pub fast_threshold: u8,
    pub grid_cell_size: u32,
    pub max_keypoints_per_cell: usize,
    pub min_new_landmark_pixel_distance: f32,
    pub lk: LkParams,
    pub stereo: StereoMatchParams,
    pub solver: SolverConfig,
}

impl Default for VioParams {
    fn default() -> Self {
        VioParams {
            keyframe_stride: 10,
            window_size: 8,
            fast_threshold: 20,
            grid_cell_size: 40,
            max_keypoints_per_cell: 3,
            min_new_landmark_pixel_distance: 12.0,
            lk: LkParams::default(),
            stereo: StereoMatchParams::default(),
            solver: SolverConfig::default(),
        }
    }
}

struct Track {
    pixel: (f32, f32),
    landmark_id: usize,
}

#[derive(Clone, Copy)]
enum Camera {
    Cam0,
    Cam1,
}

struct Observation {
    landmark_id: usize,
    camera: Camera,
    normalized: Vector2<f64>,
}

struct WindowKeyframe {
    timestamp_ns: u64,
    state: KeyframeState,
    observations: Vec<Observation>,
    /// The IMU factor connecting the *previous* window keyframe to this
    /// one (`None` only for the very first keyframe of the whole
    /// trajectory). Stored per-keyframe, not just for the newest pair, so
    /// every consecutive edge still inside the window is included every
    /// time `run_optimization` rebuilds the problem — dropping the oldest
    /// keyframe naturally drops its incoming edge too.
    imu_edge: Option<(Preintegration, f64)>,
}

#[derive(Debug, Clone, Copy)]
pub struct VioFrameResult {
    /// World -> body pose (`p_body = pose.transform(p_world)`), the most
    /// recently optimized state for the newest keyframe.
    pub pose_world_to_body: SE3,
    pub is_keyframe: bool,
    pub window_len: usize,
    pub num_landmarks: usize,
}

/// A sliding-window visual-inertial odometry pipeline: LK-tracks stereo-
/// matched landmarks frame-to-frame (reusing `slam_frontend`'s stereo
/// matching), and every `keyframe_stride` frames promotes the frame to a
/// keyframe — preintegrating the buffered raw IMU into an IMU factor,
/// adding reprojection factors for tracked/newly-triangulated landmarks,
/// and jointly optimizing the whole window via `slam_optim`.
///
/// The window is naive fixed-lag (oldest keyframe dropped when full, no
/// marginalization prior folding its information into the rest) — see
/// `memory/decisions` for why real marginalization was scoped out of this
/// first working version.
pub struct VioPipeline {
    rig: StereoRig,
    rect: StereoRectification,
    t_bs_cam0: SE3,
    gravity_world: Vector3<f64>,
    params: VioParams,

    landmarks: Vec<Vector3<f64>>,
    tracks: Vec<Track>,
    window: VecDeque<WindowKeyframe>,
    prev_pyramid: Option<ImagePyramid>,
    imu_buffer: Vec<ImuSample>,
    frame_index: usize,
}

impl VioPipeline {
    pub fn new(rig: StereoRig, initial_state: KeyframeState, initial_timestamp_ns: u64, gravity_world: Vector3<f64>, params: VioParams) -> Self {
        let rect = rig.rectify();
        let t_bs_cam0 = rig.t_bs_cam0;
        VioPipeline {
            rig,
            rect,
            t_bs_cam0,
            gravity_world,
            params,
            landmarks: Vec::new(),
            tracks: Vec::new(),
            window: {
                let mut w = VecDeque::new();
                w.push_back(WindowKeyframe {
                    timestamp_ns: initial_timestamp_ns,
                    state: initial_state,
                    observations: Vec::new(),
                    imu_edge: None,
                });
                w
            },
            prev_pyramid: None,
            imu_buffer: Vec::new(),
            frame_index: 0,
        }
    }

    pub fn init_map(&mut self, left: &GrayImage, right: &GrayImage) {
        let keyframe_idx = self.window.len() - 1;
        let state = self.window[keyframe_idx].state;
        self.add_new_landmarks(left, right, &state, keyframe_idx);
        self.prev_pyramid = Some(ImagePyramid::build(left, 4));
    }

    pub fn num_landmarks(&self) -> usize {
        self.landmarks.len()
    }

    pub fn latest_state(&self) -> KeyframeState {
        self.window.back().unwrap().state
    }

    /// Processes one stereo frame. `imu_since_last` is the raw IMU stream
    /// between the previous processed frame and this one.
    pub fn process_frame(&mut self, left: &GrayImage, right: &GrayImage, timestamp_ns: u64, imu_since_last: &[ImuSample]) -> Option<VioFrameResult> {
        self.imu_buffer.extend_from_slice(imu_since_last);
        self.frame_index += 1;

        let prev_pyramid = self.prev_pyramid.as_ref()?;
        let curr_pyramid = ImagePyramid::build(left, 4);
        let prev_positions: Vec<(f32, f32)> = self.tracks.iter().map(|t| t.pixel).collect();
        let results = track_pyramid(prev_pyramid, &curr_pyramid, &prev_positions, &self.params.lk);

        let (w, h) = (left.width() as f32, left.height() as f32);
        let mut surviving = Vec::with_capacity(self.tracks.len());
        for (track, r) in self.tracks.iter().zip(results.iter()) {
            if r.found && r.x >= 0.0 && r.y >= 0.0 && r.x < w && r.y < h {
                surviving.push(Track { pixel: (r.x, r.y), landmark_id: track.landmark_id });
            }
        }
        self.tracks = surviving;
        self.prev_pyramid = Some(curr_pyramid);

        if self.tracks.len() < 6 {
            return None;
        }

        let is_keyframe = self.frame_index.is_multiple_of(self.params.keyframe_stride);
        if !is_keyframe {
            return Some(VioFrameResult {
                pose_world_to_body: self.window.back().unwrap().state.pose,
                is_keyframe: false,
                window_len: self.window.len(),
                num_landmarks: self.landmarks.len(),
            });
        }

        // Initial pose guess via PnP against currently tracked landmarks
        // (reuses M3's well-tested DLT + Gauss-Newton refine).
        let points_world: Vec<Vector3<f64>> = self.tracks.iter().map(|t| self.landmarks[t.landmark_id]).collect();
        let observations: Vec<Vector2<f64>> = self
            .tracks
            .iter()
            .map(|t| self.rig.cam0.unproject_to_normalized(Vector2::new(t.pixel.0 as f64, t.pixel.1 as f64)))
            .collect();
        let initial_pose = estimate_pose_dlt(&points_world, &observations)?;
        let initial_pose = refine_pose_gauss_newton(&points_world, &observations, initial_pose, 10);

        let prev_state = self.window.back().unwrap().state;
        let prev_timestamp = self.window.back().unwrap().timestamp_ns;
        let dt = (timestamp_ns - prev_timestamp) as f64 * 1e-9;

        let mut preint = Preintegration::new(prev_state.bias_gyro, prev_state.bias_accel);
        for pair in self.imu_buffer.windows(2) {
            let step_dt = (pair[1].timestamp_ns - pair[0].timestamp_ns) as f64 * 1e-9;
            preint.integrate_measurement(pair[0].gyro, pair[0].accel, step_dt);
        }
        self.imu_buffer.clear();

        let new_state = KeyframeState::new(initial_pose, prev_state.velocity, prev_state.bias_gyro, prev_state.bias_accel);
        let mut new_kf = WindowKeyframe {
            timestamp_ns,
            state: new_state,
            observations: Vec::new(),
            imu_edge: Some((preint, dt)),
        };
        for t in &self.tracks {
            let n = self.rig.cam0.unproject_to_normalized(Vector2::new(t.pixel.0 as f64, t.pixel.1 as f64));
            new_kf.observations.push(Observation { landmark_id: t.landmark_id, camera: Camera::Cam0, normalized: n });
        }
        self.window.push_back(new_kf);
        let new_keyframe_idx = self.window.len() - 1;

        self.add_new_landmarks(left, right, &new_state, new_keyframe_idx);

        if self.window.len() > self.params.window_size {
            self.window.pop_front();
        }

        self.run_optimization();

        Some(VioFrameResult {
            pose_world_to_body: self.window.back().unwrap().state.pose,
            is_keyframe: true,
            window_len: self.window.len(),
            num_landmarks: self.landmarks.len(),
        })
    }

    fn add_new_landmarks(&mut self, left: &GrayImage, right: &GrayImage, state: &KeyframeState, keyframe_idx: usize) {
        let keypoints = detect_grid(left, self.params.fast_threshold, self.params.grid_cell_size, self.params.max_keypoints_per_cell);
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
        // state.pose is world->body; cam0 = body->world undone then body->cam0.
        let body_to_world = state.pose.inverse();
        let t_cam0_to_body = self.t_bs_cam0;
        for m in matches {
            let point_body = t_cam0_to_body.transform(&m.point_cam0);
            let point_world = body_to_world.transform(&point_body);
            let landmark_id = self.landmarks.len();
            self.landmarks.push(point_world);
            self.tracks.push(Track { pixel: m.left_pixel, landmark_id });

            let n0 = self.rig.cam0.unproject_to_normalized(Vector2::new(m.left_pixel.0 as f64, m.left_pixel.1 as f64));
            self.window[keyframe_idx].observations.push(Observation { landmark_id, camera: Camera::Cam0, normalized: n0 });

            // Right-image observation, from the same stereo match: anchors
            // the metric stereo constraint directly in the optimizer at
            // this landmark's creation keyframe.
            let t10 = self.rig.relative_pose_cam1_from_cam0();
            let point_cam1 = t10.transform(&m.point_cam0);
            let n1 = self.rig.cam1.unproject_to_normalized(self.rig.cam1.project(point_cam1));
            self.window[keyframe_idx].observations.push(Observation { landmark_id, camera: Camera::Cam1, normalized: n1 });
        }
    }

    fn run_optimization(&mut self) {
        let mut local_landmark_ids: HashMap<usize, usize> = HashMap::new();
        let mut local_landmarks = Vec::new();
        let mut reprojection_obs = Vec::new();

        for (kf_idx, kf) in self.window.iter().enumerate() {
            for obs in &kf.observations {
                let local_idx = *local_landmark_ids.entry(obs.landmark_id).or_insert_with(|| {
                    local_landmarks.push(self.landmarks[obs.landmark_id]);
                    local_landmarks.len() - 1
                });
                let t_bs_cam = match obs.camera {
                    Camera::Cam0 => self.rig.t_bs_cam0,
                    Camera::Cam1 => self.rig.t_bs_cam1,
                };
                reprojection_obs.push(ReprojectionObservation {
                    keyframe_idx: kf_idx,
                    landmark_idx: local_idx,
                    t_bs_cam,
                    observed_normalized: obs.normalized,
                });
            }
        }

        let keyframes: Vec<KeyframeState> = self.window.iter().map(|kf| kf.state).collect();
        let mut imu_factors = Vec::new();
        let mut bias_rw_factors = Vec::new();
        // window[0]'s `imu_edge` (if any) connects to a keyframe that has
        // already slid out of the window — only edges among keyframes
        // *currently* in the window (kf_idx >= 1) are usable here.
        for (kf_idx, kf) in self.window.iter().enumerate().skip(1) {
            if let Some((preint, dt)) = &kf.imu_edge {
                imu_factors.push(ImuFactorSpec { i: kf_idx - 1, j: kf_idx, preint: preint.clone(), dt: *dt });
                bias_rw_factors.push(BiasRwFactorSpec { i: kf_idx - 1, j: kf_idx });
            }
        }

        let mut problem = Problem {
            keyframes,
            landmarks: local_landmarks,
            reprojection_obs,
            imu_factors,
            bias_rw_factors,
            gravity_world: self.gravity_world,
        };

        optimize(&mut problem, &self.params.solver);

        for (kf_idx, kf) in self.window.iter_mut().enumerate() {
            kf.state = problem.keyframes[kf_idx];
        }
        for (&global_id, &local_idx) in &local_landmark_ids {
            self.landmarks[global_id] = problem.landmarks[local_idx];
        }
    }
}
