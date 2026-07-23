use std::collections::{BTreeMap, HashMap, VecDeque};
use std::time::{Duration, Instant};

use image::GrayImage;
use nalgebra::{Vector2, Vector3};
use slam_core::SE3;
use slam_dataset::ImuSample;
use slam_frontend::{match_stereo_keypoints, StereoMatchParams};
use slam_geometry::{estimate_pose_dlt, refine_pose_gauss_newton, StereoRectification, StereoRig};
use slam_imu::Preintegration;
use slam_optim::{marginalize_keyframe, optimize, BiasRwFactorSpec, ImuFactorSpec, KeyframeState, MarginalizationInput, PriorFactor, Problem, ReprojectionObservation, SolverConfig, UniqueLandmarkObservation};
use slam_vision::{detect_grid, track_pyramid, ImagePyramid, LkParams};

#[derive(Debug, Clone, Copy)]
pub struct VioParams {
    pub keyframe_stride: usize,
    pub window_size: usize,
    pub fast_threshold: u8,
    pub grid_cell_size: u32,
    pub max_keypoints_per_cell: usize,
    pub min_new_landmark_pixel_distance: f32,
    /// Sanity bound on the PnP-derived pose jump *per frame* (~20Hz,
    /// `VoParams::max_pose_jump_meters`'s exact counterpart, `decisions/
    /// 0009`) — the root-cause fix: rejects an implausible PnP result
    /// before it ever enters the window at all.
    pub max_pose_jump_meters: f64,
    /// A second, looser sanity bound at the marginalization boundary
    /// itself (keyframe-to-keyframe, ~10x the frame interval, so a looser
    /// threshold than `max_pose_jump_meters`) — defense in depth: catches
    /// anything that still reaches an eviction implausibly displaced
    /// (e.g. from many chained track-loss recoveries) before folding it
    /// into a prior that would otherwise retain it indefinitely instead of
    /// naive-drop's "forgotten at the next eviction" behavior. See
    /// `marginalize_evicted_keyframe`'s doc comment.
    pub max_marginalization_pose_jump_meters: f64,
    /// Caps how many of the most recent keyframes `global_bundle_
    /// adjustment` includes, instead of literal unbounded `history`
    /// (`plan/STAGE4.md` M1, closing the gap `plan/STAGE2.md`'s own
    /// Risks section predicted: `solver.rs`'s dense O(dim^3) LU solve,
    /// never replaced for this call site, made a full-sequence global BA
    /// pass take ~957s on `MH_01_easy`'s 741 keyframes — confirmed via
    /// live profiling, not assumed — dominating total wall-clock far
    /// past the per-frame VIO loop's own already-real-time cost). Older
    /// keyframes outside the cap simply keep whatever pose the
    /// windowed/marginalized solve already gave them; without loop
    /// closure chained into this pipeline, global BA over the *full*
    /// history wasn't preventing the accumulated-drift problem full-
    /// sequence runs also surfaced anyway (`memory/progress/2026-07-23-
    /// stage4-m0-mh01-full-sequence-measured.md`), so bounding its scope
    /// is a real cost fix, not a knowingly-worse accuracy tradeoff.
    pub max_global_ba_keyframes: usize,
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
            max_pose_jump_meters: 2.0,
            max_marginalization_pose_jump_meters: 10.0,
            // ~1.5x the ~100-keyframe scale Stage 2/3's own bounded-clip
            // testing already exercised (see the field's own doc comment
            // for the cost math this bounds) — comfortably "global"
            // (spans well past the sliding window) while keeping the
            // dense solve's cost small and independent of total sequence
            // length.
            max_global_ba_keyframes: 150,
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

/// Propagates `prev` forward by the (bias-corrected) preintegrated IMU
/// measurement, using the same forward physics model
/// `slam_optim::imu_residual` checks a state *against* — this computes
/// the predicted state directly, for when there's no vision update to
/// correct it with (M6's track-loss recovery: coast on IMU alone rather
/// than failing outright). `state.pose` is `world -> body`, so the
/// propagation (naturally expressed in `world -> body`'s inverse,
/// `R_wb`/body's world position) is converted back at the end.
fn propagate_state(prev: &KeyframeState, preint: &Preintegration, gravity_world: Vector3<f64>, dt: f64) -> KeyframeState {
    let (delta_r, delta_v, delta_p) = preint.corrected(prev.bias_gyro, prev.bias_accel);
    let r_wb_prev = prev.pose.rotation.inverse();
    let p_prev = prev.pose.inverse().translation;

    let r_wb_new = r_wb_prev.compose(&delta_r);
    let v_new = prev.velocity + gravity_world * dt + r_wb_prev.transform(&delta_v);
    let p_new = p_prev + prev.velocity * dt + 0.5 * gravity_world * dt * dt + r_wb_prev.transform(&delta_p);

    let r_bw_new = r_wb_new.inverse();
    let pose_world_to_body = SE3::new(r_bw_new, -r_bw_new.transform(&p_new));
    KeyframeState::new(pose_world_to_body, v_new, prev.bias_gyro, prev.bias_accel)
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
    /// `true` if this frame recovered from track loss: too few vision
    /// tracks survived, so the pose comes from IMU-only propagation
    /// (`propagate_state`) rather than a PnP/optimizer estimate, with the
    /// local landmark map reset around it. See `process_frame`'s doc
    /// comment.
    pub recovered: bool,
}

/// A sliding-window visual-inertial odometry pipeline: LK-tracks stereo-
/// matched landmarks frame-to-frame (reusing `slam_frontend`'s stereo
/// matching), and every `keyframe_stride` frames promotes the frame to a
/// keyframe — preintegrating the buffered raw IMU into an IMU factor,
/// adding reprojection factors for tracked/newly-triangulated landmarks,
/// and jointly optimizing the whole window via `slam_optim`.
///
/// The window is a real marginalized sliding window (Stage 2 M1, closing
/// `decisions/0007`): when the oldest keyframe slides out, it's Schur-
/// complemented into a `PriorFactor` on the new oldest keyframe
/// (`marginalize_evicted_keyframe`) instead of being dropped outright —
/// its IMU/bias-random-walk connectivity and any landmarks *only it*
/// observed are folded in; landmarks it shares with a still-active
/// keyframe simply lose its contribution (a documented simplification,
/// see `marginalize_evicted_keyframe`'s doc comment).
pub struct VioPipeline {
    rig: StereoRig,
    rect: StereoRectification,
    t_bs_cam0: SE3,
    gravity_world: Vector3<f64>,
    params: VioParams,

    landmarks: Vec<Vector3<f64>>,
    tracks: Vec<Track>,
    window: VecDeque<WindowKeyframe>,
    /// The prior folding in everything marginalized out of the window so
    /// far, attached to `window[0]` (`None` only before the window has
    /// ever evicted a keyframe — i.e. `window[0]` is still the whole
    /// trajectory's true first keyframe). Included as an extra factor in
    /// every `run_optimization` call, and replaced (not just added to)
    /// whenever `window[0]` itself gets marginalized in turn.
    prior: Option<PriorFactor>,
    /// Keyframes that have slid out of `window` are kept here purely so a
    /// later `global_bundle_adjustment` pass (M8) has the full
    /// trajectory's observations to work with — `run_optimization` itself
    /// never looks at this, only the current `window` + `prior`. Now
    /// partly redundant with marginalization's own compact `prior` for
    /// the windowed path specifically; left as-is since `global_bundle_
    /// adjustment` still needs the raw per-keyframe observations
    /// (reconciling the two is a good follow-up, not required for M1's
    /// own checkpoint — see `memory/decisions`).
    history: Vec<WindowKeyframe>,
    prev_pyramid: Option<ImagePyramid>,
    imu_buffer: Vec<ImuSample>,
    frame_index: usize,

    /// Cumulative wall-clock spent in the continuous, per-frame VIO loop
    /// (pyramid build, LK tracking, PnP, new-landmark detection/stereo
    /// matching) vs. the windowed backend optimization
    /// (`run_optimization`) and the one-shot global BA pass
    /// (`global_bundle_adjustment`) — the basis for `plan/STAGE2.md`'s
    /// real-time bar, exposed via `timing()`.
    vision_time: Duration,
    optimization_time: Duration,
    global_ba_time: Duration,
}

/// Cumulative wall-clock timing since a `VioPipeline` was created — see
/// the struct fields' doc comment for what each bucket covers.
#[derive(Debug, Clone, Copy, Default)]
pub struct VioTiming {
    pub vision_seconds: f64,
    pub optimization_seconds: f64,
    pub global_ba_seconds: f64,
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
            prior: None,
            history: Vec::new(),
            prev_pyramid: None,
            imu_buffer: Vec::new(),
            frame_index: 0,
            vision_time: Duration::ZERO,
            optimization_time: Duration::ZERO,
            global_ba_time: Duration::ZERO,
        }
    }

    pub fn timing(&self) -> VioTiming {
        VioTiming {
            vision_seconds: self.vision_time.as_secs_f64(),
            optimization_seconds: self.optimization_time.as_secs_f64(),
            global_ba_seconds: self.global_ba_time.as_secs_f64(),
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
    /// between the previous processed frame and this one. If too few
    /// vision tracks survive (or PnP fails on a degenerate point
    /// configuration), this is track loss — rather than failing
    /// permanently, IMU-only propagation (`propagate_state`) provides a
    /// fallback pose, the local map is reset around it (fresh stereo-
    /// matched landmarks), and a keyframe is created regardless of the
    /// usual stride (`VioFrameResult::recovered = true`). Returns `None`
    /// only if recovery itself finds nothing to re-anchor to.
    pub fn process_frame(&mut self, left: &GrayImage, right: &GrayImage, timestamp_ns: u64, imu_since_last: &[ImuSample]) -> Option<VioFrameResult> {
        self.imu_buffer.extend_from_slice(imu_since_last);
        self.frame_index += 1;

        let track_start = Instant::now();
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
        self.vision_time += track_start.elapsed();

        let prev_state = self.window.back().unwrap().state;
        let prev_timestamp = self.window.back().unwrap().timestamp_ns;
        let dt = (timestamp_ns - prev_timestamp) as f64 * 1e-9;

        // IMU doesn't care whether vision tracking succeeded — always
        // preintegrate the buffered samples.
        let mut preint = Preintegration::new(prev_state.bias_gyro, prev_state.bias_accel);
        for pair in self.imu_buffer.windows(2) {
            let step_dt = (pair[1].timestamp_ns - pair[0].timestamp_ns) as f64 * 1e-9;
            preint.integrate_measurement(pair[0].gyro, pair[0].accel, step_dt);
        }
        self.imu_buffer.clear();

        let pnp_start = Instant::now();
        let vision_pose = if self.tracks.len() >= 6 {
            // PnP against currently tracked landmarks (reuses M3's well-
            // tested DLT + Gauss-Newton refine).
            let points_world: Vec<Vector3<f64>> = self.tracks.iter().map(|t| self.landmarks[t.landmark_id]).collect();
            let observations: Vec<Vector2<f64>> = self
                .tracks
                .iter()
                .map(|t| self.rig.cam0.unproject_to_normalized(Vector2::new(t.pixel.0 as f64, t.pixel.1 as f64)))
                .collect();
            // Same DLT-PnP-can-occasionally-produce-a-wildly-wrong-pose
            // vulnerability `decisions/0009` found and fixed in
            // `VoPipeline` (no RANSAC/outlier rejection, `decisions/0003`)
            // — VioPipeline shares the identical PnP call and was
            // predicted there to need the same guard "if a similar
            // corruption shows up in a future full-VIO-sequence test." It
            // did: validating Stage 2 M1's marginalization surfaced a real
            // divergence (an implausible-jump pose entering the window,
            // then propagating through subsequent frames) that was always
            // latent here, just never caught before — naive fixed-lag
            // dropping happened to "forget" the corrupt keyframe quickly
            // enough that Sim3-aligned ATE stayed plausible-looking
            // (`decisions/0009`'s own caveat about that metric), but
            // marginalization's job is to *retain* information, so it
            // retained the corruption too. Filtering it out here, at the
            // source, fixes it for both paths.
            estimate_pose_dlt(&points_world, &observations)
                .map(|initial| refine_pose_gauss_newton(&points_world, &observations, initial, 10))
                .filter(|pose| {
                    let jump = (pose.inverse().translation - prev_state.pose.inverse().translation).norm();
                    jump.is_finite() && jump < self.params.max_pose_jump_meters
                })
        } else {
            None
        };
        self.vision_time += pnp_start.elapsed();

        let (initial_pose, initial_velocity, is_keyframe, recovered) = match vision_pose {
            Some(pose) => (pose, prev_state.velocity, self.frame_index.is_multiple_of(self.params.keyframe_stride), false),
            None => {
                let propagated = propagate_state(&prev_state, &preint, self.gravity_world, dt);
                (propagated.pose, propagated.velocity, true, true)
            }
        };

        if !is_keyframe {
            return Some(VioFrameResult {
                pose_world_to_body: self.window.back().unwrap().state.pose,
                is_keyframe: false,
                window_len: self.window.len(),
                num_landmarks: self.landmarks.len(),
                recovered: false,
            });
        }

        if recovered {
            self.tracks.clear();
        }

        let new_state = KeyframeState::new(initial_pose, initial_velocity, prev_state.bias_gyro, prev_state.bias_accel);
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

        let landmarks_start = Instant::now();
        self.add_new_landmarks(left, right, &new_state, new_keyframe_idx);
        self.vision_time += landmarks_start.elapsed();

        if recovered && self.tracks.is_empty() {
            // Nothing to re-anchor to (e.g. a genuinely blank frame) —
            // undo the tentative keyframe and report unrecoverable, same
            // contract as the doc comment promises.
            self.window.pop_back();
            return None;
        }

        if self.window.len() > self.params.window_size {
            if let Some(evicted) = self.window.pop_front() {
                self.marginalize_evicted_keyframe(&evicted);
                self.history.push(evicted);
            }
        }

        let opt_start = Instant::now();
        self.run_optimization();
        self.optimization_time += opt_start.elapsed();

        Some(VioFrameResult {
            pose_world_to_body: self.window.back().unwrap().state.pose,
            is_keyframe: true,
            window_len: self.window.len(),
            num_landmarks: self.landmarks.len(),
            recovered,
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

    /// Schur-complements `evicted` (just popped from the front of
    /// `window`) into a new `PriorFactor` on the new `window[0]`,
    /// replacing `self.prior` (Stage 2 M1, closing `decisions/0007`).
    ///
    /// Landmarks `evicted` shares with a still-active window keyframe, or
    /// that `self.tracks` (the live, frame-by-frame LK tracker, not just
    /// keyframe observations) is still actively following, are *not*
    /// folded in — only landmarks *nothing* still needs. Folding in a
    /// landmark `self.tracks` still references was a real bug found while
    /// validating this milestone: its position in `self.landmarks` would
    /// freeze forever (marginalization eliminates it from the optimizer
    /// entirely), while future frames kept using that frozen, increasingly
    /// stale position for PnP. Landmarks are also only folded in when
    /// `evicted` itself recorded a genuine stereo pair for them (>= 2
    /// observations at `evicted`): one with only a monocular "still
    /// tracking it" entry (its real stereo pair was created at an even
    /// earlier, already-marginalized keyframe) gives a rank-deficient,
    /// poorly-conditioned contribution on its own — safer to drop, same
    /// as the shared-landmark case.
    ///
    /// Guards against a second real bug found validating this milestone:
    /// naive fixed-lag dropping (the old behavior) *forgets* a keyframe
    /// whose pose went implausibly wrong (a rare but real DLT-PnP failure
    /// mode, `decisions/0009`) the moment it slides out of the window —
    /// marginalization instead *locks it in* as an increasingly confident
    /// prior with nothing to correct it, which measurably diverged to
    /// absurd (multi-kilometer, then multi-trillion-meter) poses within
    /// ~100 frames on a real MH_01 run before this guard existed. If the
    /// relative jump between `evicted` and the new oldest keyframe is
    /// implausible, or the resulting prior isn't finite, this keyframe's
    /// information is dropped instead of folded in (falling back to
    /// naive-drop behavior for just this one eviction, and resetting
    /// `self.prior` to `None` rather than keeping a now-stale one) —
    /// exactly the same "reject, don't propagate" discipline `decisions/
    /// 0009` established for raw PnP results, applied at the
    /// marginalization boundary too.
    fn marginalize_evicted_keyframe(&mut self, evicted: &WindowKeyframe) {
        let Some(new_oldest) = self.window.front() else {
            return;
        };

        let pose_jump = (evicted.state.pose.inverse().translation - new_oldest.state.pose.inverse().translation).norm();
        if !pose_jump.is_finite() || pose_jump > self.params.max_marginalization_pose_jump_meters {
            self.prior = None;
            return;
        }

        // BTreeMap, not HashMap: `unique_landmarks`' iteration order below
        // feeds `slam_optim::marginalize_keyframe`'s own accumulation into
        // a shared matrix (floating-point addition isn't associative) —
        // same determinism discipline as `decisions/0011`.
        let mut by_landmark: BTreeMap<usize, Vec<&Observation>> = BTreeMap::new();
        for obs in &evicted.observations {
            by_landmark.entry(obs.landmark_id).or_default().push(obs);
        }

        let still_observed: std::collections::HashSet<usize> = self.window.iter().flat_map(|kf| kf.observations.iter().map(|o| o.landmark_id)).chain(self.tracks.iter().map(|t| t.landmark_id)).collect();

        let unique_landmarks: Vec<UniqueLandmarkObservation> = by_landmark
            .into_iter()
            .filter(|(landmark_id, obs_list)| !still_observed.contains(landmark_id) && obs_list.len() >= 2)
            .map(|(landmark_id, obs_list)| {
                let observations = obs_list
                    .into_iter()
                    .map(|o| {
                        let t_bs_cam = match o.camera {
                            Camera::Cam0 => self.rig.t_bs_cam0,
                            Camera::Cam1 => self.rig.t_bs_cam1,
                        };
                        (t_bs_cam, o.normalized)
                    })
                    .collect();
                UniqueLandmarkObservation { landmark: self.landmarks[landmark_id], observations }
            })
            .collect();

        let input = MarginalizationInput {
            state_k: evicted.state,
            state_k1: new_oldest.state,
            incoming_prior: self.prior,
            imu_edge: new_oldest.imu_edge.clone(),
            unique_landmarks,
            gravity_world: self.gravity_world,
            config: self.params.solver,
        };
        self.prior = marginalize_keyframe(&input).filter(|p| p.information.iter().all(|v| v.is_finite()) && p.information_vector.iter().all(|v| v.is_finite()));
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
            priors: self.prior.into_iter().collect(),
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

    /// `(timestamp, world -> body pose)` for every keyframe ever created
    /// (retained `history` plus the current `window`), in trajectory
    /// order — the full picture `run_optimization`'s bounded window never
    /// sees at once.
    pub fn all_keyframe_poses(&self) -> Vec<(u64, SE3)> {
        self.history.iter().chain(self.window.iter()).map(|kf| (kf.timestamp_ns, kf.state.pose)).collect()
    }

    /// M8: a single "global" bundle-adjustment pass over the most recent
    /// `params.max_global_ba_keyframes` retained keyframes (history plus
    /// current window, in creation order), reusing the same
    /// `slam_optim::Problem`/`optimize` machinery `run_optimization`
    /// uses per-window — just with a much larger (but still bounded, see
    /// `VioParams::max_global_ba_keyframes`'s own doc comment for why
    /// "bounded" and not "literal full history") span included instead
    /// of the small sliding window. A one-shot pass (e.g. after loop
    /// closure, or at the end of a run), not something to call every
    /// frame. Returns the number of keyframes actually included in this
    /// pass (== every keyframe ever created only when that total is
    /// under the cap — small test scenarios stay literally "the whole
    /// trajectory," as before). The oldest *included* keyframe (not
    /// necessarily the true first keyframe of the whole trajectory, once
    /// the cap is active) is the gauge anchor for this call, same
    /// generic mechanism every windowed `run_optimization` call already
    /// uses — `slam_optim::Problem`'s own indexing is always relative to
    /// "whatever's first in this `Problem`," so bounding scope here needs
    /// no protocol change on the solver side.
    pub fn global_bundle_adjustment(&mut self) -> usize {
        let start = Instant::now();
        let total = self.global_bundle_adjustment_inner();
        self.global_ba_time += start.elapsed();
        total
    }

    fn global_bundle_adjustment_inner(&mut self) -> usize {
        let mut local_landmark_ids: HashMap<usize, usize> = HashMap::new();
        let mut local_landmarks = Vec::new();
        let mut reprojection_obs = Vec::new();
        let mut keyframes = Vec::new();
        let mut imu_factors = Vec::new();
        let mut bias_rw_factors = Vec::new();

        let total_ever = self.history.len() + self.window.len();
        let included_start = total_ever.saturating_sub(self.params.max_global_ba_keyframes);

        for (global_idx, kf) in self.history.iter().chain(self.window.iter()).enumerate().skip(included_start) {
            let kf_idx = global_idx - included_start;
            keyframes.push(kf.state);
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
            // `kf_idx > 0` (not just "has an imu_edge"): once the cap
            // excludes the true first keyframe, the oldest *included*
            // keyframe still has a real `imu_edge` pointing at a now-
            // excluded previous keyframe — including it here would
            // reference a keyframe index that doesn't exist in this
            // bounded `Problem` (an out-of-range/underflow bug the
            // unbounded case never had to guard against, since
            // `history[0]`'s own `imu_edge` is always `None`).
            if kf_idx > 0 {
                if let Some((preint, dt)) = &kf.imu_edge {
                    imu_factors.push(ImuFactorSpec { i: kf_idx - 1, j: kf_idx, preint: preint.clone(), dt: *dt });
                    bias_rw_factors.push(BiasRwFactorSpec { i: kf_idx - 1, j: kf_idx });
                }
            }
        }

        let total = keyframes.len();
        let mut problem = Problem {
            keyframes,
            landmarks: local_landmarks,
            reprojection_obs,
            imu_factors,
            bias_rw_factors,
            // The oldest *included* keyframe (local index 0) is this
            // call's gauge anchor — the true first keyframe of the whole
            // trajectory only when `included_start == 0` (total ever
            // created is under the cap). No prior is supplied for it: an
            // older-but-excluded keyframe's own information isn't folded
            // in as a soft constraint, it's simply left out of this pass
            // (see `max_global_ba_keyframes`'s doc comment for why that's
            // an acceptable tradeoff here, not a silent accuracy cut).
            priors: Vec::new(),
            gravity_world: self.gravity_world,
        };

        optimize(&mut problem, &self.params.solver);

        for (global_idx, kf) in self.history.iter_mut().chain(self.window.iter_mut()).enumerate().skip(included_start) {
            kf.state = problem.keyframes[global_idx - included_start];
        }
        for (&global_id, &local_idx) in &local_landmark_ids {
            self.landmarks[global_id] = problem.landmarks[local_idx];
        }

        total
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use slam_core::SO3;

    /// Same synthetic model as `slam_optim::imu_factor`'s and
    /// `slam_frontend::vi_init`'s tests (constant angular + world
    /// velocity under gravity): `propagate_state` from keyframe i's true
    /// state using a preintegration built from the same raw IMU should
    /// land on keyframe j's true state — the forward-model counterpart of
    /// `imu_residual`'s zero-residual check.
    #[test]
    fn propagate_state_matches_ground_truth_motion() {
        let w_true = Vector3::new(0.2, -0.15, 0.25);
        let v_true = Vector3::new(0.4, 0.1, -0.15);
        let g_true = Vector3::new(0.0, 0.0, -9.81);
        let dt_total = 0.5;

        let body_pose_at = |t: f64| SE3::new(SO3::exp(w_true * t), v_true * t);
        let world_to_body_at = |t: f64| body_pose_at(t).inverse();

        let prev = KeyframeState::new(world_to_body_at(0.0), v_true, Vector3::zeros(), Vector3::zeros());
        let expected_next = KeyframeState::new(world_to_body_at(dt_total), v_true, Vector3::zeros(), Vector3::zeros());

        let rate_hz = 200.0;
        let steps = (dt_total * rate_hz) as usize;
        let dt_step = 1.0 / rate_hz;
        let mut preint = Preintegration::new(Vector3::zeros(), Vector3::zeros());
        for i in 0..steps {
            let t = i as f64 * dt_step;
            let r_wb = body_pose_at(t).rotation;
            let specific_force = r_wb.inverse().transform(&(-g_true));
            preint.integrate_measurement(w_true, specific_force, dt_step);
        }

        let propagated = propagate_state(&prev, &preint, g_true, dt_total);
        assert_relative_eq!(propagated.pose.matrix(), expected_next.pose.matrix(), epsilon = 1e-3);
        assert_relative_eq!(propagated.velocity, expected_next.velocity, epsilon = 1e-3);
    }
}
