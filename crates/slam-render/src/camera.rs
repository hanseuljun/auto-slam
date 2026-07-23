use nalgebra::{Matrix4, Point3, Vector3};

/// An orbit camera: position parameterized as `target + distance * dir(yaw,
/// pitch)`, world-up `+Y`, right-handed, matching the convention `look_at_rh`
/// below assumes. This is the "3D rendering library" side of camera control
/// (`plan/STAGE3.md` M1) — mouse-drag orbit maps to `orbit`, scroll to
/// `zoom`, drag-with-modifier to `pan`; none of those input bindings live
/// here, this struct only owns the resulting camera state and the matrices
/// derived from it.
#[derive(Debug, Clone, Copy)]
pub struct OrbitCamera {
    pub target: Point3<f64>,
    pub distance: f64,
    /// Radians, rotation around world `+Y`, unclamped (wraps naturally
    /// since it only ever feeds `sin`/`cos`).
    pub yaw: f64,
    /// Radians, elevation above the target's horizontal plane. Clamped to
    /// `(-PITCH_LIMIT, PITCH_LIMIT)` on every mutation, strictly inside
    /// +-90 degrees, so `up` (world `+Y`) is never parallel to the view
    /// direction — at exactly the poles, `look_at_rh`'s `forward.cross(up)`
    /// degenerates to zero and the camera basis becomes undefined.
    pub pitch: f64,
    pub fov_y_radians: f64,
    pub aspect: f64,
    pub near: f64,
    pub far: f64,
}

/// Keeps `pitch` strictly inside +-90 degrees (see `OrbitCamera::pitch`'s
/// doc comment for why exactly 90 degrees is unsafe, not just inconvenient).
const PITCH_LIMIT: f64 = 89.0_f64 / 180.0 * std::f64::consts::PI;
const MIN_DISTANCE: f64 = 1e-3;

impl OrbitCamera {
    pub fn new(target: Point3<f64>, distance: f64, aspect: f64) -> Self {
        OrbitCamera {
            target,
            distance: distance.max(MIN_DISTANCE),
            yaw: 0.0,
            pitch: 0.0,
            fov_y_radians: 60.0_f64.to_radians(),
            aspect,
            near: 0.05,
            far: 1000.0,
        }
    }

    /// The camera's world-space eye position: `distance` out from `target`
    /// along the direction `(yaw, pitch)` describe, `yaw=0, pitch=0`
    /// placing the eye on `target`'s `+Z` side looking back toward `-Z`
    /// (matches `look_at_rh`'s forward-vector convention below).
    pub fn eye(&self) -> Point3<f64> {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let offset = Vector3::new(self.distance * cp * sy, self.distance * sp, self.distance * cp * cy);
        self.target + offset
    }

    /// Mouse-drag orbit: adjusts `yaw`/`pitch` by the given deltas (radians),
    /// clamping `pitch` to stay inside `PITCH_LIMIT`.
    pub fn orbit(&mut self, delta_yaw: f64, delta_pitch: f64) {
        self.yaw += delta_yaw;
        self.pitch = (self.pitch + delta_pitch).clamp(-PITCH_LIMIT, PITCH_LIMIT);
    }

    /// Scroll-wheel zoom: scales `distance` multiplicatively (so zooming
    /// feels consistent at any distance, not additively), clamped above
    /// `MIN_DISTANCE` so the eye can never coincide with `target` (which
    /// would make `look_at_rh`'s forward vector undefined).
    pub fn zoom(&mut self, factor: f64) {
        self.distance = (self.distance * factor).max(MIN_DISTANCE);
    }

    /// Drag-to-pan: moves `target` along the camera's own right/up axes,
    /// scaled by `distance` so a fixed pixel-drag pans by a fixed fraction
    /// of the visible scene regardless of current zoom level.
    pub fn pan(&mut self, delta_right: f64, delta_up: f64) {
        let eye = self.eye();
        let forward = (self.target - eye).normalize();
        let right = forward.cross(&Vector3::y()).normalize();
        let up = right.cross(&forward);
        let scale = self.distance;
        self.target += right * (delta_right * scale) + up * (delta_up * scale);
    }

    /// Right-handed look-at view matrix (world -> camera space), camera
    /// looking down `-Z` in its own space, `+Y` up, `+X` right — the
    /// standard OpenGL/wgpu convention `projection_matrix` below assumes.
    pub fn view_matrix(&self) -> Matrix4<f64> {
        look_at_rh(&self.eye(), &self.target, &Vector3::y())
    }

    /// Right-handed perspective projection producing wgpu's clip-space
    /// depth range `z_ndc in [0, 1]` (not OpenGL's `[-1, 1]` -
    /// `nalgebra::Perspective3` assumes the latter, so this is hand-written
    /// rather than reused, a real but narrow "own the rendering math"
    /// case per `plan/STAGE3.md`'s dependency policy).
    pub fn projection_matrix(&self) -> Matrix4<f64> {
        perspective_wgpu(self.fov_y_radians, self.aspect, self.near, self.far)
    }

    pub fn view_projection_matrix(&self) -> Matrix4<f64> {
        self.projection_matrix() * self.view_matrix()
    }

    /// Projects a world-space point to pixel coordinates in a
    /// `viewport_width x viewport_height` image (origin top-left, `+Y`
    /// down, matching image/screen convention rather than NDC's `+Y` up).
    /// `None` if the point is behind the eye (`clip.w <= 0`) or otherwise
    /// unprojectable.
    pub fn project_to_pixels(&self, world: &Point3<f64>, viewport_width: f64, viewport_height: f64) -> Option<(f64, f64)> {
        let clip = self.view_projection_matrix() * world.to_homogeneous();
        if clip.w <= 1e-12 {
            return None;
        }
        let ndc_x = clip.x / clip.w;
        let ndc_y = clip.y / clip.w;
        let px = (ndc_x * 0.5 + 0.5) * viewport_width;
        let py = (1.0 - (ndc_y * 0.5 + 0.5)) * viewport_height;
        Some((px, py))
    }
}

/// Standard right-handed look-at matrix (glm::lookAtRH convention):
/// `eye`/`target`/`up` in world space, `+Y` up, camera looks down its own
/// `-Z`. Hand-written rather than `nalgebra::Isometry3::look_at_rh`'s
/// output directly, to keep this file self-contained and the matrix layout
/// (row-major reading order into `Matrix4::new`, column-major storage,
/// same as `nalgebra` uses throughout) explicit for the unit tests below.
fn look_at_rh(eye: &Point3<f64>, target: &Point3<f64>, up: &Vector3<f64>) -> Matrix4<f64> {
    let f = (target - eye).normalize();
    let s = f.cross(up).normalize();
    let u = s.cross(&f);
    #[rustfmt::skip]
    let m = Matrix4::new(
        s.x, s.y, s.z, -s.dot(&eye.coords),
        u.x, u.y, u.z, -u.dot(&eye.coords),
        -f.x, -f.y, -f.z, f.dot(&eye.coords),
        0.0, 0.0, 0.0, 1.0,
    );
    m
}

/// Right-handed perspective projection for wgpu's clip space (`z_ndc in
/// [0, 1]`, camera looking down `-Z` in view space) — see `projection_
/// matrix`'s doc comment for why this isn't `nalgebra::Perspective3`.
fn perspective_wgpu(fov_y_radians: f64, aspect: f64, near: f64, far: f64) -> Matrix4<f64> {
    let f = 1.0 / (fov_y_radians / 2.0).tan();
    #[rustfmt::skip]
    let m = Matrix4::new(
        f / aspect, 0.0, 0.0, 0.0,
        0.0, f, 0.0, 0.0,
        0.0, 0.0, far / (near - far), (far * near) / (near - far),
        0.0, 0.0, -1.0, 0.0,
    );
    m
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn eye_at_zero_yaw_pitch_sits_on_targets_positive_z_side() {
        let cam = OrbitCamera::new(Point3::origin(), 5.0, 1.0);
        let eye = cam.eye();
        assert_relative_eq!(eye, Point3::new(0.0, 0.0, 5.0), epsilon = 1e-9);
    }

    #[test]
    fn look_at_target_projects_to_viewport_center() {
        let cam = OrbitCamera::new(Point3::new(1.0, 2.0, -3.0), 8.0, 16.0 / 9.0);
        let (px, py) = cam.project_to_pixels(&cam.target, 1600.0, 900.0).expect("target must be in front of the eye");
        assert_relative_eq!(px, 800.0, epsilon = 1e-6);
        assert_relative_eq!(py, 450.0, epsilon = 1e-6);
    }

    #[test]
    fn orbiting_ninety_degrees_yaw_moves_eye_from_z_axis_to_x_axis() {
        let mut cam = OrbitCamera::new(Point3::origin(), 3.0, 1.0);
        cam.orbit(std::f64::consts::FRAC_PI_2, 0.0);
        assert_relative_eq!(cam.eye(), Point3::new(3.0, 0.0, 0.0), epsilon = 1e-9);
    }

    #[test]
    fn pitch_never_reaches_the_pole_even_with_a_huge_delta() {
        let mut cam = OrbitCamera::new(Point3::origin(), 3.0, 1.0);
        cam.orbit(0.0, 1000.0);
        assert!(cam.pitch < std::f64::consts::FRAC_PI_2);
        assert!(cam.pitch > 0.0);
        // The camera basis must stay well-defined (no NaNs) right up
        // against the clamp.
        assert!(cam.view_matrix().iter().all(|v| v.is_finite()));
    }

    #[test]
    fn zoom_out_doubles_distance_and_never_reaches_zero() {
        let mut cam = OrbitCamera::new(Point3::origin(), 4.0, 1.0);
        cam.zoom(2.0);
        assert_relative_eq!(cam.distance, 8.0, epsilon = 1e-9);
        cam.zoom(0.0);
        assert!(cam.distance > 0.0, "distance must never hit exactly zero (undefined eye/target overlap)");
    }

    #[test]
    fn a_point_to_the_right_in_view_space_projects_to_the_right_half_of_the_screen() {
        let cam = OrbitCamera::new(Point3::origin(), 5.0, 1.0);
        // Eye at (0,0,5) looking at the origin: world +X is the camera's
        // local right, per `look_at_rh`'s basis (`right = forward x up`
        // with `forward = -Z`, `up = +Y`, giving `right = +X`).
        let point_to_the_right = Point3::new(1.0, 0.0, 0.0);
        let (px, _py) = cam.project_to_pixels(&point_to_the_right, 1000.0, 1000.0).unwrap();
        assert!(px > 500.0, "expected right-of-center pixel x, got {px}");
    }

    #[test]
    fn near_and_far_planes_map_to_wgpus_zero_and_one_depth() {
        let cam = OrbitCamera::new(Point3::origin(), 0.0, 1.0);
        let proj = perspective_wgpu(cam.fov_y_radians, 1.0, 0.1, 100.0);
        let near_clip = proj * nalgebra::Vector4::new(0.0, 0.0, -0.1, 1.0);
        let far_clip = proj * nalgebra::Vector4::new(0.0, 0.0, -100.0, 1.0);
        assert_relative_eq!(near_clip.z / near_clip.w, 0.0, epsilon = 1e-9);
        assert_relative_eq!(far_clip.z / far_clip.w, 1.0, epsilon = 1e-9);
    }

    #[test]
    fn pan_moves_target_along_camera_right_and_up_not_world_axes() {
        let mut cam = OrbitCamera::new(Point3::origin(), 5.0, 1.0);
        cam.orbit(std::f64::consts::FRAC_PI_2, 0.0); // eye now on +X axis, looking at origin
        cam.pan(1.0, 0.0); // "right" from the eye's new orientation
        // Camera is now looking down -X (eye at +X looking at origin), so
        // its local "right" is world -Z (forward x up = (-1,0,0) x (0,1,0)
        // = (0,0,-1)... let's just assert it moved off the original X axis
        // in Z, not in X/Y, since that's the basis-dependent part being
        // tested.
        assert!(cam.target.z.abs() > 1e-6, "pan should move target along the camera's local right axis");
        assert_relative_eq!(cam.target.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(cam.target.y, 0.0, epsilon = 1e-9);
    }
}
