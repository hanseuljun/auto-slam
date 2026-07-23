use nalgebra::{Point3, Vector3};
use slam_core::SE3;

/// One vertex of a line segment: position + color, `f32` (the GPU's
/// native precision) even though `OrbitCamera`'s own math stays `f64` for
/// consistency with the rest of this repo's estimation code — the
/// `f64 -> f32` narrowing happens once, right before upload, in
/// `Vertex::new`.
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
}

impl Vertex {
    pub fn new(position: &Point3<f64>, color: [f32; 3]) -> Self {
        Vertex { position: [position.x as f32, position.y as f32, position.z as f32], color }
    }
}

/// A set of line segments to draw this frame — the primitive this
/// milestone's rendering library actually exports: a trajectory is a
/// polyline (`plan/STAGE3.md` M2), a ground-plane grid and coordinate
/// axes (this milestone) are also just line segments. Stored as a flat
/// vertex list, two vertices per segment, drawn with `wgpu::PrimitiveTopology::LineList`
/// (no index buffer — line counts here are small enough that the
/// simplicity is worth more than the memory saving).
#[derive(Debug, Clone, Default)]
pub struct Scene {
    pub vertices: Vec<Vertex>,
}

impl Scene {
    pub fn new() -> Self {
        Scene::default()
    }

    pub fn add_line(&mut self, from: &Point3<f64>, to: &Point3<f64>, color: [f32; 3]) {
        self.vertices.push(Vertex::new(from, color));
        self.vertices.push(Vertex::new(to, color));
    }

    /// A polyline through `points`, `n-1` segments for `n` points — the
    /// primitive `plan/STAGE3.md` M2 will feed a run's `trajectory.csv`
    /// through.
    pub fn add_polyline(&mut self, points: &[Point3<f64>], color: [f32; 3]) {
        for pair in points.windows(2) {
            self.add_line(&pair[0], &pair[1], color);
        }
    }

    /// A ground-plane grid in the world `XZ` plane (`Y = 0`, matching
    /// `OrbitCamera`'s world-up convention), `divisions` lines per axis,
    /// spanning `[-half_extent, half_extent]`.
    pub fn add_grid(&mut self, half_extent: f64, divisions: u32, color: [f32; 3]) {
        let divisions = divisions.max(1);
        let step = (2.0 * half_extent) / divisions as f64;
        for i in 0..=divisions {
            let offset = -half_extent + i as f64 * step;
            self.add_line(&Point3::new(offset, 0.0, -half_extent), &Point3::new(offset, 0.0, half_extent), color);
            self.add_line(&Point3::new(-half_extent, 0.0, offset), &Point3::new(half_extent, 0.0, offset), color);
        }
    }

    /// A coordinate-axes gizmo at the world origin: red/green/blue for
    /// X/Y/Z, the standard convention so orientation is legible at a
    /// glance without a legend.
    pub fn add_axes(&mut self, length: f64) {
        let origin = Point3::origin();
        self.add_line(&origin, &Point3::new(length, 0.0, 0.0), [1.0, 0.0, 0.0]);
        self.add_line(&origin, &Point3::new(0.0, length, 0.0), [0.0, 1.0, 0.0]);
        self.add_line(&origin, &Point3::new(0.0, 0.0, length), [0.0, 0.0, 1.0]);
    }

    /// A small crosshair (3 orthogonal line segments centered on `pos`) —
    /// the landmark/point-cloud marker (`plan/STAGE3.md` M2). A single
    /// pixel-sized dot isn't reliably visible with `LineList`-only
    /// rendering (no dedicated point-sprite pipeline, deliberately kept
    /// out of scope for this milestone), so a crosshair is this crate's
    /// stand-in for "a point," same spirit as `add_axes` standing in for
    /// a full 3D model of a coordinate frame.
    pub fn add_point_marker(&mut self, pos: &Point3<f64>, half_size: f64, color: [f32; 3]) {
        self.add_line(&Point3::new(pos.x - half_size, pos.y, pos.z), &Point3::new(pos.x + half_size, pos.y, pos.z), color);
        self.add_line(&Point3::new(pos.x, pos.y - half_size, pos.z), &Point3::new(pos.x, pos.y + half_size, pos.z), color);
        self.add_line(&Point3::new(pos.x, pos.y, pos.z - half_size), &Point3::new(pos.x, pos.y, pos.z + half_size), color);
    }

    /// A crosshair marker per point — the bulk form for a landmark cloud.
    pub fn add_point_markers(&mut self, points: &[Point3<f64>], half_size: f64, color: [f32; 3]) {
        for p in points {
            self.add_point_marker(p, half_size, color);
        }
    }

    /// A schematic camera/keyframe-pose marker: local coordinate axes at
    /// `pose`'s origin, plus a small pyramid wireframe (apex at the
    /// camera center, a square base `scale` out along local `+Z`) so a
    /// trajectory's keyframe poses (`plan/STAGE3.md` M2) read as "a
    /// camera looking this way," not just a bare dot. This is a legible
    /// stand-in for orientation, not a calibrated FOV frustum — it
    /// doesn't use `slam-geometry`'s actual intrinsics, on purpose: exact
    /// FOV isn't the point of a trajectory overview, and coupling this
    /// crate to per-sequence calibration would cut against `plan/
    /// STAGE3.md`'s "`slam-render` depends on `slam-core` only" layout.
    /// Local `+Z` "forward" is a convention choice for this marker only,
    /// not a claim about any specific camera's own frame.
    pub fn add_pose_marker(&mut self, pose: &SE3, scale: f64, color: [f32; 3]) {
        let to_world = |local: Vector3<f64>| -> Point3<f64> { Point3::from(pose.transform(&local)) };

        let center = to_world(Vector3::zeros());
        self.add_line(&center, &to_world(Vector3::new(scale, 0.0, 0.0)), [1.0, 0.0, 0.0]);
        self.add_line(&center, &to_world(Vector3::new(0.0, scale, 0.0)), [0.0, 1.0, 0.0]);
        self.add_line(&center, &to_world(Vector3::new(0.0, 0.0, scale)), [0.0, 0.0, 1.0]);

        let half = scale * 0.5;
        let corners = [
            to_world(Vector3::new(-half, -half, scale)),
            to_world(Vector3::new(half, -half, scale)),
            to_world(Vector3::new(half, half, scale)),
            to_world(Vector3::new(-half, half, scale)),
        ];
        for corner in &corners {
            self.add_line(&center, corner, color);
        }
        for i in 0..4 {
            self.add_line(&corners[i], &corners[(i + 1) % 4], color);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polyline_of_n_points_produces_n_minus_one_segments() {
        let mut scene = Scene::new();
        let points: Vec<Point3<f64>> = (0..5).map(|i| Point3::new(i as f64, 0.0, 0.0)).collect();
        scene.add_polyline(&points, [1.0, 1.0, 1.0]);
        assert_eq!(scene.vertices.len(), 2 * 4);
    }

    #[test]
    fn grid_produces_two_lines_per_division_step() {
        let mut scene = Scene::new();
        scene.add_grid(10.0, 4, [0.5, 0.5, 0.5]);
        // divisions=4 -> 5 grid lines per axis (0..=4), two axes.
        assert_eq!(scene.vertices.len(), 2 * 2 * 5);
    }

    #[test]
    fn axes_are_red_green_blue_for_x_y_z() {
        let mut scene = Scene::new();
        scene.add_axes(2.0);
        assert_eq!(scene.vertices.len(), 6);
        assert_eq!(scene.vertices[1].color, [1.0, 0.0, 0.0]);
        assert_eq!(scene.vertices[3].color, [0.0, 1.0, 0.0]);
        assert_eq!(scene.vertices[5].color, [0.0, 0.0, 1.0]);
        assert_eq!(scene.vertices[1].position, [2.0, 0.0, 0.0]);
        assert_eq!(scene.vertices[3].position, [0.0, 2.0, 0.0]);
        assert_eq!(scene.vertices[5].position, [0.0, 0.0, 2.0]);
    }

    #[test]
    fn point_marker_is_three_lines_centered_on_the_point() {
        let mut scene = Scene::new();
        scene.add_point_marker(&Point3::new(1.0, 2.0, 3.0), 0.5, [1.0, 1.0, 0.0]);
        assert_eq!(scene.vertices.len(), 6);
        assert_eq!(scene.vertices[0].position, [0.5, 2.0, 3.0]);
        assert_eq!(scene.vertices[1].position, [1.5, 2.0, 3.0]);
    }

    #[test]
    fn point_markers_scale_linearly_with_point_count() {
        let mut scene = Scene::new();
        let points: Vec<Point3<f64>> = (0..4).map(|i| Point3::new(i as f64, 0.0, 0.0)).collect();
        scene.add_point_markers(&points, 0.1, [0.0, 1.0, 1.0]);
        assert_eq!(scene.vertices.len(), 4 * 6);
    }

    #[test]
    fn pose_marker_at_identity_matches_local_frame_geometry() {
        let mut scene = Scene::new();
        scene.add_pose_marker(&SE3::identity(), 2.0, [1.0, 1.0, 1.0]);
        // 3 axis lines + 4 apex-to-corner lines + 4 base edges = 11 lines.
        assert_eq!(scene.vertices.len(), 11 * 2);
        // The X axis line (first line) must run from the origin to (2,0,0)
        // when the pose is identity (world frame == local frame).
        assert_eq!(scene.vertices[0].position, [0.0, 0.0, 0.0]);
        assert_eq!(scene.vertices[1].position, [2.0, 0.0, 0.0]);
    }

    #[test]
    fn pose_marker_translates_with_the_given_pose() {
        use approx::assert_relative_eq;
        use nalgebra::Vector3;
        use slam_core::SO3;

        let mut scene = Scene::new();
        let translation = Vector3::new(5.0, -1.0, 3.0);
        let pose = SE3::new(SO3::identity(), translation);
        scene.add_pose_marker(&pose, 1.0, [1.0, 0.0, 1.0]);

        // Every line's start vertex in this marker either is the camera
        // center itself or (for the base square's edges) a translated
        // corner — either way, translating the pose must shift every
        // vertex's position by exactly `translation` relative to the
        // identity-pose case, since there's no rotation involved here.
        let mut identity_scene = Scene::new();
        identity_scene.add_pose_marker(&SE3::identity(), 1.0, [1.0, 0.0, 1.0]);
        for (moved, base) in scene.vertices.iter().zip(identity_scene.vertices.iter()) {
            let moved_pos = Vector3::new(moved.position[0] as f64, moved.position[1] as f64, moved.position[2] as f64);
            let base_pos = Vector3::new(base.position[0] as f64, base.position[1] as f64, base.position[2] as f64);
            assert_relative_eq!(moved_pos - base_pos, translation, epsilon = 1e-5);
        }
    }
}
