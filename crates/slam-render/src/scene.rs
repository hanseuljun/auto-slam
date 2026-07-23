use nalgebra::Point3;

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
}
