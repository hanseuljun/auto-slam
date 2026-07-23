use std::path::Path;

use nalgebra::Point3;
use slam_core::{SE3, SO3};
use slam_render::Scene;

pub const ESTIMATED_COLOR: [f32; 3] = [1.0, 0.6, 0.0];
pub const GROUNDTRUTH_COLOR: [f32; 3] = [0.2, 0.6, 1.0];

/// One run's trajectory, converted into a `slam-render` `Scene` plus
/// enough of a bounding-box summary (`center`/`extent`) to frame an
/// `OrbitCamera` sensibly on load, instead of a fixed default that might
/// not show the trajectory at all depending on its real-world scale.
pub struct LoadedTrajectory {
    pub scene: Scene,
    pub center: Point3<f64>,
    pub extent: f64,
    /// Per-keyframe timestamps, same order as the trajectory's own
    /// points — the video panel's playback index space (`plan/
    /// STAGE3.md` M4).
    pub timestamps: Vec<u64>,
    /// Per-keyframe ATE (Umeyama-aligned position error against
    /// groundtruth), same order/index space as `timestamps` — the
    /// graphs panel's headline series (`plan/STAGE3.md` M5). Empty if
    /// alignment isn't possible (e.g. fewer than 3 points); the graphs
    /// panel treats that the same as "no run selected" rather than
    /// erroring the whole load over a plot that just can't be drawn.
    pub ate_series: Vec<f64>,
    /// The estimated trajectory's own world-space positions, same
    /// order/index space as `timestamps`/`ate_series` — lets `App`
    /// highlight "the keyframe at the current cursor" in the 3D panel
    /// (`plan/STAGE3.md` M6's synced playback) without re-parsing
    /// `trajectory.csv` on every frame just to look up one point.
    pub positions: Vec<Point3<f64>>,
}

/// The actual "data adapter" `plan/STAGE3.md` M2 originally scoped
/// inside `slam-render` itself — landed here instead, in `bin/slam-viz`,
/// since this is the first point in the dependency graph where both
/// `slam-eval` (owns `read_trajectory_csv`) and `slam-render` (owns
/// `Scene`) are already dependencies (`memory/decisions/0018`,
/// `plan/STAGE3.md` M2's Result note).
pub fn load_run_scene(run_dir: &Path) -> anyhow::Result<LoadedTrajectory> {
    let points = slam_eval::read_trajectory_csv(run_dir.join("trajectory.csv"))?;
    anyhow::ensure!(!points.estimated.is_empty(), "trajectory.csv at {} has no rows", run_dir.display());

    let estimated: Vec<Point3<f64>> = points.estimated.iter().map(|v| Point3::from(*v)).collect();
    let groundtruth: Vec<Point3<f64>> = points.groundtruth.iter().map(|v| Point3::from(*v)).collect();
    let (center, extent) = bounding_sphere(&estimated);
    let ate_series = slam_eval::compute_ate_series(&points.estimated, &points.groundtruth).unwrap_or_default();

    let mut scene = Scene::new();
    scene.add_grid((extent * 1.5).max(1.0), 10, [0.3, 0.3, 0.3]);
    scene.add_polyline(&estimated, ESTIMATED_COLOR);
    scene.add_polyline(&groundtruth, GROUNDTRUTH_COLOR);

    // A handful of keyframe-pose markers along the estimated path (not
    // one per point - that would clutter a few-hundred-keyframe
    // trajectory), oriented as identity rotations: `trajectory.csv`
    // only carries positions (`slam_eval::TrajectoryPoints`), not
    // orientations, so these mark "a keyframe was here," not "facing
    // this way" - a real, documented simplification, not a bug.
    let target_marker_count = 15usize;
    let stride = (estimated.len() / target_marker_count).max(1);
    let marker_scale = (extent * 0.03).max(1e-3);
    for i in (0..estimated.len()).step_by(stride) {
        scene.add_pose_marker(&SE3::new(SO3::identity(), estimated[i].coords), marker_scale, [0.8, 0.8, 0.8]);
    }

    Ok(LoadedTrajectory { scene, center, extent, timestamps: points.timestamps, ate_series, positions: estimated })
}

/// Axis-aligned bounding box center + diagonal extent, used to frame the
/// `OrbitCamera` on a newly loaded trajectory. Not a true minimal
/// bounding *sphere* despite the name's informal use elsewhere in this
/// file - a box center is more than good enough for framing a camera and
/// is far cheaper than a real min-enclosing-sphere computation.
fn bounding_sphere(points: &[Point3<f64>]) -> (Point3<f64>, f64) {
    if points.is_empty() {
        return (Point3::origin(), 1.0);
    }
    let mut min = points[0];
    let mut max = points[0];
    for p in points {
        min = Point3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
        max = Point3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
    }
    let center = Point3::from((min.coords + max.coords) * 0.5);
    let extent = (max - min).norm().max(1e-6);
    (center, extent)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn loads_a_real_trajectory_csv_with_sane_bounds_and_vertex_counts() {
        let dir = std::env::temp_dir().join(format!("slam-viz-test-scene-load-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();

        // A known, simple trajectory: a straight line from (0,0,0) to
        // (10,0,0), so the bounding-box center and extent are exactly
        // checkable, not just "non-zero."
        let n = 21;
        let estimated: Vec<Vector3<f64>> = (0..n).map(|i| Vector3::new(i as f64 * 0.5, 0.0, 0.0)).collect();
        let groundtruth = estimated.clone();
        let timestamps: Vec<u64> = (0..n as u64).collect();
        slam_eval::write_trajectory_csv(dir.join("trajectory.csv"), &timestamps, &estimated, &groundtruth).unwrap();

        let loaded = load_run_scene(&dir).expect("load should succeed");

        assert!((loaded.center.x - 5.0).abs() < 1e-9, "expected bounding-box center at x=5.0, got {}", loaded.center.x);
        assert!((loaded.center.y).abs() < 1e-9);
        assert!((loaded.center.z).abs() < 1e-9);
        assert!((loaded.extent - 10.0).abs() < 1e-9, "expected extent 10.0 (a straight 10-unit line), got {}", loaded.extent);

        // Grid + 2 polylines (estimated, groundtruth, n-1 segments each)
        // + pose markers - just check it's at least the two polylines'
        // worth of vertices, not an exact count that would break on
        // every unrelated tuning of grid density or marker stride.
        let min_expected_polyline_vertices = 2 * (n - 1) * 2;
        assert!(loaded.scene.vertices.len() >= min_expected_polyline_vertices, "expected at least {} vertices from the two polylines alone, got {}", min_expected_polyline_vertices, loaded.scene.vertices.len());

        assert_eq!(loaded.timestamps, timestamps, "loaded timestamps must match trajectory.csv's own timestamp column, for the video panel's playback sync (plan/STAGE3.md M4)");

        // estimated == groundtruth here, so the aligned ATE series must
        // be (near-)zero at every point, and in the same order/length as
        // the trajectory itself - the graphs panel's headline series
        // (plan/STAGE3.md M5).
        assert_eq!(loaded.ate_series.len(), n);
        assert!(loaded.ate_series.iter().all(|&e| e < 1e-9), "identical estimated/groundtruth trajectories must give ~zero ATE at every point, got {:?}", loaded.ate_series);

        assert_eq!(loaded.positions.len(), n);
        assert!((loaded.positions[0] - Point3::origin()).norm() < 1e-9);
        assert!((loaded.positions[n - 1] - Point3::new(10.0, 0.0, 0.0)).norm() < 1e-9);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn missing_trajectory_csv_is_a_real_error_not_a_panic() {
        let dir = std::env::temp_dir().join(format!("slam-viz-test-scene-load-missing-{}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).unwrap();
        assert!(load_run_scene(&dir).is_err());
        std::fs::remove_dir_all(&dir).ok();
    }
}
