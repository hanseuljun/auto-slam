use std::path::PathBuf;

use nalgebra::Point3;
use slam_render::{GpuContext, LineRenderer, OffscreenTarget, OrbitCamera, Scene};

use crate::graphs::GraphsPanel;
use crate::runs::{discover_runs, DiscoveredRun};
use crate::scene_load::load_run_scene;
use crate::video::VideoPlayer;

/// `bin/slam-viz`'s application state (`plan/STAGE3.md` M3): a run
/// picker (left panel) plus a 3D trajectory view (central panel). The 3D
/// view is rendered off-screen via `slam-render`'s already-tested
/// `LineRenderer`/`OffscreenTarget` (its own `wgpu::Device`, entirely
/// separate from `eframe`'s), then displayed as a plain `egui` texture —
/// simpler and lower-risk than sharing a single `wgpu` device/render
/// pass between `egui`'s own rendering and a hand-written pipeline, at
/// the cost of a CPU pixel round-trip per frame. That cost is a
/// non-issue for this milestone: `plan/STAGE3.md` scopes this app as
/// post-hoc visualization of a completed run, not anything held to the
/// real-time bar Stage 2 earned.
pub struct App {
    runs_dir: PathBuf,
    runs: Vec<DiscoveredRun>,
    selected_dir: Option<PathBuf>,
    error: Option<String>,

    gpu: GpuContext,
    offscreen: OffscreenTarget,
    renderer: LineRenderer,
    scene: Scene,
    camera: OrbitCamera,
    video: VideoPlayer,
    graphs: GraphsPanel,
    /// The estimated trajectory's own world-space positions for the
    /// currently loaded run, same index space as the video panel's
    /// `scrub_index` — `plan/STAGE3.md` M6's synced-cursor highlight in
    /// the 3D panel reads this rather than re-loading `trajectory.csv`
    /// every frame.
    positions: Vec<Point3<f64>>,
}

impl App {
    pub fn new(runs_dir: PathBuf, data_dir: PathBuf) -> Self {
        let gpu = GpuContext::new().expect("slam-viz requires a GPU adapter to render the 3D panel");
        let offscreen = OffscreenTarget::new(&gpu, 640, 480);
        let renderer = LineRenderer::new(&gpu, slam_render::OFFSCREEN_COLOR_FORMAT);
        let camera = OrbitCamera::new(Point3::origin(), 10.0, 640.0 / 480.0);
        let video = VideoPlayer::new(data_dir);
        let graphs = GraphsPanel::default();

        let mut app = App { runs_dir, runs: Vec::new(), selected_dir: None, error: None, gpu, offscreen, renderer, scene: Scene::new(), camera, video, graphs, positions: Vec::new() };
        app.refresh_runs();
        if let Some(first) = app.runs.first().cloned() {
            app.select_run(&first);
        } else {
            app.scene.add_grid(10.0, 10, [0.3, 0.3, 0.3]);
            app.scene.add_axes(2.0);
        }
        app
    }

    fn refresh_runs(&mut self) {
        self.runs = discover_runs(&self.runs_dir);
    }

    fn select_run(&mut self, run: &DiscoveredRun) {
        match load_run_scene(&run.dir) {
            Ok(loaded) => {
                self.video.load_for_run(&run.meta.sequence_name, loaded.timestamps.clone());
                self.graphs.load_for_run(loaded.ate_series.clone(), run.meta.timing);
                self.positions = loaded.positions;
                self.scene = loaded.scene;
                self.camera.target = loaded.center;
                self.camera.distance = (loaded.extent * 1.5).max(1.0);
                self.selected_dir = Some(run.dir.clone());
                self.error = None;
            }
            Err(e) => {
                self.error = Some(format!("failed to load {}: {e}", run.dir.display()));
            }
        }
    }

    fn run_picker(&mut self, ui: &mut egui::Ui) {
        ui.heading("Runs");
        if ui.button("Refresh").clicked() {
            self.refresh_runs();
        }
        ui.separator();
        if self.runs.is_empty() {
            ui.label(format!("No runs found under {}", self.runs_dir.display()));
            ui.label("Run `cargo run --release --bin slam-run` first.");
        }
        let mut clicked: Option<DiscoveredRun> = None;
        egui::ScrollArea::vertical().show(ui, |ui| {
            for run in &self.runs {
                let selected = self.selected_dir.as_deref() == Some(run.dir.as_path());
                let label = format!("{}\n{}\nATE rmse {:.3}m, RT factor {}", run.meta.sequence_name, run.meta.run_id, run.meta.ate.rmse, run.meta.timing.map(|t| format!("{:.3}", t.real_time_factor())).unwrap_or_else(|| "n/a".to_string()));
                if ui.selectable_label(selected, label).clicked() {
                    clicked = Some(run.clone());
                }
            }
        });
        if let Some(run) = clicked {
            self.select_run(&run);
        }
        if let Some(err) = &self.error {
            ui.separator();
            ui.colored_label(egui::Color32::RED, err);
        }
    }

    fn trajectory_view(&mut self, ui: &mut egui::Ui) {
        let avail = ui.available_size();
        let (width, height) = ((avail.x.max(1.0)) as u32, (avail.y.max(1.0)) as u32);
        if self.offscreen.width != width || self.offscreen.height != height {
            self.offscreen = OffscreenTarget::new(&self.gpu, width, height);
            self.camera.aspect = width as f64 / height as f64;
        }

        // A cheap per-frame clone (a few hundred vertices at most) rather
        // than mutating `self.scene` permanently: the cursor highlight
        // (`plan/STAGE3.md` M6's synced playback) needs to move every
        // frame the video panel's scrub position changes, without
        // re-loading the run or leaving stale highlight markers behind
        // from a previous cursor position.
        let mut frame_scene = self.scene.clone();
        if let Some(&cursor_pos) = self.positions.get(self.video.scrub_index()) {
            frame_scene.add_point_marker(&cursor_pos, self.camera.distance * 0.02, [1.0, 1.0, 0.0]);
        }

        self.renderer.render(&self.gpu, &frame_scene, &self.camera.view_projection_matrix(), self.offscreen.color_view(), self.offscreen.depth_view(), wgpu::Color { r: 0.05, g: 0.05, b: 0.08, a: 1.0 });
        let pixels = self.offscreen.read_pixels_rgba8(&self.gpu);
        let image = egui::ColorImage::from_rgba_unmultiplied([width as usize, height as usize], &pixels);
        let texture = ui.ctx().load_texture("slam-viz-3d-view", image, egui::TextureOptions::LINEAR);

        let response = ui.add(egui::Image::new(&texture).sense(egui::Sense::click_and_drag()));
        if response.dragged_by(egui::PointerButton::Primary) {
            let delta = response.drag_delta();
            self.camera.orbit(-delta.x as f64 * 0.006, -delta.y as f64 * 0.006);
        } else if response.dragged_by(egui::PointerButton::Secondary) {
            let delta = response.drag_delta();
            self.camera.pan(-delta.x as f64 * 0.0025, delta.y as f64 * 0.0025);
        }
        let scroll = ui.input(|i| i.raw_scroll_delta.y);
        if scroll.abs() > 0.0 {
            self.camera.zoom((-scroll as f64 * 0.002).exp());
        }
        ui.ctx().request_repaint();
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("run_picker").min_width(240.0).show(ctx, |ui| self.run_picker(ui));
        egui::SidePanel::right("video_panel").min_width(320.0).show(ctx, |ui| self.video.ui(ui));
        // Reads video's scrub_index *after* its own ui() call above has
        // already applied this frame's slider drag/play-advance, so the
        // graphs panel's cursor line and the 3D panel's highlight below
        // both reflect the same up-to-date instant (`plan/STAGE3.md` M6).
        let cursor = self.video.scrub_index();
        egui::TopBottomPanel::bottom("graphs_panel").min_height(220.0).show(ctx, |ui| self.graphs.ui(ui, Some(cursor)));
        egui::CentralPanel::default().show(ctx, |ui| self.trajectory_view(ui));
    }
}
