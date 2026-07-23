use egui_plot::{Bar, BarChart, Line, Plot, PlotPoints, VLine};

/// `plan/STAGE3.md` M5's graphs panel: per-keyframe ATE over the run,
/// plus a per-stage timing breakdown bar chart. `egui_plot` renders the
/// charts themselves — UI chart-widget infra, not this stage's "3D
/// rendering library" goal (`memory/decisions/0018`'s dependency-policy
/// split), same category as `egui` itself.
#[derive(Default)]
pub struct GraphsPanel {
    ate_series: Vec<f64>,
    timing: Option<slam_eval::TimingBreakdown>,
}

impl GraphsPanel {
    /// Called from `App::select_run` with the newly loaded run's own
    /// data — no separate load step, since both inputs are already
    /// computed by the time a run is selected (`scene_load::
    /// load_run_scene`'s `ate_series`, `RunMeta::timing`).
    pub fn load_for_run(&mut self, ate_series: Vec<f64>, timing: Option<slam_eval::TimingBreakdown>) {
        self.ate_series = ate_series;
        self.timing = timing;
    }

    /// `cursor_index` is the shared playback position (`plan/STAGE3.md`
    /// M6, driven by the video panel's own scrub index) — drawn as a
    /// vertical line on the ATE plot so all three panels visibly track
    /// the same instant, not just the video and 3D panels.
    pub fn ui(&self, ui: &mut egui::Ui, cursor_index: Option<usize>) {
        ui.heading("Graphs");
        if self.ate_series.is_empty() {
            ui.label("No run selected.");
            return;
        }

        ui.label("ATE (aligned position error) per keyframe, meters:");
        let points: PlotPoints = self.ate_series.iter().enumerate().map(|(i, &e)| [i as f64, e]).collect();
        Plot::new("slam-viz-ate-plot").height(180.0).show(ui, |plot_ui| {
            plot_ui.line(Line::new(points).name("ATE"));
            if let Some(cursor) = cursor_index {
                plot_ui.vline(VLine::new(cursor as f64).name("cursor"));
            }
        });

        if let Some(t) = self.timing {
            ui.separator();
            ui.label("Timing breakdown, seconds:");
            let bars = vec![
                Bar::new(0.0, t.vision_seconds).name("vision"),
                Bar::new(1.0, t.optimization_seconds).name("optimization"),
                Bar::new(2.0, t.global_ba_seconds).name("global BA"),
                Bar::new(3.0, t.loop_closure_seconds).name("loop closure"),
            ];
            Plot::new("slam-viz-timing-plot").height(150.0).show(ui, |plot_ui| {
                plot_ui.bar_chart(BarChart::new(bars));
            });
            ui.label(format!("real-time factor: {:.3}", t.real_time_factor()));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_run_selected_by_default() {
        let panel = GraphsPanel::default();
        assert!(panel.ate_series.is_empty());
        assert!(panel.timing.is_none());
    }

    #[test]
    fn load_for_run_stores_the_given_series_and_timing_verbatim() {
        let mut panel = GraphsPanel::default();
        let series = vec![0.1, 0.2, 0.15];
        let timing = slam_eval::TimingBreakdown { vision_seconds: 1.0, optimization_seconds: 2.0, global_ba_seconds: 3.0, loop_closure_seconds: 0.5, data_seconds: 10.0 };
        panel.load_for_run(series.clone(), Some(timing));
        assert_eq!(panel.ate_series, series);
        assert!(panel.timing.is_some());
    }
}
