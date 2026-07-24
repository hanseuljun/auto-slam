/// Per-stage wall-clock timing for one sequence run, plus the duration of
/// sensor data actually processed — the basis for `plan/STAGE2.md`'s
/// real-time bar. `vision_seconds` and `optimization_seconds` are the
/// continuous, per-frame VIO loop (frontend tracking + windowed backend
/// optimization) that would need to keep up with a live sensor;
/// `global_ba_seconds` and `loop_closure_seconds` are separate, one-shot
/// batch passes not held to the same per-frame bar (see the plan's M5
/// scope note).
#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
pub struct TimingBreakdown {
    pub vision_seconds: f64,
    pub optimization_seconds: f64,
    pub global_ba_seconds: f64,
    pub loop_closure_seconds: f64,
    pub data_seconds: f64,
}

impl TimingBreakdown {
    /// Wall-clock spent in the continuous VIO loop divided by the amount
    /// of sensor data that loop processed. <= 1.0 is `plan/STAGE2.md`'s
    /// real-time bar; global BA and loop closure are deliberately excluded
    /// (they're batch passes, not per-frame).
    pub fn real_time_factor(&self) -> f64 {
        if self.data_seconds <= 0.0 {
            return f64::INFINITY;
        }
        (self.vision_seconds + self.optimization_seconds) / self.data_seconds
    }

    /// Wall-clock for *everything* (VIO loop + global BA + loop closure)
    /// divided by the amount of sensor data processed — `plan/STAGE4.md`
    /// goal 2's redefinition of "real-time," covering what `real_time_
    /// factor` deliberately excludes. Needed because `real_time_factor`
    /// alone can look real-time while total wall-clock isn't: on a full,
    /// un-truncated `MH_01_easy` run before Stage 4 M1's fix, this factor
    /// was ~5.9 (957s of global BA against 184s of data) while `real_time_
    /// factor` still reported 0.686 (`memory/progress/2026-07-23-stage4-
    /// m0-mh01-full-sequence-measured.md`) — a technically-passing number
    /// that didn't mean what it used to once global BA stopped being
    /// negligible.
    pub fn whole_run_factor(&self) -> f64 {
        if self.data_seconds <= 0.0 {
            return f64::INFINITY;
        }
        (self.vision_seconds + self.optimization_seconds + self.global_ba_seconds + self.loop_closure_seconds) / self.data_seconds
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn real_time_factor_of_one_means_keeping_up_exactly() {
        let t = TimingBreakdown { vision_seconds: 6.0, optimization_seconds: 4.0, global_ba_seconds: 100.0, loop_closure_seconds: 0.0, data_seconds: 10.0 };
        assert!((t.real_time_factor() - 1.0).abs() < 1e-12, "global BA time must not count toward the per-frame real-time factor");
    }

    #[test]
    fn zero_data_seconds_reports_infinite_factor_not_a_divide_by_zero_panic() {
        let t = TimingBreakdown::default();
        assert!(t.real_time_factor().is_infinite());
        assert!(t.whole_run_factor().is_infinite());
    }

    #[test]
    fn whole_run_factor_counts_global_ba_and_loop_closure_unlike_real_time_factor() {
        let t = TimingBreakdown { vision_seconds: 6.0, optimization_seconds: 4.0, global_ba_seconds: 8.0, loop_closure_seconds: 2.0, data_seconds: 10.0 };
        assert!((t.real_time_factor() - 1.0).abs() < 1e-12);
        assert!((t.whole_run_factor() - 2.0).abs() < 1e-12, "whole_run_factor must include global BA + loop closure, unlike real_time_factor");
    }
}
