/// Per-stage wall-clock timing for one sequence run, plus the duration of
/// sensor data actually processed — the basis for `plan/STAGE2.md`'s
/// real-time bar. `vision_seconds` and `optimization_seconds` are the
/// continuous, per-frame VIO loop (frontend tracking + windowed backend
/// optimization) that would need to keep up with a live sensor;
/// `global_ba_seconds` and `loop_closure_seconds` are separate, one-shot
/// batch passes not held to the same per-frame bar (see the plan's M5
/// scope note).
#[derive(Debug, Clone, Copy, Default)]
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
    }
}
