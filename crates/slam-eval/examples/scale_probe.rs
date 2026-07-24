use std::env;

fn main() {
    let path = env::args().nth(1).expect("usage: scale_probe <trajectory.csv> [window_seconds] [step]");
    let window_seconds: f64 = env::args().nth(2).map(|s| s.parse().unwrap()).unwrap_or(20.0);
    let step: usize = env::args().nth(3).map(|s| s.parse().unwrap()).unwrap_or(5);

    let pts = slam_eval::read_trajectory_csv(&path).expect("read csv");
    let n = pts.timestamps.len();
    let data_seconds = (pts.timestamps[n - 1] - pts.timestamps[0]) as f64 * 1e-9;
    let avg_rate = n as f64 / data_seconds;
    let window_len = (window_seconds * avg_rate).round() as usize;
    println!("n={n} data_seconds={data_seconds:.1} avg_rate={avg_rate:.2} kf/s window_len={window_len}");

    let series = slam_eval::compute_sliding_window_scale(&pts.estimated, &pts.groundtruth, window_len, step);
    println!("{} windows", series.len());
    let t0 = pts.timestamps[0];
    for (start, scale) in &series {
        let t = (pts.timestamps[*start] - t0) as f64 * 1e-9;
        println!("t={t:7.1}s  scale={scale:.4}");
    }

    // Whole-trajectory scale for reference.
    if let Some(a) = slam_eval::umeyama_alignment(&pts.estimated, &pts.groundtruth) {
        println!("whole-trajectory scale = {:.4}", a.scale);
    }

    if let Some(ratios) = slam_eval::compute_axis_scale_ratios(&pts.estimated, &pts.groundtruth) {
        println!("per-axis std ratio (rotated est / gt): x={:.3} y={:.3} z={:.3}", ratios.x, ratios.y, ratios.z);
    }
}
