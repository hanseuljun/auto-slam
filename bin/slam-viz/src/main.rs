//! `slam-viz`: the visualization application (`plan/STAGE3.md` goals
//! 2/3) — a run picker (Stage 3 M0's `runs/<sequence>/<run_id>/` history)
//! next to a 3D trajectory view built on `slam-render` (goal 1).

mod app;
mod runs;
mod scene_load;

use std::path::PathBuf;

use clap::Parser;

#[derive(Parser)]
#[command(name = "slam-viz", about = "Browse bin/slam-run's per-run history and view a run's trajectory in 3D")]
struct Cli {
    /// Directory containing per-sequence run history (Stage 3 M0's
    /// `runs/<sequence>/<run_id>/` layout, written by `bin/slam-run`).
    #[arg(long, default_value = "runs")]
    runs_dir: PathBuf,

    /// Skip opening a window: discover runs, load the most recent one's
    /// scene, print counts, and exit. A fast, scriptable smoke check for
    /// this app's non-visual logic (`plan/STAGE3.md`'s "Verifying a GUI
    /// deliverable" #3) — not a substitute for actually looking at the
    /// app, just an early warning between visual checks (and usable in
    /// this repo's own `cargo test`-adjacent verification without a
    /// human at a keyboard).
    #[arg(long)]
    dump_scene_stats: bool,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    if cli.dump_scene_stats {
        return dump_scene_stats(&cli.runs_dir);
    }

    let runs_dir = cli.runs_dir;
    eframe::run_native("slam-viz", eframe::NativeOptions::default(), Box::new(move |_cc| Box::new(app::App::new(runs_dir)))).map_err(|e| anyhow::anyhow!("eframe error: {e}"))
}

fn dump_scene_stats(runs_dir: &std::path::Path) -> anyhow::Result<()> {
    let discovered = runs::discover_runs(runs_dir);
    println!("discovered {} run(s) under {}", discovered.len(), runs_dir.display());
    for run in &discovered {
        println!("  {} / {} — ATE rmse={:.3}m", run.meta.sequence_name, run.meta.run_id, run.meta.ate.rmse);
    }

    let Some(latest) = discovered.first() else {
        println!("no runs to load a scene from");
        return Ok(());
    };
    let loaded = scene_load::load_run_scene(&latest.dir)?;
    println!(
        "loaded scene for {} / {}: {} vertices, center=({:.2}, {:.2}, {:.2}), extent={:.2}",
        latest.meta.sequence_name,
        latest.meta.run_id,
        loaded.scene.vertices.len(),
        loaded.center.x,
        loaded.center.y,
        loaded.center.z,
        loaded.extent
    );
    Ok(())
}
