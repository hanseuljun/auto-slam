---
name: stage3-m5-graphs-panel
description: Stage 3 M5 done — bin/slam-viz gained a graphs panel (per-keyframe ATE line chart + timing bar chart via egui_plot), backed by a new slam_eval::compute_ate_series that compute_ate itself now reuses internally. RPE-over-time scoped out deliberately, not silently dropped.
metadata:
  type: progress
---

# Stage 3 M5 — graphs panel (done)

## What landed

- `slam_eval::compute_ate_series` (`crates/slam-eval/src/align.rs`):
  the same Umeyama-alignment machinery `compute_ate` already used,
  refactored to expose the full per-point error series in original
  order instead of only summary stats. `compute_ate` now calls this
  internally rather than duplicating the alignment+error computation —
  a real simplification, not just new surface area, and its own
  pre-existing tests (identical points -> zero ATE, known offset ->
  bounded RMSE, Umeyama recovers a known similarity transform) all
  still pass unchanged, confirming the refactor is behavior-preserving.
- `bin/slam-viz/src/graphs.rs`: `GraphsPanel` — an `egui_plot` line
  chart of per-keyframe aligned ATE, and a bar chart of the run's
  timing breakdown (vision/optimization/global BA/loop closure) plus
  its real-time factor. Both plotted from data already loaded at
  run-selection time (no new file I/O triggered by opening this panel):
  `scene_load::LoadedTrajectory` gained an `ate_series: Vec<f64>` field
  (computed in `load_run_scene` via the new `compute_ate_series`,
  right alongside where `estimated`/`groundtruth` are already in
  scope), and `RunMeta::timing` was already available from M0.
  `egui_plot` (a new dependency) renders the charts — UI chart-widget
  infra per `decisions/0018`'s dependency-policy split, not this
  stage's "hand-written rendering library" goal.
- Wired into `App`: `select_run` now also calls `graphs.load_for_run`,
  and `update()` adds a bottom `egui::TopBottomPanel` for it.

## A real, deliberate scope cut (not silently dropped)

`plan/STAGE3.md`'s M5 text mentioned "ATE/RPE over the run." Landed
ATE in full; RPE-over-time would need the identical treatment applied
to a different function (`compute_rpe`/`RpeStats` would need their own
"expose the series" variant, since RPE's per-pair errors currently
only feed into aggregate stats, same as ATE did before this
milestone). Cut for scope, recorded explicitly in `plan/STAGE3.md`'s
M5 entry as a legitimate, bounded follow-up rather than left as an
unstated gap someone has to rediscover by reading the code.

## Verification

2 new `slam-eval` tests: `compute_ate_series` returns results in the
original per-point order (not sorted, unlike the stats path — a real
distinction, since a time-series plot needs index-order data), and the
RMSE computed by hand from the series matches `compute_ate`'s own RMSE
exactly (the refactor's actual correctness guarantee, not just "it
returns something"). 2 new `bin/slam-viz` tests for `GraphsPanel`
(starts empty; `load_for_run` stores what it's given verbatim,
including a `None` timing case). Strengthened `scene_load.rs`'s
existing `loads_a_real_trajectory_csv_...` test to also assert the new
`ate_series` field: for a fixture where estimated == groundtruth, every
point must be ~zero, in the same order and count as the trajectory.

Real-data check via `--dump-scene-stats` (unchanged CLI flag, extended
to print the new field): `MH_05_difficult`'s real run loads a 101-point
ATE series — consistent with that run's already-reported 0.455m
summary ATE (the exact series-to-RMSE equivalence is what `slam-eval`'s
new unit test actually proves; this is just confirming the real-data
plumbing produces a sane count, same spirit as M3/M4's own real-data
checks).

`cargo clippy --workspace --all-targets` clean; full workspace `cargo
test` re-run to confirm no regressions elsewhere.

Next: `plan/STAGE3.md` M6 (synced playback across all three panels —
the video panel's scrub index and the graphs panel's per-keyframe
x-axis are already the same index space as the 3D panel's trajectory
points, so this is now mostly "add one shared cursor," not a new data
model) or M7 (run-browser polish / multi-run comparison).
