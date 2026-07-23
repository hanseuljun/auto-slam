---
name: stage3-m3-slam-viz-app-shell
description: Stage 3 M3 done — new bin/slam-viz app (egui run picker + 3D panel), verified end-to-end against this session's own real runs/ history via a --dump-scene-stats flag, not just synthetic fixtures.
metadata:
  type: progress
---

# Stage 3 M3 — `bin/slam-viz` application shell + 3D panel (done)

## What landed

New binary crate `bin/slam-viz`, three modules:

- `runs.rs`: `discover_runs(runs_dir)` scans `runs/<sequence>/<run_id>/
  meta.json` (Stage 3 M0's layout), returns them most-recent-first,
  silently skips anything without a valid `meta.json` (including the
  older `runs/<sequence>/trajectory.csv` latest-snapshot path M0 kept
  additive — a real case that needed its own test, not just an assumed
  non-issue).
- `scene_load.rs`: `load_run_scene(run_dir)` — reads a run's
  `trajectory.csv` via `slam_eval::read_trajectory_csv` (M2), builds a
  `slam_render::Scene` (grid, estimated + groundtruth polylines in
  distinct colors, keyframe-pose markers along the estimated path via
  `Scene::add_pose_marker`), and computes a bounding-box center/extent
  so the camera can frame a newly loaded trajectory sensibly instead of
  starting from a fixed default that might not show anything depending
  on the run's real-world scale. This is the actual "data adapter" `plan/
  STAGE3.md` M2 originally scoped inside `slam-render` — landed here
  instead per M2's own documented scope refinement
  (`memory/progress/2026-07-23-stage3-m2-...md`).
- `app.rs`: the `eframe::App` — a left run-picker panel (sequence/run_id/
  ATE/real-time-factor per entry, click to load) and a central 3D panel.

## The one real engineering decision: how the 3D panel actually renders

The plan's own text ("loads the selected run's trajectory into the M2
3D panel") left open *how* `slam-render`'s custom `wgpu` pipeline gets
into an `egui` UI. The textbook-correct way (`egui_wgpu`'s custom
paint-callback API) shares one `wgpu::Device` between `egui`'s own
rendering and a caller's custom draw calls — but the callback hands you
an *already-open* render pass, not a fresh encoder, so `LineRenderer`
(built around owning its own render pass, encoder, and depth attachment)
would need a real second code path just for that integration, with
depth-buffer format/attachment-set compatibility to get right too.
Went with the simpler, lower-risk option instead: the 3D panel renders
into `slam-render`'s already-tested `OffscreenTarget` (its own,
independent `wgpu::Device` — confirmed `eframe` doesn't even default to
`wgpu` for its own rendering, it pulled in `glow`/OpenGL crates during
the build, so there was never a device to share regardless), reads the
pixels back, and displays them as a plain `egui::ColorImage` texture.
Real cost: one CPU pixel round-trip per frame. Real mitigating factor:
`plan/STAGE3.md` explicitly scopes this app as post-hoc visualization
of a *completed* run, not anything held to Stage 2's hard-won real-time
bar — the tradeoff is the right one for this milestone's actual
requirements, not a shortcut that will need revisiting.

## Verification

5 new `cargo test`s: `discover_runs` sorts multiple sequences/runs
correctly most-recent-first, returns empty (not panics) for a missing
directory, and correctly skips a sequence directory that only has the
old latest-snapshot layout (no `meta.json`) — a real edge case the
mixed-layout M0 design created, worth its own test rather than an
assumption. `load_run_scene` checked against a *known-shape* synthetic
trajectory (a straight 10-unit line) so the bounding-box center/extent
are exactly checkable, not just "non-zero"; missing-`trajectory.csv` is
confirmed to be a real `anyhow` error, not a panic.

Beyond synthetic fixtures: added a `--dump-scene-stats` CLI flag (the
plan's own suggested non-visual smoke check, "Verifying a GUI
deliverable" #3) and ran it against this session's *real* `runs/`
history (7 real runs left over from Stage 3 M0's own testing earlier
this session). It correctly discovered all 7, sorted them, and loaded
the most recent (`MH_05_difficult`) into a real `Scene`: 818 vertices,
center `(14.93, -5.84, -8.25)`, extent `40.31` — genuine end-to-end
proof the whole non-visual pipeline (file discovery -> JSON parsing ->
CSV parsing -> `Scene` construction) works against real pipeline
output, not just data this session invented for a test fixture.

`cargo clippy --workspace --all-targets` clean. Windowed interactive
mode (drag-orbit/pan, scroll-zoom, run-picker clicks) builds and is
clippy-clean but, same as M1's `orbit_demo`, not run by the agent —
launching a real GUI window isn't something this session's tools can
drive or observe; needs the user's own `cargo run --release --bin
slam-viz` to confirm visually.

Next: `plan/STAGE3.md` M4 (video frame panel, reusing `slam-dataset`'s
existing timestamp/frame-index sync logic) or M5 (graphs panel) —
either is now a straightforward "add another `egui` panel" following
this milestone's established pattern, not a new architectural decision.
