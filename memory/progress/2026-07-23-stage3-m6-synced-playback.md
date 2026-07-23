---
name: stage3-m6-synced-playback
description: Stage 3 M6 done — one shared cursor (the video panel's own scrub_index) now drives a 3D-panel highlight marker and a graphs-panel VLine, all reading the same per-keyframe index the three panels' underlying data already shared by construction.
metadata:
  type: progress
---

# Stage 3 M6 — synced playback across all three panels (done)

## What landed

- `VideoPlayer::scrub_index()`: a public accessor for the panel's own
  playback position (already existed internally since M4, just not
  exposed).
- `LoadedTrajectory::positions: Vec<Point3<f64>>` (`scene_load.rs`):
  the estimated trajectory's raw world-space points, same index space
  as the already-existing `timestamps`/`ate_series` fields — needed so
  `App` can look up "where is the cursor's keyframe in 3D space" every
  frame without re-parsing `trajectory.csv`.
- `App`: reads `self.video.scrub_index()` once per frame, *after*
  the video panel's own `ui()` call (so that frame's slider drag or
  play-auto-advance is already applied), then:
  - 3D panel: clones the persistent `Scene` (cheap — a few hundred
    vertices) and adds a bright-yellow crosshair at
    `positions[cursor]` before rendering, so the highlight moves every
    frame without leaving stale markers or needing a reload.
  - Graphs panel: `GraphsPanel::ui` gained a `cursor_index: Option<usize>`
    parameter, drawing an `egui_plot::VLine` at that x-position on the
    ATE plot.
  - Video panel: already had this from M4 (`cursor -> timestamps[cursor]
    -> nearest cam0 frame`), no change needed.

## Why "sync" didn't need new cross-checking logic

`positions`, `timestamps`, and `ate_series` are all built from the same
`slam_eval::TrajectoryPoints` in `load_run_scene`, one field per CSV
column of the same run — they're guaranteed equal-length and row-
aligned by construction, not by a separate synchronization mechanism
that could drift. The "shared cursor" *is* just one integer indexing
all three the same way (directly for two of them, through the existing
timestamp->frame lookup for the third). This is why M6 turned out to
be mostly wiring, not new data-model work — flagged as likely in M5's
own writeup ("the video panel's scrub index and the graphs panel's
per-keyframe x-axis are already the same index space... mostly 'add
one shared cursor,' not a new data model"), which held up.

## Verification

`scene_load.rs`'s existing real-fixture test extended (not duplicated
with a parallel "sync" test) to also assert `positions.len() == n` and
spot-check the first/last position values against the known straight-
line fixture — this, combined with the pre-existing `timestamps`/
`ate_series` length assertions on the same fixture, *is* the
consistency check the plan's M6 text asked for: all three built from
the same source, same length, same order, checked against known
values. A separate "given a cursor, check all three resolve
consistently" test would just be re-verifying the same construction a
second way, so wasn't added on top.

`cargo build`/`cargo clippy -p slam-viz --all-targets` clean, `cargo
test -p slam-viz` all 9 passing unchanged in count (this milestone
extended an existing test rather than adding new ones — a legitimate
outcome when the new behavior's correctness follows directly from
already-tested construction, not a gap). Re-ran `--dump-scene-stats`
against this session's real `runs/` history to confirm the `Loaded
Trajectory` change didn't regress the existing real-data path. Full
workspace `cargo test`/`cargo clippy` re-run for regressions elsewhere.
Windowed visual confirmation (does the highlight/cursor-line actually
*look* synced while dragging the video scrub bar) needs the user's own
`cargo run --release --bin slam-viz` — same as every prior interactive
half this stage.

Next: `plan/STAGE3.md` M7 (run-browser polish — this is largely already
working via M0/M3's run picker; M7's remaining scope is mostly the
optional multi-run comparison stretch goal, explicitly not required).
With M0-M6 done, Stage 3's three goals are functionally met: a hand-
written 3D rendering library (goal 1), an app showing the trajectory
next to video and graphs, now synced (goal 2), and per-run browsing
(goal 3, via the run picker). M7 is polish on top of a working whole,
not a missing capability.
