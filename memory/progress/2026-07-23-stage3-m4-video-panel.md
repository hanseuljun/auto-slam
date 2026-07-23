---
name: stage3-m4-video-panel
description: Stage 3 M4 done — bin/slam-viz gained a video panel synced to the selected run's keyframe timestamps, via a new slam_dataset::EuRocSequence::nearest_cam0_frame_index lookup, verified against the real MH_01_easy dataset already in this repo.
metadata:
  type: progress
---

# Stage 3 M4 — video frame panel (done)

## What landed

- `slam_dataset::EuRocSequence::nearest_cam0_frame_index(timestamp_ns)`
  (`crates/slam-dataset/src/sequence.rs`): binary search over
  `cam0_frames` (already guaranteed sorted by `load`) for the frame
  closest to a query timestamp, with proper tie-breaking and clamping
  at both ends. `plan/STAGE3.md`'s M4 text assumed this kind of lookup
  already existed somewhere in `slam-dataset` to "reuse" — it didn't;
  the closest existing thing was `GroundTruthTrajectory::interpolate`
  in `slam-eval`, which interpolates *between* two states rather than
  finding a discrete nearest frame, a different operation. Added the
  real thing to the crate that actually owns `CameraFrame`/
  `EuRocSequence`, which is where the plan's own crate-boundary logic
  says it belongs.
- `bin/slam-viz/src/video.rs`: `VideoPlayer` — loads a run's sequence
  lazily from `<data_dir>/<sequence_name>/mav0` when a run is selected
  (a second, independent load from the dataset a run was originally
  computed against; a run's own `trajectory.csv` only carries
  timestamps/positions, not raw frames). Scrub slider moves through the
  run's own keyframe-timestamp index space (not raw `cam0` indices —
  already the natural granularity a trajectory viewer's user cares
  about), each position mapped to a `cam0` frame via the new lookup,
  decoded and displayed as an `egui` texture (grayscale bytes
  replicated to RGBA, no dedicated grayscale texture format used, kept
  simple). Play/pause auto-advances at a fixed ~10 keyframes/sec.
- Wired into `App`: `select_run` now also calls `video.load_for_run`,
  and `update()` adds a right-side `egui::SidePanel` for it. Added a
  `--data-dir` CLI flag (default `data/machine_hall`, matching `bin/
  slam-run`'s own convention) so the video panel knows where to find
  raw sequence data independent of the `--runs-dir` used for history.

## Verification

4 new `slam-dataset` tests for `nearest_cam0_frame_index` (exact match,
between-two-frames picks the closer one, before-first/after-last
clamps to the nearest end, single-frame array). 2 new `bin/slam-viz`
tests for `VideoPlayer` — both against the *real* `MH_01_easy` dataset
already checked into this repo (`data/machine_hall/MH_01_easy/`), not
synthetic fixtures: loading a real sequence and syncing to a real
`cam0_frames[10]`'s own timestamp correctly resolves back to index 10;
a nonexistent sequence name sets a user-visible error string, not a
panic. Had to fix the working-directory assumption in these new tests
(`cargo test`'s CWD is the crate's own manifest directory, not the
workspace root — used `env!("CARGO_MANIFEST_DIR")` + `../../data/
machine_hall`, the same convention `slam-dataset`'s own pre-existing
tests already use, rather than a bare relative path that would only
have worked by accident depending on how tests are invoked).

`cargo clippy --workspace --all-targets` clean; full workspace `cargo
test` re-run to confirm no regressions elsewhere. Windowed play/scrub
itself not run by the agent — same reasoning as every prior
milestone's interactive half (M1's `orbit_demo`, M3's run-picker/3D
drag controls): needs the user's own `cargo run --release --bin
slam-viz` to confirm visually.

Next: `plan/STAGE3.md` M5 (graphs panel — ATE/RPE/timing plotted
alongside the 3D and video panels, likely via `egui_plot`) or M6
(synced playback across all three panels — the video panel's own
scrub index already *is* the shared cursor concept M6 needs, so this
milestone incidentally laid groundwork for it too).
