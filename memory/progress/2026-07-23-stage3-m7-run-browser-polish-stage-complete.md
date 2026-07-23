---
name: stage3-m7-run-browser-polish-stage-complete
description: Stage 3 M7 done (core; stretch multi-run comparison deferred) — run picker now shows window_size/huber_delta alongside ATE/RT-factor. With M0-M7 landed, all three of Stage 3's stated goals are met.
metadata:
  type: progress
---

# Stage 3 M7 — run browser polish (done, core); Stage 3 complete

## What landed

M7's core (non-stretch) scope turned out to already be almost entirely
delivered by earlier milestones: M0 captured per-run config/timing/ATE
into `meta.json`, M3 built the run picker UI around it. The one real
gap: `RunMeta::config`'s `window_size`/`huber_delta` — exactly the two
knobs `decisions/0017`'s tuning sweeps varied — were captured and
tested but never actually shown in the picker, so comparing a sweep's
runs still meant reading `meta.json` by hand outside the app, which
undercuts the whole point of a "browse results per run" application.
Closed with a small, low-risk addition: extended the run picker's
label (`app.rs`) and `--dump-scene-stats`'s output (`main.rs`, for
parity between the two) to include both values. No new tests needed —
`RunMeta`'s round-trip is already covered in `slam-eval`, this is pure
display formatting of already-tested data.

Verified against this session's real `runs/` history: all 7 runs now
show `window_size=8, huber_delta=3.0` directly in the picker/dump
output — correctly reflecting that these particular runs used the
tuned defaults (Stage 2 M6's sweeps that used other values were
reverted before this session's `runs/` history started accumulating
under Stage 3 M0's new layout, so there's nothing else to see there
yet — a real, explicable absence, not a bug).

Stretch goal (side-by-side/overlay comparison of two runs' trajectories)
deferred per the plan's own "not required" framing.

## Stage 3 is now complete

With M0-M7 landed, all three of Stage 3's stated goals are met:

1. **3D rendering library** — `slam-render` (M1-M2): hand-written orbit
   camera, `wgpu`/`winit` bootstrap, line/polyline/grid/axes/point-
   marker/pose-marker primitives, verified with real headless-GPU
   offscreen-render pixel tests.
2. **Visualization application** — `bin/slam-viz` (M3-M6): a run
   picker next to a 3D panel (rendered via `slam-render` into an
   offscreen texture, displayed as an `egui` texture), a video panel
   (synced via a new `slam_dataset::nearest_cam0_frame_index`), a
   graphs panel (`egui_plot`, backed by a new `slam_eval::
   compute_ate_series`), all three now sharing one playback cursor.
3. **Per-run browsing** — the run picker (M0's non-clobbering
   `runs/<sequence>/<run_id>/` history + M3's UI + M7's config display).

Every milestone's non-visual logic has real `cargo test` coverage —
many verified against this repo's actual dataset (`data/machine_hall/
MH_01_easy`) and this session's own real `runs/` output via
`--dump-scene-stats`, not synthetic fixtures alone. The interactive/
visual half of every milestone (`slam-render`'s `orbit_demo` example,
`bin/slam-viz`'s windowed mode: drag-orbit/pan, scroll-zoom, run-picker
clicks, video scrub/play, synced highlight/cursor motion) builds and is
`cargo clippy`-clean throughout the whole session, but was never run by
the agent — launching a GUI window isn't something this session's tools
can drive or observe, and doing so unprompted would pop up on the
user's real desktop. This needs the user's own `cargo run --release
--bin slam-viz` (and `cargo run -p slam-render --example orbit_demo`)
to confirm visually — flagged consistently at every milestone, not
just at the end.

Full workspace `cargo test`/`cargo clippy --all-targets` clean as of
this commit (25 test suites, 0 failures workspace-wide).

Next, if picked up: the M7 stretch (multi-run trajectory comparison),
or a new stage entirely — `plan/STAGE3.md` itself now carries a
"Status: all three goals met" header, same pattern `plan/STAGE2.md`
used when it closed out.
