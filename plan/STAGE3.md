# Stage 3: Trajectory visualization

## Goal

Three goals, tackled in this order (each later one leans on the one
before it):

1. **A 3D rendering library that can visualize the trajectory.** A new
   crate, `slam-render`, with hand-written camera/scene/primitive code
   (on top of a GPU API, not a pre-built 3D engine) capable of drawing a
   SLAM trajectory in 3D — polylines, keyframe poses, landmarks.
2. **A visualization application** that shows that 3D trajectory view
   next to the corresponding video frames and diagnostic graphs, all
   kept in sync against a shared playback cursor.
3. **Per-run browsing.** Users can pick which run's results to view in
   that application, not just the most recently computed one — this
   makes Stage 2's `bin/slam-run` output (`docs/RESULTS.md`,
   `memory/decisions/0017`'s tuning sweeps, etc.) inspectable
   interactively instead of only as CSV/text.

Everything else about Stage 1/2 still applies unless stated otherwise
here: same dataset, deterministic runs, and the "own algorithm work,
infra crates fine" dependency policy — extended below to say what
counts as "the algorithm" in a graphics/UI context, since that's a new
kind of decision this stage introduces.

## Dependency policy addendum (extends `decisions/0001`)

Stage 1/2's rule — infra crates fine, SLAM logic hand-written — needs a
concrete answer for graphics/UI, which is a new domain for this repo:

- **The 3D rendering library itself (this stage's goal 1) is hand-
  written**, same as every SLAM algorithm in Stages 1-2: camera
  math (view/projection, orbit/fly controls), the scene/primitive
  abstraction (how polylines, points, and pose markers become GPU draw
  calls), and the trajectory-specific drawing logic. This is the actual
  deliverable the user asked for — pulling in an existing 3D
  scene/rendering engine (`bevy`, `three-d`, `kiss3d`, ...) instead of
  writing `slam-render` would skip the goal, not meet it.
- **The GPU API and windowing are infra**, exactly like `nalgebra` for
  linear algebra or `image` for PNG decoding in Stage 1: `wgpu` (cross-
  platform GPU access), `winit` (window/event loop), `bytemuck`/
  `pollster` (buffer layout / async glue `wgpu` needs). Reimplementing a
  graphics driver or windowing system from scratch would be the graphics
  equivalent of Stage 1's rejected "reimplement PNG decoding" — effort
  with no payoff for the actual goal.
- **Application UI chrome (goal 2's panels, layout, run-picker widgets,
  scrub bar, and the 2D graph plots) is also infra**, not "the rendering
  library" — `egui` (+ `egui-wgpu`/`eframe` for windowing integration)
  for the app shell, `egui_plot` (or equivalent) for the graphs panel.
  This mirrors Stage 1 allowing `plotters`/CSV for `slam-eval` output
  instead of hand-rolling a plotting library: 2D UI chart widgets aren't
  the thing this stage exists to build (the *3D trajectory* rendering
  is), so buying that off the shelf is consistent with the existing
  policy, not a relaxation of it.
- Still no OpenCV/g2o/Ceres/existing SLAM crates, and no changes to any
  Stage 1/2 SLAM algorithm code to accommodate visualization — this
  stage consumes `bin/slam-run`'s existing output (CSV/metadata), it
  doesn't reach back into the VIO loop.

## Scope: post-hoc visualization, not live/in-loop

This stage visualizes *completed* runs (`bin/slam-run`'s output
directory), not the pipeline while it's executing. Threading live
visualization into the real-time-critical VIO loop risks the real-time
bar Stage 2 just spent two milestones earning back — out of scope here,
a legitimate candidate for a later stage if ever wanted.

## Workspace layout additions

```
auto-slam/
  crates/
    slam-render/     # NEW: hand-written 3D rendering library (camera,
                      # scene graph, line/point/frustum primitives) on
                      # top of wgpu — this stage's goal-1 deliverable
  bin/
    slam-viz/         # NEW: the visualization application (goal 2/3) —
                      # 3D trajectory panel + video panel + graphs panel,
                      # synced playback, per-run picker
```

`slam-render` depends on `slam-core` (reuse SO3/SE3 for camera pose
math, no duplicate Lie-group code) but nothing else pipeline-specific —
it's a general small 3D visualization library that happens to be built
for this project, not coupled to VIO internals. `bin/slam-viz` depends
on `slam-render`, `slam-dataset` (video frames, calibration), and
`slam-eval` (trajectory/ATE/RPE types), reusing existing types rather
than re-parsing CSVs with new ad hoc code.

## Milestones

Same discipline as Stages 1-2: each milestone lands as a working,
tested increment, no big-bang integration at the end.

### M0 — Multi-run output layout (small backend prerequisite) — Done

- `bin/slam-run` currently overwrites `runs/<sequence>/trajectory.csv`
  and `runs/summary.csv` on every invocation (confirmed in
  `docs/RESULTS.md`'s "How to reproduce" — one run, one place). Goal 3
  needs *history*: change it to write each invocation to its own
  directory, e.g. `runs/<sequence>/<run_id>/{trajectory.csv,
  meta.json}`, `run_id` a sortable timestamp. `meta.json` carries what a
  run picker needs to show without re-parsing the trajectory CSV: ATE/
  RPE summary, timing breakdown, git commit, config
  (`VioParams`/`SolverConfig` values actually used — the exact kind of
  thing `decisions/0017`'s tuning sweeps changed run to run).
- Keep `runs/summary.csv` (or an equivalent aggregate) as the existing
  cross-sequence table `docs/RESULTS.md` depends on — this is additive,
  not a breaking rename of what Stage 2 already built.
- Test: running `slam-run` twice produces two distinct, non-clobbering
  run directories with correct `meta.json` contents; `cargo test`
  covering the run-id/metadata-writing logic directly (not just via a
  full pipeline run).
- **Result**: landed as designed — `slam-eval::run_meta` (`RunConfig`/
  `RunMeta`, `generate_run_id`, `current_git_commit`, JSON read/write),
  `bin/slam-run` writes `runs/<sequence>/<run_id>/{trajectory.csv,
  meta.json}` per invocation alongside the unchanged latest-snapshot
  paths. 3 new `cargo test`s (`slam-eval` 16 -> 19); verified
  non-clobbering by running `slam-run` on `MH_01_easy` twice and
  confirming two distinct run directories, and re-ran the full
  5-sequence harness to confirm ATE/RT-factor numbers are unchanged
  from `docs/RESULTS.md`'s baseline. Full writeup: `memory/progress/
  2026-07-23-stage3-m0-multi-run-history.md`.

### M1 — `slam-render`: rendering-library foundations — Done

- `wgpu` context + `winit` window/event loop bootstrap; an orbit
  camera (mouse-drag orbit, scroll zoom, pan) with hand-written view/
  projection matrix math; a ground-plane grid and coordinate-axes
  gizmo so orientation is legible at a glance.
- Test: camera math is checkable without a GPU — unit tests that a
  known world point projects to the expected screen-space location for
  a known camera pose (e.g., a point at the look-at target projects to
  the viewport center). A headless/offscreen render smoke test (render
  to a texture, assert non-trivial pixel content) if the dev machine's
  GPU stack supports it in headless mode — flagged as a risk below since
  headless GPU access can be environment-dependent.
- **Result**: the flagged risk turned out not to bite — confirmed via a
  throwaway probe *before* writing any real code that this repo's dev
  machine gets a real headless Metal adapter (`AdapterInfo { name: "Apple
  M1", ..., backend: Metal }`) with no window/display needed, so the
  offscreen smoke test could be a genuine GPU-backed pixel-readback
  test, not just a "did it panic" check. Landed: `OrbitCamera` (7 unit
  tests covering eye/view/projection math, pole-clamping, pan/zoom/orbit),
  `GpuContext` (instance/adapter/device/queue bootstrap), `Scene` (line/
  polyline/grid/axes primitives — polyline support pulled forward from
  M2 since it fell out for free once lines existed), `LineRenderer` +
  `OffscreenTarget` (a real render-to-texture-and-read-pixels-back test:
  renders grid+axes, asserts non-background pixels exist and specifically
  near the viewport center where the gizmo's look-at-target origin
  projects). Also caught and fixed a real portability bug before it could
  bite: `LineRenderer` initially hardcoded its pipeline's color format to
  match the offscreen target, which would have panicked the first time it
  rendered into a window surface using a different native format (common
  on macOS, `Bgra8UnormSrgb`) — fixed by taking `color_format` as a
  parameter instead. 14 `cargo test`s, all passing, `cargo clippy` clean.
  A `cargo run -p slam-render --example orbit_demo` window (mouse-drag
  orbit/pan, scroll zoom) exists for the human-verification half of this
  milestone (see "Verifying a GUI deliverable") — built and clippy-clean,
  but not run by the agent itself, since driving/observing a live GUI
  window isn't something this tool has a way to do; needs the user's own
  visual confirmation.

### M2 — Trajectory & pose-graph primitives

- Extend `slam-render` with the actual drawing primitives this stage
  needs: 3D polylines (estimated trajectory, ground truth, distinct
  colors), point markers (landmarks, if cheaply available from a run's
  output), camera-frustum/axes markers at keyframe poses.
- A data adapter from `slam-eval`'s trajectory/ATE types (not a new,
  parallel CSV parser) into `slam-render` scene objects.
- Test: unit tests for the CSV/data → scene-object conversion (point
  counts, bounding-box sanity against known trajectory extent, correct
  coordinate handedness) using a real `runs/` fixture from Stage 2.
  Visual confirmation (a small example rendering a real trajectory) is
  this milestone's human-verification step — see "Verifying a GUI
  deliverable" below for why that's the right bar here, not a plain-text
  substitute.

### M3 — `bin/slam-viz`: application shell + 3D panel

- New binary: `egui`-based window, a run picker listing
  `runs/<sequence>/<run_id>/` directories from M0 (sequence, timestamp,
  ATE headline number visible in the list), loads the selected run's
  trajectory into the M2 3D panel.
- Test: run-listing/metadata-loading logic unit tested against a fixture
  `runs/` directory (correct runs found, sorted, metadata parsed);
  manual run of the app confirming the 3D panel actually renders the
  selected trajectory.

### M4 — Video frame panel

- Displays `cam0` (and optionally `cam1`) frames from `slam-dataset`,
  synced to a playback time/frame index, with a scrub bar and play/
  pause.
- Reuses `slam-dataset`'s existing timestamp/frame-index lookup rather
  than reimplementing sync logic a second time.
- Test: unit tests for time → frame-index lookup against real dataset
  timestamps; manual confirmation that scrubbing shows the correct
  frame.

### M5 — Graphs panel

- Time-series plots alongside the 3D and video panels: ATE/RPE over the
  run (or per-keyframe, if that granularity is cheaply available),
  per-stage timing breakdown. `egui_plot` (or equivalent) renders the
  plots themselves — UI chart-widget infra, not this stage's "rendering
  library" goal (see dependency policy above).
- Test: unit tests on data-series preparation (run data → plot series,
  correct time/index alignment); manual visual check.

### M6 — Synced playback across all three panels

- One scrubber/time cursor drives all three panels together: the
  highlighted pose/frustum in the 3D view, the displayed video frame,
  and a cursor line in the graphs panel all reflect the same instant.
- Test: an integration test on the underlying time-sync wiring (given a
  cursor time, assert the correct keyframe index / video frame index /
  graph-series index are all selected consistently) — this is testable
  without rendering pixels, so it's a real `cargo test`, not just a
  manual check; manual visual confirmation that the panels are visibly
  in sync is the human-verification layer on top.

### M7 — Run browser polish: this is goal 3's actual delivery

- The run picker (M3) becomes the primary way to answer "what did run
  X look like" — enough metadata per run (sequence, timestamp, ATE
  summary, real-time factor, and ideally the config values that
  differed, e.g. `huber_delta`/`window_size` from `decisions/0017`'s
  sweeps) to pick meaningfully without leaving the app.
- Stretch, optional/deferred unless earlier milestones leave easy room:
  side-by-side or overlay comparison of two runs' trajectories (e.g.,
  re-visualizing Stage 2 M6's window_size=6-vs-8 sweep) — a strong
  validation of the whole stage's investment against real history, but
  not required to meet the three stated goals.

## Verifying a GUI deliverable (extends CLAUDE.md's verification section)

CLAUDE.md's standing rule is `cargo test` plus `bin/slam-inspect`'s
plain-text/CSV output as the human-readable check. That second half
doesn't translate literally here — the entire point of this stage is a
*visual* deliverable, so "read the text output" isn't the right
verification model for `bin/slam-viz` itself. This stage's actual
verification bar, per milestone:

1. **Everything that isn't pixels stays under `cargo test`** — camera
   math, data loading/adapters, time-sync logic, run-metadata parsing.
   This is most of each milestone's real logic and it's fully testable
   without a GPU or a human looking at anything.
2. **`bin/slam-viz` itself is verified by running it and looking at
   it** — there is no substitute for a human confirming the 3D view,
   video panel, and graphs actually look right and stay in sync; this
   replaces (for this stage only) CLAUDE.md's "read the test app's
   output" step with "run the test app and look at it."
3. Where practical, `bin/slam-viz` also gets a `--dump-scene-stats`-
   style flag that prints plain counts (vertices/points/frames loaded,
   panel state) so there's still a fast, scriptable, `cargo test`-
   adjacent smoke check that doesn't require a human in the loop for
   every regression — not a replacement for #2, but a cheap early
   warning between visual checks.

## Out of scope for Stage 3

Same carry-forward list as Stages 1-2 (dense/mesh reconstruction,
multi-session/map-merging, semantic mapping, non-`machine_hall` EuRoC
rooms, other datasets, GPU/SIMD micro-optimization), plus: live/in-loop
visualization during a run (see "Scope" above), video/GIF export or
screen recording, editing/annotation tools, multi-window or multi-
monitor layouts, a web/WASM build target, and VR/AR display — a single
native desktop window is the target for this stage.

## Risks

- **Headless/CI-style automated testing of actual rendering output is
  inherently harder than Stages 1-2's numeric checks** — a wrong camera
  sign still "renders," just wrong, the same silent-bug shape Stage 1's
  own Risks section flagged for the optimizer. Mitigate by pushing as
  much logic as possible out of "must look at pixels to verify" and into
  unit-testable camera/data-adapter math (see "Verifying a GUI
  deliverable" above) — minimize, don't eliminate, the pixels-only
  surface area.
- **GPU/windowing stack behavior is platform-dependent** and this repo
  has one development machine (macOS/Darwin, per Stage 2's own "real-
  time is measured on one machine" caveat) — `wgpu`/`winit` are chosen
  specifically for being cross-platform-honest about this, but don't
  assume behavior verified here ports to Linux/Windows without
  re-checking.
- **Scope creep into a general SLAM GUI** is the obvious failure mode
  for a visualization stage — stay anchored to the three stated goals
  (render a trajectory, show it next to video+graphs, browse past runs)
  and treat anything beyond that (live tuning controls, map editing,
  new SLAM features triggered from the UI) as a different, later stage's
  problem even if it looks like a small addition once the app exists.
- **`bin/slam-run`'s output format is a dependency this stage doesn't
  own** — M0's multi-run layout change touches Stage 2 code; keep it
  additive (existing `docs/RESULTS.md` reproduction instructions and
  `runs/summary.csv` consumers keep working) rather than a breaking
  rename, so this stage doesn't silently invalidate Stage 2's own
  documented verification story.
