# auto-slam

A stereo-inertial SLAM program and reconstruction library, written in Rust
from scratch: feature detection/tracking, stereo matching, IMU
preintegration, the nonlinear optimizer, marginalization, and loop closure
are all implemented in this repo — no OpenCV, no g2o/GTSAM/Ceres, no
pre-built SLAM/VO crates. Standard infrastructure crates (`nalgebra`,
`image`, `serde`, `csv`, `rayon`) are used for linear algebra, image
decoding, and parsing.

It targets the EuRoC `machine_hall` stereo-inertial dataset
(`MH_01_easy` .. `MH_05_difficult`), aiming for accuracy competitive with
published stereo-inertial SLAM systems (ORB-SLAM3, OKVIS, VINS-Fusion,
Kimera-VIO), and — as of Stage 2 — real-time processing (1 second of
sensor data processed in at most 1 second of wall-clock, **now
confirmed on full, un-truncated sequences too, Stage 4 M0/M1**, see
below), and — as of Stage 3 — trajectory visualization (`slam-render` +
`bin/slam-viz`, **done**). Current plan: [`plan/STAGE4.md`](plan/STAGE4.md)
— `bin/slam-run --full` is now real-time on every sequence; next is
confirming full-sequence accuracy doesn't regress before flipping the
default. [`plan/STAGE1.md`](plan/STAGE1.md),
[`plan/STAGE2.md`](plan/STAGE2.md), and
[`plan/STAGE3.md`](plan/STAGE3.md) are all done and worth reading for
that history.

## Status

Stage 1 milestones M0-M8 are done: stereo visual odometry, IMU
preintegration/initialization, a sliding-window backend that jointly
optimizes both, track-loss recovery verified across full, un-truncated
real sequences, loop closure with a measurable, real-data accuracy win,
and a global bundle-adjustment pass over the full trajectory. Stage 2's
M0 (evaluation + timing harness, finishing Stage 1's M9), M1 (real
sliding-window marginalization, closing `decisions/0007`), M5 (the
real-time bar itself — met via M1, ahead of schedule), and M6 (accuracy
closing pass, finishing Stage 1's M10 — its ad hoc-knob tuning space is
now exhausted, see below) are also done, meaning **all of Stage 2's
milestones are now landed or deliberately deferred**, both of Stage 2's
own goals met — see [`docs/RESULTS.md`](docs/RESULTS.md) for real,
reproducible accuracy and real-time-factor numbers. Stage 3 (trajectory
visualization — `slam-render`, a hand-written 3D rendering library, plus
`bin/slam-viz`, an app that shows a run's trajectory next to its video
frames and diagnostic graphs, synced, and lets users browse past runs)
is also done: M0-M7 all landed (M7's optional multi-run-comparison
stretch deferred, not required) — see below. **Stage 4** (full-sequence
real-time VIO — M0/M1 done, M2 in progress) exists because Stage 2's
own real-time-factor number was only ever measured on the 600-frame
bounded clip `bin/slam-run` defaults to, a gap `plan/STAGE2.md`'s own
Risks section predicted ("a truncated clip that happens to fit inside
the window can look real-time for reasons that have nothing to do with
actually fixing the scaling") and `docs/RESULTS.md` admits was never
closed ("not re-benchmarked with `--full` yet"). See
[`plan/STAGE1.md`](plan/STAGE1.md), [`plan/STAGE2.md`](plan/STAGE2.md),
[`plan/STAGE3.md`](plan/STAGE3.md), and [`plan/STAGE4.md`](plan/STAGE4.md)
for the full milestone lists and [`memory/progress/`](memory/progress/)
for a session-by-session log of what landed and when.

| Milestone | What it adds | Status |
|---|---|---|
| Stage 1 M0 | Workspace scaffold, EuRoC dataset I/O | Done |
| Stage 1 M1 | Camera model, stereo rectification, triangulation, PnP | Done |
| Stage 1 M2 | Image pyramid, FAST detector, Lucas-Kanade tracking | Done |
| Stage 1 M3 | Stereo matching + VO pipeline, first ATE checkpoint | Done |
| Stage 1 M4 | IMU preintegration + static/dynamic VI initialization | Done |
| Stage 1 M5 | Sliding-window VIO backend (fuses M3's VO with M4's IMU) | Done |
| Stage 1 M6 | Track-loss recovery, robustness, full-sequence runs | Done |
| Stage 1 M7 | Loop closure (BoW, geometric verification, pose graph) | Done |
| Stage 1 M8 | Global bundle adjustment over the full trajectory | Done |
| Stage 2 M0 | Evaluation + timing harness (finishes Stage 1 M9) | Done |
| Stage 2 M1 | Sliding-window marginalization (closes Stage 1 `decisions/0007`) | Done |
| Stage 2 M2-M4 | Analytic Jacobians, sparse solve, `rayon` parallelism | Deferred — not required, real-time bar already met (see M5) |
| Stage 2 M5 | Real-time validation (factor ≤ 1.0) | **Done — met via M1 alone** |
| Stage 2 M6 | Accuracy closing pass (finishes Stage 1 M10) | **Done — see below** |
| Stage 3 M0 | Non-clobbering per-run output history in `bin/slam-run` | Done |
| Stage 3 M1 | `slam-render` foundations (camera, GPU bootstrap, line/grid/axes primitives) | Done |
| Stage 3 M2 | Trajectory/pose-graph scene primitives (point/pose markers) | Done |
| Stage 3 M3 | `bin/slam-viz` app shell + 3D panel, run picker | Done |
| Stage 3 M4 | Video frame panel, synced to keyframe timestamps | Done |
| Stage 3 M5 | Graphs panel (per-keyframe ATE + timing bar chart) | Done |
| Stage 3 M6 | Synced playback (shared cursor across 3D/video/graphs) | Done |
| Stage 3 M7 | Run-browser polish (config values shown in the picker) | **Done — stretch (multi-run overlay) deferred** |
| Stage 4 M0 | Full-sequence baseline measurement (all 5 sequences) | Done |
| Stage 4 M1 | Bound `global_bundle_adjustment`'s scope (real-time fix) | **Done — whole-run wall-clock now ≤1.0x on all 5 sequences** |
| Stage 4 M2 | Root-cause full-sequence ATE regression (5.6x-25.6x vs. bounded clip) | Not started — confirmed real, see `docs/RESULTS.md` |
| Stage 4 M3 | Flip `bin/slam-run`'s default to full-sequence | Blocked on M2 — see `plan/STAGE4.md` |

As of M3, running `bin/slam-inspect` (below) on the five `MH_*` sequences
reports stereo-only (no IMU, no backend optimization, no loop closure) VO
with ATE RMSE in the 11-17cm range over ~130 real frames per sequence —
proof the frontend produces a geometrically sane trajectory, not yet the
SOTA VIO accuracy bar (2-9cm). As of M4, it also reports static
(stationary-window) and dynamic (moving-start) IMU initialization per
sequence: gyro bias and a gravity vector recovered from real IMU data,
magnitude typically within a couple m/s² of 9.81 (see
`memory/decisions/0005-...md` for why accelerometer bias isn't estimated
at this stage). As of M5, it also reports full stereo-inertial VIO (joint
reprojection + IMU optimization) on sequences with a stationary bootstrap
window: ATE currently ~matches, not yet clearly beats, the VO-only number
on the same clip — expected given the backend's window is still naive
fixed-lag (no marginalization) and uses ad hoc, not covariance-derived,
noise weights (`memory/decisions/0006-...md`, `0007-...md`); closing that
gap is explicitly M10's job, not a sign M5 is broken. As of M6, an
`#[ignore]`d (expensive, run manually) test runs every frame of every
`MH_*` sequence end-to-end (~14,000 frames total) with zero unrecoverable
tracking failures — full-sequence ATE is multiple meters (expected: pure
VO/VIO drift with no loop closure or global BA yet, not a regression from
the short-clip numbers above), but the pipeline never gets permanently
lost, recovering (fresh landmarks, or IMU-only propagation for the VIO
pipeline) whenever a frame is genuinely untrackable. As of M7, MH_05
(the sequence with a real loop — it revisits its own start position at
the very end, after ~98m of travel) shows a real, measurable loop-closure
win: BoW place recognition + geometric verification + pose-graph
optimization takes full-sequence ATE from ~5.6m down to ~3.3m. As of M8,
it also reports one global bundle-adjustment pass (reusing M5's own
solver, just over every keyframe ever created instead of the sliding
window) with before/after ATE on the same clip: on the short, loop-free
MH_01 clip shown by default this holds essentially flat (~0.104m ->
~0.104m) rather than clearly improving — expected, not a bug, since a
short window-only clip leaves little "unfinished optimization" for a
global pass to clean up (see `memory/progress/2026-07-21-m8-...md` for
why a longer sequence, or a post-loop-closure run, is where global BA's
real win should show up).

Stage 2's M0 added `bin/slam-run` (below) and found two things worth
knowing before trusting any of the numbers above too literally: (1) a
real determinism bug — `slam-optim`'s solver used to accumulate landmark
contributions via a `HashMap`, whose randomized-per-process iteration
order made re-running the *identical* pipeline on the *identical*
sequence produce different trajectories (three runs of the same 600-frame
MH_01 clip gave three different keyframe counts before the fix); see
`memory/decisions/0011` — fixed, and now bit-for-bit reproducible run to
run; (2) the per-frame VIO loop (tracking + windowed optimization) is
already close to Stage 2's real-time bar (factor 1.09-1.20 on two of
three runnable sequences) even before any of Stage 2's planned
speedups, while the global bundle-adjustment pass is wildly
disproportionate (tens of seconds for a 30-second clip) — a direct,
measured confirmation of why Stage 2 tackles marginalization before
anything else. Full numbers, methodology, and honest caveats (two
sequences don't run yet, pending initializer robustness work) in
[`docs/RESULTS.md`](docs/RESULTS.md).

Stage 2's M1 replaced the naive fixed-lag window (`decisions/0007`) with
real Schur-complement marginalization — the departing keyframe's IMU/
bias-random-walk connectivity and uniquely-observed landmarks fold into a
compact prior instead of being dropped. Getting this safe on real data
found three more real bugs (`memory/decisions/0012`-`0014`), the most
significant of which turned out to be a latent bug in `VioPipeline`
itself, unrelated to marginalization's own math: it never got the raw-PnP
pose-jump sanity check `VoPipeline` gained in M7 (`decisions/0009`) — a
gap that decision had explicitly predicted would eventually matter.
Naive-drop's "forget a bad keyframe the moment it's evicted" behavior had
been accidentally masking this for two milestones; marginalization's
whole point is to *retain* information, so it stopped masking it. Fixed
at the source (`VioPipeline` now filters implausible PnP poses exactly
like `VoPipeline` does) plus a second guard at the marginalization
boundary itself (defense in depth). Real checkpoint: marginalization's
own net effect on MH_01 is now within noise of a from-scratch (non-
marginalized) baseline under the same fix (0.169m/104 keyframes vs.
0.164m/109) — holding steady as `plan/STAGE2.md`'s M1 requires, though
this short clip doesn't show marginalization's "biggest accuracy lever"
framing dramatically (a longer or more information-starved run is where
that should show up, a good follow-up not required for this checkpoint).

**Stage 2's real-time goal (M5) turned out to already be met by M1
alone.** The plan originally expected M2 (analytic IMU Jacobians), M3
(sparse solve), and M4 (`rayon` parallelism) to be needed first — but the
PnP corruption fixed above had been triggering cascades of expensive
track-loss-recovery keyframes, and removing the cause removed most of
that wasted cost too. Real-time factor dropped from 1.198/0.357/1.086
(MH_01/04/05) to 0.543/0.398/0.523 — comfortably under the 1.0 bar on
every runnable sequence, roughly half the budget to spare. `plan/
STAGE2.md` now marks M2-M4 deferred rather than required, and M6
(finishing Stage 1's M10, accuracy tuning) is next.

Stage 2's M6 started by closing the gap in the paragraph above:
`MH_02_easy` and `MH_03_medium` weren't producing any numbers at all.
Measuring (not guessing) the actual best-achievable stationary window
per sequence found both were genuinely stationary but just barely past
the bootstrap threshold (0.093/0.090 rad/s against a 0.09 cutoff,
`decisions/0015`); loosening it to 0.10 fixed both. `docs/RESULTS.md`
now has all five `MH_*` sequences.

M6 then tried four more tuning directions, all measured on real data,
all reverted as net regressions: (1) real `sensor.yaml`-derived noise
weighting, replacing the ad hoc `SolverConfig` weights `decisions/0006`
flagged, at two scopes — both regressed ATE on most sequences, since
the simplified formula ignores bias-*uncertainty*'s contribution to
preintegration error, which only full nonlinear covariance propagation
would capture (`decisions/0016`); (2) a larger `window_size` (8 -> 12) —
regressed all five sequences; (3) the outlier-gating Huber threshold,
tried both tighter (1.5) and looser (5.0) than the default 3.0 — both
destabilize MH_05 specifically for only small, inconsistent gains
elsewhere; (4) a smaller `window_size` (6, 4) — 6 helps MH_04
substantially but regresses three of the other four sequences, 4 is
worse everywhere (`decisions/0017`). Every direction either regresses
accuracy outright or trades one sequence's accuracy for another's,
never improving all five at once — M6's ad hoc-knob tuning space is
exhausted for this pipeline's architecture; a further win needs the
same class of structural work as the deferred M2/M3 (analytic IMU
Jacobians, real preintegration covariance), not more scalar sweeps.
**M6 is accordingly considered done** — current real numbers, all
real-time factors comfortably under 1.0: MH_01 0.151m, MH_02 0.184m,
MH_03 0.511m, MH_04 1.174m, MH_05 0.455m — see `docs/RESULTS.md`. With
M0-M6 all landed (M2-M4 deferred by M1's finding), both of Stage 2's
goals — real-time VIO and finishing Stage 1 — are met.

**Stage 3's M0** made `bin/slam-run` write a non-clobbering history entry
per invocation (`runs/<sequence>/<run_id>/{trajectory.csv, meta.json}`,
`meta.json` carrying ATE/RPE/timing plus the exact config and git commit
used) instead of only overwriting the latest run — the prerequisite for
goal 3 (per-run browsing) that also means every future tuning sweep
leaves a real on-disk trace, not just a memory/commit-message summary.
**Stage 3's M1** added the new `slam-render` crate: a hand-written orbit
camera (mouse-drag orbit, scroll zoom, pan — `OrbitCamera` in `camera.rs`,
its view/projection matrices unit-tested against known camera poses,
no GPU needed), a `wgpu`/`winit` bootstrap (`GpuContext`), and line-
segment scene primitives (`add_line`/`add_polyline`/`add_grid`/
`add_axes`). Verified with genuine GPU-backed tests, not just math: this
repo's development machine gets a real headless Metal adapter with no
window needed, confirmed before writing any rendering code, so
`slam-render`'s own tests render a grid+axes scene to an off-screen
texture and read the actual pixels back to assert the gizmo appears
where the camera math says it should. `cargo run -p slam-render
--example orbit_demo` opens an interactive window (drag to orbit, scroll
to zoom) for the human-verification half of this milestone — building
and passing `cargo clippy` is confirmed, but actually looking at it is
for a human, not this repo's automated checks (see `plan/STAGE3.md`'s
"Verifying a GUI deliverable" section for why that split is the right
bar for this stage).

**Stage 3's M2** extended `slam-render` with `Scene::add_point_marker`
(a crosshair, since `LineList`-only rendering has no point-sprite
pipeline) and `Scene::add_pose_marker` (local axes + a small pyramid
wireframe at an `SE3` pose, via `slam-core::SE3::transform`). The
plan's original text implied a CSV-to-`Scene` "data adapter" living
inside `slam-render` itself, which turned out to be in tension with
the plan's own "`slam-render` depends on `slam-core` only" layout —
resolved by adding `slam_eval::read_trajectory_csv` (the exact inverse
of the existing writer) instead, so a consumer that already depends on
both crates (`bin/slam-viz`, M3) does the actual "load a real run and
build a Scene" wiring, one line, rather than coupling `slam-render` to
this pipeline's CSV format.

**Stage 3's M3** added `bin/slam-viz`: an `egui` run picker (listing
`runs/<sequence>/<run_id>/` history, click to load) next to a 3D panel.
Rather than sharing one `wgpu` device/render pass between `egui`'s own
rendering and `slam-render`'s custom pipeline (real but fiddly
engineering — `egui`'s callback API hands you an already-open render
pass, not a fresh encoder), the 3D panel renders into `slam-render`'s
already-tested `OffscreenTarget` and displays the read-back pixels as a
plain `egui` texture — one CPU pixel round-trip per frame, a real cost
but a non-issue for a post-hoc viewer not held to Stage 2's real-time
bar. A `--dump-scene-stats` flag gives a non-visual smoke check: run
against this session's own real `runs/` history it correctly discovered
all 7 real runs, sorted them, and loaded the most recent into a real
scene (818 vertices, sane bounding-box center/extent) — genuine
end-to-end verification against real pipeline output, not just
synthetic test fixtures. `cargo run --release --bin slam-viz` opens the
interactive app (drag-orbit/pan, scroll-zoom, click a run in the left
panel) for the same human-verification step M1's demo needs.

**Stage 3's M4** added `bin/slam-viz`'s video panel: a scrub slider over
the selected run's own keyframe timestamps, each position synced to the
matching `cam0` frame via a new `slam_dataset::EuRocSequence::
nearest_cam0_frame_index` (binary search — didn't exist before this
milestone, despite the plan's text assuming it did) and displayed as an
`egui` texture, with play/pause at a fixed ~10 keyframes/sec. Verified
against the real `MH_01_easy` dataset already in this repo, not just
synthetic fixtures: loading a real sequence and syncing to a real
frame's own timestamp resolves back to the exact expected index.

**Stage 3's M5** added `bin/slam-viz`'s graphs panel: an `egui_plot`
line chart of per-keyframe aligned ATE, and a bar chart of the run's
timing breakdown plus its real-time factor. Backed by a new
`slam_eval::compute_ate_series` — the same Umeyama-alignment machinery
`compute_ate` already used, refactored to expose the full per-point
error series instead of only summary stats; `compute_ate` itself now
calls it internally (zero duplicated logic), and its own pre-existing
tests still pass unchanged, confirming the refactor didn't alter
behavior. RPE-over-time was deliberately scoped out rather than
attempted alongside ATE — it would need the same "expose the series"
treatment applied to a different function, recorded as a legitimate
follow-up in `plan/STAGE3.md` rather than silently dropped.

**Stage 3's M6** synced all three panels to one shared cursor — the
video panel's own scrub position (already there since M4), now read by
the 3D panel (highlights the matching keyframe with a crosshair marker)
and the graphs panel (a vertical line on the ATE plot at that index).
This turned out to be mostly wiring, not new data-model work: the
video panel's timestamps, the 3D panel's trajectory points, and the
graphs panel's ATE series were already built from the same source, row
-for-row, so one shared index into all three *is* the sync mechanism,
not a separate thing to keep consistent. With M0-M6 done, Stage 3's
three goals are functionally met — a hand-written 3D rendering library,
an app showing the trajectory next to synced video and graphs, and
per-run browsing.

**Stage 3's M7** closed the one real gap left in "browse results per
run": `RunMeta`'s config values (`window_size`/`huber_delta` — the two
knobs `decisions/0017`'s tuning sweeps varied) were captured and tested
since M0 but never actually shown in the run picker, so comparing a
sweep's runs still meant reading `meta.json` by hand outside the app.
Extended the picker's label and `--dump-scene-stats`'s output to
include both — pure display formatting of already-tested data, no new
logic needed. The stretch goal (side-by-side/overlay comparison of two
runs' trajectories) is deferred, per the plan's own "not required"
framing. **With M0-M7 done, all three of Stage 3's goals are met** —
`slam-render` (a real, hand-written 3D rendering library, verified with
genuine GPU-backed pixel-readback tests, not just camera math),
`bin/slam-viz` (3D + video + graphs panels, synced to one cursor), and
per-run browsing (the run picker, now showing enough per-run detail to
compare tuning sweeps without leaving the app). Every milestone's
non-visual logic has real `cargo test` coverage, much of it verified
against this repo's actual dataset and this session's own real `runs/`
output, not synthetic fixtures alone — the interactive/visual half of
every milestone builds and is `cargo clippy`-clean throughout, but was
intentionally never run by the agent (launching a GUI window isn't
something these tools can drive or observe, and doing so unprompted
would pop up on the user's real desktop): `cargo run -p slam-render
--example orbit_demo` and `cargo run --release --bin slam-viz` are
there for the user's own visual confirmation.

**Stage 4's M0/M1**: `bin/slam-run --full`'s real-time performance had
never actually been measured — `plan/STAGE2.md`'s own Risks section
predicted this gap ("a truncated clip that happens to fit inside the
window can look real-time for reasons that have nothing to do with
actually fixing the scaling") and `docs/RESULTS.md` admitted it was
never closed. Measuring `MH_01_easy`'s full sequence live-profiled a
real bottleneck: `global_bundle_adjustment` still solved densely
(`O(n^3)`) over *every keyframe ever created* — Stage 2 M1's
marginalization only ever bounded the windowed solver, not this call
site — costing 957 seconds on that one sequence's 741 keyframes. Fixed
by capping global BA to the most recent 150 keyframes
(`VioParams::max_global_ba_keyframes`) instead of unbounded history —
no new linear algebra, just bounding what goes into the same,
already-tested solver. Global BA's cost dropped ~123x (957s -> 7.8s);
confirmed on all 5 sequences, whole-run wall-clock is now under the
data duration everywhere (0.49x-0.81x). Confirmed the fix didn't trade
accuracy for speed, by measuring `MH_01_easy`'s ATE both before and
after on the identical sequence: 3.869m -> 3.868m, unchanged — global
BA over the full history wasn't preventing drift anyway (this harness
doesn't chain in loop closure), so bounding its scope cost nothing.
That said, full-sequence ATE is genuinely 5.6x-25.6x worse than the
bounded-clip numbers above, on every sequence — a real, confirmed,
pre-existing gap (not caused by this fix), and Stage 4's next open
item (M2) before the default can flip to full-sequence. See
`docs/RESULTS.md`'s "Full-sequence results" section for the complete
before/after table.

## Building

Requires a Rust toolchain (install via [rustup](https://rustup.rs) if you
don't have one):

```
cargo build --release
```

## Running the test app

`bin/slam-inspect` is the running, human-readable record of what the
pipeline can currently do — it's extended alongside each milestone rather
than replaced by throwaway demos. It expects the EuRoC data under
`data/machine_hall/` (gitignored; not included in this repo).

```
cargo run --release --bin slam-inspect                        # all sequences under data/machine_hall
cargo run --release --bin slam-inspect -- data/machine_hall/MH_01_easy  # one sequence
```

For each sequence, it prints (and this is how to confirm the status table
above is real, not just claimed):

- calibration values parsed from `sensor.yaml` (cam0/cam1 intrinsics +
  distortion, IMU noise parameters)
- stereo rectification stats (baseline, rectified intrinsics) plus a
  synthetic triangulation round-trip check against the real calibration
- dataset load stats (frame/IMU counts, merged event-stream size)
- vision frontend stats: FAST keypoints detected and LK tracking survival
  rate across a handful of real frames
- stereo VO stats: landmarks initialized, frames successfully tracked, and
  ATE (Sim3-aligned against ground truth) over a real clip
- IMU initialization: a stationary-window static initializer (gyro bias +
  gravity) if the sequence has one, and the moving-start dynamic
  vision-IMU alignment initializer (gyro bias + gravity, reusing the VO
  keyframes above) always
- stereo-inertial VIO stats (sequences with a stationary bootstrap window
  only): landmarks, keyframes, and ATE for the full sliding-window
  backend — directly comparable to the stereo-VO-only ATE above — plus a
  one-shot global bundle-adjustment pass over every keyframe (M8), with
  before/after ATE
- loop closure (MH_05 only — the sequence with a real, documented loop):
  the detected/verified revisit and ATE with vs. without pose-graph
  optimization, run over the full sequence (takes ~40s in release, so
  this section alone dominates the tool's runtime)
- a raw ground-truth trajectory summary (span, bounding box) as a sanity
  check on units/frame

## Running the evaluation harness

`bin/slam-run` (Stage 2's M0, finishing Stage 1's M9) is the dedicated
accuracy/timing benchmarking tool — where `bin/slam-inspect` shows
per-milestone intermediate state, `bin/slam-run` runs the full pipeline
end to end and reports the numbers in [`docs/RESULTS.md`](docs/RESULTS.md).

```
cargo run --release --bin slam-run                       # all sequences, bounded default (~30s of data each)
cargo run --release --bin slam-run -- --full              # full, un-truncated sequences (slow — see docs/RESULTS.md)
cargo run --release --bin slam-run -- data/machine_hall/MH_01_easy  # one sequence
```

Writes `runs/<sequence>/trajectory.csv` (per-timestamp estimated vs.
groundtruth position, for external plotting) and `runs/summary.csv` (the
aggregate ATE/RPE/timing table) — both gitignored, regenerate them
locally rather than trusting stale copies.

## Testing

```
cargo test --workspace
cargo clippy --all-targets
```

Every crate's own `src/*.rs` files carry unit tests (including
finite-difference checks for anything Jacobian-shaped, and round-trips
against the real EuRoC calibration/data, not just synthetic-only cases).
`crates/slam-frontend`'s integration test runs the full VO pipeline over
real frames end-to-end, so `cargo test` takes on the order of tens of
seconds, not milliseconds — that's expected.

## Repository layout

```
crates/           # slam-core, slam-dataset, slam-vision, slam-geometry,
                   # slam-imu, slam-optim, slam-frontend, slam-backend,
                   # slam-loopclosure, slam-eval — see plan/STAGE1.md for
                   # what each is responsible for and in which milestone
bin/slam-inspect/  # per-milestone intermediate-state test app
bin/slam-run/      # accuracy/timing evaluation harness (Stage 2 M0)
data/              # EuRoC dataset (gitignored, not in this repo)
docs/RESULTS.md    # accuracy + real-time-factor numbers vs. published SOTA
runs/              # bin/slam-run's output (gitignored, regenerate locally)
plan/STAGE1.md     # original 11-milestone plan (mostly done)
plan/STAGE2.md     # current plan: real-time VIO + finishing Stage 1
memory/            # cross-session project memory (progress log, design
                   # decisions, gotchas) — see memory/README.md
```

## For contributors (human or AI)

- [`CLAUDE.md`](CLAUDE.md) — working protocol: verification requirements
  (tests + the test app), the project-memory system, and the git workflow.
- [`memory/README.md`](memory/README.md) — how the memory system is
  organized and why.
