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
sensor data processed in at most 1 second of wall-clock, **now met**, see
below). Current plan: [`plan/STAGE2.md`](plan/STAGE2.md);
[`plan/STAGE1.md`](plan/STAGE1.md) is the original 11-milestone plan
(mostly done) that Stage 2 finishes and builds on.

## Status

Stage 1 milestones M0-M8 are done: stereo visual odometry, IMU
preintegration/initialization, a sliding-window backend that jointly
optimizes both, track-loss recovery verified across full, un-truncated
real sequences, loop closure with a measurable, real-data accuracy win,
and a global bundle-adjustment pass over the full trajectory. Stage 2's
M0 (evaluation + timing harness, finishing Stage 1's M9), M1 (real
sliding-window marginalization, closing `decisions/0007`), and M5 (the
real-time bar itself — met via M1, ahead of schedule) are also done — see
[`docs/RESULTS.md`](docs/RESULTS.md) for real, reproducible accuracy and
real-time-factor numbers. See [`plan/STAGE1.md`](plan/STAGE1.md) and
[`plan/STAGE2.md`](plan/STAGE2.md) for the full milestone lists and
[`memory/progress/`](memory/progress/) for a session-by-session log of
what landed and when.

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
| Stage 2 M6 | Accuracy closing pass (finishes Stage 1 M10) | Not started |

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
