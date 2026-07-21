# Stage 1: Stereo-Inertial SLAM on EuRoC `machine_hall`

## Goal

Build a Rust stereo-inertial visual SLAM pipeline, from scratch (own frontend,
backend, optimizer, loop closure — no OpenCV/g2o/ceres/existing SLAM crates),
that consumes the EuRoC `machine_hall` sequences already present in
`data/machine_hall/` and reaches accuracy competitive with published
stereo-inertial state of the art (ORB-SLAM3, OKVIS, VINS-Fusion, Kimera-VIO):
roughly **2-9 cm ATE RMSE** depending on sequence difficulty. Success is
measured against ground truth, not against a fixed absolute number — the bar
is "in the same ballpark as the published SOTA systems on the same sequence,"
re-checked against the primary sources once we have real results.

Dependency policy: standard Rust infrastructure crates are fair game
(`nalgebra` for linear algebra, `image`/`png` for decoding, `serde`/`csv` for
I/O, `rayon` for parallelism). Everything that is actually "the SLAM
algorithm" — feature detection & matching, tracking, IMU preintegration, the
nonlinear optimizer, marginalization, loop closure, pose-graph optimization —
is implemented by us.

Out of scope for Stage 1 (candidates for a later stage): dense/mesh
reconstruction, multi-session/map-merging, semantic mapping, real-time
performance targets (correctness and accuracy first, speed later),
non-`machine_hall` EuRoC rooms (`vicon_room`), other datasets (TUM-VI, KITTI).

## Dataset (already on disk, confirmed)

`data/machine_hall/{MH_01_easy .. MH_05_difficult}/mav0/`:

- `cam0/`, `cam1/`: global-shutter stereo pair, 752x480 @ 20 Hz, PNG frames
  named by nanosecond timestamp, indexed by `data.csv`. `sensor.yaml` per
  camera gives `T_BS` (extrinsics to body frame), pinhole intrinsics
  `[fu, fv, cu, cv]`, and radial-tangential distortion (4 coeffs, i.e. no k3).
- `imu0/`: ADIS16448 @ 200 Hz, `data.csv` = `[t, wx, wy, wz, ax, ay, az]`.
  `sensor.yaml` gives noise density / random walk for gyro and accel and
  `T_BS` (identity — IMU defines the body frame).
- `leica0/`: Leica MS50 total-station position fixes (sparse, prism offset in
  `sensor.yaml`) — only present in some sequences, not our primary ground
  truth source.
- `state_groundtruth_estimate0/`: Vicon/Leica-fused ground truth
  `[t, p_RS_R (xyz), q_RS (wxyz), v_RS_R, b_w, b_a]` at high rate — this is
  what we evaluate ATE/RPE against.
- `body.yaml`: cosmetic (MAV name only).

Known gotchas to design around: camera and IMU timestamps are not
synchronized to the same epoch as groundtruth in a trivial 1:1 way (need
nearest/interpolated lookup by timestamp), MH_01/02/03 start with the MAV
stationary (usable for static IMU bias/gravity init), MH_04/05 start in
motion (need a robust dynamic initializer as fallback), and groundtruth is in
a Vicon/Leica world frame unrelated to the SLAM world frame, so evaluation
needs a similarity (Sim3) alignment step, not a direct comparison.

## Workspace layout

```
auto-slam/
  Cargo.toml                 # workspace
  crates/
    slam-core/               # SO3/SE3/Sim3, quaternions, common point/pose types
    slam-dataset/            # EuRoC reader: yaml calib, csv streams, PNG loading, time sync
    slam-vision/             # image pyramids, FAST/ORB-style detector, descriptors, LK optical flow
    slam-geometry/           # pinhole+radtan model, rectification, triangulation, PnP, epipolar/essential
    slam-imu/                # IMU preintegration (on-manifold), bias jacobians, noise propagation
    slam-optim/              # Gauss-Newton/LM, sparse Schur-complement solver, robust kernels, autodiff-free analytic Jacobians
    slam-frontend/           # tracking, keyframe selection, static/dynamic initializer
    slam-backend/            # sliding-window VIO optimization, marginalization (Schur complement priors)
    slam-loopclosure/        # place recognition (own bag-of-visual-words), Sim3/SE3 pose-graph optimization
    slam-eval/                # trajectory alignment (Umeyama), ATE/RPE, CSV/plot export
  bin/
    slam-run/                # CLI: run a sequence end-to-end, dump trajectory + stats
  data/machine_hall/...       # already present
  plan/STAGE1.md              # this file
```

Crate boundaries follow the natural dependency order (`core` → `dataset` /
`vision` / `geometry` → `imu` → `optim` → `frontend`/`backend` →
`loopclosure` → `eval`), so each milestone below both adds a capability and
becomes independently testable.

## Milestones

Each milestone should land as a working, tested increment — do not let this
turn into one big-bang integration at the end.

### M0 — Workspace scaffold + dataset I/O
- Cargo workspace, crate skeletons, CI-able `cargo test`/`cargo clippy`.
- `slam-dataset`: parse `sensor.yaml` (T_BS, intrinsics, distortion, noise
  params), parse all `data.csv` streams, decode PNGs on demand (lazy, not
  all-in-memory), produce a single time-ordered event iterator merging
  cam0/cam1/imu0 timestamps.
- `slam-eval`: parse `state_groundtruth_estimate0/data.csv`, provide
  timestamp-interpolated pose lookup.
- Test: load MH_01_easy, assert frame counts, IMU rate, calibration values
  match the yaml, plot/print a raw groundtruth trajectory to sanity check
  units/frame.

### M1 — Geometry & camera model
- `slam-geometry`: pinhole projection/unprojection, radial-tangential
  distortion + inverse (iterative undistort), stereo rectification (compute
  rectifying rotations from the two `T_BS` extrinsics, rectified intrinsics,
  baseline), linear + reprojection-refined triangulation, P3P/EPnP for
  pose-from-points, 8-point/5-point + RANSAC for relative pose (used by the
  initializer).
- Test: synthetic 3D points projected through the real MH_01 calibration,
  round-tripped through triangulation/PnP, check sub-pixel/mm-level
  recovery; verify rectified stereo pair on a real frame has near-zero
  vertical disparity on matched points.

### M2 — Vision frontend primitives
- `slam-vision`: image pyramid, a corner detector (FAST or Harris) with
  grid-based non-max suppression for even distribution, a binary descriptor
  (BRIEF-style) or patch-based Lucas-Kanade optical flow tracker (pick one
  primary tracking strategy — recommend LK for temporal tracking + a
  descriptor only for loop closure/relocalization, since that's what gives
  ORB-SLAM3/VINS-class systems their robustness split).
- Test: track a known feature across a handful of consecutive real MH_01
  frames, verify track survives and reprojects consistently; benchmark
  detector distribution (no clustering in one image region).

### M3 — Stereo visual frontend + static map bootstrap
- Detect+track features per left frame, stereo-match against right frame
  using rectified epipolar constraint, triangulate to get an initial local
  map of 3D landmarks with depth (stereo gives scale for free — no
  monocular-SFM ambiguity here).
- Keyframe selection heuristic (parallax/track-count/time-based).
- Test: run stereo VO-only (no IMU yet) on MH_01_easy, compute ATE against
  groundtruth after Sim3 alignment — this is the first end-to-end accuracy
  checkpoint even before IMU fusion is wired in.

### M4 — IMU preintegration & VI initialization
- `slam-imu`: on-manifold IMU preintegration between keyframes (position,
  velocity, rotation deltas + bias Jacobians), per EuRoC noise/bias-walk
  params from `sensor.yaml`.
- Initializer: static case (MH_01-03 start stationary) — estimate gravity
  direction and gyro bias from the first stationary window directly.
  Dynamic case (MH_04/05 start in motion) — implement the standard
  vision-IMU alignment (linear system solving for gravity, scale
  confirmation since stereo already fixes scale, accel bias, and initial
  velocities from a short window of stereo keyframes + preintegrated IMU).
- Test: on a stationary clip, recovered gravity vector magnitude ≈ 9.81 and
  gyro bias in the sensor's spec'd range; on MH_04's moving start, initializer
  converges within the first few seconds.

### M5 — Sliding-window visual-inertial backend
- `slam-optim`: Levenberg-Marquardt with analytic Jacobians (SE3/SO3
  local-parameterization tangent-space updates), sparse structure exploiting
  the reprojection + IMU-factor sparsity via Schur complement (marginalize
  landmarks per iteration like classic BA solvers do), Huber/Cauchy robust
  kernel on reprojection residuals.
- `slam-backend`: sliding window of N keyframes, reprojection factors
  (stereo), IMU preintegration factors between consecutive keyframes, bias
  random-walk factors; marginalize oldest keyframe into a prior (Schur
  complement) instead of dropping it, to retain information (this is the
  single biggest accuracy lever versus a naive fixed-lag window).
- Test: on MH_01/02/03 (easier, mostly static-friendly), full VIO pipeline
  ATE should already approach single-digit cm; compare against VO-only
  numbers from M3 to confirm IMU fusion is actually helping, not hurting.

### M6 — Robust tracking & map maintenance
- Outlier rejection (reprojection-error gating + RANSAC at initialization,
  chi-square gating in the optimizer), track loss recovery (short-term
  relocalization via last keyframe's landmarks), landmark culling
  (low-parallax / high-error point removal), keyframe culling (redundant
  keyframe removal to bound window/map growth).
- Test: full run on all five MH sequences end-to-end without crashing/losing
  tracking permanently; record per-sequence ATE/RPE.

### M7 — Loop closure
- `slam-loopclosure`: own bag-of-visual-words vocabulary built from the
  descriptors already extracted in M2/M3 (train offline on `machine_hall`
  frames), place-recognition query against keyframe database, geometric
  verification (PnP/RANSAC + Sim3/SE3 estimation) on a candidate match,
  pose-graph optimization (SE3, since stereo fixes scale — no Sim3 drift
  correction needed the way monocular ORB-SLAM requires) to distribute the
  loop-closure correction across the trajectory.
- Test: MH_05 (has a loop) shows measurable ATE improvement with loop
  closure enabled vs. disabled.

### M8 — Global bundle adjustment pass
- After loop closure, run a full (or windowed-but-large) BA over the
  corrected pose graph + landmarks to squeeze out residual drift — this is
  the step that gets systems from "good VIO" to "SOTA SLAM" numbers.
- Test: global BA strictly improves or holds ATE relative to pre-BA, on
  every sequence — regressions here mean a solver/Jacobian bug, not "BA
  doesn't help."

### M9 — Evaluation harness & benchmarking
- `slam-eval`: Umeyama Sim3/SE3 alignment against
  `state_groundtruth_estimate0`, ATE (RMSE/mean/median/std), RPE at multiple
  deltas, per-sequence and aggregate reports, CSV export of estimated vs.
  ground-truth trajectory for external plotting.
- `bin/slam-run`: one command runs a full sequence and prints the report;
  a wrapper script runs all five `MH_*` sequences and produces a summary
  table.
- Deliverable: a results table (this repo's numbers vs. the published
  ORB-SLAM3/OKVIS/VINS-Fusion/Kimera numbers on the same sequences, cited
  from their papers) checked in alongside the code, not just in chat.

### M10 — Accuracy closing pass
- Once end-to-end numbers exist, this is a targeted debugging/tuning
  milestone, not new features: revisit noise weighting (make sure
  information matrices actually reflect the `sensor.yaml` noise densities
  instead of ad hoc weights), initializer robustness on the harder
  sequences (MH_04/05), outlier-gating thresholds, keyframe/window sizing.
  Only reopen earlier milestones' code if profiling/error analysis points
  there.

## Cross-cutting infrastructure (mostly `slam-core` / `slam-optim`)

- Lie groups: SO3/SE3 (exp/log maps, adjoint, right/left Jacobians) as the
  backbone for both the optimizer's local parameterization and IMU
  preintegration on-manifold updates.
- One general sparse nonlinear least-squares solver shared by the VIO
  backend, loop-closure pose graph, and global BA — same machinery, three
  different factor graphs, rather than three bespoke solvers.
- Deterministic, reproducible runs (fixed RANSAC seeds) so accuracy
  regressions are attributable to code changes, not run-to-run noise.

## Suggested crates (infrastructure only, not SLAM logic)

`nalgebra` (linear algebra + geometry types), `png`/`image` (decode
`cam0`/`cam1` frames), `serde`/`serde_yaml`/`csv` (parse `sensor.yaml` /
`data.csv`), `rayon` (parallel feature detection/matching across the stereo
pair and across keyframes), `anyhow`/`thiserror` (error handling), `plotters`
or CSV-only output (trajectory export) — no OpenCV, no g2o/GTSAM/Ceres
bindings, no pre-built SLAM/VO crates.

## Risks

- **Optimizer correctness bugs are silent** — a wrong Jacobian sign still
  "converges," just to a worse optimum. Mitigate with numerical Jacobian
  checks (finite-difference vs analytic) as a standing unit test, not a
  one-off.
- **Dynamic initialization (MH_04/05) is the hardest single component** —
  budget real time for it in M4; a bad initializer poisons every later
  milestone's numbers and looks like a backend bug.
- **Timestamp/frame alignment bugs masquerade as accuracy bugs** — verify
  M0's time-sync logic thoroughly before trusting any downstream ATE number.
- **"State of the art" is a moving, sequence-dependent target** — treat the
  cited published numbers as approximate reference points to re-verify from
  the original papers, not exact pass/fail thresholds.
