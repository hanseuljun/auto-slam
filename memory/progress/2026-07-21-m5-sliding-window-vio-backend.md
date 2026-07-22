# M5 — sliding-window visual-inertial backend

Landed the sixth milestone from `plan/STAGE1.md`, following M0-M4. This is
the milestone that actually fuses M3's stereo VO with M4's IMU
preintegration into one joint estimator — the biggest single piece of
Stage 1 so far.

## What's done

### `slam-optim`: LM solver with Schur-complement landmark elimination
- `KeyframeState` (15 DoF: pose + velocity + gyro bias + accel bias) with
  SE3 left-multiplicative retraction.
- Reprojection factor: analytic Jacobian, structurally identical to
  `slam_geometry::refine_pose_gauss_newton`'s (reused derivation).
- IMU preintegration factor: 9-DoF residual, **numerical** (central-
  difference) Jacobians rather than hand-derived analytic ones —
  deliberate risk/complexity tradeoff, see `decisions/0006`.
- Bias random-walk factor: trivial exact analytic Jacobian.
- Huber robust kernel on reprojection residuals.
- LM solver: builds the reduced normal equations (landmarks Schur-
  eliminated each iteration, exploiting that the landmark-landmark
  Hessian block is exactly block-diagonal — no two landmarks ever share a
  factor), damped with accept/reject, gauge-fixes keyframe 0 as the
  window's anchor.
- Validated end-to-end on a noise-free synthetic toy problem (4 keyframes,
  20 landmarks, all three factor types, perturbed initial guess) —
  converged to ground truth on the *first* attempt, unlike M4's dynamic
  initializer which needed real debugging.

### `slam-backend`: the sliding-window VIO pipeline
- `VioPipeline`: LK-tracks stereo-matched landmarks (reusing
  `slam_frontend`'s stereo matching) frame-to-frame; every
  `keyframe_stride` frames, promotes to a keyframe — preintegrates the
  buffered raw IMU into an `ImuFactorSpec`, adds reprojection factors
  (existing tracked landmarks get cam0-only factors; newly-triangulated
  landmarks get both cam0+cam1, anchoring the stereo/metric constraint at
  creation), and runs `slam_optim::optimize` over the current window.
- The window is **naive fixed-lag** (oldest keyframe dropped outright, no
  marginalization prior) — see `decisions/0007` for why, and what real
  marginalization would build on top of this.

### Real bugs found and fixed this session (both caught before shipping)
1. **Dangling IMU edge after window slide.** Each keyframe stores the IMU
   factor connecting it to the *previous* keyframe. After popping the
   oldest keyframe, the new `window[0]` still had an edge referencing a
   keyframe no longer in the window — `kf_idx - 1` underflowed
   (`usize::MAX`), immediate panic. Fixed by skipping `window[0]`'s edge
   when rebuilding factors (its neighbor is gone; naive fixed-lag means
   that information is simply lost, matching `decisions/0007`).
2. **Only the newest IMU factor was ever added to the optimizer** — an
   earlier version of `run_optimization` took a single `latest_imu_factor`
   parameter instead of iterating every keyframe's stored edge, meaning
   older-but-still-in-window consecutive pairs contributed no IMU
   constraint at all. Fixed by storing `imu_edge: Option<(Preintegration,
   f64)>` per keyframe and iterating the whole window when rebuilding the
   problem, not just the latest addition.

## Real-data checkpoint

`vio_ate_on_mh01_is_competitive_with_vo_only`
(`crates/slam-backend/src/tests_integration.rs`): stereo-inertial VIO over
~80 real MH_01_easy frames, ATE ~0.11-0.14m (varies slightly release vs.
debug — normal for an iterative solver with floating-point order-
dependence, not a correctness concern) across a handful of keyframes.
`slam-inspect`'s new "stereo-inertial VIO" section shows the same pattern
on MH_01 and MH_04 (both have a stationary bootstrap window); MH_02/MH_03
skip gracefully (no stationary window for the static-init bootstrap this
demo path uses — the dynamic initializer path, shown separately, still
works there). MH_05's clip happens to be near-static throughout (its
stationary window starts at t=0), making its ATE number degenerate
(alignment of an almost-single-point trajectory segment is not
meaningful) — not a bug, just an uninformative test case; noted here so a
future session doesn't chase a phantom "0.000m is suspicious" issue
without first checking whether the underlying motion was actually that
small.

**This roughly matches, not clearly beats, M3's VO-only ATE** on the same
sequence. That's an honest, expected result given `decisions/0005`
(no real accel bias estimation yet), `decisions/0006` (ad hoc, not
covariance-derived, noise weights), and `decisions/0007` (no
marginalization) are all still open — any of them is a plausible reason
IMU fusion isn't yet the clear win it should eventually be. Not treated as
a red flag; treated as the known-remaining-work list for M10.

## Not done yet (correctly out of scope for this M5 pass)

- Marginalization (`decisions/0007`).
- Covariance-propagated (not ad hoc) information weights (`decisions/0006`
  and `SolverConfig`'s own doc comment).
- Real accelerometer bias estimation (`decisions/0005`, still open).
- Outlier rejection beyond reprojection's Huber kernel; track-loss
  recovery; keyframe culling — M6.
