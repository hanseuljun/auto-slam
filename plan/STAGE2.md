# Stage 2: Real-time VIO + finishing Stage 1

## Goal

Two goals for this stage, in the order they're tackled below (the second
depends on the first landing, and the third depends on both):

1. **Real-time VIO.** Processing N seconds of a sequence's `cam0`/`imu0`
   data through the continuous tracking + sliding-window backend loop
   (`slam-frontend`/`slam-backend`, Stage 1's M5/M6) should take at most N
   seconds of wall-clock time on the machine this repo is developed on —
   a real-time factor ≤ 1.0. This is a hard functional requirement, not a
   nice-to-have: Stage 1 explicitly deferred it ("real-time performance
   targets... speed later"), and this stage is "later."
2. **Finish `plan/STAGE1.md`.** Two of its eleven milestones never landed:
   M9 (evaluation harness & benchmarking) and M10 (accuracy closing pass).
   M9 in particular was attempted once already this session and rolled
   back — see "What we already know" below for why, so this stage doesn't
   repeat that mistake blind.

Everything else about Stage 1 still applies unless stated otherwise here:
same dependency policy (infra crates fine, SLAM algorithms hand-written),
same dataset, same "out of scope" list, same cross-cutting infrastructure
(one shared `slam-optim` solver, Lie-group backbone, deterministic runs).

## What we already know (don't rediscover this the hard way)

The first attempt at Stage 1's M9 this session ran the current pipeline
(M5's sliding-window VIO + M8's global bundle adjustment, no marginalization)
over one full, un-truncated sequence (`MH_01_easy`, ~184s of data) and did
not finish in under ~30 minutes wall-clock — multiple orders of magnitude
from real-time, and slow enough that it was rolled back rather than
finished blind. Reading the code (not guessing) turned up a concrete,
verifiable root cause:

`crates/slam-optim/src/solver.rs`'s `build_normal_equations` allocates a
**dense** `DMatrix<f64>` normal-equations system sized
`(problem.keyframes.len() - 1) * STATE_DIM` (`STATE_DIM = 15`), and
`optimize`'s inner loop solves it with a dense LU decomposition
(`damped.lu().solve(...)`) on *every* Levenberg-Marquardt iteration. This
is fine for `slam-backend`'s windowed optimization (`problem.keyframes.len()`
is bounded by `VioParams::window_size`, small, ~8) — but M8's
`global_bundle_adjustment` builds its `Problem` from `history` (every
keyframe ever created, per `decisions/0007`: M5's window drops the oldest
keyframe outright instead of marginalizing it into a prior, so nothing
ever leaves `history`) plus the current window. For a full sequence that's
hundreds of keyframes, so the dense system's dimension — and the O(dim³)
cost of solving it, every iteration — grows with the *entire trajectory
length*. This is the concrete, measured (not assumed) reason M8's global
BA doesn't scale, and it's the first thing M1 below addresses.

Two other known, already-documented gaps compound this:
`decisions/0006` (IMU factor Jacobians are finite-difference/numerical,
not analytic — extra residual evaluations every iteration) and the fact
that `rayon` has been an allowed dependency since Stage 1's own suggested-
crates list but is not used anywhere in the codebase yet (vision frontend
work — FAST detection, LK tracking, stereo matching — is entirely serial).

## Milestones

Same discipline as Stage 1: each milestone lands as a working, tested
increment, verified via both `cargo test` and `bin/slam-inspect`/
`bin/slam-run`'s real output — no big-bang integration at the end, and no
milestone closes on an assumed number instead of a measured one.

### M0 — Evaluation + timing harness (finishes Stage 1's M9, extended) — Done

- `slam-eval`: Umeyama Sim3/SE3 alignment, ATE (RMSE/mean/median/std), RPE
  at multiple deltas, per-sequence and aggregate CSV reports — as
  originally scoped in `plan/STAGE1.md`'s M9.
- New for this stage: per-stage wall-clock timing (vision frontend,
  windowed backend optimization, global BA, loop closure), reported
  alongside accuracy, on every sequence run — this is the harness the
  real-time bar (goal 1) is measured against, so it has to exist before
  any optimization work below can claim to have helped.
- `bin/slam-run`: one command runs a sequence and prints both the
  accuracy report and the timing breakdown. Given "What we already know"
  above, default to a bounded/truncated run (like `bin/slam-inspect`'s own
  pattern) with an explicit flag for a full-sequence run — a harness that
  takes 30+ minutes to produce one number isn't usable for iterating on
  the rest of this stage.
- Deliverable: `docs/RESULTS.md` with an accuracy table (this repo vs.
  published ORB-SLAM3/OKVIS/VINS-Fusion/Kimera numbers, cited from their
  papers) and a timing table (wall-clock seconds per second of sensor
  data, per sequence, per stage) — the baseline every later milestone in
  this stage is measured against.

### M1 — Sliding-window marginalization (closes `decisions/0007`) — Done

- Schur-complement the oldest keyframe into a compact prior factor when
  it slides out of the window, instead of dropping it (M5's current naive
  fixed-lag) or retaining it forever unbounded (M8's `history`, the direct
  cause of the scaling problem above).
- With marginalization, older trajectory information lives in compact
  per-keyframe priors instead of literal retained keyframes — a "global"
  pass only needs the active window plus whatever loop closure touches,
  not the entire history, which removes the O(sequence length) growth in
  `global_bundle_adjustment`'s problem size at the root.
- Test: ATE does not regress vs. M0's naive-fixed-lag baseline on any
  sequence (`decisions/0007` frames marginalization as "the single
  biggest accuracy lever" — confirm that, don't just confirm "still
  runs"), and M0's timing harness shows global BA's cost no longer
  scales with full sequence length.
- **Result**: three real bugs found and fixed getting this safe on real
  data (`decisions/0012`-`0014`) — most significantly, `VioPipeline` had
  never gotten `decisions/0009`'s PnP pose-jump guard, a gap that
  decision had explicitly predicted. Fixing it (plus the marginalization-
  boundary guard `decisions/0013` adds) didn't just fix accuracy — it
  also collapsed the real-time factor (see M5 below), since the
  eliminated corruption had been triggering cascades of expensive
  track-loss-recovery keyframes.

**Unplanned finding: M1 alone met M5's real-time bar.** Fixing the root-
cause corruption above didn't just improve accuracy — spurious PnP
failures had been triggering cascading track-loss recoveries, each one
costing a full extra round of stereo matching/landmark detection.
Removing the cause removed most of that cost too. Re-running M0's
harness after M1 (see M5 below) showed the real-time bar already met on
every runnable sequence, comfortably, without M2-M4. Those three
milestones are accordingly **re-scoped from required to optional/
deferred** — see their entries below — and M6 (finishing Stage 1's M10)
becomes the priority. This is exactly the kind of mid-plan finding
`CLAUDE.md` asks to fold back into the plan document itself, not just
note in passing.

### M2 — Analytic IMU-factor Jacobians (closes `decisions/0006`) — Deferred, not required

M5's real-time bar is already met without this (see M1's finding above).
Doing it anyway would trade a real, if slow, correctness story (numerical
Jacobians, deliberately chosen in `decisions/0006` over a genuinely
error-prone 18-block hand derivation) for a speed benefit that's no
longer needed. Left here as a legitimate future optimization if profiling
ever shows the numerical Jacobian mattering again (e.g. after M3/M4, or
on a slower machine), not deleted from the plan.

Original scope, kept for reference if reopened: replace
`imu_residual_jacobian`'s finite-difference Jacobian with a hand-derived
analytic one (the standard Forster et al. preintegration Jacobian
blocks); test via finite-difference-vs-analytic agreement using the
existing numerical implementation as the oracle, then delete the
numerical path.

### M3 — Sparse-aware normal-equations solve — Deferred, not required

M5's real-time bar is already met without this. `solver.rs`'s dense
`DMatrix`/LU solve is fine at M1's now-bounded problem sizes (window ~8
keyframes, a compact prior). Original scope, kept for reference: exploit
the actual sparsity pattern (keyframes connect only to temporally-
adjacent keyframes and the landmarks they co-observe) instead of a dense
solve, if a future, larger-scale use case (e.g. much bigger windows)
ever needs it.

### M4 — Parallel vision frontend — Deferred, not required

M5's real-time bar is already met without this. Original scope, kept for
reference: parallelize per-frame data-parallel work (LK tracking, FAST
detection, stereo matching) with `rayon` (allowed since Stage 1, unused
so far) if a future profiling pass finds vision cost dominant again —
would need care to preserve Stage 1's determinism requirement (fixed
reduction order, not reliant on thread scheduling for correctness).

### M5 — Real-time validation against the 1-minute bar — Done

- Re-run M0's harness on the runnable sequences. Report real-time factor
  (wall-clock seconds spent / seconds of sensor data processed) for the
  continuous VIO loop specifically — frontend tracking plus windowed
  backend optimization, the part that would need to keep up with a live
  sensor feed.
- Bar: real-time factor ≤ 1.0 on every sequence run.
- Scope note: this bar applies to the per-frame VIO loop only. Loop
  closure (Stage 1 M7) and global BA (Stage 1 M8, now bounded by M1) are
  occasional, one-shot batch passes by design — not held to the same
  per-frame real-time bar.
- **Result**: met on every runnable sequence after M1 alone (MH_01
  0.543, MH_04 0.398, MH_05 0.523 — see `docs/RESULTS.md`), roughly half
  the available budget to spare, without needing M2-M4. See M1's
  "unplanned finding" above for why.

### M6 — Finish Stage 1's M10: accuracy closing pass — In progress

- Now that M1-M5 make iteration fast enough to actually iterate on: real
  `sensor.yaml`-derived noise weighting (replacing the ad hoc weights
  `decisions/0006` flagged), initializer robustness on MH_04/05,
  outlier-gating threshold tuning, keyframe/window sizing — re-running
  the full accuracy+timing harness after each change to confirm it
  actually helps, not just "still runs."
- Test: updated `docs/RESULTS.md` accuracy table showing measurable
  improvement over M0's baseline on every sequence tuned, with the
  real-time bar from M5 still holding (a tuning pass that regresses speed
  back out of real-time doesn't count as done).
- **Sub-step done: initializer bootstrap fix.** `MH_02_easy`/
  `MH_03_medium` weren't producing any numbers at all (not just
  "inaccurate" — skipped entirely). Measured the actual per-sequence
  stationary-window quality and found the bootstrap threshold was
  genuinely too tight for both; loosened it with real margin above every
  sequence's measured value (`decisions/0015`). All five sequences now
  have real numbers in `docs/RESULTS.md`.
- **Sub-step tried and reverted: sensor.yaml-derived noise weighting.**
  Built and measured at two scopes (full derivation of all IMU/
  reprojection/bias weights, then a narrower version keeping IMU pose/
  velocity weights at their tuned values) — both regressed real ATE on
  most sequences. The simplified "integrated white noise" formula ignores
  bias-uncertainty coupling that only full nonlinear preintegration
  covariance propagation would capture; the ad hoc weights, hand-tuned
  against real data, outperform it. Reverted; `solver_config_from_sensor_
  noise` exists and is tested but isn't wired into the default pipeline.
  Full writeup: `decisions/0016`. Doing this properly is now understood
  to need the same class of work as the deferred M2 (real preintegration
  covariance, not just noise densities plugged into isolated formulas) —
  a real, separate, larger undertaking, not a quick sub-step.
- **Sub-step tried and reverted: larger `window_size` (8 -> 12).**
  Unambiguous — regressed ATE on all five sequences (MH_03 doubled,
  MH_05 nearly doubled) and raised the real-time factor on 4 of 5.
  Reverted; `VioParams::default()`'s `window_size: 8` unchanged.
- Remaining open: outlier-gating threshold tuning. A *smaller* window
  might be worth trying before a larger one, if window sizing gets
  revisited (bigger clearly isn't better at this scale with the current
  ad hoc weights). Initializer robustness specifically for MH_04/MH_05 is
  lower priority than the MH_02/03 fix was — both already produce real
  numbers via the dynamic (not static) initializer, so there's no
  equivalent "produces nothing at all" gap to close there.

## Out of scope for Stage 2

Same list as Stage 1 (dense/mesh reconstruction, multi-session/map-
merging, semantic mapping, non-`machine_hall` EuRoC rooms, other
datasets), plus: GPU acceleration, SIMD micro-optimization below the
algorithmic level, and real-time targets on hardware other than this
repo's own development machine — "real-time" here means "the algorithm
and its data structures don't scale worse than the sensor rate," not "fast
on a Raspberry Pi."

## Risks

- **Marginalization (M1) is real engineering, not a config flag.** It's
  the same Schur-complement machinery `slam-optim` already has for
  landmarks, generalized one level up to keyframges — but "generalized"
  still means new code, new failure modes, and a real chance of a subtle
  prior-construction bug that looks like "it converged, just to a worse
  answer" (the same silent-bug risk Stage 1's own Risks section flagged
  for the original solver). Budget real debugging time, and reuse the
  established technique from Stage 1's M4/M7 debugging: check whether
  ground truth satisfies the assembled system before assuming the solver
  is at fault.
- **M0's default-run truncation could hide the exact scaling problem this
  stage exists to fix.** Keep an explicit full-sequence mode in
  `bin/slam-run` (even if slow) so M5's real-time validation is measured
  on real, complete sequences before declaring victory — a truncated clip
  that happens to fit inside the window can look real-time for reasons
  that have nothing to do with actually fixing the scaling.
- **Parallelism (M4) can silently break determinism** (Stage 1's own
  cross-cutting requirement) if reduction order isn't fixed — a race that
  only perturbs the 8th decimal place of a landmark position is exactly
  the kind of bug that's invisible until it isn't.
- **"Real-time" is measured on one machine.** Like Stage 1's own SOTA
  numbers, treat the 1.0 real-time-factor bar as validated on this repo's
  development machine, not as a portable guarantee — re-check it if the
  development environment changes.
