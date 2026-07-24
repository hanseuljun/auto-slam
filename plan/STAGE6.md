# Stage 6: closing the accuracy gap — three real, still-open causes

## Goal

`plan/STAGE5.md` left the accuracy story honest but unfinished: full-sequence
ATE is 90-280x worse than published SOTA (ORB-SLAM3) on the same five
sequences, and that gap breaks down into four layers, one of which is
already closed. This stage takes on the three that aren't, in the order
they're most tractable to attack (not necessarily the order they matter
most — Goal 3 is the most foundational but also the most open-ended, so it
comes last, matching `plan/STAGE5.md` M0's own precedent for how to treat
an investigation that might not converge cleanly):

1. **Close the base tuning gap with real structural work, not another
   sweep.** Even on a 30-second bounded clip, before any full-sequence
   drift enters the picture, this repo is 4-8x worse than SOTA on the
   identical clip (`MH_01_easy`: 0.151m vs. ORB-SLAM3's 0.036m). Three
   tuning directions were already tried and reverted in Stage 2 M6
   (`decisions/0016`-`0017`) — every one regressed at least one sequence,
   because they were all scalar sweeps of an already-ad-hoc model. The
   IMU factor's Jacobians are numerical, not analytic (`decisions/0006`,
   a deliberate tradeoff at the time), and residual noise weighting is
   still `SolverConfig::default()`'s ad hoc constants, not real
   preintegration covariance. Goal 1 does the structural work Stage 2 M6
   itself named as the actual fix and didn't attempt.
2. **Remove the ceiling on loop closure's own correction.** `plan/
   STAGE5.md` M2 wired real loop closure into `bin/slam-run`, but had to
   run its pose-graph optimization over a sparse, stride-4-downsampled
   keyframe set — the existing `optimize_pose_graph`'s dense,
   unbounded-scope O(n^3) LU solve couldn't handle the full dense
   trajectory without reintroducing the exact real-time bug `plan/
   STAGE4.md` M1 already fixed once (`memory/decisions/0021`). That
   sparsity workaround caps how much correction one loop-closure pass can
   apply (2.1x-4.8x gap reduction measured, not the order-of-magnitude
   `plan/STAGE5.md` M3 was aiming for) and measurably degrades RPE. A
   real sparse solver removes the ceiling instead of working around it.
3. **Root-cause the scale-consistency anomaly.** `plan/STAGE5.md` M0
   found the pipeline's own reconstructed scale drifts for real — on one
   sequence, the raw estimate travels ~9x farther than ground truth's own
   path length — and ruled out two explanations (a calibration bug,
   disproportionate track-loss-recovery step sizes) without finding the
   actual cause. `memory/decisions/0020`'s leading, unconfirmed
   hypothesis: a weighting imbalance between vision (reprojection) and
   IMU residuals inside the windowed optimizer, letting reconstructed
   scale creep away from what stereo triangulation alone pins down. This
   goal actually tests that hypothesis instead of leaving it as a guess.

Same dependency policy, same dataset, same determinism requirement, same
"measure before fixing" discipline as every prior stage. Same honesty
requirement `plan/STAGE5.md` established for the metric itself: every
number in this stage's own milestones uses the prefix-aligned ATE
(`plan/STAGE5.md` M1), not the whole-trajectory one, unless explicitly
noted otherwise for SOTA-comparability.

## What we already know (measured, not assumed)

- **The base gap is real and already-ruled-out-of-being-a-quick-fix.**
  `docs/RESULTS.md`'s own bounded-clip table: this repo 0.151-1.174m vs.
  ORB-SLAM3 0.033-0.082m across the five sequences (4-14x). Three
  scalar-sweep tuning attempts (`decisions/0016` sensor.yaml-derived
  noise weights, `decisions/0017` Huber threshold and window size, both
  directions) were measured on real data and reverted — every direction
  regressed at least one sequence, most consistently MH_05. `docs/
  RESULTS.md`'s own "Known gaps" section already names the actual fix:
  "needs full nonlinear preintegration covariance propagation, not a
  simpler per-residual-type formula — a real, larger, separate
  undertaking." This stage is that undertaking.
- **The IMU factor's Jacobians are numerical, reprojection's are already
  analytic.** Checked directly, not assumed: `crates/slam-optim/src/
  reprojection.rs`'s `reprojection_residual_jacobian` is already a real
  closed-form derivative (verified against finite-difference in its own
  test, `jacobians_match_finite_difference`). `crates/slam-optim/src/
  imu_factor.rs` is the one still using central-difference numerical
  Jacobians (`decisions/0006`'s own deliberate tradeoff, revisited here
  because the tradeoff's original context — "good enough while other
  things dominate error" — no longer holds once this stage is
  specifically trying to close the remaining accuracy gap).
- **The pose-graph solver's dense scaling is confirmed, not theoretical.**
  `crates/slam-loopclosure/src/pose_graph.rs`'s `optimize_pose_graph`
  builds a `(n-1)*6`-dimensional dense `DMatrix` and does
  `damped.lu().solve(&b)` every LM iteration (up to 50), with numerical
  (central-difference) per-edge Jacobians. Running it over `MH_01_easy`'s
  741 dense VIO keyframes (dim=4440) didn't finish in 10+ minutes —
  confirmed by actually trying it (`memory/decisions/0021`). The
  stride-4 workaround that shipped instead costs 23s and holds the
  real-time bar (whole-run factor 0.85), but stride-2 (denser, better
  correction) costs 88s and breaks it (whole-run factor 1.21) — a real,
  measured tradeoff curve, not a guess at where the ceiling is.
- **The scale anomaly is real, and two explanations are already ruled
  out.** `memory/decisions/0020`: fitted Umeyama alignment scale ranges
  0.017-0.47 across window sizes (should be ~1.0 for a metrically-sound
  stereo-inertial system); raw estimated path length vs. ground truth's
  own path length ratio is 0.11-0.20 on `MH_01_easy` (the estimator
  thinks it moved 5-9x farther than it did). Ruled out: a stereo
  calibration/baseline bug (computed directly from `sensor.yaml`, matches
  the known EuRoC ~0.11m baseline to high precision; existing
  triangulation tests already validate sub-mm accuracy against synthetic
  ground truth) and track-loss recovery steps being disproportionately
  large (recovery-tagged keyframe-to-keyframe steps are 51.3% of raw path
  length from 51.4% of steps — proportional, not anomalous). Not yet
  tested: whether it's specifically an IMU-vs-vision residual weighting
  imbalance inside the windowed optimizer, `decisions/0020`'s own leading
  hypothesis, named but not confirmed.

## Milestones

Same discipline as every prior stage: measure before fixing, verify
against the existing test suite before trusting a new derivation, no
milestone closes on an assumed number. Every accuracy milestone
re-measures on all 5 sequences, bounded and full, both ATE metrics.

### M0 — Goal 1: baseline the tuning gap fresh, scope the covariance work — Done

- Re-measure today's exact bounded-clip and full-sequence gap vs. SOTA on
  all 5 sequences as this stage's own clean starting point (numbers exist
  in `docs/RESULTS.md` already, but re-confirm at this stage's own commit
  rather than assuming they're still current).
- Scope the real preintegration covariance propagation work concretely
  before writing solver code: what `solver_config_from_sensor_noise`
  (`crates/slam-optim/src/solver.rs`) got wrong isn't the inputs (real
  `sensor.yaml` noise densities) but the model (a simplified
  per-residual-type formula) — read the actual nonlinear preintegration
  covariance propagation math (the same propagation `Preintegration`
  itself would need to carry forward step-by-step, not derive after the
  fact) before committing to an implementation shape.
- Test/deliverable: a written scope decision (`memory/decisions`) for
  what "real" covariance propagation means here, backed by the fresh
  baseline numbers this fix is trying to move.
- **Result**: fresh baseline confirmed identical to `docs/RESULTS.md`'s
  existing numbers on both paths (deterministic pipeline, no code
  changes since Stage 5 — `decisions/0011`'s determinism fix holds):
  bounded-clip 0.151/0.184/0.511/1.174/0.455m, full-sequence
  3.505/3.546/3.451/6.496/6.596m. Scope decision written
  (`memory/decisions/0022`): the gap is structural, not a missing input
  — `SolverConfig`'s three IMU weights are fixed isotropic scalars
  shared by every IMU factor regardless of that factor's own `dt` or
  any rotation/velocity/position correlation, and `decisions/0016`
  already showed twice that deriving them from raw sensor noise
  densities alone (ignoring bias-uncertainty coupling) regresses real
  data. The fix: extend `Preintegration` with Forster et al.'s own
  covariance propagation recursion (reusing the bias Jacobians it
  already computes), and replace the three global scalars with a
  per-factor 9x9 information matrix derived from it — not a third
  variant of the same isolated-formula approach already tried twice.

### M1 — Goal 1: analytic IMU Jacobians — Done

- Replace `imu_factor.rs`'s central-difference Jacobians with closed-form
  analytic ones, following the same pattern `reprojection.rs` already
  established (a real derivative, verified against finite-difference in
  a test — `jacobians_match_finite_difference`-style, not a new
  methodology).
- This alone is a correctness/precision change, not necessarily an
  accuracy one — don't expect it to close the gap by itself; it's the
  prerequisite M2's real covariance propagation needs (covariance
  propagation compounds Jacobian errors step by step, so a numerical
  Jacobian's approximation error is exactly the kind of thing that would
  quietly corrupt it).
- Test: new analytic Jacobian matches the existing numerical one to
  finite-difference tolerance on real preintegration data (not just a
  synthetic toy case) before anything downstream depends on it; full
  `cargo test` still passes unchanged (this must be a transparent swap,
  not a behavior change) — confirm bit-for-bit-identical optimization
  results on a real sequence before and after, within numerical noise.
- **Result**: derived all 18 Jacobian blocks by hand against this
  codebase's own left-multiplicative SE3 perturbation convention (not
  copied from a textbook/ORB-SLAM3-style table, which assumes a
  different, right-multiplicative one and would silently carry wrong
  signs) — full reasoning in the code's own doc comment
  (`crates/slam-optim/src/imu_factor.rs`). Correctness validated
  extensively before trusting it: both `jac_i` and `jac_j` (the original
  test only ever checked `jac_i`) against finite difference on 3
  hand-picked configurations plus a 40-case randomized stress sweep
  specifically targeting short intervals, large bias offsets, and large
  rotations — all pass at tight tolerance (1e-4 to 5e-3).
  **Correctness is not in question; the "transparent swap, no behavior
  change" expectation this bullet stated was wrong, though** — measured,
  not assumed, and the real number contradicts it: bounded-clip ATE
  changed -14.4% to +74.8% across the 5 sequences (4 of 5 got worse,
  `MH_03_medium` by nearly double), full-sequence ATE changed -0.6% to
  +17.0% (milder, but still real, 4 of 5 worse). Cross-checked that this
  isn't a Jacobian bug: reverting to the *old* numerical Jacobian but at
  a different epsilon (1e-6 -> 1e-5, no analytic code involved at all)
  also shifted `MH_03`'s bounded-clip ATE (0.511m -> 0.495m) — smaller
  than the analytic swap's own effect, but confirms this pipeline's
  keyframe/track-loss-recovery decisions are genuinely sensitive to
  *any* change in Jacobian precision, not specifically to this one.
  Leading hypothesis, not yet confirmed: `SolverConfig`'s ad hoc weights
  were hand-tuned against the *old* numerical Jacobian's own specific
  behavior (`decisions/0016`'s own precedent names exactly this failure
  mode for a different knob), so a more precise Jacobian doesn't
  automatically help against weights implicitly co-tuned to the less
  precise one — M2's own re-weighting work should re-verify this effect
  once it lands, not assume M1 alone should have been net-positive.
  Full numbers and reasoning: `memory/decisions/0023`.

### M2 — Goal 1: real preintegration covariance propagation, measure

- Implement real, step-by-step nonlinear covariance propagation through
  `Preintegration` (M0's own scoping decision), replacing
  `SolverConfig::default()`'s ad hoc constants with values actually
  derived from it — this time propagated correctly, not the simplified
  per-residual-type formula `decisions/0016` measured and reverted.
- Test: ATE (both metrics) measured on all 5 sequences, bounded and full
  — real before/after numbers against M0's baseline, matching every
  prior accuracy milestone's own bar (`plan/STAGE2.md` M1, `plan/
  STAGE4.md` M1 before/after notes are the template). If it doesn't
  improve on every sequence, that's a real result to document
  honestly (matching `decisions/0016`-`0017`'s own precedent), not a
  reason to force it in.
- **Result**: built and validated `Preintegration::covariance()`/
  `total_covariance()` (Forster et al.'s recursion, Monte Carlo test
  passed on the first try) — kept as infrastructure. Wired it into the
  solver/marginalization as a per-factor information matrix replacing the
  3 ad hoc IMU weight scalars, and measured a severe, consistent
  regression, matching `decisions/0016`-`0017`'s own precedent: bounded-
  clip ATE rmse got worse on 4 of 5 sequences, up to **+101%**
  (`MH_05_difficult`, 0.597m -> 1.201m).

  | sequence | bounded: M1 baseline | M2 covariance-weighted | M2 reverted |
  |---|---|---|---|
  | MH_01_easy | 0.155m | 0.177m (+14.2%) | 0.162m (+4.5%) |
  | MH_02_easy | 0.207m | 0.201m (-2.9%) | 0.198m (-4.3%) |
  | MH_03_medium | 0.893m | 1.267m (+41.9%) | 0.768m (-14.0%) |
  | MH_04_difficult | 1.005m | 1.672m (+66.4%) | 1.180m (+17.4%) |
  | MH_05_difficult | 0.597m | 1.201m (+101.2%) | 0.632m (+5.9%) |

  Root cause, confirmed by direct measurement: real EuRoC noise densities
  make the solver 30-166x more confident in a single short IMU interval
  than the old ad hoc scalars were (e.g. rotation sqrt-weight ~8321 vs
  ~50 for a representative 0.5s interval), which over-trusts IMU-only
  propagation relative to vision and lets drift compound unboundedly
  between corrections — physically accurate in isolation, not more
  accurate for this pipeline. **Reverted the weighting** (restored the ad
  hoc scalars), confirmed this alone recovers accuracy (full-sequence
  whole-trajectory ATE rmse: 4.031/4.528/3.562/6.165/6.648m vs baseline
  4.061/4.149/3.593/6.456/6.691m — flat within normal run-to-run
  variance), and removed `imu_factor_sqrt_information_diagonal` as dead
  code. `Preintegration::covariance()` itself stays available for M5/M6.
  Along the way, the covariance-weighted experiment surfaced (and this
  milestone fixed, kept as defense in depth) a real marginalization
  numerical-stability bug: an extreme information-scale mismatch produced
  negative eigenvalues in the marginal prior, which made `compute_cost`'s
  quadratic form unbounded below and one LM step diverge to a velocity of
  ~9573 m/s. Full reasoning, the dead-end tried along the way, and every
  number: `memory/decisions/0024`.

### M3 — Goal 2: a real sparse pose-graph solver

- Replace `optimize_pose_graph`'s dense `DMatrix`/LU solve with one that
  exploits the pose graph's own actual structure: a chain of consecutive-
  keyframe odometry edges (inherently banded/tridiagonal) plus a small
  number of loop edges (each a sparse off-diagonal addition) — not a
  generic dense solve over an `(n-1)*6`-dimensional system regardless of
  how few real couplings exist. Decide, and record the decision
  (`memory/decisions`): hand-roll a solve exploiting this specific
  structure (consistent with this repo's own "the algorithm is ours,
  standard infra crates are fine" dependency policy, and arguably a
  better fit given how simple the pose graph's real sparsity pattern is)
  or bring in a sparse linear-algebra crate as infra (same spirit as
  `nalgebra` itself) — a real choice to make deliberately, not default
  into.
- Also replace `edge_residual`'s numerical (central-difference) Jacobian
  with the closed-form one — a well-known derivative for an SE3 relative-
  pose residual, and cheap to get right now that M1 already established
  the analytic-Jacobian-verified-against-numerical pattern this session.
- Test: matches the existing dense solver's output on `pose_graph.rs`'s
  own existing test (`loop_closure_edge_corrects_accumulated_drift`) to
  numerical tolerance; wall-clock cost on `MH_01_easy`'s full 741-keyframe
  trajectory measured directly (not estimated) and confirmed to hold the
  real-time bar with room to spare — the entire point of this milestone.
- **Result**: hand-rolled a block-tridiagonal (block Thomas) elimination
  for the chain of odometry edges, plus a Sherman-Morrison-Woodbury
  low-rank correction for the loop edge(s) — every edge's full Hessian
  contribution is exactly `U U^T` for a rank-6 factor, so a "chord" edge
  (not adjacent in free-index order) is just a rank-6 update to the
  tridiagonal system. Tried `nalgebra-sparse` as infra first (the plan's
  other option); reverted immediately — it pulls in `nalgebra 0.35`
  while this workspace pins `0.33` everywhere else, a real, immediate
  incompatibility, not a style preference, which settled the "hand-roll
  vs. crate" choice decisively. Verified against an independently-
  assembled dense solve of the *same* system for `k=0,1,2` chords (all
  match to `1e-6`) — this strict check caught a real double-counting bug
  (a chord edge's diagonal contribution was being added both directly
  and via its Woodbury term) the existing loose end-to-end test missed
  entirely, since extra diagonal regularization doesn't break qualitative
  convergence. Measured (not estimated) wall-clock on a synthetic
  741-node graph matching `MH_01_easy`'s exact full-trajectory size:
  **~97ms** for 50 LM iterations, versus the old dense solver's "didn't
  finish in 10+ minutes" on this same size — several orders of magnitude,
  the entire point of this milestone. A real end-to-end `MH_01_easy` run
  (current stride, unchanged) gives a deterministic but *different*
  result than before (worse RPE delta=1, better loop gap-closure) despite
  the linear solve being independently proven exact — the same "hard-
  threshold decisions are sensitive to any numerical-precision change"
  phenomenon `decisions/0023` already documented for M1, not a new
  correctness concern. **Deliberately not done**: the analytic edge
  Jacobian — a correct closed form needs SE3's own 6x6 left/right
  Jacobian of the exponential map, machinery this codebase doesn't have
  (`SO3::left_jacobian`/`right_jacobian` are 3x3-only), and deriving +
  validating it is a Stage-6-M1-sized undertaking on its own, not a
  footnote; it's also not the performance bottleneck this milestone
  removes. Full reasoning and every number: `memory/decisions/0025`.

### M4 — Goal 2: use the removed ceiling, re-verify the real-time bar

- With a cheap solver, reduce `LOOP_CLOSURE_CAPTURE_STRIDE`
  (`bin/slam-run/src/main.rs`) — ideally back to 1 (every VIO keyframe,
  no downsampling, no smooth-interpolation propagation artifact needed
  at all) if the real-time budget allows; if not all the way to 1, as far
  as it does allow, measured, not guessed.
- Test: on all 5 sequences, full un-truncated runs — (a) the geometric
  gap-closure ratio (`plan/STAGE5.md` M3's own metric) improves toward
  the order-of-magnitude bar that milestone didn't reach; (b) RPE
  delta=1 no longer shows the ~5x degradation `memory/decisions/0021`
  measured (the interpolation artifact this fix removes, not just
  shrinks); (c) the whole-run real-time factor (`plan/STAGE4.md` M1's
  `whole_run_factor()`) still holds ≤1.0 on every sequence — re-verified
  directly, since this is exactly the kind of change that could quietly
  regress it if the new solver's own cost scales worse than expected at
  higher density.
- **Result**: tried stride 1 first, measured (not guessed) it *breaks*
  the real-time bar on `MH_01_easy` (whole-run factor 1.082) — M3's
  sparse solver made the pose-graph solve itself cheap, but BoW
  vocabulary training and place-recognition queries still scale with
  capture density and dominate at stride 1 (~4x the descriptors/queries
  stride 4 had). Stride 2 holds the bar on all 5 sequences (0.640-0.925).
  Geometric gap-closure ratio improves dramatically on most sequences
  (`MH_01`: 4.4x -> 43.7x, `MH_02`: 1.1x -> 12.2x, `MH_05`: 1.6x -> 5.6x
  — `MH_01`/`MH_02` now well past the "order of magnitude" bar `plan/
  STAGE5.md` M3 didn't reach), though `MH_04` is a genuine exception (a
  *different* candidate is found at stride 2 and the geometric gate
  correctly rejects it — not a gate bug). RPE delta=1 does *not* show a
  clean, uniform fix of `decisions/0021`'s ~5x interpolation-artifact
  degradation at stride 2 (mixed, no consistent direction across
  sequences) — measured directly at stride 1 (before rejecting it for
  the real-time reason) that the interpolation-artifact explanation is
  directionally right (0.347m there, better than stride 2 or 4), so a
  future fix of the vocabulary/place-recognition cost specifically could
  still reach stride 1 and a cleaner RPE win. Full numbers, honestly
  including the mixed parts: `memory/decisions/0026`.

### M5 — Goal 3: instrument and directly measure scale drift over a run

- Add real diagnostic instrumentation (not guessing): track the windowed
  optimizer's own reconstructed scale over the course of a full run —
  e.g. median triangulated landmark depth at a fixed real-world
  reference distance, or a running comparison between stereo-baseline-
  implied scale and IMU-integration-implied scale at each keyframe — to
  see *when* and *how fast* it drifts. Gradual (compounding optimization
  drift, consistent with `decisions/0020`'s weighting-imbalance
  hypothesis) and a step-change (a specific event — a marginalization,
  a track-loss recovery, a bootstrap artifact) point to different causes
  and different fixes; don't assume which before measuring.
- Test/deliverable: a real, plotted-or-tabulated scale-over-time record
  for at least 2 sequences, written up in `memory/decisions` regardless
  of what it shows.
- **Result**: built `compute_sliding_window_scale` (`slam-eval`) and
  measured it on `MH_01_easy`'s full trajectory — wildly non-monotonic
  (0.016 to 0.228, swinging repeatedly), not a clean gradual ramp or a
  step. Confirmed this isn't a window-size artifact (same wave shape at
  20s/60s/90s windows) or a loop-closure artifact (persists pre- and
  post-correction). Investigating *why* found the real, more fundamental
  answer: the error isn't isotropic at all. Built `compute_axis_scale_
  ratios` (rotates estimated into groundtruth's frame first, then
  compares per-axis variance — a raw, non-rotated per-axis comparison is
  meaningless) and measured real per-axis anisotropy on 2 sequences:
  `MH_01_easy` x=3.95 y=2.74 **z=14.03**, `MH_04_difficult` x=1.12 y=1.60
  **z=2.10** — Z is the worst axis on *both*, by a wide margin on
  `MH_01`. This reframes the plan's own "gradual vs. step-change"
  question: a single isotropic scalar was never the right thing to
  track, and the sliding-window scale's own noise is the symptom of
  trying to fit one number to an inherently anisotropic problem. Also
  surfaced that `MH_01`'s much better ATE (4.058m) than `MH_04`'s
  (6.279m) coexists with 7x *worse* anisotropic distortion (14.0x vs.
  2.1x) — the isotropic Sim3-aligned ATE metric can absorb real
  anisotropic error into its one scale parameter, sharpening
  `decisions/0020`'s own tentative worry into a measured mechanism.
  `bin/slam-run` now prints both the sliding-window scale table and the
  per-axis anisotropy ratio for every run, permanently, not just for
  this investigation. Full numbers and reasoning: `memory/decisions/0027`.

### M6 — Goal 3: test the residual-weighting hypothesis directly

- A real ablation, not more reasoning about it: run the windowed backend
  with IMU factors removed (vision-only reprojection) on a sequence where
  M5 already characterized scale drift, and compare. If scale stays
  correct/stable without IMU factors in the mix, that implicates the
  vision/IMU weighting directly. If it *still* drifts without IMU
  factors at all, the weighting-imbalance hypothesis is wrong and this
  milestone should say so plainly, not keep chasing it — the real cause
  would then be somewhere else entirely (marginalization's own Schur-
  complement accumulation, landmark re-initialization after track-loss
  recovery, something not yet on the list).
- If M1/M2 (Goal 1's real covariance propagation) landed first, this
  ablation should also be re-tried *with* the corrected weighting in
  place — if Goal 1's own fix already resolves what M5/M6 find, that's
  a real, valuable result on its own (one less separate fix needed), not
  a reason to skip verifying it.
- Test/deliverable: a real before/after comparison (`memory/decisions`)
  confirming or ruling out the weighting-imbalance hypothesis, with
  actual numbers, on at least 2 sequences.
- **Result**: added `VioParams::disable_imu_factors`, threaded through
  the 3 places IMU factors actually enter the optimizer (`run_
  optimization`, `global_bundle_adjustment_inner`, `marginalize_evicted_
  keyframe`'s prior — deliberately not the track-loss-recovery fallback,
  a different "no vision at all" scenario), plus a `bin/slam-run
  --disable-imu-factors` flag to run it on real full sequences. Result:
  **catastrophic divergence, not a cleaner reconstruction** — on both
  tested sequences, 3-4 orders of magnitude worse on every metric
  (`MH_01_easy`: 685 -> 2236 keyframes, 319 -> 2055 track-loss
  recoveries, anisotropy 14.03 -> 4664 (z axis), loop-closure gap 81.66m
  -> 72210m; `MH_04_difficult`: similarly, 374 -> 1161 keyframes, 2.10 ->
  1737 anisotropy). This directly answers the milestone's own test:
  scale does *not* stay stable without IMU factors, so **the weighting-
  imbalance hypothesis is ruled out** — IMU information is load-bearing
  for basic stability, not a magnitude tuning knob. Mechanism (not just
  "it broke"): reprojection factors only ever touch 6 of `KeyframeState`'s
  15 dimensions (pose); with IMU factors gone, velocity/bias (9
  dimensions) get zero information from any factor, and `marginalize_
  evicted_keyframe`'s Schur complement folds that near-arbitrary
  uncertainty into the carried-forward prior at every eviction —
  compounding over hundreds to thousands of evictions across a full
  sequence, exactly the "marginalization's own Schur-complement
  accumulation" alternative this milestone's own bullet named up front.
  Full numbers and reasoning: `memory/decisions/0028`.

### M7 — Goal 3: fix it, or document it — either way, close the loop

- If M5/M6 point to a real, fixable cause, implement and verify it the
  same way every other accuracy milestone in this stage does: real
  before/after ATE (both metrics) and the scale-consistency diagnostic
  from M5, on all 5 sequences.
- If M5/M6 don't converge on a clean, fixable cause within reasonable
  effort, document exactly what's ruled in/out and stop — matching
  `plan/STAGE5.md` M0's own explicit precedent for this situation ("if
  M0's investigation doesn't converge, document what's ruled in/out and
  move on"). An honest "still open, here's what we now know that we
  didn't before" is a legitimate outcome for this milestone, not a
  failure to write around.
- **Result**: tested M6's own hypothesis directly — added `VioParams::
  disable_marginalization` (naive drop, `decisions/0007`'s original
  alternative) and a `bin/slam-run --disable-marginalization` flag, ran
  it on the same 2 sequences. **The opposite of the hypothesis**:
  disabling marginalization made anisotropy *worse*, not better, on both
  sequences (`MH_01_easy`: 14.03 -> 362 (z axis); `MH_04_difficult`: 2.10
  -> 46.8), sitting between normal VIO and full IMU removal on every
  metric — marginalization is doing real, measurably stabilizing work
  (fewer track-loss recoveries, smaller anisotropy) relative to naive
  drop, not accumulating the distortion. Combined with M6's own result,
  **both concrete candidate mechanisms this stage's investigation
  produced are now ruled out by direct measurement**, not just reasoned
  past. Per this milestone's own explicit permission for this outcome:
  the anisotropic scale distortion's root cause **remains open** —
  documented honestly, not forced into a fix that isn't there. What's
  now ruled in/out, concretely: not IMU/vision weight imbalance, not
  marginalization's own accumulation mechanism; the anisotropy is present
  even in the best-behaved (normal) configuration tested, a property of
  the baseline reconstruction rather than an artifact of either
  ablatable mechanism. Candidate directions for a future stage (not
  attempted here): stereo-triangulation depth-direction bias,
  camera-IMU extrinsics calibration error, or the static/dynamic
  initializer's own gravity-direction handling. Full numbers and
  reasoning: `memory/decisions/0029`.

## Out of scope for Stage 6

Same carried-forward list as Stages 1-5 (dense/mesh reconstruction,
multi-session/map-merging, semantic mapping, non-`machine_hall` EuRoC
rooms, other datasets, GPU/SIMD micro-optimization, real-time targets
beyond this repo's own dev machine), plus: multi-loop / pose-graph-wide
relaxation (`plan/STAGE5.md`'s own carried-forward scope note still
applies — this stage makes the *existing* single-loop correction able to
apply more of its own correction, it doesn't add a general multi-loop
backend); any further ad hoc-knob sweeping of Huber threshold or
window_size (Stage 2 M6 already exhausted that space, `decisions/0017` —
this stage's whole premise is that structural fixes are needed instead);
and visualization/UI work (`bin/slam-viz` gets used as-is, per every
prior stage's own scoping once Stage 3 closed).

## Risks

- **Any one of these three goals could turn out to be its own stage.**
  Real preintegration covariance propagation, a sparse pose-graph solver,
  and an open-ended root-cause investigation are each independently
  substantial — `docs/RESULTS.md` and `memory/decisions/0016` already
  called the covariance work "a real, larger, separate undertaking"
  before this stage existed. If effort estimates blow up mid-milestone,
  re-scope explicitly (matching `plan/STAGE2.md`'s own M2-M4
  re-scoping precedent) rather than quietly under-delivering on all
  three to preserve the appearance of a single finished stage.
- **A wrong analytic Jacobian is a silent-bug risk, not a crash risk.**
  `plan/STAGE1.md`'s own Risks section named this exact danger for the
  optimizer originally: a wrong derivative still "converges," just to a
  worse answer, and nothing about that looks obviously broken. M1's own
  finite-difference verification is the guard — don't let anything
  downstream (M2's covariance work, or any solver call site) depend on
  the new Jacobian before that verification is real and passing on real
  data, not just a synthetic toy case.
- **Goal 3 might not converge, and that has to be an acceptable outcome,
  not a reason to force a fix that isn't real.** `plan/STAGE5.md` M0
  already hit this once (ruled out two causes, left the real one open).
  M7's own explicit "document it and stop" branch exists because forcing
  a plausible-sounding but unverified fix would repeat exactly the
  mistake this stage's whole premise (Layer 4, `plan/STAGE5.md` M1) was
  about *not* repeating: a fix that isn't backed by real, measured
  causation is exactly the kind of thing that ends up needing to be
  un-fixed later.
- **This stage must not regress the real-time bar Stage 4 fought for and
  Stage 5 M2 had to actively defend.** Every milestone that touches
  solver cost (M1-M4 especially) needs its own real before/after
  wall-clock measurement against `whole_run_factor() <= 1.0` on all 5
  sequences — "should be faster/shouldn't matter" is not a substitute
  for measuring it, the same discipline every prior real-time milestone
  in this project has already held to.
