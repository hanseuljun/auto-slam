# 0024: Stage 6 M2 — real preintegration covariance, tried as solver weighting, reverted

## Context

`plan/STAGE6.md` M2's goal: replace `SolverConfig`'s 3 fixed scalar IMU
weights (`imu_rotation_weight`/`imu_velocity_weight`/`imu_position_weight`,
never physically derived — `decisions/0016` found the simpler "integrated
white noise over a representative dt" shortcut regressed real accuracy)
with a real per-factor information matrix derived from each IMU factor's
own `Preintegration::total_covariance` (Forster et al.'s recursion:
9-dim error state `[δφ;δv;δp]`, state-transition matrix A (9x9) and
noise-input matrix B (9x6), `Σ_{k+1} = A_k Σ_k A_k^T + B_k Σ_η B_k^T`,
plus bias-uncertainty terms via the existing bias Jacobians).

## What was built and validated

- `Preintegration::covariance()`/`total_covariance()` (`crates/slam-imu/
  src/preintegration.rs`): real step-by-step covariance propagation
  through `integrate_measurement`, plus bias-uncertainty terms combining
  `total_covariance`'s two random-walk-density arguments with the
  existing bias Jacobians. Validated via Monte Carlo (4000 trials, Box-
  Muller sampling, compared analytic vs. sample covariance diagonal at
  35% relative tolerance) — passed on the first try, and this part is
  **kept**: it's real, tested infrastructure, just not wired into the
  solver's own weighting (see below).
- Diagonal-only, not full 9x9 correlated: `total_covariance`'s off-
  diagonal correlation structure was deliberately not modeled — matches
  this solver's existing sqrt-information-diagonal weighting pattern
  (reprojection, bias-random-walk) and was good enough to validate the
  propagation itself. Moot for weighting now that the weighting use was
  reverted (see below), but the propagation and this scoping choice both
  remain sound if something else consumes `covariance()`/
  `total_covariance()` later.

## Tried as solver/marginalization weighting — measured, reverted

`imu_factor_sqrt_information_diagonal` fed `total_covariance`'s diagonal
into `solver::compute_cost`/`build_normal_equations` and
`marginalization::marginalize_keyframe`, replacing the 3 ad hoc scalars
entirely. Measured on all 5 `machine_hall` sequences (bounded 30s clips,
`--frames 600`), against the Stage 6 M1 baseline:

| sequence | M1 baseline (ad hoc weights) | M2 covariance-weighted | M2 reverted (ad hoc, kept marginalization fix) |
|---|---|---|---|
| MH_01_easy | 0.155m | 0.177m (+14.2%) | 0.162m (+4.5%) |
| MH_02_easy | 0.207m | 0.201m (-2.9%) | 0.198m (-4.3%) |
| MH_03_medium | 0.893m | 1.267m (+41.9%) | 0.768m (-14.0%) |
| MH_04_difficult | 1.005m | 1.672m (+66.4%) | 1.180m (+17.4%) |
| MH_05_difficult | 0.597m | 1.201m (+101.2%) | 0.632m (+5.9%) |

Covariance-based weighting regressed bounded-clip ATE on 4 of 5
sequences, up to +101%. Reverting the weighting (restoring the ad hoc
scalars) while **keeping** the marginalization numerical-stability fixes
(see below) recovered — and on 2 of 5 sequences slightly beat — the M1
baseline. This isolates the regression to the weighting scheme itself,
not to any Jacobian or marginalization-numerics change made alongside it.

**Root cause, confirmed by direct measurement, not assumed:** for a
representative 0.5s keyframe interval with EuRoC's real ADIS16448 noise
densities, the covariance-derived sqrt-information weights are 30-166x
*more confident* than the old ad hoc scalars — rotation ~8321 vs the ad
hoc scalar's implied 50, velocity ~556 vs 20, position ~2036 vs 50 (all
computed directly from `Preintegration::total_covariance`, not
estimated) — while reprojection weight (`sqrt(1/0.002^2)` = 500) stayed
fixed. Physically, EuRoC's IMU really is that precise over a *single*
short interval in isolation. But the solver isn't reasoning about a
single interval in isolation: over-trusting short-horizon IMU-only
propagation relative to vision means less correction from reprojection
when bias estimation or vision tracking (this pipeline sees 40-91 track-
loss recoveries per 600-frame bounded clip) introduces real error, so
drift compounds unboundedly between corrections instead of being pulled
back. This is the **same failure mode `decisions/0016` already found and
reverted** for the bias-random-walk weights — physically-derived
per-factor IMU weighting is a second, independent instance of "more
physically correct in isolation" not implying "more accurate in this
specific pipeline."

**Decision: keep the ad hoc scalars for `imu_rotation_weight`/
`imu_velocity_weight`/`imu_position_weight`** (matching pre-M2 behavior
exactly), keep `Preintegration::covariance()`/`total_covariance()` as
validated, tested infrastructure for future uncertainty-aware work (e.g.
`plan/STAGE6.md` M5/M6's scale-drift investigation might want it), and
remove `imu_factor_sqrt_information_diagonal` (the solver-facing
consumer) since nothing calls it once the weighting reverted — genuinely
dead code, not speculative infrastructure to keep around.

## A real numerical bug the (now-reverted) weighting surfaced along the way

While covariance-based weighting was still in place, using real
(non-zero) noise densities in a marginalization test
(`imu_plus_unique_landmarks_marginalization_prior_alone_recovers_ground_
truth_k1`, previously passing) caused catastrophic divergence (final
velocity ~9573 m/s, translation ~800m) — not slow convergence, a real
bug:

1. `marginalize_keyframe`'s Schur complement (`h_kk_reg`'s inversion)
   mixed reprojection-scale information (~1e5-1e6) with the
   covariance-derived weighting's tiny bias-block entries (~1e-9) in the
   same 15x15 matrix — a ~1e14-15 dynamic range, at double precision's
   own limit — producing small but real **negative** eigenvalues in the
   output marginal information matrix (confirmed via `symmetric_eigen()`,
   e.g. -3.3e-6 alongside legitimate positive eigenvalues as small as
   ~1e-9, not negligible by comparison).
2. `solver::compute_cost`'s prior term is the quadratic form `delta^T *
   information * delta - 2 * information_vector . delta` — unbounded
   below along any negative-eigenvalue direction, so LM's own
   accept/reject gate (`trial_cost < current_cost`) *rewarded*
   arbitrarily large steps along it, exactly the divergence observed.
3. The companion `imu_only_...` test (no landmarks) happened not to
   excite that direction within its perturbation/iteration budget, so it
   passed even with the same latent bug present — a real, silent
   landmine, not something specific to the landmark case.

**Fix, kept as defense in depth even after the weighting itself was
reverted** (today's ad hoc weights don't reach anywhere near this
dynamic range, but the fix is strictly more numerically sound regardless
of what weighting scheme feeds it, and costs nothing):

- **Numerically stable elimination**: `jacobi_scaled_solve` replaced
  `h_kk_reg.try_inverse()` (explicit inverse formation) with Jacobi
  (diagonal) preconditioning down to a well-conditioned scaled matrix,
  Cholesky solve on *that*, then unscale — solving directly for
  `h_kk_inv * h_kk1` and `h_kk_inv * b_k` rather than forming `h_kk_inv`
  and multiplying. Measured to agree with the old plain-inverse result to
  ~1e-13 on the well-conditioned `imu_only` case (so this alone wasn't
  why results differed) — real algorithmic best practice, but on its own
  didn't fix the negative-eigenvalue problem (same root cause: the
  *matrix itself*, not just how it's inverted).
- **Dead end tried along the way**: a first PSD-projection attempt
  reconstructed the information matrix via `V * clip(Λ, 0) * V^T` (the
  textbook approach). This fixed the landmark case's divergence but
  broke the previously-passing `imu_only` case badly (a large spurious
  rotation error) — this matrix's eigenvalues cluster tightly near zero
  across several of its 15 dimensions (bias directions are only weakly
  observable from a single short IMU edge — a real, physically-expected
  near-null subspace, not a code bug), so that subspace's eigenvectors
  are numerically near-arbitrary, and reconstructing from them scrambles
  information a gentler fix never touches.
- **Actual fix kept**: `project_onto_psd_cone` only needs the *smallest*
  eigenvalue (not the eigenvectors) and, if negative, shifts the whole
  spectrum up by a uniform `-min_eigenvalue + 1e-12` — mathematically
  guarantees PSD (a uniform spectral shift can't introduce new negative
  eigenvalues) while leaving every other direction's information exactly
  as computed, since it never touches the eigenvectors at all.

All 18 `slam-optim` tests, plus the full workspace suite, pass with the
final state: ad hoc IMU weights restored, marginalization solve fixes
kept.

## Follow-ups not taken

- `imu_factor_sqrt_information_diagonal` was removed as dead code rather
  than kept "for later" — if a future milestone wants per-factor
  covariance weighting again (e.g. after M3's sparse solver or M6's
  weighting-hypothesis test change the picture), it's a small function to
  re-add from this decision's own description, not lost knowledge.
- Threading real per-sequence calibration noise densities into
  `VioParams` (`gyro_noise_density`/`accel_noise_density` fields, EuRoC
  defaults `1.6968e-4`/`2.0000e-3`, still used to feed `Preintegration::
  new` so `.covariance()` itself stays real) rather than reading them
  from `seq.calibration.imu0` directly — `bin/slam-run` doesn't yet wire
  the per-sequence YAML value through (all 5 `machine_hall` sequences
  share the same ADIS16448 sensor and identical values, so this is a real
  gap only if a future dataset has different per-sequence noise).
