---
name: stage6-m0-baseline-and-covariance-scope
description: Stage 6 M0 done — fresh baseline confirmed identical to docs/RESULTS.md on both bounded-clip and full-sequence paths (deterministic, no drift since Stage 5). Scoped the real preintegration covariance propagation work concretely (memory/decisions/0022): extend Preintegration with Forster et al.'s own covariance recursion, replace SolverConfig's three fixed isotropic IMU weights with a per-factor information matrix, not a third variant of the isolated-formula approach decisions/0016 already tried and reverted twice.
metadata:
  type: progress
---

# Stage 6 M0: baseline confirmed, covariance work scoped

## Baseline

Re-ran both `bin/slam-run` paths at the current commit (`87925a1`, Stage
5 complete) rather than trusting `docs/RESULTS.md`'s numbers from
memory. Both matched exactly:

- Bounded clip (`--frames 600`): 0.151 / 0.184 / 0.511 / 1.174 / 0.455m
- Full sequence (default): 3.505 / 3.546 / 3.451 / 6.496 / 6.596m

Confirms the pipeline is still deterministic (`decisions/0011`) and
nothing has drifted since Stage 5 closed — a real check, not an assumed
one, matching this project's own discipline for every stage's own M0.

## Covariance propagation, scoped concretely

Read `crates/slam-optim/src/solver.rs`'s actual weight-application code
(not just its doc comments) to pin down the gap precisely: the three IMU
weights (`imu_rotation_weight`/`imu_velocity_weight`/`imu_position_
weight`) are fixed isotropic scalars applied identically to *every* IMU
factor in a problem, regardless of that factor's own preintegration
interval or any rotation/velocity/position correlation. Combined with
`decisions/0016`'s own already-diagnosed root cause (deriving these from
raw sensor noise densities ignores bias-uncertainty's contribution to
preintegration error, and regressed real data twice at two different
scopes), the fix is now well-defined rather than open-ended: extend
`Preintegration` (`crates/slam-imu/src/preintegration.rs`, whose own doc
comment already flagged covariance propagation as "deferred... where it
has an actual consumer") with the Forster et al. covariance recursion,
reusing the bias Jacobians it already maintains, and replace the three
global scalars with a per-factor 9x9 information matrix. Full reasoning
and the exact propagation-equation shape: `memory/decisions/0022`.

Also confirmed M1 (analytic IMU Jacobians) doesn't strictly block M2
(the covariance recursion's transition matrices are a related but
different derivation from the residual Jacobian `imu_factor.rs` uses in
the solver) — M1 stays first anyway, to keep one fewer moving part under
suspicion when M2's real-data measurement runs.

## What's next

`plan/STAGE6.md` M1: replace `imu_factor.rs`'s central-difference
Jacobians with closed-form analytic ones, verified against finite
difference on real preintegration data, matching the pattern
`reprojection.rs` already established.
