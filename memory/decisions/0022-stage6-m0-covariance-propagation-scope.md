---
name: stage6-m0-covariance-propagation-scope
description: Stage 6 M0 scope decision — real preintegration covariance propagation means extending Preintegration to propagate a 9x9 covariance matrix via the Forster et al. recursion (reusing the bias Jacobians it already computes), then replacing SolverConfig's three ad hoc scalar IMU weights with a per-factor information matrix derived from it, instead of another isolated per-residual-type formula (the approach decisions/0016 already tried twice and reverted).
metadata:
  type: decision
---

# Stage 6 M0: what "real" preintegration covariance propagation means here

## The gap, precisely

`SolverConfig`'s three IMU weights (`imu_rotation_weight`,
`imu_velocity_weight`, `imu_position_weight`, `crates/slam-optim/src/
solver.rs`) are fixed, isotropic scalars applied uniformly to **every**
IMU factor in the problem, regardless of that factor's own preintegration
interval (`dt`) or any correlation between rotation/velocity/position
error. Two real consequences, confirmed by reading the weight-application
code directly (`solver.rs`'s `cost`/Jacobian-weighting functions): a
factor spanning a normal ~0.5s keyframe interval and one spanning a much
longer interval (e.g. after a track-loss-recovery-forced gap, `plan/
STAGE4.md` M2's own 45-52% recovery-rate finding) get the *same* weight,
when the longer one should physically be trusted less — and the three
error components are weighted independently when the real preintegration
error is correlated across them (rotation error propagates into velocity
and position error through the same physics `Preintegration::corrected`
already models via its bias Jacobians).

`decisions/0016` already tried the "obvious" fix twice — deriving these
three weights from `sensor.yaml`'s raw noise densities via the same
"integrated white noise over a representative dt" formula that works
fine for the bias-random-walk weights — and both attempts regressed real
accuracy (MH_03 more than doubled, 0.511m -> 1.045m, at the full-scope
attempt). Root cause, already diagnosed there: that formula assumes
*perfectly known bias*, so it ignores bias *uncertainty*'s own
contribution to preintegration error entirely — a real term that only
the full nonlinear preintegration covariance would include, via the
bias-coupling Jacobians `Preintegration` already computes and exposes
(`d_rotation_d_bias_gyro`, `d_velocity_d_bias_gyro`, `d_velocity_d_bias_
accel`, `d_position_d_bias_gyro`, `d_position_d_bias_accel`) but doesn't
yet turn into an actual covariance matrix. `crates/slam-imu/src/
preintegration.rs`'s own doc comment already flagged this as deferred
("Covariance propagation... is deferred to M5, where it has an actual
consumer") — Stage 1 M5 built the consumer (the windowed backend) but
never came back to build the covariance itself; `SolverConfig`'s own doc
comment repeats the same deferral. This isn't a new discovery, just the
first time this stage's own scope commits to actually doing it.

## The decision: implement Forster et al.'s own covariance recursion, not another isolated formula

`Preintegration` already implements the on-manifold preintegration
*mean* (rotation/velocity/position deltas) and *bias Jacobians* from
Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial
Odometry" — the same paper defines a covariance propagation recursion
alongside the mean/Jacobian one, reusing the *same* per-step quantities
`integrate_measurement` already computes (the rotation increment, the
bias Jacobians). Concretely:

1. Extend `Preintegration`'s state with a 9x9 covariance matrix (rotation,
   velocity, position — matching `imu_residual`'s own 9-dim residual
   ordering), propagated one step at a time inside `integrate_measurement`
   via `Sigma_{k+1} = A_k Sigma_k A_k^T + B_k Sigma_eta B_k^T`, where
   `A_k` is the same discrete-time state-transition Jacobian the mean/
   bias-Jacobian propagation already implicitly uses, and `Sigma_eta` is
   the per-step raw gyro/accel measurement noise covariance (from
   `sensor.yaml`'s noise densities, the same inputs `solver_config_from_
   sensor_noise` already takes — this isn't a new input source, just a
   different, more complete use of it).
2. Bias uncertainty growth (the bias random walk) feeds into this same
   recursion rather than being a separate, disconnected weight
   (`bias_gyro_rw_weight`/`bias_accel_rw_weight` currently exist as their
   own factor type, `BiasRwFactorSpec` — keep that factor as-is; the new
   covariance is specifically for the 9-dim IMU factor's own rotation/
   velocity/position residual, not a replacement for bias random walk
   modeling).
3. In `slam-optim`, replace the three scalar `imu_*_weight` fields'
   effect with a per-factor 9x9 information matrix (`Sigma^-1`, or its
   Cholesky factor for the sqrt-weighted-residual form the solver already
   uses elsewhere — matching the existing `sqrt_reproj_w`/`sqrt_w`
   pattern) computed from that specific `ImuFactorSpec`'s own
   `Preintegration`. Each factor gets its *own* weight, naturally
   dt-dependent and naturally correlated across rotation/velocity/
   position, instead of one global scalar per component shared by every
   factor in the problem.

## Why not another per-residual-type formula

`decisions/0016` already tried the cheaper alternative twice, at two
different scopes, and both regressed real data — repeating a third
variant of "derive a scalar from `sensor.yaml` some other way" would be
the same mistake with different arithmetic. The actual, named gap is
structural (missing bias-uncertainty coupling and missing dt/correlation
dependence), and the fix has to be structural too: real propagation
through the same Jacobians `Preintegration` already maintains for exactly
this purpose, not a new closed-form shortcut.

## What this depends on, and what depends on it

M1 (analytic IMU Jacobians) isn't strictly required before this — the
covariance recursion's `A_k`/`B_k` matrices are a different (if related)
derivation from the residual's own Jacobian used in the solver's normal
equations, and `Preintegration`'s bias Jacobians are already analytic,
not numerical (only the *residual*'s Jacobian in `imu_factor.rs` is
numerical, per `decisions/0006`). But M1 is still done first in this
stage's own milestone order because a wrong residual Jacobian would be
much harder to notice once this covariance work also changes the
weights it's multiplied against — better to have one less moving part
under suspicion when M2's own real-data measurement either improves
things or doesn't.

## Fresh baseline (this stage's own starting point, not assumed from Stage 5)

Full-sequence run, all 5 sequences, current commit (`87925a1`, Stage 5
complete): see `docs/RESULTS.md`'s existing table and `memory/
decisions/0021`'s own numbers — re-confirmed fresh as part of this
milestone rather than trusted from memory; no change expected or found
(same commit, deterministic pipeline, `decisions/0011`).
