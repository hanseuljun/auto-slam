---
name: sensor-yaml-derived-imu-weights-reverted
description: solver_config_from_sensor_noise (deriving SolverConfig weights from sensor.yaml's real noise densities) exists in slam-optim, tested, but is NOT wired into bin/slam-run or bin/slam-inspect — measured on real MH_* data at two scope levels and both regressed accuracy on most sequences (4 of 5), so the tuned ad hoc defaults stay in use. A genuine negative result, recorded rather than discarded.
metadata:
  type: decision
---

# Decision: sensor.yaml-derived IMU/reprojection weights tried, measured, reverted

## Decision

`slam_optim::solver_config_from_sensor_noise` exists, is exported, and is
unit-tested (`solver::tests::sensor_noise_derived_weights_move_the_right_
direction`), but `bin/slam-run` and `bin/slam-inspect` both construct
`VioParams` with `SolverConfig::default()` (the original ad hoc weights),
not this function. It's available for future use, not wired in by
default.

## Why

Stage 2 M6 (finishing Stage 1's M10) explicitly scopes "real
`sensor.yaml`-derived noise weighting (replacing the ad hoc weights
`decisions/0006` flagged)." Two versions were built and measured against
real data via `bin/slam-run` across all five `MH_*` sequences:

**Full version** (derived `reprojection_weight`, `imu_rotation_weight`,
`imu_velocity_weight`, `imu_position_weight`, `bias_gyro_rw_weight`,
`bias_accel_rw_weight` from `sensor.yaml`'s noise/random-walk densities
via the standard "integrated white noise" scaling, `Var[∫w dt] = sigma^2
* dt`): regressed MH_02 (0.184m -> 0.194m) and MH_03 (0.511m -> 1.045m,
more than doubled), despite improving MH_01/04/05. Root cause identified:
this formula only models the white-noise component of gyro/accel
measurement error assuming *perfectly known bias* — it ignores bias
*uncertainty*'s own contribution to preintegration error, which the full
nonlinear preintegration covariance (bias-coupling Jacobians,
`Preintegration`'s own state) would include. The derived
`imu_rotation_weight` came out ~27,800x more "confident" than the tuned
ad hoc value — a huge, unwarranted swing in relative trust between IMU
and vision.

**Narrowed version** (dropped the IMU rotation/velocity/position
derivation — kept it at `Default`'s tuned values — derived only
`reprojection_weight` and the `bias_gyro_rw_weight`/`bias_accel_rw_weight`
split, both more directly justified: reprojection weight is a simple
pixel-noise-to-normalized-coordinate conversion, and the *bias random
walk* densities directly describe bias uncertainty growth, unlike the
noise densities used for rotation/velocity/position): **still regressed
4 of 5 sequences** — MH_01 0.169->0.174, MH_02 0.184->0.201, MH_03
0.511->0.546, MH_05 0.455->0.925 (more than doubled) — only MH_04
improved (1.191->0.787).

Even the narrowest, most directly-justified derivation didn't hold up
against real data. The likely explanation: `sensor.yaml`'s noise
densities are nominal manufacturer/calibration specs, not necessarily
what's actually realized in this specific real recording — and the
original ad hoc weights, hand-tuned against real MH data during M5/M6,
had likely absorbed other unmodeled error sources (feature
matching/detection noise beyond pure pixel noise, camera-IMU sync
jitter, non-Gaussian outliers) that a textbook noise-density formula
can't capture. This matches the same class of risk `decisions/0006`
flagged for the IMU factor's Jacobian — "looks more principled" doesn't
automatically mean "measurably better," and only real-data measurement
settles it.

## How to apply

Don't re-wire `solver_config_from_sensor_noise` into the real binaries
without new evidence it helps. If revisited: the real fix is *full*
nonlinear preintegration covariance propagation (via `Preintegration`'s
own Jacobians, accounting for bias-uncertainty coupling) — not a bigger
version of the same "isolated per-residual-type formula" approach tried
here, which has now failed twice at two different scopes. That's a
separate, larger undertaking with the same correctness-risk profile as
`decisions/0006`'s deferred analytic IMU Jacobian — likely warrants its
own milestone, not a quick sub-step. If attempted, measure the *same*
way this was measured (`bin/slam-run` across all five real sequences,
compare against `docs/RESULTS.md`'s current numbers) before wiring it in
as the default — that discipline is what caught this regression before
it shipped.
