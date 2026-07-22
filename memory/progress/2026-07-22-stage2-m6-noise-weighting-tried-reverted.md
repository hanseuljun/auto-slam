---
name: stage2-m6-noise-weighting-tried-reverted
description: Stage 2 M6 sub-step — sensor.yaml-derived SolverConfig noise weights were built, tested, and measured against real data at two scopes; both regressed accuracy on most sequences, so reverted (decisions/0016). A genuine negative result worth recording, not a milestone completion.
metadata:
  type: progress
---

# Stage 2 M6 — noise weighting: tried, measured, reverted

Continuing `plan/STAGE2.md`'s M6 (finishing Stage 1's M10) after the
MH_02/MH_03 bootstrap fix (`2026-07-21-stage2-m6-mh0203-bootstrap-fix.md`):
attempted the milestone's other headline item, "real `sensor.yaml`-derived
noise weighting (replacing the ad hoc weights `decisions/0006` flagged)."

## What happened

Built `slam_optim::solver_config_from_sensor_noise`, split
`SolverConfig::bias_rw_weight` into separate `bias_gyro_rw_weight`/
`bias_accel_rw_weight` (gyro and accel bias random walk are physically
distinct processes with different densities — lumping them was itself
part of the ad hoc scheme), and wired the derivation into `bin/slam-run`/
`bin/slam-inspect`.

Measured on real data (`bin/slam-run`, all five `MH_*` sequences) at two
scopes:

1. **Full derivation** (reprojection, IMU rotation/velocity/position,
   bias-gyro/accel-rw, all from `sensor.yaml`): regressed MH_02 and
   MH_03's ATE (MH_03 more than doubled, 0.511m -> 1.045m), improved
   MH_01/04/05.
2. **Narrowed derivation** (dropped IMU rotation/velocity/position,
   which are the ones the simplified formula gets most wrong — see
   below — kept only reprojection + bias-rw derived): still regressed 4
   of 5 sequences (only MH_04 improved).

Root cause understood, not just observed: the "integrated white noise"
formula used (`Var[∫w dt] = sigma^2 * dt`) only models measurement noise
assuming *perfectly known* bias — it has no term for bias *uncertainty*'s
own contribution to preintegration error, which the full nonlinear
covariance propagation (via `Preintegration`'s own bias Jacobians) would
include. The derived `imu_rotation_weight` came out ~27,800x more
"confident" than the tuned ad hoc value in the full-derivation version —
a large, unwarranted swing. Even the narrower version (reprojection +
bias-rw only, both more directly justified derivations) still
underperformed the ad hoc weights, most likely because those weights
were hand-tuned against real data during M5/M6 and had implicitly
absorbed other unmodeled error sources (feature-matching noise beyond
pure pixel noise, sync jitter) that a textbook noise-density formula
doesn't capture.

## Decision

Reverted: `bin/slam-run` and `bin/slam-inspect` both use
`SolverConfig::default()` again. `solver_config_from_sensor_noise` stays
in `slam-optim`, exported, unit-tested — available if a future session
does the full covariance-propagation version — just not wired in as the
default, since the simpler version measurably doesn't help. Full
writeup: `memory/decisions/0016`.

## Real numbers (bounded 600-frame/~30s clips, current committed state)

After this revert plus the earlier bootstrap-threshold fix (`decisions/
0015` — which also shifted MH_01's own number slightly, since a looser
threshold changed *which* stationary window it uses, even though MH_01
always ran):

| Sequence | ATE rmse | Real-time factor |
|---|---|---|
| MH_01_easy | 0.151m | 0.589 |
| MH_02_easy | 0.184m | 0.540 |
| MH_03_medium | 0.511m | 0.578 |
| MH_04_difficult | 1.174m | 0.412 |
| MH_05_difficult | 0.455m | 0.518 |

All five real-time factors comfortably under the 1.0 bar, consistent
with M5's earlier finding. `docs/RESULTS.md` updated with these
(replacing all five rows, not just the ones that changed, to keep the
table measured at one consistent code state).

## State at end of session / what's left in M6

- Noise weighting: properly fixing this needs full nonlinear
  preintegration covariance propagation (same correctness-risk class as
  the deferred Stage 2 M2), not attempted this session — a real,
  separate, larger undertaking.
- Outlier-gating threshold tuning and keyframe/window sizing remain
  open, not yet attempted.
- MH_04/MH_05-specific initializer robustness is lower priority than the
  MH_02/03 fix was: both already produce real numbers via the dynamic
  (not static) initializer, so there's no "produces nothing at all" gap
  to close there, unlike MH_02/03 before `decisions/0015`.

## Lesson worth carrying forward

This is the same "measure before assuming, and be willing to walk back a
change that doesn't measurably help" discipline this session applied
repeatedly (`decisions/0009`, `0012`-`0014`), now applied to a *negative*
result instead of a bug fix. A more "principled-looking" derivation
(real sensor densities instead of hand-picked constants) is not
automatically better — only real-data measurement settles it, and this
project's own harness (`bin/slam-run`, built in M0 specifically for this
kind of before/after comparison) is what caught it before it shipped as
a regression.
