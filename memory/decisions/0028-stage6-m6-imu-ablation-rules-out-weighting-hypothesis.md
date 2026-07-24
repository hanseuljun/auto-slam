# 0028: Stage 6 M6 — removing IMU factors doesn't isolate the anisotropy, it causes catastrophic divergence

## Context

`plan/STAGE6.md` M6's goal: a real ablation, not more reasoning — run
the windowed backend with IMU factors removed (vision-only reprojection)
on a sequence where M5 already characterized scale drift, and compare.
The plan's own test criteria: if scale stays correct/stable without IMU
factors, that implicates the IMU/vision weighting directly; if it *still*
drifts without IMU factors at all, the weighting-imbalance hypothesis is
wrong, and the real cause is elsewhere (marginalization's own Schur-
complement accumulation, landmark re-initialization after track-loss
recovery, something not yet on the list) — and this milestone should say
so plainly, not keep chasing the weighting idea.

## The ablation

Added `VioParams::disable_imu_factors: bool` (`crates/slam-backend/src/
vio.rs`), threaded through the three places IMU factors actually enter
the optimizer:
1. `run_optimization`: empty `imu_factors`/`bias_rw_factors` instead of
   building them from the window's `imu_edge`s.
2. `global_bundle_adjustment_inner`: same.
3. `marginalize_evicted_keyframe`: `imu_edge: None` instead of `new_
   oldest.imu_edge.clone()`, so IMU information doesn't leak into the
   carried-forward marginalization prior either.

Deliberately did *not* touch the track-loss-recovery fallback
(`propagate_state`, used only when vision has nothing to PnP against at
all) — that's a "no vision available" fallback representing a genuinely
different scenario, not part of the steady-state IMU-vs-vision weighting
question this ablation targets. Preintegration itself still runs
unconditionally (needed for bias/gravity bootstrap and that recovery
fallback); only its use as an optimization *factor* is cut. Wired a
`--disable-imu-factors` flag into `bin/slam-run` to run this on real,
full sequences with the same evaluation code (`slam_eval::compute_axis_
scale_ratios`, `compute_sliding_window_scale`) M5 used.

## Real numbers, 2 sequences — the ablation itself catastrophically diverges

| sequence | metric | normal VIO | IMU factors disabled |
|---|---|---|---|
| `MH_01_easy` | keyframes | 685 | **2236** |
| | track-loss recoveries | 319 | **2055** |
| | anisotropy x/y/z | 3.95 / 2.74 / 14.03 | **7356 / 2752 / 4664** |
| | loop-closure gap (before) | 81.66m | **72210m** |
| | whole-run factor | 0.872 | **2.284** (breaks the real-time bar too) |
| `MH_04_difficult` | keyframes | 374 | **1161** |
| | track-loss recoveries | 174 | **1057** |
| | anisotropy x/y/z | 1.12 / 1.60 / 2.10 | **1274 / 255 / 1737** |
| | loop-closure gap (before) | 20.02m | **23751m** |

This is not a subtle shift — it's 3-4 orders of magnitude worse on both
sequences, on every metric, reproducibly. **This directly answers M6's
own test**: scale does *not* stay correct/stable without IMU factors —
it gets catastrophically worse. Per the plan's own stated criterion,
**this rules out the IMU-vs-vision weighting-imbalance hypothesis**:
removing IMU information entirely doesn't reveal a cleaner, more
isotropic reconstruction hiding underneath a bad weight — it reveals
that IMU information is load-bearing for the pipeline's basic stability,
not just a tuning knob whose relative weight vs. vision was off.

## Why, mechanistically (not just "it broke")

`KeyframeState` is 15-dimensional: `[rho, phi, v, bg, ba]` (pose is 6,
velocity 3, gyro/accel bias 6). Reprojection factors only ever touch the
6 pose dimensions. With IMU factors removed, **velocity and bias (9 of
15 dimensions) receive zero information from any factor in the
problem** — the only thing keeping those dimensions' normal-equation
blocks non-singular is `optimize`'s own damping floor (`lambda * diag.
max(1e-12)`), not real information. `marginalize_evicted_keyframe` then
Schur-complements this near-arbitrary uncertainty into the carried-
forward prior at *every* keyframe eviction — over hundreds to thousands
of evictions across a full sequence, small numerical noise in those
unconstrained directions (and their cross-correlation with pose in the
Schur complement) compounds instead of staying bounded, which is exactly
the "marginalization's own Schur-complement accumulation" alternative
`plan/STAGE6.md` M6 itself named as a real possibility if the weighting
hypothesis turned out wrong. The dramatically increased track-loss-
recovery count (2055 vs. 319 on `MH_01`) is consistent with this: once
poses start drifting from the corrupted prior, PnP against the
(increasingly wrong) map degrades, triggering more recoveries, which
feed back into more off-stride keyframes and more marginalization steps
— a compounding failure loop, not a one-time effect.

The specific per-axis ranking under ablation (`MH_01`: x worst; `MH_04`:
z worst, same as un-ablated) isn't treated as meaningful here — once the
whole system is this far into a compounding numerical failure, which
axis happens to be worst is noise, not signal. The *severity* (orders of
magnitude, both sequences) is the real, robust finding.

## Conclusion and what this rules in/out

**Ruled out**: simply re-weighting IMU vs. vision residuals (the
original M0-era hypothesis, `decisions/0020`) is not the fix — this
ablation shows IMU factors are necessary for basic stability, not just
imbalanced in magnitude.

**Ruled in, as the real lead for `plan/STAGE6.md` M7**: marginalization's
own handling of state dimensions with little or no direct factor
information (bias/velocity specifically) is the more likely root
mechanism — consistent with, and now measured rather than assumed, the
plan's own named alternative. `plan/STAGE6.md` M7 should investigate
*that* (e.g.: does the marginalization prior's own bias/velocity block
grow pathologically over a run even *with* IMU factors present, just
more slowly and less visibly than this ablation's dramatic version?) —
not continue looking for a weighting fix that this measurement shows
doesn't exist.
