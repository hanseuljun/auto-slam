---
name: stage5-m0-ate-alignment-uses-a-bounded-prefix-not-full-trajectory
description: Stage 5 M0 decision — root-caused the scale anomaly (Finding 4, plan/STAGE5.md), ruled out a calibration/geometry bug and disproportionate track-loss-recovery contribution, confirmed the pipeline's own reconstructed scale genuinely drifts over long runs (a real, separate, out-of-scope-for-Stage-5 estimator issue). For goal 1's "honest ATE near the start" metric: keep Sim3 (free-scale) alignment, but fit it using a bounded ~60-150 keyframe prefix (~30s, matching the existing bounded-clip convention) instead of the entire trajectory, added as a new metric alongside the existing full-trajectory one.
metadata:
  type: decision
---

# Stage 5 M0: what "honest ATE" should mean here

## What was ruled out

Three candidate explanations for `plan/STAGE5.md`'s Finding 4 (fitted
alignment scale far from 1.0, worse the longer a run continues) were
checked against real evidence, not assumed:

1. **A stereo calibration/baseline bug.** Computed the true baseline
   directly from `MH_01_easy`'s `sensor.yaml` (cam0/cam1 `T_BS`
   translations): ~0.1101m, matching the well-known EuRoC VI-sensor
   baseline. `crates/slam-geometry`'s `relative_pose_cam1_from_cam0` and
   its existing tests already use these exact real calibration numbers
   and round-trip triangulation to sub-mm accuracy
   (`bin/slam-inspect`'s own "triangulation round-trip check" output).
   Ruled out.
2. **IMU propagation physics being wrong (units/gravity/integration).**
   `crates/slam-backend/src/vio.rs`'s `propagate_state_matches_ground_
   truth_motion` test already validates the forward-physics model
   against a synthetic constant-angular/world-velocity ground truth to
   1e-3 tolerance. Ruled out.
3. **Track-loss recovery (IMU-only coasting, `plan/STAGE4.md` M2's
   45-52% keyframe rate) disproportionately inflating raw path length.**
   Directly measured: on `MH_01_easy`'s full run, recovery-tagged
   keyframe-to-keyframe steps account for 51.3% of total raw path length
   from 51.4% of all steps — essentially proportional, not
   disproportionate (mean step size 0.987 vs. 0.990 for non-recovery
   steps). Ruled out as *the* explanation, though recovery's sheer
   frequency likely still contributes to ordinary drift alongside every
   other estimation step.

## What was confirmed real

Forcing the alignment's scale to a fixed 1.0 (as would be correct if the
pipeline's own reconstruction were genuinely metric, which the ruled-out
checks above suggest it should be at the calibration/geometry level)
produces **dramatically worse** alignment than allowing free scale:
on `MH_01_easy`'s full run, fixed-scale (SE3) alignment gives 150-297m
error depending on window size, vs. 5-140m for free-scale (Sim3) at the
same windows. This means the pipeline's *own* reconstructed scale
genuinely drifts away from ground truth over a long run — it's not
solely an evaluation-methodology artifact, and not explained by either
ruled-out cause above. Most likely mechanism (not yet confirmed): the
windowed optimizer jointly weighs stereo reprojection residuals (which
pin absolute scale at each landmark's creation) against IMU factors:
over many keyframes/marginalization cycles, small relative-weighting
imbalances could let the *optimized* scale creep away from the
geometrically-correct one, distinct from either input being "wrong" on
its own. This is a real, substantial estimator-behavior question — but
fully root-causing and fixing *why* the optimizer lets scale drift is
comparable in scope to the noise-weighting investigation `decisions/
0016` already flagged as "a real, larger, separate undertaking, not a
sign this harness is broken." Out of scope for Stage 5's own goal 1
(which only needs the *metric* to stop hiding this, not the estimator
to stop drifting) — worth a dedicated future stage if pursued.

## The decision: bounded-prefix Sim3 alignment, not fixed-scale

Given scale drift is real (not a metric artifact to paper over) but not
this stage's job to fix, forcing scale=1.0 in the metric would just
re-expose an already-flagged, separately-scoped problem as a much
larger, less legible number — not what goal 1 needs. Instead: **keep
Sim3 (free-scale) alignment, but fit it using a bounded prefix of the
trajectory instead of every point in it.**

Swept prefix window sizes (10/30/60/100/150/200/400/full) on two
sequences (`MH_01_easy` full run, 725 keyframes; `MH_05_difficult` full
run, 446 keyframes):

| Sequence | k=10 | k=60 | k=100 | k=150 | full (today's metric) |
|---|---|---|---|---|---|
| MH_01 err[0] | 0.071 (unstable: rmse_all=88, err[end]=139) | 0.165 | 0.186 | 0.190 | 3.109 |
| MH_01 rmse_all | 88.4 | 5.17 | 5.44 | 5.37 | 3.868 |
| MH_05 err[0] | — | 0.195 | 0.215 | 0.469 | 1.735 |
| MH_05 rmse_all | — | 11.27 | 11.23 | 15.63 | 6.818 |

`k=10` confirms the mechanism (near-zero err[0]) but is numerically
unstable — a poorly-conditioned small window's rotation uncertainty has
a lever-arm effect (a small angular error translates into 100+m of
apparent error far from the anchor). `k=60-100` is the sweet spot on
both sequences: near-zero err[0] (0.165-0.215m, in the same range as
this pipeline's own bounded-600-frame-clip numbers already in `docs/
RESULTS.md`) without the small-window blowup, and — importantly — a
real, *larger* `rmse_all` than today's full-trajectory metric (5.2-15.6m
vs. 3.9-6.8m), because it's no longer letting late drift pull the fit to
flatter the aggregate number. `k=60-100` keyframes is also not an
arbitrary new magic number: it's within the same ~30s duration as the
bounded-clip mode `plan/STAGE4.md` M3 kept as `--frames 600`'s
fast-iteration mode (~100-106 keyframes at this pipeline's typical
stride+recovery rate), reusing an already-load-bearing concept in this
codebase rather than inventing a new one.

## What M1 implements

A new `compute_ate` variant in `crates/slam-eval` that fits the Umeyama
alignment using only the first `align_prefix_len` points (caller-
supplied, not hardcoded — `bin/slam-run` will derive it from a duration
like "first 30s of data" via timestamps, since keyframe *count* isn't
portable across sequences with different track-loss-recovery rates) but
applies the resulting transform to the *entire* trajectory, alongside
the existing full-trajectory-fit `compute_ate` (kept, not removed, for
continuity with `docs/RESULTS.md`'s existing SOTA-comparison table —
clearly labeled as such, not presented as the only "accuracy" number).
