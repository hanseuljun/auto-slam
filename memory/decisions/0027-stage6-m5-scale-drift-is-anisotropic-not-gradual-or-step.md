# 0027: Stage 6 M5 — the scale anomaly is anisotropic, not simply gradual or a step

## Context

`plan/STAGE6.md` M5's goal: instrument and directly measure the scale
anomaly `decisions/0020` found (the fitted Sim3 alignment scale was
nowhere near ~1.0), to determine whether it's gradual (compounding
optimization drift) or a step-change (one specific event). Built
`compute_sliding_window_scale` (`crates/slam-eval/src/align.rs`): fits
`umeyama_alignment` over a sliding window of consecutive trajectory
points, giving a *local*, over-time scale instead of one whole-
trajectory number.

## First result: the isotropic metric doesn't cleanly answer the question

Running this on `MH_01_easy`'s full trajectory (20s windows) gave wildly
non-monotonic values: 0.016, 0.008, 0.034, 0.228, 0.077, 0.025, 0.019,
0.039, 0.069, 0.170, 0.078, 0.031, 0.017 — swinging over an order of
magnitude, repeatedly, not settling into either a smooth monotonic ramp
(gradual) or two flat plateaus with one transition (step-change).
Confirmed this isn't a step-conditioning artifact: the same qualitative
wave-like pattern persists at window sizes of 20s, 60s, and 90s (half the
trajectory), and persists whether measured on the loop-closure-corrected
trajectory or the raw pre-correction one (though the exact values shift
between the two, since the loop correction reshapes the trajectory).

## Investigating why: the true error is anisotropic, not a uniform scale

Sanity-checked the raw trajectory shapes directly: on `MH_01_easy`,
estimated path length is 317m vs. groundtruth's 79m (~4x), but the
per-axis standard deviations (after rotating estimated into
groundtruth's own frame via `umeyama_alignment`'s fitted rotation — a
raw, non-rotated per-axis comparison is meaningless, since each
trajectory's own world frame is arbitrary) are **not proportional**:
x≈4.0x, y≈2.7x, z≈14.0x (post-loop-closure) — confirmed again on the
pre-loop-closure trajectory (x≈1.5x, y≈7.1x, z≈13.5x — the x/y ordering
flips depending on whether the loop correction, which mostly reshapes
the horizontal plane, is applied, but z stays consistently the largest
in both). A single isotropic Umeyama scale **cannot represent this** —
it reports a single compromise number per fit, which is exactly why the
sliding-window series swings so much: different windows sample different
mixes of x/y/z motion (a real flight path's own direction of travel
isn't uniform over time), so the "compromise" isotropic scale drifts
toward whichever axis's own (very different) true ratio dominates that
window's own motion. Built `compute_axis_scale_ratios` (rotates into a
common frame first, then compares per-axis variance) to measure this
directly instead of inferring it from the isotropic metric's own noise.

## Real numbers, 2 sequences

| sequence | ATE rmse | x ratio | y ratio | z ratio |
|---|---|---|---|---|
| `MH_01_easy` | 4.058m | 3.950 | 2.739 | **14.030** |
| `MH_04_difficult` | 6.279m | 1.124 | 1.600 | **2.097** |

**Z is the worst axis on both sequences**, by a wide margin on `MH_01`
and a smaller but still-largest margin on `MH_04`. `MH_01`'s absolute
distortion (up to 14x) is far more severe than `MH_04`'s (up to 2.1x)
despite `MH_01` having the *better* ATE (4.058m vs. 6.279m) — a real,
important finding in its own right: the isotropic Sim3-aligned ATE
metric can absorb a large amount of anisotropic error into its one scale
parameter, making a run look more accurate than its underlying
reconstruction actually is. This sharpens `decisions/0020`'s own
tentative worry ("this comparison may be more favorable to this repo
than a strictly fair one would be") into a specific, measured mechanism,
not just a suspicion.

## What this does and doesn't answer

**Doesn't answer** the plan's original framing (gradual vs. step-change)
cleanly, because that framing assumed a single scalar scale was the
right thing to track in the first place — it isn't. The isotropic
sliding-window series is real data, but it's better understood as noise
from trying to fit one number to an inherently 3-parameter (at least)
problem, not as a drift *rate* to characterize.

**Does answer**, more usefully: the scale anomaly isn't isotropic at
all, and the Z axis is consistently the most distorted across 2
sequences with very different overall character (an "easy" and a
"difficult" one) — a specific, reproducible, and (given it survives
both loop-closure states and multiple window sizes) robust signal that
`plan/STAGE6.md` M6/M7 should investigate directly (e.g.: is Z EuRoC's
own gravity-adjacent axis, and does the IMU-vs-vision weighting
hypothesis specifically implicate vertical/gravity-direction handling
more than horizontal?), rather than continuing to look for a gradual-
vs-step answer to a question this measurement shows was underspecified.

## Instrumentation kept as real, reusable capability

- `slam_eval::compute_sliding_window_scale` and `compute_axis_scale_
  ratios` (both tested — the latter validated against a synthetic
  trajectory with a *known* anisotropic distortion, recovering the
  planted ratios to within finite-sampling noise).
- `bin/slam-run`'s per-sequence report now prints both a sliding-window
  scale table and the per-axis anisotropy ratio for every run — a
  permanent diagnostic, not a one-off measurement, so any future run
  (any sequence, any config) surfaces this signal automatically.
- `crates/slam-eval/examples/scale_probe.rs`: a small standalone tool
  (`cargo run -p slam-eval --example scale_probe -- <trajectory.csv>
  [window_seconds] [step]`) for probing an existing run's saved
  `trajectory.csv` at any window size without re-running the pipeline —
  used during this investigation to check whether the wave pattern was a
  window-size artifact (it isn't) and is generally useful for follow-up
  work on this same question.
