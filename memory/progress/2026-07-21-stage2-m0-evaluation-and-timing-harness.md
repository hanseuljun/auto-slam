---
name: stage2-m0-evaluation-and-timing-harness
description: Stage 2 M0 done — finished Stage 1's M9 (slam-eval RPE/CSV reports, bin/slam-run, docs/RESULTS.md vs published SOTA) extended with wall-clock timing; found and fixed a real determinism bug (HashMap iteration order) along the way.
metadata:
  type: progress
---

# Stage 2 M0 — evaluation + timing harness (finishes Stage 1 M9)

Landed the first milestone of `plan/STAGE2.md`, rebuilding what the
earlier rolled-back M9 attempt tried to do, this time with timing
instrumentation and a bounded default run built in from the start (per
that attempt's own lesson, `memory/progress/2026-07-21-stage2-plan.md`).

## What's done

- `slam-eval`: `compute_rpe`/`RpeStats` (translation-only RPE at a
  configurable delta, Sim3-aligned same as `compute_ate`),
  `TimingBreakdown`/`real_time_factor()` (vision + optimization time,
  divided by data duration — global BA and loop closure excluded, per
  `plan/STAGE2.md`'s scope note), `build_report`/`TrajectoryReport`,
  `write_trajectory_csv`/`write_summary_csv`.
- `slam_backend::VioPipeline`: per-stage wall-clock timing
  (`vision_time`/`optimization_time`/`global_ba_time` fields, `timing()`
  accessor returning `VioTiming`) — instrumented directly in
  `process_frame` (piecewise, since there are multiple early-return paths)
  and `global_bundle_adjustment`.
- `bin/slam-run` (new crate): runs the full VIO+global-BA pipeline over
  one or more sequences, prints ATE/RPE/timing, writes per-sequence
  trajectory CSVs + an aggregate summary CSV. Defaults to a **bounded**
  600-frame (~30s) run per sequence, not a full sequence — `--full` opts
  into the complete run. This is the direct fix for the earlier attempt's
  mistake (a full run took 30+ minutes before any usable number came
  back); the bounded tool itself takes ~70-90s per sequence, practical to
  iterate with.
- `docs/RESULTS.md`: accuracy table (this repo vs. ORB-SLAM3/OKVIS/
  VINS-Fusion/Kimera, cited from arXiv:2007.11898 Table II and
  arXiv:2202.09199 Table I — cross-validated, both papers report matching
  numbers for the systems they share) and a real-time-factor table, with
  honest caveats about what's and isn't apples-to-apples (bounded clip vs.
  full sequence, no loop closure chained in, two sequences don't
  bootstrap yet).

## A real bug found and fixed along the way

Building this harness meant running the *same* pipeline on the *same*
input more than once for the first time in the project's history — and
that immediately surfaced genuine nondeterminism: three `cargo run`s of
the identical 600-frame `MH_01_easy` clip gave three different keyframe
counts (242, 68, 113) and different ATE numbers. Root cause: `slam-optim`'s
solver (`crates/slam-optim/src/solver.rs`) accumulated landmark
Schur-complement contributions into the shared normal-equations
matrix/vector by iterating `HashMap`s, whose default hasher is randomized
per *process* (not per input) — so floating-point summation order, and
therefore the exact numerical result, differed run to run, and compounded
over hundreds of LM iterations into meaningfully different trajectories.
Fixed by switching to `BTreeMap` (deterministic, key-sorted iteration)
everywhere iteration order could affect an accumulated result. Confirmed
fixed empirically (three repeat runs now give bit-identical 261
keyframes / 0.137m ATE) and with a permanent regression test
(`solver::tests::optimize_result_does_not_depend_on_observation_insertion_order`).
Full writeup: `memory/decisions/0011`.

This was a real violation of `plan/STAGE1.md`'s own stated cross-cutting
requirement ("deterministic, reproducible runs... so accuracy regressions
are attributable to code changes, not run-to-run noise") that had been
silently present since M5 introduced the Schur-complement solver, and
would have undermined every number in this milestone's own deliverable
had it not been caught before finalizing `docs/RESULTS.md`.

## Real numbers (bounded 600-frame/~30s clips, from `docs/RESULTS.md`)

- MH_01_easy: ATE rmse 0.137m, real-time factor 1.198 (vision 31.5s +
  optimization 4.4s over 29.9s of data), global BA 46.3s (separate).
- MH_04_difficult: ATE rmse 1.481m, real-time factor 0.357.
- MH_05_difficult: ATE rmse 1.501m, real-time factor 1.086, global BA
  42.1s.
- MH_02_easy/MH_03_medium: skipped — no stationary window found for the
  IMU bootstrap with the current thresholds (a pre-existing gap,
  `bin/slam-inspect` shows the same thing independently, not new).

**Two of three runnable sequences are already close to Stage 2's
real-time bar (1.0) on the VIO loop alone**, even before any of Stage 2's
planned speedups — the real, measured problem is global BA (tens of
seconds for a 30-second clip), directly confirming `plan/STAGE2.md`'s
"What we already know" and validating the plan's ordering (marginalization
before claiming victory on the real-time goal).

## Not done yet (correctly out of scope for M0)

- MH_02/MH_03 initializer robustness — Stage 2's M6 (finishing Stage 1's
  M10).
- Loop closure isn't chained into `bin/slam-run`'s benchmarked number —
  same reasoning as the earlier rolled-back attempt's scope note (loop
  closure operates on `VoPipeline`, only MH_05 has a real loop); still a
  good M6/M10-adjacent follow-up, not M0's job.
- The actual speedups (marginalization, analytic IMU Jacobians, sparse
  solve, `rayon` parallelism) — Stage 2's M1-M4, next.
