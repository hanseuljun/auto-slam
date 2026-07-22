---
name: solver-uses-btreemap-not-hashmap-for-determinism
description: slam-optim's LM solver now accumulates landmark Schur-complement contributions via BTreeMap, not HashMap — HashMap's per-process-randomized iteration order made re-running the identical pipeline on the identical sequence produce measurably different trajectories, a real violation of Stage 1's own determinism requirement, found while building Stage 2's M0 benchmarking harness.
metadata:
  type: decision
---

# Decision: `slam-optim`'s solver uses `BTreeMap`, not `HashMap`, wherever iteration order affects accumulated floating-point results

## Decision

`crates/slam-optim/src/solver.rs`'s `build_normal_equations` and
`optimize` now use `std::collections::BTreeMap` instead of `HashMap` for
`LandmarkSchur::h_lp`, `NormalEquations::landmark_schur`, and the
`by_landmark` grouping built at the top of `build_normal_equations`.

## Why

Building Stage 2's M0 (the evaluation + timing harness, finishing Stage
1's M9), `bin/slam-run` was run three times on the *identical* bounded
600-frame `MH_01_easy` clip and produced three different results: 242,
68, and 113 keyframes, with correspondingly different ATE numbers. No
code changed between runs — same binary, same input.

Root cause: `build_normal_equations` groups reprojection observations by
landmark into a `HashMap`, then iterates that map to Schur-eliminate each
landmark and accumulate its contribution into the *shared* `h_pp`/`b_p`
normal-equations matrix/vector (`crates/slam-optim/src/solver.rs`, the
`for (&landmark_idx, obs_list) in &by_landmark` loop and the nested
`h_lp` double-loop inside it). Floating-point addition isn't associative,
so the order landmarks get folded into `h_pp`/`b_p` affects the exact
(if tiny, per-step) numerical result. `HashMap`'s default hasher
(`RandomState`) is seeded once per process, not per input — so the same
program re-run as a fresh process gets a different random seed, hence a
different bucket layout, hence a different iteration order, hence a
different (if initially tiny) numerical result. Over hundreds of LM
iterations across hundreds of keyframes, tiny per-step differences
compound and can flip discrete decisions with hard thresholds nearby
(e.g. `VioPipeline::process_frame`'s `self.tracks.len() >= 6` PnP gate,
or a track-loss recovery boundary) — which is how a sub-ULP-scale FP
difference turned into a 3-4x difference in keyframe count.

`plan/STAGE1.md`'s own cross-cutting infrastructure section already
states this as a requirement — "Deterministic, reproducible runs (fixed
RANSAC seeds) so accuracy regressions are attributable to code changes,
not run-to-run noise" — but only called out RANSAC seeding explicitly;
this `HashMap` usage (introduced with M5's Schur-complement solver) was
an unnoticed second source of the same class of bug, invisible until a
tool that runs the same input more than once (`bin/slam-run`, built for
Stage 2's M0) surfaced it.

`BTreeMap` iterates in ascending key order — a pure function of the keys
themselves (landmark/keyframe indices, which are assigned deterministically
elsewhere in the codebase), independent of insertion order or any
process-level random seed. Confirmed fixed empirically: three repeated
`cargo run --release --bin slam-run -- data/machine_hall/MH_01_easy` runs
after this change all report identical 261 keyframes and identical
0.137m ATE.

## How to apply

Any future collection that gets *iterated* to accumulate into a shared
floating-point result (a matrix, a vector, a running sum feeding into a
numerical comparison) must not use `HashMap` for that iteration — use
`BTreeMap`, or collect keys into a `Vec` and sort them first, before
relying on the iteration order. `HashMap` is still fine for pure lookups
(`slam_backend::vio.rs`'s `local_landmark_ids`, for instance, is only
ever used via `.entry()`/indexed lookup keyed off other, already-
deterministic iteration — never iterated itself to accumulate a shared
value — so it was correctly left as-is; verify this reasoning holds
before assuming any given `HashMap` is safe, don't just convert every one
reflexively).

If a future milestone (Stage 2's M3, sparse-aware normal equations, or
M4, `rayon` parallelism) introduces new shared-accumulator patterns —
especially parallel reduction, where thread scheduling order is its own
nondeterminism source on top of container iteration order — re-apply the
same discipline: fix the reduction order explicitly (e.g. `rayon`'s
`fold`+`reduce` with a deterministic combine order, not an unordered
`for_each` writing into shared mutable state).
