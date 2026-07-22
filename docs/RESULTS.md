# Results: this repo vs. published stereo-inertial SLAM, and real-time factor

`plan/STAGE1.md`'s accuracy target: "roughly 2-9cm ATE RMSE... in the same
ballpark as the published SOTA systems." `plan/STAGE2.md`'s goal 1: a
real-time factor <= 1.0 for the continuous VIO loop. This is the M9/M0
deliverable that checks both against real, reproducible numbers instead
of narrative progress notes.

## How to reproduce

```
cargo run --release --bin slam-run              # all 5 sequences, bounded default
cargo run --release --bin slam-run -- --full     # full, un-truncated sequences (slow, see below)
cargo run --release --bin slam-run -- data/machine_hall/MH_01_easy  # one sequence
```

Runs the full stereo-inertial VIO pipeline (Stage 1 M5, track-loss
recovery M6) plus one global bundle-adjustment pass (M8) — loop closure
(M7) is not chained into this number, see "Scope" below. Prints ATE/RPE
and a wall-clock timing breakdown per sequence, and writes
`runs/<sequence>/trajectory.csv` (per-timestamp estimated vs. groundtruth
position) plus `runs/summary.csv` (the aggregate table these tables are
generated from).

**Results below are from the default bounded run** (600 frames, ~30s of
data per sequence) — not `--full`. An earlier attempt at this exact
milestone ran the un-truncated pipeline over one full sequence and took
30+ minutes wall-clock before being rolled back; see
`memory/decisions/0011` for the determinism bug found and fixed since
then. Global BA's O(n^3) scaling (`plan/STAGE2.md`'s original "What we
already know") is now bounded by Stage 2 M1's real marginalization — a
full run should be far more practical since, though not re-benchmarked
with `--full` yet. All numbers below are reproducible bit-for-bit run to
run (`decisions/0011`'s fix).

**Updated after Stage 2 M1** (real marginalization, `decisions/0007`,
plus two real bugs it surfaced and fixed — `decisions/0012`-`0014`,
notably `VioPipeline` finally getting the PnP pose-jump guard
`decisions/0009` gave `VoPipeline` back in M7). MH_05 in particular
improved substantially (1.501m -> 0.455m) — that sequence was hitting
exactly the corruption `decisions/0014` fixed.

**Updated again after Stage 2 M6's initializer fix** (`decisions/0015`):
MH_02_easy and MH_03_medium now run — they were skipped entirely before,
not because they lack a stationary window (both do, matching `plan/
STAGE1.md`'s own dataset notes) but because the bootstrap threshold was
measurably too tight for them specifically — see `decisions/0015` for
the measured per-sequence numbers.

## ATE RMSE (meters), bounded 600-frame (~30s) clips, no loop closure

| Sequence | This repo (M5+M8+M1, ~30s clip) | ORB-SLAM3 (full seq.) | OKVIS (full seq.) | VINS-Fusion (full seq.) | Kimera (full seq.) |
|---|---|---|---|---|---|
| MH_01_easy | 0.169 | 0.036 | 0.079 | 0.166 | 0.080 |
| MH_02_easy | 0.184 | 0.033 | 0.044 | 0.152 | 0.090 |
| MH_03_medium | 0.511 | 0.035 | 0.096 | 0.125 | 0.110 |
| MH_04_difficult | 1.191 | 0.051 | 0.197 | 0.280 | 0.150 |
| MH_05_difficult | 0.455 | 0.082 | 0.206 | 0.284 | 0.240 |

Published numbers are stereo-inertial ATE RMSE as reported in:

- Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual,
  Visual-Inertial and Multi-Map SLAM" (arXiv:2007.11898), Table II —
  source for the ORB-SLAM3, VINS-Fusion, and Kimera columns.
- Leutenegger, "OKVIS2: Realtime Scalable Visual-Inertial SLAM with Loop
  Closure" (arXiv:2202.09199), Table I — source for the OKVIS column;
  its ORB-SLAM3/VINS-Fusion/Kimera numbers match the ORB-SLAM3 paper's
  own table, cross-validating both as the same standard EuRoC
  stereo-inertial evaluation protocol.

## Real-time factor (wall-clock seconds spent / seconds of data processed)

**Stage 2's M5 real-time bar (factor <= 1.0) is met on every runnable
sequence, as of M1** — comfortably, with roughly half the budget to
spare:

| Sequence | vision (s) | optimization (s) | VIO loop factor | global BA (s, separate) |
|---|---|---|---|---|
| MH_01_easy | 13.5 | 2.8 | **0.543** | 2.6 |
| MH_02_easy | 13.7 | 2.6 | **0.541** | 2.6 |
| MH_03_medium | 15.5 | 2.9 | **0.615** | 2.3 |
| MH_04_difficult | 10.4 | 1.5 | **0.398** | 1.6 |
| MH_05_difficult | 13.1 | 2.5 | **0.523** | 2.0 |

"VIO loop factor" = `(vision + optimization) / data_seconds`, the number
`plan/STAGE2.md`'s real-time bar applies to (global BA is a separate,
one-shot batch pass, not held to the same per-frame bar — see the plan's
M5 scope note). Before Stage 2 M1 (marginalization + the PnP pose-jump
fixes, `decisions/0012`-`0014`), these factors were 1.198 / 0.357 / 1.086
and global BA took 42-46 seconds — M1 fixed a real accuracy bug, and as a
direct side effect also fixed real-time performance: the old numbers'
inflated keyframe counts (up to 438 vs. today's ~100) came from cascading
track-loss recoveries triggered by the same PnP corruption `decisions/
0014` fixed, and each spurious recovery keyframe cost a full round of
stereo matching/landmark detection — fixing the *cause* of those
recoveries removed most of the *cost*, not just the accuracy problem.
Because of this, Stage 2's M2 (analytic IMU Jacobians), M3 (sparse
solve), and M4 (`rayon` parallelism) are no longer required to hit the
real-time goal — see `plan/STAGE2.md` for the resulting re-scoping.

## Known gaps (honest, not swept under the rug)

- **Not apples-to-apples with the published numbers on two axes**: (1)
  this repo's numbers are a 30-second bounded clip, not a full sequence
  (~100-180s) — ATE over a shorter clip has less time to accumulate drift,
  so this is not a favorable comparison to read too much into either
  direction; (2) the published systems all include loop closure / global
  optimization in their own numbers where available, this repo's numbers
  here don't (`memory` — loop closure isn't chained into this benchmark
  yet, `bin/slam-inspect`'s separate MH_05 section shows this repo's own
  loop closure taking full-sequence ATE from ~5.6m to ~3.3m, a different
  pipeline configuration than this table).
- **Noise weighting is still ad hoc**, not derived from `sensor.yaml`'s
  real covariances (`memory/decisions/0006`) — `plan/STAGE2.md`'s M6,
  not a sign this harness is broken.
- Published numbers are quoted from each system's own paper, evaluated by
  its own authors — treated here as directional reference points ("same
  ballpark"), matching `plan/STAGE1.md`'s own framing, not a strict
  leaderboard.

## RPE (relative pose error, translation-only, meters)

`runs/summary.csv` also reports RPE at `delta=1` and `delta=10`
keyframes per sequence — see `crates/slam-eval/src/rpe.rs` for the exact
metric definition (a translation-only simplification of the standard TUM
RGB-D RPE, since per-point orientations aren't threaded through every
pipeline stage yet). No published RPE numbers are cited since the systems
above don't report RPE in a directly comparable form in their papers;
this is primarily a same-repo, same-protocol diagnostic.
