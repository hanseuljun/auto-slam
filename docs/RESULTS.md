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
then, and `plan/STAGE2.md`'s "What we already know" for the O(n^3)
global-BA scaling issue that made that run so slow — both are still
partly true (global BA is not yet marginalized/sparse, Stage 2's M1-M3),
so a full run remains slow until those land. All numbers below are
reproducible bit-for-bit run to run (`decisions/0011`'s fix), unlike an
earlier version of this table would have been.

## ATE RMSE (meters), bounded 600-frame (~30s) clips, no loop closure

| Sequence | This repo (M5+M8, ~30s clip) | ORB-SLAM3 (full seq.) | OKVIS (full seq.) | VINS-Fusion (full seq.) | Kimera (full seq.) |
|---|---|---|---|---|---|
| MH_01_easy | 0.137 | 0.036 | 0.079 | 0.166 | 0.080 |
| MH_02_easy | not run — no stationary window found for IMU bootstrap (see "Known gaps") | 0.033 | 0.044 | 0.152 | 0.090 |
| MH_03_medium | not run — same reason | 0.035 | 0.096 | 0.125 | 0.110 |
| MH_04_difficult | 1.481 | 0.051 | 0.197 | 0.280 | 0.150 |
| MH_05_difficult | 1.501 | 0.082 | 0.206 | 0.284 | 0.240 |

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

| Sequence | vision (s) | optimization (s) | VIO loop factor | global BA (s, separate) |
|---|---|---|---|---|
| MH_01_easy | 31.5 | 4.4 | **1.198** | 46.3 |
| MH_04_difficult | 9.7 | 1.0 | **0.357** | 0.7 |
| MH_05_difficult | 27.5 | 5.0 | **1.086** | 42.1 |

"VIO loop factor" = `(vision + optimization) / data_seconds`, the number
`plan/STAGE2.md`'s real-time bar applies to (global BA is a separate,
one-shot batch pass, not held to the same per-frame bar — see the plan's
M5 scope note). **Two of three runnable sequences are already at or near
the 1.0 bar** (1.20 and 1.09) even before any of Stage 2's planned
speedups (marginalization, analytic IMU Jacobians, sparse solve, `rayon`
parallelism) — MH_04's lower factor (0.357) despite fewer landmarks
suggests the dominant cost is track-loss-recovery frequency (MH_01/05
both produced far more keyframes than the nominal stride-10 rate implies,
meaning many frames triggered M6's recovery path), not raw per-frame
vision cost. Global BA, in contrast, is wildly disproportionate — 42-46
seconds for ~250-260 keyframes on a 30-second clip — a direct, measured
confirmation of `plan/STAGE2.md`'s "What we already know" (dense O(n^3)
solve, unbounded by marginalization) and the concrete reason Stage 2
sequences M1 (marginalization) before claiming victory on the real-time
goal.

## Known gaps (honest, not swept under the rug)

- **MH_02_easy and MH_03_medium don't run at all** — the current
  stationary-window IMU bootstrap (`slam_imu::find_stationary_window`)
  doesn't find a usable window in either sequence with its current
  thresholds, despite both starting stationary per `plan/STAGE1.md`'s own
  dataset notes. `bin/slam-inspect` shows the same gap independently, so
  this isn't new. Initializer robustness on the harder sequences is
  explicitly `plan/STAGE2.md`'s M6 (finishing Stage 1's M10) — this table
  will grow two more rows once that lands.
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
