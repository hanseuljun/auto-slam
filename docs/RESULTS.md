# Results: this repo vs. published stereo-inertial SLAM, and real-time factor

`plan/STAGE1.md`'s accuracy target: "roughly 2-9cm ATE RMSE... in the same
ballpark as the published SOTA systems." `plan/STAGE2.md`'s goal 1: a
real-time factor <= 1.0 for the continuous VIO loop. This is the M9/M0
deliverable that checks both against real, reproducible numbers instead
of narrative progress notes.

## How to reproduce

```
cargo run --release --bin slam-run                    # all 5 sequences, full un-truncated (Stage 4 M3 default; ~10 min total)
cargo run --release --bin slam-run -- --frames 600    # bounded/fast-iteration mode (~30s of data each)
cargo run --release --bin slam-run -- data/machine_hall/MH_01_easy  # one sequence, full
```

Runs the full stereo-inertial VIO pipeline (Stage 1 M5, track-loss
recovery M6), one global bundle-adjustment pass (M8), and — as of `plan/
STAGE5.md` M2 — loop closure (M7), gated on a real geometric check (see
"Loop closure and honest ATE (Stage 5)" below). Prints ATE (both the
legacy whole-trajectory-aligned number and the honest prefix-aligned
one, `plan/STAGE5.md` M1)/RPE and a wall-clock timing breakdown per
sequence, and writes
`runs/<sequence>/trajectory.csv` (per-timestamp estimated vs. groundtruth
position, always the latest run) plus `runs/summary.csv` (the aggregate
table these tables are generated from) — same as always.

**Since Stage 3's M0**, each invocation additionally writes a non-
clobbering history entry per sequence at
`runs/<sequence>/<run_id>/{trajectory.csv, meta.json}` (`run_id` a
sortable `YYYYMMDD-HHMMSS-mmm` timestamp) — `meta.json` carries the ATE/
RPE/timing numbers plus the exact `VioParams`/`SolverConfig` values and
git commit that run used, so re-running `slam-run` while tuning (as
`memory/decisions/0016`-`0017` did) no longer overwrites the previous
attempt's numbers. This is additive: the `runs/<sequence>/trajectory.csv`
and `runs/summary.csv` paths above are unchanged and still reflect the
latest run. `plan/STAGE3.md`'s `bin/slam-viz` (goal 3, not yet built)
will browse this history.

**As of Stage 4 M3, `bin/slam-run`'s default is the full, un-truncated
sequence** — see "Full-sequence results" below for those numbers, now
what a plain `cargo run --release --bin slam-run` (no flags) actually
produces. **The tables immediately below (up to "Full-sequence
results") are from the bounded/fast-iteration mode instead** (`--frames
600`, ~30s of data per sequence) — kept as an explicit opt-in for quick
tuning turnaround (`decisions/0016`-`0017`'s sweeps used exactly this),
not the default any more. An earlier attempt at this exact milestone ran
the un-truncated pipeline over one full sequence and took 30+ minutes
wall-clock before being rolled back; see `memory/decisions/0011` for the
determinism bug found and fixed since then. Global BA's O(n^3) scaling
over the *windowed* solver (`plan/STAGE2.md`'s original "What we already
know") is bounded by Stage 2 M1's real marginalization, but
`global_bundle_adjustment` itself was not — it kept the same O(n^3)
scaling over literal unbounded history until Stage 4 M1 fixed it; see
"Full-sequence results" below for the real, measured before/after. All
numbers below are reproducible bit-for-bit run to run
(`decisions/0011`'s fix).

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
the measured per-sequence numbers. Loosening it also shifted *which*
stationary window MH_01_easy itself uses (an earlier-but-still-valid one
now qualifies too), which is why MH_01's own number moved slightly even
though nothing about MH_01's pipeline changed — every number below is
freshly re-measured at the same current code state, not assembled from
runs at different points in time.

Stage 2 M6 also tried deriving `SolverConfig`'s noise weights from
`sensor.yaml`'s real densities instead of the ad hoc `Default` values —
measured on real data at two scopes, both regressed accuracy on most
sequences, reverted (`decisions/0016`). The numbers below use the
original ad hoc weights.

**Stage 2 M6 concluded** after also sweeping the outlier-gating (Huber)
threshold and `window_size` in both directions (`decisions/0017`) —
every direction tried regresses at least one sequence (most
consistently MH_05) for only small, inconsistent gains elsewhere, so
none meets M6's "improvement on every sequence" bar. The numbers below
are the final M6 numbers: same as M1's, since every M6 tuning attempt
(noise weighting, window size, Huber threshold) was reverted except the
MH_02/03 bootstrap fix, which is already reflected here.

## ATE RMSE (meters), bounded 600-frame (~30s) clips (`--frames 600`), no loop closure

| Sequence | This repo (M5+M8+M1, ~30s clip) | ORB-SLAM3 (full seq.) | OKVIS (full seq.) | VINS-Fusion (full seq.) | Kimera (full seq.) |
|---|---|---|---|---|---|
| MH_01_easy | 0.151 | 0.036 | 0.079 | 0.166 | 0.080 |
| MH_02_easy | 0.184 | 0.033 | 0.044 | 0.152 | 0.090 |
| MH_03_medium | 0.511 | 0.035 | 0.096 | 0.125 | 0.110 |
| MH_04_difficult | 1.174 | 0.051 | 0.197 | 0.280 | 0.150 |
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
| MH_01_easy | 14.8 | 2.8 | **0.589** | 2.7 |
| MH_02_easy | 13.6 | 2.6 | **0.540** | 2.6 |
| MH_03_medium | 14.6 | 2.8 | **0.578** | 2.1 |
| MH_04_difficult | 10.9 | 1.5 | **0.412** | 1.8 |
| MH_05_difficult | 13.1 | 2.4 | **0.518** | 2.1 |

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

## Full-sequence results (Stage 4 M0-M3 — now the default)

**These numbers predate `plan/STAGE5.md`'s loop closure (M2) and honest
prefix-aligned ATE (M1) — see "Loop closure and honest ATE (Stage 5)"
below for what a plain `cargo run --release --bin slam-run` actually
produces today.** Kept here as the Stage 4 historical record (real-time
fix, before loop closure existed).

`plan/STAGE4.md`'s goal: make a full, un-truncated run (not just the
bounded 600-frame clip above) both real-time and no worse on accuracy,
then make it the default (M3) — done; at the time, a plain `cargo run
--release --bin slam-run` (no flags) produced the numbers below. Measured
on all 5 sequences, foreground (background execution of a multi-minute
run proved unreliable this session — see `memory/progress/2026-07-23-
stage4-m0-mh01-full-sequence-measured.md`).

**Before the M1 fix**: `MH_01_easy` alone was profiled live (macOS
`sample`) mid-run — 100% of sampled stack frames were inside
`global_bundle_adjustment`'s dense LU solve, which took **957.2s** on
that sequence's 741 keyframes (`(741-1)*15=11100`-dimensional dense
system, `O(dim^3)`). This confirms `plan/STAGE2.md`'s own Risks section,
written in advance: *"a truncated clip that happens to fit inside the
window can look real-time for reasons that have nothing to do with
actually fixing the scaling."* `global_bundle_adjustment` was never
bounded by Stage 2 M1's marginalization (that only bounds the
*windowed* solver) — it still built its `Problem` from every keyframe
ever created.

**The fix (`plan/STAGE4.md` M1)**: `VioParams::max_global_ba_keyframes`
(default 150) caps global BA to the most recent N keyframes instead of
literal unbounded history — no new linear algebra, reuses the existing,
already-tested `Problem`/`optimize` machinery, just bounds what goes
into it. All 5 sequences, full un-truncated, after the fix:

| Sequence | keyframes | track-loss recoveries | vision (s) | optimization (s) | global BA (s) | total wall-clock (s) | data (s) | whole-run factor | ATE full (m) | ATE bounded clip (m) |
|---|---|---|---|---|---|---|---|---|---|---|
| MH_01_easy | 741 | 382 (51.6%) | 115.2 | 26.8 | 7.8 | 149.8 | 184.0 | **0.814** | 3.868 | 0.151 |
| MH_02_easy | 552 | 255 (46.2%) | 79.3 | 18.5 | 7.7 | 105.5 | 152.0 | **0.694** | 3.854 | 0.184 |
| MH_03_medium | 536 | 270 (50.4%) | 71.2 | 16.2 | 7.4 | 94.8 | 134.9 | **0.702** | 3.460 | 0.511 |
| MH_04_difficult | 364 | 164 (45.1%) | 37.9 | 5.2 | 7.0 | 50.1 | 101.6 | **0.493** | 6.600 | 1.174 |
| MH_05_difficult | 456 | 230 (50.4%) | 54.6 | 12.1 | 7.4 | 74.1 | 113.6 | **0.652** | 6.818 | 0.455 |

"whole-run factor" = `(vision + optimization + global_ba) /
data_seconds` — redefined for Stage 4's goal 2 to count *everything*,
not just the per-frame loop (the old `real_time_factor()` metric
excludes global BA by design, which is exactly what let this gap hide:
it reported 0.686 for `MH_01_easy` even at 957s of global-BA cost).
**Goal 2 (real-time on the full sequence) is now met on every sequence**
— global BA's cost dropped ~120x (957.2s -> 7.8s on `MH_01_easy`) and
is now roughly flat (~7-8s) regardless of sequence length, since it no
longer scales with total keyframe count.

"track-loss recoveries" (`bin/slam-run` now counts and reports these,
Stage 4 M2 — previously only "unrecoverable single frames," always 0,
was reported) are keyframes forced by too-few-surviving-LK-tracks rather
than the usual stride, using IMU-only propagation with a reset local map
(`plan/STAGE1.md` M6). **45-52% of all keyframes on every sequence are
recoveries** — investigated as M2's leading candidate for the accuracy
gap below, but ruled out as the *differentiating* cause: the bounded
600-frame clip shows the same rate (MH_01's own bounded clip: 47 of 106
keyframes, 44.3%), so this is a pervasive pipeline characteristic at
both scales, not something that newly appears or worsens on full
sequences. Real, and worth a future stage's attention as a frontend-
robustness gap, but not what explains why full-sequence ATE is worse
than the bounded clip's.

**Goal 3 (accuracy) is met — the gap vs. the bounded clip is confirmed
natural full-sequence drift, not a regression (`plan/STAGE4.md` M2).**
Full-sequence ATE is worse than the bounded clip on every sequence
(5.6x-25.6x), confirmed *not* caused by the M1 fix itself
(`MH_01_easy`'s ATE was 3.869m unbounded vs. 3.868m bounded-scope,
essentially identical). The right question wasn't "is this multiple too
big" in the abstract — it was whether full-sequence ATE is worse than
this pipeline's own already-known, already-documented full-sequence
drift, or in line with it. It's in line with it: `plan/STAGE1.md` M6
(`memory/progress/2026-07-21-m6-robust-tracking-and-full-sequence-runs.md`)
already measured full-sequence ATE for the pure-VO pipeline (no IMU
fusion, no windowed backend, no global BA at all) over a year-old
codebase revision, and documented multi-meter drift there as *"expected,
not a regression... this is what no-loop-closure full-sequence flight
looks like."* Re-run fresh against the current code
(`full_sequence_runs_survive_all_five_sequences_without_permanent_loss`,
`crates/slam-frontend/src/lib.rs`, `#[ignore]`d, 2026-07-23) to get an
apples-to-apples current-code baseline:

| Sequence | VO-only full ATE (m), no IMU/backend/BA | VIO full ATE (m), M0/M1 table above | Δ |
|---|---|---|---|
| MH_01_easy | 3.389 | 3.868 | +14% |
| MH_02_easy | 3.872 | 3.854 | -0.5% |
| MH_03_medium | 3.410 | 3.460 | +1.5% |
| MH_04_difficult | 6.533 | 6.600 | +1.0% |
| MH_05_difficult | 5.615 | 6.818 | +21% |

Full VIO lands within ~20% of a completely independent, previously-
documented baseline on every sequence (matching almost exactly on 3 of
5) — despite the two pipelines sharing almost no code path beyond the
frontend (VO-only has no IMU factors, no windowed marginalized backend,
no global BA pass at all). Both are dominated by the same structural
cause: no loop closure means multi-minute flights accumulate multi-meter
drift, and no amount of windowed/global optimization corrects an error
that has no absolute reference to correct against. This cross-validation
— an independent measurement, from a different stage, using a different
code path, landing in the same range — is what "explainable by natural
drift-over-time, not a bug-shaped regression" (`plan/STAGE4.md` M2's own
bar) actually requires; a plausible-sounding story alone wouldn't have
been enough. No fix needed or applied; M2 closes with this finding, not
a code change.

## Loop closure and honest ATE (Stage 5)

`plan/STAGE5.md`'s two goals: (1) ATE should be near zero close to a
trajectory's start, since there's no time to have drifted yet — the
existing whole-trajectory Umeyama alignment let later drift mask this;
(2) sequences that return to their own starting point (confirmed true of
all 5 `machine_hall` sequences via groundtruth) should show real evidence
of loop closure, not stay at high ATE regardless. **Both are done, and
together they're what a plain `cargo run --release --bin slam-run` (no
flags) actually produces today** — this supersedes the "Full-sequence
results" table above, which predates both.

**Goal 1 (M0/M1, `memory/decisions/0020`)**: `crates/slam-eval` gained
`compute_ate_prefix_aligned` — the Umeyama fit uses only the first 30
seconds of data instead of the whole trajectory, so a run's own later
drift can't pull the alignment and mask how accurate the early portion
really was. Confirmed directly (not just via the aggregate number):
`MH_01_easy`'s per-point error at its very first keyframe is 0.185m
under this metric, vs. 3.1m under the old whole-trajectory fit for the
same underlying (nearly identical) pose estimate.

**Goal 2 (M2, `memory/decisions/0021`)**: loop closure (`plan/STAGE1.md`
M7) is now chained into `bin/slam-run`'s actual pipeline, on every
sequence, gated on a real geometric check — a detected/verified loop's
correction is only kept if it verifiably shrinks the trajectory's own
start/end position gap (found necessary the hard way: applying a
verified correction unconditionally regressed `MH_01_easy` during this
milestone's own baseline check). Also had to fix a real regression of
Stage 4's own real-time work along the way: naively running pose-graph
optimization over every dense VIO keyframe reintroduced the exact dense
O(n^3) scaling bug `plan/STAGE4.md` M1 already closed for global BA —
fixed with a sparse pose graph plus smooth SE3-interpolated propagation
onto the dense trajectory, which has its own real, measured RPE cost
(see the table below and `memory/decisions/0021` for the full tradeoff
this session swept, including why a denser/higher-quality alternative
was rejected for breaking the real-time bar).

All 5 sequences, full un-truncated runs, before (Stage 4 M0-M3's own
numbers) vs. after (Stage 5 M1+M2 applied):

| Sequence | whole-traj. ATE before -> after | prefix-aligned ATE before -> after | RPE d=1 before -> after | start/end gap before -> after |
|---|---|---|---|---|
| MH_01_easy | 3.868m -> 3.505m | 5.412m -> 6.893m | 0.162m -> 0.863m | 299.2m -> 145.4m |
| MH_02_easy | 3.854m -> 3.546m | 7.787m -> 7.548m | 0.218m -> 1.266m | 58.7m -> 12.2m |
| MH_03_medium | 3.460m -> 3.451m | 17.180m -> 9.844m | 0.378m -> 0.892m | 71.1m -> 32.5m |
| MH_04_difficult | 6.600m -> 6.496m | 9.689m -> 7.681m | 0.398m -> 1.137m | 32.5m -> 14.5m |
| MH_05_difficult | 6.818m -> 6.596m | 12.945m -> 7.537m | 0.322m -> 1.293m | 145.8m -> 81.4m |

**The geometric claim (goal 2's own bar) holds on every sequence**: a
real loop is detected, verified, and its correction accepted by the
gate — the trajectory's own start/end gap shrinks 2x-4.8x everywhere,
in this pipeline's own (ungrounded) world-frame units. Whole-trajectory
ATE improves slightly on every sequence too. The honest, prefix-aligned
ATE improves substantially on 4 of 5 (MH_03 nearly halves, MH_05 nearly
halves, MH_04 and MH_02 both improve) but gets *worse* on `MH_01_easy`
specifically (5.412m -> 6.893m) despite its own loop closure applying
successfully — an open, honestly-reported finding, not yet fully
explained (`MH_01`'s own correction is the largest in absolute magnitude
of the five, and its post-correction residual gap the largest too,
plausibly related, not confirmed). **RPE consistently degrades on every
sequence** (the interpolated-propagation cost documented above) — real,
open, and worth a future stage's attention (a genuinely sparse pose-graph
solver, rather than this session's stride-based workaround, is the
likely fix, matching the same "sparse solve" deferral pattern this
repo's other dense-solver call sites have already taken).

## Known gaps (honest, not swept under the rug)

- **Not apples-to-apples with the published numbers on two axes**: (1)
  this repo's numbers are a 30-second bounded clip, not a full sequence
  (~100-180s) — ATE over a shorter clip has less time to accumulate drift,
  so this is not a favorable comparison to read too much into either
  direction — and this table's own bounded-clip numbers don't include
  loop closure either (`--frames 600`'s ~30s clip is too short for any
  of the loops `plan/STAGE5.md` Finding 2 found — those only close at a
  sequence's very end); (2) the published systems' own numbers use a
  fixed-scale (not free-scale) alignment, appropriate since stereo-
  inertial scale should be directly observable — this repo's numbers
  above use a free-scale (Sim3) alignment throughout, which `plan/
  STAGE5.md` M0 found is currently masking a real, unexplained scale
  characteristic in this pipeline's own reconstruction (`memory/
  decisions/0020`) — not yet resolved, so this comparison may be more
  favorable to this repo than a strictly fair one would be.
- **IMU noise weighting is still ad hoc**, not derived from `sensor.yaml`'s
  real covariances (`memory/decisions/0006`) — tried twice now and
  reverted both times: a simpler per-residual-type formula in Stage 2 M6
  (`decisions/0016`), and full nonlinear preintegration covariance
  propagation (Forster et al.'s own recursion, correctly implemented and
  Monte-Carlo-validated) as a per-factor information matrix in Stage 6 M2
  (`memory/decisions/0024`) — both measurably regressed real data (M2's
  effect was larger: up to +101% bounded-clip ATE on one sequence).
  Root cause understood, not just observed: EuRoC's real noise densities
  make a short IMU interval look 30-166x more trustworthy than the ad hoc
  scalars this repo keeps using, but this pipeline needs vision to
  correct IMU-only drift more often than that implies, so "physically
  correct in isolation" isn't "more accurate here." Not a sign the
  weighting math is wrong — a real, twice-confirmed property of this
  specific pipeline.
- **The remaining ad hoc knobs (Huber threshold, `window_size`) are
  at a local optimum for the current pipeline, not a global one** —
  M6 swept both in both directions (`memory/decisions/0017`) and every
  direction regressed at least one sequence, most consistently MH_05.
  Further accuracy gains likely need the same structural work as the
  noise-weighting gap above (analytic IMU Jacobians, real preintegration
  covariance) rather than more scalar sweeps.
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
