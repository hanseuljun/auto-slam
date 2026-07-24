---
name: stage5-m2-loop-closure-sparse-graph-and-geometric-gate
description: Stage 5 M2 decisions — three real findings while wiring loop closure into bin/slam-run's actual VIO pipeline. (1) Applying a detected/verified loop correction unconditionally can regress accuracy (found on MH_01_easy's VO-only baseline check) — fixed with a geometric gate that only accepts a correction if it verifiably shrinks the trajectory's own start/end gap. (2) Running pose-graph optimization over every dense VIO keyframe (up to 741) reintroduces the exact dense-O(n^3) scaling bug Stage 4 M1 already fixed for global BA — fixed by running the graph over a sparse, stride-4-downsampled keyframe set and propagating the correction onto the dense trajectory via smooth SE3 interpolation. (3) That interpolated propagation measurably degrades RPE (local frame-to-frame consistency) — a real, documented, open tradeoff against a denser stride that would fix it but breaks the real-time bar.
metadata:
  type: decision
---

# Stage 5 M2: wiring loop closure in without regressing what Stage 4 fixed

## Finding 1: a verified loop correction isn't automatically a good one

Baseline check (`plan/STAGE5.md` M2's own first bullet): ran the existing
`slam_loopclosure` detection+verification (previously only exercised on
`MH_05_difficult` via `bin/slam-inspect`'s `VoPipeline`-only demo)
against all 5 sequences. Detection succeeded everywhere, but applying
the "best by inlier count" candidate unconditionally gave mixed results:
`MH_01_easy`'s ATE went from 3.370m to 3.907m — the loop closure the
existing algorithm chose (a real, verified, 73-inlier match) made things
*worse*. `MH_03`/`MH_04`/`MH_05` improved; `MH_02` was flat.

**Decision**: never apply a correction blindly. `find_and_apply_loop_
closure` (`bin/slam-run/src/main.rs`) computes the trajectory's own
start/end position gap (in the pipeline's own ungrounded world frame,
not aligned to groundtruth) before and after the pose-graph correction,
and only keeps the correction if `gap_after < gap_before` — a direct,
cheap, human-checkable geometric claim (`plan/STAGE5.md` M3's own bar),
not just "a loop was detected and passed descriptor verification."
Candidate selection also changed from "most inliers anywhere" to
"largest keyframe-id gap" (spans the most of the trajectory) — a more
direct proxy for "connects this sequence's own start back to its own
end" (`plan/STAGE5.md` goal 2's actual framing) than raw match
confidence.

## Finding 2: pose-graph optimization over every dense keyframe reintroduces Stage 4's own bug

First working version captured a loop-closure keyframe at every VIO
keyframe (`is_keyframe == true`, up to 741 on `MH_01_easy`'s full run)
and ran `optimize_pose_graph` over all of them. Real-world result:
didn't finish in 10+ minutes (killed at 625s+ CPU time and still
running). Root cause: `crates/slam-loopclosure/src/pose_graph.rs`'s
`optimize_pose_graph` builds a dense `(n-1)*6`-dimensional Hessian and
does a dense LU solve *every LM iteration* (up to 50) — no sparsity
exploited, no bounded scope. At n=741, `dim=4440`, and a 4440-dim dense
solve × 50 iterations is the *exact* O(n^3)-over-unbounded-keyframe-count
scaling `plan/STAGE4.md` M1 already root-caused and fixed for
`global_bundle_adjustment` — reintroduced here, in a new call site, by
not applying the same lesson.

**Decision**: run the pose graph over a *sparse* set of loop-closure
keyframes (`LOOP_CLOSURE_CAPTURE_STRIDE`, captured every Nth VIO
keyframe, decoupled from VIO's own — track-loss-recovery-inflated —
keyframe cadence), not the dense trajectory. The sparse graph's
correction is then propagated onto the dense trajectory by *smoothly
interpolating* between the two bracketing sparse nodes' correction
deltas (SE3 log-space lerp — the same primitive `pose_graph.rs`'s own
`edge_residual` already uses), not applied as a single global transform
or a discrete nearest-node assignment (tried first, see Finding 3).

## Finding 3: the interpolated propagation has a real, measured cost — RPE

A discrete "nearest sparse node" propagation (simpler than interpolation)
was tried first and measurably injected a real discontinuity: on
`MH_01_easy`, RPE delta=1 rmse jumped from 0.162m (pre-loop-closure) to
1.104m — a ~7x degradation from artificial jumps at every stride
boundary. Switching to smooth SE3 interpolation between bracketing
sparse nodes reduced this to 0.863m — better, but still ~5x worse than
before. Swept the density/cost tradeoff directly rather than guessing:

| `LOOP_CLOSURE_CAPTURE_STRIDE` | sparse nodes | loop-closure cost | whole-run factor | RPE delta=1 rmse |
|---|---|---|---|---|
| 4 (chosen) | ~185 | 23.2s | **0.850** (holds) | 0.863m |
| 2 | ~370 | 88.7s | **1.205** (breaks) | 0.511m |

Denser sampling measurably halves the RPE hit but **breaks the
real-time bar** `plan/STAGE4.md` fought hard to establish — and this
stage's own Risks section named exactly this danger in advance ("this
could look similar to Stage 4's own 'flip a default before measuring'
wound if rushed"). **Decision**: keep stride 4. The real-time bar is
non-negotiable; the RPE cost is a real, open, documented limitation
(worth a future stage's attention — e.g. a genuinely sparse pose-graph
solver, matching the same "sparse solve" work `plan/STAGE2.md` M3 and
this repo's other dense-solver call sites have all similarly deferred,
not a sign this integration is broken), not something to silently
accept or silently hide.

## Measured on all 5 sequences, full un-truncated runs, after both fixes

| Sequence | loop applied | inliers | start/end gap before -> after | whole-run factor |
|---|---|---|---|---|
| MH_01_easy | yes | 30 | 299.2m -> 145.4m | 0.846 |
| MH_02_easy | yes | 18 | 58.7m -> 12.2m | 0.787 |
| MH_03_medium | yes | 28 | 71.1m -> 32.5m | 0.798 |
| MH_04_difficult | yes | 15 | 32.5m -> 14.5m | 0.562 |
| MH_05_difficult | yes | 22 | 145.8m -> 81.4m | 0.740 |

Every sequence: a loop is detected, verified, and the gate accepts the
correction — the raw start/end gap shrinks by roughly 2x-4.8x on every
one, and the real-time bar (`plan/STAGE4.md`'s own, whole-run factor
<=1.0) holds throughout (0.56-0.85). Whole-trajectory ATE (`compute_
ate`) improved slightly on every sequence too. The honest, prefix-
aligned ATE (`plan/STAGE5.md` M1) improved substantially on 4 of 5
(MH_03 17.180m->9.844m, MH_05 12.945m->7.537m, MH_04 9.689m->7.681m,
MH_02 7.787m->7.548m) but got *worse* on `MH_01_easy` specifically
(5.412m->6.893m) — worth `plan/STAGE5.md` M3's own closer look, since
`MH_01`'s own loop closure (299m->145m gap, the largest raw correction
of all five) still leaves the largest *absolute* residual gap too.
