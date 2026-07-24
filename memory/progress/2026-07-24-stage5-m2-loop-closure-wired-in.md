---
name: stage5-m2-loop-closure-wired-in
description: Stage 5 M2 done — bin/slam-run now detects, verifies, and applies loop closure on every sequence (not just a MH_05 VO-only demo), gated on a real geometric check (start/end gap must verifiably shrink) rather than trusting descriptor-match confidence alone. Hit and fixed a real regression of Stage 4's own real-time fix along the way (pose-graph optimization over dense keyframes reintroduced the exact O(n^3) scaling bug M1 already closed for global BA); fixed via a sparse graph + smooth SE3-interpolated propagation, which has its own real, documented RPE cost.
metadata:
  type: progress
---

# Stage 5 M2: loop closure wired into the real pipeline

Implements `plan/STAGE5.md` goal 2. Three real findings along the way,
each with a corresponding fix — full numbers and reasoning in `memory/
decisions/0021`; this note is the shorter narrative.

## What changed

- `bin/slam-run/src/main.rs`: captures a `slam_loopclosure::LoopKeyframe`
  every `LOOP_CLOSURE_CAPTURE_STRIDE`-th (4) VIO keyframe during the
  existing per-frame loop (reuses already-loaded images, no new I/O).
  After `global_bundle_adjustment`, rebases each captured keyframe's
  landmarks to whatever pose global BA settled on (a real correctness
  concern for keyframes within its bounded scope, not just a
  formality — `rebase_loop_keyframe_landmarks`), then runs detection
  (BoW vocabulary + `KeyframeDatabase` query) + geometric verification +
  pose-graph optimization as a post-processing pass, and propagates the
  result onto the dense trajectory before it feeds ATE/RPE/`trajectory
  .csv` — not a side calculation like `bin/slam-inspect`'s older demo.
  `TimingBreakdown::loop_closure_seconds` is now real, not hardcoded 0.0.
- Added `slam-loopclosure` as a `bin/slam-run` dependency, `slam-vision`
  + `approx` as dev-dependencies (for the new unit tests).
- 4 new unit tests (`world_to_cam0`, `rebase_loop_keyframe_landmarks`
  round-trip/no-op checks) — `bin/slam-run` had none before; this
  session's own hand-derived SE3 composition math (world<->body<->cam0
  conversions, landmark rebasing) was exactly the kind of thing worth
  pinning down with a test, not just trusting by inspection.

## Three real findings, not assumed

1. **A verified loop correction isn't automatically a good one.**
   Baseline-checked the existing detector against all 5 sequences (not
   just `MH_05_difficult`) before building the real integration —
   detection worked everywhere, but applying the "most inliers" match
   unconditionally regressed `MH_01_easy` (VO-only baseline: ATE
   3.370m -> 3.907m). Fixed with a geometric gate: only keep a
   correction if it verifiably shrinks the trajectory's own start/end
   position gap (a direct, cheap, human-checkable claim — exactly what
   `plan/STAGE5.md` M3 asks for, built into the fix itself, not just
   the later verification).
2. **Pose-graph optimization over every dense VIO keyframe reintroduced
   Stage 4 M1's own bug.** First attempt (build the graph over all 741
   of `MH_01_easy`'s VIO keyframes) didn't finish in 10+ minutes —
   `optimize_pose_graph`'s dense, unbounded-scope O(n^3) LU solve is the
   *exact* scaling mistake `plan/STAGE4.md` M1 already root-caused and
   fixed for `global_bundle_adjustment`, reintroduced here by not
   applying the same lesson to a new call site. Fixed by running the
   graph over a sparse (stride-4) keyframe subset and propagating the
   correction onto the dense trajectory via smooth SE3 log-space
   interpolation between bracketing sparse nodes (a discrete nearest-
   node version was tried first and measurably worse — see next point).
3. **The interpolated propagation has a real, open RPE cost.** Swept the
   stride/cost/quality tradeoff directly: stride 4 costs ~23s and holds
   the real-time bar (whole-run factor 0.85) but RPE delta=1 rmse
   degrades ~5x (0.162m -> 0.863m on `MH_01_easy`); stride 2 roughly
   halves that RPE hit but costs 88s and *breaks* the real-time bar
   (whole-run factor 1.205). Kept stride 4 — the real-time bar is
   non-negotiable (this stage's own Risks section named this exact
   danger in advance) — and documented the RPE tradeoff as a real, open
   limitation rather than silently accepting or hiding it.

## Verified on all 5 sequences, full un-truncated runs

| Sequence | loop applied | inliers | start/end gap before -> after | whole-run factor |
|---|---|---|---|---|
| MH_01_easy | yes | 30 | 299.2m -> 145.4m | 0.846 |
| MH_02_easy | yes | 18 | 58.7m -> 12.2m | 0.787 |
| MH_03_medium | yes | 28 | 71.1m -> 32.5m | 0.798 |
| MH_04_difficult | yes | 15 | 32.5m -> 14.5m | 0.562 |
| MH_05_difficult | yes | 22 | 145.8m -> 81.4m | 0.740 |

A loop is detected, verified, and accepted by the gate on **every**
sequence — real-time bar holds throughout (0.56-0.85, all <=1.0).
Whole-trajectory ATE improved slightly everywhere; the honest prefix-
aligned ATE (`plan/STAGE5.md` M1) improved substantially on 4 of 5 but
got *worse* on `MH_01_easy` specifically (5.412m -> 6.893m) — flagged
for M3's own closer look, not swept under the rug.

`cargo test --workspace` and `cargo clippy --all-targets` both clean.

## What's next

`plan/STAGE5.md` M3: verify geometrically (already partly built into
M2's own gate, but M3's own bar is the full before/after ATE/RPE table
plus `docs/RESULTS.md`/`README.md` updates) and investigate why
`MH_01_easy`'s honest ATE got worse despite its loop closure applying
successfully — the largest raw correction of the five, but still the
largest absolute residual gap too.
