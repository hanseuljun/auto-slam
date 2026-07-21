---
name: umeyama-ate-brought-forward-to-m3
description: Umeyama Sim3 alignment + ATE RMSE landed in slam-eval at M3, not M9, because M3's own checkpoint test needs it.
metadata:
  type: decision
---

# Decision: implement Umeyama alignment + ATE in slam-eval now (M3), not M9

## Decision

`crates/slam-eval/src/align.rs` (`umeyama_alignment`, `compute_ate`) landed
during M3, not M9 as `plan/STAGE1.md`'s milestone breakdown literally
assigns "Umeyama Sim3/SE3 alignment... ATE" to. `decisions/0001` (see
implications section) had also flagged Sim3 as an M9-only concern for
`slam-core`.

## Why

M3's own spec says: "run stereo VO-only (no IMU yet) on MH_01_easy, compute
ATE against groundtruth after Sim3 alignment — this is the first
end-to-end accuracy checkpoint." M3 cannot be verified without alignment +
ATE existing already. M9 is still real work on top of this — RPE at
multiple deltas, per-sequence/aggregate reports, CSV export, and the
published-SOTA comparison table — none of which exist yet. This decision
only concerns the two primitives M3 needs directly.

## Placement

Lives in `slam-eval` (not `slam-core`) since it's evaluation logic, not a
general Lie-group primitive — `Sim3Alignment` here is a plain
`{scale, rotation: Matrix3, translation: Vector3}` struct, not a
`slam_core::Sim3` manifold type with its own exp/log. If a future milestone
needs Sim3 *as an optimization variable* (e.g. monocular-style scale drift
correction in pose-graph optimization — Stage 1 doesn't need this per
`decisions/0001`, since stereo fixes scale), that would still be a new
`slam_core::Sim3` built when that consumer exists, following the same
build-when-needed pattern as `decisions/0003`.

## How to apply

M9's job is no longer "invent alignment/ATE from scratch" — it's "extend
`align.rs` with RPE, build the per-sequence/aggregate report format, wire
up `bin/slam-run`, and produce the published-comparison table." Don't
re-derive Umeyama; `compute_ate`/`umeyama_alignment` are already tested
against a known synthetic similarity transform.
