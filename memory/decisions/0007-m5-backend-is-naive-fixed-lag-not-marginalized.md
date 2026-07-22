---
name: m5-backend-is-naive-fixed-lag-not-marginalized
description: slam-backend's first working sliding window drops the oldest keyframe outright (naive fixed-lag) instead of marginalizing it into a prior — explicitly the inferior baseline plan/STAGE1.md contrasts marginalization against, scoped this way to get a working M5 checkpoint first.
metadata:
  type: decision
---

# Decision: M5's sliding window is naive fixed-lag, not marginalized (yet)

## Decision

`slam_backend::VioPipeline` drops the oldest keyframe outright when the
window exceeds `window_size` (`crates/slam-backend/src/vio.rs`), along
with its incoming IMU edge and its unique landmark observations. No
Schur-complement marginalization prior is computed and carried forward.

## Why

`plan/STAGE1.md` M5 itself frames this precisely: "marginalize the oldest
keyframe into a prior ... instead of dropping it, to retain information
(**this is the single biggest accuracy lever versus a naive fixed-lag
window**)" — i.e., the plan's own language treats naive fixed-lag windowing
as a real, describable (if inferior) baseline, not a disqualifying
omission. Given the size of M5's other required pieces landing in this
same session (the LM solver + Schur-complement-over-landmarks itself, the
reprojection/IMU/bias-random-walk factors, the sliding-window manager),
implementing full marginalization in the same pass risked ending the
session with *nothing* working end-to-end. Staging it — naive window
first, verified against real data, marginalization as a documented
follow-up — matches this repo's established pattern (`decisions/0003`,
`0004`, `0005`: land a working, tested increment, defer the harder
generalization to when there's a concrete need/consumer to validate
against).

## Real-data result with this scope

`vio_ate_on_mh01_is_competitive_with_vo_only`
(`crates/slam-backend/src/tests_integration.rs`) and `slam-inspect`'s
"stereo-inertial VIO" section: MH_01 ATE ~0.11-0.14m over a handful of
keyframes, essentially matching (not clearly beating) M3's VO-only
~0.137m. This is consistent with a correct-but-unrefined system: no bug
found, but marginalization (this decision), ad hoc noise weights
(`decisions/0006`, `SolverConfig`'s doc comment), and no real
accelerometer bias estimation (`decisions/0005`) are all still open, and
any one of them could plausibly be why IMU fusion isn't yet clearly
outperforming stereo VO alone the way `plan/STAGE1.md`'s SOTA target
implies it eventually should.

## How to apply

When implementing real marginalization (still M5's remaining scope, or
pushed into M6 if time-boxed similarly to this session): the Schur-
complement machinery to do it already exists in `slam-optim`'s
`build_normal_equations` (landmark elimination) — marginalizing a
*keyframe* is the same idea one level up (eliminate the departing
keyframe's state variables via Schur complement against everything it's
connected to, producing a dense prior factor over the neighboring
keyframe's 15-dim state, added back into future `Problem`s). Don't
re-derive this from scratch; generalize what's already there. Re-run
`vio_ate_on_mh01_is_competitive_with_vo_only` before/after to confirm
marginalization actually improves ATE, not just "runs without crashing" —
per the plan's own framing, it should be a real accuracy lever, and if it
isn't, that's worth understanding before moving on.

## Closed (Stage 2 M1)

Real marginalization landed in `plan/STAGE2.md`'s M1 —
`slam_optim::marginalize_keyframe` + `slam_backend::VioPipeline::
marginalize_evicted_keyframe`. On the real-data checkpoint this
session's clip didn't show a large accuracy win (consistent with M8's
own earlier finding that short clips leave little for an information-
retention pass to clean up), but it's now real, not naive-drop, and
didn't regress. See `memory/progress/2026-07-21-stage2-m1-
marginalization.md` and `decisions/0012`-`0014` for the (substantial)
real-data debugging story getting it safe.
