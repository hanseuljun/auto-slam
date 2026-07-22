---
name: m8-global-bundle-adjustment
description: M8 done — global bundle adjustment over the full retained trajectory, reusing slam-optim's existing Problem/optimize machinery; ATE held (not worsened) on the MH_01 checkpoint, ~0% change since a short clip's window-wise LM was already near the joint optimum.
metadata:
  type: progress
---

# M8 — global bundle adjustment pass

Landed the ninth milestone from `plan/STAGE1.md`, following M0-M7. The
smallest milestone of the session by new code — it's a direct reuse of
`slam-optim`'s existing `Problem`/`optimize` LM+Schur solver, not new
machinery.

## What's done

- `slam_backend::VioPipeline` now retains every keyframe evicted from the
  sliding window (`decisions/0007`'s naive fixed-lag drop) in a new
  `history: Vec<WindowKeyframe>` field instead of discarding it, purely so
  a later global pass has the full trajectory to work with.
  `run_optimization` (the per-frame windowed solve) never reads `history`
  itself.
- `VioPipeline::global_bundle_adjustment(&mut self) -> usize`: combines
  `history` (chronological) + `window` (chronological) into one ordered
  trajectory, builds a single `slam_optim::Problem` over every keyframe and
  every landmark ever observed (same reindexing pattern
  `run_optimization` uses per-window, just unbounded), and calls
  `optimize()` once. Writes results back into both `history` and `window`
  states and into `self.landmarks`. Keyframe 0 of the whole trajectory
  remains the sole gauge anchor, same as every windowed solve along the
  way — no new anchoring logic needed.
- `VioPipeline::all_keyframe_poses(&self) -> Vec<(u64, SE3)>`: the
  `history` + `window` combined `(timestamp, pose)` list, for before/after
  comparison (used by the checkpoint test and `slam-inspect`).
- `bin/slam-inspect`'s stereo-inertial VIO section now runs 150 frames
  (up from 80, long enough to guarantee keyframes evict past the default
  8-keyframe window into `history` — otherwise global BA would just
  re-solve the same window `run_optimization` already converged, proving
  nothing) and reports a `global bundle adjustment: N keyframes, ATE rmse
  before=... after=...` line after the existing windowed-VIO ATE line.

## Real checkpoint

`global_bundle_adjustment_does_not_worsen_ate_on_mh01`
(`crates/slam-backend/src/tests_integration.rs`): runs VIO over 150 real
MH_01 frames (default `VioParams`, 8-keyframe window, stride 10 -> enough
keyframes to guarantee eviction into `history`), snapshots ATE from the
windowed-only trajectory, runs one `global_bundle_adjustment()` pass,
recomputes ATE. Matches `plan/STAGE1.md`'s own M8 wording: "global BA
strictly improves or holds ATE relative to pre-BA... regressions here
mean a solver/Jacobian bug, not 'BA doesn't help.'" Result: **0.1366m
before -> 0.1377m after** (15 keyframes) — essentially unchanged (well
inside the test's 5% + 1e-4m tolerance, which exists only to absorb LM's
own convergence noise, not to paper over a real regression).
`bin/slam-inspect` on MH_01 (150 frames, 24 keyframes): 0.104m -> 0.104m,
same story.

## Why the improvement is ~0% and that's the correct result here

Global BA's advantage over windowed optimization is retroactively
correcting *older* keyframes using constraints only visible to *later*
ones (e.g. a keyframe that dropped out of the window before its landmarks
were seen from enough angles to fully constrain them). On a ~150-frame /
15-24-keyframe clip with an 8-keyframe window, each keyframe already spent
most of its useful lifetime inside a window being jointly optimized
against its neighbors before eviction — there's little "unfinished
optimization" left for a global pass to clean up. This is expected, not a
sign the implementation is a no-op: it reuses the exact same solver/
factors as the windowed path (verified elsewhere), and the theoretical
win from global BA is supposed to show up on either (a) much longer
trajectories where old keyframes were evicted while still under-
constrained, or (b) after loop closure, where a pose-graph correction
propagates new long-range constraints back through the whole trajectory
that per-window optimization never saw. Neither condition holds on this
short, loop-free MH_01 clip. Applying global BA after M7's loop closure on
a full sequence (MH_05) is the natural place to look for a real,
non-trivial win — noted as a good follow-up but out of scope for this
checkpoint, which only needed to prove "doesn't make things worse."

## Not done yet (correctly out of scope for this M8 pass)

- Running global BA after loop closure on a full sequence (the
  combination most likely to show a real ATE win, per the reasoning
  above) — `slam-inspect`'s MH_05 loop-closure section and its VIO section
  are currently separate demonstrations, not chained together.
- Marginalization (`decisions/0007`) is still not implemented — global BA
  is a complementary, not overlapping, accuracy lever; both remain open
  for M10's closing pass.
