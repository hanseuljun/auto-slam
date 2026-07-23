---
name: stage4-m1-global-ba-bounded-all-sequences-measured
description: Stage 4 M0+M1 done — global_bundle_adjustment now caps its scope at VioParams::max_global_ba_keyframes (150) instead of unbounded history. Confirmed on all 5 sequences: whole-run wall-clock now <=1.0x data duration (was up to 5.9x), ATE unaffected (confirmed via a real before/after on the same sequence). Full-sequence ATE regression vs. the bounded clip (5.6x-25.6x) is real and confirmed independent of this fix — M2's open item.
metadata:
  type: progress
---

# Stage 4 M0+M1: global BA bounded, all 5 sequences measured

## The fix

`crates/slam-backend/src/vio.rs`: added `VioParams::
max_global_ba_keyframes` (default 150). `global_bundle_adjustment_
inner` now includes only the most recent `max_global_ba_keyframes`
keyframes (history + window, in creation order) instead of literal
unbounded `history` — the confirmed root cause (`memory/progress/
2026-07-23-stage4-m0-mh01-full-sequence-measured.md`) of a ~957s
single-call cost on `MH_01_easy`'s 741-keyframe full sequence.

Deliberately reuses the existing `slam_optim::Problem`/`optimize`
machinery completely unchanged — no new linear algebra, no sparse
solver, no new numerical code. `Problem`'s own gauge-fixing convention
(keyframe *local* index 0 is the anchor, whatever's first in that
specific `Problem`) already generalizes to "the oldest included
keyframe," so bounding scope needed no protocol change on the solver
side — this is what kept the fix's correctness risk low despite the
real accuracy stakes (Stage 1/2's own repeated worry: a wrong Jacobian
or indexing bug still "converges," just to a silently worse answer).

One real new correctness risk, found by reading the code (not
guessing) and specifically guarded + tested: once the cap excludes the
true first keyframe, the new oldest-*included* keyframe still has a
real `imu_edge` pointing at a keyframe that's now excluded from this
`Problem`. The old unbounded loop's `if let Some((preint, dt)) =
&kf.imu_edge` check alone doesn't catch this — it would build an
`ImuFactorSpec { i: kf_idx - 1, ... }` with `kf_idx == 0`, an
underflow. Fixed with an explicit `kf_idx > 0` guard. New test
`global_bundle_adjustment_respects_max_global_ba_keyframes_cap`
(`crates/slam-backend/src/tests_integration.rs`) runs real MH_01 data
with a small cap (5) specifically to exercise this path — it would
panic, not just report a wrong number, if the guard were wrong or
missing, and separately checks keyframes older than the cap are
bit-for-bit untouched by the pass. Both this test and the pre-existing
`global_bundle_adjustment_does_not_worsen_ate_on_mh01` (whose own
150-frame scenario stays comfortably under the new 150-keyframe cap,
so its `assert_eq!(n, before.len())` still holds unchanged) pass.

## Measured results, all 5 sequences, full un-truncated

Before the fix, only `MH_01_easy` was measured (background attempts at
the other 4 weren't tried — see the linked M0 memory file for why:
each full run cost ~15-20 minutes at the unfixed cost, and the user
chose "fix first, then measure all 5 once" over measuring twice).
After the fix, all 5 measured for real, in the foreground:

| Sequence | keyframes | global BA (s) | whole-run factor | ATE full (m) | ATE bounded (m) | ratio |
|---|---|---|---|---|---|---|
| MH_01_easy | 741 | 7.8 | 0.814 | 3.868 | 0.151 | 25.6x |
| MH_02_easy | 552 | 7.7 | 0.694 | 3.854 | 0.184 | 20.9x |
| MH_03_medium | 536 | 7.4 | 0.702 | 3.460 | 0.511 | 6.8x |
| MH_04_difficult | 364 | 7.0 | 0.493 | 6.600 | 1.174 | 5.6x |
| MH_05_difficult | 456 | 7.4 | 0.652 | 6.818 | 0.455 | 15.0x |

"whole-run factor" = `(vision+optimization+global_ba)/data_seconds` —
the redefinition of Stage 4's goal 2 the user chose this session (the
old `real_time_factor()` metric excludes global BA by design, which is
exactly what hid this gap: it reported 0.686 on `MH_01_easy` even at
957s of global-BA cost). **Goal 2 (real-time on the full sequence) is
now met on every sequence**: global BA's cost dropped to a roughly
flat ~7-8s regardless of sequence length (previously scaled as O(n^3)
with keyframe count), and total wall-clock is now under the data
duration everywhere.

**Goal 3 (accuracy) is not yet met** — full-sequence ATE is 5.6x-25.6x
worse than the bounded clip on every sequence, a real, confirmed,
pre-existing gap (not caused by this fix — `MH_01_easy`'s own ATE
barely moved, 3.869m unbounded vs. 3.868m capped, measured on the
identical sequence before and after). `docs/RESULTS.md`'s "Full-
sequence results" section has the full table and framing; `plan/
STAGE4.md` M2 is the open item — this harness doesn't chain in loop
closure by design, which predicts *some* extra drift over a much
longer trajectory, but the magnitude (and the fact that global BA over
the *full* unbounded history, before this fix, still left ATE this
high) is worth real investigation before accepting "no loop closure"
as the whole story.

## Documentation updated

`docs/RESULTS.md` gained a new "Full-sequence results (Stage 4 M0/M1)"
section with the before/after table and framing; the stale "not
re-benchmarked with `--full` yet" caveat (Stage 2's own, now resolved)
was corrected. `plan/STAGE4.md`: M0 and M1 marked Done with full Result
notes; goal 2's text updated to record the whole-run-wall-clock
redefinition as a real decision (made via `AskUserQuestion`, not
assumed); M2 updated to state the regression is confirmed real and
still open, not hypothetical.

## What's next

`plan/STAGE4.md` M2 (root-cause the accuracy regression) is the next
open milestone — not started. M3 (flip `bin/slam-run`'s default to
full-sequence) is explicitly gated on M2 landing, per the plan's own
ordering and Stage 2's own origin-story risk ("this could reopen the
exact wound Stage 2 was created to close" — don't flip the default
before accuracy is actually confirmed sound, not just fast).
