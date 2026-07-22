---
name: stage2-m5-real-time-bar-met
description: Stage 2 M5 done — real-time factor <=1.0 confirmed on every runnable sequence, met by M1's marginalization/PnP-guard fixes alone. M2-M4 (analytic Jacobians, sparse solve, rayon parallelism) re-scoped from required to deferred in plan/STAGE2.md as a result.
metadata:
  type: progress
---

# Stage 2 M5 — real-time validation (met early, via M1)

Not a separately-implemented milestone — a measurement that landed as a
direct consequence of M1's bug fixes, discovered by re-running M0's
benchmarking harness after M1 rather than assuming more work (M2-M4) was
still needed before checking.

## What happened

After Stage 2 M1 landed (real marginalization + the PnP pose-jump fixes,
`decisions/0012`-`0014`), re-running `bin/slam-run` across every runnable
sequence showed the real-time factor had dropped far below the 1.0 bar on
all three:

| Sequence | Before M1 | After M1 |
|---|---|---|
| MH_01_easy | 1.198 | **0.543** |
| MH_04_difficult | 0.357 | **0.398** (was already under) |
| MH_05_difficult | 1.086 | **0.523** |

This wasn't the plan's expected path — `plan/STAGE2.md` originally
scoped M2 (analytic IMU Jacobians), M3 (sparse solve), and M4 (`rayon`
parallelism) as the levers to pull before M5 could pass. Instead, fixing
the accuracy bugs in M1 fixed the speed problem too: the PnP corruption
`decisions/0014` fixed had been triggering cascades of track-loss
recovery, and each spurious recovery keyframe cost a full extra round of
stereo matching/landmark detection (the actual dominant cost, not the
IMU factor's numerical Jacobian or the dense solver M2/M3 targeted).
Removing the corruption removed most of the wasted work along with it.

## Why this mattered for the plan, not just the code

Per `CLAUDE.md`'s instruction to update the plan itself when it changes,
not just the code: `plan/STAGE2.md` now marks M2-M4 "Deferred, not
required" rather than quietly skipping them or leaving the plan
inaccurate. This is a genuine instance of the project's own stated
discipline ("if it isn't [helping], that's worth understanding before
moving on") — verified by actually re-running the harness after M1,
not assumed from the original plan's reasoning about where the cost
"should" be.

## State at end of session

Stage 2's M0, M1, M5 are done; M2-M4 deferred (not abandoned — their
original scope is preserved in `plan/STAGE2.md` in case a future
profiling pass or a much larger window size ever needs them); M6
(finishing Stage 1's M10, accuracy closing pass) is next.
