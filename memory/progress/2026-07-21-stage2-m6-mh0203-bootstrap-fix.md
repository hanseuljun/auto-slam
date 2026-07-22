---
name: stage2-m6-mh0203-bootstrap-fix
description: Stage 2 M6 (accuracy closing pass, in progress) — first sub-step done, MH_02_easy and MH_03_medium now bootstrap and run instead of being skipped entirely, via a measured stationary-window threshold fix (decisions/0015).
metadata:
  type: progress
---

# Stage 2 M6 — accuracy closing pass (in progress): MH_02/MH_03 bootstrap fix

Started `plan/STAGE2.md`'s M6 (finishing Stage 1's M10) with the
highest-impact, most obviously-a-gap item first: two of five sequences
weren't producing *any* numbers at all, not even a bad ATE — `bin/slam-run`
and `bin/slam-inspect` both reported "no stationary window to bootstrap
from, skipping" for `MH_02_easy` and `MH_03_medium`.

## What happened

Measured (not guessed) each sequence's best-achievable 200-sample-window
max gyro norm by sliding a window over the real `imu0/data.csv` gyro
series:

| Sequence | best window max \|gyro\| (rad/s) |
|---|---|
| MH_01_easy | 0.088 |
| MH_02_easy | 0.093 |
| MH_03_medium | 0.090 |
| MH_04_difficult | 0.088 |
| MH_05_difficult | 0.086 |

`find_stationary_window`'s threshold (`max_gyro_norm`, used at every call
site as a literal `0.09`) was tuned tightly enough that MH_02 and MH_03
were *just* over it — not because they lack a genuinely stationary start
(they do, matching `plan/STAGE1.md`'s own dataset notes), but because the
ADIS16448's noise floor in those two specific recordings sits a hair
higher than in MH_01/04/05. Loosened to `0.10` (comfortable margin above
every sequence's actual best value) at all 7 call sites across
`slam-imu`, `slam-backend`, `bin/slam-inspect`, `bin/slam-run`.
`decisions/0015` has the full writeup.

## Real checkpoint

`bin/slam-run` on both sequences (600-frame bounded clips): MH_02_easy
ATE rmse=0.184m, real-time factor 0.541; MH_03_medium ATE rmse=0.511m,
real-time factor 0.615 — both plausible, both comfortably under the 1.0
real-time bar, both now producing real numbers instead of a skip message.
Full workspace test suite (all crates, including the three now-passing-
at-0.10 `find_stationary_window` call sites in tests) still green.
`docs/RESULTS.md` updated with both sequences' rows in the accuracy and
real-time-factor tables — this repo now has real, reproducible numbers
on all five `MH_*` sequences for the first time this session.

## State at end of session / what's left in M6

Per `plan/STAGE2.md`'s M6 scope, still open:

- Real `sensor.yaml`-derived noise weighting (replacing the ad hoc
  `SolverConfig` weights `decisions/0006` flagged) — the biggest
  remaining accuracy lever per the plan's own framing, not yet started.
- Outlier-gating threshold tuning.
- Keyframe/window sizing tuning.
- MH_04_difficult's own initializer robustness (it already runs via the
  dynamic vision-IMU alignment initializer, not the static one this
  session's fix touched — `decisions/0015` only affects the *static*
  bootstrap path, MH_04/05 were never using it).

Re-run `bin/slam-run`'s full harness after each future change to confirm
it actually helps (`docs/RESULTS.md`'s tables are the ground truth to
compare against), not just "still runs" — same discipline as every other
milestone this session.
