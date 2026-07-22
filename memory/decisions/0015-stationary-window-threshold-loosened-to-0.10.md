---
name: stationary-window-threshold-loosened-to-0.10
description: slam_imu::find_stationary_window's max_gyro_norm threshold moved from 0.09 to 0.10 rad/s at every call site — measured, not guessed: MH_02_easy and MH_03_medium's best achievable 200-sample window was 0.093/0.090, just barely over the old threshold, causing them to skip entirely (no ATE/timing numbers at all) despite both genuinely having a stationary start.
metadata:
  type: decision
---

# Decision: stationary-window bootstrap threshold loosened from 0.09 to 0.10 rad/s

## Decision

Every call site of `slam_imu::find_stationary_window` (`crates/slam-imu/
src/initializer.rs`, `crates/slam-backend/src/tests_integration.rs`,
`bin/slam-inspect/src/main.rs`, `bin/slam-run/src/main.rs`) now passes
`0.10` instead of `0.09` for `max_gyro_norm`.

## Why

`docs/RESULTS.md`'s benchmark table was missing `MH_02_easy` and
`MH_03_medium` entirely — `bin/slam-run` and `bin/slam-inspect` both
reported "no stationary window to bootstrap from, skipping" for both,
despite `plan/STAGE1.md`'s own dataset notes documenting that MH_01-03
all start with the MAV stationary.

Measured the actual best-achievable 200-sample-window max gyro norm per
sequence (a small standalone script over each sequence's real
`imu0/data.csv`, not a guess):

| Sequence | best 200-sample window max \|gyro\| (rad/s) |
|---|---|
| MH_01_easy | 0.088 |
| MH_02_easy | 0.093 |
| MH_03_medium | 0.090 |
| MH_04_difficult | 0.088 |
| MH_05_difficult | 0.086 |

MH_02 and MH_03 were both *just* over the old 0.09 threshold (0.093 and
0.090) — not because they lack a genuinely stationary window, but
because the ADIS16448's noise floor in those two specific recordings sits
a hair higher than in MH_01/04/05. `0.10` admits both with a small margin
while staying well above every sequence's actual best value (0.086-0.093),
so it doesn't meaningfully loosen what counts as "stationary."

Confirmed as a real fix, not just "stops erroring": both sequences now
produce plausible ATE (0.184m / 0.511m) and real-time-factor (0.541 /
0.615) numbers via `bin/slam-run`, in the same range as the other three
sequences — see `docs/RESULTS.md`.

## How to apply

If a future dataset or a different IMU makes even 0.10 too tight (or too
loose) again, re-measure per-sequence rather than nudging the constant
blind — the script used here just slides a window over each sequence's
raw gyro-norm series and reports the minimum achievable max, which is a
five-minute check that avoids guessing. This is a good example of the
"measure before assuming" discipline this project has applied
repeatedly — the fix here isn't "try a bigger number and see if it
compiles," it's "find the actual number the data supports and pick a threshold
with real margin above it."
