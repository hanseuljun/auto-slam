---
name: stage4-m3-full-sequence-default-stage-complete
description: Stage 4 M3 done, stage complete — bin/slam-run's default flipped from the 600-frame bounded clip to the full un-truncated sequence (--full removed, --frames N now the opt-in bounded/fast mode). Added TimingBreakdown::whole_run_factor() so the harness reports the metric Stage 4's goal 2 actually gates on directly, not just via manual recomputation. bin/slam-viz confirmed to need no code changes by loading a real 725-keyframe full-sequence run through it. All of Stage 4 (M0-M3) is done.
metadata:
  type: progress
---

# Stage 4 M3: default flipped to full-sequence, stage complete

## The change

`bin/slam-run/src/main.rs`'s `Cli`: removed the `--full` boolean flag;
`frames` is now `Option<usize>` instead of `usize` with a 600 default.
`None` (the new default, i.e. omitting the flag) runs the full
un-truncated sequence; `Some(n)` (`--frames N`) caps the run at `n`
frames, the old bounded/fast-iteration mode. This is a real inversion,
not just a renamed flag — the unflagged behavior actually changed.

`RunConfig::frame_cap`/`full_sequence` (the `meta.json` schema
`bin/slam-viz`'s run picker reads) keep their existing shape:
`full_sequence: frame_cap.is_none()`, `frame_cap:
frame_cap.unwrap_or(full_len)` — no serde-breaking change, `frame_cap`
now just means "the actual frame count used" when running full instead
of "the requested cap," which stays meaningful either way.

## New diagnostic: `TimingBreakdown::whole_run_factor()`

`crates/slam-eval/src/timing.rs` gained `whole_run_factor()` = `(vision
+ optimization + global_ba + loop_closure) / data_seconds`, alongside
the pre-existing `real_time_factor()` (vision+optimization only). Stage
4 M0's own finding flagged that `real_time_factor()` alone can misreport
"real-time" once global BA stops being negligible (it reported 0.686 on
`MH_01_easy`'s full sequence pre-M1-fix while true wall-clock was ~5.9x
data duration) — `docs/RESULTS.md`'s "whole-run factor" column existed
only as a manually-computed number until now. `bin/slam-run`'s
per-sequence line and summary table now print both factors directly, so
running the harness itself surfaces the number Stage 4's goal 2 actually
gates on, not just a per-frame-loop number that quietly stopped meaning
what it used to. Two new unit tests in `timing.rs` cover this (one
checks `whole_run_factor` includes global BA + loop closure where
`real_time_factor` doesn't; the zero-data-seconds test now also checks
`whole_run_factor`'s infinite-not-panicking behavior).

## Verification

- `cargo run --release --bin slam-run` (no flags), all 5 sequences,
  full run: ATE rmse 3.868/3.854/3.460/6.600/6.818m — matches M0-M2's
  already-recorded numbers exactly. Total wall-clock ~9 minutes.
- `--frames 600` still reproduces the original bounded-clip numbers
  (0.151m etc. on `MH_01_easy`) unchanged.
- `bin/slam-viz` needed no code changes, as the plan predicted — but
  verified this rather than assuming it: temporarily added an `#[ignore]`
  test calling `load_run_scene` directly on a real 725-keyframe
  full-sequence run directory (`runs/MH_01_easy/20260724-021335-894`),
  confirmed it loads cleanly (3292 vertices, sane bounding box), then
  removed the test (it depended on a local run directory path, not
  something to keep as a committed test).
- `cargo test --workspace --release`: all passing.

## Documentation updated

`docs/RESULTS.md`: "How to reproduce" commands updated to the new
`--frames` semantics; the bounded-clip tables' framing updated to say
they're now the opt-in fast mode, not the default; "Full-sequence
results" section retitled to note it's now what the default run
produces. `README.md`: status paragraph, milestone table, and the
Stage 4 narrative section all updated to reflect M3 (and M2) as done.
`plan/STAGE4.md`: M3 marked Done with this Result — all of Stage 4
(M0-M3) is now complete.

## What's next

Stage 4 is complete. No open milestone remains in `plan/STAGE4.md`. A
future stage would need a new plan document (following the same
discipline: measure before fixing, real numbers over assumed ones) if
further work on this pipeline is wanted — candidates surfaced but
explicitly out-of-scope during Stage 4: the ~45-52% track-loss recovery
rate (real, pervasive, worth frontend-robustness attention on its own,
`memory/progress/2026-07-24-stage4-m2-accuracy-regression-ruled-out.md`),
and closing the loop-closure gap for full-sequence accuracy (this
harness still doesn't chain M7 into `bin/slam-run`'s numbers).
