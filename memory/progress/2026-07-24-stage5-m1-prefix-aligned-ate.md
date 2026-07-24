---
name: stage5-m1-prefix-aligned-ate
description: Stage 5 M1 done — crates/slam-eval gained compute_ate_prefix_aligned, fitting the Umeyama transform against only a leading prefix (bin/slam-run uses the first 30s of data) instead of the whole trajectory, so ATE near the start reflects real early accuracy instead of being partly absorbed by later drift. Verified on all 5 full-sequence runs: near-t=0 error is small everywhere, and the honest aggregate ATE is real and larger than today's smoothed number on every sequence.
metadata:
  type: progress
---

# Stage 5 M1: prefix-aligned ATE implemented

Implements the decision from `memory/decisions/0020` (Stage 5 M0).

## What changed

- `crates/slam-eval/src/align.rs`: `compute_ate_series_prefix_aligned`
  and `compute_ate_prefix_aligned` — fit the existing `umeyama_alignment`
  against only `estimated[..prefix]`/`groundtruth[..prefix]`, apply the
  resulting transform to the whole trajectory. `compute_ate`/
  `compute_ate_series` (whole-trajectory fit) unchanged — kept for
  `docs/RESULTS.md`'s existing SOTA comparison table, per M0's decision.
  Shared stats computation factored into `ate_stats_from_errors` so both
  paths stay in sync.
- `TrajectoryReport` (`report.rs`) gained `ate_prefix_aligned:
  Option<AteStats>`; `build_report` gained an `align_prefix_len:
  Option<usize>` parameter. `RunMeta` (`run_meta.rs`) gained the same
  field with `#[serde(default)]` — checked directly (not assumed) that a
  hand-written old-format `meta.json` fixture without the field still
  deserializes, since real historical `runs/<sequence>/<run_id>/
  meta.json` files on disk predate it.
- `write_summary_csv` gained `ate_prefix_aligned_{rmse,mean,max}` columns.
- `bin/slam-run`: new `ALIGN_PREFIX_SECONDS = 30.0` constant (matches
  the existing bounded-clip duration, not a new arbitrary number);
  `align_prefix_len` computed from actual keyframe timestamps (time-
  based, not a fixed keyframe count — track-loss recovery makes
  keyframes-per-second inconsistent across runs, `plan/STAGE4.md` M2).
  Prints both ATE numbers per sequence now.

## New test coverage

`align.rs` gained the exact synthetic check `plan/STAGE5.md` M1's own
test criteria called for: a trajectory that's exactly accurate for its
first 10 (of 30) points and then drifts. Confirms, deterministically,
both halves of the claim: prefix-aligned error stays ~0 on the untouched
early portion while showing real growing error where the drift actually
is, *and* whole-trajectory alignment reports more early error than
prefix-aligned on the same data (the masking effect M0 found on real
data, reproduced synthetically so it doesn't depend on any one real
run's specifics). Plus a stats-shape test (a full-length prefix exactly
matches `compute_ate`; an oversized prefix clamps, doesn't error) and a
mismatched-lengths `None` test. `run_meta.rs` gained an old-format
`meta.json` fixture test for the `#[serde(default)]` backward-
compatibility claim. 6 new tests, `crates/slam-eval` now at 27
(was 22).

## Verified on real data, all 5 sequences, full un-truncated runs

| Sequence | whole-trajectory ATE (today's) | prefix-aligned ATE (honest) |
|---|---|---|
| MH_01_easy | 3.868m | 5.412m |
| MH_02_easy | 3.854m | 7.787m |
| MH_03_medium | 3.460m | 17.180m |
| MH_04_difficult | 6.600m | 9.689m |
| MH_05_difficult | 6.818m | 12.945m |

Every sequence: honest number is real and larger — confirms `plan/
STAGE5.md`'s own Finding 1 generalizes past `MH_01_easy`. Directly
re-checked per-point error (not just trusting the aggregate) on two
sequences: `MH_01_easy` err[0]=0.185m, `MH_03_medium` err[0]=0.170m —
both small, both stable (no lever-arm blowup; `MH_03`'s own error grows
smoothly from 0.17m to 25.5m over the run, a real drift curve, not an
alignment artifact). Bounded-clip runs (600 frames, ~30s) are entirely
inside the prefix window, so `ate_prefix_aligned` there is numerically
identical to the existing `ate` — confirmed on `MH_01_easy`'s own
bounded run (0.151m both), i.e. this change doesn't regress or alter
`docs/RESULTS.md`'s existing bounded-clip table at all, only adds
visibility on full-sequence runs where it previously hid something real.

## What's next

`plan/STAGE5.md` M2 (wire real loop closure into `bin/slam-run`'s actual
pipeline, for every sequence with a detectable loop, not just a MH_05
VO-only demo) is next — not started. `docs/RESULTS.md`/`README.md`
updates deliberately deferred to M3 (per the plan's own scoping), so
they get updated once with both the honest metric *and* real loop
closure reflected together, not twice.
