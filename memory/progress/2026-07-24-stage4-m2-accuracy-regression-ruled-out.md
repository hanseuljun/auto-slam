---
name: stage4-m2-accuracy-regression-ruled-out
description: Stage 4 M2 done — the 5.6x-25.6x full-sequence-vs-bounded-clip ATE gap is confirmed natural full-sequence drift (no loop closure), not a bug. Cross-validated against Stage 1 M6's independent VO-only full-sequence baseline (different code path entirely), re-run fresh on current code: full VIO lands within ~20% of it on every sequence. Track-loss recovery rate (45-52% of all keyframes) was the leading candidate investigated and found real but not differentiating — same rate at bounded and full scale.
metadata:
  type: progress
---

# Stage 4 M2: full-sequence ATE gap investigated, confirmed not a regression

## The question

`plan/STAGE4.md` M0/M1 found full-sequence ATE 5.6x-25.6x worse than the
bounded 600-frame clip on every sequence. M2's job wasn't "is that
multiple big" in the abstract — it was whether it's worse than this
pipeline's own natural full-sequence drift should look, or in line with
it (the plan's own framing: "full-sequence ATE is *expected* to be
numerically larger... this isn't 'the number went up = regression'").

## Candidate 1: track-loss recovery rate (investigated, ruled out as the differentiator)

Instrumented `bin/slam-run` (kept permanently, cheap) to count
`VioFrameResult::recovered` keyframes — forced off-stride when too few
LK tracks survive, using IMU-only propagation with a full local-map
reset (`plan/STAGE1.md` M6). Result, all 5 full sequences:

| Sequence | keyframes | recoveries | rate |
|---|---|---|---|
| MH_01_easy | 741 | 382 | 51.6% |
| MH_02_easy | 552 | 255 | 46.2% |
| MH_03_medium | 536 | 270 | 50.4% |
| MH_04_difficult | 364 | 164 | 45.1% |
| MH_05_difficult | 456 | 230 | 50.4% |

Strikingly high — roughly half of every sequence's keyframes are
IMU-only coasts, not vision-corrected estimates. But re-running
`MH_01_easy`'s **bounded** 600-frame clip with the same instrumentation
gives 47/106 = 44.3%, essentially the same rate as the full run's
51.6%. Since the good bounded-clip ATE (0.151m) already contains this
same recovery rate, recovery frequency itself isn't what's
differentially worse on the full run — ruled out as *this milestone's*
cause, though it's a real, pervasive frontend-tracking-robustness
characteristic (on EuRoC's "easy" `machine_hall` sequences, no less)
worth a future stage's attention on its own, separate from Stage 4's
scope.

## Candidate 2: independent VO-only baseline (decisive)

`plan/STAGE1.md` M6 (`memory/progress/2026-07-21-m6-robust-tracking-and-full-sequence-runs.md`)
already ran a full-sequence, un-truncated checkpoint on `VoPipeline`
(pure visual odometry — no IMU factors, no windowed marginalized
backend, no global BA at all) back in Stage 1, and documented the
multi-meter ATE it found as *"expected, not a regression... this is
what no-loop-closure full-sequence flight looks like."* That's an
independent measurement from a different stage, using a code path that
shares essentially nothing with `VioPipeline`'s backend beyond the LK
tracker/PnP frontend.

Re-ran that same test fresh against current code
(`full_sequence_runs_survive_all_five_sequences_without_permanent_loss`,
`crates/slam-frontend/src/lib.rs`, `#[ignore]`d — `cargo test --release
-p slam-frontend ... -- --ignored --nocapture`, ~3.5 minutes) rather
than trusting a several-day-old memory note, since intervening tuning
(`decisions/0015`-`0017`) could plausibly have shifted VO-only behavior
too:

| Sequence | VO-only full ATE (m) | VIO full ATE (m) | Δ |
|---|---|---|---|
| MH_01_easy | 3.389 | 3.868 | +14% |
| MH_02_easy | 3.872 | 3.854 | -0.5% |
| MH_03_medium | 3.410 | 3.460 | +1.5% |
| MH_04_difficult | 6.533 | 6.600 | +1.0% |
| MH_05_difficult | 5.615 | 6.818 | +21% |

Full VIO (IMU fusion + marginalized window + capped global BA) lands
within ~20% of pure VO-only on every sequence, matching within 2% on 3
of 5 — despite the two pipelines being almost entirely different code.
Both are dominated by the same structural fact: no loop closure means a
multi-minute flight accumulates multi-meter drift that no amount of
windowed or global optimization can correct without an absolute
reference to correct against (`docs/RESULTS.md`'s own "Known gaps"
section already frames this for the bounded-clip-vs-SOTA comparison —
same underlying cause, now confirmed to apply at full-sequence scale
too, not a new problem introduced by running longer).

## Verdict

Full-sequence ATE is explainable by natural drift-over-time, cross-
validated against an independent baseline, not a bug-shaped regression.
No fix needed or applied — M2 closes on this finding. `plan/STAGE4.md`
M3 (flip `bin/slam-run`'s default to full-sequence) is now unblocked.

## What changed in the repo

- `bin/slam-run/src/main.rs`: counts and reports track-loss recoveries
  in the per-sequence summary line (`"{lost_frames} unrecoverable single
  frames, {recovered_frames} track-loss recoveries"`) — a real,
  previously-invisible diagnostic, kept permanently since it directly
  answers a question this milestone needed answered and costs nothing
  to compute.
- `docs/RESULTS.md`: "Full-sequence results" section gained the
  recovery-rate table, the VO-only cross-validation table, and an
  updated "Goal 3 (accuracy) is met" conclusion (previously "not yet
  met").
- `plan/STAGE4.md`: M2 marked Done with this Result.

## What's next

`plan/STAGE4.md` M3: flip `bin/slam-run`'s default from the bounded
600-frame clip to the full sequence, keep `--frames N` for fast-
iteration mode, update `docs/RESULTS.md`'s headline tables and
`README.md`'s reproduction instructions.
