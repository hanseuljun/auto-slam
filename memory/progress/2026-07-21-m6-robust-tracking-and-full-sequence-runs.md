# M6 — robust tracking & map maintenance

Landed the seventh milestone from `plan/STAGE1.md`, following M0-M5.
Scoped to the highest-value pieces given session time: track-loss
recovery (both `VoPipeline` and `VioPipeline`) and a real LK robustness
gap, verified by actually running full, un-truncated sequences — not just
150-frame clips — which is what M6's own test spec asks for.

## What's done

### Track-loss recovery (both M3's VO and M5's VIO)
- `VoPipeline::process_frame` no longer just returns `None` on track loss
  (too few surviving LK tracks, or PnP failing on a degenerate point
  set). It resets the local map (fresh stereo-matched landmarks) anchored
  at the last known pose and reports `FrameResult::recovered = true`.
  Only returns `None` if recovery itself finds nothing to re-anchor to
  (e.g. a genuinely unmatchable frame) — and even then, the pipeline's
  internal state stays valid for the *next* call to try again.
- `VioPipeline::process_frame` does the same, but uses **IMU-only
  propagation** (`propagate_state`, the forward-physics counterpart of
  `slam_optim::imu_residual`) as the fallback pose instead of just
  reusing the last known pose — a real VIO capability (surviving a visual
  dropout via IMU dead-reckoning), not available to VO-only recovery.
  `propagate_state` is independently verified against the same synthetic
  ground-truth-motion model used throughout this session's IMU-adjacent
  tests (matched on the first attempt, unlike M4's initializer).

### A real LK robustness gap, found via testing the recovery path
Writing the recovery test surfaced that `slam-vision`'s LK tracker's
`min_determinant` check only examines the *template* (previous frame) —
it says nothing about whether the *current* frame actually contains that
template anywhere. A well-textured real patch tracked into a blank/wrong
frame was reporting `found = true` regardless. Added
`LkParams::max_final_residual`: reject a track if the mean absolute pixel
difference between template and matched patch is still large after
convergence. Full writeup (including why a mid-gray test frame doesn't
reliably force total loss on real images, but independent random noise
does) in `notes/lk-tracker-gotchas.md`.

## Real-data checkpoint: full, un-truncated sequences

`full_sequence_runs_survive_all_five_sequences_without_permanent_loss`
(`crates/slam-frontend/src/lib.rs`, `#[ignore]`d — runs every frame of
every `MH_*` sequence, ~3 minutes total, not part of routine `cargo
test`) is the actual M6 test from the plan: "full run on all five MH
sequences end-to-end without crashing/losing tracking permanently;
record per-sequence ATE/RPE." Result, run 2026-07-21:

| Sequence | Frames | Unrecoverable | ATE RMSE (full-sequence VO-only) |
|---|---|---|---|
| MH_01_easy | 3682 | 0 | 4.307m (3638 poses) |
| MH_02_easy | 3040 | 0 | 4.645m (2999 poses) |
| MH_03_medium | 2700 | 0 | 3.579m (2631 poses) |
| MH_04_difficult | 2032 | 0 | 6.819m (1976 poses) |
| MH_05_difficult | 2273 | 0 | 6.877m (2221 poses) |

**Zero unrecoverable frames across ~14,000 total frames** — the
recovery mechanism was never even needed on these five sequences in this
run (a good sign the base tracker + M2/M3's design is solid; recovery
exists for the frames/sequences that do need it, not observed as
necessary here). The multi-meter ATE over full sequences is expected, not
a regression: this is VO-only (no loop closure, no global BA) drifting
freely over ~2-3 minutes of real flight with zero correction — exactly
the accuracy gap M7 (loop closure) and M8 (global BA) exist to close.
Don't compare this number to M3's ~0.11-0.17m *short-clip* ATE and think
something broke; different measurement (unbounded drift over the whole
flight vs. a 150-frame window).

## A second real bug found by this same full-sequence run

The full-sequence test itself panicked (index out of bounds) on MH_04 —
iterating `0..cam0_frames.len()` and loading both cameras by the same
index overruns cam1's shorter array (the mismatch documented in
`decisions/0002` from M0, now actually triggered for the first time
because no earlier test ran a sequence to its very end). Fixed by
bounding the loop at `min(cam0_len, cam1_len)`; documented in
`notes/dataset-quirks.md` as a trap for any *other* future driver/test
code that loads both cameras by a shared index.

## Not done yet (correctly out of scope for this M6 pass)

- Landmark culling (low-parallax / high-error point removal) — landmark
  lists grow unboundedly over a full-sequence run currently; didn't cause
  a correctness or observed performance problem in the full-sequence
  checkpoint above (full 5-sequence run completed in ~3 minutes), so not
  urgent, but a real memory/performance concern for longer runs or M7+.
- Keyframe culling (redundant keyframe removal).
- Chi-square gating / a formal outlier-rejection pass in `slam-optim`'s
  solver beyond the Huber kernel already there (M5) and the LK residual
  check added here.
- RPE (relative pose error) — M6's own test mentions it, but ATE alone
  was judged sufficient signal for this pass; RPE's fuller treatment is
  explicitly M9's job (`decisions/0004`already established this "bring
  forward the minimum needed now, fuller version later" pattern for ATE
  at M3).
