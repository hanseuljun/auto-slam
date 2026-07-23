---
name: stage2-m6-huber-and-window-size-tried-reverted
description: Stage 2 M6's remaining open items (Huber threshold, smaller window_size) both swept and measured against real data, both reverted — closes out M6, ad hoc-knob tuning space exhausted.
metadata:
  type: progress
---

# Stage 2 M6 — Huber threshold and window size: tried, measured, reverted (M6 done)

Stage 2 M6: tried the two remaining open tuning items from `plan/
STAGE2.md`'s M6 section — outlier-gating (Huber) threshold, and a
smaller `window_size` (6 and 4, since 12 had already regressed
everything). Measured both on the full 5-sequence `bin/slam-run`
harness against the M0 baseline.

Huber threshold: tried 1.5 (tighter) and 5.0 (looser) against the
default 3.0. Both directions destabilize MH_05 specifically (roughly
doubles-to-triples its ATE either way) for only small, inconsistent
gains on the other sequences. Reverted.

window_size: tried 6 and 4 against the default 8. window_size=6 helps
MH_04 substantially (1.174m -> 0.847m) but regresses MH_01/MH_03/MH_05;
window_size=4 is worse than the default on every sequence. Neither
meets the plan's "improvement on every sequence" bar. Reverted.

Full writeup: `decisions/0017`. Working tree is back to the exact M0
baseline config (`huber_delta: 3.0`, `window_size: 8`) — `git diff`
clean before this commit's memory-only changes.

This closes out M6's two remaining open items from `plan/STAGE2.md`.
What's left of Stage 1's M10/Stage 2's M6 scope (initializer robustness
specifically for MH_04/MH_05) was already noted as lower priority since
both sequences produce real numbers via the dynamic initializer — no
"produces nothing" gap to close there, unlike the MH_02/03 fix
(`decisions/0015`). Recommending M6 be considered functionally done:
the cheap, ad hoc-knob-sweep tuning space is exhausted for this
pipeline's current architecture; further accuracy gains need the
larger, structural work already named and deferred in M2/M3
(analytic IMU Jacobians, real preintegration covariance) rather than
more scalar sweeps.
