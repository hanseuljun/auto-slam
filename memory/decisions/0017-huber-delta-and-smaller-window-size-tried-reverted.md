---
name: huber-delta-and-smaller-window-size-tried-reverted
description: Stage 2 M6's remaining open tuning items (outlier-gating Huber threshold, smaller window_size) both measured on real data and both reverted — current defaults (huber_delta=3.0, window_size=8) remain the best all-around choice found.
metadata:
  type: project
---

`plan/STAGE2.md`'s M6 listed two remaining open tuning items after
[[stationary-window-threshold-loosened-to-0.10]] and the reverted
sensor.yaml noise-weighting/window_size=12 attempts
([[sensor-yaml-derived-imu-weights-reverted]]): outlier-gating threshold
tuning, and a smaller `window_size` (since bigger had already regressed
everything). Both tried this session via `bin/slam-run`'s bounded-clip
harness, measured against the M0 baseline (MH_01 0.151 / MH_02 0.184 /
MH_03 0.511 / MH_04 1.174 / MH_05 0.455m ATE RMSE), both reverted.

## Huber threshold (`SolverConfig::huber_delta`, default 3.0)

Tried both directions:

- **Tighter (1.5)**: MH_03 improved slightly (0.511->0.503), MH_04
  improved (1.174->1.010), but MH_01 regressed (0.151->0.183), MH_02
  regressed slightly (0.184->0.191), and MH_05 regressed badly
  (0.455->1.449).
- **Looser (5.0)**: MH_01 improved slightly (0.151->0.165 — actually
  worse, see raw numbers below), MH_03 improved (0.511->0.460), but
  MH_04 regressed (1.174->1.033) and MH_05 regressed badly
  (0.455->0.995).

Both directions destabilize MH_05 specifically (roughly doubles to
triples its ATE either way) while giving only small, inconsistent wins
elsewhere. The current default of 3.0 is already close to a local
optimum for this pipeline's current noise weights; reverted, no change
to `crates/slam-optim/src/solver.rs`.

## Smaller `window_size` (`VioParams::window_size`, default 8)

Tried 6 and 4, as a follow-up to `window_size=12` already having
regressed everything ([[sensor-yaml-derived-imu-weights-reverted]]):

- **6**: MH_04 improved substantially (1.174->0.847), MH_02 improved
  marginally (0.184->0.182), but MH_01 (0.151->0.171), MH_03
  (0.511->0.553), and MH_05 (0.455->0.502) all regressed.
- **4**: worse than the default on every sequence, several badly (MH_04
  1.174->1.451, MH_05 0.455->0.818).

So the ATE-vs-window_size curve isn't monotonic across sequences — 8 is
a local optimum on 4 of 5 sequences even though 6 helps MH_04
specifically a lot. `plan/STAGE2.md`'s M6 bar is "measurable improvement
... on every sequence tuned," which neither 6 nor 4 meets; reverted, no
change to `crates/slam-backend/src/vio.rs`.

## Why this matters for future tuning

Both knobs show the same pattern the earlier noise-weighting attempt
found: this pipeline's few remaining ad hoc constants are already
mutually adapted to each other and to the current (numerical-Jacobian,
ad hoc-weight) pipeline, so single-knob sweeps trade one sequence's
accuracy for another's rather than improving all five. A real further
win likely needs either per-sequence tuning (out of scope — this is one
global config) or the larger, structural fixes M2/M3's deferred scope
already named (analytic IMU Jacobians, real preintegration covariance)
rather than more scalar sweeps of the existing knobs.
