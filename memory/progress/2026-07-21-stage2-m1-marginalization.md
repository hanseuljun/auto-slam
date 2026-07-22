---
name: stage2-m1-marginalization
description: Stage 2 M1 done — real Schur-complement keyframe marginalization in slam-optim + slam-backend, closing decisions/0007. Found and fixed three real bugs along the way, including a latent VioPipeline vulnerability decisions/0009 had explicitly predicted.
metadata:
  type: progress
---

# Stage 2 M1 — sliding-window marginalization (closes `decisions/0007`)

Landed the second milestone of `plan/STAGE2.md`. The largest single
debugging effort of the session — the core math was right quickly, but
getting it safe on real data took three real bug fixes.

## What's done

- `slam-optim::state::KeyframeState::local`: the exact tangent-space
  inverse of `retract` (round-trip verified, not just approximately).
- `slam-optim::solver`: `PriorFactor` (information-form Gaussian prior,
  First-Estimate-Jacobian re-linearization), generalized gauge-anchoring
  (`Problem.priors` lets keyframe 0 be free-but-constrained instead of
  hard-fixed once a prior exists), prior accumulation in
  `build_normal_equations`/`compute_cost`/backsubstitution.
- `slam-optim::marginalization`: `marginalize_keyframe` — Schur-
  complements a departing keyframe (with its incoming prior, IMU/bias-rw
  edge, and uniquely-observed landmarks) into a new prior on the
  keyframe that inherits it. Verified with two isolated tests at 1e-6
  precision (IMU-only, and IMU+unique-landmarks) plus a 4-keyframe joint-
  vs-marginalized consistency test.
- `slam_backend::VioPipeline::marginalize_evicted_keyframe`: wires this
  into the real sliding window, replacing naive fixed-lag dropping.

## Real checkpoint

`bin/slam-run` on real MH_01 data (600-frame bounded clip): with
marginalization, ATE 0.169m/104 keyframes; with the same PnP fix but
marginalization disabled, 0.164m/109 keyframes — matching within noise,
confirming marginalization itself doesn't regress accuracy. (The original
naive-drop baseline, 0.137m/261 keyframes, predates the PnP fix below and
isn't a fair comparison — see the bug writeup.) `plan/STAGE2.md`'s own
"single biggest accuracy lever" framing isn't dramatically demonstrated on
this short clip, consistent with M8's own earlier finding that short
clips leave little unfinished optimization for any global-information
pass to clean up — a longer run or a genuinely under-constrained window
is where marginalization's real win should show up, a good follow-up but
not required for this checkpoint.

## Three real bugs found and fixed validating this on real data

1. **Marginalization ate landmarks the live tracker still needed**
   (`decisions/0012`). `marginalize_evicted_keyframe` originally only
   checked "does another *keyframe* still observe this landmark" before
   folding it in and eliminating it — missing that `self.tracks` (the
   frame-by-frame LK tracker) can still be actively following a landmark
   before it's ever recorded in another keyframe's observations. Folding
   such a landmark in froze its position forever while future frames kept
   using that stale position for PnP.

2. **Marginalization had no defense against locking in a bad pose**
   (`decisions/0013`). Naive fixed-lag dropping "forgets" an occasional
   implausible keyframe pose the moment it evicts — marginalization's
   whole point is the opposite (retain information), so an unguarded bad
   pose got folded into an increasingly confident prior with nothing left
   to correct it. Measured: printed keyframe positions grew from a
   plausible few meters to ~30,000m after one eviction, to multi-
   trillion-meter magnitudes within ~100 frames, before a pose-jump guard
   at the marginalization boundary was added.

3. **The actual root cause: `VioPipeline` never got `decisions/0009`'s
   PnP pose-jump guard** (`decisions/0014`). Bug 2's guard was real and
   worth keeping, but chasing where the bad pose *came from* led further
   upstream: `VioPipeline`'s raw DLT-PnP output was never filtered for
   plausibility at all, unlike `VoPipeline`'s (M7, `decisions/0009`) —
   which had explicitly predicted this exact gap ("VioPipeline... likely
   shares this vulnerability... isn't yet protected"). Confirmed as the
   real root cause empirically: with only this fix (marginalization still
   disabled), a run that previously diverged to absurd magnitudes instead
   gave a clean, plausible trajectory.

## Debugging technique notes (worth rereading)

- **Isolate before integrating.** Two tight (1e-6) isolated tests — IMU-
  edge-only marginalization, then IMU+landmarks — caught nothing wrong,
  which correctly ruled out the core Schur-complement math before
  spending time on the real-pipeline integration bugs above. When a full
  integration test fails but isolated unit tests pass, the bug is almost
  always in the integration/wiring, not the core algorithm — that's
  exactly where it was.
- **A diagnostic that starts from exact ground truth is a strong,
  cheap correctness check.** Before touching real MH_01 data, verifying
  that optimizing an assembled system starting *at* ground truth stays
  at ground truth (cost ~1e-26) proved the marginalization formula
  itself was exactly right, well before the real-data bugs were found —
  this separated "is the math right" from "is the LM solver robust to
  a large perturbation," two very different questions that looked like
  one failing test at first.
- **When a metric "looks fine," check whether it's structurally unable
  to detect the failure mode you're worried about**, same lesson as
  `decisions/0009`. Here it went one step further: two *different*
  latent corruption sources (bugs 2 and 3) were both individually masked
  by the same Sim3-aligned-ATE-on-a-short-clip blind spot, and only
  became visible once marginalization's information-retention property
  removed the "gets forgotten before it matters" safety net naive-drop
  had accidentally been providing.
