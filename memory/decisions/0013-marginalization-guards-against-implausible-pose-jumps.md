---
name: marginalization-guards-against-implausible-pose-jumps
description: VioPipeline's marginalization (Stage 2 M1) rejects folding a keyframe into the prior chain if its pose jumped implausibly from the new oldest keyframe, or if the resulting prior isn't finite — naive fixed-lag dropping used to just "forget" an occasional bad pose at the next eviction, but marginalization locks information in, so an unguarded bad pose diverged catastrophically (to multi-trillion-meter positions) within ~100 frames on real MH_01 data.
metadata:
  type: decision
---

# Decision: marginalization rejects implausible pose jumps and non-finite results at the eviction boundary

## Decision

`VioPipeline::marginalize_evicted_keyframe` (`crates/slam-backend/src/vio.rs`)
now: (1) checks the relative translation jump between the evicted keyframe
and the new oldest keyframe against `VioParams::
max_marginalization_pose_jump_meters` (default 10.0m — looser than
`max_pose_jump_meters`'s 2.0m since this is keyframe-to-keyframe, ~10x the
frame interval) before attempting to fold anything in, and (2) filters the
resulting `PriorFactor` to require every entry of `information`/
`information_vector` be finite. Either failure resets `self.prior` to
`None` (falling back to naive-drop behavior for that one eviction) instead
of keeping a stale or corrupt prior.

## Why

Validating Stage 2 M1 against real MH_01 data (600-frame bounded run),
ATE regressed sharply (0.137m -> 0.168m) and keyframe count nearly
doubled (261 -> 438) versus the naive-fixed-lag baseline. Diagnostic
logging traced it to a specific mechanism: naive fixed-lag dropping
*forgets* a keyframe the moment it slides out of the window, so an
occasional implausible pose (the underlying, separate bug fixed by
`decisions/0014`) never had lasting consequences. Marginalization's whole
point is the opposite — retain information instead of discarding it — so
without a guard, one bad pose got folded into an increasingly confident
prior with nothing left to correct it, and the corruption compounded
every subsequent marginalization step: printed keyframe positions grew
from a plausible few meters to ~30,000m within one eviction, then to
multi-trillion-meter magnitudes within roughly 100 frames.

## How to apply

This is the same "reject implausible results at the boundary, don't
propagate them" discipline `decisions/0009` established for raw PnP
output, applied one layer up: anywhere information gets *retained* across
time (a prior, a cache, an accumulator) needs its own sanity check,
because retention turns a transient bad value into a permanent one. Don't
assume a guard at one layer (e.g. `decisions/0014`'s fix at the PnP
source) makes a guard at a later layer redundant — defense in depth
matters here specifically because the two guards catch different things
(a single bad PnP result vs. anything that reaches the window
implausibly displaced, e.g. via a chain of track-loss recoveries).
