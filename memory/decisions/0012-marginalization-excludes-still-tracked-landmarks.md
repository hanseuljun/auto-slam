---
name: marginalization-excludes-still-tracked-landmarks
description: VioPipeline's marginalization (Stage 2 M1) only folds a landmark into the departing keyframe's prior if nothing else — including the live, frame-by-frame LK tracker, not just other keyframes' observations — still references it; folding in a still-tracked landmark froze its position while future frames kept using it for PnP, measurably degrading accuracy.
metadata:
  type: decision
---

# Decision: marginalization excludes landmarks `self.tracks` still references, not just landmarks other keyframes observe

## Decision

`VioPipeline::marginalize_evicted_keyframe` (`crates/slam-backend/src/vio.rs`)
only treats a landmark as safe to fold into the departing keyframe's prior
(and thus eliminate from the optimizer entirely) if **no other window
keyframe observes it AND `self.tracks` — the live, per-frame LK
tracker — isn't still following it**. The first version of this code only
checked the former.

## Why

A landmark can be actively tracked frame-to-frame by LK (`self.tracks`)
for several frames before it gets recorded into another keyframe's
`observations` list (that only happens when a *new* keyframe is created,
every `keyframe_stride` frames). A landmark whose only keyframe
observation was at the keyframe about to be marginalized, but that
`self.tracks` was still actively following, would pass the "no other
keyframe observes it" check and get folded into the prior — eliminated
from the optimizer for good. Its position in `self.landmarks` then froze
permanently (marginalized landmarks are never touched again), while
subsequent frames kept using that increasingly stale, unoptimized
position for PnP — a real bug found while validating Stage 2 M1 against
real MH_01 data, before the fix measurably worse than the naive-fixed-lag
baseline it was supposed to match or beat.

## How to apply

Any future code that decides "is X still needed" by checking one
data structure (here, keyframe observation lists) has to consider *every*
place that data could still be referenced — in this codebase, that means
checking `self.tracks` too, not just `self.window`. If a similar
"can I discard this" decision comes up elsewhere (e.g. landmark culling,
`plan/STAGE2.md`'s M10), re-derive the full set of live references rather
than assuming the first place you'd naturally look is the only one.
