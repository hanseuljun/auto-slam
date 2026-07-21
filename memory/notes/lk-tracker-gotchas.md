# Lucas-Kanade tracker gotchas

Living notes on `crates/slam-vision`'s pyramidal LK tracker
(`crates/slam-vision/src/lk.rs`). Add to this as more is learned.

## Coarse pyramid levels can't support a full-size window — don't abort the
## whole track when that happens

First implementation of `track_single_point` aborted the entire track
(`found = false`) the instant any pyramid level's tracking window fell
outside that level's image bounds. This is wrong: a `window_radius=7`
(15x15) window is often *larger than or comparable to* the coarsest pyramid
level itself (e.g. a 752x480 image's 4th level is 94x60 — a point 8px from
that level's edge already can't fit the window), so on real images this
killed nearly every track at initialization. Fixed by only marking a track
lost if the *finest* level (level 0, where accuracy actually matters) fails
to refine; coarser-level failures just skip refinement at that level and
keep propagating the current displacement guess to the next finer level.
See `decisions` — no separate decision file for this, it's a bug fix, not a
design choice; recorded here so a future session doesn't reintroduce it
when touching `lk_iterate_level`.

## Straight-edge points are a textbook aperture-problem trap in test fixtures

An early unit test picked tracking points at the midpoints of a synthetic
square's edges (e.g. `(40, 20)` on a square spanning `x:[20,60), y:[20,60)`
with the point on the top edge). Those points have a locally 1D image
gradient (intensity varies with `y` only inside the window), which makes
the 2x2 LK structure matrix singular in the edge-parallel direction — the
classic aperture problem, and *correct* rejection by the
`min_determinant` check, not a bug. Fixed by moving test points near the
square's *corners* instead, where both x and y gradients are present. Keep
this in mind for any future LK/KLT/optical-flow test fixture: corners
track, straight edges don't (by design).

## Real MH_01 tracking survival rate (informal baseline, 2026-07-20)

`slam-inspect`'s vision-frontend section (grid-FAST on frame 0 -> LK
tracked across 5 consecutive real frames, no forward-backward check or
outlier gating yet) sees roughly 93-97% raw survival across all five
`MH_*` sequences. Useful as an informal regression baseline: if a future
change to the tracker or detector drops this dramatically on the same
sequences, that's a signal something broke, not just "tracking is hard."
