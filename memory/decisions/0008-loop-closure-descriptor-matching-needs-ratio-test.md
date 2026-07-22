---
name: loop-closure-descriptor-matching-needs-ratio-test
description: M7's BRIEF descriptor matching required a Lowe's-ratio-test filter (not just an absolute Hamming threshold) to get real geometric verification working on MH_05's actual loop — absolute-threshold-only matching let through many coincidental non-correspondences.
metadata:
  type: decision
---

# Decision: descriptor matching needs a ratio test, not just an absolute Hamming threshold

## Decision

`slam_loopclosure`'s `match_descriptors` (`crates/slam-loopclosure/src/geometric_verification.rs`)
requires both an absolute Hamming-distance threshold *and* a Lowe's-ratio-
test check (best match must beat the second-best by `max_ratio`, default
0.8) before accepting a mutual-nearest-neighbor descriptor match.

## Why

Testing against MH_05's real, groundtruth-confirmed loop (keyframe 22 and
keyframe 111 revisit the same physical place — see
`notes/dataset-quirks.md`) with absolute-threshold-only matching
(Hamming < 60 out of 256 bits) found 20-90 "matches" per candidate pair,
every single one geometrically inconsistent (0-2 inliers out of dozens,
across 51-99 verification attempts on two different test runs). With a few
hundred candidate descriptors on each side, order statistics on 256-bit
Hamming distances mean an absolute threshold alone lets many *coincidental*
matches through even at a seemingly strict cutoff — the minimum distance
among ~500 comparisons is a much more favorable draw than a single
comparison's expected value. Adding the ratio test (reject a match if the
second-best candidate is nearly as good as the best — i.e. the match isn't
actually distinctive) fixed this immediately: the same MH_05 pair went
from 0 inliers everywhere to matches like 38/38, 30/31, 21/21 inliers, and
the full loop-closure pipeline (place recognition -> geometric
verification -> pose-graph optimization) produced a genuine, measurable
ATE improvement (5.6m -> 3.3m on the real MH_05 checkpoint).

## How to apply

If a future change to the descriptor (e.g. orientation-invariant BRIEF/
rBRIEF, a larger vocabulary, or a different bit count) seems to regress
match quality, check the ratio test is still wired in before assuming the
descriptor itself changed — it's very easy to lose implicitly (e.g. if
`match_descriptors` is ever refactored to compute only the single best
match per query instead of best-two). Don't loosen `max_hamming_distance`
as a first response to "not enough matches found" — per this finding,
the *ratio* is what actually controls false-positive rate; the absolute
threshold is a much weaker filter on its own at these candidate-set sizes.
