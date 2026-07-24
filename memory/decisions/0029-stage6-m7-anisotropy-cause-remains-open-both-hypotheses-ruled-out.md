# 0029: Stage 6 M7 — both candidate causes ruled out; the anisotropy stays open, documented not fixed

## Context

`plan/STAGE6.md` M7's goal: fix the scale anomaly if M5/M6 point to a
real, fixable cause, or document exactly what's ruled in/out and stop —
explicitly permitted as a legitimate outcome ("an honest 'still open,
here's what we now know that we didn't before' is a legitimate outcome
for this milestone, not a failure to write around"), matching `plan/
STAGE5.md` M0's own precedent for exactly this situation.

## The direct test: disable marginalization, measure

M6's ablation (`decisions/0028`) ruled out IMU-vs-vision weighting and
pointed to a specific *hypothesized* mechanism instead: marginalization's
own Schur-complement accumulation of near-unconstrained bias/velocity
uncertainty, compounding over many evictions. This milestone tested that
hypothesis directly rather than stopping at the hypothesis: added
`VioParams::disable_marginalization` (discards an evicted keyframe's
information instead of folding it into a prior — the "naive fixed-lag"
scheme `decisions/0007` originally compared against and chose against on
accuracy grounds) and a `bin/slam-run --disable-marginalization` flag,
then ran it on the same 2 sequences with the same evaluation code.

**If the hypothesis were right, removing marginalization should reduce
the anisotropy** (no more Schur-complement accumulation to compound).
**It didn't** — it got worse, on both sequences:

| sequence | metric | normal VIO | marginalization disabled | IMU factors disabled (`decisions/0028`, for scale) |
|---|---|---|---|---|
| `MH_01_easy` | keyframes | 685 | **1413** | 2236 |
| | track-loss recoveries | 319 | **1137** | 2055 |
| | anisotropy x/y/z | 3.95 / 2.74 / 14.03 | **1181 / 539 / 362** | 7356 / 2752 / 4664 |
| | loop-closure gap (before) | 81.66m | **11061m** | 72210m |
| `MH_04_difficult` | keyframes | 374 | **510** | 1161 |
| | track-loss recoveries | 174 | **325** | 1057 |
| | anisotropy x/y/z | 1.12 / 1.60 / 2.10 | **38.7 / 15.4 / 46.8** | 1274 / 255 / 1737 |
| | loop-closure gap (before) | 20.02m | **1190m** | 23751m |

Marginalization-disabled sits *between* normal VIO and full IMU removal
on every metric — clearly worse than normal (hundreds-to-low-thousands x
anisotropy vs. single digits), clearly better than removing IMU entirely
(which is thousands x). **This is the opposite of what the Schur-
complement-accumulation hypothesis predicted.** Marginalization is doing
real, stabilizing work — folding an evicted keyframe's information
forward instead of discarding it outright reduces track-loss recoveries
(1137 -> 319 when marginalization is restored) and reduces anisotropy by
1-2 orders of magnitude, not increases it.

## Conclusion: both candidate mechanisms ruled out, cause remains open

Two real, measured ablations now stand:
- **IMU-vs-vision weighting** (`decisions/0028`): ruled out — removing
  IMU factors entirely causes catastrophic divergence, not a cleaner
  reconstruction.
- **Marginalization's Schur-complement accumulation** (this decision):
  ruled out — removing marginalization *also* causes divergence (milder
  than removing IMU, but still real and in the wrong direction for the
  hypothesis), not an improvement.

Both of the concrete, actionable mechanisms this stage's own
investigation produced have now been tested directly, not just reasoned
about, and both point the *opposite* direction from "removing this fixes
the anisotropy." The honest conclusion, per the plan's own explicit
permission for this outcome: **the root cause of the anisotropic scale
distortion (`decisions/0027`) remains open.** What this investigation
does establish, concretely, for whoever picks this up next:

- It is **not** simply IMU/vision weight imbalance (M6).
- It is **not** simply marginalization's own information-accumulation
  mechanism (M7, this decision) — if anything, marginalization measurably
  *helps* stability relative to the naive-drop alternative, on every
  metric checked.
- The anisotropy is present even in the *best-behaved* configuration
  tested (normal VIO, both mechanisms intact) — it's a property of the
  baseline reconstruction itself, not an artifact introduced by a
  specific factor type or accumulation mechanism this stage could
  isolate by removing it.
- Both ablations' *directionally consistent* Z-axis vulnerability under
  stress (`MH_04`'s z=46.8 under marginalization-disabled, still the
  worst axis, matching normal VIO's own z=2.10-worst finding) is worth
  carrying forward as a hint, even though the mechanism producing it
  wasn't found here — it survived across the normal case and one of the
  two ablations (not the other, where the whole system was too far into
  compounding failure for axis ranking to mean anything, per
  `decisions/0028`'s own caveat).

## Candidate directions not yet tried (for a future stage)

Not attempted here (would each be their own real investigation, not a
quick add-on to this one): a stereo-triangulation-specific depth-
direction bias (systematic error along the camera's own optical axis,
which stereo geometry could plausibly concentrate onto one world-frame
axis depending on the camera's typical viewing direction through a run);
a camera-IMU extrinsics calibration error specific to one axis; gravity-
direction/vertical-axis handling in the static/dynamic initializer
specifically (`slam-imu`'s own `static_initialize`/dynamic init, unlike
IMU *factors* or marginalization, was never ablated here).

## Stage 6 status

With M7 concluded (documented, not fixed — a legitimate, plan-sanctioned
outcome), all of Stage 6's milestones are landed: M1/M2 (Goal 1, real
Jacobian + covariance work, both landing with real accuracy effects),
M3/M4 (Goal 2, sparse solver + stride reduction, a real measured win),
M5/M6/M7 (Goal 3, real instrumentation reframing the scale question,
then two real ablations ruling out both candidate mechanisms and leaving
an honest, well-characterized open question instead of a false "fixed").
