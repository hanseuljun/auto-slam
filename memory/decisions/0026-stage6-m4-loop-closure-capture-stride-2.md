# 0026: Stage 6 M4 — LOOP_CLOSURE_CAPTURE_STRIDE reduced from 4 to 2, not 1

## Context

`plan/STAGE6.md` M4's goal: with M3's sparse pose-graph solver removing
the O(n^3) ceiling, reduce `LOOP_CLOSURE_CAPTURE_STRIDE` (`bin/slam-run/
src/main.rs`) — "ideally back to 1... if the real-time budget allows; if
not all the way to 1, as far as it does allow, measured, not guessed."

## Measured, not guessed: stride 1 breaks the real-time bar

Tried stride 1 first (every VIO keyframe, no downsampling — the ideal
per the plan's own preference). Measured directly on `MH_01_easy` (the
largest sequence, 685 keyframes full-run): **whole-run factor 1.082 —
over the bar**, with `loop_closure` cost alone at 67.84s (up from ~17s at
stride 4). M3's sparse solver made the *pose-graph solve itself* cheap
(~97ms even at 741 nodes, `decisions/0025`), but it isn't the only cost
that scales with capture density — BoW vocabulary training (over every
captured keyframe's descriptors) and place-recognition queries (one per
captured keyframe against the growing database) both scale roughly
linearly with keyframe count too, and at stride 1 that's ~4x the
descriptors/queries stride 4 had. Removing the O(n^3) ceiling didn't
remove *every* cost that scales with capture density, just the worst one.

Tried stride 2 next: holds the real-time bar on **all 5 sequences**
(whole-run factor 0.640-0.925 — MH_02/MH_03 have the least margin at
~0.91-0.93, still real headroom). **Decision: stride 2**, per the plan's
own "if not all the way to 1, as far as it does allow" fallback.

## Real, measured effect — mixed, not a clean uniform win

Geometric gap-closure ratio (`plan/STAGE5.md` M3's own metric,
before-gap / after-gap), stride 4 (post-M3 solver) vs. stride 2:

| sequence | stride 4 gap before -> after | stride 2 gap before -> after | ratio (stride 2) |
|---|---|---|---|
| MH_01_easy | 81.660m -> 18.688m | 81.660m -> 1.868m | **43.7x** |
| MH_02_easy | 90.187m -> 80.393m | 91.189m -> 7.491m | **12.2x** |
| MH_03_medium | — | 99.484m -> 38.054m | 2.6x |
| MH_04_difficult | 20.024m -> 13.242m (applied) | 20.024m -> 20.710m (**not applied** — gate rejected it) | — |
| MH_05_difficult | 173.556m -> 110.749m | 173.556m -> 30.766m | 5.6x |

`MH_01`/`MH_02` now dramatically exceed `plan/STAGE5.md` M3's own
"order of magnitude" bar that milestone didn't reach (2.1x-4.8x at the
time). `MH_03`/`MH_05` improve substantially too. `MH_04` is a genuine,
honest exception: at stride 2, a *different* loop candidate is detected
(the finer-grained capture changes which keyframe pairs get compared)
and this specific candidate's correction doesn't verifiably shrink the
gap — so the geometric gate (`decisions/0021`) correctly rejects it,
same as it would have at any stride. Not a regression in the gate's own
logic, just a different candidate landing on the wrong side of it.

RPE delta=1 — the metric `decisions/0021` found degrading ~5x from the
smooth-interpolation correction at stride 4 — does **not** show a clean,
uniform improvement at stride 2 (0.637m-1.663m across the 4 sequences
where a loop applied, vs. 0.815m-1.663m at stride 4 post-M3 — no
consistent direction). This is the *same* phenomenon `decisions/0025`
already documented for M3 itself: different stride values change the
pose graph's own size and which specific loop candidate gets found,
which — through the same nonlinear-LM path sensitivity already
established — can land the corrected trajectory at a genuinely different
(not uniformly better-or-worse) local optimum, not just remove an
interpolation artifact in isolation. Measured directly at stride 1 on
`MH_01_easy` before rejecting it for the real-time-bar reason: RPE
delta=1 there was 0.347m — much better than either stride 2 or 4 — so
the *interpolation-artifact* explanation is directionally right (finer
capture, smaller gaps, less to interpolate over) even though stride 2
alone doesn't cleanly demonstrate it end-to-end on every sequence.

## Decision

Ship stride 2. The real-time bar is the non-negotiable constraint here
(same framing `decisions/0021`/`plan/STAGE4.md` already established) and
stride 2 is the densest value that measurably holds it on all 5
sequences. The gap-closure win is real and substantial on most
sequences; the RPE picture is genuinely mixed, documented honestly here
rather than glossed over — a future milestone that wants a cleaner
`decisions/0021` fix would need to attack the vocabulary/place-
recognition cost directly (e.g. incremental vocabulary updates instead
of full retraining per run), not just the solver, to get stride back to 1.
