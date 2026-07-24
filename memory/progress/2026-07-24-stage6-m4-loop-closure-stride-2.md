Stage 6 M4 done: reduced LOOP_CLOSURE_CAPTURE_STRIDE from 4 to 2 (not
all the way to 1), measured directly rather than guessed.

Tried stride 1 first (the plan's preferred outcome now that M3's sparse
solver removed the pose-graph solve's O(n^3) cost) — measured it breaks
the real-time bar on MH_01_easy (whole-run factor 1.082). The solve
itself is cheap now (~97ms even at 741 nodes), but BoW vocabulary
training and place-recognition queries still scale with capture density
and dominate at stride 1.

Stride 2 holds the real-time bar on all 5 sequences (0.640-0.925).
Geometric gap-closure ratio improves dramatically on most sequences
(MH_01: 4.4x->43.7x, MH_02: 1.1x->12.2x, MH_05: 1.6x->5.6x) — MH_01/MH_02
now well past the "order of magnitude" bar Stage 5 M3 didn't reach.
MH_04 is a genuine exception: a different loop candidate is found at
stride 2 and the geometric gate correctly rejects it (not a bug).

RPE delta=1 does not show a clean, uniform fix of decisions/0021's ~5x
interpolation-artifact degradation at stride 2 - mixed results, no
consistent direction. Measured at stride 1 (before rejecting it) that
the interpolation-artifact explanation is directionally correct (0.347m
there, better than stride 2 or 4) - a future fix targeting the
vocabulary/place-recognition cost specifically could still reach stride
1 for a cleaner win.

Documented honestly, including the mixed parts, rather than picking only
the favorable numbers. Full details: memory/decisions/0026.
