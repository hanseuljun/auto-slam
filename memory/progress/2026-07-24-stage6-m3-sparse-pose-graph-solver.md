Stage 6 M3 done: replaced `optimize_pose_graph`'s dense O(n^3) `DMatrix`/
LU solve with one exploiting the pose graph's real structure (a chain of
odometry edges plus a small number of loop edges).

Tried `nalgebra-sparse` as infra first (the plan's other option) — it
pulls in `nalgebra 0.35` alongside this workspace's pinned `0.33`, a real
version conflict, not a preference. Reverted immediately, hand-rolled
instead: block-tridiagonal (block Thomas) elimination for the chain, plus
a Sherman-Morrison-Woodbury low-rank correction for loop edges (every
edge's Hessian contribution is exactly `U U^T` for a rank-6 factor, so a
non-adjacent "chord" edge is just a rank-6 update to the tridiagonal
system).

Verified against an independently-assembled dense solve for k=0/1/2
chords — all match to 1e-6. This strict check caught a real bug (a
chord's diagonal contribution was double-counted: once directly, once
via its Woodbury term) that the existing loose end-to-end test completely
missed, since extra diagonal regularization doesn't break qualitative
convergence.

Measured wall-clock on a synthetic 741-node graph (matching MH_01_easy's
full trajectory size): ~97ms for 50 LM iterations, versus the old dense
solver's "didn't finish in 10+ minutes" on this exact size — several
orders of magnitude, the entire point of this milestone.

A real MH_01_easy end-to-end run (current stride, unchanged) gives a
deterministic but different result than before (worse RPE, better loop
gap-closure) despite the linear solve being independently proven exact —
confirmed via two identical reruns, not nondeterminism. Same class of
"hard-threshold decisions sensitive to any numerical-precision change"
already documented for M1 (decisions/0023), not a new correctness
concern.

Deliberately deferred: the analytic edge Jacobian (needs SE3's own 6x6
left/right Jacobian of the exponential map, machinery this codebase
doesn't have — a Stage-6-M1-sized undertaking on its own, and not the
performance bottleneck this milestone removes).

Full details: memory/decisions/0025.
