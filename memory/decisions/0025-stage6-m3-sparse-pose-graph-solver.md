# 0025: Stage 6 M3 — sparse pose-graph solver (block Thomas + Woodbury), hand-rolled

## Context

`plan/STAGE6.md` M3's goal: replace `optimize_pose_graph`'s dense
`DMatrix`/LU solve — `O(n^3)` in the number of free nodes — with one
that exploits the pose graph's own real structure. `bin/slam-run`'s pose
graph is always: a chain of consecutive-keyframe odometry edges (block-
tridiagonal) plus exactly one loop edge (`memory/decisions/0021`'s own
sparse-capture design). `plan/STAGE6.md` M1's own doc comment recorded
the dense solve failing to even finish in 10+ minutes on `MH_01_easy`'s
full 741-keyframe trajectory (dim=4440) — the scaling bug `plan/
STAGE5.md` M2 already found reintroduces Stage 4's own O(n^3) problem
once the graph gets large, which is why `bin/slam-run` currently only
runs the pose graph over a *sparse-captured* subset
(`LOOP_CLOSURE_CAPTURE_STRIDE=4`) rather than every dense VIO keyframe.

## Decision: hand-rolled block-tridiagonal + Woodbury, not a sparse crate

The plan explicitly left this open: "hand-roll a solve exploiting this
specific structure... or bring in a sparse linear-algebra crate as
infra... a real choice to make deliberately, not default into." Tried
`nalgebra-sparse` (0.12.0) first — it pulls in `nalgebra 0.35`, while
this workspace pins `nalgebra 0.33` throughout (`SE3`/`SO3`/every
`slam-*` crate's own matrix types). `cargo add` confirmed this directly:
adding it created a **second, incompatible copy of nalgebra** in the
dependency tree, meaning every matrix/vector passed to the sparse solver
would need explicit conversion glue, plus real compile-time/binary-size
bloat for two parallel linear-algebra stacks. Reverted immediately
(`cargo remove`, restored `Cargo.lock`/`Cargo.toml`) — this isn't "infra,
same spirit as `nalgebra` itself" when it can't even share a `nalgebra`
version with the rest of the workspace.

This resolved the plan's own hedge in favor of hand-rolling: the pose
graph's real structure (a chain plus a small, fixed number of extra
edges — in practice exactly one) is exactly the kind of *simple*
sparsity pattern a general sparse-matrix library is overkill for, and
that's now confirmed to be true on dependency-compatibility grounds too,
not just algorithmic-simplicity grounds.

## The algorithm

**Block Thomas elimination** for the tridiagonal part: the free-node
Hessian (`diag[i]` = 6x6 diagonal block, `offdiag[i]` = `H[i,i+1]`) is
solved via the standard block-tridiagonal forward-elimination/back-
substitution sweep (Golub & Van Loan), `O(free_n)` block operations
instead of a dense `O(free_n^3)` factorization — generalized here to a
multi-column right-hand side (`block_tridiagonal_solve`), since the
Woodbury step below needs to solve against several right-hand sides at
once.

**Sherman-Morrison-Woodbury** for the loop edge(s): every edge's full
Gauss-Newton Hessian contribution (diagonal blocks *and* cross term) is
exactly `U_e U_e^T` for a rank-6 `dim x 6` matrix `U_e` (a direct
consequence of the normal equations always being a Gram matrix of that
edge's own stacked Jacobian — `U_e`'s only two nonzero 6x6 row-blocks are
the edge's Jacobian blocks, transposed). A "chord" edge (connecting two
free nodes that aren't adjacent in free-index order, so its cross term
doesn't fit the tridiagonal band) is therefore a rank-6 update to the
tridiagonal system; `k` chords stacked side by side give a `dim x 6k`
correction matrix `U`. Woodbury reduces `(H_tridiag + U U^T)^{-1} b` to
one multi-RHS tridiagonal solve (against `[b | U]`, `1+6k` columns) plus
a dense `6k x 6k` solve for the small correction term — negligible for
`k` on the order of 1-2, and the whole thing stays `O(free_n)` since `k`
doesn't grow with graph size. Implemented generally (arbitrary `k`), even
though production only ever has `k=1`, since the generalization cost
nothing extra once derived.

## A real bug the strict cross-check caught, that the loose one didn't

First implementation **double-counted a chord edge's diagonal
contribution**: added directly to `diag[]` (as every edge's diagonal
naturally is) *and* again via the Woodbury `U_e U_e^T` term (which
reconstructs the edge's *entire* Hessian contribution, diagonal blocks
included, not just the off-diagonal cross term). Caught by
`solve_normal_equations_matches_a_dense_solve_of_the_same_system`'s
2-chord case — a direct, isolated comparison against a hand-assembled
dense system — not by the existing end-to-end
`loop_closure_edge_corrects_accumulated_drift` test, which only checks
qualitative convergence (error decreases, node tied to the loop edge
lands close to truth) and isn't sensitive to an extra diagonal boost (it
still generally *helps* convergence stability, so the loose test kept
passing throughout). This is exactly why the milestone's own test bullet
called for matching the dense solver's output, not just re-running the
existing test unchanged — confirmed valuable in practice, not just in
principle. Fix: a chord edge's diagonal blocks are now *only* ever
supplied by the Woodbury term; `diag[]` gets nothing from it directly.

A second, smaller bug on the way: the `Chord` struct's `u_lo`/`u_hi`
fields need to store the *transposed* Jacobian blocks (`wji^T`, not
`wji`) to match the `U U^T` factorization the Woodbury correction
assumes — `offdiag`'s own formula (`u_lo^T * u_hi`) uses the raw,
untransposed blocks directly, so the same local variables can't be
reused as-is for both purposes. Both bugs were on the *first* draft, not
regressions of previously-working code — caught before commit by the two
new isolated tests, not by any accuracy regression that could have
reached `bin/slam-run`'s real output.

## Verification

- `solve_normal_equations_matches_dense_with_no_chords` (`k=0`) and
  `solve_normal_equations_matches_a_dense_solve_of_the_same_system`
  (`k=2`, two overlapping-range chords) both compare the sparse solve
  against an independently-assembled dense `DMatrix`/LU solve of the
  *exact same* system, to `1e-6` — the direct algebraic cross-check the
  milestone's own test bullet asked for.
- `loop_closure_edge_corrects_accumulated_drift` (pre-existing, k=1
  end-to-end nonlinear case) still passes unchanged.
- `solve_normal_equations_matches_a_dense_solve_with_exactly_one_chord`
  (`k=1`) added on top of the `k=0`/`k=2` cases above — the *exact*
  shape `bin/slam-run` actually uses (one loop edge), isolated in case
  Woodbury's small correction system has a bug specific to a single
  6x6 (rather than 12x12) correction the `k=2` case wouldn't exercise.
  Also passes to `1e-6`.
- `optimizes_a_741_keyframe_graph_well_within_the_real_time_budget`:
  measured (not estimated) wall-clock on a synthetic 741-node chain +
  1 loop edge, matching `MH_01_easy`'s exact full-trajectory keyframe
  count — **~97ms** for 50 LM iterations, versus the old dense solver's
  "didn't finish in 10+ minutes" on this same size (`plan/STAGE6.md`
  M1's own doc comment) — several orders of magnitude, with the entire
  real-time budget (`MH_01_easy` is 184s of data) to spare.
- **Real end-to-end run on `MH_01_easy` (current `LOOP_CLOSURE_CAPTURE_
  STRIDE=4`, unchanged) gives a measurably different result** than the
  pre-M3 baseline — worse RPE delta=1 (0.815m -> 1.460m) but a *tighter*
  loop gap-closure (81.660m -> 18.688m, vs. the old solver's -> 42.379m)
  — even though every linear-solve unit test above proves the sparse
  solve is algebraically exact (matches dense to `1e-6`) for `k=0,1,2`.
  Confirmed this isn't nondeterminism (reran `MH_01_easy` twice,
  bit-for-bit identical both times — ATE rmse 3.846m, same gap-closure
  number, same RPE, to the digit). The explanation: `optimize_pose_
  graph`'s 50-iteration Levenberg-Marquardt loop re-linearizes at the
  *current* estimate every iteration, and `compute_cost`'s accept/reject
  gate (`trial_cost < current_cost`) is a hard floating-point threshold
  — tiny (~1e-13, machine-precision-level) differences between a dense
  LU solve and this Woodbury-based one, propagated through 50 iterations
  of a nonlinear problem, can flip an accept/reject decision at some
  iteration and send the trajectory down a different (but equally valid)
  path to a different local optimum. This is the *same* class of
  behavior `memory/decisions/0023` already documented for Stage 6 M1's
  analytic-Jacobian change ("this pipeline's own hard-threshold
  decisions are genuinely sensitive to *any* change in numerical
  precision, not specific to one particular change") — not a new
  correctness concern, given the linear solve itself is independently
  proven exact. `plan/STAGE6.md` M4 (which changes `LOOP_CLOSURE_
  CAPTURE_STRIDE`) will need its own fresh before/after numbers anyway,
  since M3 changes the baseline it measures against.

## Deliberately not done: analytic edge Jacobian

`plan/STAGE6.md` M3 also considered replacing `edge_residual`'s numerical
(central-difference) Jacobian with a closed form. Not done here: a
correct derivation needs SE3's own 6x6 left/right Jacobian of the
exponential map (the coupled rotation/translation block from Barfoot or
Solà's "micro Lie theory") — this codebase has no existing machinery for
it (`SO3::left_jacobian`/`right_jacobian` are 3x3, SO3-only), and deriving
+ validating it properly is the kind of substantial, dedicated-milestone
undertaking Stage 6 M1 was for the IMU factor, not a footnote to this
one. It's also not the performance bottleneck this milestone removes —
12 cheap 6-dim residual evaluations per edge per iteration is negligible
next to the O(n^3) linear solve that's gone now.
