---
name: dynamic-init-fixes-accel-bias-at-zero
description: The M4 dynamic (moving-start) VI initializer solves for velocities + gravity only, with accelerometer bias fixed at zero, because jointly solving for accel bias made the linear system exactly rank-deficient.
metadata:
  type: decision
---

# Decision: dynamic VI initializer fixes accelerometer bias at zero

## Decision

`slam_frontend::dynamic_initialize`'s stage 2
(`solve_gravity_bias_velocity` in `crates/slam-frontend/src/vi_init.rs`)
solves the linear least-squares system for `[v_0..v_{k-1}, g]` only.
Accelerometer bias is *not* a joint unknown — it's fixed at zero (both in
the linear solve and in the `Preintegration` calls that feed it).

## Why

Jointly solving for `[v_i, g, b_a]` (the textbook formulation, matching
what `plan/STAGE1.md` M4 describes: "gravity, scale confirmation... accel
bias, and initial velocities") makes the system's coefficient matrix
**exactly** rank-deficient by one — confirmed empirically, not assumed:
a synthetic scenario with substantial rotation (~1 rad total, 6 keyframes
over 3s) whose ground-truth `[v_i, g, b_a]` was independently verified to
satisfy every row of the assembled system exactly
(`ground_truth_satisfies_the_assembled_linear_system` test) still produces
a zero singular value when solved jointly
(`debug_singular_values_reveal_rank_deficiency` test, both in
`crates/slam-frontend/src/vi_init.rs`). So *some* direction in
`(v_i, g, b_a)`-space is fundamentally unobservable from position+velocity
constraints alone over a short window — not a bug to fix, a real property
of the problem given only a handful of keyframes.

This matches practical experience elsewhere: several published VIO
initializers (e.g. VINS-Mono's initial linear alignment) also treat accel
bias as small/negligible in the bootstrap stage rather than solving for it
jointly, deferring real bias estimation to the nonlinear backend that
follows.

## Alternatives considered

- **Solve `[v_i, g, b_a]` jointly anyway, using the SVD pseudo-inverse's
  minimum-norm solution to handle the null space**: rejected — the
  minimum-norm solution is an arbitrary point along the null space
  direction, not a meaningful estimate; empirically it recovered gravity
  vectors wildly far from the true value (tested: true `|g|≈9.79`,
  recovered `|g|≈2.1` to `10.5` depending on scenario).
- **Add the gravity-magnitude-constrained refinement pass** (as ORB-SLAM3/
  VINS-Mono do on top of their linear stage) to break the degeneracy:
  would likely help, but is real additional work beyond a first working
  M4 checkpoint — explicitly deferred to M10 ("accuracy closing pass") if
  error analysis later shows it's needed.
- **Use a longer initialization window** (more keyframes, more motion
  diversity) to try to make the joint system full-rank: plausible in
  principle but unverified, and conflicts with M4's own bar ("converges
  within the first few seconds" — plan/STAGE1.md).

## How to apply

`DynamicInitResult::accel_bias` is currently always `Vector3::zeros()` —
it's kept as a field (not deleted) so callers have one consistent result
shape, but carries no information. Don't build downstream logic that
trusts this value. Real accelerometer bias estimation for MH_04/05-style
sequences is M5's job (the sliding-window backend, with a proper nonlinear
IMU factor, more data, and the observability that a longer window + more
diverse motion provides). If M5's bias estimates come out implausible on
MH_04/05, revisit this decision rather than assuming M5's optimizer is
buggy — the initial seed of zero could be part of the story.

See also [[pnp-via-dlt-plus-refinement-not-p3p-epnp]] and
[[event-stream-models-three-independent-streams]] for the sibling pattern
of "verified via a real observability/data check, not just an oversight."
