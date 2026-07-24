---
name: stage6-m1-analytic-imu-jacobian
description: Stage 6 M1 done — implemented and extensively validated analytic IMU Jacobians (43 finite-difference test cases, both jac_i and jac_j, correctness solid), but found the plan's own "transparent swap, no behavior change" expectation was wrong: real accuracy effect measured, -14.4% to +74.8% on bounded clips, mostly negative. Ruled out a derivation bug via a control experiment (numerical Jacobian at a different epsilon alone also shifts results). Leading hypothesis: SolverConfig's ad hoc weights were implicitly co-tuned to the old numerical Jacobian's own behavior.
metadata:
  type: progress
---

# Stage 6 M1: analytic IMU Jacobian — correct, but a real accuracy surprise

## What shipped

`crates/slam-optim/src/imu_factor.rs`'s `imu_residual_jacobian` replaced
central-difference with closed-form analytic Jacobians, derived by hand
against this codebase's own left-multiplicative SE3 perturbation
convention (not copied from a textbook table, which would assume a
different, right-multiplicative one and silently carry wrong signs).

## The validation effort, and why it went further than the plan asked

The plan's own M1 bullet asked for finite-difference verification "on
real preintegration data." Started there, but the *original* test this
was replacing had a real gap worth noticing: it only ever checked
`jac_i` (`jac_j` was computed and discarded, `let _ = jac_j_a;`). Fixed
that, then — once the real pipeline showed a much bigger accuracy effect
than expected (see below) — went further still: a 40-case randomized
stress sweep targeting short intervals, large bias offsets, and large
rotations specifically, since a subtle sign error is exactly the kind of
thing that could hide in a region 3 hand-picked configs don't reach. All
43 cases pass at tight tolerance (1e-4 to 5e-3, tighter than the
original test's 1e-3).

## The real finding: correctness held, "no behavior change" didn't

Measured ATE before/after on all 5 sequences, both bounded-clip and
full-sequence: real, mostly-negative changes (bounded: -14.4% to +74.8%,
`MH_03_medium` nearly doubling; full: -0.6% to +17.0%, milder but still
real). This directly contradicted the plan's own stated expectation
("bit-for-bit-identical... within numerical noise") — worth recording
that this was a wrong assumption written into the plan itself, not just
an implementation detail.

Before accepting this, checked whether the 43-case validation had
somehow missed a real bug: reverted to the *old* numerical Jacobian (zero
analytic code) but changed only its own finite-difference epsilon
(1e-6 -> 1e-5). That alone *also* moved `MH_03`'s bounded-clip ATE
(0.511m -> 0.495m) — smaller than the analytic swap's effect, but
confirms the underlying phenomenon is real pipeline sensitivity to
Jacobian precision generally (this pipeline's hard-threshold track-loss-
recovery decisions cascade unpredictably from tiny numerical
differences), not an error specific to this derivation.

Leading hypothesis, not yet confirmed: `SolverConfig`'s ad hoc IMU
weights were hand-tuned (Stage 2 M6) entirely with the old numerical
Jacobian in the loop, and may have implicitly absorbed/compensated for
its specific (slightly imprecise) behavior — the same class of "looks
more principled, regresses real data anyway" surprise `decisions/0016`
already found for a different knob. M2's own re-weighting work
(replacing those same ad hoc weights with ones derived from real
covariance propagation) is the natural place to re-verify this, not
something to chase down further right now.

## Decision

Keep the analytic Jacobian — correctness is solid, and M2's own
covariance propagation work needs this precision (it compounds Jacobian
errors step by step, so a numerical Jacobian's own truncation error
would quietly corrupt it). M2's before/after measurement is the real
test of whether Goal 1's work nets out positive; M1's own regression is
now documented so M2 gets read against the right baseline. Full
reasoning and numbers: `memory/decisions/0023`.

## What's next

`plan/STAGE6.md` M2: real preintegration covariance propagation
(Forster et al.'s own recursion, scoped in `memory/decisions/0022`),
replacing the same ad hoc weights this milestone's own findings
implicate.
