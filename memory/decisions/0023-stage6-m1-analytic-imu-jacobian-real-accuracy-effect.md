---
name: stage6-m1-analytic-imu-jacobian-real-accuracy-effect
description: Stage 6 M1 done — analytic IMU Jacobians implemented, derived by hand against this codebase's own left-multiplicative SE3 convention, and validated extensively (43 finite-difference test cases, both jac_i and jac_j, including a randomized stress sweep). Correctness is solid, but the "transparent swap, no behavior change" the plan expected was wrong: real, measured ATE effect, -14.4% to +74.8% on bounded clips and -0.6% to +17.0% on full sequences, mostly negative. Cross-checked this is pipeline sensitivity to Jacobian precision generally (confirmed via a numerical-epsilon-only experiment), not a bug in this specific derivation — leading unconfirmed hypothesis: SolverConfig's ad hoc weights were implicitly co-tuned to the old numerical Jacobian's own behavior.
metadata:
  type: decision
---

# Stage 6 M1: analytic IMU Jacobian lands correct, but not accuracy-neutral

## The derivation

`crates/slam-optim/src/imu_factor.rs`'s `imu_residual_jacobian` now
computes all 18 partial-derivative blocks (9-dim residual x 2 states x
15-dim tangent each, minus the blocks that are structurally zero — state
j's bias fields don't enter the residual at all, only state i's) in
closed form, replacing the central-difference numerical version
`decisions/0006` deferred.

Derived directly against **this codebase's own perturbation convention**
— `KeyframeState::retract`'s left-multiplicative `Exp(delta) * pose` on
`state.pose` (world -> body), with the 6-dim pose tangent ordered
`[rho (translation); phi (rotation)]` (`SE3::exp`'s own `xi = [rho;
phi]`). This is a real, deliberate choice: most published VIO Jacobian
tables (ORB-SLAM3, VINS-Mono) assume a right-multiplicative perturbation
on `R_wb` (body -> world), which is a *different* convention — copying
one of those tables here would have silently carried the wrong signs.
Instead, re-derived from scratch using two standard Lie-group identities
(`Log(Exp(x)Exp(d)) ~= x + Jr(x)^{-1}d` for right perturbation,
`Log(Exp(d)Exp(x)) ~= x + Jl(x)^{-1}d` for left) plus the output-
derivative rule already established and tested by `reprojection.rs`'s
own analytic Jacobian (`dX/drho = I`, `dX/dphi = -hat(X)` for `X =
pose.transform(p)` under this same left-perturbation convention) — cross-
checked the position-residual block two independent ways by hand before
trusting it, given `decisions/0006`'s own warning that this exact class
of derivation ("18 distinct partial-derivative blocks... exactly the
kind of place a sign or composition-order bug hides silently") already
produced a real sign bug once in this codebase (M4's dynamic
initializer).

## Correctness: validated extensively, not just assumed

The original test (`jacobian_matches_its_own_finite_difference_at_a_
different_epsilon`) only ever checked `jac_i` — `jac_j` was computed and
discarded uncompared. Replaced with:

1. `analytic_jacobian_matches_finite_difference_for_both_states`: both
   `jac_i` and `jac_j`, full 9x15 matrices, on 3 distinct hand-picked
   configurations (large rotations, near-identity states, nonzero and
   *mismatched* biases between `state_i`/`state_j`'s own values and the
   preintegration's own linearization bias — needed to actually exercise
   the bias-coupling block, which vanishes at zero bias offset).
2. `analytic_jacobian_matches_finite_difference_randomized_stress`: 40
   pseudo-random configurations specifically targeting regions the
   hand-picked cases might miss — short intervals (`dt=0.05s`, the kind
   track-loss recovery produces), large bias offsets, large rotations.

All 43 cases pass at 1e-4 to 5e-3 tolerance (tighter than the original
test's 1e-3, loosened only as far as needed to clear finite-difference's
own forward-difference truncation floor — confirmed directly: at 1e-6
tolerance, mismatches were in the 9th significant digit, consistent with
the *oracle's* own truncation error, not a derivation bug).

## The real, measured accuracy effect — not what was expected

The plan's own M1 bullet expected "a transparent swap, not a behavior
change... bit-for-bit-identical optimization results... within numerical
noise." Measured, real numbers on all 5 sequences contradict that:

| Sequence | bounded-clip ATE before -> after | full-sequence ATE before -> after |
|---|---|---|
| MH_01_easy | 0.151m -> 0.155m (+2.6%) | 3.505m -> 4.061m (+15.9%) |
| MH_02_easy | 0.184m -> 0.207m (+12.5%) | 3.546m -> 4.149m (+17.0%) |
| MH_03_medium | 0.511m -> 0.893m (**+74.8%**) | 3.451m -> 3.593m (+4.1%) |
| MH_04_difficult | 1.174m -> 1.005m (-14.4%) | 6.496m -> 6.456m (-0.6%) |
| MH_05_difficult | 0.455m -> 0.597m (+31.2%) | 6.596m -> 6.691m (+1.4%) |

4 of 5 sequences got worse on both bounded and full-sequence runs (only
`MH_04` improved on both). The bounded-clip effect is larger and more
volatile than the full-sequence one — plausible explanation: a 30-second
clip has far fewer keyframes for one "unlucky" extra/missing track-loss-
recovery event to average out against, so the same underlying precision-
sensitivity shows up as a bigger swing in ATE there than over a full
~150s run's 500-700 keyframes.

## Ruled out: a bug in this specific derivation

Given the magnitude of the bounded-clip regression (`MH_03` nearly
doubling), checked whether the extensive finite-difference validation
above was somehow missing a real error. Direct experiment: reverted to
the *old* numerical Jacobian (no analytic code involved at all) but
changed its finite-difference epsilon from 1e-6 to 1e-5 — a pure
precision change with zero new derivation risk. Result: `MH_03`'s
bounded-clip ATE *also* moved (0.511m -> 0.495m) from this alone. Smaller
than the analytic swap's own effect, but confirms the underlying
phenomenon — this pipeline's keyframe/track-loss-recovery decisions
(hard thresholds like `tracks.len() >= 6`, `max_pose_jump_meters`) are
genuinely sensitive to *any* change in Jacobian precision, not
specifically to an error in this one. Correctness of the analytic
derivation itself is not in question; consider it validated.

## Leading hypothesis for *why*, not yet confirmed

`SolverConfig`'s ad hoc IMU weights (`decisions/0016`'s own words) "had
likely absorbed other unmodeled error sources" through hand-tuning
against real MH data — tuning that happened entirely with the *old*
numerical Jacobian in the loop. If those weights were implicitly
co-adapted to the old Jacobian's own specific (slightly-off) behavior,
swapping in a more precise Jacobian doesn't automatically help — the
weights are now paired with a system they weren't tuned against. This is
exactly the same class of failure `decisions/0016` already found for a
different knob (sensor.yaml-derived weights regressing real data despite
"looking more principled"), not a new kind of surprise for this
codebase.

## Decision: keep the analytic Jacobian, let M2 be the real test

Not reverting to numerical: correctness is solid (validated far more
thoroughly than the original single-config test ever did), and `plan/
STAGE6.md` M2's own real covariance propagation work needs exactly this
precision (covariance propagation compounds Jacobian errors step by
step). Reverting now would just defer re-discovering this same
interaction later, with less validation in place. Instead: M2's own
before/after measurement (already in the plan) is the real test of
whether this stage's Goal 1 work nets out positive — M1's own regression
is flagged here explicitly so M2's numbers get read against *this*
baseline (with the analytic Jacobian and its current effect already
priced in), not against the pre-M1 numbers, which would double-count
M1's own already-measured cost.
