---
name: imu-factor-uses-numerical-jacobians
description: slam-optim's IMU preintegration factor uses central-difference numerical Jacobians instead of hand-derived analytic ones, a deliberate risk/complexity tradeoff: the reprojection factor still uses analytic Jacobians.
metadata:
  type: decision
---

# Decision: IMU factor Jacobians are numerical, not analytic

## Decision

`slam_optim::imu_residual_jacobian` (`crates/slam-optim/src/imu_factor.rs`)
computes its two 9x15 Jacobian blocks via central finite differences on
`KeyframeState::retract`, not hand-derived closed-form expressions. The
sibling reprojection factor (`reprojection.rs`) *does* use an analytic
Jacobian.

## Why

The IMU factor has 9 residuals x 2 states x 15 tangent dims = 18 distinct
partial-derivative blocks to derive by hand, several involving the
preintegration bias-correction Jacobians composed with SE3 perturbations —
real derivation complexity, and exactly the kind of place a sign or
composition-order bug hides silently (`plan/STAGE1.md`'s own stated risk).
M4's dynamic initializer already produced one real sign bug in a much
simpler propagation equation (`decisions/0005`), caught only by an
explicit ground-truth-residual check built *before* trusting the result.

`Preintegration::corrected` is O(1) (first-order bias correction, no
re-integration from raw IMU samples), so the 30 extra residual evaluations
central differences need per factor per iteration are computationally
negligible at this stage's problem sizes (a handful to ~15 keyframes).
`plan/STAGE1.md` explicitly deprioritizes performance for Stage 1
("correctness and accuracy first, speed later" — M0's stated non-goal).

The reprojection factor's Jacobian, by contrast, is structurally identical
to `slam_geometry::refine_pose_gauss_newton`'s pose Jacobian (already
validated, reused directly), so deriving it analytically carried much
less risk for much less code — no reason to pay the numerical-Jacobian
cost there too.

## Alternatives considered

- **Derive the IMU factor Jacobian analytically anyway**: matches typical
  production VIO systems (ORB-SLAM3, VINS-Fusion) and would be faster;
  rejected for *now* given the derivation risk relative to the size of
  this session's remaining scope. `imu_factor.rs`'s own tests
  (`residual_is_zero_for_self_consistent_states`,
  `jacobian_matches_its_own_finite_difference_at_a_different_epsilon`)
  give confidence the *residual* and its Jacobian's *numerical stability*
  are both right; only the "is it fast enough" question is left open.

## How to apply

If profiling in a later milestone (M10, or if `slam-backend`'s window
grows large enough that solve time actually matters) shows the IMU
factor's Jacobian computation is a bottleneck, replace it with a hand-
derived analytic version — validate it the same way SO3's right Jacobian
and `Preintegration`'s bias Jacobians were validated: a finite-difference
test *before* trusting it in the solver, not after. Don't assume "it's
numerical, so it must be slow" without measuring first; at current problem
sizes it likely isn't the bottleneck.
