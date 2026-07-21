---
name: pnp-via-dlt-plus-refinement-not-p3p-epnp
description: M1's pose-from-points solver is linear DLT camera resectioning + Gauss-Newton SE3 refinement, not literal P3P/EPnP; minimal-point solvers are deferred to M3/M4.
metadata:
  type: decision
---

# Decision: PnP is DLT (>=6 points) + SE3 Gauss-Newton refinement, not P3P/EPnP

## Decision

`slam_geometry::estimate_pose_dlt` + `refine_pose_gauss_newton`
(`crates/slam-geometry/src/pnp.rs`) implement pose-from-points as: linear
camera resectioning via the classic 12-unknown DLT (needs >= 6 non-coplanar
points, solved via SVD nullspace, with the standard `det(M) < 0` sign-flip
trick to simultaneously fix the projective sign ambiguity and guarantee a
proper rotation), then polished by Gauss-Newton on the SE(3) manifold using
`slam_core::SO3::hat`/`SE3::exp` for the pose Jacobian (`[I | -hat(p)]`,
left-multiplicative update).

`plan/STAGE1.md`'s M1 section names "P3P/EPnP" specifically. Those are
*minimal-point* solvers (3-4 points) designed to be the inner loop of a
RANSAC-based robust estimator. Stage 1 doesn't have a RANSAC consumer for
them yet — that's M3 (stereo frontend track initialization) / M4 (dynamic
VI initializer). Implementing P3P/EPnP now, with no caller and no way to
meaningfully test them beyond synthetic minimal-set correctness, was judged
premature relative to the plan's own "land as a working, tested increment"
guidance.

## Alternatives considered

- **Implement P3P (Kneip's or Grunert's closed-form solution) now**:
  matches the plan's literal wording, and is genuinely needed eventually.
  Rejected for *now* only — real risk of a subtle closed-form-solver bug
  going unnoticed without a RANSAC harness exercising it against noisy/
  outlier data, which doesn't exist until M3/M4.
- **Implement EPnP now**: same reasoning — more general (N >= 4 points, no
  minimal-set branching) but still primarily valuable as a RANSAC inner
  loop or a faster alternative to DLT+refine, neither of which exists yet.

## How to apply

When M3 (stereo initializer / track-loss recovery) or M4 (dynamic VI
initializer) need a RANSAC pose estimator, implement P3P then, with the
RANSAC loop as the first real caller and test harness (inject outliers,
verify inlier recovery) rather than testing it in isolation. Don't assume
`estimate_pose_dlt` is RANSAC-safe as-is — it has no outlier rejection and
needs >= 6 points, both wrong for a RANSAC minimal solver.

See also [[event-stream-models-three-independent-streams]] for the sibling
M1 decision on cam0/cam1 pairing.
