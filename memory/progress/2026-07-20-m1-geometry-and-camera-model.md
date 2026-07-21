# M1 — geometry & camera model

Landed the second milestone from `plan/STAGE1.md`, following M0.

## What's done

- `slam-core` got real content (was a stub after M0, since it had no
  consumer yet): `SO3` (quaternion-backed, own exp/log/hat/vee/right+left
  Jacobian implementations) and `SE3` (built on `SO3`, own exp/log via the
  left-Jacobian `V` matrix). 9 unit tests: roundtrips, compose/inverse
  identities, and — per the plan's "optimizer bugs are silent" risk — a
  finite-difference check of the SO3 right Jacobian, since that's exactly
  the kind of bug that "converges to a worse optimum" silently.
- `slam-geometry`: `PinholeCamera` (radtan distort/undistort via
  fixed-point iteration), `StereoRig`/`StereoRectification` (Bouguet-style
  rectification: rectifying rotations, shared rectified intrinsics,
  baseline), `triangulate_linear`/`triangulate_refine` (DLT + Gauss-Newton
  reprojection refinement, N-view), `estimate_pose_dlt`/
  `refine_pose_gauss_newton` (camera resectioning + SE3 Gauss-Newton —
  see `decisions/0003-...md` for why this is DLT+refine rather than
  literal P3P/EPnP). 14 unit tests plus 2 integration tests that round-trip
  synthetic points through the *real* MH_01 calibration (sub-mm/near-zero
  recovery) rather than only synthetic-camera tests.
- Extended `bin/slam-inspect` (per CLAUDE.md's mandate to grow the same
  app, not spawn new demos) with a "stereo rectification" section per
  sequence: baseline, rectified intrinsics, and a live triangulation
  round-trip check using the real calibration. Verified baseline ≈ 0.110m
  (matches known EuRoC MH baseline) across all five sequences.
- `cargo test --workspace` (31 tests) and `cargo clippy --all-targets`
  both clean.

## Gotchas hit during verification

- The first PnP test used a scene where all points satisfied
  `z = 5 + 0.3x - 0.2y` — i.e. exactly coplanar. Camera resectioning via the
  full 12-parameter DLT is a known-degenerate problem for coplanar points;
  the linear-only recovery was off by several degrees of rotation even
  noise-free. Fixed by adding a nonlinear (`x*y`, `x^2*y`) term to break
  coplanarity. Worth remembering for any *future* PnP/DLT test fixture too.

## Not done yet (correctly out of scope for M1)

- P3P/EPnP and 8-point/5-point relative pose + RANSAC — deferred to M3/M4
  where a RANSAC consumer actually exists; see `decisions/0003-...md`.
- Sim3 in `slam-core` — still no consumer; comes in M9 (trajectory
  alignment).
- Actual image rectification (remapping pixels) and real-feature-match
  disparity verification — M1's rectification test instead proves the
  *mathematical* property (any 3D point projects to equal rows in both
  rectified cameras, verified both synthetically and via real MH_01
  extrinsics) since M2's tracker doesn't exist yet to produce real matches.
