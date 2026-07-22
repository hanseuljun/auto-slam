# M4 — IMU preintegration & VI initialization

Landed the fifth milestone from `plan/STAGE1.md`, following M0-M3. This
session picked up after an internet disconnection rolled back an earlier
in-progress attempt at M4 — rebuilt from scratch, faster the second time
since the debugging lessons carried over even though the code didn't.

## What's done

- `slam-imu::Preintegration`: on-manifold IMU preintegration (Forster et
  al.), ΔR/Δv/Δp accumulation plus first-order bias Jacobians, validated
  by finite-difference-vs-reintegration tests (same discipline as
  `slam-core`'s SO3 right-Jacobian test).
- `slam_imu::find_stationary_window` + `static_initialize`: scans for a
  genuinely-still IMU window (rather than assuming sample 0) and estimates
  gyro bias + gravity from it.
- `slam_frontend::dynamic_initialize` (`vi_init.rs`): the moving-start
  case. Two stages — (1) gyro bias via a small least-squares rotation
  alignment between VO-derived and preintegrated relative rotations, (2)
  gravity + per-keyframe velocity via a linear least-squares system built
  from the standard IMU position/velocity integration equations. See
  `decisions/0005-...md` for why accelerometer bias is fixed at zero
  rather than jointly solved (confirmed exact rank deficiency, not a
  workaround for a bug).
- `bin/slam-inspect` extended with a "static IMU init" / "dynamic VI init"
  section per sequence.
- 15 new tests (6 in `slam-imu`, 9 in `slam-frontend`'s `vi_init` module
  plus a new real-MH_04 integration test), 69 workspace tests total,
  `cargo clippy --all-targets` clean.

## The bug, and how it was actually found (worth remembering as a technique)

The dynamic initializer's linear system gave a coherent-looking but wrong
answer on synthetic data with a *known* ground truth. Print-debugging
individual matrix entries was slow and inconclusive. The move that
actually worked: **plug the true `[v_i, g, b_a]` into the assembled
`(A, b)` system and check the residual directly**
(`ground_truth_satisfies_the_assembled_linear_system`, added *before*
re-attempting the fix). That immediately separated "the physics equation
is wrong" from "the solver is wrong" — the residual was nonzero, isolating
the bug to equation assembly in one step instead of many rounds of
`eprintln!`. The actual bug: the velocity equation's `g` and `b_a`
coefficients had the wrong sign (position's coefficients were
consistently sign-flipped throughout, which is harmless — flipping an
entire equation by -1 doesn't change its solution — but velocity's were
inconsistently mixed, which isn't harmless). Once the residual check
passed, a *second*, different issue remained (the rank deficiency covered
in `decisions/0005`), caught by checking singular values directly rather
than assuming "residual is zero" meant "the solve is correct" — those are
different questions (correct equations can still have a non-unique
solution).

**Takeaway for future numerically-heavy code in this repo**: when a linear
system's solved answer doesn't match a synthetic ground truth, check (a)
does ground truth satisfy `A*x=b` (equation-construction bug if not), then
(b) is `A` full rank (observability/degeneracy issue if not) — in that
order, before spending time on print-debugging the assembly loop itself.

## Real-data checkpoint

`dynamic_initializer_converges_on_mh04_moving_start`
(`crates/slam-frontend/src/lib.rs`) runs stereo VO over ~5s of real
MH_04_difficult, feeds the resulting keyframes + raw IMU into
`dynamic_initialize`, and checks the recovered gravity magnitude lands
within ~2 m/s² of 9.81 and gyro bias is plausible. Passes, but the margin
is real: `slam-inspect` across all five sequences shows dynamic-init
gravity magnitudes ranging **5.18 to 10.5** (not a tight cluster around
9.81) — this is an unrefined linear initializer with no gravity-magnitude
constraint and accel bias pinned at zero, fed real noisy VO input, so a
wide spread is expected, not alarming. Don't read more precision into
these numbers than they have; M5's backend is what turns this bootstrap
into an actually-accurate estimate.

## Real-data finding: the plan's static/dynamic sequence grouping is a
## rough label, not a per-sequence guarantee

Checked via `slam-inspect` across all five sequences (see
`notes/dataset-quirks.md`): MH_01 and MH_05 have a findable stationary
window (MH_05's is at sample 0, genuinely stationary from the start),
**MH_04 also has one** (~12.6s in, despite being the "difficult/moving"
sequence), but **MH_02 and MH_03 have none** under
`find_stationary_window`'s threshold. Always try the static path first and
fall back to dynamic on `None`, regardless of which `MH_*` sequence.

## Not done yet (correctly out of scope for M4)

- Gravity-magnitude-constrained refinement of the dynamic initializer —
  deferred to M10 if error analysis shows the current spread matters.
- Real accelerometer bias estimation — M5's backend.
- IMU preintegration covariance propagation — M5, its first real consumer
  (already deferred once, at M4's start; still true).
