---
name: stage3-m2-trajectory-pose-primitives
description: Stage 3 M2 done — slam-render gained point/pose-marker scene primitives (SE3-based camera markers, crosshair landmark markers), and slam-eval gained read_trajectory_csv as the data-adapter's CSV half, deliberately kept out of slam-render to respect the plan's crate-dependency boundary.
metadata:
  type: progress
---

# Stage 3 M2 — trajectory & pose-graph primitives (done)

## What landed

- `Scene::add_point_marker`/`add_point_markers` (`crates/slam-render/
  src/scene.rs`): a 3-line crosshair centered on a point — this crate's
  stand-in for "a point," since `LineList`-only rendering (no dedicated
  point-sprite pipeline, out of scope for this milestone) can't reliably
  show a single-pixel dot.
- `Scene::add_pose_marker`: local axes (red/green/blue, same convention
  as `add_axes`) plus a small pyramid wireframe (apex at the camera
  center, square base along local `+Z`) at an `slam_core::SE3` pose —
  the concrete reason `slam-render` takes `slam-core` as a dependency
  (added to `Cargo.toml` this milestone), using `SE3::transform` to map
  local marker geometry into world space. Explicitly schematic, not a
  calibrated FOV frustum (doesn't touch `slam-geometry` intrinsics) —
  the plan's own "`slam-render` depends on `slam-core` only" layout note
  made that boundary explicit, not an oversight.
- `slam_eval::read_trajectory_csv` + `TrajectoryPoints` (`crates/
  slam-eval/src/report.rs`): the exact inverse of the existing
  `write_trajectory_csv`, round-trip tested.

## A real scope refinement, not just an implementation detail

`plan/STAGE3.md`'s original M2 text said "a data adapter from
`slam-eval`'s trajectory/ATE types ... into `slam-render` scene
objects," which reads like it should live inside `slam-render`. But the
plan's own workspace-layout section already said `slam-render` depends
on `slam-core` only, *not* `slam-eval`/`slam-dataset` — those two
statements are in tension, and the dependency-boundary one is the
better rule to keep (it's what keeps `slam-render` a genuinely general,
reusable 3D library rather than one more crate coupled to this
pipeline's specific CSV format). Resolved by splitting the adapter: the
CSV-reading half (`read_trajectory_csv`) lives in `slam-eval`, which
already owns `write_trajectory_csv`; the actual "read a run and call
`Scene::add_polyline`" glue is one line and belongs wherever both
crates are already dependencies — `bin/slam-viz` (M3), not `slam-render`
itself. Updated `plan/STAGE3.md`'s M2 section to record this
explicitly, and moved M2's originally-scoped "visual confirmation with
a real trajectory" to M3 (where a real run can actually be loaded) —
this milestone's own render test and `orbit_demo` update use a
synthetic spiral trajectory instead.

## Verification

5 new `slam-render` tests (2 point-marker: crosshair geometry, linear
scaling with point count; 2 pose-marker: identity-pose geometry matches
local-frame math, translating the pose translates every vertex by
exactly the same amount — a real check against `SE3::transform`, not
just "it doesn't panic"; 1 GPU offscreen-render test combining a
polyline + pose markers + point markers, confirming they render
together without conflict) — `slam-render` 14 -> 19. 1 new `slam-eval`
test (`read_trajectory_csv` round-trips `write_trajectory_csv` on
non-trivial float values) — `slam-eval` 19 -> 20. `cargo clippy
--workspace --all-targets` clean. `examples/orbit_demo.rs` extended
with the same synthetic trajectory + pose/point markers, so the
existing "run it yourself" human-verification path (`plan/STAGE3.md`'s
"Verifying a GUI deliverable") now shows M2's primitives too, not just
M1's grid/axes.

Next: `plan/STAGE3.md` M3 (`bin/slam-viz` application shell + 3D panel
— this is where `read_trajectory_csv` and `Scene` actually meet for the
first time, loading a real run).
