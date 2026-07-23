---
name: stage3-render-lib-hand-written-ui-chrome-off-shelf
description: plan/STAGE3.md's dependency-policy split for visualization — the 3D rendering library (camera/scene/primitives) is hand-written on top of wgpu, but app UI chrome and 2D graph widgets use egui/egui_plot as infra, same spirit as Stage 1 allowing plotters/CSV instead of a hand-rolled plotting library.
metadata:
  type: decision
---

# Decision: `slam-render` is hand-written, UI chrome/graph widgets are off-the-shelf

## Decision

`plan/STAGE3.md`'s goal 1 ("a 3D rendering library that can visualize
the trajectory") is met by a new `slam-render` crate with hand-written
camera math, scene/primitive abstraction, and trajectory-drawing logic
— same "own the algorithm, buy the infra" split [[dependency-and-modality-policy]]
established for Stages 1-2. Concretely:

- **Hand-written** (this stage's actual deliverable): camera view/
  projection math and orbit/fly controls, the scene-graph/primitive
  abstraction (how polylines/points/frustums become GPU draw calls),
  trajectory-specific drawing logic.
- **Infra, not the algorithm**: `wgpu` (GPU API) + `winit` (windowing) —
  the graphics-stack equivalent of Stage 1's `nalgebra`/`image`. Building
  a graphics driver or windowing system from scratch would mirror Stage
  1's explicitly-rejected "reimplement PNG decoding" alternative.
- **Also infra**: `egui`/`eframe` for the application's UI chrome
  (panels, run-picker widgets, scrub bar) and `egui_plot` for the graphs
  panel's 2D charts. These aren't "the 3D rendering library" the user
  asked for — they're UI widgets, the same category Stage 1 already
  allowed `plotters`/CSV for `slam-eval` output rather than hand-rolling
  a plotting library.

## Alternatives considered

- Pull in an existing 3D scene engine (`bevy`, `three-d`, `kiss3d`)
  instead of `slam-render`: rejected — this would skip goal 1 entirely,
  not satisfy it; the user explicitly asked for a rendering *library*,
  paralleling how Stage 1 explicitly asked for hand-written SLAM
  algorithms rather than pre-built ones.
- Hand-roll the 2D graph plotting too, for maximum "from scratch"
  consistency: rejected as scope creep relative to what was actually
  asked — goal 1 names the *3D* rendering library specifically; the
  graphs are one of three things goal 2's application shows *next to*
  that 3D view, not a second rendering-library deliverable. Consistent
  with Stage 1 not hand-rolling `plotters`/CSV export either.
- Build a fully custom immediate-mode GUI framework instead of `egui`:
  rejected for the same reason as rebuilding a graphics driver — pure
  UI chrome infra, not visible to the user as "the rendering library."

## Implications for later work

- If a future stage wants a genuinely custom-styled UI (not `egui`'s
  look), that's a real scope change to revisit explicitly, not a quiet
  swap — `bin/slam-viz`'s panels are built against `egui`'s widget model.
- `slam-render` is scoped for trajectory visualization (polylines,
  points, pose markers) — not a general mesh/dense-reconstruction
  renderer, matching `plan/STAGE3.md`'s carried-forward "no dense/mesh
  reconstruction" out-of-scope item. Extending it for meshes later is a
  new decision, not assumed.

## Source

Plan authored per the user's three explicit asks (3D rendering library,
visualization app showing trajectory+video+graphs, per-run browsing),
`plan/STAGE3.md` written this session.
