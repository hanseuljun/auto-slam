---
name: stage3-m1-slam-render-foundations
description: Stage 3 M1 done — new slam-render crate (hand-written orbit camera math, wgpu/winit bootstrap, line/grid/axes primitives), verified with real headless GPU offscreen-render tests, not just camera-math unit tests.
metadata:
  type: progress
---

# Stage 3 M1 — `slam-render` foundations (done)

## What landed

- `crates/slam-render/src/camera.rs`: `OrbitCamera` (target/distance/yaw/
  pitch state), hand-written `look_at_rh` view matrix and a `perspective_
  wgpu` projection matrix (not `nalgebra::Perspective3` — that assumes
  OpenGL's `z in [-1,1]` clip range, wgpu uses `[0,1]`, a real, narrow
  case of "own the rendering math" per `decisions/0018`). `orbit`/`pan`/
  `zoom` mutators, pitch clamped strictly inside +-90 degrees (exactly at
  the pole, `forward.cross(up)` degenerates and the camera basis becomes
  undefined — a real bug class, not a cosmetic guard, per the `pitch`
  field's own doc comment). 7 unit tests, all pure math, no GPU needed:
  eye position at known angles, look-at-target projects to viewport
  center, near/far planes map to wgpu's `0`/`1` depth, pan moves along
  camera-local (not world) axes, pitch clamp holds under an absurdly
  large input delta without producing NaNs.
- `gpu.rs`: `GpuContext` (instance/adapter/device/queue). Before writing
  this, ran a throwaway probe (`wgpu::Instance::request_adapter` in a
  standalone scratch binary) to check whether this repo's dev machine
  can get a GPU adapter *without* a window/display attached — confirmed
  yes (`AdapterInfo { name: "Apple M1", ..., backend: Metal }`), which
  de-risked the rest of M1 considerably: `plan/STAGE3.md`'s own Risks
  section had flagged headless GPU testing as environment-dependent, and
  measuring this *before* committing to a testing strategy (rather than
  discovering it mid-implementation) meant the offscreen tests below
  could be written as real pixel-content assertions from the start, not
  a fallback "just check it doesn't panic."
- `scene.rs`: `Scene`/`Vertex` — line-segment primitives (`add_line`,
  `add_polyline`, `add_grid`, `add_axes`). `add_polyline` is technically
  M2 scope (`plan/STAGE3.md` lists it under "trajectory primitives") but
  fell out for free once single-line-segment support existed, so it
  landed here instead of being deferred artificially.
- `renderer.rs`: `LineRenderer` (a `wgpu::RenderPipeline` for
  `PrimitiveTopology::LineList` against a WGSL shader in `shaders/
  line.wgsl`) and `OffscreenTarget` (color+depth texture pair with a
  real `read_pixels_rgba8` — GPU texture -> padded buffer -> CPU `Vec<u8>`
  round trip, handling wgpu's `COPY_BYTES_PER_ROW_ALIGNMENT` padding).
  2 tests: an empty scene renders as a uniform clear color (sanity check
  on the readback pipeline itself, independent of any scene content),
  and a grid+axes scene produces non-background pixels generally *and*
  specifically in a small window around the viewport center (where the
  axes gizmo's origin — the look-at target — must project). The first
  version of that second test asserted the *exact* center pixel, which
  failed: the blue Z-axis line points directly at the camera from the
  demo's default viewpoint, rasterizing to a near-zero-length line whose
  exact covered pixel(s) aren't something to pin down bit-for-bit —
  loosened to a 9x9-pixel neighborhood check, which is robust to that
  and still tests the real thing (the gizmo appears where geometry says
  it should).
- **Real bug caught before it could bite**: `LineRenderer::new` initially
  hardcoded `wgpu::TextureFormat::Rgba8Unorm` into the pipeline's
  fragment target, matching `OffscreenTarget`. A window surface's native
  format is platform-negotiated (`surface.get_capabilities(...)`) and
  often isn't that — e.g. `Bgra8UnormSrgb` is common on macOS — so the
  very first windowed render would have hit a wgpu pipeline/target
  format-mismatch validation error. Fixed by making `color_format` a
  parameter of `LineRenderer::new` instead of a fixed constant, renamed
  the offscreen-specific constant to `OFFSCREEN_COLOR_FORMAT` to make
  the distinction explicit. Caught by writing the windowed example next
  and reasoning through what it would need, not by a test — a case
  where the "verifying a GUI deliverable" split (push what's testable
  into `cargo test`, but still *think through* the untestable part
  carefully) mattered.
- `examples/orbit_demo.rs`: a `winit` window (mouse-drag orbit, right-
  drag pan, scroll zoom) showing the grid+axes scene — the actual
  human-verification artifact for this milestone. Builds and
  `cargo clippy`-clean. **Not run by the agent**: driving/observing a
  live GUI window isn't something this session's tools can do (no way
  to see pixels on a real screen or send mouse/keyboard events to a
  window), and launching one unprompted would pop up on the user's real
  desktop — left for the user to run themselves via `cargo run -p
  slam-render --example orbit_demo` per `plan/STAGE3.md`'s own
  "Verifying a GUI deliverable" section, which anticipated exactly this
  split.

## Verification

`cargo test -p slam-render`: 14/14 passing (7 camera, 3 scene, 1 gpu
context, 2 renderer/offscreen, plus the pipeline-format fix verified by
rerunning after the change). `cargo build`/`cargo clippy` clean for both
the library and the `orbit_demo` example. Full workspace `cargo test`/
`cargo clippy` re-run after adding the new crate to confirm no
regressions elsewhere (unrelated to `slam-render`'s own code, just
confirming the new workspace member and its new dependencies —
`wgpu`/`winit`/`bytemuck`/`pollster` — don't break anything else).

Next: `plan/STAGE3.md` M2 (trajectory & pose-graph primitives — camera
frustum/keyframe markers, and a data adapter from `slam-eval`'s
`RunMeta`/trajectory CSV types into `Scene` objects, building on M1's
`add_polyline`).
