//! Hand-written 3D rendering library (`plan/STAGE3.md` goal 1): camera
//! math, GPU context bootstrap, and scene primitives for visualizing a
//! SLAM trajectory. `wgpu`/`winit` are infra (the GPU API and windowing),
//! same category as `nalgebra`/`image` elsewhere in this repo — the
//! camera/scene/primitive code itself is this stage's actual deliverable,
//! not a wrapper around an existing 3D engine (`memory/decisions/0018`).

mod camera;
mod gpu;
mod renderer;
mod scene;

pub use camera::OrbitCamera;
pub use gpu::GpuContext;
pub use renderer::{LineRenderer, OffscreenTarget, DEPTH_FORMAT, OFFSCREEN_COLOR_FORMAT};
pub use scene::{Scene, Vertex};
