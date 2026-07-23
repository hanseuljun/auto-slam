//! Interactive human-verification demo for `plan/STAGE3.md` M1: a window
//! showing the ground-plane grid + coordinate-axes gizmo, controllable
//! with an orbit camera. This is the part of M1 that genuinely needs a
//! person looking at it (`slam-render`'s own `cargo test` already covers
//! everything checkable without eyes — camera math, offscreen render
//! pixel content) — see `plan/STAGE3.md`'s "Verifying a GUI deliverable"
//! section for why that split is the right bar for this stage, not a
//! gap in test coverage.
//!
//! Run with `cargo run -p slam-render --example orbit_demo`.
//!
//! Controls: left-drag to orbit, right-drag to pan, scroll to zoom.
//! What to look for: a gray ground-plane grid in the `XZ` plane, and a
//! red/green/blue `X`/`Y`/`Z` axes gizmo at the origin, both rotating
//! smoothly around the origin as you drag, panning under right-drag, and
//! scaling under scroll — with no flicker, tearing, or NaN-driven
//! blowups at extreme angles (in particular, drag all the way up/down to
//! confirm the near-the-pole pitch clamp doesn't glitch).

use std::sync::Arc;

use nalgebra::Point3;
use slam_render::{GpuContext, LineRenderer, OrbitCamera, Scene, DEPTH_FORMAT};
use winit::event::{ElementState, Event, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::EventLoop;
use winit::window::WindowBuilder;

fn main() -> anyhow::Result<()> {
    let event_loop = EventLoop::new()?;
    let window = Arc::new(WindowBuilder::new().with_title("slam-render orbit demo (plan/STAGE3.md M1)").with_inner_size(winit::dpi::LogicalSize::new(1024.0, 768.0)).build(&event_loop)?);

    let gpu = GpuContext::new()?;
    let surface = gpu.instance.create_surface(window.clone())?;
    let caps = surface.get_capabilities(&gpu.adapter);
    let surface_format = caps.formats.first().copied().ok_or_else(|| anyhow::anyhow!("window surface reports no supported formats"))?;

    let size = window.inner_size();
    let mut config = wgpu::SurfaceConfiguration {
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        format: surface_format,
        width: size.width.max(1),
        height: size.height.max(1),
        present_mode: caps.present_modes[0],
        alpha_mode: caps.alpha_modes[0],
        view_formats: vec![],
        desired_maximum_frame_latency: 2,
    };
    surface.configure(&gpu.device, &config);

    let mut depth_view = create_depth_view(&gpu, config.width, config.height);
    let renderer = LineRenderer::new(&gpu, surface_format);

    let mut scene = Scene::new();
    scene.add_grid(10.0, 10, [0.35, 0.35, 0.35]);
    scene.add_axes(3.0);

    let mut camera = OrbitCamera::new(Point3::origin(), 15.0, config.width as f64 / config.height as f64);

    let mut left_dragging = false;
    let mut right_dragging = false;
    let mut last_cursor: Option<(f64, f64)> = None;

    event_loop.run(move |event, elwt| {
        elwt.set_control_flow(winit::event_loop::ControlFlow::Poll);
        match event {
            Event::WindowEvent { event, window_id } if window_id == window.id() => match event {
                WindowEvent::CloseRequested => elwt.exit(),
                WindowEvent::Resized(new_size) => {
                    config.width = new_size.width.max(1);
                    config.height = new_size.height.max(1);
                    surface.configure(&gpu.device, &config);
                    depth_view = create_depth_view(&gpu, config.width, config.height);
                    camera.aspect = config.width as f64 / config.height as f64;
                }
                WindowEvent::MouseInput { state, button, .. } => {
                    let pressed = state == ElementState::Pressed;
                    match button {
                        MouseButton::Left => left_dragging = pressed,
                        MouseButton::Right => right_dragging = pressed,
                        _ => {}
                    }
                    if !pressed {
                        last_cursor = None;
                    }
                }
                WindowEvent::CursorMoved { position, .. } => {
                    let (x, y) = (position.x, position.y);
                    if let Some((lx, ly)) = last_cursor {
                        let (dx, dy) = (x - lx, y - ly);
                        if left_dragging {
                            camera.orbit(-dx * 0.005, -dy * 0.005);
                        } else if right_dragging {
                            camera.pan(-dx * 0.002, dy * 0.002);
                        }
                    }
                    last_cursor = Some((x, y));
                }
                WindowEvent::MouseWheel { delta, .. } => {
                    let scroll = match delta {
                        MouseScrollDelta::LineDelta(_, y) => y as f64,
                        MouseScrollDelta::PixelDelta(pos) => pos.y * 0.02,
                    };
                    camera.zoom((-scroll * 0.1).exp());
                }
                WindowEvent::RedrawRequested => {
                    match surface.get_current_texture() {
                        Ok(frame) => {
                            let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());
                            renderer.render(&gpu, &scene, &camera.view_projection_matrix(), &view, &depth_view, wgpu::Color { r: 0.05, g: 0.05, b: 0.08, a: 1.0 });
                            frame.present();
                        }
                        Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => surface.configure(&gpu.device, &config),
                        Err(e) => eprintln!("surface error: {e:?}"),
                    }
                }
                _ => {}
            },
            Event::AboutToWait => window.request_redraw(),
            _ => {}
        }
    })?;
    Ok(())
}

fn create_depth_view(gpu: &GpuContext, width: u32, height: u32) -> wgpu::TextureView {
    let texture = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("orbit_demo depth"),
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    texture.create_view(&wgpu::TextureViewDescriptor::default())
}
