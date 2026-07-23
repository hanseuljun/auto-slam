use nalgebra::Matrix4;
use wgpu::util::DeviceExt;

use crate::gpu::GpuContext;
use crate::scene::{Scene, Vertex};

/// Color format for `OffscreenTarget` specifically. `LineRenderer` itself
/// takes its color format as a parameter (not a fixed constant) precisely
/// because a *window surface*'s native format is platform-chosen (often
/// `Bgra8UnormSrgb` on macOS) — hardcoding one format into the pipeline
/// would panic the first time it was asked to render into a surface
/// texture of a different format than this offscreen constant.
pub const OFFSCREEN_COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
}

/// `Matrix4<f64>` -> a column-major `[[f32; 4]; 4]`, the layout WGSL's
/// `mat4x4<f32>` uniform expects (each column is already a 16-byte-aligned
/// `vec4`, so a flat column-major array needs no extra padding — unlike,
/// say, an array of `vec3`s). `OrbitCamera`'s own math stays `f64` for
/// consistency with the rest of this repo's estimation code; this is the
/// one narrowing point, right before upload.
fn to_uniform_matrix(m: &Matrix4<f64>) -> [[f32; 4]; 4] {
    let mut out = [[0.0f32; 4]; 4];
    for col in 0..4 {
        for row in 0..4 {
            out[col][row] = m[(row, col)] as f32;
        }
    }
    out
}

/// Draws a `Scene`'s line segments against a view-projection matrix, into
/// whatever color/depth attachments the caller provides — an offscreen
/// texture (this crate's own tests, see `OffscreenTarget`) or a window
/// surface (`bin/slam-viz`, `plan/STAGE3.md` M3+). This is the actual
/// "3D rendering library" primitive-drawing code (`memory/decisions/0018`),
/// as opposed to `gpu.rs`'s bare-infra device/queue bootstrap.
pub struct LineRenderer {
    pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

impl LineRenderer {
    /// `color_format` must match whatever the caller will actually render
    /// into — `OFFSCREEN_COLOR_FORMAT` for `OffscreenTarget`, or a window
    /// surface's own negotiated format (`surface.get_capabilities(...)`)
    /// for a windowed renderer.
    pub fn new(gpu: &GpuContext, color_format: wgpu::TextureFormat) -> Self {
        let shader = gpu.device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("slam-render line shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/line.wgsl").into()),
        });

        let uniform_buffer = gpu.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("slam-render view-proj uniform"),
            contents: bytemuck::bytes_of(&Uniforms { view_proj: to_uniform_matrix(&Matrix4::identity()) }),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let bind_group_layout = gpu.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("slam-render bind group layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let bind_group = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("slam-render bind group"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: uniform_buffer.as_entire_binding() }],
        });

        let pipeline_layout = gpu.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("slam-render pipeline layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 0, shader_location: 0 },
                wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 12, shader_location: 1 },
            ],
        };

        let pipeline = gpu.device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("slam-render line pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState { module: &shader, entry_point: "vs_main", buffers: &[vertex_layout], compilation_options: Default::default() },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState { format: color_format, blend: Some(wgpu::BlendState::REPLACE), write_mask: wgpu::ColorWrites::ALL })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState { topology: wgpu::PrimitiveTopology::LineList, ..Default::default() },
            depth_stencil: Some(wgpu::DepthStencilState { format: DEPTH_FORMAT, depth_write_enabled: true, depth_compare: wgpu::CompareFunction::Less, stencil: wgpu::StencilState::default(), bias: wgpu::DepthBiasState::default() }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
        });

        LineRenderer { pipeline, uniform_buffer, bind_group }
    }

    pub fn render(&self, gpu: &GpuContext, scene: &Scene, view_proj: &Matrix4<f64>, color_view: &wgpu::TextureView, depth_view: &wgpu::TextureView, clear_color: wgpu::Color) {
        gpu.queue.write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(&Uniforms { view_proj: to_uniform_matrix(view_proj) }));

        let mut encoder = gpu.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("slam-render encoder") });
        let vertex_buffer = (!scene.vertices.is_empty()).then(|| {
            gpu.device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("slam-render vertex buffer"), contents: bytemuck::cast_slice(&scene.vertices), usage: wgpu::BufferUsages::VERTEX })
        });

        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("slam-render pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment { view: color_view, resolve_target: None, ops: wgpu::Operations { load: wgpu::LoadOp::Clear(clear_color), store: wgpu::StoreOp::Store } })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment { view: depth_view, depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Store }), stencil_ops: None }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            if let Some(vb) = &vertex_buffer {
                pass.set_pipeline(&self.pipeline);
                pass.set_bind_group(0, &self.bind_group, &[]);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.draw(0..scene.vertices.len() as u32, 0..1);
            }
        }
        gpu.queue.submit(std::iter::once(encoder.finish()));
    }
}

/// An off-screen color+depth render target, for headless use — this
/// crate's own tests (no window/surface needed) and, later, any
/// screenshot/export feature. `read_pixels_rgba8` round-trips the color
/// attachment back to the CPU for pixel-level assertions.
pub struct OffscreenTarget {
    pub width: u32,
    pub height: u32,
    color_texture: wgpu::Texture,
    color_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
}

impl OffscreenTarget {
    pub fn new(gpu: &GpuContext, width: u32, height: u32) -> Self {
        let size = wgpu::Extent3d { width, height, depth_or_array_layers: 1 };
        let color_texture = gpu.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("slam-render offscreen color"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: OFFSCREEN_COLOR_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());

        let depth_texture = gpu.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("slam-render offscreen depth"),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        OffscreenTarget { width, height, color_texture, color_view, depth_view }
    }

    pub fn color_view(&self) -> &wgpu::TextureView {
        &self.color_view
    }

    pub fn depth_view(&self) -> &wgpu::TextureView {
        &self.depth_view
    }

    /// Reads the color attachment back as tightly-packed RGBA8 rows
    /// (`width * height * 4` bytes, no row padding) — `wgpu` requires
    /// `COPY_BYTES_PER_ROW_ALIGNMENT`-aligned rows for the GPU-side copy,
    /// so this strips that padding back out before returning.
    pub fn read_pixels_rgba8(&self, gpu: &GpuContext) -> Vec<u8> {
        let bytes_per_pixel = 4u32;
        let unpadded_bytes_per_row = self.width * bytes_per_pixel;
        let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let padded_bytes_per_row = unpadded_bytes_per_row.div_ceil(align) * align;

        let buffer_size = (padded_bytes_per_row * self.height) as wgpu::BufferAddress;
        let readback_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor { label: Some("slam-render readback buffer"), size: buffer_size, usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ, mapped_at_creation: false });

        let mut encoder = gpu.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("slam-render readback encoder") });
        encoder.copy_texture_to_buffer(
            wgpu::ImageCopyTexture { texture: &self.color_texture, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
            wgpu::ImageCopyBuffer { buffer: &readback_buffer, layout: wgpu::ImageDataLayout { offset: 0, bytes_per_row: Some(padded_bytes_per_row), rows_per_image: Some(self.height) } },
            wgpu::Extent3d { width: self.width, height: self.height, depth_or_array_layers: 1 },
        );
        gpu.queue.submit(std::iter::once(encoder.finish()));

        let slice = readback_buffer.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |result| {
            sender.send(result).expect("readback channel closed before receiving map result");
        });
        gpu.device.poll(wgpu::Maintain::Wait);
        receiver.recv().expect("readback map_async never signaled").expect("failed to map readback buffer");

        let padded: Vec<u8> = slice.get_mapped_range().to_vec();
        readback_buffer.unmap();

        let mut unpadded = Vec::with_capacity((unpadded_bytes_per_row * self.height) as usize);
        for row in 0..self.height as usize {
            let start = row * padded_bytes_per_row as usize;
            let end = start + unpadded_bytes_per_row as usize;
            unpadded.extend_from_slice(&padded[start..end]);
        }
        unpadded
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::OrbitCamera;
    use nalgebra::Point3;
    use slam_core::{SE3, SO3};

    #[test]
    fn offscreen_render_of_grid_and_axes_produces_non_background_pixels() {
        let gpu = GpuContext::new().expect("GPU context required for this test");
        let target = OffscreenTarget::new(&gpu, 256, 256);
        let renderer = LineRenderer::new(&gpu, OFFSCREEN_COLOR_FORMAT);

        let mut scene = Scene::new();
        scene.add_grid(5.0, 5, [0.4, 0.4, 0.4]);
        scene.add_axes(3.0);

        let camera = OrbitCamera::new(Point3::origin(), 10.0, target.width as f64 / target.height as f64);
        renderer.render(&gpu, &scene, &camera.view_projection_matrix(), target.color_view(), target.depth_view(), wgpu::Color::BLACK);

        let pixels = target.read_pixels_rgba8(&gpu);
        assert_eq!(pixels.len(), (target.width * target.height * 4) as usize);

        let non_background = pixels.chunks_exact(4).filter(|p| p[0] > 10 || p[1] > 10 || p[2] > 10).count();
        assert!(non_background > 0, "expected at least one non-background pixel from the grid/axes lines, rendered scene appears empty");

        // The origin (where all three axes meet) is the look-at target,
        // so the gizmo must project near the viewport center — check a
        // small neighborhood around the exact center pixel rather than
        // that one pixel itself: the blue (Z) axis points directly at the
        // camera from this viewpoint and rasterizes to a near-zero-length
        // line, so which exact pixel(s) a GPU's line rasterizer covers
        // there isn't something to pin down bit-for-bit, only that the
        // gizmo is visible in that immediate area.
        let half_window = 4usize;
        let cx = (target.width / 2) as usize;
        let cy = (target.height / 2) as usize;
        let found_near_center = (cy.saturating_sub(half_window)..=(cy + half_window).min(target.height as usize - 1)).any(|y| {
            (cx.saturating_sub(half_window)..=(cx + half_window).min(target.width as usize - 1)).any(|x| {
                let idx = (y * target.width as usize + x) * 4;
                let p = &pixels[idx..idx + 4];
                p[0] > 10 || p[1] > 10 || p[2] > 10
            })
        });
        assert!(found_near_center, "expected the axes gizmo (drawn through the look-at target) to be visible near the viewport center");
    }

    #[test]
    fn empty_scene_renders_only_the_clear_color() {
        let gpu = GpuContext::new().expect("GPU context required for this test");
        let target = OffscreenTarget::new(&gpu, 32, 32);
        let renderer = LineRenderer::new(&gpu, OFFSCREEN_COLOR_FORMAT);
        let camera = OrbitCamera::new(Point3::origin(), 10.0, 1.0);
        renderer.render(&gpu, &Scene::new(), &camera.view_projection_matrix(), target.color_view(), target.depth_view(), wgpu::Color { r: 0.2, g: 0.2, b: 0.2, a: 1.0 });

        let pixels = target.read_pixels_rgba8(&gpu);
        // 0.2 in linear Rgba8Unorm -> ~51/255.
        assert!(pixels.chunks_exact(4).all(|p| (p[0] as i32 - 51).abs() <= 2), "empty scene should render as a uniform clear color");
    }

    #[test]
    fn offscreen_render_of_a_trajectory_with_pose_and_point_markers_produces_non_background_pixels() {
        // A synthetic stand-in for what `bin/slam-viz` (`plan/STAGE3.md`
        // M3) will build from a real run's `trajectory.csv` (read via
        // `slam_eval::read_trajectory_csv`) plus a few keyframe poses and
        // landmark points — `slam-render` itself stays decoupled from
        // `slam-eval` (see `plan/STAGE3.md`'s workspace layout), so this
        // test exercises the same `Scene` primitives with local data
        // instead of a real CSV round trip.
        let gpu = GpuContext::new().expect("GPU context required for this test");
        let target = OffscreenTarget::new(&gpu, 256, 256);
        let renderer = LineRenderer::new(&gpu, OFFSCREEN_COLOR_FORMAT);

        let mut scene = Scene::new();
        let trajectory: Vec<Point3<f64>> = (0..20).map(|i| Point3::new(i as f64 * 0.3, (i as f64 * 0.4).sin(), 0.0)).collect();
        scene.add_polyline(&trajectory, [1.0, 0.6, 0.0]);
        scene.add_pose_marker(&SE3::new(SO3::identity(), trajectory[0].coords), 0.6, [0.8, 0.8, 0.8]);
        scene.add_pose_marker(&SE3::new(SO3::identity(), trajectory[19].coords), 0.6, [0.8, 0.8, 0.8]);
        scene.add_point_markers(&[Point3::new(1.0, 1.0, 0.5), Point3::new(-1.0, 0.5, -0.5)], 0.15, [0.2, 1.0, 0.2]);

        let center = trajectory[10];
        let camera = OrbitCamera::new(center, 8.0, target.width as f64 / target.height as f64);
        renderer.render(&gpu, &scene, &camera.view_projection_matrix(), target.color_view(), target.depth_view(), wgpu::Color::BLACK);

        let pixels = target.read_pixels_rgba8(&gpu);
        let non_background = pixels.chunks_exact(4).filter(|p| p[0] > 10 || p[1] > 10 || p[2] > 10).count();
        assert!(non_background > 0, "expected the trajectory polyline + pose/point markers to render visibly, got an all-background image");
    }
}
