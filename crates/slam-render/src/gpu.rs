/// Thin bootstrap around `wgpu`'s instance/adapter/device/queue — the GPU
/// API is infra (`memory/decisions/0018`), so this module is deliberately
/// small: it exists to be reused by both an offscreen render target (this
/// crate's own tests, no window needed) and a real window surface
/// (`bin/slam-viz`, `plan/STAGE3.md` M3+), not to add behavior of its own.
pub struct GpuContext {
    pub instance: wgpu::Instance,
    pub adapter: wgpu::Adapter,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
}

impl GpuContext {
    /// Creates a GPU context with no window/surface attached — works
    /// headless (confirmed on this repo's own development machine: Metal
    /// doesn't need a display server the way X11 does), which is what
    /// makes the offscreen render smoke test in `renderer.rs` possible at
    /// all without a real window.
    pub fn new() -> anyhow::Result<Self> {
        let instance = wgpu::Instance::default();
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())).ok_or_else(|| anyhow::anyhow!("no GPU adapter available"))?;
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default(), None))?;
        Ok(GpuContext { instance, adapter, device, queue })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn context_creation_succeeds_headless() {
        let ctx = GpuContext::new();
        assert!(ctx.is_ok(), "GPU context creation failed: {:?}", ctx.err());
    }
}
