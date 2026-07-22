//! Sliding-window visual-inertial optimization and marginalization
//! (Stage 1 milestone M5). The window is currently naive fixed-lag (see
//! `memory/decisions` for why real marginalization was scoped out of this
//! first working version).

mod tests_integration;
mod vio;

pub use vio::{VioFrameResult, VioParams, VioPipeline};
