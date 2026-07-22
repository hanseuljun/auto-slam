//! Sliding-window visual-inertial optimization (Stage 1 milestone M5)
//! with real Schur-complement keyframe marginalization (Stage 2 M1,
//! closing `decisions/0007`) instead of naive fixed-lag dropping.

mod tests_integration;
mod vio;

pub use vio::{VioFrameResult, VioParams, VioPipeline, VioTiming};
