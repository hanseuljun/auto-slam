//! SO3/SE3 Lie groups and common point/pose types shared across the
//! pipeline (Stage 1, cross-cutting). Sim3 is added in M9 when trajectory
//! alignment first needs it — no consumer for it yet.

mod se3;
mod so3;

pub use se3::SE3;
pub use so3::SO3;
