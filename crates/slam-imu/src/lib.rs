//! On-manifold IMU preintegration with bias Jacobians, plus static/dynamic
//! gravity+bias initializers (Stage 1 milestone M4). Covariance propagation
//! is deferred to M5, where the backend optimizer is the first consumer.

mod initializer;
mod preintegration;

pub use initializer::{find_stationary_window, static_initialize, StaticInitResult};
pub use preintegration::Preintegration;
