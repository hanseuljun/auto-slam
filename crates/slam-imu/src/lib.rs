//! On-manifold IMU preintegration with bias Jacobians and covariance
//! propagation, plus static/dynamic gravity+bias initializers (Stage 1
//! milestone M4). Covariance propagation landed in `plan/STAGE6.md` M2,
//! closing a gap Stage 1 M5 deferred and Stage 2 M6 (`decisions/0016`)
//! found couldn't be shortcut with a simpler formula.

mod initializer;
mod preintegration;

pub use initializer::{find_stationary_window, static_initialize, StaticInitResult};
pub use preintegration::{Covariance9, Preintegration};
