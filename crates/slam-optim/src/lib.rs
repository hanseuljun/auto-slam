//! Sparse Levenberg-Marquardt solver with Schur-complement marginalization
//! and robust kernels (Stage 1 milestone M5).

mod bias_random_walk;
mod huber;
mod imu_factor;
mod reprojection;
mod solver;
mod state;

pub use bias_random_walk::bias_random_walk_residual_jacobian;
pub use huber::huber_weight;
pub use imu_factor::{imu_residual, imu_residual_jacobian};
pub use reprojection::reprojection_residual_jacobian;
pub use solver::{optimize, BiasRwFactorSpec, ImuFactorSpec, Problem, ReprojectionObservation, SolverConfig};
pub use state::{KeyframeState, STATE_DIM};
