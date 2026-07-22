//! Sparse Levenberg-Marquardt solver with Schur-complement landmark
//! elimination and robust kernels (Stage 1 milestone M5), plus keyframe
//! marginalization into a prior factor (Stage 2 M1, closing
//! `decisions/0007`).

mod bias_random_walk;
mod huber;
mod imu_factor;
mod marginalization;
mod reprojection;
mod solver;
mod state;

pub use bias_random_walk::bias_random_walk_residual_jacobian;
pub use huber::huber_weight;
pub use imu_factor::{imu_residual, imu_residual_jacobian};
pub use marginalization::{marginalize_keyframe, MarginalizationInput, UniqueLandmarkObservation};
pub use reprojection::reprojection_residual_jacobian;
pub use solver::{optimize, BiasRwFactorSpec, ImuFactorSpec, PriorFactor, Problem, ReprojectionObservation, SolverConfig};
pub use state::{KeyframeState, STATE_DIM};
