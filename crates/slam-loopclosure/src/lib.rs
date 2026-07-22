//! Bag-of-visual-words place recognition, geometric verification, and
//! pose-graph optimization (Stage 1 milestone M7).

mod capture;
mod geometric_verification;
mod place_recognition;
mod pose_graph;
mod tests_integration;
mod vocabulary;

pub use capture::{capture_loop_keyframe, CaptureParams, KeyframeMeta, LoopKeyframe};
pub use geometric_verification::{verify_loop_candidate, GeometricVerificationParams, VerifiedLoop};
pub use place_recognition::KeyframeDatabase;
pub use pose_graph::{optimize_pose_graph, PoseGraphEdge};
pub use vocabulary::{BowVector, Vocabulary};
