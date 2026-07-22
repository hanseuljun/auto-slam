use nalgebra::{Vector2, Vector3};
use slam_core::SE3;
use slam_geometry::{estimate_pose_dlt, refine_pose_gauss_newton};
use slam_vision::Descriptor;

/// For a query descriptor, the best and second-best Hamming distances
/// found in a target set (and the best match's index).
fn best_two(query: &Descriptor, targets: &[Descriptor]) -> Option<(usize, u32, u32)> {
    let mut best: Option<(usize, u32)> = None;
    let mut second_best_dist = u32::MAX;
    for (idx, target) in targets.iter().enumerate() {
        let d = query.hamming_distance(target);
        match best {
            Some((_, bd)) if d < bd => {
                second_best_dist = bd;
                best = Some((idx, d));
            }
            Some((_, bd)) => {
                if d < second_best_dist && d != bd {
                    second_best_dist = d;
                }
            }
            None => best = Some((idx, d)),
        }
    }
    best.map(|(idx, d)| (idx, d, second_best_dist))
}

/// Mutual-nearest-neighbor descriptor matching, gated by both an absolute
/// Hamming threshold *and* a ratio test (the best match must clearly beat
/// the second-best, Lowe's-ratio-test style) — the ratio test turned out
/// to matter a lot in practice: with a few hundred candidate descriptors,
/// pure order statistics on 256-bit Hamming distances means an absolute
/// threshold alone lets many coincidental (non-corresponding) matches
/// through even at a seemingly-strict cutoff. Found by testing against
/// real MH_05 loop-closure candidates — every match passed the absolute
/// threshold but almost none were geometrically consistent, until the
/// ratio test was added (see `memory/notes`).
fn match_descriptors(a: &[Descriptor], b: &[Descriptor], max_hamming_distance: u32, max_ratio: f32) -> Vec<(usize, usize)> {
    let best_in_b: Vec<Option<(usize, u32, u32)>> = a.iter().map(|da| best_two(da, b)).collect();
    let best_in_a: Vec<Option<(usize, u32, u32)>> = b.iter().map(|db| best_two(db, a)).collect();

    let passes_ratio = |dist: u32, second: u32| -> bool { second == u32::MAX || (dist as f32) < max_ratio * (second as f32) };

    let mut matches = Vec::new();
    for (i, best) in best_in_b.iter().enumerate() {
        let Some((j, dist, second)) = *best else { continue };
        if dist > max_hamming_distance || !passes_ratio(dist, second) {
            continue;
        }
        if let Some((back_i, _, _)) = best_in_a[j] {
            if back_i == i {
                matches.push((i, j));
            }
        }
    }
    matches
}

#[derive(Debug, Clone, Copy)]
pub struct GeometricVerificationParams {
    pub max_hamming_distance: u32,
    /// Lowe's-ratio-test threshold: the best match must have Hamming
    /// distance less than `max_ratio * second_best_distance` (see
    /// `match_descriptors`'s doc comment for why this matters more than
    /// the absolute threshold alone).
    pub max_ratio: f32,
    /// Reprojection error threshold, in normalized image coordinates.
    pub max_reprojection_error: f64,
    pub min_inliers: usize,
}

impl Default for GeometricVerificationParams {
    fn default() -> Self {
        GeometricVerificationParams {
            max_hamming_distance: 60,
            max_ratio: 0.8,
            max_reprojection_error: 0.02,
            min_inliers: 15,
        }
    }
}

/// A verified loop closure: the relative pose mapping the *candidate*
/// (older) keyframe's world-frame points into the *current* keyframe's
/// camera frame (`p_current_cam = relative_pose.transform(p_world)`,
/// where `p_world` is expressed in the candidate keyframe's own landmark
/// frame — the caller is responsible for interpreting this as the loop
/// constraint between the two keyframes' poses).
#[derive(Debug, Clone, Copy)]
pub struct VerifiedLoop {
    pub relative_pose: SE3,
    pub num_inliers: usize,
}

/// Verifies a loop-closure candidate: matches descriptors between the
/// current keyframe's observations and the candidate keyframe's, gathers
/// the resulting 3D (candidate's triangulated landmarks) - 2D (current
/// keyframe's observed pixels) correspondences, and estimates + verifies
/// a relative pose via DLT + Gauss-Newton refine (M1's PnP — the same
/// robustness caveat applies: no RANSAC, so a high `min_inliers` bar after
/// the fact is what actually protects against a bad geometric fit here,
/// not outlier rejection during the solve itself; deferred for the same
/// reason M1's PnP deferred a literal RANSAC minimal solver — see
/// `memory/decisions`).
pub fn verify_loop_candidate(
    current_normalized: &[Vector2<f64>],
    current_descriptors: &[Descriptor],
    candidate_descriptors: &[Descriptor],
    candidate_landmarks_world: &[Vector3<f64>],
    params: &GeometricVerificationParams,
) -> Option<VerifiedLoop> {
    let matches = match_descriptors(current_descriptors, candidate_descriptors, params.max_hamming_distance, params.max_ratio);
    if matches.len() < params.min_inliers {
        return None;
    }

    let points_world: Vec<Vector3<f64>> = matches.iter().map(|&(_, j)| candidate_landmarks_world[j]).collect();
    let observations: Vec<Vector2<f64>> = matches.iter().map(|&(i, _)| current_normalized[i]).collect();

    let initial = estimate_pose_dlt(&points_world, &observations)?;
    let pose = refine_pose_gauss_newton(&points_world, &observations, initial, 10);

    let num_inliers = points_world
        .iter()
        .zip(observations.iter())
        .filter(|(p, obs)| {
            let cam = pose.transform(p);
            if cam.z <= 1e-6 {
                return false;
            }
            let predicted = Vector2::new(cam.x / cam.z, cam.y / cam.z);
            (predicted - *obs).norm() < params.max_reprojection_error
        })
        .count();

    if num_inliers < params.min_inliers {
        return None;
    }

    Some(VerifiedLoop { relative_pose: pose, num_inliers })
}

#[cfg(test)]
mod tests {
    use super::*;
    use slam_core::SO3;

    fn synthetic_descriptor(seed: u64) -> Descriptor {
        let mut state = seed;
        let mut bits = [0u64; 4];
        for b in &mut bits {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            *b = state;
        }
        Descriptor(bits)
    }

    #[test]
    fn recovers_known_relative_pose_from_synthetic_correspondences() {
        let true_pose = SE3::new(SO3::exp(Vector3::new(0.1, -0.05, 0.15)), Vector3::new(0.3, -0.1, 0.2));

        let mut landmarks = Vec::new();
        let mut descriptors = Vec::new();
        for i in 0..30u64 {
            let x = ((i as f64) * 0.37).sin() * 1.5;
            let y = ((i as f64) * 0.53).cos() * 1.5;
            let z = 4.0 + ((i as f64) * 0.19).sin();
            landmarks.push(Vector3::new(x, y, z));
            descriptors.push(synthetic_descriptor(i));
        }

        let current_normalized: Vec<Vector2<f64>> = landmarks
            .iter()
            .map(|p| {
                let cam = true_pose.transform(p);
                Vector2::new(cam.x / cam.z, cam.y / cam.z)
            })
            .collect();

        let result = verify_loop_candidate(
            &current_normalized,
            &descriptors,
            &descriptors,
            &landmarks,
            &GeometricVerificationParams { min_inliers: 10, ..GeometricVerificationParams::default() },
        )
        .expect("should verify");

        assert_eq!(result.num_inliers, 30);
        approx::assert_relative_eq!(result.relative_pose.rotation.matrix(), true_pose.rotation.matrix(), epsilon = 1e-6);
        approx::assert_relative_eq!(result.relative_pose.translation, true_pose.translation, epsilon = 1e-6);
    }

    #[test]
    fn too_few_descriptor_matches_rejects() {
        let descriptors_a: Vec<Descriptor> = (0..5).map(synthetic_descriptor).collect();
        let descriptors_b: Vec<Descriptor> = (100..105).map(synthetic_descriptor).collect();
        let points = vec![Vector2::zeros(); 5];
        let landmarks = vec![Vector3::new(0.0, 0.0, 3.0); 5];

        let result = verify_loop_candidate(&points, &descriptors_a, &descriptors_b, &landmarks, &GeometricVerificationParams::default());
        assert!(result.is_none());
    }
}
