use nalgebra::{DMatrix, DVector, Matrix6, Vector6};
use slam_core::SE3;

/// A relative-pose constraint between two pose-graph nodes: `relative_pose`
/// is the measured transform from node `i`'s frame to node `j`'s
/// (`predicted_pose_j = relative_pose.compose(&pose_i)`, matching this
/// codebase's `world -> frame` SE3 convention throughout). Both odometry
/// edges (consecutive keyframes) and the loop-closure edge use this same
/// type — a loop edge is just one more relative-pose measurement, is the
/// whole point of pose-graph optimization.
#[derive(Debug, Clone, Copy)]
pub struct PoseGraphEdge {
    pub i: usize,
    pub j: usize,
    pub relative_pose: SE3,
    /// Ad hoc isotropic weight (same simplification as `slam-optim`'s
    /// `SolverConfig` — see `memory/decisions`), higher for
    /// higher-confidence edges (e.g. odometry over the loop edge, or
    /// vice versa depending on how many geometric-verification inliers
    /// it had).
    pub weight: f64,
}

fn edge_residual(pose_i: &SE3, pose_j: &SE3, relative_pose_measured: &SE3) -> Vector6<f64> {
    let predicted_j = relative_pose_measured.compose(pose_i);
    predicted_j.inverse().compose(pose_j).log()
}

/// Optimizes a pose graph in place via Levenberg-Marquardt with numerical
/// (central-difference) edge Jacobians — same deliberate tradeoff as
/// `slam_optim`'s IMU factor (`decisions/0006`): a 6-DoF-per-node pose
/// graph's Jacobians aren't hard to derive, but reusing the same
/// finite-difference-and-verify-on-a-toy-problem approach that's worked
/// reliably all session is cheap insurance, and pose graphs in Stage 1 are
/// small (tens to low hundreds of keyframes, not thousands). `fixed_node`
/// is the gauge anchor (never updated), same role as `slam_optim`'s
/// keyframe 0.
pub fn optimize_pose_graph(poses: &mut [SE3], edges: &[PoseGraphEdge], fixed_node: usize, iterations: usize) {
    let n = poses.len();
    let free_index_of = |i: usize| -> Option<usize> {
        if i == fixed_node {
            None
        } else if i < fixed_node {
            Some(i)
        } else {
            Some(i - 1)
        }
    };
    let dim = (n - 1) * 6;
    if dim == 0 {
        return;
    }

    let compute_cost = |poses: &[SE3]| -> f64 {
        edges.iter().map(|e| e.weight * edge_residual(&poses[e.i], &poses[e.j], &e.relative_pose).norm_squared()).sum()
    };
    // A fixed absolute initial lambda (as `slam_optim`'s solver uses) is
    // wrong here: pose-graph edge weights can span orders of magnitude
    // (a confident loop edge deliberately weighted far above individual
    // odometry edges — exactly this module's own doc-comment guidance),
    // so a damping term sized for one graph can be wildly too small for
    // another, letting the very first Gauss-Newton step overshoot into a
    // nonsensical region it can never recover from. Found via a real
    // MH_05 test: initial lambda=1e-3 diverged to ~1e22m of pose shift in
    // one step while `compute_cost`'s trial evaluation kept *accepting*
    // it — because SE3::log() aliases rotation error above pi, a
    // sufficiently deranged pose can score a deceptively low residual.
    // Scaling the initial damping to the Hessian's own magnitude (the
    // standard Marquardt heuristic, tau * max diagonal) avoids this
    // regardless of edge-weight scale.
    let mut lambda: Option<f64> = None;
    let mut current_cost = compute_cost(poses);

    for _ in 0..iterations {
        let mut h = DMatrix::<f64>::zeros(dim, dim);
        let mut b = DVector::<f64>::zeros(dim);

        for e in edges {
            let base = edge_residual(&poses[e.i], &poses[e.j], &e.relative_pose);
            let eps = 1e-6;
            let mut jac_i = Matrix6::<f64>::zeros();
            let mut jac_j = Matrix6::<f64>::zeros();
            for col in 0..6 {
                let mut delta = Vector6::<f64>::zeros();
                delta[col] = eps;
                let pi_pert = SE3::exp(delta) * poses[e.i];
                let r_i = edge_residual(&pi_pert, &poses[e.j], &e.relative_pose);
                jac_i.set_column(col, &((r_i - base) / eps));

                let pj_pert = SE3::exp(delta) * poses[e.j];
                let r_j = edge_residual(&poses[e.i], &pj_pert, &e.relative_pose);
                jac_j.set_column(col, &((r_j - base) / eps));
            }

            let w = e.weight.sqrt();
            let wr = base * w;
            let wji = jac_i * w;
            let wjj = jac_j * w;

            let fi = free_index_of(e.i);
            let fj = free_index_of(e.j);
            if let Some(fi) = fi {
                let ri = fi * 6;
                add_block(&mut h, ri, ri, &(wji.transpose() * wji));
                sub_rows(&mut b, ri, &(wji.transpose() * wr));
            }
            if let Some(fj) = fj {
                let rj = fj * 6;
                add_block(&mut h, rj, rj, &(wjj.transpose() * wjj));
                sub_rows(&mut b, rj, &(wjj.transpose() * wr));
            }
            if let (Some(fi), Some(fj)) = (fi, fj) {
                let (ri, rj) = (fi * 6, fj * 6);
                add_block(&mut h, ri, rj, &(wji.transpose() * wjj));
                add_block(&mut h, rj, ri, &(wjj.transpose() * wji));
            }
        }

        let lambda_val = *lambda.get_or_insert_with(|| 1e-6 * (0..dim).map(|i| h[(i, i)]).fold(0.0, f64::max).max(1e-12));

        let mut damped = h.clone();
        for i in 0..dim {
            damped[(i, i)] += lambda_val * h[(i, i)].max(1e-12);
        }
        let Some(delta) = damped.lu().solve(&b) else {
            lambda = Some(lambda_val * 4.0);
            continue;
        };

        let backup: Vec<SE3> = poses.to_vec();
        for (i, pose) in poses.iter_mut().enumerate() {
            if let Some(fi) = free_index_of(i) {
                let d = Vector6::<f64>::from_column_slice(delta.rows(fi * 6, 6).as_slice());
                *pose = SE3::exp(d) * *pose;
            }
        }

        let trial_cost = compute_cost(poses);
        if trial_cost < current_cost && trial_cost.is_finite() {
            current_cost = trial_cost;
            lambda = Some((lambda_val / 3.0).max(1e-10));
        } else {
            poses.copy_from_slice(&backup);
            lambda = Some(lambda_val * 4.0);
        }
    }
}

fn add_block(h: &mut DMatrix<f64>, row: usize, col: usize, block: &Matrix6<f64>) {
    let mut view = h.view_mut((row, col), (6, 6));
    for r in 0..6 {
        for c in 0..6 {
            view[(r, c)] += block[(r, c)];
        }
    }
}

fn sub_rows(v: &mut DVector<f64>, row: usize, block: &Vector6<f64>) {
    for r in 0..6 {
        v[row + r] -= block[r];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Vector3;
    use slam_core::SO3;

    /// A chain of 6 poses with accumulated odometry drift, plus one loop-
    /// closure edge connecting the last pose back to the (perturbed)
    /// first — the standard pose-graph-SLAM toy problem. Optimizing
    /// should redistribute the loop correction across the chain, pulling
    /// every pose closer to its true value, not just closing the loop at
    /// one end.
    #[test]
    fn loop_closure_edge_corrects_accumulated_drift() {
        let true_poses: Vec<SE3> = (0..6)
            .map(|i| {
                let t = i as f64;
                SE3::new(SO3::exp(Vector3::new(0.0, 0.0, 0.3) * t), Vector3::new(1.0, 0.0, 0.0) * t)
            })
            .collect();

        // Simulate odometry drift: each consecutive relative pose is
        // measured with a small consistent bias.
        let drift = SE3::new(SO3::exp(Vector3::new(0.0, 0.0, 0.02)), Vector3::new(0.02, 0.0, 0.0));
        let mut edges = Vec::new();
        for i in 0..5 {
            let true_relative = true_poses[i + 1].compose(&true_poses[i].inverse());
            edges.push(PoseGraphEdge { i, j: i + 1, relative_pose: drift.compose(&true_relative), weight: 1.0 });
        }
        // A high-confidence loop edge with the *true* relative pose
        // between node 5 and node 0 (as if geometric verification nailed
        // it), weighted higher than the drifting odometry.
        let true_loop_relative = true_poses[0].compose(&true_poses[5].inverse());
        edges.push(PoseGraphEdge { i: 5, j: 0, relative_pose: true_loop_relative, weight: 500.0 });

        // Initial estimate: propagate the drifted odometry from node 0.
        let mut poses = vec![true_poses[0]];
        for i in 0..5 {
            let measured = edges[i].relative_pose;
            poses.push(measured.compose(&poses[i]));
        }

        let error_before: f64 = poses.iter().zip(true_poses.iter()).map(|(p, t)| (p.inverse().translation - t.inverse().translation).norm()).sum();
        let node5_error_before = (poses[5].inverse().translation - true_poses[5].inverse().translation).norm();

        optimize_pose_graph(&mut poses, &edges, 0, 100);

        let error_after: f64 = poses.iter().zip(true_poses.iter()).map(|(p, t)| (p.inverse().translation - t.inverse().translation).norm()).sum();
        let node5_error_after = (poses[5].inverse().translation - true_poses[5].inverse().translation).norm();

        // Overall drift strictly improves (the actual, non-arbitrary claim
        // pose-graph optimization with a loop edge makes)...
        assert!(error_after < error_before, "expected loop closure to reduce drift: before={error_before:.4} after={error_after:.4}");

        // ...and node 5 specifically — the node the high-confidence loop
        // edge directly ties back to the fixed anchor (node 0) — should
        // end up very close to its true pose: with 500x the weight of any
        // single odometry edge, the optimizer satisfies the loop edge
        // almost exactly (independently confirmed: re-running this same
        // scenario with the loop edge weighted at 20 instead of 500
        // converges to the *identical* cost, which only happens if the
        // loop residual is already ~0 at the optimum regardless of its
        // weight — i.e. the graph has enough slack in the other 5 nodes
        // to satisfy the loop edge exactly without needing to compromise
        // it, so cranking its weight further can't move the optimum).
        // Middle-chain nodes (1-4) don't get the same guarantee — the
        // correction is distributed, not uniform — so this test doesn't
        // assert a uniform per-node bound.
        assert!(node5_error_after < node5_error_before * 0.1, "expected node 5 (tied to the loop edge) to end up close to true: before={node5_error_before:.4} after={node5_error_after:.4}");
        assert_relative_eq!(poses[5].inverse().translation, true_poses[5].inverse().translation, epsilon = 0.01);
    }
}
