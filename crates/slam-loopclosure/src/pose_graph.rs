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

/// Central-difference Jacobian of `edge_residual` w.r.t. both poses'
/// left-multiplicative tangent perturbations (`pose -> Exp(xi)*pose`,
/// this codebase's own convention). Deliberately still numerical, not
/// analytic (`plan/STAGE6.md` M3's own bullet considered replacing it):
/// a correct closed form needs SE3's own 6x6 left/right Jacobian of the
/// exponential map (the coupled rotation/translation block from Barfoot
/// or Sola's "micro Lie theory"), a real, separate derivation this
/// codebase has no existing machinery for (`SO3::left_jacobian`/
/// `right_jacobian` are 3x3, SO3-only) — deriving and validating it is
/// exactly the kind of substantial, validation-heavy undertaking Stage 6
/// M1 was its own dedicated milestone for, not a footnote to this one.
/// This function isn't the performance bottleneck this milestone
/// removes either (12 cheap 6-dim residual evaluations per edge per
/// iteration, versus the O(n^3) dense linear solve below) — a deliberate
/// scope decision, not an oversight (`memory/decisions/0006` precedent).
fn edge_residual_jacobian(pose_i: &SE3, pose_j: &SE3, relative_pose_measured: &SE3) -> (Vector6<f64>, Matrix6<f64>, Matrix6<f64>) {
    let base = edge_residual(pose_i, pose_j, relative_pose_measured);
    let eps = 1e-6;
    let mut jac_i = Matrix6::<f64>::zeros();
    let mut jac_j = Matrix6::<f64>::zeros();
    for col in 0..6 {
        let mut delta = Vector6::<f64>::zeros();
        delta[col] = eps;
        let pi_pert = SE3::exp(delta) * *pose_i;
        let r_i = edge_residual(&pi_pert, pose_j, relative_pose_measured);
        jac_i.set_column(col, &((r_i - base) / eps));

        let pj_pert = SE3::exp(delta) * *pose_j;
        let r_j = edge_residual(pose_i, &pj_pert, relative_pose_measured);
        jac_j.set_column(col, &((r_j - base) / eps));
    }
    (base, jac_i, jac_j)
}

/// One "chord" edge — connects two free nodes that aren't adjacent in
/// free-index order, so its cross term doesn't fit the block-tridiagonal
/// band `solve_normal_equations` builds for every other edge. Stored as
/// the two 6x6 blocks of the rank-6 factor `u_lo`/`u_hi` such that this
/// edge's *entire* Hessian contribution (both its diagonal blocks, which
/// still go straight into `diag`, and its off-diagonal block, which
/// doesn't) equals `U U^T` for a `dim x 6` matrix `U` that is `u_lo` at
/// block-row `lo` and `u_hi` at block-row `hi` and zero elsewhere — see
/// `solve_normal_equations`'s doc comment for why that identity is
/// exactly what makes the Woodbury correction below valid.
struct Chord {
    lo: usize,
    hi: usize,
    u_lo: Matrix6<f64>,
    u_hi: Matrix6<f64>,
}

/// Optimizes a pose graph in place via Levenberg-Marquardt, using a
/// linear solve that exploits this graph's own real structure — a chain
/// of consecutive-keyframe odometry edges (block-tridiagonal) plus a
/// small number of loop edges (`bin/slam-run` always adds exactly one;
/// this handles any number) — instead of a generic dense `DMatrix`/LU
/// solve over the whole `(n-1)*6`-dimensional system regardless of how
/// sparse the real coupling is (`plan/STAGE6.md` M3, replacing the O(n^3)
/// solve `plan/STAGE5.md` M2 already found reintroduces Stage 4's own
/// scaling bug once the graph gets large). See `solve_normal_equations`'s
/// doc comment for the algorithm (block-tridiagonal Thomas elimination +
/// a Sherman-Morrison-Woodbury correction for the loop edges) and
/// `memory/decisions/0025` for why this was chosen over bringing in a
/// sparse linear-algebra crate as infra. `fixed_node` is the gauge
/// anchor (never updated), same role as `slam_optim`'s keyframe 0 — need
/// not be node 0 itself; the free-index remapping below (and the
/// tridiagonal structure it preserves) works regardless of where it
/// sits in the chain.
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
    let free_n = n - 1;
    let dim = free_n * 6;
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
        let mut diag = vec![Matrix6::<f64>::zeros(); free_n];
        let mut offdiag = vec![Matrix6::<f64>::zeros(); free_n.saturating_sub(1)];
        let mut chords: Vec<Chord> = Vec::new();
        let mut b = DVector::<f64>::zeros(dim);

        for e in edges {
            let (base, jac_i, jac_j) = edge_residual_jacobian(&poses[e.i], &poses[e.j], &e.relative_pose);
            let w = e.weight.sqrt();
            let wr = base * w;
            let wji = jac_i * w;
            let wjj = jac_j * w;

            let fi = free_index_of(e.i);
            let fj = free_index_of(e.j);
            // `b` always gets this edge's full contribution directly,
            // regardless of how its Hessian block below is routed — `b`
            // is linear, so there's no Woodbury split to worry about.
            if let Some(fi) = fi {
                let updated = b.rows(fi * 6, 6) - wji.transpose() * wr;
                b.rows_mut(fi * 6, 6).copy_from(&updated);
            }
            if let Some(fj) = fj {
                let updated = b.rows(fj * 6, 6) - wjj.transpose() * wr;
                b.rows_mut(fj * 6, 6).copy_from(&updated);
            }

            match (fi, fj) {
                (Some(fi), Some(fj)) => {
                    let (lo, hi, u_lo, u_hi) = if fi < fj { (fi, fj, wji, wjj) } else { (fj, fi, wjj, wji) };
                    if hi == lo + 1 {
                        // Fits the tridiagonal band: diag *and* offdiag
                        // both go directly in — no Woodbury needed for
                        // this edge at all.
                        diag[lo] += u_lo.transpose() * u_lo;
                        diag[hi] += u_hi.transpose() * u_hi;
                        offdiag[lo] += u_lo.transpose() * u_hi;
                    } else {
                        // A chord: its *entire* contribution — diagonal
                        // blocks included — is deferred to the Woodbury
                        // correction in `solve_normal_equations` via
                        // `U_c U_c^T`. Adding its diagonal parts to
                        // `diag` here too, on top of that, would double-
                        // count them (a real bug this exact mistake
                        // produced once — caught by `solve_normal_
                        // equations_matches_a_dense_solve_of_the_same_
                        // system`'s multi-chord case, not by the looser
                        // end-to-end test below, which only checks
                        // qualitative convergence and isn't sensitive to
                        // an extra diagonal boost). `Chord` stores the
                        // *factor* `U U^T` needs (see its own doc
                        // comment): row-block `lo` = `u_lo^T`, row-block
                        // `hi` = `u_hi^T` — transposed relative to the
                        // raw Jacobian blocks here.
                        chords.push(Chord { lo, hi, u_lo: u_lo.transpose(), u_hi: u_hi.transpose() });
                    }
                }
                (Some(fi), None) => diag[fi] += wji.transpose() * wji,
                (None, Some(fj)) => diag[fj] += wjj.transpose() * wjj,
                (None, None) => {}
            }
        }

        let lambda_val = *lambda.get_or_insert_with(|| 1e-6 * diag.iter().flat_map(|d| (0..6).map(|k| d[(k, k)])).fold(0.0, f64::max).max(1e-12));
        for d in &mut diag {
            for k in 0..6 {
                d[(k, k)] += lambda_val * d[(k, k)].max(1e-12);
            }
        }

        let Some(delta) = solve_normal_equations(&diag, &offdiag, &chords, &b) else {
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

/// Solves the damped normal equations `(D + offdiag + chords) * x = b`
/// for this iteration's pose delta, in `O(free_n)` instead of the
/// `O(free_n^3)` a generic dense LU solve over the whole system costs.
///
/// `diag`/`offdiag` alone describe a block-tridiagonal matrix (`diag[i]`
/// is the `i`-th 6x6 diagonal block, `offdiag[i]` is `H[i, i+1]`,
/// `offdiag[i]^T` is the symmetric `H[i+1, i]`) — exactly what a chain of
/// consecutive-keyframe odometry edges produces, solvable via the
/// standard block-tridiagonal (block Thomas) forward/backward sweep,
/// `O(free_n)` block operations instead of a dense factorization.
///
/// `chords` are the edges that don't fit that band (in practice, the one
/// loop-closure edge `bin/slam-run` always adds, connecting two nodes far
/// apart in the trajectory). Every edge's full Hessian contribution
/// (diagonal *and* cross blocks) equals `U U^T` for a `dim x 6` matrix
/// `U` that's `u_lo`/`u_hi` at its two block-rows and zero elsewhere (a
/// direct consequence of the Gauss-Newton normal equations always being a
/// Gram matrix of that edge's own stacked Jacobian) — so a chord's cross
/// term is a rank-6 update to the tridiagonal system, and `k` chords
/// stacked side by side give a `dim x 6k` correction matrix `U`. Sherman-
/// Morrison-Woodbury then reduces `(H_tridiag + U U^T)^{-1} b` to: solve
/// the *tridiagonal* system against `[b | U]` combined (one `O(free_n)`
/// sweep, `1+6k` right-hand sides), then a dense `6k x 6k` solve for the
/// small correction term — negligible cost for `k` on the order of 1-2,
/// and still `O(free_n)` overall since `k` doesn't grow with graph size.
///
/// Returns `None` if any block (or the small correction system) isn't
/// invertible, mirroring the old dense solver's own `lu().solve()`
/// fallback — the caller backs off (raises `lambda`) rather than
/// treating this as fatal.
fn solve_normal_equations(diag: &[Matrix6<f64>], offdiag: &[Matrix6<f64>], chords: &[Chord], b: &DVector<f64>) -> Option<DVector<f64>> {
    let dim = diag.len() * 6;
    let k = chords.len();

    if k == 0 {
        let rhs = DMatrix::from_column_slice(dim, 1, b.as_slice());
        let x = block_tridiagonal_solve(diag, offdiag, &rhs)?;
        return Some(DVector::from_column_slice(x.column(0).as_slice()));
    }

    let mut rhs = DMatrix::<f64>::zeros(dim, 1 + 6 * k);
    rhs.view_mut((0, 0), (dim, 1)).copy_from(b);
    for (ci, c) in chords.iter().enumerate() {
        let col0 = 1 + ci * 6;
        rhs.view_mut((c.lo * 6, col0), (6, 6)).copy_from(&c.u_lo);
        rhs.view_mut((c.hi * 6, col0), (6, 6)).copy_from(&c.u_hi);
    }
    let z = block_tridiagonal_solve(diag, offdiag, &rhs)?;
    let z_b = z.column(0).into_owned();
    let z_u = z.view((0, 1), (dim, 6 * k)).into_owned();

    // U^T applied to a `dim`-row matrix only ever touches its `lo`/`hi`
    // block-rows, since that's all of `U` that's nonzero per chord.
    let apply_u_transpose = |m: &DMatrix<f64>| -> DMatrix<f64> {
        let mut out = DMatrix::<f64>::zeros(6 * k, m.ncols());
        for (ci, c) in chords.iter().enumerate() {
            let contrib = c.u_lo.transpose() * m.view((c.lo * 6, 0), (6, m.ncols())) + c.u_hi.transpose() * m.view((c.hi * 6, 0), (6, m.ncols()));
            out.view_mut((ci * 6, 0), (6, m.ncols())).copy_from(&contrib);
        }
        out
    };

    let mut small = DMatrix::<f64>::identity(6 * k, 6 * k);
    small += apply_u_transpose(&z_u);
    let rhs_small = apply_u_transpose(&DMatrix::from_column_slice(dim, 1, z_b.as_slice()));
    let y = small.lu().solve(&rhs_small.column(0).into_owned())?;

    Some(z_b - &z_u * &y)
}

/// The block-tridiagonal (block Thomas) forward-elimination-then-back-
/// substitution sweep, generalized to a multi-column right-hand side
/// (`solve_normal_equations` needs to solve against `b` and `U`'s
/// columns together, in one pass). Standard algorithm — see e.g. Golub &
/// Van Loan's treatment of block-tridiagonal systems — adapted here to
/// 6x6 blocks and matrix (not just vector) right-hand sides.
fn block_tridiagonal_solve(diag: &[Matrix6<f64>], offdiag: &[Matrix6<f64>], rhs: &DMatrix<f64>) -> Option<DMatrix<f64>> {
    // 6 fixed rows (one block), dynamic columns (`rhs`'s own width) —
    // what a `Matrix6 * DMatrix` product actually produces in nalgebra,
    // not a fully dynamic `DMatrix`.
    type Mat6xN = nalgebra::OMatrix<f64, nalgebra::Const<6>, nalgebra::Dyn>;

    let n = diag.len();
    let ncols = rhs.ncols();
    if n == 0 {
        return Some(DMatrix::zeros(0, ncols));
    }

    // Forward sweep: c_prime[i] = (modified diag)^-1 * offdiag[i] (the
    // "upper" elimination factor, only defined for i < n-1), d_prime[i]
    // = (modified diag)^-1 * (modified rhs block).
    let mut c_prime: Vec<Matrix6<f64>> = Vec::with_capacity(n.saturating_sub(1));
    let mut d_prime: Vec<Mat6xN> = Vec::with_capacity(n);

    let d0_inv = diag[0].try_inverse()?;
    if n > 1 {
        c_prime.push(d0_inv * offdiag[0]);
    }
    d_prime.push(d0_inv * rhs.view((0, 0), (6, ncols)));

    for i in 1..n {
        let lower = offdiag[i - 1].transpose();
        let modified_diag = diag[i] - lower * c_prime[i - 1];
        let modified_diag_inv = modified_diag.try_inverse()?;
        if i < n - 1 {
            c_prime.push(modified_diag_inv * offdiag[i]);
        }
        let modified_rhs = rhs.view((i * 6, 0), (6, ncols)) - lower * &d_prime[i - 1];
        d_prime.push(modified_diag_inv * modified_rhs);
    }

    // Back substitution.
    let mut x = DMatrix::<f64>::zeros(n * 6, ncols);
    x.view_mut(((n - 1) * 6, 0), (6, ncols)).copy_from(&d_prime[n - 1]);
    for i in (0..n - 1).rev() {
        let next_block = x.view(((i + 1) * 6, 0), (6, ncols)).clone_owned();
        let xi = &d_prime[i] - c_prime[i] * next_block;
        x.view_mut((i * 6, 0), (6, ncols)).copy_from(&xi);
    }
    Some(x)
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

    /// Direct, isolated check of `solve_normal_equations` itself (no
    /// nonlinear residuals, no LM loop): builds a random SPD block-
    /// tridiagonal-plus-chord system, solves it via the sparse path, and
    /// compares against a dense `DMatrix` solve of the *same* system
    /// assembled explicitly — the most direct possible cross-check that
    /// the Woodbury correction is algebraically right, independent of
    /// whether the pose-graph LM loop happens to mask a bug (as the
    /// end-to-end test above could).
    #[test]
    #[allow(clippy::needless_range_loop)] // index-writes into a dense matrix at computed block offsets, not a collection iteration
    fn solve_normal_equations_matches_a_dense_solve_of_the_same_system() {
        let n = 8; // free nodes
        let dim = n * 6;
        let mut state = 42u64;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        let rand_mat = |next: &mut dyn FnMut() -> f64| Matrix6::<f64>::from_fn(|_, _| next());

        // Build random per-node "edge" factors (each contributes U U^T,
        // same structure real edges produce), diagonal for the chain,
        // plus two chords (one connecting non-adjacent free nodes twice,
        // to also exercise k=2).
        let mut diag = vec![Matrix6::<f64>::zeros(); n];
        let mut offdiag = vec![Matrix6::<f64>::zeros(); n - 1];
        // A well-conditioned diagonal contribution so every block stays
        // invertible: identity plus a random PSD-ish perturbation.
        for d in &mut diag {
            *d = Matrix6::<f64>::identity() * 10.0;
        }
        for i in 0..n - 1 {
            let u = rand_mat(&mut next) * 0.1;
            diag[i] += u.transpose() * u;
            diag[i + 1] += u.transpose() * u;
            offdiag[i] += u.transpose() * u;
        }
        let chords = vec![
            Chord { lo: 0, hi: 3, u_lo: rand_mat(&mut next) * 0.1, u_hi: rand_mat(&mut next) * 0.1 },
            Chord { lo: 2, hi: 6, u_lo: rand_mat(&mut next) * 0.1, u_hi: rand_mat(&mut next) * 0.1 },
        ];
        // Deliberately *not* added to `diag` here — a chord's diagonal
        // blocks are supplied entirely by `solve_normal_equations`'s own
        // Woodbury correction (`U_c U_c^T`, which reconstructs both the
        // diagonal and cross blocks), matching the real edge-processing
        // loop in `optimize_pose_graph`; `diag`/`offdiag` only ever
        // describe the band. `dense` (the independent reference) does
        // need this edge's diagonal contribution added explicitly, since
        // it represents the *whole* system directly, not split into a
        // band-plus-correction the way `diag`/`chords` are.
        let b = DVector::<f64>::from_fn(dim, |_, _| next());

        // Assemble the exact same system as a dense matrix.
        let mut dense = DMatrix::<f64>::zeros(dim, dim);
        for i in 0..n {
            dense.view_mut((i * 6, i * 6), (6, 6)).copy_from(&diag[i]);
        }
        for i in 0..n - 1 {
            dense.view_mut((i * 6, (i + 1) * 6), (6, 6)).copy_from(&offdiag[i]);
            dense.view_mut(((i + 1) * 6, i * 6), (6, 6)).copy_from(&offdiag[i].transpose());
        }
        for c in &chords {
            let lo_lo = dense.view((c.lo * 6, c.lo * 6), (6, 6)).into_owned() + c.u_lo * c.u_lo.transpose();
            dense.view_mut((c.lo * 6, c.lo * 6), (6, 6)).copy_from(&lo_lo);
            let hi_hi = dense.view((c.hi * 6, c.hi * 6), (6, 6)).into_owned() + c.u_hi * c.u_hi.transpose();
            dense.view_mut((c.hi * 6, c.hi * 6), (6, 6)).copy_from(&hi_hi);
            dense.view_mut((c.lo * 6, c.hi * 6), (6, 6)).copy_from(&(c.u_lo * c.u_hi.transpose()));
            dense.view_mut((c.hi * 6, c.lo * 6), (6, 6)).copy_from(&(c.u_hi * c.u_lo.transpose()));
        }

        let x_sparse = solve_normal_equations(&diag, &offdiag, &chords, &b).expect("well-conditioned system should solve");
        let x_dense = dense.lu().solve(&b).expect("well-conditioned dense system should solve");

        assert_relative_eq!(x_sparse, x_dense, epsilon = 1e-6);
    }

    /// Same cross-check with exactly one chord (`k=1`) — the *actual*
    /// shape `bin/slam-run`'s own pose graph always has (a chain plus
    /// exactly one loop edge, `memory/decisions/0021`'s sparse-capture
    /// design), isolated from the `k=2` case above in case the Woodbury
    /// correction has a bug specific to a single-chord small system
    /// (`6*k=6` instead of `12`) that the `k=2` case wouldn't exercise.
    #[test]
    #[allow(clippy::needless_range_loop)] // index-writes into a dense matrix at computed block offsets, not a collection iteration
    fn solve_normal_equations_matches_a_dense_solve_with_exactly_one_chord() {
        let n = 8;
        let dim = n * 6;
        let mut state = 99u64;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        let rand_mat = |next: &mut dyn FnMut() -> f64| Matrix6::<f64>::from_fn(|_, _| next());

        let mut diag = vec![Matrix6::<f64>::identity() * 10.0; n];
        let mut offdiag = vec![Matrix6::<f64>::zeros(); n - 1];
        for i in 0..n - 1 {
            let u = rand_mat(&mut next) * 0.1;
            diag[i] += u.transpose() * u;
            diag[i + 1] += u.transpose() * u;
            offdiag[i] += u.transpose() * u;
        }
        let chords = vec![Chord { lo: 0, hi: n - 1, u_lo: rand_mat(&mut next) * 0.1, u_hi: rand_mat(&mut next) * 0.1 }];
        let b = DVector::<f64>::from_fn(dim, |_, _| next());

        let mut dense = DMatrix::<f64>::zeros(dim, dim);
        for i in 0..n {
            dense.view_mut((i * 6, i * 6), (6, 6)).copy_from(&diag[i]);
        }
        for i in 0..n - 1 {
            dense.view_mut((i * 6, (i + 1) * 6), (6, 6)).copy_from(&offdiag[i]);
            dense.view_mut(((i + 1) * 6, i * 6), (6, 6)).copy_from(&offdiag[i].transpose());
        }
        for c in &chords {
            let lo_lo = dense.view((c.lo * 6, c.lo * 6), (6, 6)).into_owned() + c.u_lo * c.u_lo.transpose();
            dense.view_mut((c.lo * 6, c.lo * 6), (6, 6)).copy_from(&lo_lo);
            let hi_hi = dense.view((c.hi * 6, c.hi * 6), (6, 6)).into_owned() + c.u_hi * c.u_hi.transpose();
            dense.view_mut((c.hi * 6, c.hi * 6), (6, 6)).copy_from(&hi_hi);
            dense.view_mut((c.lo * 6, c.hi * 6), (6, 6)).copy_from(&(c.u_lo * c.u_hi.transpose()));
            dense.view_mut((c.hi * 6, c.lo * 6), (6, 6)).copy_from(&(c.u_hi * c.u_lo.transpose()));
        }

        let x_sparse = solve_normal_equations(&diag, &offdiag, &chords, &b).expect("well-conditioned system should solve");
        let x_dense = dense.lu().solve(&b).expect("well-conditioned dense system should solve");

        assert_relative_eq!(x_sparse, x_dense, epsilon = 1e-6);
    }

    /// Same cross-check with zero chords (the pure block-tridiagonal
    /// path, `k=0`) — isolates that code path from the Woodbury one
    /// above.
    #[test]
    #[allow(clippy::needless_range_loop)] // index-writes into a dense matrix at computed block offsets, not a collection iteration
    fn solve_normal_equations_matches_dense_with_no_chords() {
        let n = 5;
        let dim = n * 6;
        let mut state = 7u64;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        let rand_mat = |next: &mut dyn FnMut() -> f64| Matrix6::<f64>::from_fn(|_, _| next());

        let mut diag = vec![Matrix6::<f64>::identity() * 10.0; n];
        let mut offdiag = vec![Matrix6::<f64>::zeros(); n - 1];
        for i in 0..n - 1 {
            let u = rand_mat(&mut next) * 0.1;
            diag[i] += u.transpose() * u;
            diag[i + 1] += u.transpose() * u;
            offdiag[i] += u.transpose() * u;
        }
        let b = DVector::<f64>::from_fn(dim, |_, _| next());

        let mut dense = DMatrix::<f64>::zeros(dim, dim);
        for i in 0..n {
            dense.view_mut((i * 6, i * 6), (6, 6)).copy_from(&diag[i]);
        }
        for i in 0..n - 1 {
            dense.view_mut((i * 6, (i + 1) * 6), (6, 6)).copy_from(&offdiag[i]);
            dense.view_mut(((i + 1) * 6, i * 6), (6, 6)).copy_from(&offdiag[i].transpose());
        }

        let x_sparse = solve_normal_equations(&diag, &offdiag, &[], &b).expect("well-conditioned system should solve");
        let x_dense = dense.lu().solve(&b).expect("well-conditioned dense system should solve");

        assert_relative_eq!(x_sparse, x_dense, epsilon = 1e-6);
    }

    /// The real point of `plan/STAGE6.md` M3: measures wall-clock cost
    /// directly (not estimated) on a graph the size of `MH_01_easy`'s
    /// *full*, un-truncated trajectory (741 keyframes, dim=4440) — the
    /// exact size `plan/STAGE6.md` M1's own doc comment recorded the old
    /// dense `DMatrix`/LU solve failing to even finish in 10+ minutes on
    /// (the scaling bug `plan/STAGE5.md` M2 found this milestone removes
    /// the ceiling on). A synthetic chain-plus-one-loop-edge graph, same
    /// structure `bin/slam-run`'s own pose graph always has (one edge per
    /// consecutive keyframe pair, one loop edge) — not literally MH_01's
    /// own poses, since this test's job is measuring solver cost, not
    /// re-verifying accuracy (the existing accuracy tests already do
    /// that on realistic, if smaller, graphs).
    #[test]
    fn optimizes_a_741_keyframe_graph_well_within_the_real_time_budget() {
        let n = 741;
        let true_poses: Vec<SE3> = (0..n)
            .map(|i| {
                let t = i as f64;
                SE3::new(SO3::exp(Vector3::new(0.0, 0.0, 0.01) * t), Vector3::new(0.1, 0.0, 0.0) * t)
            })
            .collect();

        let drift = SE3::new(SO3::exp(Vector3::new(0.0, 0.0, 0.001)), Vector3::new(0.001, 0.0, 0.0));
        let mut edges = Vec::with_capacity(n);
        for i in 0..n - 1 {
            let true_relative = true_poses[i + 1].compose(&true_poses[i].inverse());
            edges.push(PoseGraphEdge { i, j: i + 1, relative_pose: drift.compose(&true_relative), weight: 1.0 });
        }
        // The one loop edge `bin/slam-run` always adds, connecting the
        // trajectory's (near) end back toward its start.
        let true_loop_relative = true_poses[0].compose(&true_poses[n - 1].inverse());
        edges.push(PoseGraphEdge { i: n - 1, j: 0, relative_pose: true_loop_relative, weight: 5000.0 });

        let mut poses = vec![true_poses[0]];
        for i in 0..n - 1 {
            let measured = edges[i].relative_pose;
            poses.push(measured.compose(&poses[i]));
        }

        // `MH_01_easy` is 184s of data (`docs/RESULTS.md`) — this solve
        // is one part of one loop-closure pass over the whole sequence,
        // so it needs to be a small fraction of that, with real room to
        // spare (`plan/STAGE4.md` M1's own bar: whole-run factor <= 1.0
        // counting loop closure's total cost, not just this one solve).
        // Measured (not estimated) at ~97ms on this machine — 3-4 orders
        // of magnitude under the old dense solver's "didn't finish in
        // 10+ minutes" on this exact graph size (`memory/decisions/0025`
        // has the full before/after comparison). 5s is a generous bound,
        // not the real number, so this test stays robust to slower CI
        // hardware without losing its actual point.
        let start = std::time::Instant::now();
        optimize_pose_graph(&mut poses, &edges, 0, 50);
        let elapsed = start.elapsed();

        assert!(elapsed.as_secs_f64() < 5.0, "sparse pose-graph solve on a 741-node graph took {elapsed:?}, expected well under 5s (the old dense solve didn't finish in 10+ minutes on this exact size)");

        // Not just fast — still correct: node n-1 (tied directly to the
        // loop edge) ends up close to its true pose.
        assert_relative_eq!(poses[n - 1].inverse().translation, true_poses[n - 1].inverse().translation, epsilon = 0.5);
    }
}
