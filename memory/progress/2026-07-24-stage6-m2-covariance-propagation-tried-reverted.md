Stage 6 M2 done: real preintegration covariance propagation implemented,
validated, then its use as solver/marginalization IMU weighting measured
and reverted after a real accuracy regression.

Built `Preintegration::covariance()`/`total_covariance()` — Forster et
al.'s recursion (9-dim error state, state-transition + noise-input
matrices), validated via Monte Carlo (4000 trials) on the first try. Fed
it into `imu_factor_sqrt_information_diagonal` to replace `SolverConfig`'s
3 ad hoc IMU weight scalars with a real per-factor information matrix.

Measured on all 5 sequences (bounded 30s clips) against the M1 baseline
(0.155/0.207/0.893/1.005/0.597m): covariance-weighted regressed 4 of 5,
up to +101% (`MH_05_difficult`). Root cause: EuRoC's real noise densities
imply the solver should trust a single short IMU interval 30-166x more
than the old ad hoc scalars did, but over-trusting short-horizon IMU
propagation relative to vision lets drift compound unboundedly between
corrections — the same failure mode `decisions/0016` already found and
reverted for bias-random-walk weights. Reverted the weighting (restored
ad hoc scalars), which recovered to (and on 2/5 sequences slightly beat)
the M1 baseline: 0.162/0.198/0.768/1.180/0.632m.

Along the way, using real noise densities in a marginalization test
surfaced a genuine, independent numerical bug: `h_kk`'s Schur-complement
inversion mixing reprojection-scale info (~1e6) with covariance-derived
info (~1e-9) produced small negative eigenvalues in the output prior,
which `compute_cost`'s quadratic form turned into an unbounded-below cost
— one LM step diverged to a velocity of ~9573 m/s. Fixed via Jacobi-
scaled Cholesky solve (instead of a plain matrix inverse) plus a gentle
PSD guarantee (shift the whole eigenvalue spectrum up by the smallest
negative eigenvalue's magnitude, rather than reconstructing from a full
eigendecomposition — a first attempt at the latter broke a different,
previously-passing test by scrambling a physically-real near-null
eigenvector subspace). Kept this fix as defense in depth even after
reverting the weighting that originally triggered it.

Final state: `Preintegration::covariance()`/`total_covariance()` kept as
validated infrastructure (available for M5/M6's scale-drift work);
`imu_factor_sqrt_information_diagonal` removed as genuinely dead code;
`SolverConfig`'s IMU weights back to the pre-M2 ad hoc scalars;
marginalization's numerically-safer Schur complement kept. All 151
workspace tests pass (2 ignored, expensive), clippy clean.

Full details: `memory/decisions/0024`.
