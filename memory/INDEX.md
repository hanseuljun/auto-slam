# Memory index

One line per entry, newest first within each section. See `README.md` for
what goes where.

## progress/
- [2026-07-21-stage2-plan.md](progress/2026-07-21-stage2-plan.md) — wrote `plan/STAGE2.md`: real-time VIO (factor <=1.0) + finishing Stage 1's M9/M10, after a rolled-back M9 attempt found global BA's dense O(n^3) solve doesn't scale (decisions/0007's unbounded history is the root cause).
- [2026-07-21-m8-global-bundle-adjustment.md](progress/2026-07-21-m8-global-bundle-adjustment.md) — M8 done: global BA over the full retained trajectory, reusing slam-optim's Problem/optimize. ATE held (0.1366m→0.1377m) on a short loop-free MH_01 clip — expected; a real win likely needs a longer sequence and/or post-loop-closure.
- [2026-07-21-m7-loop-closure.md](progress/2026-07-21-m7-loop-closure.md) — M7 done: BoW vocabulary + geometric verification + pose-graph optimization, real MH_05 loop verified, ATE 5.6m→3.3m. Four real bugs found and fixed, including a hidden VoPipeline corruption.
- [2026-07-21-m6-robust-tracking-and-full-sequence-runs.md](progress/2026-07-21-m6-robust-tracking-and-full-sequence-runs.md) — M6 done: track-loss recovery (VO + IMU-propagation for VIO), LK final-residual fix, full 5-sequence run with zero unrecoverable frames across ~14,000 frames.
- [2026-07-21-m5-sliding-window-vio-backend.md](progress/2026-07-21-m5-sliding-window-vio-backend.md) — M5 done: `slam-optim` LM+Schur solver, `slam-backend` VioPipeline (naive fixed-lag window), VIO ATE ~matches M3's VO-only on MH_01/04. 81 workspace tests.
- [2026-07-21-m4-imu-preintegration-and-initializers.md](progress/2026-07-21-m4-imu-preintegration-and-initializers.md) — M4 done: `slam-imu` preintegration + static initializer, `slam-frontend` dynamic VI initializer, 69 workspace tests. Includes a debugging-technique writeup worth rereading before the next tricky linear-algebra bug.
- [2026-07-20-m3-stereo-vo-checkpoint.md](progress/2026-07-20-m3-stereo-vo-checkpoint.md) — M3 done: stereo matching + `VoPipeline` in `slam-frontend`, first ATE checkpoint (11-17cm RMSE, VO-only) across all 5 MH_* sequences.
- [2026-07-20-m2-vision-frontend-primitives.md](progress/2026-07-20-m2-vision-frontend-primitives.md) — M2 done: `slam-vision` (pyramid, grid-FAST, pyramidal LK), slam-inspect extended, 41 workspace tests, clippy clean.
- [2026-07-20-m1-geometry-and-camera-model.md](progress/2026-07-20-m1-geometry-and-camera-model.md) — M1 done: `slam-core` SO3/SE3, `slam-geometry` (pinhole/distortion, rectification, triangulation, DLT PnP), slam-inspect extended, tests+clippy clean.
- [2026-07-20-m0-workspace-and-dataset-io.md](progress/2026-07-20-m0-workspace-and-dataset-io.md) — M0 done: Cargo workspace, `slam-dataset`/`slam-eval` implemented, `bin/slam-inspect` test app, tests+clippy clean.
- [2026-07-20-stage1-plan.md](progress/2026-07-20-stage1-plan.md) — wrote `plan/STAGE1.md`, set up CLAUDE.md + this memory directory; no pipeline code yet.

## decisions/
- [0009-vo-rejects-implausible-pose-jumps.md](decisions/0009-vo-rejects-implausible-pose-jumps.md) — VoPipeline now rejects PnP poses implying implausible translation jumps; a real corruption found by M7's full-MH_05 test, invisible to M6's own Sim3-aligned ATE checkpoint.
- [0008-loop-closure-descriptor-matching-needs-ratio-test.md](decisions/0008-loop-closure-descriptor-matching-needs-ratio-test.md) — M7's descriptor matching needed a Lowe's-ratio-test filter, not just an absolute Hamming threshold, to get real geometric verification working on MH_05.
- [0007-m5-backend-is-naive-fixed-lag-not-marginalized.md](decisions/0007-m5-backend-is-naive-fixed-lag-not-marginalized.md) — M5's sliding window drops the oldest keyframe outright instead of marginalizing it into a prior; a documented staged-scope choice, not an oversight.
- [0006-imu-factor-uses-numerical-jacobians.md](decisions/0006-imu-factor-uses-numerical-jacobians.md) — slam-optim's IMU factor uses central-difference numerical Jacobians instead of 18 hand-derived analytic blocks; reprojection factor still analytic.
- [0005-dynamic-init-fixes-accel-bias-at-zero.md](decisions/0005-dynamic-init-fixes-accel-bias-at-zero.md) — M4's dynamic VI initializer fixes accel bias at zero instead of solving for it jointly; confirmed exact rank deficiency, not an oversight.
- [0004-umeyama-ate-brought-forward-to-m3.md](decisions/0004-umeyama-ate-brought-forward-to-m3.md) — Umeyama Sim3 alignment + ATE landed in `slam-eval` at M3, not M9, since M3's own checkpoint test needs it.
- [0003-pnp-via-dlt-plus-refinement-not-p3p-epnp.md](decisions/0003-pnp-via-dlt-plus-refinement-not-p3p-epnp.md) — M1 PnP is DLT+Gauss-Newton refine, not literal P3P/EPnP; those wait for a RANSAC consumer in M3/M4.
- [0002-event-stream-models-three-independent-streams.md](decisions/0002-event-stream-models-three-independent-streams.md) — `EventStream` merges cam0/cam1/imu0 independently, not as index-paired stereo, because MH_04 has mismatched cam0/cam1 counts.
- [0001-dependency-and-modality-policy.md](decisions/0001-dependency-and-modality-policy.md) — infra crates OK, SLAM logic hand-written; target is stereo+IMU (VIO) with loop closure, not stereo-only or mono-inertial.

## notes/
- [lk-tracker-gotchas.md](notes/lk-tracker-gotchas.md) — pyramidal LK bugs/fixes: coarse-level window bounds, aperture-problem test-fixture trap, informal survival-rate baseline, final-residual check (M6) and why noise beats blank frames for forcing test track loss.
- [dataset-quirks.md](notes/dataset-quirks.md) — EuRoC `machine_hall` layout, timestamp/sync gotchas, static vs. dynamic sequence starts.
