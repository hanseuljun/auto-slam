# Memory index

One line per entry, newest first within each section. See `README.md` for
what goes where.

## progress/
- [2026-07-21-m4-imu-preintegration-and-initializers.md](progress/2026-07-21-m4-imu-preintegration-and-initializers.md) — M4 done: `slam-imu` preintegration + static initializer, `slam-frontend` dynamic VI initializer, 69 workspace tests. Includes a debugging-technique writeup worth rereading before the next tricky linear-algebra bug.
- [2026-07-20-m3-stereo-vo-checkpoint.md](progress/2026-07-20-m3-stereo-vo-checkpoint.md) — M3 done: stereo matching + `VoPipeline` in `slam-frontend`, first ATE checkpoint (11-17cm RMSE, VO-only) across all 5 MH_* sequences.
- [2026-07-20-m2-vision-frontend-primitives.md](progress/2026-07-20-m2-vision-frontend-primitives.md) — M2 done: `slam-vision` (pyramid, grid-FAST, pyramidal LK), slam-inspect extended, 41 workspace tests, clippy clean.
- [2026-07-20-m1-geometry-and-camera-model.md](progress/2026-07-20-m1-geometry-and-camera-model.md) — M1 done: `slam-core` SO3/SE3, `slam-geometry` (pinhole/distortion, rectification, triangulation, DLT PnP), slam-inspect extended, tests+clippy clean.
- [2026-07-20-m0-workspace-and-dataset-io.md](progress/2026-07-20-m0-workspace-and-dataset-io.md) — M0 done: Cargo workspace, `slam-dataset`/`slam-eval` implemented, `bin/slam-inspect` test app, tests+clippy clean.
- [2026-07-20-stage1-plan.md](progress/2026-07-20-stage1-plan.md) — wrote `plan/STAGE1.md`, set up CLAUDE.md + this memory directory; no pipeline code yet.

## decisions/
- [0005-dynamic-init-fixes-accel-bias-at-zero.md](decisions/0005-dynamic-init-fixes-accel-bias-at-zero.md) — M4's dynamic VI initializer fixes accel bias at zero instead of solving for it jointly; confirmed exact rank deficiency, not an oversight.
- [0004-umeyama-ate-brought-forward-to-m3.md](decisions/0004-umeyama-ate-brought-forward-to-m3.md) — Umeyama Sim3 alignment + ATE landed in `slam-eval` at M3, not M9, since M3's own checkpoint test needs it.
- [0003-pnp-via-dlt-plus-refinement-not-p3p-epnp.md](decisions/0003-pnp-via-dlt-plus-refinement-not-p3p-epnp.md) — M1 PnP is DLT+Gauss-Newton refine, not literal P3P/EPnP; those wait for a RANSAC consumer in M3/M4.
- [0002-event-stream-models-three-independent-streams.md](decisions/0002-event-stream-models-three-independent-streams.md) — `EventStream` merges cam0/cam1/imu0 independently, not as index-paired stereo, because MH_04 has mismatched cam0/cam1 counts.
- [0001-dependency-and-modality-policy.md](decisions/0001-dependency-and-modality-policy.md) — infra crates OK, SLAM logic hand-written; target is stereo+IMU (VIO) with loop closure, not stereo-only or mono-inertial.

## notes/
- [lk-tracker-gotchas.md](notes/lk-tracker-gotchas.md) — pyramidal LK bugs/fixes: coarse-level window bounds shouldn't kill a whole track, aperture-problem test-fixture trap, informal MH_01 survival-rate baseline.
- [dataset-quirks.md](notes/dataset-quirks.md) — EuRoC `machine_hall` layout, timestamp/sync gotchas, static vs. dynamic sequence starts.
