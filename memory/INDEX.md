# Memory index

One line per entry, newest first within each section. See `README.md` for
what goes where.

## progress/
- [2026-07-20-m0-workspace-and-dataset-io.md](progress/2026-07-20-m0-workspace-and-dataset-io.md) — M0 done: Cargo workspace, `slam-dataset`/`slam-eval` implemented, `bin/slam-inspect` test app, tests+clippy clean.
- [2026-07-20-stage1-plan.md](progress/2026-07-20-stage1-plan.md) — wrote `plan/STAGE1.md`, set up CLAUDE.md + this memory directory; no pipeline code yet.

## decisions/
- [0002-event-stream-models-three-independent-streams.md](decisions/0002-event-stream-models-three-independent-streams.md) — `EventStream` merges cam0/cam1/imu0 independently, not as index-paired stereo, because MH_04 has mismatched cam0/cam1 counts.
- [0001-dependency-and-modality-policy.md](decisions/0001-dependency-and-modality-policy.md) — infra crates OK, SLAM logic hand-written; target is stereo+IMU (VIO) with loop closure, not stereo-only or mono-inertial.

## notes/
- [dataset-quirks.md](notes/dataset-quirks.md) — EuRoC `machine_hall` layout, timestamp/sync gotchas, static vs. dynamic sequence starts.
