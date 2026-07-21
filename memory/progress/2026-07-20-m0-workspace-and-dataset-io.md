# M0 — workspace scaffold + dataset I/O

Landed the first working milestone from `plan/STAGE1.md`.

## What's done

- Installed the Rust toolchain (rustup, stable 1.97.1) — this machine had no
  `cargo` before this session.
- Cargo workspace with all ten `crates/` from the plan's layout plus
  `bin/slam-inspect`. Nine of the crates are intentionally empty
  (doc-comment-only `lib.rs` stating their future scope/milestone) since
  M0 is scaffold-only for them; only `slam-dataset` and `slam-eval` have
  real implementations, per the plan's M0 scope.
- `slam-dataset`: `Calibration`/`CameraCalibration`/`ImuCalibration` parsed
  from `sensor.yaml` (serde_yaml), `EuRocSequence::load` parsing all three
  `data.csv` streams (cam0/cam1/imu0), lazy PNG decoding
  (`load_cam0_image`/`load_cam1_image`, not eager), and a merged
  time-ordered `EventStream` iterator (three-way merge over already-sorted
  streams — see `notes/dataset-quirks.md` for why cam0/cam1/imu0 are kept
  as three independent streams rather than assuming stereo-pair index
  alignment).
- `slam-eval`: `GroundTruthTrajectory::load` parsing
  `state_groundtruth_estimate0/data.csv`, `interpolate(timestamp_ns)` doing
  lerp (position) + slerp (orientation) between bracketing states, `None`
  outside the trajectory's time range.
- `bin/slam-inspect`: the CLAUDE.md-mandated test app. Takes zero or more
  sequence directories (defaults to everything under `data/machine_hall`),
  prints calibration, frame/IMU counts, event-stream size +
  time-ordering check, and a raw groundtruth trajectory summary
  (state count, span, start/end position, bounding box) per sequence.
  Verified by hand against all five `MH_*` sequences.
- 9 unit tests across `slam-dataset`/`slam-eval` (calibration values vs.
  the raw yaml, frame/IMU counts vs. `wc -l`, real PNG decode + resolution
  check, event-stream ordering/completeness, groundtruth exact/midpoint
  interpolation, out-of-range `None`). `cargo test` and
  `cargo clippy --all-targets` both clean.

## Notable finding during verification

Running `slam-inspect` across all five sequences (not just MH_01, which is
what the plan's M0 test explicitly calls for) surfaced that MH_04_difficult
has mismatched cam0/cam1 frame counts (2033 vs 2032) — see
`notes/dataset-quirks.md`. Worth having checked all five now rather than
discovering it mid-M3 when stereo matching assumes paired frames.

## Not done yet (correctly out of scope for M0)

Everything past dataset I/O: geometry/camera model (M1), vision frontend
(M2), and so on. `slam-core` is still just a stub even though it's
"cross-cutting" — it has no consumers yet, so there was nothing concrete to
build; M1 is where it gets real content (SO3/SE3, used by the camera model
and rectification).

## Commit

Pushed as part of this session's M0 commit — see `git log` for the exact
SHA; not duplicating that here since it's derivable from the repo.
