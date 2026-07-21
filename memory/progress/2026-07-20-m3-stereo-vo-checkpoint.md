# M3 — stereo visual frontend + static map bootstrap

Landed the fourth milestone from `plan/STAGE1.md`, following M0/M1/M2. This
is the plan's first end-to-end accuracy checkpoint.

## What's done

- `slam-eval::align` (brought forward from M9 — see
  `decisions/0004-umeyama-ate-brought-forward-to-m3.md`): Umeyama Sim3
  alignment + ATE (RMSE/mean/median/std/max), tested against a known
  synthetic similarity transform.
- `slam-frontend::stereo`: epipolar-constrained stereo matching. Given a
  left keypoint, walks candidate disparities along the *rectified*
  epipolar line, maps each candidate back to a raw right-image pixel
  (raw-space patch SSD correlation, not a full rectified-image remap — see
  the module doc comment for why that approximation is fine given EuRoC's
  near-parallel stereo rig), parabola sub-pixel refinement, then
  triangulates directly from the rectified depth formula. Tested against
  synthetic "fingerprint" patches stamped through the real MH_01
  calibration (recovers a 3m-depth point to ~15cm, consistent with
  1px-disparity-step precision at that depth — see the module's test
  comment for the error-budget math).
- `slam-frontend::vo`: `VoPipeline` — a stereo-only (no IMU yet), no-
  backend-optimization VO loop. `init()` stereo-matches+triangulates an
  initial landmark map (world frame = frame 0's cam0 frame). Each
  `process_frame()` LK-tracks existing landmarks, estimates pose via
  `estimate_pose_dlt` + `refine_pose_gauss_newton` (M1's PnP, its first
  real caller), and triggers a new keyframe (fresh stereo matching, more
  landmarks) once the live track count drops below a threshold — with a
  pixel-proximity filter so new keyframes don't pile up near-duplicate
  landmarks next to still-alive tracks.
- End-to-end checkpoint test (`slam-frontend`'s `integration_tests`): runs
  `VoPipeline` over 150 real MH_01_easy frames (~6.4s @ 20Hz), aligns onto
  ground truth via Umeyama, computes ATE. **Result: RMSE 13.7cm** — no
  tracking loss, no divergence. `bin/slam-inspect` now reports the same
  checkpoint for all five `MH_*` sequences: RMSE ranges 11-17cm over
  ~100-130 groundtruth-covered poses each, all landmark counts in the
  hundreds, zero tracking-loss events across all five. This is explicitly
  *not* the SOTA VIO bar from `plan/STAGE1.md` (2-9cm needs IMU fusion +
  backend optimization, M4/M5) — it's proof the frontend produces a
  geometrically sane trajectory at all, which is what M3 asks for.
- `cargo test --workspace` (54 tests, ~40s dominated by the VO integration
  test in debug mode) and `cargo clippy --all-targets` both clean.

## Not done yet (correctly out of scope for M3)

- IMU fusion, sliding-window backend, marginalization, loop closure,
  global BA — M4 through M8.
- Outlier rejection beyond the SSD threshold in stereo matching and the
  implicit (no explicit RANSAC) robustness of DLT PnP — M6.
- Full rectified-image remap for stereo matching, if the raw-space patch
  approximation turns out to be a real accuracy bottleneck once the
  backend is in place and errors need to shrink further — noted as a
  possible revisit, not committed to now (YAGNI until proven necessary).

## See also

- `decisions/0004-umeyama-ate-brought-forward-to-m3.md` — why alignment/ATE
  exist in `slam-eval` already, ahead of the plan's literal M9 assignment.
