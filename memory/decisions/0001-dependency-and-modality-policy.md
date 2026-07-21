---
date: 2026-07-20
status: accepted
---

# 0001: Dependency policy + target modality for Stage 1

## Decision

1. **Dependencies**: use standard Rust infrastructure crates freely
   (`nalgebra` for linear algebra, `image`/`png` for decoding, `serde` +
   `serde_yaml` + `csv` for parsing, `rayon` for parallelism, `anyhow`/
   `thiserror` for errors). Implement all SLAM-specific algorithms
   ourselves: feature detection/tracking, stereo matching, IMU
   preintegration, the nonlinear least-squares optimizer, marginalization,
   loop closure/place recognition, pose-graph optimization. No OpenCV
   bindings, no g2o/GTSAM/Ceres bindings, no pre-built SLAM/VO crates.
2. **Modality/accuracy bar**: target full stereo + IMU (visual-inertial)
   SLAM with loop closure and global BA — not stereo-only VO and not
   monocular-inertial — because that's the class of system ("ORB-SLAM3
   stereo-inertial", OKVIS, VINS-Fusion, Kimera-VIO) that defines "state of
   the art" accuracy on EuRoC `machine_hall` (roughly 2-9cm ATE RMSE
   depending on sequence).

## Alternatives considered

- Nearly-from-scratch (own linear algebra/image codecs too): rejected as
  disproportionate effort for no accuracy benefit — the value is in the
  estimation algorithms, not reimplementing PNG decoding.
- Allow CV helper crates (existing ORB/feature-detector crates, or OpenCV
  for undistortion): rejected to keep the estimation *and* frontend
  algorithm work meaningfully "ours," per the user's framing of the project
  as writing the SLAM program "by yourself."
- Stereo-only VO first, add IMU later: rejected as the primary target
  (still useful as an intermediate checkpoint — see `plan/STAGE1.md` M3 —
  but not where Stage 1 stops) because stereo-only VO tops out well short of
  the accuracy bar the user asked for.
- Monocular + IMU: rejected — harder to reach top accuracy than stereo
  (scale must be inferred/converge from IMU rather than being available
  directly from the baseline), and the dataset gives us a calibrated stereo
  pair, so there's no reason to throw that away for Stage 1.

## Source

User explicitly chose these via an `AskUserQuestion` prompt when
`plan/STAGE1.md` was first written (recommended options in both cases).

## Implications for later work

- Any future stage targeting a different dataset without stereo (e.g.
  monocular-only footage) will need a new initializer and won't get scale
  for free — don't assume the M4 initializer design ports as-is.
- If accuracy targets turn out to be unreachable with the "no CV helper
  crates" constraint within reasonable effort, that's a decision to revisit
  explicitly with the user, not to quietly relax.
