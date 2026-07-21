# M2 — vision frontend primitives

Landed the third milestone from `plan/STAGE1.md`, following M0/M1.

## What's done

- `slam-vision`: `ImagePyramid` (own 2x2 box-filter downsample), FAST-9
  corner detector (`detect_fast`, full 16-point Bresenham-circle test with
  circular run-length checking) with grid-based non-max suppression
  (`detect_grid`, caps keypoints per cell for even spatial distribution —
  the ORB-SLAM-style approach the plan calls for), and pyramidal
  Lucas-Kanade-Tomasi optical flow (`track_pyramid` — forward-additive,
  per-level structure-matrix/determinant trackability gating, coarse-to-
  fine displacement propagation). Chose LK as the primary temporal tracker
  per the plan's own recommendation; a descriptor for loop closure is
  deferred to M7 (no consumer yet, same YAGNI reasoning as M1's P3P/EPnP
  deferral).
- 10 unit tests (pyramid shape/averaging, FAST on a synthetic checkerboard
  corner + flat-image rejection, grid NMS per-cell cap, LK on a known
  synthetic translation + flat-region rejection + bilinear sampling) plus
  1 integration test tracking real grid-FAST keypoints across consecutive
  real MH_01 frames and asserting both spatial distribution (no
  clustering) and majority track survival.
- Extended `bin/slam-inspect` with a "vision frontend" section: keypoint
  count on frame 0 + LK survival rate/percentage across 5 real frames, for
  all five sequences. Observed 93-97% survival across all `MH_*`
  sequences — see `notes/lk-tracker-gotchas.md` for this as an informal
  regression baseline.
- `cargo test --workspace` (41 tests) and `cargo clippy --all-targets`
  both clean.

## Bugs hit and fixed during verification (see `notes/lk-tracker-gotchas.md`)

1. **Coarse-level window-out-of-bounds was killing entire tracks.** First
   version treated *any* pyramid level's tracking window falling outside
   that level's (possibly tiny) image as total track failure. On real
   752x480 images with a 15x15 window, this failed nearly every track at
   the coarsest pyramid level. Fixed: only a level-0 (finest) failure
   marks the track lost; coarser-level failures just skip refinement there
   and keep the propagated displacement guess.
2. **Synthetic test fixture picked edge-midpoint tracking points**, which
   are a textbook aperture-problem degenerate case (1D gradient, singular
   structure matrix) — correctly rejected by the tracker, incorrectly
   expected to succeed by the test. Fixed by moving test points near the
   synthetic square's corners.

Both are documented in `notes/lk-tracker-gotchas.md` rather than
`decisions/`, since they're bug fixes recovered via testing against real
data (exactly the "verify by running the real thing" value CLAUDE.md's
test-app mandate is for), not a choice between genuine design alternatives.

## Not done yet (correctly out of scope for M2)

- Descriptors (BRIEF-style) for loop closure/relocalization — M7.
- Forward-backward consistency checks, NCC-based match verification,
  outlier gating beyond the structure-matrix determinant threshold — M6
  ("Robust tracking & map maintenance").
- Stereo matching (cam0-cam1) and keyframe selection — M3, next.
