---
name: vo-rejects-implausible-pose-jumps
description: VoPipeline now rejects PnP poses implying an implausible per-frame translation jump, treating them as track loss instead of accepting them — a real corruption found by M7's full-MH_05-sequence test, invisible to M6's own full-sequence checkpoint because Sim3-aligned ATE can mask it.
metadata:
  type: decision
---

# Decision: VoPipeline rejects implausible PnP pose jumps

## Decision

`slam_frontend::VoPipeline::process_frame` (`crates/slam-frontend/src/vo.rs`)
now rejects a PnP-estimated pose outright — treating it as track loss and
triggering the M6 recovery path — if it implies a per-frame translation
jump larger than `VoParams::max_pose_jump_meters` (default 2.0m, already
far beyond plausible MAV motion at these datasets' ~20Hz frame rate).

## Why

Building M7's real MH_05 checkpoint (running `VoPipeline` over the *full*
2273-frame sequence, not a short clip) surfaced that `estimate_pose_dlt`
(M1, no RANSAC/outlier rejection — `decisions/0003`) occasionally produces
a catastrophically wrong pose from a degenerate point configuration while
still numerically "succeeding": observed translations up to ~1e20 meters,
starting around keyframe 23 and persisting (each corrupt pose becomes the
reference for triangulating the next batch of landmarks, so the corruption
is self-reinforcing once it starts). `process_frame` never returned `None`
for these frames — DLT + refine "worked," so nothing flagged it.

**This was invisible to M6's own full-sequence robustness checkpoint.**
That test's success criterion was "zero unrecoverable frames" plus a
plausible-looking ATE number, and it reported MH_05 at 6.877m RMSE — high
but not obviously broken for a VO-only full-sequence run. The reason:
`slam_eval::compute_ate` Sim3-aligns (scale + rotation + translation)
before computing residuals, and a sufficiently smooth/large distortion in
the trajectory's shape can still align tolerably well, hiding the
corruption in the reported number. Confirmed directly by dumping
`vo_poses` and checking orthonormality/translation-magnitude sanity, not
by ATE alone.

## How to apply

Don't treat "Sim3-aligned ATE looks plausible" as proof a trajectory is
sane — it's a good *accuracy* metric once you already trust the poses are
valid SE3 elements with bounded magnitude, not a corruption detector. If a
future milestone's checkpoint gives a suspiciously round or unexpectedly
*stable* ATE number across parameter changes that should matter, consider
directly sanity-checking the underlying pose sequence (orthonormality,
translation magnitude, finiteness) the way this was caught, rather than
trusting the aggregate metric alone.

`VioPipeline` (M5) uses the same DLT+refine PnP for its own pose guess and
likely shares this vulnerability — it wasn't hit here (M7's checkpoint
only runs `VoPipeline`), but isn't yet protected by an equivalent check.
Worth applying the same fix there if a similar corruption shows up in a
future full-VIO-sequence test.
