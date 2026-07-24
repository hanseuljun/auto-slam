Stage 6 M7 done, Stage 6 complete: tested M6's own marginalization
hypothesis directly, found the opposite of what was predicted, and
concluded the anisotropic scale distortion's root cause remains open -
a legitimate, plan-sanctioned outcome, not a failure.

Added VioParams::disable_marginalization (naive drop, decisions/0007's
original alternative) and a bin/slam-run --disable-marginalization flag.
If marginalization's Schur-complement accumulation were really
compounding the anisotropy, removing it should have helped. Instead, on
both tested sequences, disabling marginalization made anisotropy worse
(MH_01_easy: 14.03->362 z-axis; MH_04_difficult: 2.10->46.8), sitting
between normal VIO and full IMU removal on every metric - marginalization
is doing real, measurably stabilizing work relative to naive drop, not
accumulating the distortion.

Combined with M6's own result (IMU-vs-vision weighting ruled out), both
concrete candidate mechanisms this investigation produced are now ruled
out by direct measurement, not just reasoned past. Per the plan's own
explicit permission for this situation, the anisotropy's root cause
remains open, documented honestly:
- Not IMU/vision weight imbalance (M6).
- Not marginalization's own accumulation mechanism (M7).
- Present even in the best-behaved (normal) configuration tested - a
  property of the baseline reconstruction, not an artifact of either
  ablatable mechanism.

Candidate directions for a future stage (not attempted): stereo
triangulation depth-direction bias, camera-IMU extrinsics calibration
error, the static/dynamic initializer's own gravity-direction handling.

With M7 concluded, all of Stage 6's milestones are landed: M1/M2 (real
Jacobian + covariance work, both with real accuracy effects), M3/M4
(sparse solver + stride reduction, a real measured win), M5/M6/M7 (real
instrumentation reframing the scale question, then two real ablations
ruling out both candidate mechanisms and leaving a well-characterized
open question instead of a false "fixed").

Full details: memory/decisions/0029.
