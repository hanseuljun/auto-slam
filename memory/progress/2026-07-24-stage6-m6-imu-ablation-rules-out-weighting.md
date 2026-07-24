Stage 6 M6 done: real IMU-factor ablation rules out the weighting-
imbalance hypothesis for the anisotropic scale distortion M5 found.

Added VioParams::disable_imu_factors, threaded through the 3 places IMU
factors actually enter the optimizer (run_optimization,
global_bundle_adjustment_inner, marginalize_evicted_keyframe's prior -
deliberately not the track-loss-recovery fallback, a separate "no vision
at all" scenario). Wired a bin/slam-run --disable-imu-factors flag to
run this on real full sequences with the same M5 evaluation code.

Result: catastrophic divergence, not a cleaner isotropic reconstruction
hiding underneath a bad weight. Both tested sequences got 3-4 orders of
magnitude worse on every metric (MH_01_easy: 685->2236 keyframes,
319->2055 track-loss recoveries, z-axis anisotropy 14.03->4664,
loop-closure gap 81.66m->72210m; MH_04_difficult similarly). This
directly answers M6's own test: scale does not stay stable without IMU
factors, so the weighting-imbalance hypothesis is ruled out - IMU
information is load-bearing for basic stability, not a magnitude tuning
knob.

Mechanism: reprojection factors only touch 6 of KeyframeState's 15
dimensions (pose). Without IMU factors, velocity/bias (9 dimensions) get
zero information from any factor, and marginalize_evicted_keyframe's
Schur complement folds that near-arbitrary uncertainty into the
carried-forward prior at every eviction - compounding over hundreds to
thousands of evictions across a full sequence. This matches the plan's
own predicted alternative ("marginalization's own Schur-complement
accumulation") almost exactly.

M7 should investigate that mechanism directly (does the marginalization
prior's bias/velocity block grow pathologically even with IMU factors
present, just more slowly?) rather than continue looking for a
weighting fix this measurement shows doesn't exist.

Full details: memory/decisions/0028.
