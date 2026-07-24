Stage 6 M5 done: built real scale-drift instrumentation, measured on 2
sequences, and found the scale anomaly is anisotropic, not gradual or a
step-change as the plan originally framed the question.

compute_sliding_window_scale (slam-eval) fits Umeyama over a sliding
window, giving a local-in-time scale instead of one whole-trajectory
number. On MH_01_easy this swung wildly and non-monotonically (0.016 to
0.228), not describable as gradual drift or a clean step. Confirmed this
wasn't a window-size or loop-closure artifact (same wave shape at
20s/60s/90s, and pre/post loop correction).

Investigating why led to the real finding: the error isn't isotropic.
Built compute_axis_scale_ratios (rotates estimated into groundtruth's
frame first via Umeyama's own fitted rotation, then compares per-axis
variance - a raw non-rotated comparison is meaningless since each
trajectory's world frame is arbitrary). Measured real per-axis
anisotropy on MH_01_easy (x=3.95 y=2.74 z=14.03) and MH_04_difficult
(x=1.12 y=1.60 z=2.10) - Z is the worst axis on both sequences, by a
wide margin on MH_01. A single isotropic Umeyama scale can't represent
this; the sliding-window noise is the symptom of fitting one number to
an inherently anisotropic problem.

Also found: MH_01's better ATE (4.058m) coexists with 7x worse
anisotropic distortion than MH_04's (6.279m ATE) - the isotropic
Sim3-aligned ATE metric absorbs real anisotropic error into its scale
parameter, so it can look more accurate than the reconstruction actually
is. Sharpens decisions/0020's own tentative worry into a measured
mechanism.

Kept both functions as tested, reusable instrumentation - bin/slam-run
now prints the scale-drift table and anisotropy ratio for every run,
permanently. Added crates/slam-eval/examples/scale_probe.rs, a small
standalone tool for probing any saved trajectory.csv at any window size
without re-running the pipeline.

Full details: memory/decisions/0027.
