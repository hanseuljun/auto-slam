# M7 — loop closure

Landed the eighth milestone from `plan/STAGE1.md`, following M0-M6. This
was the largest single milestone of the session in debugging surface area
— four real bugs found and fixed before the checkpoint worked, two of
them outside `slam-loopclosure` itself.

## What's done

- `slam-vision::descriptor`: a 256-bit BRIEF-style binary descriptor (own
  fixed, deterministic sampling pattern — not true BRIEF's Gaussian
  sampling, but the same idea) with Hamming distance.
- `slam-loopclosure`: `Vocabulary` (flat k-means in Hamming space, nearest-
  centroid assignment, per-bit-majority centroid update — the standard
  binary-descriptor adaptation of k-means), `KeyframeDatabase` (BoW L1
  similarity query, temporally-gapped), `verify_loop_candidate`
  (descriptor matching + DLT/Gauss-Newton PnP + inlier-count gate),
  `optimize_pose_graph` (SE3-only LM, numerical Jacobians — same
  `decisions/0006` tradeoff as the IMU factor), `capture_loop_keyframe`
  (bundles stereo-matched landmarks with descriptors for a keyframe).

## Real checkpoint

`loop_closure_measurably_improves_ate_on_mh05`
(`crates/slam-loopclosure/src/tests_integration.rs`, `#[ignore]`d —
full-sequence + vocabulary training is genuinely expensive, ~40s release/
~15min debug, confirmed not estimated): runs stereo VO over the full real
MH_05 sequence, detects and verifies the sequence's actual loop (keyframe
22 <-> 111, 38/38 inliers), and pose-graph-optimizes. **ATE: 5.613m
without loop closure -> 3.293m with — a genuine 41% improvement**,
comfortably clearing the plan's own bar ("measurable ATE improvement").
`bin/slam-inspect` runs the same path for MH_05 on every normal invocation.

## Four real bugs found before this worked (all fixed, all worth
## remembering)

1. **Absolute Hamming threshold alone let false matches through.**
   20-90 "matches" per candidate pair, consistently 0-2 geometric inliers
   out of dozens, across the first ~150 verification attempts tried. A
   Lowe's-ratio-test filter (best match must clearly beat second-best)
   fixed this immediately — order statistics on 256-bit Hamming distances
   over a few hundred candidates make an absolute cutoff a much weaker
   filter than it looks. `decisions/0008`.
2. **Pose-graph LM diverged on the real (114-node, unevenly-weighted)
   graph** — a fixed absolute initial damping (`lambda=1e-3`, copied from
   `slam-optim`'s solver) was sized for a different problem's edge-weight
   scale; the very first step overshot into a region where `SE3::log()`'s
   rotation-angle aliasing above π made a nonsensical pose score a
   deceptively low cost, so the solver kept "successfully" accepting
   catastrophic steps. Fixed by scaling initial lambda to the Hessian's
   own diagonal magnitude (the standard Marquardt heuristic) instead of a
   fixed constant. Caught by literally printing the total pose shift
   after optimization (1e22m) rather than trusting a plausible-looking
   ATE number — see finding 4 below for why that distinction mattered.
3. **Used an absolute pose as if it were a relative pose-graph edge.**
   `verify_loop_candidate`'s returned pose is a PnP result against
   landmarks already expressed in the shared rolling VO world frame — an
   independent *absolute* estimate of the current keyframe's pose, not a
   transform relative to the candidate keyframe. Feeding it directly into
   `PoseGraphEdge` (which expects a true relative transform) produced a
   nonsensical edge; this is what was actually behind bug 2's divergence
   (bug 2's LM fix was still worth making, but wasn't the root cause of
   *this* symptom). Fixed by composing with the candidate's own
   pre-optimization VO pose to get a proper relative edge.
4. **`VoPipeline` was silently producing corrupt poses on the real
   full-MH_05 run** (translations up to ~1e20m, starting ~keyframe 23,
   self-reinforcing since each corrupt pose becomes the reference for
   triangulating the next landmarks) — a real regression in M3/M6 code,
   invisible to M6's own full-sequence checkpoint because
   `slam_eval::compute_ate`'s Sim3 alignment can hide a sufficiently
   smooth large distortion in the trajectory shape. Root cause: M1's
   known, documented DLT-PnP limitation (no RANSAC/outlier rejection,
   `decisions/0003`) hit in the wild on a long run. Fixed with a pose-
   jump sanity check (`VoParams::max_pose_jump_meters`, `decisions/0009`)
   that rejects an implausible PnP result and triggers M6's recovery path
   instead of accepting it. Bonus: full-sequence ATE on all five
   sequences *improved* after this fix (e.g. MH_01 4.3m -> 3.4m), not just
   "stopped being wrong" — rejecting bad poses and recovering produces a
   better trajectory than accepting them.

## Not done yet (correctly out of scope for this M7 pass)

- Orientation-invariant descriptors (oriented BRIEF/rBRIEF) — the current
  descriptor has no rotation invariance; MH_05's loop happened to verify
  anyway (compatible viewing angle on revisit), but a loop revisited from
  a substantially different heading might not. Worth revisiting if a
  future sequence's loop doesn't verify.
- Hierarchical/scalable vocabulary (DBoW2-style vocabulary tree) — flat
  k-means is fine at Stage 1's scale (hundreds of keyframes), would not
  scale to a much longer-running system.
- Applying `decisions/0009`'s pose-jump check to `VioPipeline` (M5) too —
  it shares the same underlying PnP, wasn't hit by any test yet, but
  isn't protected either.
