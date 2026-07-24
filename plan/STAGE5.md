# Stage 5: Honest drift measurement, and real loop closure

## Goal

Two goals, in the order the user gave them:

1. **ATE should be near zero close to the start of a trajectory.**
   Ground truth is available from (near) t=0, and there's no time for
   drift to have accumulated yet — an estimator that's actually tracking
   correctly should show small error there. Right now it doesn't (see
   "What we already know" below): reported ATE near t=0 is large, on
   every sequence, and gets *larger* the longer the run continues past
   that point — even though the underlying pose estimate for those early
   frames doesn't change. That's the metric, not the tracking, and needs
   fixing before goal 2's "did loop closure actually help" can be judged
   honestly.
2. **Every sequence in this dataset is itself a loop** — the drone takes
   off and lands back near where it started — **but ATE stays high
   regardless.** Loop closure (`plan/STAGE1.md` M7) exists in this repo
   and measurably helps where it's been tried, but it was never wired
   into the pipeline `bin/slam-run` actually reports numbers for, and
   was only ever demonstrated on one sequence. Make loop closure real:
   detect and correct the loop on every sequence that has one, inside
   the actual VIO pipeline, and verify geometrically that the loop is
   actually closed — not just that a number went down.

Same dependency policy, same dataset, same determinism requirement, same
"measure before fixing" discipline as Stages 1-4. Goal 1 comes first
because goal 2 needs a trustworthy ruler to be judged against — using
today's metric, a real loop-closure win could look smaller (or bigger)
than it actually is, for reasons that have nothing to do with whether
the loop was actually closed.

## What we already know (measured this session, not assumed)

**Finding 1: near-t=0 ATE is large, and gets worse the longer the run
continues, even though the early pose estimate itself doesn't.**
Compared `MH_01_easy`'s raw (pre-alignment) estimated poses for its
first ~20 keyframes between the bounded 600-frame run and the full
3682-frame run: nearly identical (max diff ~0.10m, consistent with
small global-BA-scope differences, not tracking divergence — see `plan/
STAGE4.md` M1's `max_global_ba_keyframes`). Yet the *reported* ATE at
that same early point, after this repo's existing Umeyama alignment
(`crates/slam-eval/src/align.rs`, `umeyama_alignment` — a full
similarity transform fit over *every* point in the trajectory, at once)
is 0.18m on the bounded run and **3.1m on the full run** — a ~17x
difference for frames whose underlying estimate barely changed. The
mechanism: Umeyama's least-squares fit is a compromise over *all* points
supplied to it; once later keyframes have drifted far (`plan/
STAGE4.md`'s own M2 finding — full-sequence drift without loop closure
is real and large), fitting them well pulls the transform away from what
would best align the early, still-accurate portion. The metric is
*spreading* later drift backward onto earlier frames that were never
actually wrong. Confirmed directly: re-aligning `MH_01_easy`'s full-run
trajectory using only its first 10 keyframes (instead of all 725) drops
err[0] to 0.07m — but the same fit produces enormous downstream error
(rmse 88m over the whole trajectory, err at the end reaching 138m) that
the current whole-trajectory fit was quietly absorbing. Neither number
is "the fix" on its own (see Finding 3) — but both make the same point:
today's single metric can't distinguish "tracked well, drifted late" from
"was wrong from the start," and goal 1 is about making that
distinguishable.

**Finding 2: every sequence is a loop, not just `MH_05_difficult`.**
`memory/notes/dataset-quirks.md` already documented this for MH_05
("position at t=111s is within 0.15m of the t=0s start position, after
~98m of travel"). Checked all five sequences' raw groundtruth CSVs the
same way this session (closest-approach pair with >=15s time separation
*and* a real distance traveled between them, to rule out "the drone was
just hovering," not merely "closest any two points ever get"):

| Sequence | duration | distance between start & closest-matching later point | path traveled in between |
|---|---|---|---|
| MH_01_easy | 181.9s | 0.25m | 80.5m |
| MH_02_easy | 150.0s | 0.16m | 73.3m |
| MH_03_medium | 131.5s | 0.14m | 130.6m |
| MH_04_difficult | 98.7s | 0.26m | 91.6m |
| MH_05_difficult | 111.0s | 0.13m | 97.5m |

All five: start and end are within ~0.13-0.26m of each other after
70-130m of real flight — every `machine_hall` sequence takes off and
lands back at (approximately) its starting point. Loop closure isn't a
one-sequence special case here; it's the dataset's own structure, and
this repo's current numbers ignore it on 4 of 5 sequences entirely.

**Finding 3: loop closure exists, works, and is not wired into the real
pipeline.** `crates/slam-loopclosure` (`plan/STAGE1.md` M7) has BoW
vocabulary training, keyframe database query, geometric verification,
and pose-graph optimization — real, tested code. But `bin/slam-run`
(what produces every number in `docs/RESULTS.md`) hardcodes
`loop_closure_seconds: 0.0` and never calls into `slam_loopclosure` at
all. The only place it's demonstrated is `bin/slam-inspect`'s
`print_loop_closure`, hardcoded to run only when `name ==
"MH_05_difficult"` (`bin/slam-inspect/src/main.rs:105`), and only on
`VoPipeline` (pure visual odometry — no IMU, no windowed backend, no
global BA), not `VioPipeline`. Re-ran it fresh this session:
`ATE without=5.613m with=3.293m` — real, but a 41% reduction leaving
3.3m of error on a loop that closes to within 0.13m in ground truth is
not "properly closed" by any reasonable bar, and this number has never
been measured on the other 4 sequences at all, nor on the actual
IMU-fused pipeline the rest of this project's numbers come from.

**Finding 4 (new, unprompted, but directly relevant to goal 1): the
alignment's own fitted *scale* is far from 1.0, and gets worse the
longer the run continues — worth root-causing, not just working around.**
`umeyama_alignment` fits a full similarity transform (rotation +
**free scale** + translation), appropriate for monocular VO where scale
is fundamentally unobservable — but this is a *stereo-inertial* system,
where metric scale should be directly observable (known stereo baseline
+ IMU) and should need no correction close to 1.0 if reconstruction is
metrically sound. Measured fitted scale on `MH_01_easy`: **0.0173** over
the full 725-keyframe trajectory, **0.0216** over the bounded 101-keyframe
clip — both far from 1.0, and moving further from it as the window
grows. Checked whether this is just small-sample noise by sweeping the
alignment window size (10/30/60/100/150/200 keyframes): scale ranges
from 0.02 to 0.47 depending on window, never near 1.0, non-monotonically.
Directly measured raw (pre-alignment) path length vs. ground truth's own
path length: on the full run, the estimated trajectory travels **715.75
"m" while the ground truth only covers 78.79m** (ratio 0.11); on the
bounded clip, 32.39 vs. 6.62m (ratio 0.20). **The estimator thinks it's
travelling several times farther than it actually is, and increasingly
so the longer it runs** — consistent with compounding dead-reckoning-
style drift (plausibly connected to `plan/STAGE4.md` M2's own finding
that 45-52% of all keyframes are IMU-only track-loss-recovery coasts,
though that's a hypothesis to test, not yet confirmed) rather than a
fixed calibration constant, since a fixed miscalibration would produce
roughly the same ratio regardless of how long the run continues, and it
doesn't. Two consequences worth Stage 5 actually resolving, not
guessing at: (a) a free-scale (Sim3) alignment can fully hide a
scale/drift problem like this from ATE — it's *exactly* the kind of
thing goal 1's metric fix needs to stop absorbing silently; (b) published
stereo-inertial SOTA systems (the ORB-SLAM3/OKVIS/VINS-Fusion/Kimera
numbers `docs/RESULTS.md` compares against) conventionally report ATE
under a **fixed-scale** (SE3, not Sim3) alignment, since their systems
also have observable metric scale — meaning this repo's existing
`docs/RESULTS.md` comparison table may itself not be apples-to-apples,
independent of goal 1's original start-of-trajectory motivation.

## Milestones

Same discipline as every prior stage: measure before fixing, fix before
declaring done, no milestone closes on an assumed number.

### M0 — Root-cause the scale/drift finding, decide what "honest ATE" means here — Done

- Finding 4 above is a real, measured anomaly, not yet a diagnosis.
  Determine whether the free-scale absorption is masking (a) a genuine
  reconstruction-scale bug (stereo baseline/extrinsics, camera
  intrinsics, or an IMU/gravity-magnitude unit mismatch — check with
  synthetic/known-scale test cases the same way `plan/STAGE1.md`'s own
  IMU/geometry milestones did) or (b) compounding dead-reckoning drift
  from track-loss recovery (`plan/STAGE4.md` M2's 45-52% recovery rate)
  with no single "bug" to fix, or (c) something else. Profile/test,
  don't assume — same bar as every prior stage's root-cause milestones.
- Decide, with evidence (use `AskUserQuestion` if it's a real call the
  user should weigh in on, e.g. whether to keep the existing Sim3-based
  numbers for SOTA comparability alongside a new metric, or replace the
  headline metric outright): what alignment methodology `docs/
  RESULTS.md`'s numbers should use going forward. The naive "align to
  just the first few keyframes" tried this session is *not* the answer
  as-is — it's numerically unstable (a poorly-conditioned small early
  window's rotation uncertainty has a lever-arm effect: a small angular
  error translates into tens-to-hundreds of meters of apparent error far
  from the anchor, confirmed by the 10-point-anchor test's own 88m rmse
  blowup). Needs a principled choice (e.g. a larger/weighted trusted
  prefix, a fixed-scale-not-free-scale alignment, or some other
  approach) — investigated and measured, not the first thing that
  happens to make one number look better.
- Test/deliverable: a written decision (`memory/decisions`) backed by
  real comparative numbers across multiple candidate alignment
  strategies on all 5 sequences, not a single anecdote.
- **Result**: ruled out a calibration/geometry bug (baseline computed
  directly from `sensor.yaml` matches the known EuRoC value, and
  existing triangulation/IMU-propagation tests already validate
  sub-mm/1e-3 accuracy against synthetic ground truth) and ruled out
  track-loss recovery events disproportionately inflating raw path
  length (recovery-tagged steps are 51.3% of path length from 51.4% of
  steps — proportional, not anomalous). Confirmed real: the pipeline's
  own reconstructed scale genuinely drifts over long runs (forcing
  scale=1.0 in the alignment gives 150-297m error vs. 5-140m for
  free-scale, at the same windows) — a real estimator-behavior question
  (likely windowed-optimizer residual weighting letting scale creep),
  substantial enough to be its own future stage, out of Stage 5's scope
  to fix. **Decision**: keep Sim3 (free-scale) alignment — forcing
  scale=1.0 would just re-expose an already-flagged, separately-scoped
  problem as a bigger number, not what goal 1 needs — but fit it using a
  bounded ~60-150-keyframe prefix (~30s, reusing the existing
  bounded-clip duration concept) instead of the entire trajectory.
  Swept window sizes 10-725 on two sequences (`MH_01_easy`,
  `MH_05_difficult`, both full runs): k=60-100 gives near-zero err[0]
  (0.165-0.215m) without the small-window lever-arm instability k=10
  showed (88-297m blowups). Full writeup with the complete sweep table:
  `memory/decisions/0020`. Verified on 2 of 5 sequences at this stage
  (time-efficient spot-check, not all 5) — M1's own test criteria
  re-verifies on every sequence once implemented in Rust, per that
  milestone's existing bar.

### M1 — Implement the chosen ATE methodology

- Implement M0's decision in `crates/slam-eval` (new function, changed
  default, or both — whatever M0 concluded). If the existing Sim3
  Umeyama alignment is kept for SOTA-comparability reasons, it must be
  clearly labeled as such (not presented as the only or default
  "accuracy" number) alongside whatever new metric actually reflects
  drift honestly.
- Test: near-t=0 ATE (first few keyframes) is small on every sequence,
  under both the bounded-clip and full-sequence run modes, and — the
  real regression check — doesn't get *worse* just because a run
  continues longer past that point (unlike today's metric, confirmed
  above to do exactly that). Existing `cargo test` coverage in
  `crates/slam-eval` extended to cover the new alignment's own
  properties (e.g. a synthetic trajectory that's accurate early and
  drifts late should show small early error and large late error, not
  spread evenly, mirroring `align.rs`'s own existing test-style
  discipline).

### M2 — Wire real loop closure into `bin/slam-run`'s actual pipeline

- Baseline first: run the existing `slam_loopclosure` detection +
  geometric verification (currently only exercised for `MH_05_difficult`
  in `bin/slam-inspect`) against all 5 sequences, not just one, to
  confirm the vision-based loop *detector* actually finds and verifies
  the loop Finding 2 shows exists in ground truth on each — appearance
  change/lighting/viewpoint could plausibly make detection fail even
  where a real loop exists geometrically; don't assume it'll "just work"
  on the other 4 the way it happened to on MH_05.
- Integrate into `bin/slam-run`'s actual per-sequence run: capture
  loop-closure keyframes during the existing per-frame loop (reusing the
  `left`/`right` images already being loaded — no new I/O), then run
  detection/verification/pose-graph optimization as a **post-processing
  pass over the trajectory `global_bundle_adjustment` produces**, mirroring
  how global BA itself is already a distinct one-shot pass rather than a
  per-frame pipeline change (lower risk, reuses an established pattern
  from `plan/STAGE1.md` M8 and `plan/STAGE4.md` M1). The corrected poses
  must actually feed the trajectory used for ATE/RPE and
  `runs/<sequence>/trajectory.csv` — not a side calculation like `bin/
  slam-inspect`'s current demo, which computes `ate_with`/`ate_without`
  purely for its own printout and never touches the pipeline's real
  output.
- `TimingBreakdown::loop_closure_seconds` already exists as a field
  (currently always 0.0, `bin/slam-run/src/main.rs`) — populate it for
  real, and re-check the whole-run real-time bar (`plan/STAGE4.md` M1's
  `whole_run_factor()`) still holds once loop closure's own cost is
  included; measure, don't assume it's free.
- Test: `bin/slam-run` (default full-sequence run, `plan/STAGE4.md` M3)
  detects and applies a verified loop correction on every sequence where
  M2's own baseline check confirms one's detectable, with real
  before/after timing.

### M3 — Verify the loop is actually closed, not just that a number moved

- A geometric check, not just an aggregate metric: for a sequence whose
  ground truth returns within ~0.13-0.26m of its start (Finding 2, all
  five), the *corrected* estimated trajectory's own start and end points
  must also end up close together after M2's fix — a direct,
  human-checkable sanity test that doesn't depend on M1's alignment
  choice being exactly right. If the loop-closed trajectory's start and
  end are still far apart, the loop wasn't really closed regardless of
  what any aggregate number says.
- Re-measure ATE/RPE with M1's honest metric, before and after M2's loop
  closure, on all 5 sequences — real numbers, matching the before/after
  discipline every prior stage's fix milestones used (`plan/STAGE2.md`
  M1's own "Result" note, `plan/STAGE4.md` M1's own before/after, are
  the templates).
- Update `docs/RESULTS.md` (a new or revised headline table reflecting
  the honest metric + real loop closure — including revisiting whether
  the existing SOTA comparison table needs a fixed-scale caveat per
  Finding 4) and `README.md`'s status summary.
- Test: for every sequence with a detected loop, start/end position gap
  in the corrected trajectory is small (not necessarily as tight as raw
  groundtruth's ~0.13-0.26m, but a real, order-of-magnitude improvement
  over the uncorrected trajectory's own start/end gap — measured, with
  the actual number reported, not just "it went down").

## Out of scope for Stage 5

Same carried-forward list as Stages 1-4 (dense/mesh reconstruction,
multi-session/map-merging, semantic mapping, non-`machine_hall` EuRoC
rooms, other datasets, GPU/SIMD micro-optimization, real-time targets
beyond this repo's own dev machine), plus: multi-loop / pose-graph-wide
relaxation (this stage closes *the* loop each sequence has — its
start-equals-end structure — not a general multi-loop SLAM backend);
re-litigating Stage 4's real-time bar beyond confirming loop closure
doesn't quietly break it (M2's own test); and any further accuracy
tuning of the windowed/global-BA solver itself (Stage 2 M6 already
closed that milestone — Stage 5 is about measurement honesty and loop
closure specifically, not another knob-sweeping pass).

## Risks

- **Finding 4 (the scale anomaly) could turn out to be a rabbit hole.**
  It wasn't part of the user's original two goals, and M0 could spend
  real time without a clean single-cause answer (Finding 4 itself
  already hedges between a calibration bug and compounding drift). If
  M0's investigation doesn't converge, document what's ruled in/out and
  move on to M1 with a metric that's honest about drift regardless of
  root cause — goal 1 doesn't strictly require Finding 4 to be fully
  explained, only that the metric stop hiding it.
- **A start-anchored (or otherwise reweighted) alignment is a real
  methodology change to every historical number in `docs/RESULTS.md`.**
  Old and new numbers won't be directly comparable without care; M1's
  own test criteria and M3's doc updates need to make clear which metric
  any given number uses, not silently swap definitions under an
  unchanged column header.
- **Applying loop-closure correction as a post-processing pass (M2) risks
  becoming out of sync with `global_bundle_adjustment`'s own bounded
  scope** (`plan/STAGE4.md` M1's `max_global_ba_keyframes`) — correcting
  poses *after* global BA already ran means the two passes need a clear,
  tested contract for which one's output is authoritative for which
  keyframes, not an assumed-safe ordering.
- **This could look similar to Stage 4's own "flip a default before
  measuring" wound if rushed.** M2/M3 must confirm loop closure actually
  helps, with real before/after numbers on every sequence, before
  `bin/slam-run`'s default behavior is treated as "loop-closed" in
  `README.md`/`docs/RESULTS.md` — same discipline `plan/STAGE4.md`'s own
  Risks section named for its own default-flip.
