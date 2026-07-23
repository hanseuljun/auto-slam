---
name: stage4-m0-mh01-full-sequence-measured
description: Stage 4 M0 (in progress) — first real full-sequence measurement (MH_01_easy). Confirms the global-BA bottleneck hypothesis via live profiling, but also finds two problems, not one: the real_time_factor metric doesn't count global BA's now-dominant wall-clock cost, and ATE regresses 25x (not explainable by drift-over-time alone).
metadata:
  type: progress
---

# Stage 4 M0 (in progress): MH_01_easy full-sequence baseline

## What was measured

`slam-run --full data/machine_hall/MH_01_easy`, run to completion in the
foreground (background execution proved unreliable earlier this
session — two prior attempts vanished without producing output after
10+ minutes; this run stayed attached and completed):

```
3682 frames, 184.0s of data, 741 keyframes, 0 unrecoverable frames
ATE rmse=3.869m mean=3.437m median=3.148m std=1.776m max=7.714m
RPE (delta=1 keyframes): rmse=0.160m mean=0.112m max=0.900m
RPE (delta=10 keyframes): rmse=1.104m mean=0.931m max=2.732m
timing: vision=102.62s optimization=23.67s global_ba=957.16s (data=184.0s)
real_time_factor()=0.686
```

Compare to the bounded 600-frame clip's numbers already in
`docs/RESULTS.md`: ATE rmse 0.151m, real-time factor 0.54ish, global BA
~2.8s.

## Finding 1 (confirmed by live profiling, not just code-reading):
global BA's dense solve is the wall-clock bottleneck

Before this run finished, sampled the live process with macOS `sample`
(`sample <pid> 5`): **100% of sampled stack frames** were inside
`nalgebra::linalg::lu::LU::new`, called from `slam_optim::solver::
optimize`, called from `slam_backend::vio::VioPipeline::
global_bundle_adjustment`. This directly confirms `plan/STAGE4.md`'s
"What we already know" hypothesis (`global_bundle_adjustment_inner`
solves densely over the full, unbounded `history` — never bounded by
Stage 2 M1's marginalization, which only bounds the *windowed*
solver). The completed run's own timing breakdown confirms it
quantitatively: `global_ba=957.16s` vs `vision+optimization=126.29s` —
global BA is **~7.6x** the entire rest of the pipeline's wall-clock
combined, for one call.

741 keyframes (not the ~368 originally estimated from `frames/stride` —
real count, not a naive extrapolation) gives a `(741-1)*15 = 11100`-
dimensional dense system per LM iteration; `nalgebra`'s dense LU is
`O(n^3)`, so `11100^3 ≈ 1.4e12` operations, times up to 6 LM iterations
— consistent with a ~16-minute single-call wall-clock cost.

## Finding 2 (new — not in the original plan's hypothesis): the
`real_time_factor()` metric itself doesn't see this bottleneck

`TimingBreakdown::real_time_factor()` is defined as `(vision_seconds +
optimization_seconds) / data_seconds` — global BA and loop closure are
*deliberately* excluded, per `plan/STAGE2.md`'s own scope note ("global
BA is a separate, one-shot batch pass, not held to the same per-frame
bar"). That was a reasonable call when global BA took ~3 seconds on the
bounded clip. At full-sequence scale, this run reports
`real_time_factor()=0.686` (looks real-time!) while total wall-clock
was `102.62+23.67+957.16=1083.45s` for `184.0s` of data — a true
end-to-end factor of **~5.9x**, i.e. running this "in real time" would
mean submitting a sequence and getting a result nearly 6x later than
the sequence itself took to record. Goal 2 of `plan/STAGE4.md`
("`slam-run` should meet the real-time criteria while running all
frames") needs to be read as *practical end-to-end wall-clock*, not
just the existing per-frame-loop metric — the metric's own scoping
decision, correct in Stage 2's context, no longer reflects "is this
usable" once global BA dominates. Worth an explicit plan update: either
redefine what "real-time" means for Stage 4's goal 2 (a whole-run
wall-clock bound, not just the per-frame-loop metric), or add a
separate "total wall-clock vs. data duration" number `docs/RESULTS.md`
reports alongside the existing metric, so a technically-passing
per-frame-loop factor can't mask an 18-minutes-for-3-minutes-of-data
experience.

## Finding 3 (new — the real accuracy question `plan/STAGE4.md` M2 exists for)

ATE went from 0.151m (bounded clip) to 3.869m (full sequence) — a 25x
increase for only ~6x more duration (30s -> 184s). RPE at delta=1
(0.160m) is close to the bounded clip's own local drift rate, so this
isn't "the per-step tracking got worse" — it looks like accumulated
*global* drift, which is plausible given this harness doesn't chain in
loop closure (`bin/slam-run`'s own documented scope: "Loop closure (M7)
is deliberately not chained in here"). But the magnitude is worth real
scrutiny before accepting "just longer-run drift" as the explanation:
against published SOTA, this pipeline goes from ~4x worse than
ORB-SLAM3 on the bounded clip (0.151m vs ~0.036m) to ~100x worse on the
full sequence (3.869m vs ORB-SLAM3's own full-sequence 0.036m) — a
disproportionate jump that plan/STAGE4.md's M2 should investigate for
real before assuming "no loop closure" fully explains it. Global BA ran
(and cost 957s doing it) and still left ATE this high, which itself is
worth understanding — is global BA's own dense solve actually
converging to a good optimum at this scale, or does something about
running LM over an 11100-dimensional system for the first time (never
exercised at this scale before) behave differently than the
well-tested ~100-keyframe case?

## Not yet done

Only `MH_01_easy` measured so far — each full-sequence run costs
~18 minutes wall-clock at this bottleneck's current cost, so measuring
all 5 sequences (`plan/STAGE4.md` M0's full scope) means roughly
1.5 hours of wall-clock if run serially, more if global BA's cost
grows worse than linearly with sequence length (plausible, given O(n^3)
and the other sequences having different lengths/keyframe counts).
Next: decide whether to (a) measure the remaining 4 sequences before
starting any fix, for a complete M0 baseline, or (b) start M1's actual
fix now that the bottleneck is confirmed on one sequence, and re-measure
all 5 after the fix lands rather than twice. Leaning toward (b) given
how expensive/slow (a) alone would be and how clear the root cause
already is — worth confirming with the user before committing to either
path.
