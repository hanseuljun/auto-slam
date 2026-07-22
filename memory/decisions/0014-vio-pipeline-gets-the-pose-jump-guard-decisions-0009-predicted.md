---
name: vio-pipeline-gets-the-pose-jump-guard-decisions-0009-predicted
description: VioPipeline now rejects PnP poses implying an implausible per-frame translation jump (VioParams::max_pose_jump_meters, mirroring VoPipeline's decisions/0009 fix exactly) — the root cause behind Stage 2 M1's marginalization divergence, and exactly the future finding decisions/0009 predicted when it noted VioPipeline "likely shares this vulnerability... isn't yet protected."
metadata:
  type: decision
---

# Decision: `VioPipeline` gets its own PnP pose-jump guard, closing a gap `decisions/0009` predicted

## Decision

`VioPipeline::process_frame` (`crates/slam-backend/src/vio.rs`) now
rejects a PnP-estimated pose outright — treating it as track loss and
triggering M6's IMU-propagation recovery — if it implies a per-frame
translation jump larger than `VioParams::max_pose_jump_meters` (default
2.0m). This is the exact same check, same default threshold, and same
root cause `decisions/0009` fixed in `VoPipeline` back in M7.

## Why

Debugging Stage 2 M1's marginalization divergence (`decisions/0013`)
traced the *root* cause past marginalization entirely: even with
marginalization's own contribution completely blocked (a diagnostic
build where every marginalization attempt was rejected), keyframe poses
still diverged to implausible magnitudes within ~100 frames on real
MH_01 data. The corruption was already present in `VioPipeline`'s raw
PnP output, entering the window unfiltered — `decisions/0009`'s own "how
to apply" section predicted exactly this: "`VioPipeline` (M5) uses the
same DLT+refine PnP for its own pose guess and likely shares this
vulnerability... isn't yet protected by an equivalent check... Worth
applying the same fix there if a similar corruption shows up in a future
full-VIO-sequence test." It had simply never been *observed* before,
because naive fixed-lag dropping "forgot" the corrupted keyframe quickly
enough that Sim3-aligned ATE stayed plausible-looking on short clips —
the same metric-masking effect `decisions/0009` itself warned about.
Marginalization's job is to retain information across evictions, so it
surfaced a bug that had been latent in `VioPipeline` since M5.

Confirmed as the actual root cause, not just a plausible theory: with
this fix alone (marginalization still disabled), MH_01's 600-frame ATE
went from diverging to a clean 0.164m with 109 keyframes; with
marginalization re-enabled on top of this fix, 0.169m with 104
keyframes — matching within noise, and both a large improvement over the
pre-fix divergence.

## How to apply

When a known vulnerability is fixed in one pipeline (`VoPipeline`) but a
sibling pipeline (`VioPipeline`) shares the same underlying mechanism
(here: identical DLT+Gauss-Newton PnP, `decisions/0003`'s known
no-RANSAC limitation) without the same guard, treat the gap as a real,
if latent, bug — not a hypothetical. `decisions/0009` already flagged
this exact gap by name; this decision is the confirmation that flagged
gaps in this codebase are worth closing proactively once a consumer
(here, marginalization) makes them visible, rather than waiting for
another full debugging cycle to rediscover the same root cause.
