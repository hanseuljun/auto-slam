---
name: stage4-whole-run-wall-clock-metric-and-bounded-global-ba
description: Two user-confirmed decisions for Stage 4 - (1) fix the global-BA bottleneck before measuring the remaining 4 sequences, rather than measuring all 5 twice; (2) redefine goal 2's real-time bar as whole-run wall-clock (vision+optimization+global_ba <= data_seconds), not just the existing per-frame-loop metric, since that metric was found to hide global BA's now-dominant cost.
metadata:
  type: decision
---

# Decision: Stage 4 execution order + real-time metric redefinition

## Decisions

1. **Fix `global_bundle_adjustment`'s confirmed bottleneck before
   measuring the remaining 4 sequences**, rather than completing M0's
   full 5-sequence baseline first. Each full-sequence run cost
   ~15-20 minutes at the unfixed cost; the root cause was already
   confirmed via live profiling on `MH_01_easy` alone, so measuring 4
   more sequences at that cost before fixing anything would have meant
   re-measuring all 5 a second time anyway once the fix landed.
2. **Redefine `plan/STAGE4.md`'s goal 2** ("the real-time bar must hold
   on the full sequence") as **whole-run wall-clock** — `(vision +
   optimization + global_ba) / data_seconds ≤ 1.0` — instead of the
   existing `TimingBreakdown::real_time_factor()` metric, which only
   counts `vision + optimization` by design (a correct scoping choice
   in `plan/STAGE2.md`'s own context, where global BA cost ~3s and
   wasn't worth counting). At full-sequence scale, that metric reported
   `real_time_factor()=0.686` on `MH_01_easy` while total wall-clock was
   `1083.45s` against `184.0s` of data (~5.9x) — the metric's original
   scoping decision stopped reflecting "is this practical to use" once
   global BA's cost stopped being negligible.

## Alternatives considered

- **Measure all 5 sequences before touching any code** (the plan's
  original M0 scope, unmodified): rejected — given the root cause was
  already confirmed via profiling on one sequence, this would have cost
  ~1-1.5 hours of wall-clock to produce numbers that were about to
  become stale the moment the fix landed, with no additional diagnostic
  value over profiling already provided.
- **Keep the existing `real_time_factor()` metric as-is, report total
  wall-clock as a second, separate number** (also offered as a real
  option): rejected in favor of redefining goal 2 directly — the user
  chose to hold the *goal* to whole-run wall-clock rather than leave
  the existing metric's scope untouched and bolt on a second number
  next to it. `real_time_factor()` itself (the per-frame-loop-only
  metric) is left unchanged in code — still a real, meaningful number
  for "is the continuous tracking loop itself keeping up" — but it's no
  longer what Stage 4's own goal 2 is measured against.

## Source

Both decided via `AskUserQuestion` mid-session, after `plan/STAGE4.md`
M0's `MH_01_easy` measurement surfaced both questions concretely (not
decided in advance / hypothetically) — recommended options in both
cases, both accepted.

## Implications for later work

- Any future stage/milestone reporting "real-time" numbers should be
  clear about which metric it means — the per-frame-loop-only
  `real_time_factor()` (Stage 2's original scope) and the whole-run
  wall-clock factor (Stage 4's goal 2) are both legitimate, answer
  different questions, and shouldn't be conflated. `docs/RESULTS.md`
  now reports both explicitly in its "Full-sequence results" section —
  follow that pattern rather than picking just one silently.
- `global_bundle_adjustment`'s bounded scope (`VioParams::max_global_
  ba_keyframes`, `decisions`-adjacent to this one but really a code
  change — see `memory/progress/2026-07-23-stage4-m1-...md`) means
  global BA's cost is now flat regardless of sequence length. If a
  *much* longer sequence or dataset is ever used, re-confirm this
  bound's cost stays acceptable rather than assuming it scales the same
  way forever.
