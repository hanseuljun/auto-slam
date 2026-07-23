# Stage 4: Full-sequence real-time VIO

## Goal

Three goals, in the order stated by the user (each depends on the one
before it landing safely):

1. **`bin/slam-run` runs the whole sequence by default**, not the
   600-frame (~30s) bounded clip Stage 2's M0 introduced. The bounded
   default was a deliberate, explicit scope-narrowing at the time
   ("a harness that takes 30+ minutes to produce one number isn't
   usable for iterating on the rest of this stage," `plan/STAGE2.md`
   M0) — a debt Stage 2 itself flagged as unpaid (see "What we already
   know" below), not an oversight to quietly work around.
2. **The real-time bar must hold on the full sequence**, not just the
   bounded clip it was actually measured on. Stage 2's own Risks
   section named this exact gap in advance and never closed it (see
   below) — Stage 4 closes it for real, with measured numbers, before
   flipping the default in goal 1. **Redefined during M0/M1** (a real
   decision, not an assumption): the existing `real_time_factor()`
   metric (`vision+optimization only`) turned out to hide global BA's
   cost entirely once that cost stopped being negligible — it reported
   "real-time" on a run whose true end-to-end wall-clock was ~5.9x the
   data duration. Goal 2 now means **whole-run wall-clock (vision +
   optimization + global BA) ≤ data duration**, the number that
   actually reflects what running `slam-run` feels like to use.
3. **Accuracy must not regress** relative to the bounded-clip numbers
   already in `docs/RESULTS.md`. Running more frames is not allowed to
   be a hidden accuracy regression — a bug that only shows up over a
   longer run (more marginalization events, more outlier-gating
   decisions, accumulated numerical drift) is exactly the kind of thing
   a 30-second clip can't surface.

Same dependency policy, same dataset, same determinism requirement as
Stages 1-3. This stage doesn't add new capabilities — it makes the
existing pipeline's default evaluation mode reflect real, full-sequence
performance instead of a bounded approximation of it.

## What we already know (don't rediscover this the hard way)

This gap was **predicted, not discovered**. Two prior artifacts said so
explicitly, and this session's own attempt to actually run `--full`
confirms the prediction was right to worry about:

- `plan/STAGE2.md`'s own Risks section: *"M0's default-run truncation
  could hide the exact scaling problem this stage exists to fix... a
  truncated clip that happens to fit inside the window can look
  real-time for reasons that have nothing to do with actually fixing
  the scaling."*
- `docs/RESULTS.md`'s own limitations section says the real-time
  numbers are **"not re-benchmarked with `--full` yet"** — Stage 2
  asserted the M1 marginalization fix should generalize to full
  sequences but never actually measured it.
- **This session**: ran `slam-run --full data/machine_hall/MH_01_easy`
  in the background. It did not complete (no `trajectory.csv` update,
  no output) after 10+ minutes before the process disappeared —
  inconclusive on its own (environment/session-boundary flakiness with
  long-running background processes was also observed this session,
  so this isn't proof of an algorithmic hang by itself), but consistent
  with the predicted gap, not evidence against it. Re-measuring
  cleanly, in the foreground, with a bounded ceiling, is Stage 4's M0.

**A concrete, code-level root-cause candidate, found by reading the
code (not guessing)**: `slam-backend/src/vio.rs`'s
`global_bundle_adjustment_inner` builds its `Problem` from
`self.history.iter().chain(self.window.iter())` — **every keyframe
ever created**, not just the bounded window. Stage 2's M1
(marginalization) only bounds the *windowed* backend's per-frame
optimization; keyframes evicted from the window still get folded into
`history` as literal retained keyframes (that's what lets
`global_bundle_adjustment` and `bin/slam-viz`'s trajectory view work at
all). `slam-optim/src/solver.rs`'s dense `DMatrix`/LU solve — the
*exact* O(dim³) scaling `plan/STAGE2.md`'s original "What we already
know" identified as M9's rollback cause — was never replaced; Stage 2's
M3 ("sparse-aware normal-equations solve") explicitly deferred it,
but that deferral's own reasoning only covered the *windowed* solver
("fine at M1's now-bounded problem sizes, window ~8 keyframes") —
it says nothing about `global_bundle_adjustment`, which still solves
over the *entire* history. A full sequence has roughly 3-6x more
keyframes than the 600-frame bounded clip (`bin/slam-run`'s own
"bounded run: 600/N frames" output: MH_01 600/3682, MH_02 600/3040,
MH_03 600/2700, MH_04 600/2032, MH_05 600/2273) — an O(n³) solve over
~3-6x more keyframes is a ~30-200x slowdown for global BA specifically,
independent of whether the per-frame VIO loop itself (already
confirmed linear-time and real-time in `plan/STAGE2.md` M5) has any
problem at all. This is the leading hypothesis M0/M1 below should
confirm or rule out with real profiling, not assume.

## Milestones

Same discipline as every prior stage: measure before fixing, fix before
changing the default, no milestone closes on an assumed number.

### M0 — Full-sequence baseline measurement — Done

- Run `bin/slam-run --full` on all 5 sequences (foreground, not
  background — this session's own attempt suggests long-running
  background processes aren't reliable to wait on in this environment;
  a bounded, foreground, explicitly-timed run is more trustworthy than
  an unbounded background one that might silently vanish). Record
  actual wall-clock, per-stage timing breakdown, real-time factor, and
  ATE/RPE for each — without changing anything yet.
- If a sequence doesn't complete within a generous but explicit ceiling
  (e.g. 20-30 minutes, matching the original M9 rollback's own bar),
  that's a real, concrete finding to document (`memory/decisions`),
  not something to work around blindly or let silently time out
  unrecorded.
- Test/deliverable: a new "full sequence" table in `docs/RESULTS.md`
  alongside the existing bounded-clip one, with the same honesty
  standard — real measured numbers, or an explicit documented stall,
  never an assumed/extrapolated one.
- **Partial result (`MH_01_easy` measured, 4 sequences remaining)**:
  `3682 frames, 741 keyframes, ATE rmse=3.869m` (bounded clip: 0.151m),
  `vision=102.6s optimization=23.7s global_ba=957.2s` (data=184.0s),
  `real_time_factor()=0.686`. Confirmed the "What we already know"
  hypothesis by live-profiling the running process (macOS `sample`):
  100% of sampled stack frames were inside `global_bundle_adjustment`'s
  dense LU solve. Two things this run surfaced that the original
  hypothesis didn't fully anticipate, both real enough to change how
  M1/M2 are scoped:
  1. **`real_time_factor()` doesn't see global BA's cost** — it's
     defined as `(vision+optimization)/data_seconds` by design (`plan/
     STAGE2.md`'s own scope note, correct when global BA cost ~3s).
     Here it reports 0.686 (looks real-time) while total wall-clock was
     1083.45s against 184.0s of data — a true end-to-end factor of
     ~5.9x. Goal 2 ("meet the real-time criteria while running all
     frames") needs a metric that actually reflects this, or it'll
     "pass" on a number that doesn't mean what it used to. M1 should
     either fix global BA's cost until it's back to not mattering for
     this metric's original scoping to be valid again, or `docs/
     RESULTS.md`/the harness should report total wall-clock
     explicitly alongside the existing per-frame-loop number so this
     can't happen silently again.
  2. **ATE regressed 25x (0.151m -> 3.869m) for only ~6x more duration**
     — not obviously explainable by "longer runs drift more" alone
     (RPE at delta=1 stayed close to the bounded clip's own rate, and
     against published SOTA this went from ~4x worse to ~100x worse).
     This harness doesn't chain in loop closure by design, which
     predicts *some* extra drift, but not necessarily this much — M2
     needs to actually investigate, not assume "no loop closure"
     covers it.
  Full writeup: `memory/progress/2026-07-23-stage4-m0-mh01-full-
  sequence-measured.md`.
- **Full result (all 5 sequences, measured after M1's fix landed —
  the user chose "fix first, then measure all 5 once" over measuring
  all 5 twice)**: every sequence's whole-run factor is now under 1.0
  (0.49-0.81 across the five). Every sequence also shows a large
  full-sequence ATE regression vs. its bounded-clip number (5.6x-25.6x)
  — a real, pre-existing gap (confirmed independent of M1's fix, see
  M1's own Result below), now M2's job. Full table: `docs/RESULTS.md`'s
  "Full-sequence results" section.

### M1 — Root-cause and fix the real-time gap (if M0 confirms one) — Done

- If M0 shows real-time factor > 1.0 (measured over the *whole* run,
  not just the per-frame VIO loop `plan/STAGE2.md` M5 already validates)
  or a global-BA/other stage taking disproportionate wall-clock, profile
  which stage actually dominates. Start from the concrete lead in "What
  we already know" above (`global_bundle_adjustment`'s O(n³) solve over
  unbounded `history`) but confirm with real timing breakdown before
  assuming that's the whole story — the same "check what's actually
  slow, don't guess" discipline `plan/STAGE2.md`'s own M0 already
  demonstrated once.
- Likely fix shape if the global-BA hypothesis holds: bound global BA's
  own problem size the same way M1 already bounded the windowed
  solver's — e.g. run it over a large-but-bounded recent window plus
  marginalization priors instead of literal full `history`, or revisit
  Stage 2 M3's deferred sparse solve now that it's not clearly optional
  for this specific call site. Don't assume the fix without measuring
  the actual bottleneck first.
- Test: re-run M0's harness after the fix, confirm the real-time factor
  actually drops and holds ≤ 1.0 on every sequence — measured, not
  assumed, same bar every prior real-time milestone held to
  (`plan/STAGE2.md` M5's own "Result" note is the template: real
  before/after numbers, not "should be faster now").
- **Result**: the global-BA hypothesis held. Fix: `VioParams::
  max_global_ba_keyframes` (default 150) caps `global_bundle_
  adjustment` to the most recent N keyframes instead of literal
  unbounded `history` — no new linear algebra (reuses the existing,
  already-tested `Problem`/`optimize` machinery unchanged, just bounds
  what goes into it), so this carries much lower correctness risk than
  a from-scratch sparse solver would have. On `MH_01_easy`: global BA
  957.2s -> **7.8s** (~123x), whole-run factor 5.89x -> **0.814x**.
  Confirmed on all 5 sequences: whole-run factor now 0.49-0.81, all
  under 1.0 (table in `docs/RESULTS.md`). Critically, `MH_01_easy`'s
  ATE was measured *before and after* this fix on the identical full
  sequence: 3.869m -> 3.868m, i.e. bounding global BA's scope cost
  nothing — global BA over the *full* unbounded history wasn't
  preventing the accuracy problem M0 found anyway (no loop closure
  means no correcting signal regardless of optimization scope), so this
  fix is a pure win, not an accuracy-for-speed tradeoff. One new,
  real correctness risk the bounded scope introduces, found by reading
  the code (not guessing) and specifically tested for: once the cap
  excludes the true first keyframe, the new oldest-*included* keyframe
  still has a real `imu_edge` pointing at a now-excluded keyframe — the
  old unbounded loop's `if let Some(imu_edge)` check alone would
  reference a nonexistent local index `-1` (an underflow); fixed with
  an explicit `kf_idx > 0` guard, and a new test (`global_bundle_
  adjustment_respects_max_global_ba_keyframes_cap`) exercises exactly
  this path against real MH_01 data — it would panic, not just report
  a wrong number, if the guard were missing.

### M2 — Root-cause and fix any accuracy regression — confirmed needed, not yet started

- **M0 confirmed this is needed, on all 5 sequences, not just
  `MH_01_easy`**: full-sequence ATE vs. bounded-clip ATE is 5.6x-25.6x
  worse across the board (table in `docs/RESULTS.md`'s "Full-sequence
  results"), and M1's own before/after on `MH_01_easy` (3.869m ->
  3.868m, unbounded vs. capped global BA) rules out the M1 fix itself
  as the cause — this is a pre-existing gap the bounded-clip numbers
  never had the chance to surface, still open.
- Compare M0's full-sequence ATE/RPE against what the bounded-clip
  numbers plus natural expected drift-over-longer-time would predict —
  full-sequence ATE is *expected* to be numerically larger than a
  30-second clip's (more time to drift), so this isn't "the number went
  up = regression"; it's "the number is worse than a longer run
  *should* look, relative to the per-second drift rate the bounded clip
  already showed" (`docs/RESULTS.md`'s own honest caveat already frames
  this distinction for the existing bounded-vs-published-SOTA
  comparison — reuse that framing here, not a new ad hoc bar).
- If a genuine regression (not just longer-duration drift) is found,
  investigate the actual cause — candidates worth checking first: more
  marginalization events over a longer run stressing an edge case M1's
  own guards (`decisions/0012`-`0014`) were built for but only verified
  on shorter clips, or outlier-gating/track-loss-recovery behavior
  compounding differently over more frames. Only reopen that code if
  profiling/error analysis actually points there, same discipline as
  every prior stage's accuracy-debugging milestones.
- Test: full-sequence ATE/RPE in `docs/RESULTS.md` is explainable by
  natural drift-over-time from the bounded-clip numbers, not a
  bug-shaped regression — and M1's real-time fix (if any) must not have
  traded accuracy for speed to get there.

### M3 — Flip the default, keep a fast bounded mode available

- Change `bin/slam-run`'s default from the 600-frame bounded clip to
  the full sequence, once M0-M2 confirm it's both real-time and
  accurate. Keep the bounded/fast mode available under an explicit flag
  (the current `--frames N` mechanism already supports this — just
  invert which behavior is the unflagged default) so quick tuning
  iteration (the exact workflow `decisions/0016`-`0017`'s sweeps used)
  isn't lost; Stage 2/3's own tuning and testing depended on a fast
  default existing.
- Update `docs/RESULTS.md`'s headline tables, `README.md`'s
  reproduction instructions, and `bin/slam-viz`'s expectations (per-run
  history will default to full-sequence runs going forward — no code
  change needed there, `bin/slam-viz` already just renders whatever
  `trajectory.csv` it's given, but worth confirming the 3D/video/graphs
  panels stay usable at full-sequence keyframe counts, not just the
  ~100-keyframe bounded case they were built and tested against).
- Test: `cargo run --release --bin slam-run` (no flags) produces
  full-sequence numbers matching M0-M2's already-recorded results; a
  `--frames 600` (or similar) flag still available and still fast, for
  anyone who needs quick-iteration mode back.

## Out of scope for Stage 4

Same carried-forward list as Stages 1-3, plus: no new accuracy features
(Stage 2 M6 already closed that milestone; this stage only guards
against *regressing* accuracy, doesn't try to improve it further) and
no new visualization features (Stage 3 already closed that stage;
`bin/slam-viz` gets used as-is here, not extended).

## Risks

- **The global-BA hypothesis in "What we already know" could be wrong,
  or only part of the story.** It's a strong, code-grounded lead, not a
  confirmed diagnosis — M0/M1 must actually profile before assuming
  it's the whole fix, same as every prior stage's "measured, not
  assumed" discipline.
- **Long-running background processes were unreliable in this session's
  environment** (a `--full` run vanished without completing or
  producing an error). M0 should run in the foreground with explicit
  timing, not rely on background execution surviving unattended for
  many minutes.
- **Flipping the default is a real workflow change.** Every prior
  stage's fast-iteration tuning work (Stage 2 M6, Stage 3's own
  development) depended on the bounded clip being fast to re-run
  repeatedly. M3 must preserve that as an explicit opt-in, not remove
  it outright, or it'll quietly make the next stage's iteration loop
  slower without anyone deciding that tradeoff on purpose.
- **This could reopen the exact wound Stage 2 was created to close.**
  Stage 2 itself exists because an earlier, blind attempt at running a
  full sequence took 30+ minutes and got rolled back
  (`plan/STAGE2.md`'s own origin story). Stage 4 must not repeat that —
  measure and fix incrementally (M0 before M1 before M3), never flip
  the default on an assumption.
