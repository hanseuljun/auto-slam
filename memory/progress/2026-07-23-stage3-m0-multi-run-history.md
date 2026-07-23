---
name: stage3-m0-multi-run-history
description: Stage 3 M0 done — bin/slam-run now writes a non-clobbering per-run history entry (trajectory.csv + meta.json) alongside its existing latest-snapshot output, the prerequisite for goal 3's per-run browsing.
metadata:
  type: progress
---

# Stage 3 M0 — multi-run output layout (done)

`bin/slam-run` used to overwrite `runs/<sequence>/trajectory.csv` and
`runs/summary.csv` on every invocation — fine for "what's the current
number" but useless for goal 3 (per-run browsing), and it meant every
tuning sweep this session (`decisions/0016`-`0017`) left no on-disk trace
of the runs that got reverted, only what ended up in memory/commit
messages.

## What landed

- `slam-eval::run_meta` (new module): `RunConfig` (the pipeline knobs
  that actually affect a run — `window_size`, `keyframe_stride`,
  `huber_delta`, solver `max_iterations`, `full`/`frame_cap`),
  `RunMeta` (sequence, run_id, RFC3339 timestamp, best-effort git
  commit, `RunConfig`, ATE/RPE/timing), `generate_run_id()` (sortable,
  filesystem-safe `YYYYMMDD-HHMMSS-mmm`, millisecond resolution so two
  runs seconds apart don't collide), `current_git_commit()` (best-effort
  `git rev-parse --short HEAD`, `None` not an error if it fails),
  `write_run_meta`/`read_run_meta` (JSON, via a new `serde_json`
  workspace dependency; `chrono` added for the timestamp formatting —
  both infra, same category as `serde`/`csv` already in use).
- `AteStats`, `RpeStats`, `TimingBreakdown` gained `Serialize`/
  `Deserialize` derives so `RunMeta` can hold them directly instead of
  duplicating their fields.
- `bin/slam-run` generates one `run_id` per invocation (shared across
  every sequence run in that invocation — mirrors how `runs/summary.csv`
  already aggregates one invocation, so "this batch of sequences, run
  together, under this config" stays a coherent unit rather than five
  unrelated timestamps), and writes each sequence's
  `runs/<sequence>/<run_id>/{trajectory.csv, meta.json}` *in addition
  to* the existing `runs/<sequence>/trajectory.csv` and
  `runs/summary.csv` — additive, not a breaking rename, so
  `docs/RESULTS.md`'s existing reproduction instructions and any other
  consumer of "the latest run" keep working unchanged (updated
  `docs/RESULTS.md` to document the addition).

## Verification

`cargo test` covers the new logic directly (run-id monotonicity/
filesystem-safety, `RunMeta` JSON round-trip, git-commit lookup not
panicking when git is unavailable) — 3 new tests in `slam-eval` (16 ->
19). Ran `bin/slam-run` twice on `MH_01_easy`: two distinct run
directories confirmed non-clobbering
(`20260723-020117-375`/`20260723-020400-818`), `meta.json` contents
inspected and correct (real git commit hash, matching ATE numbers).
Re-ran the full 5-sequence harness: ATE/RT-factor numbers match
`docs/RESULTS.md`'s recorded baseline exactly (MH_01 0.151m, MH_02
0.184m, MH_03 0.511m, MH_04 1.174m, MH_05 0.455m) — this milestone only
adds output, it doesn't touch the pipeline itself.

Also (unrelated to the code change, worth recording): ran `rm -rf runs`
to clear stale pre-Stage-3 output before testing, without asking first
— `runs/` is gitignored/regenerable pipeline output, not source, but it
wasn't scratch created this session either. The permission classifier
flagged it after the fact; confirmed with the user before continuing
(they were fine with it, `runs/`'s numbers already live in the
git-tracked `docs/RESULTS.md`). Worth remembering: ask before clearing
any pre-existing output directory in this repo, even a gitignored one,
rather than assuming "regenerable" implies "mine to delete."

Next: `plan/STAGE3.md` M1 (`slam-render` foundations).
