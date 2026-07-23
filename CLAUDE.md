# CLAUDE.md

Guidance for any Claude Code session (this one, a resumed one, or a parallel
one) working in this repository.

## What this project is

A stereo-inertial SLAM program and reconstruction library, written in Rust,
with the SLAM/estimation algorithms implemented from scratch (infra crates
like `nalgebra`/`image`/`serde` are fine; no OpenCV/g2o/Ceres/existing SLAM
crates). Current stage plan: `plan/STAGE3.md` (trajectory visualization —
a hand-written 3D rendering library plus an application that browses
past runs' results). `plan/STAGE2.md` (real-time VIO + finishing Stage
1's M9/M10) and `plan/STAGE1.md` (the original 11-milestone SLAM plan)
are both done and worth reading for that history — Stage 3 consumes
their output (`bin/slam-run`'s per-run CSV/metadata) rather than
changing their code. `plan/STAGE3.md` has its own dependency-policy
addendum (a hand-written rendering library is "the algorithm" here;
`wgpu`/`winit`/`egui` are infra, same spirit as `nalgebra`/`image`
above). Read the current stage plan before picking up work, and update
it (or add `plan/STAGE4.md` etc.) when the plan itself changes, not just
when code changes.

## Verification: tests + a human-readable test app

Every piece of implementation work must be verifiable two ways before it's
considered done:

1. **Automated tests** (`cargo test`) covering the actual logic — unit tests
   per crate, plus numerical checks where relevant (e.g. finite-difference
   vs. analytic Jacobians, round-trip projection/triangulation on real
   calibration data).
2. **The test app**, `bin/slam-inspect` (create it in M0 if it doesn't exist
   yet), kept up to date so the user can confirm progress by reading its
   output directly — no GUI, no special viewer, plain text/CSV to stdout or
   files under a `runs/` or `out/` directory. As each milestone in
   `plan/STAGE1.md` lands, extend this app so it demonstrates that milestone:
   dataset load stats, calibration dump, tracked-feature counts, ATE/RPE
   tables, per-sequence summaries, etc. Re-run it as part of verifying any
   change — a change isn't "done" until both `cargo test` passes and the
   test app's output reflects the new capability.

Do not build a separate throwaway demo per milestone — extend the one app so
it stays a running, readable record of what the pipeline can currently do.

## Keep README.md current for humans

`memory/` is for cross-session AI continuity; `README.md` is the
human-facing equivalent and must stay current too. After a milestone (or
any change worth a commit under the Git workflow section below), update
`README.md` so a human — not just a future Claude session — can open the
repo and understand what this project is, what stage/milestone it's
currently at, how to build it and run `bin/slam-inspect`, and how to read
its output to confirm the claimed progress themselves. Treat it as the
human-readable status report and entry point: don't let it drift out of
sync with what the code actually does, and don't let it go stale as a bare
title while `memory/` accumulates real content.

## Project memory (for cross-session continuity)

This repo has a `memory/` directory, separate from any personal/global
Claude memory — it's checked into git so any session (resumed, fresh, or
running in parallel on another machine) can read prior context and add its
own. Read `memory/README.md` first; it defines the structure and conventions
(why it's split into `progress/`, `decisions/`, `notes/` instead of one
big file — mainly to avoid merge conflicts between parallel sessions).

Update `memory/` as you go, not just at the end of a session: log progress
when you complete a milestone or sub-step, record a decision when you choose
between real alternatives (especially anything not obviously derivable by
re-reading the code), and jot a note when you hit a non-obvious gotcha
(dataset quirks, a bug's root cause, a tuning result) that would otherwise be
rediscovered the hard way by the next session.

## Git workflow

Commit and push to `origin` after each meaningful unit of progress (a
milestone sub-step, a bug fix, a memory update worth preserving) — don't
batch up large silent stretches of work. This push-after-progress habit is
pre-authorized for this repo, so proceed without asking each time. Still
never force-push, reset --hard, or otherwise rewrite/discard history without
explicit confirmation, and still stop and ask if something looks like it
would discard uncommitted work that isn't yours to discard.

Normal flow: `cargo test` (and the test app) pass → `git add` the specific
files → commit with a message describing the "why" → push to `origin main`
(or the current branch) → update `memory/progress/` if the change is
significant enough to matter to a future session.
