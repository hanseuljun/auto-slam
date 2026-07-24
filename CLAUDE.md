# CLAUDE.md

Guidance for any Claude Code session (this one, a resumed one, or a parallel
one) working in this repository.

## What this project is

A stereo-inertial SLAM program and reconstruction library, written in Rust,
with the SLAM/estimation algorithms implemented from scratch (infra crates
like `nalgebra`/`image`/`serde` are fine; no OpenCV/g2o/Ceres/existing SLAM
crates). No stage plan is currently in progress — `plan/STAGE6.md`
(closing the accuracy gap: real analytic IMU Jacobians + preintegration
covariance propagation, a sparse pose-graph solver removing loop
closure's own correction ceiling, and a real investigation into the
scale-consistency anomaly `plan/STAGE5.md` M0 found) just finished, all
7 milestones landed — read it for that history, including its own
honestly-documented open question (the anisotropic scale distortion's
root cause: `memory/decisions/0027`-`0029`). `plan/STAGE5.md` (honest
drift measurement + real loop closure), `plan/STAGE4.md` (full-sequence
real-time VIO), `plan/STAGE3.md` (trajectory visualization), `plan/
STAGE2.md` (real-time VIO on a bounded clip + finishing Stage 1's
M9/M10), and `plan/STAGE1.md` (the original 11-milestone SLAM plan) are
also all done and worth reading for that history. When picking up new
work, write a new `plan/STAGE7.md` (or ask what to prioritize) rather
than assuming which of Stage 6's own open threads to pull on next.
`plan/STAGE3.md` has its own dependency-policy addendum (a hand-written
rendering library is "the algorithm" there; `wgpu`/`winit`/`egui` are
infra, same spirit as `nalgebra`/`image` above). Read the current stage
plan before picking up work, and update it (or add `plan/STAGE6.md`
etc.) when the plan itself changes, not just when code changes.

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
