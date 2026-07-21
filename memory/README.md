# Project memory

This is shared, git-tracked memory for the auto-slam project — for any
Claude Code session (resumed, fresh, or running in parallel) to read before
starting work and write to as it makes progress. It is distinct from any
personal/global Claude memory system; everything here lives in the repo and
travels with `git clone`.

Read `INDEX.md` first — it's a short table of contents pointing into the
subdirectories below. Keep `INDEX.md` itself short (one line per entry); put
actual content in the linked file.

## Layout

- `progress/` — one file per unit of work, named
  `YYYY-MM-DD-short-slug.md`. Append-only in spirit: a session adds a new
  file for what it did rather than editing another session's file. This is
  the log of "what got done, in what order" — think of it as commit messages
  with more room to explain reasoning and dead ends.
- `decisions/` — one file per real design decision, named
  `NNNN-short-slug.md` (zero-padded, increasing). Write one when you choose
  between genuine alternatives (crate choice, algorithm variant, data
  format, factor-graph structure) — not for things that are obvious from
  reading the code. State the decision, the alternatives considered, and
  why — so a later session doesn't re-litigate it without new information.
- `notes/` — topic-based, one file per topic (e.g. `dataset-quirks.md`,
  `optimizer-gotchas.md`, `evaluation-methodology.md`). Unlike `progress/`
  and `decisions/`, these are living documents meant to be edited/extended
  by any session as understanding deepens — a running FAQ/gotcha sheet
  rather than a timeline.

## Why split like this instead of one file

Parallel sessions (e.g. two Claude Code instances working on different
milestones at once) editing one big log file is a merge-conflict machine.
Dated/numbered files in `progress/` and `decisions/` mean two sessions
essentially never touch the same file. `notes/` is the exception — those
are genuinely shared/living documents, so conflicts there are more likely;
keep edits to notes small and targeted (append a section, don't rewrite the
file) to keep merges easy.

## Conventions

- Every file gets a one-line pointer added to `INDEX.md` when created.
- Prefer linking between memory files (`see decisions/0002-...md`) over
  duplicating content.
- If you discover a memory file is stale or wrong (e.g. a decision that got
  reversed), don't silently delete it — add a note at the top pointing to
  whatever superseded it, and update `INDEX.md`'s one-liner to say so. The
  history of "we tried X, it didn't work, here's why" is valuable.
- This is not the place for anything derivable by reading the code or
  `git log` (file layout, current function signatures, who changed what
  line) — that goes stale immediately and the source of truth is the repo
  itself. Memory is for the "why," the plan status, and anything that took
  real effort to figure out.
